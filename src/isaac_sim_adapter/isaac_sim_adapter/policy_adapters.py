"""Upstream thin Policy Adapters for midstream ABC (ACT + scripted oracle).

These wrappers emit ``policy_adapter_metadata`` and ``ee_delta_gripper[7]``.
They do **not** load VLA weights, rewrite evaluation evidence, or claim task
success. Physical GT remains owned by ContinuousTaskEvaluator / suite runner.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Mapping, MutableMapping, Sequence

ACTION_SCHEMA_VERSION = 'panda_ee_delta_gripper_v0'
ACTION_DIM = 7
CONTRACT_VERSION = 'policy_adapter_contract_v0'


class PolicyAdapterError(ValueError):
    """Interface-lane failure; never rewritten as task success."""


def validate_ee_delta_gripper(action: Sequence[float]) -> list[float]:
    if len(action) != ACTION_DIM:
        raise PolicyAdapterError(
            f'expected action dim {ACTION_DIM}, got {len(action)}'
        )
    values = [float(x) for x in action]
    if not all(math.isfinite(v) for v in values):
        raise PolicyAdapterError('action contains non-finite values')
    return values


def load_identity_card(path: Path | str) -> dict[str, Any]:
    payload = json.loads(Path(path).read_text(encoding='utf-8'))
    if payload.get('claims_task_success') is not False:
        raise PolicyAdapterError('identity card claims_task_success must be false')
    if payload.get('action_schema_version') != ACTION_SCHEMA_VERSION:
        raise PolicyAdapterError(
            f'action_schema_version must be {ACTION_SCHEMA_VERSION}'
        )
    return dict(payload)


def _clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


class SceneActPolicyAdapter:
    """Thin wrap of ``SceneACTRuntime`` for midstream Policy Adapter contract."""

    def __init__(
        self,
        identity: MutableMapping[str, Any],
        *,
        device: str = 'cpu',
        n_action_steps: int | None = None,
    ) -> None:
        if identity.get('claims_task_success') is not False:
            raise PolicyAdapterError('claims_task_success must be false')
        self._identity = dict(identity)
        self._device = device
        self._n_action_steps = n_action_steps
        self._runtime: Any | None = None
        self._closed = False
        self._last_raw: list[float] | None = None
        self._last_exported: list[float] | None = None
        self._latency_ms: float | None = None

    def load_policy(self, checkpoint_or_endpoint: str | None) -> None:
        if self._closed:
            raise PolicyAdapterError('adapter already closed')
        if not checkpoint_or_endpoint:
            raise PolicyAdapterError('SceneActPolicyAdapter requires checkpoint path')
        from isaac_sim_adapter.scene_act_runtime import SceneACTRuntime

        self._runtime = SceneACTRuntime(
            Path(checkpoint_or_endpoint),
            self._device,
            n_action_steps=self._n_action_steps,
        )
        self._identity['loaded_from'] = checkpoint_or_endpoint
        self._identity['deploy_n_action_steps'] = self._runtime.metadata.get(
            'deploy_n_action_steps'
        )
        self._identity['chunk_size'] = self._runtime.metadata.get('chunk_size')

    def reset(self, context: Mapping[str, Any]) -> None:
        if self._runtime is None:
            raise PolicyAdapterError('load_policy required before reset')
        self._runtime.reset()
        self._identity['last_reset_context'] = dict(context)
        self._last_raw = None
        self._last_exported = None
        self._latency_ms = None

    def build_observation(self, raw_state: Mapping[str, Any]) -> dict[str, Any]:
        state = raw_state.get('state')
        rgb = raw_state.get('rgb')
        if state is None or len(state) != 8:
            raise PolicyAdapterError('raw_state.state must be length 8')
        if rgb is None:
            raise PolicyAdapterError('raw_state.rgb required for Scene ACT')
        return {
            'observation.state': [float(x) for x in state],
            'rgb': rgb,
            'observation_schema_version': self._identity['observation_schema_version'],
        }

    def predict_action(
        self,
        observation: Mapping[str, Any],
        instruction: str | None = None,
    ) -> Sequence[float]:
        del instruction  # ACT path ignores language.
        if self._runtime is None:
            raise PolicyAdapterError('load_policy required before predict_action')
        state = observation.get('observation.state')
        rgb = observation.get('rgb')
        if state is None or rgb is None:
            raise PolicyAdapterError('observation requires observation.state and rgb')
        t0 = time.perf_counter()
        action = self._runtime.infer(list(state), rgb)
        self._latency_ms = (time.perf_counter() - t0) * 1000.0
        self._last_raw = [float(x) for x in action]
        return list(self._last_raw)

    def validate_action(self, action: Sequence[float]) -> Sequence[float]:
        return validate_ee_delta_gripper(action)

    def export_action(self, action: Sequence[float]) -> list[float]:
        exported = validate_ee_delta_gripper(action)
        self._last_exported = exported
        return list(exported)

    def report_metadata(self) -> dict[str, Any]:
        return {
            'contract_version': CONTRACT_VERSION,
            'artifact_type': 'policy_adapter_metadata',
            'policy_name': self._identity['policy_name'],
            'policy_version': self._identity['policy_version'],
            'checkpoint_hash': self._identity['checkpoint_hash'],
            'dataset_version': self._identity['dataset_version'],
            'benchmark_version': self._identity['benchmark_version'],
            'observation_schema_version': self._identity['observation_schema_version'],
            'action_schema_version': ACTION_SCHEMA_VERSION,
            'trace_run_id': self._identity['trace_run_id'],
            'inference_latency_ms': self._latency_ms,
            'raw_action': self._last_raw,
            'postprocessed_action': self._last_exported,
            'safety_clipping': {
                'applied': False,
                'axes': ['none'],
                'notes': (
                    'Upstream SceneActPolicyAdapter; runtime clip owned by '
                    'bound_ee_delta_gripper / Isaac execution.'
                ),
            },
            'failure_lane': 'none',
            'claims_task_success': False,
            'deploy_n_action_steps': self._identity.get('deploy_n_action_steps'),
            'chunk_size': self._identity.get('chunk_size'),
        }

    def close(self) -> None:
        self._closed = True
        self._runtime = None


class ScriptedOraclePolicyAdapter:
    """Offline scripted-oracle wrapper emitting bounded ``ee_delta_gripper``.

    Uses ``compute_oracle_targets`` / phase plan only. Does not claim lift/place
    success; Isaac ContinuoustaskEvaluator remains the authority.
    """

    def __init__(
        self,
        identity: MutableMapping[str, Any],
        *,
        max_xyz_step: float = 0.05,
        phase_name: str = 'descend',
    ) -> None:
        if identity.get('claims_task_success') is not False:
            raise PolicyAdapterError('claims_task_success must be false')
        self._identity = dict(identity)
        self._max_xyz_step = float(max_xyz_step)
        self._phase_name = str(phase_name)
        self._loaded = False
        self._closed = False
        self._targets: Any | None = None
        self._last_raw: list[float] | None = None
        self._last_exported: list[float] | None = None
        self._latency_ms: float | None = None

    def load_policy(self, checkpoint_or_endpoint: str | None) -> None:
        if self._closed:
            raise PolicyAdapterError('adapter already closed')
        # Oracle has no weights; optional path is recorded for provenance only.
        self._identity['loaded_from'] = checkpoint_or_endpoint
        self._loaded = True

    def reset(self, context: Mapping[str, Any]) -> None:
        if not self._loaded:
            raise PolicyAdapterError('load_policy required before reset')
        from isaac_sim_adapter.scripted_oracle import compute_oracle_targets

        object_xyz = context.get('object_xyz')
        ee_xyz = context.get('ee_xyz')
        if object_xyz is None or ee_xyz is None:
            raise PolicyAdapterError('reset context requires object_xyz and ee_xyz')
        self._targets = compute_oracle_targets(object_xyz, ee_xyz)
        self._identity['last_reset_context'] = {
            'object_xyz': list(map(float, object_xyz)),
            'ee_xyz': list(map(float, ee_xyz)),
            'phase_name': self._phase_name,
        }
        self._last_raw = None
        self._last_exported = None
        self._latency_ms = None

    def build_observation(self, raw_state: Mapping[str, Any]) -> dict[str, Any]:
        ee_xyz = raw_state.get('ee_xyz')
        if ee_xyz is None or len(ee_xyz) < 3:
            raise PolicyAdapterError('raw_state.ee_xyz required')
        return {
            'ee_xyz': [float(x) for x in ee_xyz[:3]],
            'observation_schema_version': self._identity['observation_schema_version'],
        }

    def predict_action(
        self,
        observation: Mapping[str, Any],
        instruction: str | None = None,
    ) -> Sequence[float]:
        del instruction
        if self._targets is None:
            raise PolicyAdapterError('reset required before predict_action')
        from isaac_sim_adapter.scripted_oracle import phase_plan

        ee = observation.get('ee_xyz')
        if ee is None or len(ee) < 3:
            raise PolicyAdapterError('observation.ee_xyz required')
        t0 = time.perf_counter()
        plan = dict((name, (xyz, grip)) for name, xyz, grip in phase_plan(self._targets))
        if self._phase_name not in plan:
            raise PolicyAdapterError(f'unknown oracle phase: {self._phase_name}')
        target_xyz, grip = plan[self._phase_name]
        if target_xyz is None:
            raise PolicyAdapterError(f'phase {self._phase_name} has no xyz target')
        delta = [
            _clip(float(target_xyz[i]) - float(ee[i]), -self._max_xyz_step, self._max_xyz_step)
            for i in range(3)
        ]
        action = delta + [0.0, 0.0, 0.0, float(grip)]
        self._latency_ms = (time.perf_counter() - t0) * 1000.0
        self._last_raw = action
        return list(action)

    def validate_action(self, action: Sequence[float]) -> Sequence[float]:
        return validate_ee_delta_gripper(action)

    def export_action(self, action: Sequence[float]) -> list[float]:
        exported = validate_ee_delta_gripper(action)
        self._last_exported = exported
        return list(exported)

    def report_metadata(self) -> dict[str, Any]:
        return {
            'contract_version': CONTRACT_VERSION,
            'artifact_type': 'policy_adapter_metadata',
            'policy_name': self._identity['policy_name'],
            'policy_version': self._identity['policy_version'],
            'checkpoint_hash': self._identity['checkpoint_hash'],
            'dataset_version': self._identity['dataset_version'],
            'benchmark_version': self._identity['benchmark_version'],
            'observation_schema_version': self._identity['observation_schema_version'],
            'action_schema_version': ACTION_SCHEMA_VERSION,
            'trace_run_id': self._identity['trace_run_id'],
            'inference_latency_ms': self._latency_ms,
            'raw_action': self._last_raw,
            'postprocessed_action': self._last_exported,
            'safety_clipping': {
                'applied': True,
                'axes': ['dx', 'dy', 'dz'],
                'notes': f'offline max_xyz_step={self._max_xyz_step}',
            },
            'failure_lane': 'none',
            'claims_task_success': False,
            'oracle_phase': self._phase_name,
        }

    def close(self) -> None:
        self._closed = True
        self._loaded = False
        self._targets = None
