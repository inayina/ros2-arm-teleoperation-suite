# Copyright 2026 ros2-arm-teleoperation-suite contributors
# SPDX-License-Identifier: MIT

"""Unit tests for thin ACT/oracle Policy Adapters (no VLA, no Isaac)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from isaac_sim_adapter.policy_adapters import (
    ACTION_SCHEMA_VERSION,
    PolicyAdapterError,
    SceneActPolicyAdapter,
    ScriptedOraclePolicyAdapter,
    load_identity_card,
    validate_ee_delta_gripper,
)


MIDSTREAM_REGISTRY = Path(
    '/home/ina/robot-sim-lab/robot-arm-episode-data-lab/'
    'evaluation/registry/policies'
)


def _act_identity() -> dict:
    path = MIDSTREAM_REGISTRY / 'scene_act_lerobot_e3_nominal.json'
    if path.is_file():
        return load_identity_card(path)
    return {
        'contract_version': 'policy_adapter_contract_v0',
        'artifact_type': 'policy_adapter_metadata',
        'policy_name': 'scene_act_lerobot',
        'policy_version': 'e3_nominal_diagnostic_baseline',
        'checkpoint_hash': 'deadbeef',
        'dataset_version': 'fixture',
        'benchmark_version': 'single_block_controlled_v0',
        'observation_schema_version': 'scene_act_state8_rgb224_v0',
        'action_schema_version': ACTION_SCHEMA_VERSION,
        'trace_run_id': 'test_act',
        'claims_task_success': False,
    }


def _oracle_identity() -> dict:
    path = MIDSTREAM_REGISTRY / 'isaac_scripted_oracle_v2b.json'
    if path.is_file():
        return load_identity_card(path)
    return {
        'contract_version': 'policy_adapter_contract_v0',
        'artifact_type': 'policy_adapter_metadata',
        'policy_name': 'isaac_scripted_oracle',
        'policy_version': 'e3p5_v2b',
        'checkpoint_hash': 'oracle_scripted_v2b',
        'dataset_version': 'n/a',
        'benchmark_version': 'single_block_controlled_v0',
        'observation_schema_version': 'oracle_fsm_pose_v0',
        'action_schema_version': ACTION_SCHEMA_VERSION,
        'trace_run_id': 'test_oracle',
        'claims_task_success': False,
    }


class _FakeRuntime:
    def __init__(self) -> None:
        self.metadata = {
            'deploy_n_action_steps': 8,
            'chunk_size': 50,
        }
        self.reset_calls = 0

    def reset(self) -> None:
        self.reset_calls += 1

    def infer(self, state, rgb):
        del rgb
        assert len(state) == 8
        return [0.01, 0.0, -0.02, 0.0, 0.0, 0.0, 1.0]


def test_validate_ee_delta_gripper() -> None:
    assert len(validate_ee_delta_gripper([0] * 7)) == 7
    with pytest.raises(PolicyAdapterError):
        validate_ee_delta_gripper([0] * 6)


def test_scene_act_adapter_metadata_with_fake_runtime(monkeypatch) -> None:
    identity = _act_identity()
    adapter = SceneActPolicyAdapter(identity, n_action_steps=8)

    def _fake_load(self, checkpoint_or_endpoint):
        self._runtime = _FakeRuntime()
        self._identity['loaded_from'] = checkpoint_or_endpoint
        self._identity['deploy_n_action_steps'] = 8
        self._identity['chunk_size'] = 50

    monkeypatch.setattr(SceneActPolicyAdapter, 'load_policy', _fake_load)
    adapter.load_policy('/tmp/fake_ckpt.pt')
    adapter.reset({'episode_id': 'ep0'})
    obs = adapter.build_observation(
        {
            'state': [0.0] * 8,
            'rgb': np.zeros((224, 224, 3), dtype=np.uint8),
        }
    )
    raw = adapter.predict_action(obs)
    exported = adapter.export_action(raw)
    meta = adapter.report_metadata()
    assert meta['claims_task_success'] is False
    assert meta['action_schema_version'] == ACTION_SCHEMA_VERSION
    assert meta['deploy_n_action_steps'] == 8
    assert meta['chunk_size'] == 50
    assert meta['postprocessed_action'] == exported
    adapter.close()


def test_scripted_oracle_adapter_emits_bounded_delta() -> None:
    adapter = ScriptedOraclePolicyAdapter(
        _oracle_identity(),
        max_xyz_step=0.05,
        phase_name='descend',
    )
    adapter.load_policy(None)
    adapter.reset(
        {
            'object_xyz': [0.4, 0.1, 0.03],
            'ee_xyz': [0.3, 0.0, 0.4],
        }
    )
    obs = adapter.build_observation({'ee_xyz': [0.3, 0.0, 0.4]})
    raw = list(adapter.predict_action(obs))
    assert len(raw) == 7
    # Toward object XY / pick Z but clipped.
    assert abs(raw[0]) <= 0.05 + 1e-9
    assert abs(raw[1]) <= 0.05 + 1e-9
    assert abs(raw[2]) <= 0.05 + 1e-9
    exported = adapter.export_action(raw)
    meta = adapter.report_metadata()
    assert meta['claims_task_success'] is False
    assert meta['oracle_phase'] == 'descend'
    assert meta['postprocessed_action'] == exported
    assert meta['policy_name'] == 'isaac_scripted_oracle'
    adapter.close()


def test_identity_card_rejects_task_success_claim(tmp_path: Path) -> None:
    path = tmp_path / 'bad.json'
    path.write_text(
        json.dumps(
            {
                'claims_task_success': True,
                'action_schema_version': ACTION_SCHEMA_VERSION,
            }
        ),
        encoding='utf-8',
    )
    with pytest.raises(PolicyAdapterError):
        load_identity_card(path)
