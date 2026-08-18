"""Load the checked-in SmolVLA S4 runtime contract (stdlib JSON).

Midstream owns ``S4RuntimeContract`` in ``training/smolvla_s3/runtime_s4.py`` and
``configs/smolvla_s3/s4_runtime_contract.json``. This package keeps a
byte-identical copy next to this module so Isaac online code does not dual-write
chunk/K/gripper/workspace constants.

Does not load weights, start Isaac, or claim task success.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping

_PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_CONTRACT_JSON = _PACKAGE_DIR / 's4_runtime_contract.json'
CONTRACT_VERSION = 'smolvla_s3_s4_runtime_v0'


@dataclass(frozen=True)
class S4RuntimeContract:
    contract_version: str
    policy_action_semantics: str
    chunk_size: int
    n_action_steps: int
    control_rate_hz: float
    replan_period_s: float
    state_dim: int
    action_dim: int
    gripper_min: float
    gripper_max: float
    workspace_min: tuple[float, float, float]
    workspace_max: tuple[float, float, float]
    image_height: int
    image_width: int
    camera_key: str
    camera_keys: tuple[str, ...]
    claims_task_success: bool
    claims_sim2real: bool

    @property
    def execute_k(self) -> int:
        return self.n_action_steps

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload['workspace_min'] = list(self.workspace_min)
        payload['workspace_max'] = list(self.workspace_max)
        return payload


def contract_from_mapping(data: Mapping[str, Any]) -> S4RuntimeContract:
    required = (
        'contract_version',
        'policy_action_semantics',
        'chunk_size',
        'n_action_steps',
        'control_rate_hz',
        'replan_period_s',
        'state_dim',
        'action_dim',
        'gripper_min',
        'gripper_max',
        'workspace_min',
        'workspace_max',
        'image_height',
        'image_width',
        'camera_key',
        'camera_keys',
        'claims_task_success',
        'claims_sim2real',
    )
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError(f'S4 contract missing fields: {missing}')
    ws_min = tuple(float(v) for v in data['workspace_min'])
    ws_max = tuple(float(v) for v in data['workspace_max'])
    if len(ws_min) != 3 or len(ws_max) != 3:
        raise ValueError('workspace_min/max must have three components')
    camera_keys = tuple(str(key) for key in data['camera_keys'])
    if camera_keys != (
        'observation.images.scene',
        'observation.images.wrist',
    ):
        raise ValueError(f'unsupported S4 camera_keys={camera_keys!r}')
    return S4RuntimeContract(
        contract_version=str(data['contract_version']),
        policy_action_semantics=str(data['policy_action_semantics']),
        chunk_size=int(data['chunk_size']),
        n_action_steps=int(data['n_action_steps']),
        control_rate_hz=float(data['control_rate_hz']),
        replan_period_s=float(data['replan_period_s']),
        state_dim=int(data['state_dim']),
        action_dim=int(data['action_dim']),
        gripper_min=float(data['gripper_min']),
        gripper_max=float(data['gripper_max']),
        workspace_min=(ws_min[0], ws_min[1], ws_min[2]),
        workspace_max=(ws_max[0], ws_max[1], ws_max[2]),
        image_height=int(data['image_height']),
        image_width=int(data['image_width']),
        camera_key=str(data['camera_key']),
        camera_keys=camera_keys,
        claims_task_success=bool(data['claims_task_success']),
        claims_sim2real=bool(data['claims_sim2real']),
    )


def load_s4_runtime_contract(path: Path | None = None) -> S4RuntimeContract:
    target = Path(path) if path is not None else DEFAULT_CONTRACT_JSON
    if not target.is_file():
        raise FileNotFoundError(f'S4 runtime contract not found: {target}')
    data = json.loads(target.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'S4 contract must be a JSON object: {target}')
    contract = contract_from_mapping(data)
    if contract.contract_version != CONTRACT_VERSION:
        raise ValueError(
            f'unexpected S4 contract_version={contract.contract_version!r}, '
            f'expected {CONTRACT_VERSION!r}'
        )
    if contract.n_action_steps > contract.chunk_size:
        raise ValueError('n_action_steps (K) cannot exceed chunk_size')
    if contract.gripper_min != 0.0 or contract.gripper_max != 1.0:
        raise ValueError('gripper clip must be [0, 1]')
    if contract.claims_task_success or contract.claims_sim2real:
        raise ValueError('S4 contract must keep claims_* = false')
    return contract


def assert_runtime_matches_contract(
    *,
    chunk_size: int,
    n_action_steps: int,
    state_dim: int,
    action_dim: int,
    policy_action_semantics: str,
    contract: S4RuntimeContract | None = None,
) -> S4RuntimeContract:
    """Fail fast when online metadata drifts from the checked-in contract."""
    expected = contract if contract is not None else load_s4_runtime_contract()
    mismatches: list[str] = []
    checks = {
        'chunk_size': (chunk_size, expected.chunk_size),
        'n_action_steps': (n_action_steps, expected.n_action_steps),
        'state_dim': (state_dim, expected.state_dim),
        'action_dim': (action_dim, expected.action_dim),
        'policy_action_semantics': (
            policy_action_semantics,
            expected.policy_action_semantics,
        ),
    }
    for name, (actual, want) in checks.items():
        if actual != want:
            mismatches.append(f'{name}: actual={actual!r} contract={want!r}')
    if mismatches:
        raise AssertionError(
            'Isaac SmolVLA runtime drifted from S4 contract: '
            + '; '.join(mismatches)
        )
    return expected
