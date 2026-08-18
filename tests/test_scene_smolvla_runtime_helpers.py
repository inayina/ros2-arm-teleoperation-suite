"""CPU unit tests for SmolVLA S4 runtime helpers (no weight load)."""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'isaac_sim_adapter'))

from isaac_sim_adapter.s4_runtime_contract import (  # noqa: E402
    DEFAULT_CONTRACT_JSON,
    assert_runtime_matches_contract,
    load_s4_runtime_contract,
)
from isaac_sim_adapter.scene_smolvla_runtime import (  # noqa: E402
    CAMERA_KEYS,
    IMAGE_HW,
    SCENE_IMAGE_KEY,
    compose_state15,
    rgb_uint8_to_chw01,
)

MIDSTREAM_CONTRACT_JSON = Path(
    '/home/ina/robot-sim-lab/robot-arm-episode-data-lab/configs/smolvla_s3'
    '/s4_runtime_contract.json'
)


def test_compose_state15_layout():
    state = compose_state15(
        [0.1] * 7,
        [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
        0.85,
    )
    assert state.shape == (15,)
    assert state[:7].tolist() == pytest.approx([0.1] * 7)
    assert float(state[14]) == pytest.approx(0.85)


def test_rgb_uint8_to_chw01_shape():
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[10, 20] = (255, 128, 64)
    chw = rgb_uint8_to_chw01(rgb, 240, 320)
    assert chw.shape == (3, 240, 320)
    assert float(chw.min()) >= 0.0
    assert float(chw.max()) <= 1.0


def test_s4_runtime_contract_defaults_match_recovery():
    contract = load_s4_runtime_contract()
    assert contract.contract_version == 'smolvla_s3_s4_runtime_v0'
    assert contract.chunk_size == 10
    assert contract.n_action_steps == 5
    assert contract.execute_k == 5
    assert contract.control_rate_hz == 10.0
    assert contract.state_dim == 15
    assert contract.action_dim == 8
    assert contract.camera_keys == (
        'observation.images.scene',
        'observation.images.wrist',
    )
    assert contract.gripper_min == 0.0
    assert contract.gripper_max == 1.0
    assert contract.workspace_min == (0.20, -0.40, 0.02)
    assert contract.workspace_max == (0.65, 0.40, 0.75)
    assert IMAGE_HW == (contract.image_height, contract.image_width)
    assert SCENE_IMAGE_KEY == contract.camera_key
    assert CAMERA_KEYS == contract.camera_keys
    assert contract.claims_task_success is False
    assert_runtime_matches_contract(
        chunk_size=10,
        n_action_steps=5,
        state_dim=15,
        action_dim=8,
        policy_action_semantics='absolute_eef_gripper_v0',
        contract=contract,
    )
    with pytest.raises(AssertionError, match='drifted'):
        assert_runtime_matches_contract(
            chunk_size=10,
            n_action_steps=4,
            state_dim=15,
            action_dim=8,
            policy_action_semantics='absolute_eef_gripper_v0',
            contract=contract,
        )


def test_s4_contract_json_matches_midstream_when_present():
    assert DEFAULT_CONTRACT_JSON.is_file()
    if not MIDSTREAM_CONTRACT_JSON.is_file():
        pytest.skip('midstream checkout not present beside upstream')
    up = DEFAULT_CONTRACT_JSON.read_bytes()
    mid = MIDSTREAM_CONTRACT_JSON.read_bytes()
    assert hashlib.sha256(up).hexdigest() == hashlib.sha256(mid).hexdigest()
    payload = json.loads(up.decode('utf-8'))
    assert payload['chunk_size'] == 10
    assert payload['n_action_steps'] == 5
