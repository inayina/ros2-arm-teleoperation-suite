"""Prove SceneACTRuntime walks the ACT chunk instead of re-taking step 0."""

from __future__ import annotations

from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from isaac_sim_adapter.scene_act_runtime import SceneACTRuntime


class _FakeACTPolicy:
    """Minimal stand-in for LeRobot ACTPolicy.select_action queueing."""

    def __init__(self, chunk: torch.Tensor) -> None:
        self._chunk = chunk
        self.config = SimpleNamespace(
            temporal_ensemble_coeff=None,
            n_action_steps=int(chunk.shape[1]),
        )
        self.predict_calls = 0
        self._action_queue: deque[torch.Tensor] = deque()

    def eval(self) -> None:
        return None

    def reset(self) -> None:
        self._action_queue = deque([], maxlen=self.config.n_action_steps)

    def predict_action_chunk(self, batch: dict[str, torch.Tensor]) -> torch.Tensor:
        del batch
        self.predict_calls += 1
        return self._chunk.clone()

    def select_action(self, batch: dict[str, torch.Tensor]) -> torch.Tensor:
        self.eval()
        if len(self._action_queue) == 0:
            actions = self.predict_action_chunk(batch)[:, : self.config.n_action_steps]
            self._action_queue.extend(actions.transpose(0, 1))
        return self._action_queue.popleft()


def test_scene_act_runtime_consumes_later_chunk_steps(monkeypatch) -> None:
    """Later chunk gripper closes must not be discarded by always taking [0, 0]."""

    chunk = torch.zeros(1, 5, 7, dtype=torch.float32)
    # Normalized actions: step 0 stays open-ish, later steps close.
    for index, gripper in enumerate([1.0, 0.8, 0.4, 0.1, 0.0]):
        chunk[0, index, 6] = gripper

    fake_policy = _FakeACTPolicy(chunk)

    def _fake_load(path, device):
        del path, device
        metadata = {
            'normalization': {
                'state_mean': [0.0] * 8,
                'state_std': [1.0] * 8,
                'action_mean': [0.0] * 7,
                'action_std': [1.0] * 7,
            },
            'chunk_size': 5,
            'release_id': 'unit_test',
        }
        return fake_policy, metadata

    monkeypatch.setattr(
        'isaac_sim_adapter.scene_act_runtime.load_scene_act_checkpoint',
        _fake_load,
    )
    monkeypatch.setattr(
        'isaac_sim_adapter.scene_act_runtime.preprocess_rgb',
        lambda rgb, torch_mod: torch_mod.zeros(3, 224, 224),
    )

    runtime = SceneACTRuntime.__new__(SceneACTRuntime)
    runtime.torch = torch
    runtime.device = 'cpu'
    runtime.policy, runtime.metadata = _fake_load(None, 'cpu')
    runtime.state_mean = torch.zeros(8)
    runtime.state_std = torch.ones(8)
    runtime.action_mean = torch.zeros(7)
    runtime.action_std = torch.ones(7)
    runtime.reset()

    rgb = np.zeros((240, 320, 3), dtype=np.uint8)
    state = [0.0] * 8
    grippers = [runtime.infer(state, rgb)[6] for _ in range(5)]

    assert fake_policy.predict_calls == 1
    assert grippers == pytest.approx([1.0, 0.8, 0.4, 0.1, 0.0])

    # Next call replans a fresh chunk instead of repeating step 0 forever.
    assert runtime.infer(state, rgb)[6] == pytest.approx(1.0)
    assert fake_policy.predict_calls == 2


def test_scene_act_runtime_short_n_action_steps_replans_sooner(monkeypatch) -> None:
    """Deploy-time n_action_steps < chunk_size must empty the queue earlier."""

    chunk = torch.zeros(1, 5, 7, dtype=torch.float32)
    for index, gripper in enumerate([1.0, 0.9, 0.8, 0.2, 0.0]):
        chunk[0, index, 6] = gripper
    fake_policy = _FakeACTPolicy(chunk)

    def _fake_load(path, device):
        del path, device
        metadata = {
            'normalization': {
                'state_mean': [0.0] * 8,
                'state_std': [1.0] * 8,
                'action_mean': [0.0] * 7,
                'action_std': [1.0] * 7,
            },
            'chunk_size': 5,
            'release_id': 'unit_test',
        }
        return fake_policy, metadata

    monkeypatch.setattr(
        'isaac_sim_adapter.scene_act_runtime.load_scene_act_checkpoint',
        _fake_load,
    )
    monkeypatch.setattr(
        'isaac_sim_adapter.scene_act_runtime.preprocess_rgb',
        lambda rgb, torch_mod: torch_mod.zeros(3, 224, 224),
    )

    runtime = SceneACTRuntime.__new__(SceneACTRuntime)
    runtime.torch = torch
    runtime.device = 'cpu'
    runtime.policy, runtime.metadata = _fake_load(None, 'cpu')
    runtime.policy.config.n_action_steps = 2
    runtime.metadata = {**runtime.metadata, 'deploy_n_action_steps': 2}
    runtime.state_mean = torch.zeros(8)
    runtime.state_std = torch.ones(8)
    runtime.action_mean = torch.zeros(7)
    runtime.action_std = torch.ones(7)
    runtime.reset()

    rgb = np.zeros((240, 320, 3), dtype=np.uint8)
    state = [0.0] * 8
    grippers = [runtime.infer(state, rgb)[6] for _ in range(4)]

    assert grippers == pytest.approx([1.0, 0.9, 1.0, 0.9])
    assert fake_policy.predict_calls == 2
