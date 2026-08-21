"""Regression tests for edge-triggered policy runtime Hold handling."""

from __future__ import annotations

from types import SimpleNamespace

from isaac_sim_adapter.smolvla_policy_inference_node import (
    IsaacSmolVLAPolicyInferenceNode,
)


class _Scheduler:
    def __init__(self) -> None:
        self.clear_reasons: list[str] = []

    def clear_queue(self, reason: str) -> None:
        self.clear_reasons.append(reason)


class _ExecutionAdapter:
    def __init__(self) -> None:
        self.hold_values: list[bool] = []

    def set_hold(self, active: bool) -> None:
        self.hold_values.append(active)


def _node_without_ros_init():
    node = object.__new__(IsaacSmolVLAPolicyInferenceNode)
    node._runtime_hold_active = False
    node._queue_hold_active = False
    node._runtime_hold_transition_count = 0
    node._shadow_scheduler = _Scheduler()
    node._shadow_execution_adapter = _ExecutionAdapter()
    node._shadow_lifecycle = SimpleNamespace(
        health=SimpleNamespace(hold_active=False)
    )
    node._shadow_last_chunk_started = 123.0
    node._execution_adapter_mode = 'shadow'
    return node


def test_repeated_runtime_hold_samples_are_idempotent() -> None:
    node = _node_without_ros_init()

    node._on_runtime_hold(SimpleNamespace(data=False))
    assert node._shadow_scheduler.clear_reasons == []
    assert node._shadow_execution_adapter.hold_values == []

    node._on_runtime_hold(SimpleNamespace(data=True))
    node._on_runtime_hold(SimpleNamespace(data=True))
    assert node._shadow_scheduler.clear_reasons == ['risk_r2_hold']
    assert node._shadow_execution_adapter.hold_values == [True]
    assert node._runtime_hold_active is True
    assert node._runtime_hold_transition_count == 1

    node._on_runtime_hold(SimpleNamespace(data=False))
    node._on_runtime_hold(SimpleNamespace(data=False))
    assert node._shadow_scheduler.clear_reasons == [
        'risk_r2_hold',
        'healthy_recovery_replan',
    ]
    assert node._shadow_execution_adapter.hold_values == [True, True]
    assert node._runtime_hold_active is False
    assert node._runtime_hold_transition_count == 2
    assert node._queue_hold_active is True
    assert node._shadow_lifecycle.health.hold_active is True
    assert node._shadow_last_chunk_started == 0.0
