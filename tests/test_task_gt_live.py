"""Contract tests for the live ContinuousTaskEvaluator mirror."""

import importlib.util
from pathlib import Path
import time

import pytest

from synth_data_gen.continuous_evaluator import (
    ContinuousTaskEvaluator,
    EvaluatorSample,
)
from synth_data_gen.task_gt_live import build_task_gt_snapshot
from synth_data_gen.task_gt_live import populate_task_evaluation_status


def _evaluator() -> ContinuousTaskEvaluator:
    evaluator = ContinuousTaskEvaluator(validation_mode='lift')
    evaluator.reset(
        initial_object_xyz=(0.4, 0.0, 0.02),
        bin_xy=(0.4, -0.35),
        reset_monotonic_s=0.0,
    )
    return evaluator


def test_waiting_for_object_is_explicitly_unavailable() -> None:
    snapshot = build_task_gt_snapshot(
        _evaluator(),
        initialized=False,
        current_object_xyz=None,
    )
    assert snapshot.validity == 'UNAVAILABLE'
    assert snapshot.task_status == 'UNAVAILABLE'
    assert snapshot.reason_code == 'waiting_object_pose'
    assert snapshot.reach is None


def test_running_snapshot_keeps_unreached_subgoals_unknown() -> None:
    evaluator = _evaluator()
    evaluator.observe(EvaluatorSample(
        t_monotonic=1.0,
        object_xyz=(0.4, 0.0, 0.02),
        ee_xyz=(0.4, 0.0, 0.10),
        gripper=1.0,
    ))
    snapshot = build_task_gt_snapshot(
        evaluator,
        initialized=True,
        current_object_xyz=(0.4, 0.0, 0.02),
    )
    assert snapshot.task_status == 'RUNNING'
    assert snapshot.phase == 'DESCEND'
    assert snapshot.reach is True
    assert snapshot.grasp is None
    assert snapshot.lift is None
    assert snapshot.place is None


def test_final_failure_materializes_false_outcomes_and_delta() -> None:
    evaluator = _evaluator()
    snapshot = build_task_gt_snapshot(
        evaluator,
        initialized=True,
        current_object_xyz=(0.41, -0.01, 0.02),
        final_success=False,
    )
    assert snapshot.validity == 'VALID'
    assert snapshot.task_status == 'FAIL'
    assert snapshot.phase == 'DONE'
    assert snapshot.reach is False
    assert snapshot.grasp is False
    assert snapshot.lift is False
    assert snapshot.place is False
    assert snapshot.object_delta_m == pytest.approx((0.01, -0.01, 0.0))


def test_live_source_never_claims_task_success_contract_field() -> None:
    messages = pytest.importorskip('teleop_interfaces.msg')
    stamp_type = pytest.importorskip('builtin_interfaces.msg').Time
    snapshot = build_task_gt_snapshot(
        _evaluator(),
        initialized=True,
        current_object_xyz=(0.4, 0.0, 0.02),
    )
    message = populate_task_evaluation_status(
        messages.TaskEvaluationStatus(),
        snapshot,
        stamp=stamp_type(sec=3, nanosec=4),
        trace_run_id='trace_live_test',
        episode_id='episode_0001',
        event_sequence=7,
    )
    assert message.event_id == 'task_gt:trace_live_test:7'
    assert message.task_status == 'RUNNING'
    assert message.reach == message.OUTCOME_UNKNOWN
    assert message.gt_source == 'upstream_continuous_task_evaluator'
    assert message.risk_may_override is False
    assert message.claims_task_success is False


def test_live_publisher_round_trips_task_gt_over_ros(
    monkeypatch, tmp_path: Path
) -> None:
    rclpy = pytest.importorskip('rclpy')
    messages = pytest.importorskip('teleop_interfaces.msg')
    script_path = Path(__file__).parents[1] / 'scripts' / (
        'isaac_continuous_gt_recorder.py'
    )
    spec = importlib.util.spec_from_file_location('task_gt_recorder_test', script_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    rclpy.init()
    evaluator = _evaluator()
    publisher_node = module.IsaacContinuousGtNode(
        evaluator=evaluator,
        bin_xy=(0.4, -0.35),
        trace_run_id='trace_ros_test',
        episode_id='episode_ros_test',
    )
    subscriber_node = rclpy.create_node('task_gt_live_test_subscriber')
    received = []
    subscriber_node.create_subscription(
        messages.TaskEvaluationStatus,
        '/task/evaluation_status',
        received.append,
        10,
    )
    publisher_node._object = (0.4, 0.0, 0.02)
    publisher_node._ee = (0.4, 0.0, 0.10)
    try:
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            publisher_node._tick()
            rclpy.spin_once(subscriber_node, timeout_sec=0.05)
        assert len(received) == 1
        assert received[0].trace_run_id == 'trace_ros_test'
        assert received[0].task_status == 'RUNNING'
        assert received[0].gt_source == 'upstream_continuous_task_evaluator'
        assert received[0].claims_task_success is False
    finally:
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()
