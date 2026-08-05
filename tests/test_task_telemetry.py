"""Contract tests for phase/failure-onset Task GT timeline rows."""

from synth_data_gen.continuous_evaluator import ContinuousTaskEvaluator
from synth_data_gen.task_gt_live import TaskGtSnapshot
from synth_data_gen.task_telemetry import TaskTimelineTracker


def _snapshot(*, phase='HOVER', status='RUNNING', validity='VALID'):
    return TaskGtSnapshot(
        validity=validity,
        reason_code='task_running' if status == 'RUNNING' else 'task_fail',
        phase=phase,
        task_status=status,
        reach=None,
        grasp=None,
        lift=None,
        place=None,
        object_delta_m=(0.0, 0.0, 0.0) if validity == 'VALID' else None,
    )


def test_timeline_records_monotonic_phase_transition_without_success_claim():
    evaluator = ContinuousTaskEvaluator(validation_mode='lift')
    tracker = TaskTimelineTracker('run', 'episode')
    first = tracker.encode(
        _snapshot(), evaluator, event_sequence=0, monotonic_ns=10, ros_time_ns=20
    )
    second = tracker.encode(
        _snapshot(phase='DESCEND'),
        evaluator,
        event_sequence=1,
        monotonic_ns=30,
        ros_time_ns=40,
    )
    assert first['contract_version'] == 'panda_task_timeline_v1'
    assert first['phase_transition']['changed'] is False
    assert second['phase_transition'] == {
        'changed': True, 'from': 'HOVER', 'to': 'DESCEND'
    }
    assert second['claims_task_success'] is False
    assert second['claims_causal_proof'] is False


def test_observable_failure_has_exact_first_onset_and_is_sticky():
    evaluator = ContinuousTaskEvaluator(validation_mode='lift')
    tracker = TaskTimelineTracker('run', 'episode')
    tracker.encode(
        _snapshot(), evaluator, event_sequence=0, monotonic_ns=10, ros_time_ns=20
    )
    evaluator._estop_triggered = True
    onset = tracker.encode(
        _snapshot(), evaluator, event_sequence=1, monotonic_ns=30, ros_time_ns=40
    )
    later = tracker.encode(
        _snapshot(), evaluator, event_sequence=2, monotonic_ns=50, ros_time_ns=60
    )
    assert onset['failure_onset']['kind'] == 'observed_event'
    assert onset['failure_onset']['event_type'] == 'estop'
    assert onset['failure_onset']['onset_is_exact'] is True
    assert onset['failure_onset']['first_onset_in_row'] is True
    assert later['failure_onset']['onset_monotonic_ns'] == 30
    assert later['failure_onset']['first_onset_in_row'] is False


def test_terminal_nonachievement_is_not_backdated_or_called_exact():
    evaluator = ContinuousTaskEvaluator(validation_mode='lift')
    tracker = TaskTimelineTracker('run', 'episode')
    tracker.encode(
        _snapshot(), evaluator, event_sequence=0, monotonic_ns=10, ros_time_ns=20
    )
    final = tracker.encode(
        _snapshot(phase='DONE', status='FAIL'),
        evaluator,
        event_sequence=1,
        monotonic_ns=90,
        ros_time_ns=100,
        final_failure_stage='reach',
        final_failure_reason='reach not achieved by deadline',
    )
    assert final['failure_onset']['kind'] == 'terminal_nonachievement'
    assert final['failure_onset']['onset_monotonic_ns'] == 90
    assert final['failure_onset']['onset_is_exact'] is False
    assert final['failure_onset']['stage'] == 'reach'


def test_unavailable_snapshot_cannot_create_failure_onset():
    evaluator = ContinuousTaskEvaluator(validation_mode='lift')
    row = TaskTimelineTracker('run', 'episode').encode(
        _snapshot(phase='IDLE', status='UNAVAILABLE', validity='UNAVAILABLE'),
        evaluator,
        event_sequence=0,
        monotonic_ns=10,
        ros_time_ns=20,
    )
    assert row['failure_onset']['kind'] == 'unavailable'
    assert row['failure_onset']['onset_monotonic_ns'] is None
