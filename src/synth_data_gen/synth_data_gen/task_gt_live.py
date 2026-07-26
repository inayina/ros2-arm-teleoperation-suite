"""Pure mapping from ContinuousTaskEvaluator state to the live Task GT contract."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from synth_data_gen.continuous_evaluator import ContinuousTaskEvaluator, XYZ


CONTRACT_VERSION = 'panda_policy_runtime_v1'
GT_SOURCE = 'upstream_continuous_task_evaluator'


@dataclass(frozen=True)
class TaskGtSnapshot:
    """ROS-independent task status used by the publisher and contract tests."""

    validity: str
    reason_code: str
    phase: str
    task_status: str
    reach: Optional[bool]
    grasp: Optional[bool]
    lift: Optional[bool]
    place: Optional[bool]
    object_delta_m: Optional[XYZ]


def _running_phase(evaluator: ContinuousTaskEvaluator) -> str:
    raw_hint = str(evaluator._phase or '').strip().upper()
    aliases = {
        'VALIDATE': 'DONE',
        'APPROACH': 'HOVER',
        'APPROACH_XY': 'HOVER',
        'GRASP': 'CLOSE',
    }
    hint = aliases.get(raw_hint, raw_hint)
    allowed = {
        'IDLE', 'HOVER', 'DESCEND', 'CLOSE', 'LIFT',
        'TRANSPORT', 'PLACE', 'RELEASE', 'DONE',
    }
    if hint in allowed and hint != 'IDLE':
        return hint
    if evaluator._release:
        return 'RELEASE'
    if evaluator._place:
        return 'PLACE'
    if evaluator._transport:
        return 'TRANSPORT'
    if evaluator._lift:
        return 'LIFT'
    if evaluator._grasp:
        return 'CLOSE'
    if evaluator._reach:
        return 'DESCEND'
    return 'IDLE' if raw_hint == 'RESET' else 'HOVER'


def build_task_gt_snapshot(
    evaluator: ContinuousTaskEvaluator,
    *,
    initialized: bool,
    current_object_xyz: Optional[XYZ],
    final_success: Optional[bool] = None,
) -> TaskGtSnapshot:
    """Build a fail-closed live snapshot without re-evaluating task physics."""
    if not initialized:
        return TaskGtSnapshot(
            validity='UNAVAILABLE',
            reason_code='waiting_object_pose',
            phase='IDLE',
            task_status='UNAVAILABLE',
            reach=None,
            grasp=None,
            lift=None,
            place=None,
            object_delta_m=None,
        )

    final = final_success is not None

    def outcome(value: bool) -> Optional[bool]:
        return bool(value) if final or value else None

    delta = None
    initial = evaluator._initial_object_xyz
    if initial is not None and current_object_xyz is not None:
        delta = tuple(
            float(current_object_xyz[i]) - float(initial[i]) for i in range(3)
        )
    return TaskGtSnapshot(
        validity='VALID',
        reason_code=(
            'task_pass' if final_success is True
            else 'task_fail' if final_success is False
            else 'task_running'
        ),
        phase='DONE' if final else _running_phase(evaluator),
        task_status=(
            'PASS' if final_success is True
            else 'FAIL' if final_success is False
            else 'RUNNING'
        ),
        reach=outcome(evaluator._reach),
        grasp=outcome(evaluator._grasp),
        lift=outcome(evaluator._lift),
        place=outcome(evaluator._place),
        object_delta_m=delta,
    )


def populate_task_evaluation_status(
    message: Any,
    snapshot: TaskGtSnapshot,
    *,
    stamp: Any,
    trace_run_id: str,
    episode_id: str,
    event_sequence: int,
    parent_event_id: str = '',
) -> Any:
    """Populate the generated ROS message without changing task semantics."""

    def outcome_value(value: Optional[bool]) -> int:
        if value is None:
            return message.OUTCOME_UNKNOWN
        return message.OUTCOME_TRUE if value else message.OUTCOME_FALSE

    message.header.stamp = stamp
    message.header.frame_id = GT_SOURCE
    message.received_stamp = stamp
    message.contract_version = CONTRACT_VERSION
    message.event_id = f'task_gt:{trace_run_id}:{event_sequence}'
    message.parent_event_id = parent_event_id
    message.trace_run_id = trace_run_id
    message.episode_id = episode_id
    message.validity = snapshot.validity
    message.reason_code = snapshot.reason_code
    message.phase = snapshot.phase
    message.task_status = snapshot.task_status
    message.reach = outcome_value(snapshot.reach)
    message.grasp = outcome_value(snapshot.grasp)
    message.lift = outcome_value(snapshot.lift)
    message.place = outcome_value(snapshot.place)
    message.has_object_delta = snapshot.object_delta_m is not None
    if snapshot.object_delta_m is not None:
        message.object_delta_x_m = snapshot.object_delta_m[0]
        message.object_delta_y_m = snapshot.object_delta_m[1]
        message.object_delta_z_m = snapshot.object_delta_m[2]
    message.gt_source = GT_SOURCE
    message.risk_may_override = False
    message.claims_task_success = False
    return message
