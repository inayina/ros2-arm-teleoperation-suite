"""CPU and recorded-telemetry parity tests for M2 shadow execution."""

from __future__ import annotations

from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'isaac_sim_adapter'))

from isaac_sim_adapter.policy_execution_adapter import (  # noqa: E402
    ExecutionState,
    legacy_absolute_result,
    PandaPolicyExecutionAdapter,
    resolve_execution_adapter_mode,
    validate_authoritative_publisher_counts,
)
from isaac_sim_adapter.policy_runtime import (  # noqa: E402
    ABSOLUTE_ACTION_SCHEMA,
    DELTA_ACTION_SCHEMA,
    ShadowPolicyCommand,
)
from isaac_sim_adapter.policy_runtime_ros import (  # noqa: E402
    populate_execution_report,
)
import pytest  # noqa: E402


WORKSPACE_MIN = (0.20, -0.40, 0.02)
WORKSPACE_MAX = (0.65, 0.40, 0.75)
STATE = ExecutionState((0.40, 0.0, 0.30), (0.0, 0.0, 0.0, 1.0))
RECORDED_ROOT = Path(
    '/home/ina/robot-sim-lab/robot-arm-episode-data-lab/'
    'evidence/smolvla_s4_bounded5_relight_20260724T151711Z/trials'
)
MIDSTREAM_SCHEMA = Path(
    '/home/ina/robot-sim-lab/robot-arm-episode-data-lab/'
    'evaluation/schemas/policy_runtime_contract.schema.json'
)


def _command(
    action=(0.42, 0.0, 0.31, 0.0, 0.0, 0.0, 1.0, 0.7),
    *,
    sequence=0,
    schema=ABSOLUTE_ACTION_SCHEMA,
    valid_until=2_000,
) -> ShadowPolicyCommand:
    return ShadowPolicyCommand(
        event_id=f'command:episode:{sequence}',
        parent_event_id='observation:episode:0',
        trace_run_id='trace_m2',
        episode_id='episode_m2',
        observation_sequence=0,
        command_sequence=sequence,
        action_schema_version=schema,
        action=tuple(action),
        chunk_index=sequence,
        chunk_size=10,
        from_prefetched_chunk=sequence > 0,
        inference_latency_ms=25.0,
        valid_until_monotonic_ns=valid_until,
    )


def _adapter() -> PandaPolicyExecutionAdapter:
    return PandaPolicyExecutionAdapter(
        workspace_min=WORKSPACE_MIN,
        workspace_max=WORKSPACE_MAX,
    )


def test_absolute_adapter_matches_legacy_below_one_e_minus_six() -> None:
    command = _command()
    decision = _adapter().evaluate(command, STATE, now_monotonic_ns=1_000)
    legacy = legacy_absolute_result(
        command.action,
        workspace_min=WORKSPACE_MIN,
        workspace_max=WORKSPACE_MAX,
    )
    assert decision.accepted is True
    assert decision.decision == 'EXECUTED'
    assert decision.bounded_action is not None
    assert max(abs(a - b) for a, b in zip(legacy[0], decision.bounded_action[:3])) <= 1e-6
    assert max(abs(a - b) for a, b in zip(legacy[1], decision.bounded_action[3:7])) <= 1e-6
    assert abs(legacy[2] - decision.bounded_action[7]) <= 1e-6
    assert legacy[3] == decision.clipped
    assert decision.claims_task_success is False


def test_absolute_clip_axes_are_explicit() -> None:
    decision = _adapter().evaluate(
        _command((0.9, -0.8, 0.9, 0.0, 0.0, 0.0, 1.0, 1.4)),
        STATE,
        now_monotonic_ns=1_000,
    )
    assert decision.accepted is True
    assert decision.clipped is True
    assert decision.clip_axes == ('x', 'y', 'z', 'gripper')
    assert decision.reason_code == 'workspace_clipped'


def test_gripper_only_clip_uses_specific_reason() -> None:
    decision = _adapter().evaluate(
        _command((0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0, 1.4)),
        STATE,
        now_monotonic_ns=1_000,
    )
    assert decision.clip_axes == ('gripper',)
    assert decision.reason_code == 'gripper_clipped'


def test_delta7_converts_to_bounded_absolute8() -> None:
    command = _command(
        (0.01, -0.02, 0.03, 0.1, 0.0, -0.1, 0.4),
        schema=DELTA_ACTION_SCHEMA,
    )
    decision = _adapter().evaluate(command, STATE, now_monotonic_ns=1_000)
    assert decision.accepted is True
    assert decision.bounded_action is not None
    assert len(decision.bounded_action) == 8
    assert decision.bounded_action[:3] == pytest.approx((0.41, -0.02, 0.33))
    assert decision.bounded_action[7] == pytest.approx(0.4)


def test_delta_limit_uses_soft_limit_reason() -> None:
    decision = _adapter().evaluate(
        _command((0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4), schema=DELTA_ACTION_SCHEMA),
        STATE,
        now_monotonic_ns=1_000,
    )
    assert decision.clip_axes == ('x',)
    assert decision.reason_code == 'soft_limit'


@pytest.mark.parametrize(
    ('mutation', 'reason'),
    [
        ({'contract_version': 'wrong'}, 'contract_mismatch'),
        ({'action': (0.0,) * 7}, 'action_dimension_invalid'),
        ({'action': (math.nan,) + (0.0,) * 7}, 'action_non_finite'),
        ({'valid_until_monotonic_ns': 999}, 'command_ttl_expired'),
    ],
)
def test_invalid_commands_fail_closed(mutation, reason) -> None:
    decision = _adapter().evaluate(
        replace(_command(), **mutation), STATE, now_monotonic_ns=1_000
    )
    assert decision.accepted is False
    assert decision.decision == 'REJECTED'
    assert decision.reason_code == reason
    assert decision.bounded_action is None


def test_duplicate_or_regressed_sequence_is_rejected() -> None:
    adapter = _adapter()
    assert adapter.evaluate(_command(sequence=4), STATE, now_monotonic_ns=1_000).accepted
    duplicate = adapter.evaluate(_command(sequence=4), STATE, now_monotonic_ns=1_000)
    assert duplicate.reason_code == 'command_sequence_regression'


def test_hold_and_estop_decisions_do_not_execute() -> None:
    adapter = _adapter()
    adapter.set_hold(True)
    held = adapter.evaluate(_command(sequence=0), STATE, now_monotonic_ns=1_000)
    assert held.decision == 'HELD' and held.accepted is False
    adapter.set_hold(False)
    adapter.set_estop(True)
    stopped = adapter.evaluate(_command(sequence=1), STATE, now_monotonic_ns=1_000)
    assert stopped.decision == 'ESTOPPED' and stopped.accepted is False


def test_execution_report_ros_mapping_is_shadow_auditable() -> None:
    decision = _adapter().evaluate(_command(), STATE, now_monotonic_ns=1_000)
    message = SimpleNamespace(header=SimpleNamespace())
    stamp = SimpleNamespace(sec=1, nanosec=0)
    report = populate_execution_report(
        message, decision, source_stamp=stamp, received_stamp=stamp
    )
    assert report.contract_version == 'panda_policy_runtime_v1'
    assert report.adapter_name.endswith('_shadow')
    assert report.has_bounded_action is True
    assert len(report.bounded_action) == 8
    assert report.claims_task_success is False


def test_m4_mode_contract_is_explicit_and_backward_compatible() -> None:
    assert resolve_execution_adapter_mode(
        'legacy', shadow_enabled=False, dry_run=False
    ) == 'legacy'
    assert resolve_execution_adapter_mode(
        'legacy', shadow_enabled=True, dry_run=True
    ) == 'shadow'
    assert resolve_execution_adapter_mode(
        'authoritative', shadow_enabled=False, dry_run=False
    ) == 'authoritative'
    with pytest.raises(ValueError, match='requires dry_run=false'):
        resolve_execution_adapter_mode(
            'authoritative', shadow_enabled=False, dry_run=True
        )


def test_m4_authoritative_adapter_identity_and_single_publisher_gate() -> None:
    adapter = PandaPolicyExecutionAdapter(
        workspace_min=WORKSPACE_MIN,
        workspace_max=WORKSPACE_MAX,
        execution_mode='authoritative',
    )
    decision = adapter.evaluate(_command(), STATE, now_monotonic_ns=1_000)
    assert decision.adapter_name.endswith('_authoritative')
    assert decision.adapter_version == 'm4_v1'
    validate_authoritative_publisher_counts(1, 1)
    with pytest.raises(RuntimeError, match='publisher identity mismatch'):
        validate_authoritative_publisher_counts(2, 1)


def test_generated_execution_report_round_trips_over_mock_ros(
    monkeypatch, tmp_path: Path
) -> None:
    rclpy = pytest.importorskip('rclpy')
    messages = pytest.importorskip('teleop_interfaces.msg')
    from isaac_sim_adapter.policy_runtime_ros import runtime_health_qos

    PolicyExecutionReport = messages.PolicyExecutionReport
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    rclpy.init()
    publisher_node = rclpy.create_node('policy_execution_m2_mock_publisher')
    subscriber_node = rclpy.create_node('policy_execution_m2_mock_subscriber')
    received = []
    qos = runtime_health_qos()
    publisher = publisher_node.create_publisher(
        PolicyExecutionReport, '/policy/execution_report_m2_mock', qos
    )
    subscriber_node.create_subscription(
        PolicyExecutionReport,
        '/policy/execution_report_m2_mock',
        received.append,
        qos,
    )
    try:
        decision = _adapter().evaluate(
            _command(), STATE, now_monotonic_ns=1_000
        )
        now = publisher_node.get_clock().now().to_msg()
        message = populate_execution_report(
            PolicyExecutionReport(),
            decision,
            source_stamp=now,
            received_stamp=now,
        )
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(subscriber_node, timeout_sec=0.05)
        assert len(received) == 1
        assert received[0].decision == 'EXECUTED'
        assert received[0].adapter_name.endswith('_shadow')
        assert received[0].claims_task_success is False
    finally:
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()


def test_execution_decision_validates_against_m0_json_schema_when_present() -> None:
    if not MIDSTREAM_SCHEMA.is_file():
        pytest.skip('canonical M0 schema checkout not present')
    jsonschema = pytest.importorskip('jsonschema')
    decision = _adapter().evaluate(_command(), STATE, now_monotonic_ns=1_000)
    payload = {
        'contract_version': 'panda_policy_runtime_v1',
        'artifact_type': 'policy_execution_report',
        'event_id': decision.event_id,
        'parent_event_id': decision.parent_event_id,
        'trace_run_id': decision.trace_run_id,
        'episode_id': decision.episode_id,
        'source_stamp': {'sec': 1, 'nanosec': 0},
        'received_stamp': {'sec': 1, 'nanosec': 1},
        'validity': decision.validity,
        'reason_code': decision.reason_code,
        'command_sequence': decision.command_sequence,
        'accepted': decision.accepted,
        'decision': decision.decision,
        'source_action_schema_version': decision.source_action_schema_version,
        'execution_action_schema_version': (
            decision.execution_action_schema_version
        ),
        'source_action': list(decision.source_action),
        'bounded_action': list(decision.bounded_action),
        'clipped': decision.clipped,
        'clip_axes': list(decision.clip_axes),
        'hold_active': decision.hold_active,
        'estop_active': decision.estop_active,
        'adapter_name': decision.adapter_name,
        'adapter_version': decision.adapter_version,
        'claims_task_success': False,
    }
    schema = json.loads(MIDSTREAM_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft202012Validator(schema).validate(payload)


def test_recorded_s4_actions_match_legacy_parity_when_present() -> None:
    paths = sorted(RECORDED_ROOT.glob('seed_*/report.json'))
    if not paths:
        pytest.skip('canonical S4 recorded telemetry not present')
    checked = 0
    for path in paths:
        payload = json.loads(path.read_text(encoding='utf-8'))
        adapter = _adapter()
        for sequence, row in enumerate(payload['actions']):
            source_pose = row['source_ee_pose']
            state = ExecutionState(
                tuple(source_pose[:3]), tuple(source_pose[3:7])
            )
            command = _command(
                row['raw_action'], sequence=sequence, valid_until=10_000
            )
            decision = adapter.evaluate(
                command, state, now_monotonic_ns=1_000
            )
            assert decision.accepted
            assert decision.bounded_action == pytest.approx(
                row['bounded_action'], abs=1e-12
            )
            assert decision.clipped == row['action_clipped']
            checked += 1
    assert checked == 750
