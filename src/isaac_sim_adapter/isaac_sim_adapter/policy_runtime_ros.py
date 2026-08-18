"""ROS mapping for M1 shadow policy-runtime telemetry."""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from isaac_sim_adapter.policy_execution_adapter import PolicyExecutionDecision
from isaac_sim_adapter.policy_runtime import CONTRACT_VERSION
from isaac_sim_adapter.policy_runtime import RuntimeHealth, ShadowPolicyCommand
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def policy_command_qos(expected_period_s: float, ttl_s: float) -> QoSProfile:
    if expected_period_s <= 0.0 or ttl_s <= 0.0:
        raise ValueError('period and TTL must be positive')
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        deadline=Duration(seconds=1.5 * expected_period_s),
        lifespan=Duration(seconds=ttl_s),
        liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
        liveliness_lease_duration=Duration(seconds=2.0 * expected_period_s),
    )


def runtime_health_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
    )


def populate_policy_command(
    message,
    command: ShadowPolicyCommand,
    *,
    source_stamp,
    received_stamp,
    valid_until,
):
    """Populate a generated PolicyCommand without importing it at module load."""
    message.header.stamp = source_stamp
    message.received_stamp = received_stamp
    message.valid_until = valid_until
    message.contract_version = CONTRACT_VERSION
    message.event_id = command.event_id
    message.parent_event_id = command.parent_event_id
    message.trace_run_id = command.trace_run_id
    message.episode_id = command.episode_id
    message.validity = 'VALID'
    message.reason_code = 'none'
    message.observation_sequence = command.observation_sequence
    message.command_sequence = command.command_sequence
    message.action_schema_version = command.action_schema_version
    message.action = list(command.action)
    message.chunk_index = command.chunk_index
    message.chunk_size = command.chunk_size
    message.from_prefetched_chunk = command.from_prefetched_chunk
    message.inference_latency_ms = command.inference_latency_ms
    message.claims_task_success = False
    return message


def populate_execution_report(
    message,
    decision: PolicyExecutionDecision,
    *,
    source_stamp,
    received_stamp,
):
    """Populate generated PolicyExecutionReport for the M2 shadow topic."""
    message.header.stamp = source_stamp
    message.received_stamp = received_stamp
    message.contract_version = CONTRACT_VERSION
    message.event_id = decision.event_id
    message.parent_event_id = decision.parent_event_id
    message.trace_run_id = decision.trace_run_id
    message.episode_id = decision.episode_id
    message.validity = decision.validity
    message.reason_code = decision.reason_code
    message.command_sequence = decision.command_sequence
    message.accepted = decision.accepted
    message.decision = decision.decision
    message.source_action_schema_version = (
        decision.source_action_schema_version
    )
    message.execution_action_schema_version = (
        decision.execution_action_schema_version
    )
    message.source_action = list(decision.source_action)
    message.has_bounded_action = decision.bounded_action is not None
    message.bounded_action = (
        [] if decision.bounded_action is None else list(decision.bounded_action)
    )
    message.clipped = decision.clipped
    message.clip_axes = list(decision.clip_axes)
    message.hold_active = decision.hold_active
    message.estop_active = decision.estop_active
    message.adapter_name = decision.adapter_name
    message.adapter_version = decision.adapter_version
    message.claims_task_success = False
    return message


def build_runtime_health_array(
    health: RuntimeHealth,
    *,
    stamp,
    policy_name: str,
    policy_version: str,
    checkpoint_hash: str,
    observation_schema_version: str,
    shadow_only: bool = True,
    trace_run_id: str = '',
    episode_id: str = '',
) -> DiagnosticArray:
    level = {
        'VALID': DiagnosticStatus.OK,
        'WARMING_UP': DiagnosticStatus.WARN,
        'STALE': DiagnosticStatus.WARN,
        'UNAVAILABLE': DiagnosticStatus.WARN,
        'ERROR': DiagnosticStatus.ERROR,
    }[health.validity]

    def value(key: str, raw) -> KeyValue:
        if raw is None:
            rendered = 'unavailable'
        elif isinstance(raw, bool):
            rendered = str(raw).lower()
        else:
            rendered = str(raw)
        return KeyValue(key=key, value=rendered)

    status = DiagnosticStatus(
        level=level,
        name='policy_runtime/brain',
        message=health.reason_code,
        hardware_id='panda_policy_runtime_v1',
        values=[
            value('contract_version', CONTRACT_VERSION),
            value('contract_sha256', health.contract_sha256),
            value('trace_run_id', trace_run_id),
            value('episode_id', episode_id),
            value('lifecycle_state', health.lifecycle_state.value),
            value('validity', health.validity),
            value('reason_code', health.reason_code),
            value('policy_name', policy_name),
            value('policy_version', policy_version),
            value('checkpoint_hash', checkpoint_hash),
            value('observation_schema_version', observation_schema_version),
            value('policy_loaded', health.policy_loaded),
            value('observation_age_ms', health.observation_age_ms),
            value('inference_busy', health.inference_busy),
            value('inference_latency_ms_last', health.inference_latency_ms_last),
            value('inference_latency_ms_p50', health.inference_latency_ms_p50),
            value('inference_latency_ms_p95', health.inference_latency_ms_p95),
            value('inference_request_count', health.inference_request_count),
            value('inference_started_count', health.inference_started_count),
            value('inference_completion_count', health.inference_completion_count),
            value('inference_failure_count', health.inference_failure_count),
            value(
                'pending_observation_drop_count',
                health.pending_observation_drop_count,
            ),
            value('completed_result_drop_count', health.completed_result_drop_count),
            value('warmup_started_count', health.warmup_started_count),
            value('warmup_completion_count', health.warmup_completion_count),
            value('warmup_failure_count', health.warmup_failure_count),
            value('queue_depth', health.queue_depth),
            value('queue_underrun_count', health.queue_underrun_count),
            value('deadline_miss_count', health.deadline_miss_count),
            value('last_command_sequence', health.last_command_sequence),
            value(
                'last_successful_command_age_ms',
                health.last_successful_command_age_ms,
            ),
            value('hold_active', health.hold_active),
            value('estop_active', health.estop_active),
            value('failure_lane', health.failure_lane),
            value('claims_task_success', False),
            value('shadow_only', shadow_only),
        ],
    )
    array = DiagnosticArray()
    array.header.stamp = stamp
    array.status = [status]
    return array
