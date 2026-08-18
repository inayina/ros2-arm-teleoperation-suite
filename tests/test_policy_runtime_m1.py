"""CPU-only M1 tests for shadow policy runtime contracts."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'isaac_sim_adapter'))

from isaac_sim_adapter.policy_runtime import (  # noqa: E402
    ABSOLUTE_ACTION_SCHEMA,
    ActionChunkEnvelope,
    AsyncChunkInferenceWorker,
    classify_runtime_error,
    CONTRACT_DESCRIPTOR_SHA256,
    CONTRACT_LOCK_CONTENT_SHA256,
    EpisodeContext,
    LifecycleState,
    PolicyArtifact,
    PolicyBackend,
    PolicyRuntimeStateMachine,
    RawObservation,
    RuntimeHealth,
    ShadowCommandScheduler,
    SmolVlaPolicyBackend,
)
from isaac_sim_adapter.policy_runtime_ros import (  # noqa: E402
    build_runtime_health_array,
    policy_command_qos,
    populate_policy_command,
)
from isaac_sim_adapter.remote_policy_client import (  # noqa: E402
    RemoteSmolVlaPolicyBackend,
)
import pytest  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    ReliabilityPolicy,
)


MIDSTREAM_LOCK = Path(
    '/home/ina/robot-sim-lab/robot-arm-episode-data-lab/'
    'configs/policy_runtime/panda_policy_runtime_v1.lock.json'
)


def _envelope(
    *,
    observation_sequence: int = 7,
    captured_ns: int = 1_000_000_000,
) -> ActionChunkEnvelope:
    return ActionChunkEnvelope(
        observation_sequence=observation_sequence,
        observation_captured_monotonic_ns=captured_ns,
        action_schema_version=ABSOLUTE_ACTION_SCHEMA,
        actions=(
            (0.40, 0.00, 0.30, 0.0, 0.0, 0.0, 1.0, 0.8),
            (0.41, 0.00, 0.29, 0.0, 0.0, 0.0, 1.0, 0.7),
        ),
        execute_k=2,
        inference_started_monotonic_ns=1_010_000_000,
        inference_finished_monotonic_ns=1_035_000_000,
        from_native_chunk=True,
    )


def _active_scheduler() -> tuple[ShadowCommandScheduler, RuntimeHealth]:
    health = RuntimeHealth(policy_loaded=True)
    lifecycle = PolicyRuntimeStateMachine(health)
    lifecycle.configure()
    lifecycle.activate()
    scheduler = ShadowCommandScheduler(
        lifecycle=lifecycle,
        context=EpisodeContext('trace_m1', 'episode_m1'),
        command_ttl_ns=100_000_000,
        max_observation_age_ns=500_000_000,
    )
    return scheduler, health


def test_m1_contract_reference_matches_canonical_lock_when_present() -> None:
    if not MIDSTREAM_LOCK.is_file():
        pytest.skip('canonical midstream M0 checkout not present')
    lock = json.loads(MIDSTREAM_LOCK.read_text(encoding='utf-8'))
    descriptor_path = 'evaluation/examples/policy_runtime_contract_fixture.json'
    assert lock['runtime_contract_version'] == 'panda_policy_runtime_v1'
    assert lock['artifact_sha256'][descriptor_path] == CONTRACT_DESCRIPTOR_SHA256
    assert lock['content_sha256'] == CONTRACT_LOCK_CONTENT_SHA256


def test_message_definitions_are_registered_and_keep_task_claim_false() -> None:
    cmake = (ROOT / 'src/teleop_interfaces/CMakeLists.txt').read_text(
        encoding='utf-8'
    )
    for name in (
        'PolicyCommand.msg',
        'PolicyExecutionReport.msg',
        'TaskEvaluationStatus.msg',
    ):
        assert f'"msg/{name}"' in cmake
        text = (ROOT / 'src/teleop_interfaces/msg' / name).read_text(
            encoding='utf-8'
        )
        assert 'bool claims_task_success' in text
    task = (
        ROOT / 'src/teleop_interfaces/msg/TaskEvaluationStatus.msg'
    ).read_text(encoding='utf-8')
    assert 'bool risk_may_override' in task
    assert 'int8 OUTCOME_UNKNOWN=-1' in task


def test_action_chunk_rejects_schema_dimension_and_nonfinite_values() -> None:
    with pytest.raises(ValueError, match=r'action\[8\]'):
        ActionChunkEnvelope(
            observation_sequence=0,
            observation_captured_monotonic_ns=0,
            action_schema_version=ABSOLUTE_ACTION_SCHEMA,
            actions=((0.0,) * 7,),
            execute_k=1,
            inference_started_monotonic_ns=0,
            inference_finished_monotonic_ns=1,
            from_native_chunk=False,
        )
    with pytest.raises(ValueError, match='NaN'):
        ActionChunkEnvelope(
            observation_sequence=0,
            observation_captured_monotonic_ns=0,
            action_schema_version=ABSOLUTE_ACTION_SCHEMA,
            actions=((math.nan,) + (0.0,) * 7,),
            execute_k=1,
            inference_started_monotonic_ns=0,
            inference_finished_monotonic_ns=1,
            from_native_chunk=False,
        )


@pytest.mark.parametrize(
    ('error', 'expected'),
    [
        (ValueError('unknown action schema: x'), 'action_schema_unknown'),
        (ValueError('schema requires action[8]'), 'action_dimension_invalid'),
        (ValueError('action contains NaN or infinity'), 'action_non_finite'),
        (
            ValueError('SmolVLA observation state must be finite state[15]'),
            'observation_invalid',
        ),
        (RuntimeError('backend crashed'), 'lifecycle_error'),
    ],
)
def test_runtime_errors_map_to_frozen_reason_codes(error, expected) -> None:
    reason, lane = classify_runtime_error(error)
    assert reason == expected
    assert lane == (
        'system_fail' if expected == 'lifecycle_error' else 'interface_fail'
    )


def test_lifecycle_is_fail_closed_and_rejects_illegal_transitions() -> None:
    health = RuntimeHealth(policy_loaded=True)
    lifecycle = PolicyRuntimeStateMachine(health)
    assert lifecycle.state is LifecycleState.UNCONFIGURED
    assert lifecycle.command_enabled is False
    with pytest.raises(RuntimeError, match='illegal'):
        lifecycle.activate()
    lifecycle.configure()
    assert lifecycle.command_enabled is False
    lifecycle.activate()
    assert lifecycle.command_enabled is True
    assert health.validity == 'WARMING_UP'
    assert health.reason_code == 'observation_warming_up'
    lifecycle.deactivate()
    assert lifecycle.command_enabled is False
    lifecycle.cleanup()
    assert health.policy_loaded is False


def test_scheduler_emits_deterministic_sequence_and_consumes_chunk() -> None:
    scheduler, health = _active_scheduler()
    scheduler.load_chunk(_envelope())
    first = scheduler.next_command(1_100_000_000)
    second = scheduler.next_command(1_200_000_000)
    assert first is not None and second is not None
    assert first.command_sequence == 0
    assert second.command_sequence == 1
    assert first.chunk_index == 0
    assert second.chunk_index == 1
    assert first.from_prefetched_chunk is False
    assert second.from_prefetched_chunk is True
    assert first.action[-1] == pytest.approx(0.8)
    assert second.action[-1] == pytest.approx(0.7)
    assert first.event_id == 'command:episode_m1:0'
    assert first.parent_event_id == 'observation:episode_m1:7'
    assert first.claims_task_success is False
    assert health.queue_depth == 0
    assert scheduler.next_command(1_300_000_000) is None
    assert health.reason_code == 'queue_underrun'


def test_reset_clears_queue_and_sequence() -> None:
    scheduler, health = _active_scheduler()
    scheduler.load_chunk(_envelope())
    assert scheduler.next_command(1_100_000_000) is not None
    scheduler.reset(EpisodeContext('trace_reset', 'episode_reset'))
    assert health.queue_depth == 0
    assert health.reason_code == 'queue_reset'
    scheduler.load_chunk(_envelope(observation_sequence=0))
    command = scheduler.next_command(1_100_000_000)
    assert command is not None
    assert command.command_sequence == 0
    assert command.trace_run_id == 'trace_reset'


def test_stale_observation_never_generates_command() -> None:
    scheduler, health = _active_scheduler()
    scheduler.load_chunk(_envelope(captured_ns=1_000_000_000))
    assert scheduler.next_command(1_600_000_001) is None
    assert health.validity == 'STALE'
    assert health.reason_code == 'observation_stale'
    assert health.queue_depth == 0


def test_inactive_runtime_never_generates_command() -> None:
    health = RuntimeHealth(policy_loaded=True)
    lifecycle = PolicyRuntimeStateMachine(health)
    lifecycle.configure()
    scheduler = ShadowCommandScheduler(
        lifecycle=lifecycle,
        context=EpisodeContext('trace', 'episode'),
        command_ttl_ns=1,
        max_observation_age_ns=1_000,
    )
    scheduler.load_chunk(_envelope(captured_ns=0))
    assert scheduler.next_command(1) is None
    assert health.reason_code == 'lifecycle_inactive'


def test_dds_events_are_visible_in_health() -> None:
    scheduler, health = _active_scheduler()
    scheduler.record_deadline_miss()
    assert health.deadline_miss_count == 1
    assert health.reason_code == 'dds_deadline_missed'
    scheduler.record_liveliness_lost()
    assert health.reason_code == 'dds_liveliness_lost'
    assert health.validity == 'ERROR'


def test_dds_events_do_not_fault_inactive_runtime() -> None:
    health = RuntimeHealth(policy_loaded=True)
    lifecycle = PolicyRuntimeStateMachine(health)
    lifecycle.configure()
    scheduler = ShadowCommandScheduler(
        lifecycle=lifecycle,
        context=EpisodeContext('trace', 'episode'),
        command_ttl_ns=1,
        max_observation_age_ns=1,
    )
    scheduler.record_deadline_miss()
    scheduler.record_liveliness_lost()
    assert health.deadline_miss_count == 0
    assert health.validity == 'WARMING_UP'
    assert health.reason_code == 'observation_warming_up'


def test_ros_qos_and_shadow_message_mapping_match_m0() -> None:
    qos = policy_command_qos(expected_period_s=0.1, ttl_s=0.08)
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.VOLATILE
    assert qos.history is HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.deadline.nanoseconds == 150_000_000
    assert qos.lifespan.nanoseconds == 80_000_000
    assert qos.liveliness is LivelinessPolicy.MANUAL_BY_TOPIC
    assert qos.liveliness_lease_duration.nanoseconds == 200_000_000

    scheduler, _health = _active_scheduler()
    scheduler.load_chunk(_envelope())
    command = scheduler.next_command(1_100_000_000)
    message = SimpleNamespace(header=SimpleNamespace())
    stamp = SimpleNamespace(sec=1, nanosec=2)
    valid_until = SimpleNamespace(sec=1, nanosec=80_000_002)
    mapped = populate_policy_command(
        message,
        command,
        source_stamp=stamp,
        received_stamp=stamp,
        valid_until=valid_until,
    )
    assert mapped.contract_version == 'panda_policy_runtime_v1'
    assert mapped.validity == 'VALID'
    assert mapped.reason_code == 'none'
    assert mapped.action == list(command.action)
    assert mapped.claims_task_success is False


def test_health_diagnostic_has_explicit_unavailable_values() -> None:
    health = RuntimeHealth(policy_loaded=True)
    lifecycle = PolicyRuntimeStateMachine(health)
    lifecycle.configure()
    lifecycle.activate()
    stamp = SimpleNamespace(sec=1, nanosec=0)
    array = build_runtime_health_array(
        health,
        stamp=stamp,
        policy_name='smolvla_recovery_v3',
        policy_version='scene_v3_phaseaware50',
        checkpoint_hash='not_applicable',
        observation_schema_version='smolvla_panda_state15_scene_rgb_v3',
        trace_run_id='trace-health',
        episode_id='episode-health',
    )
    fields = {item.key: item.value for item in array.status[0].values}
    assert array.status[0].message == 'observation_warming_up'
    assert fields['inference_latency_ms_last'] == 'unavailable'
    assert fields['observation_age_ms'] == 'unavailable'
    assert fields['claims_task_success'] == 'false'
    assert fields['shadow_only'] == 'true'
    assert fields['trace_run_id'] == 'trace-health'
    assert fields['episode_id'] == 'episode-health'
    assert fields['inference_request_count'] == '0'
    assert fields['pending_observation_drop_count'] == '0'


def test_m4_hold_clears_queue_without_reusing_command_sequence() -> None:
    scheduler, _health = _active_scheduler()
    scheduler.load_chunk(_envelope(captured_ns=1_000_000_000))
    first = scheduler.next_command(1_000_000_100)
    assert first is not None and first.command_sequence == 0
    scheduler.clear_queue('risk_r2_hold')
    assert scheduler.next_command(1_000_000_200) is None
    scheduler.load_chunk(_envelope(
        observation_sequence=8, captured_ns=1_000_000_300
    ))
    resumed = scheduler.next_command(1_000_000_400)
    assert resumed is not None and resumed.command_sequence == 1


def test_generated_policy_command_round_trips_over_mock_ros(
    monkeypatch, tmp_path: Path
) -> None:
    rclpy = pytest.importorskip('rclpy')
    messages = pytest.importorskip('teleop_interfaces.msg')
    PolicyCommand = messages.PolicyCommand

    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    rclpy.init()
    publisher_node = rclpy.create_node('policy_runtime_m1_mock_publisher')
    subscriber_node = rclpy.create_node('policy_runtime_m1_mock_subscriber')
    received = []
    qos = policy_command_qos(expected_period_s=0.1, ttl_s=0.5)
    publisher = publisher_node.create_publisher(
        PolicyCommand, '/policy/command_m1_mock', qos
    )
    subscriber_node.create_subscription(
        PolicyCommand,
        '/policy/command_m1_mock',
        received.append,
        qos,
    )
    try:
        scheduler, _health = _active_scheduler()
        scheduler.load_chunk(_envelope(captured_ns=time.monotonic_ns()))
        command = scheduler.next_command(time.monotonic_ns())
        assert command is not None
        now = publisher_node.get_clock().now()
        message = populate_policy_command(
            PolicyCommand(),
            command,
            source_stamp=now.to_msg(),
            received_stamp=now.to_msg(),
            valid_until=now.to_msg(),
        )
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            publisher.publish(message)
            publisher.assert_liveliness()
            rclpy.spin_once(subscriber_node, timeout_sec=0.05)
        assert len(received) == 1
        assert received[0].command_sequence == 0
        assert received[0].action_schema_version == ABSOLUTE_ACTION_SCHEMA
        assert received[0].claims_task_success is False
    finally:
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()


class _FakeSmolRuntime:
    metadata = {
        'action_dim': 8,
        'chunk_size': 10,
        'deploy_n_action_steps': 5,
    }

    def __init__(self) -> None:
        self.reset_count = 0
        self.infer_count = 0
        self.predict_chunk_count = 0

    def reset(self) -> None:
        self.reset_count += 1

    def infer(self, state, image, *, task=None):
        del state, image, task
        self.infer_count += 1
        return [0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0, 0.7]

    def predict_chunk(self, state, image, *, task=None):
        del state, image, task
        self.predict_chunk_count += 1
        return [[0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0, 0.7]] * 10


def test_smolvla_backend_is_protocol_compatible_and_exports_native_chunk() -> None:
    runtime = _FakeSmolRuntime()
    backend = SmolVlaPolicyBackend(runtime)
    assert isinstance(backend, PolicyBackend)
    backend.load(
        PolicyArtifact(
            policy_name='smolvla_recovery_v3',
            policy_version='scene_v3_phaseaware50',
            checkpoint_hash='a' * 64,
            observation_schema_version='smolvla_panda_state15_scene_rgb_v3',
        )
    )
    backend.reset(EpisodeContext('trace', 'episode'))
    observation = backend.build_observation(
        RawObservation(
            observation_sequence=3,
            captured_monotonic_ns=123,
            state=[0.0] * 15,
            image=object(),
        )
    )
    chunk = backend.predict_chunk(observation)
    assert runtime.infer_count == 0
    assert runtime.predict_chunk_count == 1
    assert chunk.chunk_size == 10
    assert chunk.execute_k == 5
    assert chunk.from_native_chunk is True
    assert chunk.claims_task_success is False


def test_backend_rejects_invalid_state15() -> None:
    backend = SmolVlaPolicyBackend(_FakeSmolRuntime())
    with pytest.raises(ValueError, match=r'state\[15\]'):
        backend.build_observation(
            RawObservation(0, 0, [0.0] * 14, object())
        )


def test_remote_backend_maps_dual_camera_chunk_and_reset(monkeypatch) -> None:
    import json

    responses = [
        {"protocol_version": "smolvla_remote_inference_v1", "reset": True},
        {
            "protocol_version": "smolvla_remote_inference_v1",
            "observation_sequence": 4,
            "action_schema_version": ABSOLUTE_ACTION_SCHEMA,
            "execute_k": 5,
            "actions": [
                [0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0, 0.7]
            ] * 10,
        },
    ]
    requests = []

    class _Response:
        def __init__(self, payload):
            self._payload = payload

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            return json.dumps(self._payload).encode('utf-8')

    def fake_urlopen(request, timeout):
        del timeout
        requests.append(json.loads(request.data.decode('utf-8')))
        return _Response(responses.pop(0))

    monkeypatch.setattr(
        'isaac_sim_adapter.remote_policy_client.urlopen', fake_urlopen
    )
    backend = RemoteSmolVlaPolicyBackend('https://127.0.0.1:18080')
    backend.load(
        PolicyArtifact('smolvla_remote', 'v1', 'a' * 64, 'state15_scene_wrist')
    )
    backend.reset(EpisodeContext('trace', 'episode'))
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    observation = backend.build_observation(
        RawObservation(4, 123, [0.0] * 15, image, wrist_image=image.copy())
    )
    envelope = backend.predict_chunk(observation)
    assert envelope.chunk_size == 10
    assert envelope.execute_k == 5
    assert envelope.observation_sequence == 4
    assert requests[0]['protocol_version'] == 'smolvla_remote_inference_v1'
    assert requests[1]['state'] == [0.0] * 15
    assert requests[1]['image_encoding'] == 'jpeg'
    assert len(requests[1]['scene_jpeg_b64']) > 100
    assert len(requests[1]['wrist_jpeg_b64']) > 100


class _BlockingChunkBackend:
    def __init__(self) -> None:
        import threading

        self._health = RuntimeHealth(policy_loaded=True)
        self.entered = threading.Event()
        self.release = threading.Event()
        self.sequences: list[int] = []
        self.fail_sequence: int | None = None

    def load(self, artifact) -> None:
        del artifact

    def reset(self, context) -> None:
        del context

    def build_observation(self, raw):
        return raw

    def predict_chunk(self, observation):
        self.sequences.append(observation.observation_sequence)
        self.entered.set()
        assert self.release.wait(2.0)
        self.release.clear()
        if observation.observation_sequence == self.fail_sequence:
            raise RuntimeError('mock inference failed')
        now = time.monotonic_ns()
        return ActionChunkEnvelope(
            observation_sequence=observation.observation_sequence,
            observation_captured_monotonic_ns=(
                observation.captured_monotonic_ns
            ),
            action_schema_version=ABSOLUTE_ACTION_SCHEMA,
            actions=((0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0, 0.7),) * 10,
            execute_k=5,
            inference_started_monotonic_ns=now,
            inference_finished_monotonic_ns=now,
            from_native_chunk=True,
        )

    def health(self):
        return self._health

    def close(self) -> None:
        pass


def _model_observation(sequence: int):
    from isaac_sim_adapter.policy_runtime import ModelObservation

    return ModelObservation(
        observation_sequence=sequence,
        captured_monotonic_ns=time.monotonic_ns(),
        state=(0.0,) * 15,
        image=object(),
    )


def test_async_worker_submit_is_nonblocking_and_pending_is_latest_only() -> None:
    backend = _BlockingChunkBackend()
    worker = AsyncChunkInferenceWorker(backend)
    worker.start()
    try:
        started = time.monotonic()
        worker.submit(_model_observation(0))
        assert time.monotonic() - started < 0.05
        assert backend.entered.wait(1.0)

        worker.submit(_model_observation(1))
        worker.submit(_model_observation(2))
        assert backend.health().pending_observation_drop_count == 1

        backend.release.set()
        first = worker.wait_for_result(1.0)
        assert first is not None and first.observation_sequence == 0
        assert backend.entered.wait(1.0)
        backend.release.set()
        second = worker.wait_for_result(1.0)
        assert second is not None and second.observation_sequence == 2
        assert backend.sequences == [0, 2]
        assert backend.health().inference_request_count == 3
        assert backend.health().inference_completion_count == 2
        assert backend.health().inference_failure_count == 0
    finally:
        backend.release.set()
        assert worker.stop(1.0)


def test_async_worker_surfaces_inference_error_and_remains_bounded() -> None:
    backend = _BlockingChunkBackend()
    backend.fail_sequence = 4
    worker = AsyncChunkInferenceWorker(backend)
    worker.start()
    try:
        worker.submit(_model_observation(4))
        assert backend.entered.wait(1.0)
        backend.release.set()
        result = worker.wait_for_result(1.0)
        assert result is not None
        assert result.envelope is None
        assert isinstance(result.error, RuntimeError)
        assert backend.health().inference_failure_count == 1
        assert backend.health().inference_busy is False
    finally:
        backend.release.set()
        assert worker.stop(1.0)


def test_async_worker_marks_warmup_result_as_non_executable() -> None:
    backend = _BlockingChunkBackend()
    worker = AsyncChunkInferenceWorker(backend)
    worker.start()
    try:
        worker.submit_warmup(_model_observation(0))
        assert backend.entered.wait(1.0)
        backend.release.set()
        result = worker.wait_for_result(1.0)
        assert result is not None and result.envelope is not None
        assert result.is_warmup is True
        assert backend.health().inference_request_count == 0
        assert backend.health().warmup_started_count == 1
        assert backend.health().warmup_completion_count == 1
        assert backend.health().warmup_failure_count == 0
    finally:
        backend.release.set()
        assert worker.stop(1.0)


def test_async_worker_prefetches_replacement_while_scheduler_consumes() -> None:
    backend = _BlockingChunkBackend()
    worker = AsyncChunkInferenceWorker(backend)
    scheduler, health = _active_scheduler()
    worker.start()
    try:
        first_observation = _model_observation(10)
        worker.submit(first_observation)
        assert backend.entered.wait(1.0)
        backend.release.set()
        first = worker.wait_for_result(1.0)
        assert first is not None and first.envelope is not None
        scheduler.load_chunk(first.envelope)

        second_observation = _model_observation(11)
        worker.submit(second_observation)
        # The model is blocked on the next chunk, while control can consume
        # already-prefetched commands without waiting on model execution.
        commands = [
            scheduler.next_command(time.monotonic_ns()) for _ in range(3)
        ]
        assert all(command is not None for command in commands)
        assert [command.chunk_index for command in commands] == [0, 1, 2]
        assert health.queue_depth == 2

        backend.release.set()
        second = worker.wait_for_result(1.0)
        assert second is not None and second.envelope is not None
        scheduler.load_chunk(second.envelope)
        replacement = scheduler.next_command(time.monotonic_ns())
        assert replacement is not None
        assert replacement.observation_sequence == 11
        assert replacement.chunk_index == 0
        assert replacement.command_sequence == 3
    finally:
        backend.release.set()
        assert worker.stop(1.0)
