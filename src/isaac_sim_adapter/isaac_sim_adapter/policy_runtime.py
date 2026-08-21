"""
Model-independent M1 policy runtime primitives.

The module is deliberately ROS-free so lifecycle, chunk scheduling, validity,
and contract drift can be tested without Isaac, weights, or a running graph.
M1 only emits shadow telemetry; it does not own robot command topics.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import threading
import time
from typing import Any, Protocol, runtime_checkable, Sequence


CONTRACT_VERSION = 'panda_policy_runtime_v1'
CONTRACT_DESCRIPTOR_SHA256 = (
    'e78176b6d487b03e8602c1b58a437d88b9d7509af23dec499262bb679ada7447'
)
CONTRACT_LOCK_CONTENT_SHA256 = (
    '995be854a056860c382ae72c0437299b4be8d3dd95b03595374dc2beda824ea1'
)
ABSOLUTE_ACTION_SCHEMA = 'panda_absolute_eef_gripper_v0'
DELTA_ACTION_SCHEMA = 'panda_ee_delta_gripper_v0'
ACTION_DIMENSIONS = {
    ABSOLUTE_ACTION_SCHEMA: 8,
    DELTA_ACTION_SCHEMA: 7,
}


class LifecycleState(str, Enum):
    UNCONFIGURED = 'UNCONFIGURED'
    INACTIVE = 'INACTIVE'
    ACTIVE = 'ACTIVE'
    ERROR_PROCESSING = 'ERROR_PROCESSING'
    FINALIZED = 'FINALIZED'


@dataclass(frozen=True)
class PolicyArtifact:
    policy_name: str
    policy_version: str
    checkpoint_hash: str
    observation_schema_version: str


@dataclass(frozen=True)
class EpisodeContext:
    trace_run_id: str
    episode_id: str


@dataclass(frozen=True)
class RawObservation:
    observation_sequence: int
    captured_monotonic_ns: int
    state: Sequence[float]
    image: Any
    task: str | None = None
    wrist_image: Any = None


@dataclass(frozen=True)
class ModelObservation:
    observation_sequence: int
    captured_monotonic_ns: int
    state: tuple[float, ...]
    image: Any
    task: str | None = None
    wrist_image: Any = None


@dataclass(frozen=True)
class ActionChunkEnvelope:
    observation_sequence: int
    observation_captured_monotonic_ns: int
    action_schema_version: str
    actions: tuple[tuple[float, ...], ...]
    execute_k: int
    inference_started_monotonic_ns: int
    inference_finished_monotonic_ns: int
    from_native_chunk: bool
    claims_task_success: bool = False

    def __post_init__(self) -> None:
        if self.claims_task_success:
            raise ValueError('policy runtime cannot claim task success')
        if self.observation_sequence < 0:
            raise ValueError('observation_sequence must be non-negative')
        if self.action_schema_version not in ACTION_DIMENSIONS:
            raise ValueError(f'unknown action schema: {self.action_schema_version}')
        if not self.actions:
            raise ValueError('action chunk must not be empty')
        if not 1 <= self.execute_k <= len(self.actions):
            raise ValueError('execute_k must be in [1, chunk_size]')
        expected_dim = ACTION_DIMENSIONS[self.action_schema_version]
        for action in self.actions:
            if len(action) != expected_dim:
                raise ValueError(
                    f'{self.action_schema_version} requires action[{expected_dim}]'
                )
            if not all(math.isfinite(float(value)) for value in action):
                raise ValueError('action contains NaN or infinity')
        if self.inference_finished_monotonic_ns < self.inference_started_monotonic_ns:
            raise ValueError('inference finish precedes start')

    @property
    def chunk_size(self) -> int:
        return len(self.actions)

    @property
    def inference_latency_ms(self) -> float:
        return (
            self.inference_finished_monotonic_ns
            - self.inference_started_monotonic_ns
        ) / 1_000_000.0


@dataclass(frozen=True)
class ShadowPolicyCommand:
    event_id: str
    parent_event_id: str
    trace_run_id: str
    episode_id: str
    observation_sequence: int
    command_sequence: int
    action_schema_version: str
    action: tuple[float, ...]
    chunk_index: int
    chunk_size: int
    from_prefetched_chunk: bool
    inference_latency_ms: float
    valid_until_monotonic_ns: int
    contract_version: str = CONTRACT_VERSION
    claims_task_success: bool = False


@dataclass
class RuntimeHealth:
    lifecycle_state: LifecycleState = LifecycleState.UNCONFIGURED
    validity: str = 'WARMING_UP'
    reason_code: str = 'policy_not_loaded'
    policy_loaded: bool = False
    observation_age_ms: float | None = None
    inference_busy: bool = False
    inference_latency_ms_last: float | None = None
    inference_latency_ms_p50: float | None = None
    inference_latency_ms_p95: float | None = None
    inference_request_count: int = 0
    inference_started_count: int = 0
    inference_completion_count: int = 0
    inference_failure_count: int = 0
    pending_observation_drop_count: int = 0
    completed_result_drop_count: int = 0
    warmup_started_count: int = 0
    warmup_completion_count: int = 0
    warmup_failure_count: int = 0
    queue_depth: int = 0
    queue_underrun_count: int = 0
    deadline_miss_count: int = 0
    last_command_sequence: int | None = None
    last_successful_command_age_ms: float | None = None
    hold_active: bool = False
    estop_active: bool = False
    contract_sha256: str = CONTRACT_DESCRIPTOR_SHA256
    failure_lane: str = 'none'
    claims_task_success: bool = False
    _latencies_ms: list[float] = field(default_factory=list, repr=False)
    _last_command_monotonic_ns: int | None = field(default=None, repr=False)

    def record_latency(self, latency_ms: float) -> None:
        if not math.isfinite(latency_ms) or latency_ms < 0.0:
            raise ValueError('latency must be finite and non-negative')
        self.inference_latency_ms_last = latency_ms
        self._latencies_ms.append(latency_ms)
        self._latencies_ms = self._latencies_ms[-256:]
        ordered = sorted(self._latencies_ms)
        self.inference_latency_ms_p50 = _percentile(ordered, 0.50)
        self.inference_latency_ms_p95 = _percentile(ordered, 0.95)

    def record_command(self, sequence: int, now_monotonic_ns: int) -> None:
        self.last_command_sequence = sequence
        self._last_command_monotonic_ns = now_monotonic_ns
        self.last_successful_command_age_ms = 0.0

    def refresh_ages(self, now_monotonic_ns: int) -> None:
        if self._last_command_monotonic_ns is not None:
            self.last_successful_command_age_ms = max(
                0.0,
                (now_monotonic_ns - self._last_command_monotonic_ns) / 1_000_000.0,
            )


def _percentile(ordered: Sequence[float], quantile: float) -> float | None:
    if not ordered:
        return None
    index = max(0, math.ceil(quantile * len(ordered)) - 1)
    return float(ordered[index])


def classify_runtime_error(error: Exception) -> tuple[str, str]:
    """Map validation failures to the frozen M0 reason/failure lane."""
    details = str(error).lower()
    if 'unknown action schema' in details:
        return 'action_schema_unknown', 'interface_fail'
    if 'requires action[' in details or 'expected action[' in details:
        return 'action_dimension_invalid', 'interface_fail'
    if 'action contains nan' in details or 'action contains' in details and (
        'infinity' in details or 'non-finite' in details
    ):
        return 'action_non_finite', 'interface_fail'
    if 'observation' in details or 'state[15]' in details:
        return 'observation_invalid', 'interface_fail'
    return 'lifecycle_error', 'system_fail'


class PolicyRuntimeStateMachine:
    """Frozen M0 lifecycle with fail-closed command enablement."""

    def __init__(self, health: RuntimeHealth | None = None) -> None:
        self.health = health or RuntimeHealth()

    @property
    def state(self) -> LifecycleState:
        return self.health.lifecycle_state

    @property
    def command_enabled(self) -> bool:
        return self.state is LifecycleState.ACTIVE

    def configure(self) -> None:
        self._transition(LifecycleState.UNCONFIGURED, LifecycleState.INACTIVE)
        self.health.validity = 'WARMING_UP'
        self.health.reason_code = 'observation_warming_up'

    def activate(self) -> None:
        self._transition(LifecycleState.INACTIVE, LifecycleState.ACTIVE)
        if not self.health.policy_loaded:
            self.error('activate requested before policy load')
            raise RuntimeError('policy must be loaded before activation')
        # ACTIVE permits the scheduler to emit only after a validated
        # observation/chunk arrives; activation alone is not evidence of data.
        self.health.validity = 'WARMING_UP'
        self.health.reason_code = 'observation_warming_up'

    def deactivate(self) -> None:
        self._transition(LifecycleState.ACTIVE, LifecycleState.INACTIVE)
        self.health.validity = 'UNAVAILABLE'
        self.health.reason_code = 'lifecycle_inactive'

    def cleanup(self) -> None:
        self._transition(LifecycleState.INACTIVE, LifecycleState.UNCONFIGURED)
        self.health.policy_loaded = False
        self.health.validity = 'UNAVAILABLE'
        self.health.reason_code = 'policy_not_loaded'

    def error(self, _details: str) -> None:
        if self.state is LifecycleState.FINALIZED:
            raise RuntimeError('finalized lifecycle cannot enter error processing')
        self.health.lifecycle_state = LifecycleState.ERROR_PROCESSING
        self.health.validity = 'ERROR'
        self.health.reason_code = 'lifecycle_error'
        self.health.failure_lane = 'system_fail'

    def recover(self) -> None:
        self._transition(
            LifecycleState.ERROR_PROCESSING, LifecycleState.UNCONFIGURED
        )
        self.health.policy_loaded = False
        self.health.validity = 'UNAVAILABLE'
        self.health.reason_code = 'policy_not_loaded'
        self.health.failure_lane = 'none'

    def shutdown(self) -> None:
        if self.state is LifecycleState.FINALIZED:
            return
        self.health.lifecycle_state = LifecycleState.FINALIZED
        self.health.validity = 'UNAVAILABLE'
        self.health.reason_code = 'lifecycle_inactive'

    def _transition(
        self, expected: LifecycleState, target: LifecycleState
    ) -> None:
        if self.state is not expected:
            raise RuntimeError(
                f'illegal lifecycle transition: {self.state.value} -> {target.value}'
            )
        self.health.lifecycle_state = target


class ShadowCommandScheduler:
    """Deterministically converts a validated chunk into shadow commands."""

    def __init__(
        self,
        *,
        lifecycle: PolicyRuntimeStateMachine,
        context: EpisodeContext,
        command_ttl_ns: int,
        max_observation_age_ns: int,
    ) -> None:
        if command_ttl_ns <= 0 or max_observation_age_ns <= 0:
            raise ValueError('TTL and observation age limits must be positive')
        self.lifecycle = lifecycle
        self.context = context
        self.command_ttl_ns = command_ttl_ns
        self.max_observation_age_ns = max_observation_age_ns
        self._chunk: ActionChunkEnvelope | None = None
        self._pending_chunk: ActionChunkEnvelope | None = None
        self._chunk_index = 0
        self._command_sequence = 0
        self._last_observation_sequence: int | None = None
        self._lock = threading.RLock()

    def reset(self, context: EpisodeContext | None = None) -> None:
        with self._lock:
            self._chunk = None
            self._pending_chunk = None
            self._chunk_index = 0
            self._command_sequence = 0
            self._last_observation_sequence = None
            if context is not None:
                self.context = context
            self.lifecycle.health.queue_depth = 0
            self.lifecycle.health.validity = 'WARMING_UP'
            self.lifecycle.health.reason_code = 'queue_reset'

    def clear_queue(self, reason_code: str = 'queue_cleared') -> None:
        """Drop pending actions without resetting monotonic command identity."""
        with self._lock:
            self._chunk = None
            self._pending_chunk = None
            self._chunk_index = 0
            self.lifecycle.health.queue_depth = 0
            self.lifecycle.health.validity = 'WARMING_UP'
            self.lifecycle.health.reason_code = str(reason_code)

    def load_chunk(self, envelope: ActionChunkEnvelope) -> None:
        with self._lock:
            previous = self._last_observation_sequence
            if previous is not None and envelope.observation_sequence < previous:
                raise ValueError('observation_sequence regressed')
            self._last_observation_sequence = envelope.observation_sequence
            if self._chunk is None or self._chunk_index >= self._chunk.execute_k:
                self._chunk = envelope
                self._pending_chunk = None
                self._chunk_index = 0
                self.lifecycle.health.queue_depth = envelope.execute_k
            else:
                # Never truncate an active K-step chunk.  Async inference may
                # replace only the latest pending result; it becomes active at
                # the next chunk boundary.
                self._pending_chunk = envelope
            self.lifecycle.health.record_latency(envelope.inference_latency_ms)
            self.lifecycle.health.validity = 'VALID'
            self.lifecycle.health.reason_code = 'none'

    def next_command(self, now_monotonic_ns: int | None = None) -> ShadowPolicyCommand | None:
        with self._lock:
            now_ns = (
                time.monotonic_ns()
                if now_monotonic_ns is None
                else now_monotonic_ns
            )
            health = self.lifecycle.health
            health.refresh_ages(now_ns)
            if not self.lifecycle.command_enabled:
                health.validity = 'UNAVAILABLE'
                health.reason_code = 'lifecycle_inactive'
                return None
            if self._chunk is None or self._chunk_index >= self._chunk.execute_k:
                if self._pending_chunk is not None:
                    self._chunk = self._pending_chunk
                    self._pending_chunk = None
                    self._chunk_index = 0
                    health.queue_depth = self._chunk.execute_k
                else:
                    health.queue_depth = 0
                    health.queue_underrun_count += 1
                    health.validity = 'WARMING_UP'
                    health.reason_code = 'queue_underrun'
                    return None
            observation_age = (
                now_ns - self._chunk.observation_captured_monotonic_ns
            )
            health.observation_age_ms = max(
                0.0, observation_age / 1_000_000.0
            )
            if observation_age > self.max_observation_age_ns:
                self._chunk = None
                self._chunk_index = 0
                health.queue_depth = 0
                health.validity = 'STALE'
                health.reason_code = 'observation_stale'
                return None

            chunk = self._chunk
            index = self._chunk_index
            sequence = self._command_sequence
            command = ShadowPolicyCommand(
                event_id=f'command:{self.context.episode_id}:{sequence}',
                parent_event_id=(
                    f'observation:{self.context.episode_id}:'
                    f'{chunk.observation_sequence}'
                ),
                trace_run_id=self.context.trace_run_id,
                episode_id=self.context.episode_id,
                observation_sequence=chunk.observation_sequence,
                command_sequence=sequence,
                action_schema_version=chunk.action_schema_version,
                action=chunk.actions[index],
                chunk_index=index,
                chunk_size=chunk.chunk_size,
                from_prefetched_chunk=index > 0,
                inference_latency_ms=chunk.inference_latency_ms,
                valid_until_monotonic_ns=now_ns + self.command_ttl_ns,
            )
            self._chunk_index += 1
            self._command_sequence += 1
            health.queue_depth = chunk.execute_k - self._chunk_index
            health.validity = 'VALID'
            health.reason_code = 'none'
            health.record_command(sequence, now_ns)
            return command

    def record_deadline_miss(self) -> None:
        with self._lock:
            if not self.lifecycle.command_enabled:
                return
            self.lifecycle.health.deadline_miss_count += 1
            self.lifecycle.health.validity = 'ERROR'
            self.lifecycle.health.reason_code = 'dds_deadline_missed'

    def record_liveliness_lost(self) -> None:
        with self._lock:
            if not self.lifecycle.command_enabled:
                return
            self.lifecycle.health.validity = 'ERROR'
            self.lifecycle.health.reason_code = 'dds_liveliness_lost'


@runtime_checkable
class PolicyBackend(Protocol):
    def load(self, artifact: PolicyArtifact) -> None: ...
    def reset(self, context: EpisodeContext) -> None: ...
    def build_observation(self, raw: RawObservation) -> ModelObservation: ...
    def predict_chunk(self, observation: ModelObservation) -> ActionChunkEnvelope: ...
    def health(self) -> RuntimeHealth: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class AsyncInferenceResult:
    """One completed background prediction, successful or failed."""

    observation_sequence: int
    envelope: ActionChunkEnvelope | None = None
    error: Exception | None = None
    is_warmup: bool = False

    def __post_init__(self) -> None:
        if (self.envelope is None) == (self.error is None):
            raise ValueError('async result requires exactly one of envelope or error')


class AsyncChunkInferenceWorker:
    """Single-model worker with latest-only pending observation/result slots.

    ``submit`` never waits for model execution. While one prediction is in
    flight, a newer observation replaces the pending observation rather than
    building an unbounded stale backlog. Completed results are also latest-only
    so the ROS/control side can poll without sharing model state.

    The worker does not own the backend lifecycle. Callers must load/reset the
    backend before ``start`` and close it after ``stop`` when appropriate.
    """

    def __init__(self, backend: PolicyBackend, *, thread_name: str = 'policy-inference') -> None:
        self._backend = backend
        self._condition = threading.Condition()
        self._pending: tuple[ModelObservation, bool] | None = None
        self._result: AsyncInferenceResult | None = None
        self._stopping = False
        self._thread = threading.Thread(
            target=self._run,
            name=str(thread_name),
            daemon=True,
        )
        self._started = False

    @property
    def is_alive(self) -> bool:
        return self._thread.is_alive()

    def start(self) -> None:
        with self._condition:
            if self._started:
                raise RuntimeError('async inference worker already started')
            self._started = True
            self._thread.start()

    def submit(self, observation: ModelObservation) -> None:
        """Publish the newest observation without blocking on inference."""
        self._submit(observation, is_warmup=False)

    def submit_warmup(self, observation: ModelObservation) -> None:
        """Schedule one non-executable model warmup prediction."""
        self._submit(observation, is_warmup=True)

    def _submit(self, observation: ModelObservation, *, is_warmup: bool) -> None:
        with self._condition:
            if not self._started:
                raise RuntimeError('async inference worker is not started')
            if self._stopping:
                raise RuntimeError('async inference worker is stopping')
            health = self._backend.health()
            if not is_warmup:
                health.inference_request_count += 1
            if self._pending is not None:
                health.pending_observation_drop_count += 1
            self._pending = (observation, is_warmup)
            self._condition.notify_all()

    def poll_result(self) -> AsyncInferenceResult | None:
        """Return and clear the latest completed result without waiting."""
        with self._condition:
            result = self._result
            self._result = None
            return result

    def wait_for_result(self, timeout_s: float) -> AsyncInferenceResult | None:
        """Bounded test/teardown helper; live control should use ``poll_result``."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while self._result is None and not self._stopping:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            result = self._result
            self._result = None
            return result

    def stop(self, timeout_s: float = 2.0) -> bool:
        """Request bounded shutdown; return whether the model thread exited."""
        with self._condition:
            if not self._started:
                return True
            self._stopping = True
            self._pending = None
            self._condition.notify_all()
        self._thread.join(max(0.0, float(timeout_s)))
        return not self._thread.is_alive()

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                observation, is_warmup = self._pending
                self._pending = None
                health = self._backend.health()
                health.inference_busy = True
                if is_warmup:
                    health.warmup_started_count += 1
                else:
                    health.inference_started_count += 1
            try:
                envelope = self._backend.predict_chunk(observation)
                result = AsyncInferenceResult(
                    observation_sequence=observation.observation_sequence,
                    envelope=envelope,
                    is_warmup=is_warmup,
                )
                if is_warmup:
                    health.warmup_completion_count += 1
                else:
                    health.inference_completion_count += 1
            except Exception as error:  # surfaced to the fail-closed owner
                result = AsyncInferenceResult(
                    observation_sequence=observation.observation_sequence,
                    error=error,
                    is_warmup=is_warmup,
                )
                if is_warmup:
                    health.warmup_failure_count += 1
                else:
                    health.inference_failure_count += 1
            finally:
                health.inference_busy = False
            with self._condition:
                if self._result is not None:
                    health.completed_result_drop_count += 1
                self._result = result
                self._condition.notify_all()


class SmolVlaPolicyBackend:
    """
    Compatibility backend over the existing SceneSmolVLARuntime.

    LeRobot ``select_action`` owns its internal queue and exposes one action per
    call.  M1 therefore emits an honest singleton envelope.  It must not call
    the model repeatedly to manufacture an apparent native chunk.
    """

    def __init__(self, runtime: Any, health: RuntimeHealth | None = None) -> None:
        self._runtime = runtime
        self._health = health or RuntimeHealth()
        self._artifact: PolicyArtifact | None = None
        self._context: EpisodeContext | None = None

    def load(self, artifact: PolicyArtifact) -> None:
        metadata = self._runtime.metadata
        if int(metadata.get('action_dim', -1)) != 8:
            raise ValueError('SmolVLA backend requires native action[8]')
        self._artifact = artifact
        self._health.policy_loaded = True
        self._health.validity = 'WARMING_UP'
        self._health.reason_code = 'observation_warming_up'

    def reset(self, context: EpisodeContext) -> None:
        if self._artifact is None:
            raise RuntimeError('load must precede reset')
        self._runtime.reset()
        self._context = context

    def build_observation(self, raw: RawObservation) -> ModelObservation:
        state = tuple(float(value) for value in raw.state)
        if len(state) != 15 or not all(math.isfinite(value) for value in state):
            raise ValueError('SmolVLA observation state must be finite state[15]')
        return ModelObservation(
            observation_sequence=raw.observation_sequence,
            captured_monotonic_ns=raw.captured_monotonic_ns,
            state=state,
            image=raw.image,
            task=raw.task,
            wrist_image=raw.wrist_image,
        )

    def predict_chunk(self, observation: ModelObservation) -> ActionChunkEnvelope:
        if self._artifact is None or self._context is None:
            raise RuntimeError('backend must be loaded and reset before prediction')
        started = time.monotonic_ns()
        if observation.wrist_image is None:
            # Keep the model-independent M1 fake/runtime compatibility path
            # alive for legacy single-camera unit tests. The live S4 node
            # supplies both image fields and takes the dual-camera path.
            actions = self._runtime.predict_chunk(
                observation.state,
                observation.image,
                task=observation.task,
            )
        else:
            actions = self._runtime.predict_chunk(
                observation.state,
                observation.image,
                observation.wrist_image,
                task=observation.task,
            )
        finished = time.monotonic_ns()
        execute_k = int(self._runtime.metadata['deploy_n_action_steps'])
        return ActionChunkEnvelope(
            observation_sequence=observation.observation_sequence,
            observation_captured_monotonic_ns=observation.captured_monotonic_ns,
            action_schema_version=ABSOLUTE_ACTION_SCHEMA,
            actions=tuple(
                tuple(float(value) for value in action) for action in actions
            ),
            execute_k=execute_k,
            inference_started_monotonic_ns=started,
            inference_finished_monotonic_ns=finished,
            from_native_chunk=True,
        )

    def health(self) -> RuntimeHealth:
        return self._health

    def close(self) -> None:
        close = getattr(self._runtime, 'close', None)
        if callable(close):
            close()
