"""Bounded online SmolVLA abs-EEF inference for Isaac S4 (Recovery v3).

Publishes absolute PoseStamped + gripper commands. Does not claim task success.
Physical lift/place remain owned by ContinuousTaskEvaluator / suite GT.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from isaac_sim_adapter.effort_control import PANDA_ARM_JOINTS
from isaac_sim_adapter.policy_control import absolute_action_to_target_pose
from isaac_sim_adapter.policy_control import bound_absolute_eef_gripper
from isaac_sim_adapter.policy_control import validate_panda_joint_positions
from isaac_sim_adapter.policy_execution_adapter import ExecutionState
from isaac_sim_adapter.policy_execution_adapter import legacy_absolute_result
from isaac_sim_adapter.policy_execution_adapter import PandaPolicyExecutionAdapter
from isaac_sim_adapter.policy_execution_adapter import resolve_execution_adapter_mode
from isaac_sim_adapter.policy_execution_adapter import validate_authoritative_publisher_counts
from isaac_sim_adapter.policy_inference_node import image_message_to_rgb
from isaac_sim_adapter.policy_runtime import classify_runtime_error
from isaac_sim_adapter.policy_runtime import EpisodeContext
from isaac_sim_adapter.policy_runtime import PolicyArtifact
from isaac_sim_adapter.policy_runtime import PolicyRuntimeStateMachine
from isaac_sim_adapter.policy_runtime import RawObservation
from isaac_sim_adapter.policy_runtime import RuntimeHealth
from isaac_sim_adapter.policy_runtime import ShadowCommandScheduler
from isaac_sim_adapter.policy_runtime import SmolVlaPolicyBackend
from isaac_sim_adapter.policy_runtime_ros import build_runtime_health_array
from isaac_sim_adapter.policy_runtime_ros import policy_command_qos
from isaac_sim_adapter.policy_runtime_ros import populate_execution_report
from isaac_sim_adapter.policy_runtime_ros import populate_policy_command
from isaac_sim_adapter.policy_runtime_ros import runtime_health_qos
from isaac_sim_adapter.s4_runtime_contract import assert_runtime_matches_contract
from isaac_sim_adapter.s4_runtime_contract import load_s4_runtime_contract
from isaac_sim_adapter.scene_smolvla_runtime import compose_state15
from isaac_sim_adapter.scene_smolvla_runtime import SceneSmolVLARuntime
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64, Header, String
from teleop_interfaces.msg import SafetyStatus
from teleop_interfaces.srv import TriggerEstop


class IsaacSmolVLAPolicyInferenceNode(Node):
    """Infer SmolVLA absolute actions and publish bounded poses at control rate."""

    def __init__(self) -> None:
        super().__init__('isaac_smolvla_policy_inference')
        self._s4_contract = load_s4_runtime_contract()
        self.declare_parameter('lora_dir', '')
        self.declare_parameter('base_dir', '')
        self.declare_parameter('vlm_dir', '')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('max_actions', 150)
        self.declare_parameter('n_action_steps', int(self._s4_contract.n_action_steps))
        self.declare_parameter(
            'inference_rate_hz', float(self._s4_contract.control_rate_hz)
        )
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('observation_timeout_s', 0.5)
        self.declare_parameter('startup_timeout_s', 90.0)
        self.declare_parameter('post_action_hold_s', 2.0)
        self.declare_parameter('max_joint_excursion_rad', 3.0)
        self.declare_parameter('max_ee_excursion_m', 0.55)
        self.declare_parameter(
            'workspace_min', list(self._s4_contract.workspace_min)
        )
        self.declare_parameter(
            'workspace_max', list(self._s4_contract.workspace_max)
        )
        self.declare_parameter(
            'task', 'pick up the red box and place it in the left bin'
        )
        self.declare_parameter(
            'output_path', '/tmp/isaac_smolvla_s4_policy_report.json'
        )
        # Attribution telemetry (H1/H3): dump policy-input RGB + state15 sidecar.
        # Empty string disables dumping. Does not claim task success.
        self.declare_parameter('telemetry_dir', '')
        self.declare_parameter('camera_dump_stride', 1)
        self.declare_parameter('policy_runtime_shadow_enabled', False)
        self.declare_parameter('execution_adapter_mode', 'legacy')
        self.declare_parameter('authoritative_require_single_publisher', True)
        self.declare_parameter('policy_runtime_trace_run_id', 'runtime_shadow_m1')
        self.declare_parameter('policy_runtime_episode_id', 'episode_unassigned')
        self.declare_parameter('policy_runtime_command_ttl_s', 0.1)
        self.declare_parameter(
            'policy_runtime_policy_version', 'scene_v3_phaseaware50'
        )
        self.declare_parameter('policy_runtime_checkpoint_hash', 'not_applicable')
        self.declare_parameter(
            'policy_runtime_observation_schema_version',
            'smolvla_panda_state15_scene_rgb_v3',
        )

        lora_dir = Path(str(self.get_parameter('lora_dir').value)).expanduser()
        base_dir = Path(str(self.get_parameter('base_dir').value)).expanduser()
        vlm_dir = Path(str(self.get_parameter('vlm_dir').value)).expanduser()
        if not lora_dir.is_dir():
            raise ValueError(f'lora_dir does not exist: {lora_dir}')
        if not base_dir.is_dir():
            raise ValueError(f'base_dir does not exist: {base_dir}')
        if not vlm_dir.is_dir():
            raise ValueError(f'vlm_dir does not exist: {vlm_dir}')
        telemetry_raw = str(self.get_parameter('telemetry_dir').value).strip()
        self._telemetry_dir = (
            Path(telemetry_raw).expanduser() if telemetry_raw else None
        )
        self._camera_dump_stride = max(
            1, int(self.get_parameter('camera_dump_stride').value)
        )
        self._camera_dump_count = 0
        if self._telemetry_dir is not None:
            (self._telemetry_dir / 'camera').mkdir(parents=True, exist_ok=True)
            self._observations_jsonl = self._telemetry_dir / 'observations.jsonl'
            self._observations_jsonl.write_text('', encoding='utf-8')
            self.get_logger().info(
                f'telemetry enabled: dir={self._telemetry_dir} '
                f'camera_stride={self._camera_dump_stride}'
            )
        else:
            self._observations_jsonl = None

        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._max_actions = int(self.get_parameter('max_actions').value)
        if self._max_actions <= 0 or self._max_actions > 500:
            raise ValueError('max_actions must be in [1, 500]')
        n_action_steps = int(self.get_parameter('n_action_steps').value)
        inference_rate = float(self.get_parameter('inference_rate_hz').value)
        command_rate = float(self.get_parameter('command_rate_hz').value)
        if inference_rate <= 0.0 or command_rate <= 0.0:
            raise ValueError('inference and command rates must be positive')
        self._inference_period = 1.0 / inference_rate
        self._observation_timeout = float(
            self.get_parameter('observation_timeout_s').value
        )
        self._startup_timeout = float(self.get_parameter('startup_timeout_s').value)
        self._post_action_hold = float(
            self.get_parameter('post_action_hold_s').value
        )
        self._max_joint_excursion = float(
            self.get_parameter('max_joint_excursion_rad').value
        )
        self._max_ee_excursion = float(
            self.get_parameter('max_ee_excursion_m').value
        )
        if self._max_joint_excursion <= 0.0 or self._max_ee_excursion <= 0.0:
            raise ValueError('execution excursion limits must be positive')
        self._workspace_min = tuple(
            float(value) for value in self.get_parameter('workspace_min').value
        )
        self._workspace_max = tuple(
            float(value) for value in self.get_parameter('workspace_max').value
        )
        self._output_path = Path(str(self.get_parameter('output_path').value))
        task = str(self.get_parameter('task').value)

        requested_device = str(self.get_parameter('device').value)
        self.get_logger().info(
            f'Loading SmolVLA Recovery checkpoint on {requested_device}'
        )
        self._runtime = SceneSmolVLARuntime(
            base_dir=base_dir,
            lora_dir=lora_dir,
            vlm_dir=vlm_dir,
            device=requested_device,
            n_action_steps=n_action_steps,
            task=task,
        )
        assert_runtime_matches_contract(
            chunk_size=int(self._runtime.metadata['chunk_size']),
            n_action_steps=int(self._runtime.metadata['deploy_n_action_steps']),
            state_dim=int(self._runtime.metadata['state_dim']),
            action_dim=int(self._runtime.metadata['action_dim']),
            policy_action_semantics=str(
                self._runtime.metadata['policy_action_semantics']
            ),
            contract=self._s4_contract,
        )
        if tuple(self._workspace_min) != self._s4_contract.workspace_min:
            raise ValueError(
                'workspace_min does not match S4 contract '
                f'{self._s4_contract.workspace_min}'
            )
        if tuple(self._workspace_max) != self._s4_contract.workspace_max:
            raise ValueError(
                'workspace_max does not match S4 contract '
                f'{self._s4_contract.workspace_max}'
            )
        if abs(inference_rate - self._s4_contract.control_rate_hz) > 1e-9:
            raise ValueError(
                'inference_rate_hz must match S4 contract control_rate_hz='
                f'{self._s4_contract.control_rate_hz}'
            )
        self.get_logger().info(
            'SmolVLA deploy n_action_steps='
            f'{self._runtime.metadata.get("deploy_n_action_steps")} '
            f'(chunk_size={self._runtime.metadata.get("chunk_size")}; '
            f'contract={self._s4_contract.contract_version})'
        )
        self._execution_adapter_mode = resolve_execution_adapter_mode(
            str(self.get_parameter('execution_adapter_mode').value),
            shadow_enabled=bool(
                self.get_parameter('policy_runtime_shadow_enabled').value
            ),
            dry_run=self._dry_run,
        )
        self._policy_runtime_shadow_enabled = (
            self._execution_adapter_mode != 'legacy'
        )
        self._authoritative_require_single_publisher = bool(
            self.get_parameter('authoritative_require_single_publisher').value
        )
        self._authoritative_publisher_identity_checked = False
        self._shadow_artifact = PolicyArtifact(
            policy_name='smolvla_recovery_v3',
            policy_version=str(
                self.get_parameter('policy_runtime_policy_version').value
            ),
            checkpoint_hash=str(
                self.get_parameter('policy_runtime_checkpoint_hash').value
            ),
            observation_schema_version=str(
                self.get_parameter(
                    'policy_runtime_observation_schema_version'
                ).value
            ),
        )
        self._shadow_context = EpisodeContext(
            trace_run_id=str(
                self.get_parameter('policy_runtime_trace_run_id').value
            ),
            episode_id=str(
                self.get_parameter('policy_runtime_episode_id').value
            ),
        )
        self._shadow_backend = None
        self._shadow_lifecycle = None
        self._shadow_scheduler = None
        self._shadow_execution_adapter = None
        self._shadow_observation_sequence = 0
        self._shadow_last_chunk_started = 0.0
        if self._policy_runtime_shadow_enabled:
            health = RuntimeHealth()
            self._shadow_lifecycle = PolicyRuntimeStateMachine(health)
            self._shadow_lifecycle.configure()
            self._shadow_backend = SmolVlaPolicyBackend(self._runtime, health)
            self._shadow_backend.load(self._shadow_artifact)
            self._shadow_backend.reset(self._shadow_context)
            ttl_s = float(
                self.get_parameter('policy_runtime_command_ttl_s').value
            )
            if ttl_s <= 0.0:
                raise ValueError('policy_runtime_command_ttl_s must be positive')
            self._shadow_scheduler = ShadowCommandScheduler(
                lifecycle=self._shadow_lifecycle,
                context=self._shadow_context,
                command_ttl_ns=int(ttl_s * 1_000_000_000),
                max_observation_age_ns=int(
                    self._observation_timeout * 1_000_000_000
                ),
            )
            self._shadow_execution_adapter = PandaPolicyExecutionAdapter(
                workspace_min=self._workspace_min,
                workspace_max=self._workspace_max,
                execution_mode=self._execution_adapter_mode,
            )
        else:
            self._runtime.reset()

        self._lock = threading.Lock()
        self._observations: dict[str, tuple[float, object]] = {}
        self._target: tuple[tuple[float, ...], tuple[float, ...], float] | None = None
        self._started = time.monotonic()
        self._finished_at: float | None = None
        self._inference_busy = False
        self._last_inference_started = 0.0
        self._shutdown_requested = False
        self._actions: list[dict[str, object]] = []
        self._error: str | None = None
        self._safety_estop: bool | None = None
        self._safety_ok: bool | None = None
        self._execution_reference_joints: tuple[float, ...] | None = None
        self._execution_reference_ee: tuple[float, ...] | None = None
        self._max_observed_joint_excursion = 0.0
        self._max_observed_ee_excursion = 0.0
        self._execution_guard_reason: str | None = None
        self._report_status: str | None = None
        self._groups = {
            'sensor': ReentrantCallbackGroup(),
            'inference': MutuallyExclusiveCallbackGroup(),
            'command': MutuallyExclusiveCallbackGroup(),
        }

        self._pose_pub = self.create_publisher(PoseStamped, '/teleop/cmd_pose', 10)
        self._heartbeat_pub = self.create_publisher(Header, '/teleop/heartbeat', 10)
        self._gripper_pub = self.create_publisher(Float64, '/teleop/gripper_cmd', 10)
        self._status_pub = self.create_publisher(String, '/policy/inference_status', 10)
        self._shadow_command_pub = None
        self._shadow_health_pub = None
        self._shadow_command_type = None
        self._shadow_report_pub = None
        self._shadow_report_type = None
        if self._policy_runtime_shadow_enabled:
            # Lazy import keeps CPU source tests usable before rosidl generation.
            from teleop_interfaces.msg import PolicyCommand, PolicyExecutionReport

            ttl_s = float(
                self.get_parameter('policy_runtime_command_ttl_s').value
            )
            event_callbacks = PublisherEventCallbacks(
                deadline=lambda _event: self._shadow_scheduler.record_deadline_miss(),
                liveliness=lambda _event: self._shadow_scheduler.record_liveliness_lost(),
            )
            self._shadow_command_type = PolicyCommand
            self._shadow_report_type = PolicyExecutionReport
            self._shadow_command_pub = self.create_publisher(
                PolicyCommand,
                '/policy/command',
                policy_command_qos(self._inference_period, ttl_s),
                event_callbacks=event_callbacks,
            )
            self._shadow_health_pub = self.create_publisher(
                DiagnosticArray,
                '/policy/runtime_health',
                runtime_health_qos(),
            )
            self._shadow_report_pub = self.create_publisher(
                PolicyExecutionReport,
                '/policy/execution_report',
                runtime_health_qos(),
            )
            self.get_logger().info(
                'Policy runtime adapter enabled mode='
                f'{self._execution_adapter_mode}'
            )
        self._estop_client = self.create_client(
            TriggerEstop, '/safety/trigger_estop',
            callback_group=self._groups['sensor'],
        )
        self.create_subscription(
            JointState, '/sim/encoder_state', self._on_joint_state,
            qos_profile_sensor_data, callback_group=self._groups['sensor']
        )
        self.create_subscription(
            Float64, '/gripper/state', self._on_gripper,
            qos_profile_sensor_data, callback_group=self._groups['sensor']
        )
        self.create_subscription(
            PoseStamped, '/ee_pose', self._on_ee_pose,
            qos_profile_sensor_data, callback_group=self._groups['sensor']
        )
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image,
            qos_profile_sensor_data, callback_group=self._groups['sensor']
        )
        self.create_subscription(
            Bool, '/safety/estop', self._on_estop, 10,
            callback_group=self._groups['sensor']
        )
        if self._policy_runtime_shadow_enabled:
            self.create_subscription(
                Bool, '/policy/runtime_hold', self._on_runtime_hold, 10,
                callback_group=self._groups['sensor']
            )
        self.create_subscription(
            SafetyStatus, '/safety/status', self._on_safety_status, 10,
            callback_group=self._groups['sensor']
        )
        self.create_timer(
            1.0 / inference_rate, self._on_inference_timer,
            callback_group=self._groups['inference']
        )
        self.create_timer(
            1.0 / command_rate, self._on_command_timer,
            callback_group=self._groups['command']
        )
        if self._policy_runtime_shadow_enabled:
            self.create_timer(
                1.0 / inference_rate,
                self._on_shadow_command_timer,
                callback_group=self._groups['command'],
            )
        self.create_timer(0.1, self._on_lifecycle_timer)

    def _store(self, key: str, value: object) -> None:
        with self._lock:
            self._observations[key] = (time.monotonic(), value)

    def _trip_execution_guard(self, reason: str, *, request_estop: bool = True) -> None:
        if self._dry_run or self._execution_reference_joints is None:
            return
        with self._lock:
            if self._execution_guard_reason is not None:
                return
            self._execution_guard_reason = reason
            self._target = None
        self._error = f'execution guard: {reason}'
        self._finished_at = time.monotonic()
        self.get_logger().error(self._error)
        self._flush_report(finalize=False)
        if request_estop and self._estop_client.service_is_ready():
            request = TriggerEstop.Request()
            request.reason = f'isaac_smolvla_policy:{reason}'
            self._estop_client.call_async(request)

    def _on_joint_state(self, message: JointState) -> None:
        lookup = {name: index for index, name in enumerate(message.name)}
        if any(name not in lookup for name in PANDA_ARM_JOINTS):
            return
        if len(message.position) < len(message.name):
            return
        positions = [
            float(message.position[lookup[name]]) for name in PANDA_ARM_JOINTS
        ]
        self._store('joints', positions)
        try:
            validate_panda_joint_positions(positions)
        except ValueError as error:
            self._trip_execution_guard(str(error))
            return
        reference = self._execution_reference_joints
        if reference is not None:
            excursion = max(
                abs(value - initial)
                for value, initial in zip(positions, reference)
            )
            self._max_observed_joint_excursion = max(
                self._max_observed_joint_excursion, excursion
            )
            if excursion > self._max_joint_excursion:
                self._trip_execution_guard(
                    f'joint excursion {excursion:.6f} rad exceeds '
                    f'{self._max_joint_excursion:.6f} rad'
                )

    def _on_gripper(self, message: Float64) -> None:
        if math.isfinite(float(message.data)):
            self._store('gripper', max(0.0, min(1.0, float(message.data))))

    def _on_ee_pose(self, message: PoseStamped) -> None:
        pose = message.pose
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        if all(math.isfinite(float(value)) for value in values):
            ee_pose = tuple(float(value) for value in values)
            self._store('ee_pose', ee_pose)
            reference = self._execution_reference_ee
            if reference is not None:
                excursion = math.sqrt(sum(
                    (ee_pose[index] - reference[index]) ** 2
                    for index in range(3)
                ))
                self._max_observed_ee_excursion = max(
                    self._max_observed_ee_excursion, excursion
                )
                if excursion > self._max_ee_excursion:
                    self._trip_execution_guard(
                        f'EE excursion {excursion:.6f} m exceeds '
                        f'{self._max_ee_excursion:.6f} m'
                    )

    def _on_image(self, message: Image) -> None:
        try:
            rgb = image_message_to_rgb(message)
        except ValueError as error:
            self._error = str(error)
            return
        self._store('image', rgb)

    def _on_estop(self, message: Bool) -> None:
        self._safety_estop = bool(message.data)
        if self._shadow_execution_adapter is not None:
            self._shadow_execution_adapter.set_estop(self._safety_estop)
        if self._safety_estop:
            if self._shadow_scheduler is not None:
                self._shadow_scheduler.clear_queue('risk_r3_estop')
            self._trip_execution_guard(
                'safety E-stop became active', request_estop=False
            )

    def _on_runtime_hold(self, message: Bool) -> None:
        """Apply M4 R2 Hold and require a fresh chunk after recovery."""
        active = bool(message.data)
        self._shadow_execution_adapter.set_hold(active)
        self._shadow_lifecycle.health.hold_active = active
        self._shadow_scheduler.clear_queue(
            'risk_r2_hold' if active else 'healthy_recovery_replan'
        )
        self._shadow_last_chunk_started = 0.0
        if self._execution_adapter_mode != 'authoritative':
            return
        hold_gripper = None
        with self._lock:
            if active and 'ee_pose' in self._observations and 'gripper' in self._observations:
                ee_pose = tuple(self._observations['ee_pose'][1])
                gripper = float(self._observations['gripper'][1])
                self._target = (ee_pose[:3], ee_pose[3:7], gripper)
                hold_gripper = gripper
            elif not active:
                self._target = None
        if hold_gripper is not None:
            self._gripper_pub.publish(Float64(data=hold_gripper))

    def _on_safety_status(self, message: SafetyStatus) -> None:
        self._safety_ok = bool(message.ok)
        if not self._safety_ok:
            self._trip_execution_guard('safety status became not-ok')

    def _snapshot(self):
        now = time.monotonic()
        with self._lock:
            if any(key not in self._observations for key in (
                'joints', 'gripper', 'ee_pose', 'image'
            )):
                self._mark_shadow_observation_unready(stale=False)
                return None
            if any(now - self._observations[key][0] > self._observation_timeout
                   for key in ('joints', 'gripper', 'ee_pose', 'image')):
                self._mark_shadow_observation_unready(stale=True)
                return None
            joints = list(self._observations['joints'][1])
            gripper = float(self._observations['gripper'][1])
            ee_pose = tuple(self._observations['ee_pose'][1])
            image = np.copy(self._observations['image'][1])
            captured_monotonic_ns = int(min(
                self._observations[key][0]
                for key in ('joints', 'gripper', 'ee_pose', 'image')
            ) * 1_000_000_000)
        return joints, gripper, ee_pose, image, captured_monotonic_ns

    def _mark_shadow_observation_unready(self, *, stale: bool) -> None:
        if not self._policy_runtime_shadow_enabled:
            return
        health = self._shadow_lifecycle.health
        if health.validity == 'ERROR':
            return
        health.validity = 'STALE' if stale else 'WARMING_UP'
        health.reason_code = (
            'observation_stale' if stale else 'observation_warming_up'
        )

    def _on_inference_timer(self) -> None:
        if self._finished_at is not None or self._inference_busy:
            return
        if len(self._actions) >= self._max_actions:
            self._finished_at = time.monotonic()
            return
        snapshot = self._snapshot()
        if snapshot is None:
            return
        joints, gripper, ee_pose, image, captured_monotonic_ns = snapshot
        try:
            validate_panda_joint_positions(joints)
        except ValueError as error:
            self._error = str(error)
            self._finished_at = time.monotonic()
            self.get_logger().error(self._error)
            return
        if not self._dry_run and not (
            self._safety_ok is True and self._safety_estop is False
        ):
            return
        now = time.monotonic()
        if (
            self._policy_runtime_shadow_enabled
            and now - self._shadow_last_chunk_started
            < self._s4_contract.replan_period_s
        ):
            return
        if now - self._last_inference_started < self._inference_period:
            return
        self._last_inference_started = now
        if self._policy_runtime_shadow_enabled:
            self._shadow_last_chunk_started = now
        self._inference_busy = True
        started = time.monotonic()
        try:
            state15 = compose_state15(joints, ee_pose, gripper)
            shadow_command = None
            if self._policy_runtime_shadow_enabled:
                raw_observation = RawObservation(
                    observation_sequence=self._shadow_observation_sequence,
                    captured_monotonic_ns=captured_monotonic_ns,
                    state=state15,
                    image=image,
                    task=None,
                )
                model_observation = self._shadow_backend.build_observation(
                    raw_observation
                )
                envelope = self._shadow_backend.predict_chunk(model_observation)
                if self._shadow_lifecycle.state.value == 'INACTIVE':
                    self._shadow_lifecycle.activate()
                self._shadow_scheduler.load_chunk(envelope)
                self._shadow_observation_sequence += 1
                return
            else:
                raw_action = self._runtime.infer(state15, image)
            bounded = bound_absolute_eef_gripper(
                raw_action,
                workspace_min=self._workspace_min,
                workspace_max=self._workspace_max,
            )
            target = absolute_action_to_target_pose(bounded.values)
            latency_ms = (time.monotonic() - started) * 1000.0
            action_index = len(self._actions)
            camera_rel = self._maybe_dump_camera_frame(action_index, image)
            entry = {
                'index': action_index,
                'observation_monotonic_ns': int(captured_monotonic_ns),
                'inference_completed_monotonic_ns': time.monotonic_ns(),
                'raw_action': raw_action,
                'bounded_action': list(bounded.values),
                'action_clipped': bounded.clipped,
                'state15': state15.tolist(),
                'source_ee_pose': list(ee_pose),
                'target_position': list(target.position),
                'target_orientation_xyzw': list(target.orientation_xyzw),
                'gripper_cmd': bounded.values[7],
                'inference_latency_ms': latency_ms,
                'camera_frame': camera_rel,
            }
            self._actions.append(entry)
            self._append_observation_sidecar(entry, camera_rel)
            if shadow_command is not None:
                self._publish_shadow_command(shadow_command)
            if not self._dry_run:
                with self._lock:
                    if self._execution_reference_joints is None:
                        self._execution_reference_joints = tuple(joints)
                        self._execution_reference_ee = tuple(ee_pose[:3])
                    self._target = (
                        target.position, target.orientation_xyzw,
                        bounded.values[7],
                    )
                self._gripper_pub.publish(Float64(data=bounded.values[7]))
            self._status_pub.publish(String(data=json.dumps(entry, sort_keys=True)))
            self.get_logger().info(
                f'SmolVLA action {len(self._actions)}/{self._max_actions}: '
                f'{latency_ms:.1f} ms, clipped={bounded.clipped}'
            )
            if len(self._actions) % 10 == 0:
                self._flush_report(finalize=False)
            if len(self._actions) >= self._max_actions:
                self._finished_at = time.monotonic()
                self._flush_report(finalize=False)
        except Exception as error:
            self._error = f'{type(error).__name__}: {error}'
            if (
                self._policy_runtime_shadow_enabled
                and self._shadow_lifecycle.state.value != 'ERROR_PROCESSING'
            ):
                self._shadow_lifecycle.error(self._error)
                reason_code, failure_lane = classify_runtime_error(error)
                self._shadow_lifecycle.health.reason_code = reason_code
                self._shadow_lifecycle.health.failure_lane = failure_lane
            self._finished_at = time.monotonic()
            self.get_logger().error(self._error)
            self._flush_report(finalize=False)
        finally:
            self._inference_busy = False

    def _on_command_timer(self) -> None:
        snapshot = self._snapshot()
        if snapshot is not None and not self._shutdown_requested:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'isaac_smolvla_policy'
            self._heartbeat_pub.publish(header)
        if self._dry_run or not (
            self._safety_ok is True and self._safety_estop is False
        ):
            return
        with self._lock:
            target = self._target
        if target is None:
            return
        position, orientation, _ = target
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'panda_link0'
        message.pose.position.x, message.pose.position.y, message.pose.position.z = position
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = orientation
        self._pose_pub.publish(message)

    def _on_shadow_command_timer(self) -> None:
        """Consume chunks through the selected shadow/authoritative adapter."""
        if self._finished_at is not None:
            return
        snapshot = self._snapshot()
        if snapshot is None:
            return
        _joints, _gripper, ee_pose, _image, _captured_ns = snapshot
        now_ns = time.monotonic_ns()
        command = self._shadow_scheduler.next_command(now_ns)
        if command is None:
            return
        state = ExecutionState(
            position=tuple(ee_pose[:3]),
            orientation_xyzw=tuple(ee_pose[3:7]),
        )
        decision = self._shadow_execution_adapter.evaluate(
            command, state, now_monotonic_ns=now_ns
        )
        if (
            self._execution_adapter_mode == 'authoritative'
            and self._authoritative_require_single_publisher
            and not self._authoritative_publisher_identity_checked
        ):
            validate_authoritative_publisher_counts(
                self.count_publishers('/teleop/cmd_pose'),
                self.count_publishers('/teleop/gripper_cmd'),
            )
            self._authoritative_publisher_identity_checked = True
        self._publish_shadow_command(command)
        self._publish_shadow_report(decision)
        entry = {
            'index': len(self._actions),
            'command_sequence': command.command_sequence,
            'observation_sequence': command.observation_sequence,
            'chunk_index': command.chunk_index,
            'chunk_size': command.chunk_size,
            'raw_action': list(command.action),
            'bounded_action': (
                None
                if decision.bounded_action is None
                else list(decision.bounded_action)
            ),
            'action_clipped': decision.clipped,
            'clip_axes': list(decision.clip_axes),
            'decision': decision.decision,
            'reason_code': decision.reason_code,
            'shadow_only': self._execution_adapter_mode != 'authoritative',
            'claims_task_success': False,
        }
        if decision.accepted and decision.bounded_action is not None:
            legacy = legacy_absolute_result(
                command.action,
                workspace_min=self._workspace_min,
                workspace_max=self._workspace_max,
            )
            entry['parity'] = {
                'position_max_abs_m': max(
                    abs(left - right)
                    for left, right in zip(
                        legacy[0], decision.bounded_action[:3]
                    )
                ),
                'quaternion_max_abs': max(
                    abs(left - right)
                    for left, right in zip(
                        legacy[1], decision.bounded_action[3:7]
                    )
                ),
                'gripper_abs': abs(legacy[2] - decision.bounded_action[7]),
                'clip_match': legacy[3] == decision.clipped,
            }
            if self._execution_adapter_mode == 'authoritative':
                bounded = decision.bounded_action
                with self._lock:
                    if self._execution_reference_joints is None:
                        self._execution_reference_joints = tuple(_joints)
                        self._execution_reference_ee = tuple(ee_pose[:3])
                    self._target = (bounded[:3], bounded[3:7], bounded[7])
                self._gripper_pub.publish(Float64(data=bounded[7]))
        self._actions.append(entry)
        self._status_pub.publish(String(data=json.dumps(entry, sort_keys=True)))
        if len(self._actions) >= self._max_actions:
            self._finished_at = time.monotonic()
            self._flush_report(finalize=False)

    def _publish_shadow_command(self, command) -> None:
        """Publish M1 telemetry only; never write a teleop execution topic."""
        if self._shadow_command_pub is None:
            return
        now = self.get_clock().now()
        ttl_s = float(self.get_parameter('policy_runtime_command_ttl_s').value)
        valid_until = now + Duration(seconds=ttl_s)
        message = populate_policy_command(
            self._shadow_command_type(),
            command,
            source_stamp=now.to_msg(),
            received_stamp=now.to_msg(),
            valid_until=valid_until.to_msg(),
        )
        self._shadow_command_pub.publish(message)
        self._shadow_command_pub.assert_liveliness()

    def _publish_shadow_report(self, decision) -> None:
        if self._shadow_report_pub is None:
            return
        now = self.get_clock().now().to_msg()
        message = populate_execution_report(
            self._shadow_report_type(),
            decision,
            source_stamp=now,
            received_stamp=now,
        )
        self._shadow_report_pub.publish(message)

    def _publish_shadow_health(self) -> None:
        if self._shadow_health_pub is None:
            return
        health = self._shadow_lifecycle.health
        health.inference_busy = self._inference_busy
        health.estop_active = self._safety_estop is True
        health.refresh_ages(time.monotonic_ns())
        message = build_runtime_health_array(
            health,
            stamp=self.get_clock().now().to_msg(),
            policy_name=self._shadow_artifact.policy_name,
            policy_version=self._shadow_artifact.policy_version,
            checkpoint_hash=self._shadow_artifact.checkpoint_hash,
            observation_schema_version=(
                self._shadow_artifact.observation_schema_version
            ),
            shadow_only=self._execution_adapter_mode != 'authoritative',
            trace_run_id=self._shadow_context.trace_run_id,
            episode_id=self._shadow_context.episode_id,
        )
        self._shadow_health_pub.publish(message)

    def _maybe_dump_camera_frame(
        self, action_index: int, image: np.ndarray
    ) -> str | None:
        """Save policy-input RGB for H1 attribution; return relative path or None."""
        if self._telemetry_dir is None:
            return None
        if action_index % self._camera_dump_stride != 0:
            return None
        try:
            from PIL import Image as PILImage
        except ImportError:
            # Fallback without Pillow: write raw .npy
            rel = f'camera/action_{action_index:04d}.npy'
            np.save(self._telemetry_dir / rel, image)
            self._camera_dump_count += 1
            return rel
        rel = f'camera/action_{action_index:04d}.jpg'
        PILImage.fromarray(image).save(
            self._telemetry_dir / rel, format='JPEG', quality=90
        )
        self._camera_dump_count += 1
        return rel

    def _append_observation_sidecar(
        self, entry: dict, camera_rel: str | None
    ) -> None:
        if self._observations_jsonl is None:
            return
        row = {
            'contract_version': 'smolvla_observation_telemetry_v2',
            'index': entry['index'],
            'episode_id': self._shadow_context.episode_id,
            'observation_monotonic_ns': entry['observation_monotonic_ns'],
            'inference_completed_monotonic_ns': entry[
                'inference_completed_monotonic_ns'
            ],
            'state15': entry['state15'],
            'source_ee_pose': entry['source_ee_pose'],
            'gripper_cmd': entry['gripper_cmd'],
            'camera_frame': camera_rel,
            'inference_latency_ms': entry['inference_latency_ms'],
        }
        with self._observations_jsonl.open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(row, sort_keys=True) + '\n')

    def _flush_report(self, *, finalize: bool) -> None:
        now = time.monotonic()
        inference_pass = (
            len(self._actions) == self._max_actions
            and (self._error is None or self._execution_guard_reason is not None)
        )
        if self._dry_run:
            execution_status = 'NOT_EXECUTED'
            overall_pass = inference_pass
        else:
            execution_status = (
                'PASS'
                if inference_pass
                and self._execution_guard_reason is None
                and self._safety_ok is True
                and self._safety_estop is False
                else 'FAIL'
            )
            overall_pass = inference_pass and execution_status == 'PASS'
        self._report_status = 'PASS' if overall_pass else 'FAIL'
        with self._lock:
            final_joints = (
                list(self._observations['joints'][1])
                if 'joints' in self._observations else None
            )
            final_ee_pose = (
                list(self._observations['ee_pose'][1])
                if 'ee_pose' in self._observations else None
            )
        latencies = [
            float(a['inference_latency_ms'])
            for a in self._actions
            if isinstance(a.get('inference_latency_ms'), (int, float))
        ]
        report = {
            'status': self._report_status,
            'inference_status': 'PASS' if inference_pass else 'FAIL',
            'execution_status': execution_status,
            'dry_run': self._dry_run,
            'requested_actions': self._max_actions,
            'completed_actions': len(self._actions),
            'error': self._error,
            'policy_type': self._runtime.metadata.get('policy_type'),
            'action_type': self._runtime.metadata.get('action_type'),
            'policy_action_semantics': self._runtime.metadata.get(
                'policy_action_semantics'
            ),
            'deploy_n_action_steps': self._runtime.metadata.get(
                'deploy_n_action_steps'
            ),
            'chunk_size': self._runtime.metadata.get('chunk_size'),
            'lora_dir': self._runtime.metadata.get('lora_dir'),
            'final_safety_ok': self._safety_ok,
            'final_safety_estop': self._safety_estop,
            'execution_guard_reason': self._execution_guard_reason,
            'max_observed_joint_excursion_rad': self._max_observed_joint_excursion,
            'max_observed_ee_excursion_m': self._max_observed_ee_excursion,
            'final_joint_positions': final_joints,
            'final_ee_pose': final_ee_pose,
            'inference_latency_ms_p50': (
                float(sorted(latencies)[len(latencies) // 2]) if latencies else None
            ),
            'actions': self._actions,
            'elapsed_s': now - self._started,
            'report_finalized': finalize,
            'claims_task_success': False,
            'ran_isaac': True,
            'telemetry_dir': (
                str(self._telemetry_dir) if self._telemetry_dir else None
            ),
            'camera_frames_dumped': self._camera_dump_count,
            'gate_note': (
                'Policy report is interface/execution health only; '
                'lift success comes from ContinuousTaskEvaluator GT.'
            ),
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self._output_path.write_text(
            json.dumps(report, indent=2, sort_keys=True), encoding='utf-8'
        )
        self.get_logger().info(f'SMOLVLA_S4_POLICY_REPORT={self._output_path}')
        self.get_logger().info(f"SMOLVLA_S4_POLICY_STATUS={report['status']}")

    def _on_lifecycle_timer(self) -> None:
        self._publish_shadow_health()
        now = time.monotonic()
        if (
            self._finished_at is None
            and len(self._actions) == 0
            and now - self._started > self._startup_timeout
        ):
            self._error = (
                'startup timeout waiting for fresh observations and safety state'
            )
            self._finished_at = now
            self._flush_report(finalize=False)
        if self._finished_at is None:
            return
        if now - self._finished_at < self._post_action_hold:
            return
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self._flush_report(finalize=True)
        rclpy.shutdown()

    @property
    def succeeded(self) -> bool:
        return self._report_status == 'PASS'


def main(args=None) -> int:
    rclpy.init(args=args)
    node = IsaacSmolVLAPolicyInferenceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.succeeded else 1


if __name__ == '__main__':
    raise SystemExit(main())
