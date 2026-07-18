"""Bounded online ACT inference node for the upstream Isaac control stack."""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time

from geometry_msgs.msg import PoseStamped
from isaac_sim_adapter.effort_control import PANDA_ARM_JOINTS
from isaac_sim_adapter.policy_control import action_to_target_pose
from isaac_sim_adapter.policy_control import bound_ee_delta_gripper
from isaac_sim_adapter.policy_control import validate_panda_joint_positions
from isaac_sim_adapter.scene_act_runtime import SceneACTRuntime
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, Header, String
from teleop_interfaces.msg import SafetyStatus
from teleop_interfaces.srv import TriggerEstop
from trajectory_msgs.msg import JointTrajectory


def image_message_to_rgb(message: Image) -> np.ndarray:
    """Decode common uncompressed ROS Image encodings without cv_bridge."""
    channels = {
        'rgb8': (3, False, False),
        'bgr8': (3, True, False),
        'rgba8': (4, False, True),
        'bgra8': (4, True, True),
        'mono8': (1, False, False),
    }
    if message.encoding not in channels:
        raise ValueError(f'unsupported camera encoding: {message.encoding!r}')
    channel_count, reverse, drop_alpha = channels[message.encoding]
    row_bytes = int(message.width) * channel_count
    if int(message.step) < row_bytes:
        raise ValueError('camera row step is smaller than encoded width')
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    required = int(message.height) * int(message.step)
    if raw.size < required:
        raise ValueError('camera payload is truncated')
    rows = raw[:required].reshape(int(message.height), int(message.step))
    image = rows[:, :row_bytes].reshape(
        int(message.height), int(message.width), channel_count
    )
    if channel_count == 1:
        image = np.repeat(image, 3, axis=2)
    elif drop_alpha:
        image = image[:, :, :3]
    if reverse:
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


class IsaacPolicyInferenceNode(Node):
    """Infer ACT actions from live Isaac observations and publish bounded poses."""

    def __init__(self) -> None:
        super().__init__('isaac_policy_inference')
        self.declare_parameter('checkpoint', '')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('max_actions', 1)
        self.declare_parameter('inference_rate_hz', 2.0)
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('observation_timeout_s', 0.5)
        self.declare_parameter('startup_timeout_s', 45.0)
        self.declare_parameter('post_action_hold_s', 1.0)
        self.declare_parameter('max_translation_m', 0.005)
        self.declare_parameter('max_rotation_rad', math.radians(2.0))
        self.declare_parameter('max_joint_excursion_rad', 0.25)
        self.declare_parameter('max_ee_excursion_m', 0.03)
        self.declare_parameter('workspace_min', [0.20, -0.40, 0.15])
        self.declare_parameter('workspace_max', [0.65, 0.40, 0.75])
        self.declare_parameter('output_path', '/tmp/isaac_act_policy_report.json')

        checkpoint = Path(str(self.get_parameter('checkpoint').value)).expanduser()
        if not checkpoint.is_file():
            raise ValueError(f'checkpoint does not exist: {checkpoint}')
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._max_actions = int(self.get_parameter('max_actions').value)
        if self._max_actions <= 0 or self._max_actions > 20:
            raise ValueError('max_actions must be in [1, 20]')
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
        self._max_translation = float(
            self.get_parameter('max_translation_m').value
        )
        self._max_rotation = float(
            self.get_parameter('max_rotation_rad').value
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

        requested_device = str(self.get_parameter('device').value)
        if requested_device not in {'cpu', 'cuda'}:
            raise ValueError('device must be cpu or cuda')
        if requested_device == 'cuda':
            import torch
            if not torch.cuda.is_available():
                raise RuntimeError('device=cuda requested but CUDA is unavailable')
        self.get_logger().info(f'Loading ACT checkpoint on {requested_device}')
        self._runtime = SceneACTRuntime(checkpoint, requested_device)

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
        self._max_observed_joint_target_excursion = 0.0
        self._max_observed_ee_excursion = 0.0
        self._final_joint_target: list[float] | None = None
        self._max_observed_effort_nm = [0.0] * len(PANDA_ARM_JOINTS)
        self._final_effort_command_nm: list[float] | None = None
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
        self.create_subscription(
            SafetyStatus, '/safety/status', self._on_safety_status, 10,
            callback_group=self._groups['sensor']
        )
        self.create_subscription(
            JointTrajectory, '/joint_target', self._on_joint_target, 10,
            callback_group=self._groups['sensor']
        )
        self.create_subscription(
            Float64MultiArray, '/sim/joint_effort_cmd', self._on_effort_command,
            qos_profile_sensor_data, callback_group=self._groups['sensor']
        )
        self.create_timer(
            1.0 / inference_rate, self._on_inference_timer,
            callback_group=self._groups['inference']
        )
        self.create_timer(
            1.0 / command_rate, self._on_command_timer,
            callback_group=self._groups['command']
        )
        self.create_timer(0.1, self._on_lifecycle_timer)

    def _store(self, key: str, value: object) -> None:
        with self._lock:
            self._observations[key] = (time.monotonic(), value)

    def _trip_execution_guard(self, reason: str) -> None:
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
        if self._estop_client.service_is_ready():
            request = TriggerEstop.Request()
            request.reason = f'isaac_act_policy:{reason}'
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

    def _on_joint_target(self, message: JointTrajectory) -> None:
        if not message.points:
            return
        point = message.points[-1]
        lookup = {name: index for index, name in enumerate(message.joint_names)}
        if any(name not in lookup for name in PANDA_ARM_JOINTS):
            return
        if len(point.positions) < len(message.joint_names):
            return
        target = [
            float(point.positions[lookup[name]]) for name in PANDA_ARM_JOINTS
        ]
        if not all(math.isfinite(value) for value in target):
            self._trip_execution_guard('Servo joint target contains NaN or infinity')
            return
        self._final_joint_target = target
        reference = self._execution_reference_joints
        if reference is None:
            return
        excursion = max(
            abs(value - initial)
            for value, initial in zip(target, reference)
        )
        self._max_observed_joint_target_excursion = max(
            self._max_observed_joint_target_excursion, excursion
        )
        if excursion > self._max_joint_excursion:
            self._trip_execution_guard(
                f'Servo joint target excursion {excursion:.6f} rad exceeds '
                f'{self._max_joint_excursion:.6f} rad'
            )

    def _on_effort_command(self, message: Float64MultiArray) -> None:
        if len(message.data) != len(PANDA_ARM_JOINTS):
            return
        values = [float(value) for value in message.data]
        if not all(math.isfinite(value) for value in values):
            self._trip_execution_guard('joint effort command contains NaN or infinity')
            return
        self._final_effort_command_nm = values
        self._max_observed_effort_nm = [
            max(previous, abs(value))
            for previous, value in zip(self._max_observed_effort_nm, values)
        ]

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
        if self._safety_estop:
            self._trip_execution_guard('safety E-stop became active')

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
                return None
            if any(now - self._observations[key][0] > self._observation_timeout
                   for key in ('joints', 'gripper', 'ee_pose', 'image')):
                return None
            joints = list(self._observations['joints'][1])
            gripper = float(self._observations['gripper'][1])
            ee_pose = tuple(self._observations['ee_pose'][1])
            image = np.copy(self._observations['image'][1])
        return [*joints, gripper], ee_pose, image

    def _on_inference_timer(self) -> None:
        if self._finished_at is not None or self._inference_busy:
            return
        if len(self._actions) >= self._max_actions:
            self._finished_at = time.monotonic()
            return
        snapshot = self._snapshot()
        if snapshot is None:
            return
        state, ee_pose, image = snapshot
        try:
            validate_panda_joint_positions(state[:7])
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
        if now - self._last_inference_started < self._inference_period:
            return
        self._last_inference_started = now
        self._inference_busy = True
        started = time.monotonic()
        try:
            raw_action = self._runtime.infer(state, image)
            bounded = bound_ee_delta_gripper(
                raw_action,
                max_translation_m=self._max_translation,
                max_rotation_rad=self._max_rotation,
            )
            target = action_to_target_pose(
                ee_pose[:3], ee_pose[3:], bounded.values,
                workspace_min=self._workspace_min,
                workspace_max=self._workspace_max,
            )
            latency_ms = (time.monotonic() - started) * 1000.0
            entry = {
                'index': len(self._actions),
                'raw_action': raw_action,
                'bounded_action': list(bounded.values),
                'action_clipped': bounded.clipped,
                'workspace_clipped': target.workspace_clipped,
                'state': state,
                'source_ee_pose': list(ee_pose),
                'target_position': list(target.position),
                'target_orientation_xyzw': list(target.orientation_xyzw),
                'inference_latency_ms': latency_ms,
            }
            self._actions.append(entry)
            if not self._dry_run:
                with self._lock:
                    if self._execution_reference_joints is None:
                        self._execution_reference_joints = tuple(state[:7])
                        self._execution_reference_ee = tuple(ee_pose[:3])
                    self._target = (
                        target.position, target.orientation_xyzw,
                        bounded.values[6],
                    )
                self._gripper_pub.publish(Float64(data=bounded.values[6]))
            self._status_pub.publish(String(data=json.dumps(entry, sort_keys=True)))
            self.get_logger().info(
                f'ACT action {len(self._actions)}/{self._max_actions}: '
                f'{latency_ms:.1f} ms, clipped={bounded.clipped}'
            )
            if len(self._actions) >= self._max_actions:
                self._finished_at = time.monotonic()
        except Exception as error:
            self._error = f'{type(error).__name__}: {error}'
            self._finished_at = time.monotonic()
            self.get_logger().error(self._error)
        finally:
            self._inference_busy = False

    def _on_command_timer(self) -> None:
        snapshot = self._snapshot()
        if snapshot is not None and not self._shutdown_requested:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'isaac_act_policy'
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

    def _on_lifecycle_timer(self) -> None:
        now = time.monotonic()
        if self._finished_at is None and now - self._started > self._startup_timeout:
            self._error = 'startup timeout waiting for fresh observations and safety state'
            self._finished_at = now
        if self._finished_at is None:
            return
        if now - self._finished_at < self._post_action_hold:
            return
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
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
        report = {
            'status': self._report_status,
            'inference_status': 'PASS' if inference_pass else 'FAIL',
            'execution_status': execution_status,
            'dry_run': self._dry_run,
            'requested_actions': self._max_actions,
            'completed_actions': len(self._actions),
            'error': self._error,
            'checkpoint_release_id': self._runtime.metadata.get('release_id'),
            'action_type': self._runtime.metadata.get('action_type'),
            'final_safety_ok': self._safety_ok,
            'final_safety_estop': self._safety_estop,
            'execution_guard_reason': self._execution_guard_reason,
            'max_observed_joint_excursion_rad': self._max_observed_joint_excursion,
            'max_observed_joint_target_excursion_rad': (
                self._max_observed_joint_target_excursion
            ),
            'max_observed_ee_excursion_m': self._max_observed_ee_excursion,
            'final_joint_target': self._final_joint_target,
            'max_observed_effort_nm': self._max_observed_effort_nm,
            'final_effort_command_nm': self._final_effort_command_nm,
            'final_joint_positions': final_joints,
            'final_ee_pose': final_ee_pose,
            'actions': self._actions,
            'elapsed_s': now - self._started,
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self._output_path.write_text(
            json.dumps(report, indent=2, sort_keys=True), encoding='utf-8'
        )
        self.get_logger().info(f'ACT_POLICY_REPORT={self._output_path}')
        self.get_logger().info(f"ACT_POLICY_STATUS={report['status']}")
        rclpy.shutdown()

    @property
    def succeeded(self) -> bool:
        return self._report_status == 'PASS'


def main(args=None) -> int:
    rclpy.init(args=args)
    node = IsaacPolicyInferenceNode()
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
