"""Translate isolated Isaac topics into the upstream simulation contract."""

from __future__ import annotations

import threading
import time
from typing import Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, WrenchStamped
from isaac_sim_adapter.effort_control import LatestEffortCommand
from isaac_sim_adapter.effort_control import PANDA_ARM_JOINTS
from isaac_sim_adapter.effort_control import ZERO_EFFORT
from isaac_sim_adapter.policy_control import bound_gripper_command
from isaac_sim_adapter.policy_control import validate_panda_joint_positions
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

SUPPORTED_TARGET_OBJECTS = ('object_red_box',)


def normalize_namespace(value: str) -> str:
    """Return a single-leading-slash namespace without a trailing slash."""
    normalized = str(value).strip().strip('/')
    if not normalized:
        raise ValueError('source_namespace must not be empty')
    return f'/{normalized}'


def filter_arm_joint_state(message: JointState) -> JointState | None:
    """Extract the canonical seven Panda arm joints in deterministic order."""
    lookup = {name: index for index, name in enumerate(message.name)}
    if any(name not in lookup for name in PANDA_ARM_JOINTS):
        return None
    indices = [lookup[name] for name in PANDA_ARM_JOINTS]
    result = JointState()
    result.header = message.header
    result.name = list(PANDA_ARM_JOINTS)

    def extract(values: Sequence[float]) -> list[float]:
        if not values or len(values) <= max(indices):
            return []
        return [values[index] for index in indices]

    result.position = extract(message.position)
    result.velocity = extract(message.velocity)
    result.effort = extract(message.effort)
    return result


def extract_arm_joint_target(message: JointTrajectory) -> list[float] | None:
    """Extract the final Servo point in canonical Panda joint order."""
    if not message.points:
        return None
    lookup = {name: index for index, name in enumerate(message.joint_names)}
    if any(name not in lookup for name in PANDA_ARM_JOINTS):
        return None
    positions = message.points[-1].positions
    if len(positions) < len(message.joint_names):
        return None
    return [float(positions[lookup[name]]) for name in PANDA_ARM_JOINTS]


class IsaacSimAdapter(Node):
    """Expose P2/P3 Isaac capabilities through the existing ROS contract."""

    def __init__(self) -> None:
        super().__init__('isaac_sim_adapter')
        self.declare_parameter('source_namespace', '/isaac')
        self.declare_parameter('target_object_name', 'object_red_box')
        self.declare_parameter('startup_timeout_s', 45.0)
        self.declare_parameter('reset_timeout_s', 5.0)
        self.declare_parameter('command_timeout_s', 0.1)
        self.declare_parameter('state_timeout_s', 0.1)
        self.declare_parameter('command_forward_rate_hz', 250.0)
        self.declare_parameter('max_position_target_step_rad', 0.25)

        self._source = normalize_namespace(
            self.get_parameter('source_namespace').value
        )
        self._target_object = str(
            self.get_parameter('target_object_name').value
        )
        if self._target_object not in SUPPORTED_TARGET_OBJECTS:
            raise ValueError(
                f'P3 Isaac scene supports {SUPPORTED_TARGET_OBJECTS}; '
                f'got {self._target_object!r}'
            )

        command_timeout_s = float(
            self.get_parameter('command_timeout_s').value
        )
        state_timeout_s = float(self.get_parameter('state_timeout_s').value)
        forward_rate_hz = float(
            self.get_parameter('command_forward_rate_hz').value
        )
        if forward_rate_hz <= 0.0:
            raise ValueError('command_forward_rate_hz must be positive')
        self._max_position_target_step = float(
            self.get_parameter('max_position_target_step_rad').value
        )
        if self._max_position_target_step <= 0.0:
            raise ValueError('max_position_target_step_rad must be positive')
        self._last_arm_positions: tuple[float, ...] | None = None
        self._effort = LatestEffortCommand(
            command_timeout_s=command_timeout_s,
            state_timeout_s=state_timeout_s,
        )

        # Keep control/state isolated from reset transactions, camera work and
        # diagnostics. The command buffer remains locked because the executor
        # intentionally runs these groups concurrently.
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._reset_group = MutuallyExclusiveCallbackGroup()
        self._sensor_group = ReentrantCallbackGroup()
        self._camera_group = MutuallyExclusiveCallbackGroup()
        self._status_group = MutuallyExclusiveCallbackGroup()
        self._last_joint_time = 0.0
        self._last_object_time = 0.0
        self._last_camera_time = 0.0
        self._last_wrist_camera_time = 0.0
        self._last_ee_time = 0.0
        self._last_ft_time = 0.0
        self._reset_event = threading.Event()
        self._reset_success = False
        self._last_command_status = 'no_command'
        self._watchdog_event_count = 0

        self._encoder_pub = self.create_publisher(
            JointState, '/sim/encoder_state', qos_profile_sensor_data
        )
        self._object_pub = self.create_publisher(
            PoseStamped, '/sim/object_pose', qos_profile_sensor_data
        )
        self._camera_pub = self.create_publisher(
            Image, '/camera/scene/image_raw', qos_profile_sensor_data
        )
        self._recorder_camera_pub = self.create_publisher(
            Image, '/camera/color/image_raw', qos_profile_sensor_data
        )
        self._wrist_camera_pub = self.create_publisher(
            Image, '/camera/wrist/color/image_raw', qos_profile_sensor_data
        )
        self._ee_pub = self.create_publisher(
            PoseStamped, '/ee_pose', qos_profile_sensor_data
        )
        self._ft_pub = self.create_publisher(
            WrenchStamped, '/ft_sensor', qos_profile_sensor_data
        )
        self._gripper_pub = self.create_publisher(
            Float64, '/gripper/state', qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray, '/sim/backend_status', 10
        )
        self._target_pub = self.create_publisher(
            String, f'{self._source}/target_object_name', 1
        )
        self._reset_command_pub = self.create_publisher(
            Empty, f'{self._source}/reset_scene_cmd', 1
        )
        self._raw_effort_pub = self.create_publisher(
            Float64MultiArray,
            f'{self._source}/joint_effort_cmd',
            qos_profile_sensor_data,
        )
        self._raw_gripper_pub = self.create_publisher(
            Float64, f'{self._source}/gripper_cmd', 10
        )
        self._raw_position_pub = self.create_publisher(
            Float64MultiArray, f'{self._source}/joint_position_cmd', 10
        )
        self._event_pub = self.create_publisher(
            String, '/sim/backend_events', 20
        )

        self.create_subscription(
            JointState,
            f'{self._source}/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
            callback_group=self._control_group,
        )
        self.create_subscription(
            PoseStamped,
            f'{self._source}/object_pose',
            self._on_object_pose,
            qos_profile_sensor_data,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            PoseStamped,
            f'{self._source}/ee_pose',
            self._on_ee_pose,
            qos_profile_sensor_data,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            WrenchStamped,
            f'{self._source}/ft_sensor',
            self._on_ft_sensor,
            qos_profile_sensor_data,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            Image,
            f'{self._source}/camera/color/image_raw',
            self._on_camera,
            qos_profile_sensor_data,
            callback_group=self._camera_group,
        )
        self.create_subscription(
            Image,
            f'{self._source}/camera/wrist/color/image_raw',
            self._on_wrist_camera,
            qos_profile_sensor_data,
            callback_group=self._camera_group,
        )
        self.create_subscription(
            Float64MultiArray,
            '/sim/joint_effort_cmd',
            self._on_effort_command,
            qos_profile_sensor_data,
            callback_group=self._control_group,
        )
        self.create_subscription(
            Float64,
            '/teleop/gripper_cmd',
            self._on_gripper_command,
            10,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            JointTrajectory,
            '/joint_target',
            self._on_joint_position_target,
            10,
            callback_group=self._control_group,
        )
        self.create_subscription(
            Bool,
            f'{self._source}/reset_scene_done',
            self._on_reset_done,
            1,
            callback_group=self._sensor_group,
        )
        self.create_service(
            Trigger,
            '/sim/reset_scene',
            self._reset_scene,
            callback_group=self._reset_group,
        )
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(
            1.0, self._publish_status, callback_group=self._status_group
        )
        self.create_timer(
            1.0, self._publish_target_name, callback_group=self._status_group
        )
        self.create_timer(
            1.0 / forward_rate_hz,
            self._forward_latest_effort,
            callback_group=self._control_group,
        )
        self._started_at = time.monotonic()
        self.create_timer(
            0.5,
            self._check_startup_timeout,
            callback_group=self._status_group,
        )

        self.get_logger().info(
            f'Isaac adapter waiting for {self._source}/joint_states'
        )

    def _on_joint_state(self, message: JointState) -> None:
        filtered = filter_arm_joint_state(message)
        if filtered is None:
            self.get_logger().error(
                'Raw Isaac joint state is missing one or more Panda arm joints'
            )
            return
        self._last_joint_time = time.monotonic()
        self._effort.update_state(self._last_joint_time)
        if len(filtered.position) == len(PANDA_ARM_JOINTS):
            self._last_arm_positions = tuple(
                float(value) for value in filtered.position
            )
        self._encoder_pub.publish(filtered)
        lookup = {name: index for index, name in enumerate(message.name)}
        finger_names = ('panda_finger_joint1', 'panda_finger_joint2')
        if message.position and all(name in lookup for name in finger_names):
            opening = sum(
                float(message.position[lookup[name]]) for name in finger_names
            ) / len(finger_names)
            normalized = max(0.0, min(1.0, opening / 0.04))
            self._gripper_pub.publish(Float64(data=normalized))

    def _on_object_pose(self, message: PoseStamped) -> None:
        self._last_object_time = time.monotonic()
        self._object_pub.publish(message)

    def _on_camera(self, message: Image) -> None:
        self._last_camera_time = time.monotonic()
        # Isaac's camera helper uses simulation time while the existing
        # recorder contract uses ROS system time. Normalize at the adapter
        # boundary so latest-sample synchronization has one time base.
        message.header.stamp = self.get_clock().now().to_msg()
        self._camera_pub.publish(message)
        self._recorder_camera_pub.publish(message)

    def _on_wrist_camera(self, message: Image) -> None:
        self._last_wrist_camera_time = time.monotonic()
        message.header.stamp = self.get_clock().now().to_msg()
        self._wrist_camera_pub.publish(message)

    def _on_ee_pose(self, message: PoseStamped) -> None:
        self._last_ee_time = time.monotonic()
        # The P3 scene anchors Franka at the world origin, so world and
        # panda_link0 coincide. Publish the existing canonical EE frame.
        message.header.frame_id = 'panda_link0'
        self._ee_pub.publish(message)

    def _on_ft_sensor(self, message: WrenchStamped) -> None:
        self._last_ft_time = time.monotonic()
        self._ft_pub.publish(message)

    def _on_effort_command(self, message: Float64MultiArray) -> None:
        try:
            decision = self._effort.accept(message.data)
        except (TypeError, ValueError, OverflowError) as error:
            self.get_logger().error(f'Rejected Isaac effort command: {error}')
            self._publish_event('invalid_command', str(error))
            decision = self._effort.output()
        if decision.status != 'active':
            self._publish_event(decision.status, 'command rejected by safety gate')
        elif decision.clipped:
            self._publish_event('command_clipped', 'joint effort limit applied')
        if decision.should_publish:
            self._publish_raw_effort(decision.efforts)

    def _on_gripper_command(self, message: Float64) -> None:
        try:
            command, clipped = bound_gripper_command(message.data)
        except (TypeError, ValueError, OverflowError) as error:
            self.get_logger().error(f'Rejected Isaac gripper command: {error}')
            self._publish_event('invalid_gripper_command', str(error))
            return
        if clipped:
            self._publish_event(
                'gripper_command_clipped', 'normalized command limited to [0, 1]'
            )
        self._raw_gripper_pub.publish(Float64(data=command))

    def _on_joint_position_target(self, message: JointTrajectory) -> None:
        target = extract_arm_joint_target(message)
        if target is None:
            self._publish_event(
                'invalid_position_target', 'missing canonical Panda trajectory point'
            )
            return
        try:
            validate_panda_joint_positions(target)
        except (TypeError, ValueError, OverflowError) as error:
            self._publish_event('invalid_position_target', str(error))
            return
        current = self._last_arm_positions
        if current is None:
            self._publish_event(
                'position_target_without_state', 'joint state is unavailable'
            )
            return
        excursion = max(
            abs(value - state) for value, state in zip(target, current)
        )
        if excursion > self._max_position_target_step:
            self._publish_event(
                'position_target_rejected',
                f'excursion {excursion:.6f} rad exceeds '
                f'{self._max_position_target_step:.6f} rad',
            )
            return
        self._raw_position_pub.publish(Float64MultiArray(data=target))

    def _forward_latest_effort(self) -> None:
        decision = self._effort.output()
        if decision.should_publish:
            self._publish_raw_effort(decision.efforts)
        if decision.status == self._last_command_status:
            return
        if decision.status in {'command_stale', 'state_stale'}:
            self._watchdog_event_count += 1
            self._publish_event(
                decision.status,
                'zero-effort fail-safe applied; command history cleared',
            )
        self._last_command_status = decision.status

    def _publish_raw_effort(self, values: Sequence[float]) -> None:
        self._raw_effort_pub.publish(
            Float64MultiArray(data=[float(value) for value in values])
        )

    def _publish_event(self, event: str, details: str) -> None:
        payload = (
            f'event={event};monotonic_s={time.monotonic():.9f};details={details}'
        )
        self._event_pub.publish(String(data=payload))

    def _on_reset_done(self, message: Bool) -> None:
        self._reset_success = bool(message.data)
        self._reset_event.set()

    def _reset_scene(self, _request, response):
        self._effort.begin_reset()
        self._publish_raw_effort(ZERO_EFFORT)
        self._publish_event('reset_started', 'command history cleared')
        self._reset_event.clear()
        self._reset_success = False
        self._reset_command_pub.publish(Empty())
        timeout_s = float(self.get_parameter('reset_timeout_s').value)
        if not self._reset_event.wait(timeout=timeout_s):
            response.success = False
            response.message = 'Isaac reset acknowledgement timed out'
            self._publish_event('reset_timeout', response.message)
            return response
        response.success = self._reset_success
        response.message = (
            'Isaac scene reset complete'
            if self._reset_success
            else 'Isaac scene reset failed'
        )
        if self._reset_success:
            self._effort.complete_reset()
            self._publish_event(
                'reset_completed',
                'awaiting first post-reset state before accepting commands',
            )
        else:
            self._publish_event('reset_failed', response.message)
        return response

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name != 'target_object_name':
                continue
            if str(parameter.value) not in SUPPORTED_TARGET_OBJECTS:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        'P3 Isaac scene only supports target object '
                        f'{SUPPORTED_TARGET_OBJECTS}'
                    ),
                )
            self._target_object = str(parameter.value)
        return SetParametersResult(successful=True)

    def _publish_target_name(self) -> None:
        self._target_pub.publish(String(data=self._target_object))

    def _publish_status(self) -> None:
        now = time.monotonic()
        command_health = self._effort.snapshot(now)

        def age(last_seen: float) -> str:
            return 'never' if last_seen == 0.0 else f'{now - last_seen:.3f}'

        joint_live = self._last_joint_time > 0.0 and now - self._last_joint_time < 2.0
        object_live = (
            self._last_object_time > 0.0 and now - self._last_object_time < 2.0
        )
        camera_live = (
            self._last_camera_time > 0.0 and now - self._last_camera_time < 2.0
        )
        wrist_camera_live = (
            self._last_wrist_camera_time > 0.0
            and now - self._last_wrist_camera_time < 2.0
        )
        ee_live = self._last_ee_time > 0.0 and now - self._last_ee_time < 2.0
        ft_live = self._last_ft_time > 0.0 and now - self._last_ft_time < 2.0
        command_status = str(command_health['status'])
        command_safe = command_status in {'active', 'no_command'}
        status = DiagnosticStatus(
            level=(
                DiagnosticStatus.OK
                if joint_live and command_safe
                else DiagnosticStatus.ERROR
            ),
            name='isaac_sim_adapter/e1_action_execution',
            message=(
                'action execution ready'
                if joint_live and command_safe
                else f'not ready: joint_live={joint_live}, command={command_status}'
            ),
            hardware_id='isaac_sim_external',
            values=[
                KeyValue(key='joint_state', value=str(joint_live).lower()),
                KeyValue(key='joint_effort_execution', value='true'),
                KeyValue(key='command_qos', value='sensor_data'),
                KeyValue(key='state_qos', value='sensor_data'),
                KeyValue(key='command_status', value=command_status),
                KeyValue(
                    key='command_age_s',
                    value=str(command_health['command_age_s']),
                ),
                KeyValue(
                    key='state_age_s', value=str(command_health['state_age_s'])
                ),
                KeyValue(
                    key='watchdog_event_count',
                    value=str(self._watchdog_event_count),
                ),
                KeyValue(
                    key='reset_in_progress',
                    value=str(command_health['reset_in_progress']).lower(),
                ),
                KeyValue(
                    key='callback_isolation',
                    value='control|reset|sensor|camera|status',
                ),
                KeyValue(key='object_pose', value=str(object_live).lower()),
                KeyValue(key='scene_camera', value=str(camera_live).lower()),
                KeyValue(key='reset_scene', value='adapter_available'),
                KeyValue(key='tf', value='derived_by_robot_state_publisher'),
                KeyValue(key='ee_pose', value=str(ee_live).lower()),
                KeyValue(key='force_torque', value=str(ft_live).lower()),
                KeyValue(
                    key='force_torque_semantics',
                    value='panda_hand_incoming_joint_reaction_local_frame',
                ),
                KeyValue(key='wrist_camera', value=str(wrist_camera_live).lower()),
                KeyValue(key='joint_age_s', value=age(self._last_joint_time)),
                KeyValue(key='object_age_s', value=age(self._last_object_time)),
                KeyValue(key='camera_age_s', value=age(self._last_camera_time)),
                KeyValue(
                    key='wrist_camera_age_s',
                    value=age(self._last_wrist_camera_time),
                ),
                KeyValue(key='ee_age_s', value=age(self._last_ee_time)),
                KeyValue(key='ft_age_s', value=age(self._last_ft_time)),
            ],
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_pub.publish(array)

    def _check_startup_timeout(self) -> None:
        timeout_s = float(self.get_parameter('startup_timeout_s').value)
        if timeout_s <= 0.0 or self._last_joint_time > 0.0:
            return
        if time.monotonic() - self._started_at <= timeout_s:
            return
        self.get_logger().error(
            f'No {self._source}/joint_states received within {timeout_s:.1f}s'
        )
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacSimAdapter()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
