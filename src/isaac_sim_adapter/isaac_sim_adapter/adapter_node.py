"""Translate isolated Isaac topics into the upstream simulation contract."""

from __future__ import annotations

import threading
import time
from typing import Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Empty, Float64, String
from std_srvs.srv import Trigger


PANDA_ARM_JOINTS = tuple(f'panda_joint{i}' for i in range(1, 8))
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


class IsaacSimAdapter(Node):
    """Expose P2/P3 Isaac capabilities through the existing ROS contract."""

    def __init__(self) -> None:
        super().__init__('isaac_sim_adapter')
        self.declare_parameter('source_namespace', '/isaac')
        self.declare_parameter('target_object_name', 'object_red_box')
        self.declare_parameter('startup_timeout_s', 45.0)
        self.declare_parameter('reset_timeout_s', 5.0)

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

        self._callback_group = ReentrantCallbackGroup()
        self._last_joint_time = 0.0
        self._last_object_time = 0.0
        self._last_camera_time = 0.0
        self._last_ee_time = 0.0
        self._last_ft_time = 0.0
        self._reset_event = threading.Event()
        self._reset_success = False

        self._encoder_pub = self.create_publisher(
            JointState, '/sim/encoder_state', 10
        )
        self._object_pub = self.create_publisher(
            PoseStamped, '/sim/object_pose', 10
        )
        self._camera_pub = self.create_publisher(
            Image, '/camera/scene/image_raw', 5
        )
        self._recorder_camera_pub = self.create_publisher(
            Image, '/camera/color/image_raw', 5
        )
        self._ee_pub = self.create_publisher(PoseStamped, '/ee_pose', 10)
        self._ft_pub = self.create_publisher(WrenchStamped, '/ft_sensor', 10)
        self._gripper_pub = self.create_publisher(Float64, '/gripper/state', 10)
        self._status_pub = self.create_publisher(
            DiagnosticArray, '/sim/backend_status', 10
        )
        self._target_pub = self.create_publisher(
            String, f'{self._source}/target_object_name', 1
        )
        self._reset_command_pub = self.create_publisher(
            Empty, f'{self._source}/reset_scene_cmd', 1
        )

        self.create_subscription(
            JointState,
            f'{self._source}/joint_states',
            self._on_joint_state,
            10,
        )
        self.create_subscription(
            PoseStamped,
            f'{self._source}/object_pose',
            self._on_object_pose,
            10,
        )
        self.create_subscription(
            PoseStamped,
            f'{self._source}/ee_pose',
            self._on_ee_pose,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            f'{self._source}/ft_sensor',
            self._on_ft_sensor,
            10,
        )
        self.create_subscription(
            Image,
            f'{self._source}/camera/color/image_raw',
            self._on_camera,
            5,
        )
        self.create_subscription(
            Bool,
            f'{self._source}/reset_scene_done',
            self._on_reset_done,
            1,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            '/sim/reset_scene',
            self._reset_scene,
            callback_group=self._callback_group,
        )
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(1.0, self._publish_status)
        self.create_timer(1.0, self._publish_target_name)
        self._started_at = time.monotonic()
        self.create_timer(0.5, self._check_startup_timeout)

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

    def _on_ee_pose(self, message: PoseStamped) -> None:
        self._last_ee_time = time.monotonic()
        # The P3 scene anchors Franka at the world origin, so world and
        # panda_link0 coincide. Publish the existing canonical EE frame.
        message.header.frame_id = 'panda_link0'
        self._ee_pub.publish(message)

    def _on_ft_sensor(self, message: WrenchStamped) -> None:
        self._last_ft_time = time.monotonic()
        self._ft_pub.publish(message)

    def _on_reset_done(self, message: Bool) -> None:
        self._reset_success = bool(message.data)
        self._reset_event.set()

    def _reset_scene(self, _request, response):
        self._reset_event.clear()
        self._reset_success = False
        self._reset_command_pub.publish(Empty())
        timeout_s = float(self.get_parameter('reset_timeout_s').value)
        if not self._reset_event.wait(timeout=timeout_s):
            response.success = False
            response.message = 'Isaac reset acknowledgement timed out'
            return response
        response.success = self._reset_success
        response.message = (
            'Isaac scene reset complete'
            if self._reset_success
            else 'Isaac scene reset failed'
        )
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

        def age(last_seen: float) -> str:
            return 'never' if last_seen == 0.0 else f'{now - last_seen:.3f}'

        joint_live = self._last_joint_time > 0.0 and now - self._last_joint_time < 2.0
        object_live = (
            self._last_object_time > 0.0 and now - self._last_object_time < 2.0
        )
        camera_live = (
            self._last_camera_time > 0.0 and now - self._last_camera_time < 2.0
        )
        ee_live = self._last_ee_time > 0.0 and now - self._last_ee_time < 2.0
        ft_live = self._last_ft_time > 0.0 and now - self._last_ft_time < 2.0
        status = DiagnosticStatus(
            level=(DiagnosticStatus.OK if joint_live else DiagnosticStatus.ERROR),
            name='isaac_sim_adapter/p3_capabilities',
            message=('joint stream active' if joint_live else 'joint stream unavailable'),
            hardware_id='isaac_sim_external',
            values=[
                KeyValue(key='joint_state', value=str(joint_live).lower()),
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
                KeyValue(key='wrist_camera', value='false'),
                KeyValue(key='joint_age_s', value=age(self._last_joint_time)),
                KeyValue(key='object_age_s', value=age(self._last_object_time)),
                KeyValue(key='camera_age_s', value=age(self._last_camera_time)),
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
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
