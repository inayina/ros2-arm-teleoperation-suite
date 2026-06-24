#!/usr/bin/env python3
"""L0 teleop input node.

Publishes the master command and a heartbeat consumed by the safety layer:
  /teleop/cmd_pose       geometry_msgs/PoseStamped  @100 Hz
  /teleop/heartbeat      std_msgs/Header            @50 Hz
  /teleop/gripper_cmd    std_msgs/Float64           (event)
  /teleop/record_trigger std_msgs/String            (event)

Keyboard controls:
  W/S: +/-X, A/D: +/-Y, Q/E: +/-Z, I/K/J/L/U/O: RPY rotations,
  G: toggle gripper, R/T: record start/stop, Space: home.

M4 temporary bypass:
  When ``bypass_safety`` parameter is True, this node also publishes to
  /safe_master_pose (debug only). Default False: safety_monitor is the sole
  publisher. M5 enables full auto E-Stop via ``auto_estop_enabled``.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Header, String

from teleop_input.keyboard_reader import KeyboardReader


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class TeleopInputNode(Node):
    def __init__(self):
        super().__init__("teleop_input")
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("cmd_rate", 100.0)
        self.declare_parameter("heartbeat_rate", 50.0)
        self.declare_parameter("home_position", [0.4, 0.0, 0.5])
        self.declare_parameter("position_step_m", 0.005)
        self.declare_parameter("rotation_step_rad", math.radians(3.0))
        self.base_frame = self.get_parameter("base_frame").value
        self.position_step_m = float(self.get_parameter("position_step_m").value)
        self.rotation_step_rad = float(self.get_parameter("rotation_step_rad").value)

        self.pub_pose = self.create_publisher(PoseStamped, "/teleop/cmd_pose", 10)
        self.pub_hb = self.create_publisher(Header, "/teleop/heartbeat", 10)
        self.pub_grip = self.create_publisher(Float64, "/teleop/gripper_cmd", 10)
        self.pub_rec = self.create_publisher(String, "/teleop/record_trigger", 10)

        # M4 temporary bypass: publish /safe_master_pose directly so that
        # servo_node has an input before the M5 safety layer is wired up.
        # Set bypass_safety:=false once safety_monitor is active (M5+).
        self.declare_parameter("bypass_safety", False)
        self._bypass_safety = self.get_parameter("bypass_safety").value
        if self._bypass_safety:
            self.pub_safe = self.create_publisher(PoseStamped, "/safe_master_pose", 10)
            self.get_logger().warn(
                "bypass_safety=True: publishing /safe_master_pose directly "
                "(M4 stage — disable once M5 safety_monitor is active).")
        else:
            self.pub_safe = None

        cmd_rate = self.get_parameter("cmd_rate").value
        hb_rate = self.get_parameter("heartbeat_rate").value
        self.create_timer(1.0 / cmd_rate, self._on_cmd_timer)
        self.create_timer(1.0 / hb_rate, self._on_heartbeat)

        self.home_position = list(self.get_parameter("home_position").value)
        self.position = list(self.home_position)
        self.rpy = [0.0, 0.0, 0.0]
        self.gripper_open = False
        self.keyboard = KeyboardReader()

        if self.keyboard.enabled:
            self.get_logger().info(
                "teleop_input keyboard ready: W/S X, A/D Y, Q/E Z, Space home.")
        else:
            self.get_logger().warn(
                "stdin is not a TTY; publishing the home pose until interactive input is available.")

    def _apply_key(self, key: str):
        key_lower = key.lower()
        linear = {
            "w": (0, self.position_step_m),
            "s": (0, -self.position_step_m),
            "a": (1, self.position_step_m),
            "d": (1, -self.position_step_m),
            "q": (2, self.position_step_m),
            "e": (2, -self.position_step_m),
        }
        angular = {
            "i": (0, self.rotation_step_rad),
            "k": (0, -self.rotation_step_rad),
            "j": (1, self.rotation_step_rad),
            "l": (1, -self.rotation_step_rad),
            "u": (2, self.rotation_step_rad),
            "o": (2, -self.rotation_step_rad),
        }

        if key_lower in linear:
            axis, delta = linear[key_lower]
            self.position[axis] += delta
        elif key_lower in angular:
            axis, delta = angular[key_lower]
            self.rpy[axis] += delta
        elif key == " ":
            self.position = list(self.home_position)
            self.rpy = [0.0, 0.0, 0.0]
        elif key_lower == "g":
            self.gripper_open = not self.gripper_open
            self.pub_grip.publish(Float64(data=1.0 if self.gripper_open else 0.0))
        elif key_lower in ("r", "t"):
            self.pub_rec.publish(String(data="start" if key_lower == "r" else "stop"))

    def _on_cmd_timer(self):
        key = self.keyboard.get_key()
        if key:
            self._apply_key(key)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(self.position[0])
        msg.pose.position.y = float(self.position[1])
        msg.pose.position.z = float(self.position[2])
        qx, qy, qz, qw = _quaternion_from_rpy(*self.rpy)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub_pose.publish(msg)
        if self.pub_safe is not None:
            self.pub_safe.publish(msg)   # M4 temporary bypass → servo_node

    def _on_heartbeat(self):
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = "teleop_input"
        self.pub_hb.publish(hb)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.keyboard.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
