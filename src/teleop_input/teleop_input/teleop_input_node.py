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

from teleop_input.driver_base import TeleopDriverBase
from teleop_input.keyboard_driver import KeyboardDriver
from teleop_input.spacemouse_driver import SpaceMouseDriver
from teleop_input.gamepad_driver import GamepadDriver


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


def _rpy_from_quaternion(x, y, z, w):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TeleopInputNode(Node):
    def __init__(self):
        super().__init__("teleop_input")
        self.declare_parameter("driver_type", "keyboard")
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("cmd_rate", 100.0)
        self.declare_parameter("heartbeat_rate", 50.0)
        self.declare_parameter("home_position", [0.4, 0.0, 0.5])
        self.declare_parameter("position_step_m", 0.005)
        self.declare_parameter("rotation_step_rad", math.radians(3.0))
        self.declare_parameter("min_valid_ee_distance_m", 0.05)

        self.driver_type = self.get_parameter("driver_type").value.lower()
        self.base_frame = self.get_parameter("base_frame").value
        self.position_step_m = float(self.get_parameter("position_step_m").value)
        self.rotation_step_rad = float(self.get_parameter("rotation_step_rad").value)
        self.min_valid_ee_distance_m = float(
            self.get_parameter("min_valid_ee_distance_m").value)

        self.pub_pose = self.create_publisher(PoseStamped, "/teleop/cmd_pose", 10)
        self.pub_hb = self.create_publisher(Header, "/teleop/heartbeat", 10)
        self.pub_grip = self.create_publisher(Float64, "/teleop/gripper_cmd", 10)
        self.pub_rec = self.create_publisher(String, "/teleop/record_trigger", 10)

        # M4 temporary bypass
        self.declare_parameter("bypass_safety", False)
        self._bypass_safety = self.get_parameter("bypass_safety").value
        if self._bypass_safety:
            self.pub_safe = self.create_publisher(PoseStamped, "/safe_master_pose", 10)
            self.get_logger().warn(
                "bypass_safety=True: publishing /safe_master_pose directly "
                "(M4 stage — disable once M5 safety_monitor is active).")
        else:
            self.pub_safe = None

        self.home_position = list(self.get_parameter("home_position").value)
        self.position = list(self.home_position)
        self.rpy = [0.0, 0.0, 0.0]

        # Automatic alignment to current ee_pose to avoid singularity on startup
        self._aligned = False
        self._is_commanding = False
        self._warned_invalid_ee_pose = False
        self.sub_ee = self.create_subscription(
            PoseStamped, "/ee_pose", self._on_ee_pose_feedback, 10
        )
        self._alignment_timer = self.create_timer(2.0, self._on_alignment_timeout)

        # Load driver
        self.driver: TeleopDriverBase = None
        if self.driver_type == "keyboard":
            self.driver = KeyboardDriver(self.position_step_m, self.rotation_step_rad)
        elif self.driver_type == "spacemouse":
            self.driver = SpaceMouseDriver(self.position_step_m, self.rotation_step_rad)
        elif self.driver_type == "gamepad":
            self.driver = GamepadDriver(self.position_step_m, self.rotation_step_rad)
        else:
            self.get_logger().error(f"Unknown driver_type: {self.driver_type}")
            raise ValueError(f"Unknown driver_type: {self.driver_type}")

        if self.driver.initialize():
            detail = getattr(self.driver, "last_error", "")
            suffix = f" ({detail})" if detail else ""
            self.get_logger().info(
                f"teleop_input driver '{self.driver_type}' ready{suffix}.")
        else:
            detail = getattr(self.driver, "last_error", "")
            suffix = f": {detail}" if detail else "."
            self.get_logger().warn(
                f"Driver '{self.driver_type}' failed to initialize{suffix}")

        cmd_rate = self.get_parameter("cmd_rate").value
        hb_rate = self.get_parameter("heartbeat_rate").value
        self.create_timer(1.0 / cmd_rate, self._on_cmd_timer)
        self.create_timer(1.0 / hb_rate, self._on_heartbeat)

    def _on_alignment_timeout(self):
        if not self._aligned:
            self._aligned = True
            self.get_logger().warn("No ee_pose feedback received within 2.0s, falling back to default home_position.")
            if self._alignment_timer:
                self._alignment_timer.cancel()

    def _on_ee_pose_feedback(self, msg: PoseStamped):
        if not self._is_valid_ee_pose(msg):
            if not self._warned_invalid_ee_pose:
                self.get_logger().warn(
                    "Ignoring invalid ee_pose feedback; waiting for a finite "
                    "non-origin end-effector pose.")
                self._warned_invalid_ee_pose = True
            return

        # Continuous alignment: if not yet aligned, or if user is NOT currently pushing the joystick,
        # we align our setpoint to the robot's actual current pose to prevent drift / sudden jumps.
        if not self._aligned or not getattr(self, "_is_commanding", False):
            self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            o = msg.pose.orientation
            self.rpy = list(_rpy_from_quaternion(o.x, o.y, o.z, o.w))
            self.last_q = (o.x, o.y, o.z, o.w)
            self._aligned = True
            if self._alignment_timer:
                self._alignment_timer.cancel()

    def _is_valid_ee_pose(self, msg: PoseStamped) -> bool:
        p = msg.pose.position
        o = msg.pose.orientation
        values = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        if not all(math.isfinite(float(v)) for v in values):
            return False
        quat_norm = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
        if quat_norm < 0.5:
            return False
        distance = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        return distance >= self.min_valid_ee_distance_m

    def _on_cmd_timer(self):
        # Update pose
        pos_delta, rpy_delta = self.driver.get_pose_delta()

        # Check if there is active joystick input
        if any(d != 0.0 for d in pos_delta) or any(d != 0.0 for d in rpy_delta):
            self._is_commanding = True
        else:
            self._is_commanding = False

        self.position[0] += pos_delta[0]
        self.position[1] += pos_delta[1]
        self.position[2] += pos_delta[2]
        self.rpy[0] += rpy_delta[0]
        self.rpy[1] += rpy_delta[1]
        self.rpy[2] += rpy_delta[2]

        # Check reset
        if self.driver.get_reset_trigger():
            self.position = list(self.home_position)
            self.rpy = [0.0, 0.0, 0.0]

        # Check gripper
        gripper_cmd = self.driver.get_gripper_cmd()
        if gripper_cmd is not None:
            self.pub_grip.publish(Float64(data=float(gripper_cmd)))

        # Check record trigger
        rec_cmd = self.driver.get_record_trigger()
        if rec_cmd is not None:
            self.pub_rec.publish(String(data=rec_cmd))

        # Publish pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(self.position[0])
        msg.pose.position.y = float(self.position[1])
        msg.pose.position.z = float(self.position[2])
        qx, qy, qz, qw = _quaternion_from_rpy(*self.rpy)

        # Ensure quaternion continuity (shortest path)
        if getattr(self, "last_q", None) is not None:
            dot = qx*self.last_q[0] + qy*self.last_q[1] + qz*self.last_q[2] + qw*self.last_q[3]
            if dot < 0:
                qx, qy, qz, qw = -qx, -qy, -qz, -qw
        self.last_q = (qx, qy, qz, qw)

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub_pose.publish(msg)
        if self.pub_safe is not None:
            self.pub_safe.publish(msg)

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
        node.driver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
