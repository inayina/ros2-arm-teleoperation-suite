#!/usr/bin/env python3
"""L0 teleop input node.

Publishes the master command and a heartbeat consumed by the safety layer:
  /teleop/cmd_pose       geometry_msgs/PoseStamped  @100 Hz
  /teleop/heartbeat      std_msgs/Header            @50 Hz
  /teleop/gripper_cmd    std_msgs/Float64           (event)
  /teleop/record_trigger std_msgs/String            (event)

SCAFFOLD: emits a slow demo pose. Replace `_update_target` with real keyboard /
gamepad / Quest3 input (pynput / inputs) in M4. The safety layer is the only
consumer of /teleop/cmd_pose -- never publish /safe_master_pose from here.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Header, String


class TeleopInputNode(Node):
    def __init__(self):
        super().__init__("teleop_input")
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("cmd_rate", 100.0)
        self.declare_parameter("heartbeat_rate", 50.0)
        self.base_frame = self.get_parameter("base_frame").value

        self.pub_pose = self.create_publisher(PoseStamped, "/teleop/cmd_pose", 10)
        self.pub_hb = self.create_publisher(Header, "/teleop/heartbeat", 10)
        self.pub_grip = self.create_publisher(Float64, "/teleop/gripper_cmd", 10)
        self.pub_rec = self.create_publisher(String, "/teleop/record_trigger", 10)

        cmd_rate = self.get_parameter("cmd_rate").value
        hb_rate = self.get_parameter("heartbeat_rate").value
        self.create_timer(1.0 / cmd_rate, self._on_cmd_timer)
        self.create_timer(1.0 / hb_rate, self._on_heartbeat)

        # Home pose (demo)
        self.home = (0.4, 0.0, 0.5)
        self.t0 = self.get_clock().now()
        self.get_logger().info("teleop_input up (scaffold demo motion).")

    def _update_target(self) -> tuple:
        """Return target (x, y, z). SCAFFOLD: small circle around home."""
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        x, y, z = self.home
        return (x, y + 0.05 * math.sin(0.5 * t), z + 0.05 * math.cos(0.5 * t))

    def _on_cmd_timer(self):
        x, y, z = self._update_target()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pub_pose.publish(msg)

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
