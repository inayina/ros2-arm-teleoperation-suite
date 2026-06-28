#!/usr/bin/env python3
"""Publish a steady teleop heartbeat for scripted demos."""

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class DummyHeartbeat(Node):
    def __init__(self, rate_hz: float):
        super().__init__("dummy_heartbeat")
        self.pub = self.create_publisher(Header, "/teleop/heartbeat", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(f"Publishing /teleop/heartbeat at {rate_hz:.1f} Hz.")

    def _publish(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = "m7_dummy_heartbeat"
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=50.0)
    args = parser.parse_args()

    rclpy.init()
    node = DummyHeartbeat(args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
