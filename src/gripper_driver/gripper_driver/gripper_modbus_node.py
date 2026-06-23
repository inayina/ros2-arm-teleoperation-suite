#!/usr/bin/env python3
"""RS485 / Modbus gripper driver.

  in : /teleop/gripper_cmd  std_msgs/Float64  (0.0 closed .. 1.0 open)
  out: /gripper/state       std_msgs/Float64  (measured opening)

SCAFFOLD: models the gripper as a first-order system toward the command.
M2 task: back this with a pymodbus server (write reg 0x40, read reg 0x41).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GripperDriverNode(Node):
    def __init__(self):
        super().__init__("gripper_driver")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("speed", 2.0)  # 1/s

        self.cmd = 0.0
        self.state = 0.0
        self.speed = float(self.get_parameter("speed").value)
        rate = self.get_parameter("rate").value
        self.dt = 1.0 / rate

        self.sub = self.create_subscription(
            Float64, "/teleop/gripper_cmd", self._on_cmd, 10)
        self.pub = self.create_publisher(Float64, "/gripper/state", 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info("gripper_driver up (scaffold first-order model).")

    def _on_cmd(self, msg: Float64):
        self.cmd = max(0.0, min(1.0, msg.data))

    def _tick(self):
        err = self.cmd - self.state
        self.state += max(-self.speed * self.dt, min(self.speed * self.dt, err))
        m = Float64()
        m.data = self.state
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GripperDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
