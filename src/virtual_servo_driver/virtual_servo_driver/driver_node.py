#!/usr/bin/env python3
"""L4 virtual servo driver node.

Simulates N CANopen DS402 drives. In CAN mode it bridges the bus to MuJoCo:
  vcan0 RPDO ──▶ torque ──▶ /sim/joint_effort_cmd ──▶ mujoco_sim
  mujoco_sim ──▶ /sim/encoder_state ──▶ encoder ──▶ vcan0 TPDO

Publishes /servo_drive/status (teleop_interfaces/DriveStatusArray) for tooling.

SCAFFOLD: python-can / vcan0 are optional. Without them the node still runs the
DS402 state machines and publishes status from /sim/encoder_state, so the rest
of the stack is testable. M2 task: wire real SocketCAN PDO/SDO/SYNC/EMCY.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from teleop_interfaces.msg import DriveStatus, DriveStatusArray

from .ds402_state_machine import DS402StateMachine, State
from .current_loop import CurrentLoop

try:
    import can  # python-can
    _HAS_CAN = True
except Exception:
    _HAS_CAN = False


class VirtualServoDriver(Node):
    def __init__(self):
        super().__init__("virtual_servo_driver")
        self.declare_parameter("num_joints", 7)
        self.declare_parameter("node_ids", [1, 2, 3, 4, 5, 6, 7])
        self.declare_parameter("can_interface", "vcan0")
        self.declare_parameter("status_rate", 50.0)
        self.declare_parameter("auto_enable", True)

        self.n = int(self.get_parameter("num_joints").value)
        self.node_ids = list(self.get_parameter("node_ids").value)[: self.n]
        self.can_iface = self.get_parameter("can_interface").value
        self.auto_enable = bool(self.get_parameter("auto_enable").value)

        self.sm = [DS402StateMachine() for _ in range(self.n)]
        self.loops = [CurrentLoop() for _ in range(self.n)]
        self.q = np.zeros(self.n)
        self.qd = np.zeros(self.n)
        self.tau_out = np.zeros(self.n)

        self.bus = self._open_can()

        self.sub_enc = self.create_subscription(
            JointState, "/sim/encoder_state", self._on_encoder, qos_profile_sensor_data)
        self.pub_effort = self.create_publisher(
            Float64MultiArray, "/sim/joint_effort_cmd", qos_profile_sensor_data)
        self.pub_status = self.create_publisher(
            DriveStatusArray, "/servo_drive/status", 10)

        status_rate = self.get_parameter("status_rate").value
        self.create_timer(1.0 / status_rate, self._publish_status)
        self.create_timer(0.001, self._control_step)  # 1 kHz drive loop

        mode = "CAN" if self.bus else "no-CAN (degraded)"
        self.get_logger().info(
            f"virtual_servo_driver up: {self.n} drives, {mode}, iface={self.can_iface}.")

    def _open_can(self):
        if not _HAS_CAN:
            self.get_logger().warn("python-can not available; running without bus.")
            return None
        try:
            return can.Bus(channel=self.can_iface, interface="socketcan")
        except Exception as e:
            self.get_logger().warn(f"Cannot open {self.can_iface} ({e}); degraded mode.")
            return None

    def _on_encoder(self, msg: JointState):
        n = min(self.n, len(msg.position))
        self.q[:n] = msg.position[:n]
        if len(msg.velocity) >= n:
            self.qd[:n] = msg.velocity[:n]

    def _control_step(self):
        # SCAFFOLD: auto-bring each drive to Operation Enabled.
        # M2: replace with controlword received via RPDO.
        for i, sm in enumerate(self.sm):
            if self.auto_enable and sm.state not in (State.OPERATION_ENABLED, State.FAULT):
                sm.apply_controlword(0x06)
                sm.apply_controlword(0x07)
                sm.apply_controlword(0x0F)

        cmd = Float64MultiArray()
        cmd.data = [0.0] * self.n
        for i in range(self.n):
            enabled = self.sm[i].torque_enabled()
            tau, fault = self.loops[i].update(0.001, enabled, float(self.qd[i]))
            if fault:
                self.sm[i].fault(fault)
            self.tau_out[i] = tau
            cmd.data[i] = tau
        # TODO(M2): instead of publishing directly, this torque is the result of
        # decoding RPDO; encoder feedback below becomes TPDO frames on the bus.
        self.pub_effort.publish(cmd)

    def _publish_status(self):
        arr = DriveStatusArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.n):
            ds = DriveStatus()
            ds.header.stamp = arr.header.stamp
            ds.node_id = int(self.node_ids[i]) if i < len(self.node_ids) else i + 1
            ds.statusword = self.sm[i].statusword
            ds.ds402_state = int(self.sm[i].state)
            ds.fault_code = self.sm[i].fault_code
            ds.mode_of_operation = 10
            ds.actual_position = float(self.q[i])
            ds.actual_velocity = float(self.qd[i])
            ds.actual_torque = float(self.tau_out[i])
            ds.target_torque = float(self.loops[i].tau_cmd)
            arr.drives.append(ds)
        self.pub_status.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualServoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
