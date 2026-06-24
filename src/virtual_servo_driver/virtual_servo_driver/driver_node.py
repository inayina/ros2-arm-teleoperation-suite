#!/usr/bin/env python3
"""L4 virtual servo driver node.

Simulates 7 CANopen DS402 drives on vcan0. Bridges the fieldbus to MuJoCo:

  vcan0 RPDO ──▶ torque ──▶ /sim/joint_effort_cmd ──▶ mujoco_sim
  mujoco_sim ──▶ /sim/encoder_state ──▶ encoder ──▶ vcan0 TPDO

Publishes /servo_drive/status (teleop_interfaces/DriveStatusArray).
"""
import struct

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from teleop_interfaces.msg import DriveStatus, DriveStatusArray

from .ds402_state_machine import DS402StateMachine, State
from .current_loop import CurrentLoop
from . import pdo_codec
from .sdo_server import ObjectDictionary, SdoServer

try:
    import can
    _HAS_CAN = True
except Exception:
    _HAS_CAN = False

COB_RPDO1_BASE = 0x200
COB_TPDO1_BASE = 0x180
COB_TPDO2_BASE = 0x280
COB_SDO_RX_BASE = 0x600
COB_SDO_TX_BASE = 0x580
COB_NMT = 0x000
COB_SYNC = 0x080
COB_EMCY_BASE = 0x080
COB_HEARTBEAT_BASE = 0x700


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
        self.od = [ObjectDictionary() for _ in range(self.n)]
        self.sdo = [
            SdoServer(
                self.node_ids[i],
                self.od[i],
                on_controlword=lambda cw, idx=i: self._on_controlword(idx, cw),
                on_max_torque=lambda val, idx=i: self._on_max_torque(idx, val),
            )
            for i in range(self.n)
        ]

        self.q = np.zeros(self.n)
        self.qd = np.zeros(self.n)
        self.tau_out = np.zeros(self.n)
        self._sync_count = 0

        self.bus = self._open_can()
        self._notifier = None
        if self.bus is not None:
            self._notifier = can.Notifier(self.bus, [self._on_can_message])

        self.sub_enc = self.create_subscription(
            JointState, "/sim/encoder_state", self._on_encoder, qos_profile_sensor_data)
        self.pub_effort = self.create_publisher(
            Float64MultiArray, "/sim/joint_effort_cmd", qos_profile_sensor_data)
        self.pub_status = self.create_publisher(
            DriveStatusArray, "/servo_drive/status", 10)

        status_rate = self.get_parameter("status_rate").value
        self.create_timer(1.0 / status_rate, self._publish_status)
        if self.bus is None:
            self.create_timer(0.001, self._on_sync)

        self.create_service(Trigger, "inject_fault_joint1", self._inject_fault_joint1)

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

    def _send_can(self, cob_id: int, data: bytes):
        if self.bus is None:
            return
        msg = can.Message(arbitration_id=cob_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception as e:
            self.get_logger().warn(f"CAN send failed cob=0x{cob_id:03X}: {e}")

    def _on_can_message(self, msg: can.Message):
        cob_id = msg.arbitration_id
        if cob_id == COB_NMT:
            self._handle_nmt(bytes(msg.data))
            return
        if cob_id == COB_SYNC:
            self._on_sync()
            return

        for i, node_id in enumerate(self.node_ids):
            if cob_id == COB_RPDO1_BASE + node_id:
                torque = pdo_codec.unpack_rpdo_torque(bytes(msg.data))
                self.loops[i].set_target(torque)
                self.od[i].write(0x6071, int(torque / pdo_codec.TORQUE_SCALE))
            elif cob_id == COB_SDO_RX_BASE + node_id:
                response = self.sdo[i].handle(bytes(msg.data))
                if response:
                    self._send_can(COB_SDO_TX_BASE + node_id, response)

    def _handle_nmt(self, data: bytes):
        if not data:
            return
        if data[0] == 0x01:
            for sm in self.sm:
                if sm.state == State.NOT_READY_TO_SWITCH_ON:
                    sm.state = State.SWITCH_ON_DISABLED

    def _on_controlword(self, idx: int, cw: int):
        self.od[idx].write(0x6040, cw)
        self.sm[idx].apply_controlword(cw)
        self.od[idx].set_statusword(self.sm[idx].statusword)

    def _on_max_torque(self, idx: int, raw: int):
        self.loops[idx].max_torque = max(0.1, raw * pdo_codec.TORQUE_SCALE)

    def _on_encoder(self, msg: JointState):
        n = min(self.n, len(msg.position))
        self.q[:n] = msg.position[:n]
        if len(msg.velocity) >= n:
            self.qd[:n] = msg.velocity[:n]

    def _on_sync(self):
        if self.auto_enable and self.bus is None:
            for sm in self.sm:
                if sm.state not in (State.OPERATION_ENABLED, State.FAULT):
                    sm.apply_controlword(0x06)
                    sm.apply_controlword(0x07)
                    sm.apply_controlword(0x0F)

        cmd = Float64MultiArray()
        cmd.data = [0.0] * self.n

        for i in range(self.n):
            enabled = self.sm[i].torque_enabled()
            if self.sm[i].state in (State.QUICK_STOP_ACTIVE, State.FAULT):
                tau = self.loops[i].ramp_to_zero()
                fault = 0
            else:
                tau, fault = self.loops[i].update(0.001, enabled, float(self.qd[i]))

            if fault:
                self._raise_fault(i, fault)
                tau = 0.0

            self.tau_out[i] = tau
            cmd.data[i] = tau

            pos_cnt = pdo_codec.rad_to_counts(float(self.q[i]))
            vel_raw = int(round(float(self.qd[i]) / pdo_codec.VELOCITY_SCALE))
            self.od[i].set_feedback(pos_cnt, vel_raw, int(round(tau / pdo_codec.TORQUE_SCALE)))
            self.od[i].set_statusword(self.sm[i].statusword)

            node_id = self.node_ids[i]
            self._send_can(
                COB_TPDO1_BASE + node_id,
                pdo_codec.pack_tpdo1_position(float(self.q[i]), float(self.qd[i])))
            self._send_can(
                COB_TPDO2_BASE + node_id,
                pdo_codec.pack_tpdo2_status(self.sm[i].statusword, tau))

        self.pub_effort.publish(cmd)

        self._sync_count += 1
        if self._sync_count >= 1000:
            self._sync_count = 0
            for i, node_id in enumerate(self.node_ids):
                self._send_can(COB_HEARTBEAT_BASE + node_id, bytes([0x05]))

    def _raise_fault(self, idx: int, fault_code: int):
        if self.sm[idx].state == State.FAULT:
            return
        self.sm[idx].fault(fault_code)
        self.od[idx].set_statusword(self.sm[idx].statusword)
        emcy = struct.pack("<HBB", fault_code & 0xFFFF, 0x00, 0x00) + b"\x00" * 3
        self._send_can(COB_EMCY_BASE + self.node_ids[idx], emcy)

    def inject_fault(self, joint_index: int, fault_code: int = 0x3210):
        """Inject a fault on the selected drive (test hook)."""
        if 0 <= joint_index < self.n:
            self.loops[joint_index].inject_fault(fault_code)

    def _inject_fault_joint1(self, _request, response):
        self.inject_fault(0, 0x3210)
        response.success = True
        response.message = "Injected overcurrent fault (0x3210) on joint 1"
        return response

    def _publish_status(self):
        arr = DriveStatusArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.n):
            ds = DriveStatus()
            ds.header.stamp = arr.header.stamp
            ds.node_id = int(self.node_ids[i]) if i < len(self.node_ids) else i + 1
            ds.statusword = self.sm[i].statusword
            ds.controlword = int(self.od[i].read(0x6040) or 0)
            ds.ds402_state = int(self.sm[i].state)
            ds.fault_code = self.sm[i].fault_code
            ds.mode_of_operation = int(self.od[i].read(0x6060) or 10)
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
