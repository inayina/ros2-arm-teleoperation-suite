#!/usr/bin/env python3
"""L5 MuJoCo physics server.

Sim backplane (talks to canopen_hw_interface / virtual_servo_driver):
  in : /sim/joint_effort_cmd  std_msgs/Float64MultiArray  (applied torque)
  out: /sim/encoder_state     sensor_msgs/JointState      (ground-truth pos/vel/eff)

ROS-facing physics outputs:
  out: /ft_sensor             geometry_msgs/WrenchStamped
  out: /ee_pose               geometry_msgs/PoseStamped

If `mujoco` or the model XML is unavailable, the node falls back to a simple
per-joint integrator so the rest of the stack still runs. In M1 the default
path points at config/models/franka_panda.xml and maps controls/state by joint
and actuator name, matching the ros2_control interfaces.
"""
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray

try:
    import mujoco  # noqa: F401
    _HAS_MUJOCO = True
except Exception:  # pragma: no cover
    _HAS_MUJOCO = False

JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__("mujoco_sim")
        self.declare_parameter("model_path", "config/models/franka_panda.xml")
        self.declare_parameter("physics_rate", 1000.0)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("ee_site", "panda_ee")
        self.declare_parameter("gravity_compensation", True)
        self.declare_parameter(
            "initial_positions",
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        )

        self.physics_rate = self.get_parameter("physics_rate").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.base_frame = self.get_parameter("base_frame").value
        self.gravity_compensation = bool(self.get_parameter("gravity_compensation").value)
        self.n = len(JOINT_NAMES)

        # Sim state (fallback integrator).
        self.q = np.zeros(self.n)
        self.qd = np.zeros(self.n)
        self.tau = np.zeros(self.n)
        self.inertia = np.full(self.n, 0.5)
        self.damping = np.full(self.n, 2.0)

        self.model = None
        self.data = None
        self.joint_qposadr = []
        self.joint_dofadr = []
        self.actuator_ids = []
        self.ee_site_id = None
        self.force_sensor_id = None
        self.torque_sensor_id = None
        self._try_load_model()

        self.sub_effort = self.create_subscription(
            Float64MultiArray, "/sim/joint_effort_cmd", self._on_effort,
            qos_profile_sensor_data)
        self.pub_encoder = self.create_publisher(
            JointState, "/sim/encoder_state", qos_profile_sensor_data)
        self.pub_ft = self.create_publisher(WrenchStamped, "/ft_sensor", 10)
        self.pub_ee = self.create_publisher(PoseStamped, "/ee_pose", 10)

        self.create_timer(1.0 / self.physics_rate, self._step)
        self._pub_decim = max(1, int(self.physics_rate / self.publish_rate))
        self._k = 0

        mode = "MuJoCo" if self.model is not None else "fallback integrator"
        self.get_logger().info(f"mujoco_sim up ({mode}).")

    def _try_load_model(self):
        path = self.get_parameter("model_path").value
        if path and not os.path.isabs(path):
            path = os.path.abspath(path)
        if not (_HAS_MUJOCO and path):
            return
        try:
            import mujoco
            self.model = mujoco.MjModel.from_xml_path(path)
            self.data = mujoco.MjData(self.model)
            self._build_mujoco_name_maps(mujoco)
            self._set_initial_pose(mujoco)
            self.get_logger().info(f"Loaded MuJoCo model: {path}")
        except Exception as e:  # pragma: no cover
            self.get_logger().warn(f"MuJoCo model load failed ({e}); using fallback.")
            self.model = None

    def _build_mujoco_name_maps(self, mujoco):
        self.joint_qposadr = []
        self.joint_dofadr = []
        self.actuator_ids = []
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"MuJoCo joint '{name}' not found")
            self.joint_qposadr.append(int(self.model.jnt_qposadr[jid]))
            self.joint_dofadr.append(int(self.model.jnt_dofadr[jid]))

            motor_name = f"{name}_motor"
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_name)
            if aid < 0:
                raise RuntimeError(f"MuJoCo actuator '{motor_name}' not found")
            self.actuator_ids.append(aid)

        site_name = self.get_parameter("ee_site").value
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid >= 0:
            self.ee_site_id = sid

        self.force_sensor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "ee_force")
        self.torque_sensor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "ee_torque")

    def _set_initial_pose(self, mujoco):
        q0 = list(self.get_parameter("initial_positions").value)
        if len(q0) != self.n:
            self.get_logger().warn(
                f"initial_positions has {len(q0)} values, expected {self.n}; using zeros.")
            q0 = [0.0] * self.n
        for i, adr in enumerate(self.joint_qposadr):
            self.data.qpos[adr] = q0[i]
        mujoco.mj_forward(self.model, self.data)
        self.q = np.array([self.data.qpos[adr] for adr in self.joint_qposadr])
        self.qd = np.array([self.data.qvel[adr] for adr in self.joint_dofadr])

    def _on_effort(self, msg: Float64MultiArray):
        d = np.asarray(msg.data, dtype=float)
        if d.size >= self.n:
            self.tau = d[: self.n]

    def _step(self):
        dt = 1.0 / self.physics_rate
        if self.model is not None:
            import mujoco
            self.data.qfrc_applied[:] = 0.0
            if self.gravity_compensation:
                # MuJoCo's qfrc_bias is the generalized bias force (gravity +
                # Coriolis). Applying it makes zero effort hold the ready pose,
                # while ros2_control commands still arrive through actuators.
                for dof in self.joint_dofadr:
                    self.data.qfrc_applied[dof] = self.data.qfrc_bias[dof]
            for i, aid in enumerate(self.actuator_ids):
                self.data.ctrl[aid] = self.tau[i]
            mujoco.mj_step(self.model, self.data)
            self.q = np.array([self.data.qpos[adr] for adr in self.joint_qposadr])
            self.qd = np.array([self.data.qvel[adr] for adr in self.joint_dofadr])
        else:
            # Simple decoupled second-order integrator.
            qdd = (self.tau - self.damping * self.qd) / self.inertia
            self.qd += qdd * dt
            self.q += self.qd * dt

        self._k += 1
        if self._k % self._pub_decim == 0:
            self._publish()

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = JOINT_NAMES
        js.position = self.q.tolist()
        js.velocity = self.qd.tolist()
        js.effort = self.tau.tolist()
        self.pub_encoder.publish(js)

        ft = WrenchStamped()
        ft.header.stamp = stamp
        ft.header.frame_id = "panda_ee"
        if self.model is not None:
            wrench = self._read_ee_wrench()
            ft.wrench.force.x = float(wrench[0])
            ft.wrench.force.y = float(wrench[1])
            ft.wrench.force.z = float(wrench[2])
            ft.wrench.torque.x = float(wrench[3])
            ft.wrench.torque.y = float(wrench[4])
            ft.wrench.torque.z = float(wrench[5])
        self.pub_ft.publish(ft)  # TODO(M3): real contact wrench from MuJoCo sensors

        ee = PoseStamped()
        ee.header.stamp = stamp
        ee.header.frame_id = self.base_frame
        if self.model is not None and self.ee_site_id is not None:
            ee.pose.position.x = float(self.data.site_xpos[self.ee_site_id][0])
            ee.pose.position.y = float(self.data.site_xpos[self.ee_site_id][1])
            ee.pose.position.z = float(self.data.site_xpos[self.ee_site_id][2])
            quat = self._site_quat()
            ee.pose.orientation.x = float(quat[1])
            ee.pose.orientation.y = float(quat[2])
            ee.pose.orientation.z = float(quat[3])
            ee.pose.orientation.w = float(quat[0])
        else:
            ee.pose.orientation.w = 1.0
        self.pub_ee.publish(ee)

    def _read_sensor3(self, sensor_id):
        if sensor_id is None or sensor_id < 0:
            return np.zeros(3)
        adr = int(self.model.sensor_adr[sensor_id])
        dim = int(self.model.sensor_dim[sensor_id])
        if dim < 3:
            return np.zeros(3)
        return np.array(self.data.sensordata[adr: adr + 3])

    def _read_ee_wrench(self):
        return np.concatenate([
            self._read_sensor3(self.force_sensor_id),
            self._read_sensor3(self.torque_sensor_id),
        ])

    def _site_quat(self):
        # Convert MuJoCo site rotation matrix (row-major 3x3) to wxyz quat.
        mat = np.array(self.data.site_xmat[self.ee_site_id]).reshape(3, 3)
        trace = np.trace(mat)
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            return np.array([
                0.25 * s,
                (mat[2, 1] - mat[1, 2]) / s,
                (mat[0, 2] - mat[2, 0]) / s,
                (mat[1, 0] - mat[0, 1]) / s,
            ])
        i = int(np.argmax(np.diag(mat)))
        if i == 0:
            s = math.sqrt(1.0 + mat[0, 0] - mat[1, 1] - mat[2, 2]) * 2.0
            return np.array([
                (mat[2, 1] - mat[1, 2]) / s,
                0.25 * s,
                (mat[0, 1] + mat[1, 0]) / s,
                (mat[0, 2] + mat[2, 0]) / s,
            ])
        if i == 1:
            s = math.sqrt(1.0 + mat[1, 1] - mat[0, 0] - mat[2, 2]) * 2.0
            return np.array([
                (mat[0, 2] - mat[2, 0]) / s,
                (mat[0, 1] + mat[1, 0]) / s,
                0.25 * s,
                (mat[1, 2] + mat[2, 1]) / s,
            ])
        s = math.sqrt(1.0 + mat[2, 2] - mat[0, 0] - mat[1, 1]) * 2.0
        return np.array([
            (mat[1, 0] - mat[0, 1]) / s,
            (mat[0, 2] + mat[2, 0]) / s,
            (mat[1, 2] + mat[2, 1]) / s,
            0.25 * s,
        ])


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
