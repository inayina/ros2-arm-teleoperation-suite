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
from std_msgs.msg import Float64MultiArray, Float64

import yaml
from std_srvs.srv import Trigger

from mujoco_sim.domain_randomizer import DomainRandomizer

try:
    import mujoco  # noqa: F401
    _HAS_MUJOCO = True
    _MUJOCO_IMPORT_ERROR = ""
except Exception as exc:  # pragma: no cover
    _HAS_MUJOCO = False
    _MUJOCO_IMPORT_ERROR = str(exc)

JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
FINGER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
JOINT_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_UPPER = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
MAX_SIM_TORQUE_NM = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
MAX_GRIPPER_OPENING_M = 0.04
FALLBACK_JOINT_ORIGINS = (
    ((0.0, 0.0, 0.333), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
    ((0.0, -0.316, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((0.0825, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((-0.0825, 0.384, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((0.088, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
)
FALLBACK_HAND_ORIGIN = ((0.0, 0.0, 0.107), (0.0, 0.0, -math.pi / 4.0))
FALLBACK_EE_ORIGIN = ((0.0, 0.0, 0.10), (0.0, 0.0, 0.0))


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _transform(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = _rpy_matrix(*rpy)
    t[:3, 3] = np.asarray(xyz, dtype=float)
    return t


def _joint_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    t = np.eye(4)
    t[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return t


def _quat_wxyz_from_matrix(mat):
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


def fallback_ee_transform(q):
    """Approximate Panda FK used only when MuJoCo is unavailable."""
    t = np.eye(4)
    for qi, (xyz, rpy) in zip(q, FALLBACK_JOINT_ORIGINS):
        t = t @ _transform(xyz, rpy) @ _joint_z(float(qi))
    t = t @ _transform(*FALLBACK_HAND_ORIGIN) @ _transform(*FALLBACK_EE_ORIGIN)
    return t


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__("mujoco_sim")
        self.declare_parameter("model_path", "config/models/franka_panda.xml")
        self.declare_parameter("headless", True)
        self.declare_parameter("randomize", True)
        self.declare_parameter("randomization_path", "config/randomization.yaml")
        self.declare_parameter("physics_rate", 1000.0)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("ee_site", "panda_ee")
        self.declare_parameter("gravity_compensation", True)
        self.declare_parameter("grasp_assist_enabled", True)
        self.declare_parameter("grasp_assist_close_threshold", 0.25)
        self.declare_parameter("grasp_assist_release_threshold", 0.75)
        self.declare_parameter("grasp_assist_capture_radius", 0.09)
        self.declare_parameter("grasp_assist_hold_offset", [0.0, 0.0, -0.03])
        self.declare_parameter("contact_debug_enabled", False)
        self.declare_parameter("contact_debug_period_s", 1.0)
        self.declare_parameter(
            "initial_positions",
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        )

        self.physics_rate = self.get_parameter("physics_rate").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.base_frame = self.get_parameter("base_frame").value
        self.gravity_compensation = bool(self.get_parameter("gravity_compensation").value)
        self.n = len(JOINT_NAMES)

        # Domain Randomizer
        rand_path = self.get_parameter("randomization_path").value
        if rand_path and not os.path.isabs(rand_path):
            rand_path = os.path.abspath(rand_path)
        rand_cfg = {}
        if os.path.exists(rand_path):
            with open(rand_path, "r") as f:
                rand_cfg = yaml.safe_load(f) or {}
        if not bool(self.get_parameter("randomize").value):
            rand_cfg = {"domain_randomization": {"enabled": False}}
        self.randomizer = DomainRandomizer(rand_cfg)

        # Sim state (fallback integrator).
        self.initial_q = self._initial_positions()
        self.q = self.initial_q.copy()
        self.qd = np.zeros(self.n)
        self.tau = np.zeros(self.n)
        self.inertia = np.full(self.n, 0.5)
        self.damping = np.full(self.n, 2.0)

        self.model = None
        self.data = None
        self.joint_qposadr = []
        self.joint_dofadr = []
        self.gripper_qposadr = []
        self.actuator_ids = []
        self.ee_site_id = None
        self.target_body_id = None
        self.target_geom_id = None
        self.finger_body_ids = []
        self.finger_geom_ids = set()
        self.force_sensor_id = None
        self.torque_sensor_id = None
        self.target_joint_qposadr = None
        self.target_joint_dofadr = None
        self.gripper_cmd = MAX_GRIPPER_OPENING_M
        self.grasp_assist_enabled = bool(self.get_parameter("grasp_assist_enabled").value)
        self.grasp_close_threshold = float(self.get_parameter("grasp_assist_close_threshold").value)
        self.grasp_release_threshold = float(self.get_parameter("grasp_assist_release_threshold").value)
        self.grasp_capture_radius = float(self.get_parameter("grasp_assist_capture_radius").value)
        self.grasp_hold_offset = np.array(
            self.get_parameter("grasp_assist_hold_offset").value, dtype=float)
        self.grasp_attached = False
        self.grasp_offset = self.grasp_hold_offset.copy()
        self.grasp_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
        self.contact_debug_enabled = bool(self.get_parameter("contact_debug_enabled").value)
        self.contact_debug_period_s = max(
            0.1, float(self.get_parameter("contact_debug_period_s").value))
        self._last_contact_debug_time_s = -self.contact_debug_period_s
        self._try_load_model()

        self.sub_effort = self.create_subscription(
            Float64MultiArray, "/sim/joint_effort_cmd", self._on_effort,
            qos_profile_sensor_data)
        self.sub_grip = self.create_subscription(
            Float64, "/teleop/gripper_cmd", self._on_grip, 10)

        self.pub_encoder = self.create_publisher(
            JointState, "/sim/encoder_state", qos_profile_sensor_data)
        self.pub_ft = self.create_publisher(WrenchStamped, "/ft_sensor", 10)
        self.pub_ee = self.create_publisher(PoseStamped, "/ee_pose", 10)
        self.pub_obj = self.create_publisher(PoseStamped, "/sim/object_pose", 10)
        self.pub_gripper = self.create_publisher(Float64, "/gripper/state", 10)

        self.srv_reset = self.create_service(Trigger, "/sim/reset_scene", self._on_reset)

        self.create_timer(1.0 / self.physics_rate, self._step)
        self._pub_decim = max(1, int(self.physics_rate / self.publish_rate))
        self._k = 0

        mode = "MuJoCo" if self.model is not None else "fallback integrator"
        self.get_logger().info(f"mujoco_sim up ({mode}).")

    def _on_reset(self, request, response):
        if self.model is not None and _HAS_MUJOCO:
            import mujoco
            self._detach_grasp_assist()
            self._set_initial_pose(mujoco)
            self.randomizer.apply(self.model, self.data, mujoco)
            response.success = True
            response.message = "Scene reset and randomized."
            self.get_logger().info("Reset scene triggered.")
        else:
            response.success = False
            response.message = "No MuJoCo backend."
        return response

    def _try_load_model(self):
        path = self.get_parameter("model_path").value
        if path and not os.path.isabs(path):
            path = os.path.abspath(path)
        if not _HAS_MUJOCO:
            self.get_logger().warn(
                f"MuJoCo import failed ({_MUJOCO_IMPORT_ERROR}); using fallback integrator.")
            self._reset_fallback_state()
            return
        if not path:
            self.get_logger().warn("No MuJoCo model_path provided; using fallback integrator.")
            self._reset_fallback_state()
            return
        try:
            import mujoco
            self.model = mujoco.MjModel.from_xml_path(path)
            self.data = mujoco.MjData(self.model)
            self._build_mujoco_name_maps(mujoco)
            if self.contact_debug_enabled:
                self._log_grasp_model_params(mujoco)
            self._set_initial_pose(mujoco)
            self.randomizer.apply(self.model, self.data, mujoco)

            self.viewer = None
            if not self.get_parameter("headless").value:
                import mujoco.viewer
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

            self.get_logger().info(f"Loaded MuJoCo model: {path}")
        except Exception as e:  # pragma: no cover
            self.get_logger().warn(f"MuJoCo model load failed ({e}); using fallback.")
            self.model = None
            self.data = None
            self._reset_fallback_state()

    def _initial_positions(self):
        q0 = list(self.get_parameter("initial_positions").value)
        if len(q0) != self.n:
            self.get_logger().warn(
                f"initial_positions has {len(q0)} values, expected {self.n}; using ready pose.")
            q0 = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        q0 = np.asarray(q0, dtype=float)
        if q0.size != self.n or not np.all(np.isfinite(q0)):
            q0 = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        return np.clip(q0, JOINT_LOWER, JOINT_UPPER)

    def _reset_fallback_state(self):
        self.q = self.initial_q.copy()
        self.qd = np.zeros(self.n)
        self.tau = np.zeros(self.n)

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

        self.gripper_aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_motor")
        self.gripper_qposadr = []
        for name in FINGER_JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self.gripper_qposadr.append(int(self.model.jnt_qposadr[jid]))

        site_name = self.get_parameter("ee_site").value
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid >= 0:
            self.ee_site_id = sid

        self.target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target_object")
        self.target_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "target_object_geom")
        target_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "target_object_joint")
        if target_joint_id >= 0:
            self.target_joint_qposadr = int(self.model.jnt_qposadr[target_joint_id])
            self.target_joint_dofadr = int(self.model.jnt_dofadr[target_joint_id])

        self.finger_body_ids = [
            bid for bid in (
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_finger"),
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_finger"),
            )
            if bid >= 0
        ]
        self.finger_geom_ids = {
            gid for gid in range(self.model.ngeom)
            if int(self.model.geom_bodyid[gid]) in self.finger_body_ids
        }

        self.force_sensor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "ee_force")
        self.torque_sensor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "ee_torque")

    def _set_initial_pose(self, mujoco):
        q0 = self._initial_positions()
        for i, adr in enumerate(self.joint_qposadr):
            self.data.qpos[adr] = q0[i]
        for adr in self.gripper_qposadr:
            self.data.qpos[adr] = self.gripper_cmd
        mujoco.mj_forward(self.model, self.data)
        self.q = np.array([self.data.qpos[adr] for adr in self.joint_qposadr])
        self.qd = np.array([self.data.qvel[adr] for adr in self.joint_dofadr])

    def _on_effort(self, msg: Float64MultiArray):
        d = np.asarray(msg.data, dtype=float)
        if d.size >= self.n:
            self.tau = self._finite_tau(d[: self.n])

    def _on_grip(self, msg: Float64):
        self.gripper_cmd = float(np.clip(msg.data, 0.0, 1.0)) * MAX_GRIPPER_OPENING_M
        if self._gripper_opening_command_normalized() >= self.grasp_release_threshold:
            self._detach_grasp_assist()

    def _finite_tau(self, tau):
        tau = np.nan_to_num(tau, nan=0.0, posinf=0.0, neginf=0.0)
        return np.clip(tau, -MAX_SIM_TORQUE_NM, MAX_SIM_TORQUE_NM)

    def _sanitize_fallback_state(self):
        if not np.all(np.isfinite(self.q)) or not np.all(np.isfinite(self.qd)):
            self.get_logger().warn("Non-finite fallback state detected; resetting to ready pose.")
            self._reset_fallback_state()
            return
        self.q = np.clip(self.q, JOINT_LOWER, JOINT_UPPER)
        self.qd = np.nan_to_num(self.qd, nan=0.0, posinf=0.0, neginf=0.0)

    def _step(self):
        dt = 1.0 / self.physics_rate
        self.tau = self._finite_tau(self.tau)
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
            if getattr(self, 'gripper_aid', -1) >= 0:
                self.data.ctrl[self.gripper_aid] = self.gripper_cmd
            mujoco.mj_step(self.model, self.data)
            self._update_grasp_assist(mujoco)
            self._maybe_log_contact_debug(mujoco)

            self.q = np.array([self.data.qpos[adr] for adr in self.joint_qposadr])
            self.qd = np.array([self.data.qvel[adr] for adr in self.joint_dofadr])
            if not np.all(np.isfinite(self.q)) or not np.all(np.isfinite(self.qd)):
                self.get_logger().warn("Non-finite MuJoCo state detected; resetting scene.")
                self._set_initial_pose(mujoco)
                self.tau = np.zeros(self.n)
        else:
            # Simple decoupled second-order integrator.
            qdd = (self.tau - self.damping * self.qd) / self.inertia
            self.qd += qdd * dt
            self.q += self.qd * dt
            self._sanitize_fallback_state()

        self._k += 1
        if self._k % self._pub_decim == 0:
            if hasattr(self, 'viewer') and self.viewer is not None:
                self.viewer.sync()
            self._publish()

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = JOINT_NAMES
        js.position = np.nan_to_num(self.q, nan=0.0, posinf=0.0, neginf=0.0).tolist()
        js.velocity = np.nan_to_num(self.qd, nan=0.0, posinf=0.0, neginf=0.0).tolist()
        js.effort = self._finite_tau(self.tau).tolist()
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
        self.pub_ft.publish(ft)

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
            t_ee = fallback_ee_transform(self.q)
            quat = _quat_wxyz_from_matrix(t_ee[:3, :3])
            ee.pose.position.x = float(t_ee[0, 3])
            ee.pose.position.y = float(t_ee[1, 3])
            ee.pose.position.z = float(t_ee[2, 3])
            ee.pose.orientation.x = float(quat[1])
            ee.pose.orientation.y = float(quat[2])
            ee.pose.orientation.z = float(quat[3])
            ee.pose.orientation.w = float(quat[0])
        self.pub_ee.publish(ee)

        if self.model is not None and self.target_body_id is not None and self.target_body_id >= 0:
            obj_pose = PoseStamped()
            obj_pose.header.stamp = stamp
            obj_pose.header.frame_id = "world"
            obj_pose.pose.position.x = float(self.data.xpos[self.target_body_id][0])
            obj_pose.pose.position.y = float(self.data.xpos[self.target_body_id][1])
            obj_pose.pose.position.z = float(self.data.xpos[self.target_body_id][2])
            oq = self.data.xquat[self.target_body_id]
            obj_pose.pose.orientation.w = float(oq[0])
            obj_pose.pose.orientation.x = float(oq[1])
            obj_pose.pose.orientation.y = float(oq[2])
            obj_pose.pose.orientation.z = float(oq[3])
            self.pub_obj.publish(obj_pose)

        grip = Float64()
        grip.data = self._gripper_opening_normalized()
        self.pub_gripper.publish(grip)

    def _gripper_opening_normalized(self):
        if self.model is None or self.data is None or not self.gripper_qposadr:
            return self._gripper_opening_command_normalized()
        openings = [float(self.data.qpos[adr]) for adr in self.gripper_qposadr]
        return float(np.clip(np.mean(openings) / MAX_GRIPPER_OPENING_M, 0.0, 1.0))

    def _gripper_opening_command_normalized(self):
        return float(np.clip(self.gripper_cmd / MAX_GRIPPER_OPENING_M, 0.0, 1.0))

    def _detach_grasp_assist(self):
        self.grasp_attached = False

    def _target_object_pose(self):
        if self.target_body_id is None or self.target_body_id < 0:
            return None, None
        pos = np.array(self.data.xpos[self.target_body_id], dtype=float)
        quat = np.array(self.data.xquat[self.target_body_id], dtype=float)
        return pos, quat

    def _ee_position(self):
        if self.ee_site_id is None:
            return None
        return np.array(self.data.site_xpos[self.ee_site_id], dtype=float)

    def _set_target_object_pose(self, pos, quat, mujoco):
        if self.target_joint_qposadr is None:
            return
        adr = self.target_joint_qposadr
        self.data.qpos[adr: adr + 3] = np.asarray(pos, dtype=float)
        self.data.qpos[adr + 3: adr + 7] = np.asarray(quat, dtype=float)
        if self.target_joint_dofadr is not None:
            dof = self.target_joint_dofadr
            self.data.qvel[dof: dof + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _update_grasp_assist(self, mujoco):
        """Keep the synthetic-data cube attached once the closed gripper captures it."""
        if (
            not self.grasp_assist_enabled or
            self.target_joint_qposadr is None or
            self.ee_site_id is None or
            self.target_body_id is None or
            self.target_body_id < 0
        ):
            return

        opening = self._gripper_opening_command_normalized()
        if opening >= self.grasp_release_threshold:
            self._detach_grasp_assist()
            return

        ee_pos = self._ee_position()
        obj_pos, obj_quat = self._target_object_pose()
        if ee_pos is None or obj_pos is None:
            return

        if not self.grasp_attached and opening <= self.grasp_close_threshold:
            distance = float(np.linalg.norm(obj_pos - ee_pos))
            if distance <= self.grasp_capture_radius:
                self.grasp_attached = True
                self.grasp_offset = self.grasp_hold_offset.copy()
                self.grasp_quat_wxyz = obj_quat
                self.get_logger().info(
                    f"Grasp assist attached target_object at distance {distance:.3f} m.")

        if self.grasp_attached:
            self._set_target_object_pose(ee_pos + self.grasp_offset, self.grasp_quat_wxyz, mujoco)

    def _maybe_log_contact_debug(self, mujoco):
        if not self.contact_debug_enabled:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._last_contact_debug_time_s < self.contact_debug_period_s:
            return
        self._last_contact_debug_time_s = now_s

        obj_pos, _ = self._target_object_pose()
        ee_pos = self._ee_position()
        if obj_pos is not None and ee_pos is not None:
            dist = float(np.linalg.norm(obj_pos - ee_pos))
        else:
            dist = math.nan
        gripper_qpos = [float(self.data.qpos[adr]) for adr in self.gripper_qposadr]
        gripper_ctrl = (
            float(self.data.ctrl[self.gripper_aid])
            if getattr(self, "gripper_aid", -1) >= 0 else math.nan
        )
        arm_ctrl = [float(self.data.ctrl[aid]) for aid in self.actuator_ids]
        total_contacts, object_contacts, finger_object_contacts, samples = (
            self._contact_debug_summary(mujoco)
        )

        self.get_logger().info(
            "M7 grasp debug "
            f"object_pos={self._fmt_vec(obj_pos)} "
            f"ee_pos={self._fmt_vec(ee_pos)} "
            f"ee_object_dist={dist:.3f} "
            f"gripper_qpos={self._fmt_vec(gripper_qpos)} "
            f"gripper_cmd={self._gripper_opening_command_normalized():.3f} "
            f"gripper_ctrl={gripper_ctrl:.4f} "
            f"arm_ctrl={self._fmt_vec(arm_ctrl)} "
            f"contacts_total={total_contacts} "
            f"object_contacts={object_contacts} "
            f"finger_object_contacts={finger_object_contacts} "
            f"grasp_assist_attached={self.grasp_attached} "
            f"contact_samples={samples}"
        )

    def _contact_debug_summary(self, mujoco):
        total_contacts = int(self.data.ncon)
        object_contacts = 0
        finger_object_contacts = 0
        samples = []
        if self.target_geom_id is None or self.target_geom_id < 0:
            return total_contacts, object_contacts, finger_object_contacts, samples

        for idx in range(total_contacts):
            contact = self.data.contact[idx]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            has_object = self.target_geom_id in (geom1, geom2)
            has_finger = geom1 in self.finger_geom_ids or geom2 in self.finger_geom_ids
            if has_object:
                object_contacts += 1
                if len(samples) < 4:
                    samples.append(
                        f"{self._geom_name(mujoco, geom1)}:{self._geom_name(mujoco, geom2)}"
                    )
            if has_object and has_finger:
                finger_object_contacts += 1
        return total_contacts, object_contacts, finger_object_contacts, samples

    def _log_grasp_model_params(self, mujoco):
        target = self.target_geom_id
        body = self.target_body_id
        if target is not None and target >= 0 and body is not None and body >= 0:
            self.get_logger().info(
                "M7 grasp model target_object "
                f"mass={self.model.body_mass[body]:.4f} "
                f"inertia={self._fmt_vec(self.model.body_inertia[body], precision=6)} "
                f"geom_size={self._fmt_vec(self.model.geom_size[target], precision=4)} "
                f"friction={self._fmt_vec(self.model.geom_friction[target], precision=4)} "
                f"condim={int(self.model.geom_condim[target])} "
                f"solref={self._fmt_vec(self.model.geom_solref[target], precision=4)} "
                f"solimp={self._fmt_vec(self.model.geom_solimp[target], precision=4)}"
            )
        if self.finger_geom_ids:
            sample = sorted(self.finger_geom_ids)[0]
            self.get_logger().info(
                "M7 grasp model finger_geoms "
                f"count={len(self.finger_geom_ids)} "
                f"sample_friction={self._fmt_vec(self.model.geom_friction[sample], precision=4)} "
                f"sample_condim={int(self.model.geom_condim[sample])} "
                f"sample_solref={self._fmt_vec(self.model.geom_solref[sample], precision=4)} "
                f"sample_solimp={self._fmt_vec(self.model.geom_solimp[sample], precision=4)}"
            )

    def _geom_name(self, mujoco, geom_id):
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        return name if name else f"geom_{geom_id}"

    def _fmt_vec(self, values, precision=3):
        if values is None:
            return "None"
        return "[" + ", ".join(f"{float(v):.{precision}f}" for v in values) + "]"

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
        return _quat_wxyz_from_matrix(mat)



def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'viewer') and node.viewer is not None:
            node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
