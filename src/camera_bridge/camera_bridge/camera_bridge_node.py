#!/usr/bin/env python3
"""L6 camera bridge: MuJoCo virtual camera to ROS Image topics."""
import math
import os

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64

try:
    import mujoco
    from mujoco_sim.virtual_camera import CameraModel, VirtualCamera

    _HAS_MUJOCO = True
except Exception:  # pragma: no cover - exercised on systems without MuJoCo
    _HAS_MUJOCO = False

JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
FINGER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
MAX_GRIPPER_OPENING_M = 0.04


class CameraBridgeNode(Node):
    def __init__(self):
        super().__init__("camera_bridge")
        self.declare_parameter("model_path", "config/models/franka_panda.xml")
        self.declare_parameter("camera_name", "scene_camera")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("fovy_deg", 45.0)
        self.declare_parameter("frame_id", "scene_camera_optical_frame")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("use_mujoco_renderer", True)
        self.declare_parameter("synthetic_fallback", True)
        self.declare_parameter("tactile_mode", False)
        self.declare_parameter("gel_depth_baseline", 0.0155)
        self.declare_parameter("gel_scale", 300.0)

        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.rate = float(self.get_parameter("rate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.synthetic_fallback = bool(self.get_parameter("synthetic_fallback").value)
        self.tactile_mode = bool(self.get_parameter("tactile_mode").value)
        self.gel_depth_baseline = float(self.get_parameter("gel_depth_baseline").value)
        self.gel_scale = float(self.get_parameter("gel_scale").value)

        self._k = 0
        self._q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=float)
        self._gripper_opening = 1.0
        self._model = None
        self._data = None
        self._joint_qposadr: list[int] = []
        self._gripper_qposadr: list[int] = []
        self._target_qposadr = None
        self._target_qveladr = None
        self._object_pose = None
        self._camera = None

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.pub_color = self.create_publisher(
            Image, color_topic, qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(
            Image, depth_topic, qos_profile_sensor_data)
        self.pub_info = self.create_publisher(
            CameraInfo, camera_info_topic, qos_profile_sensor_data)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data)
        self.create_subscription(
            Float64, "/gripper/state", self._on_gripper_state, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/sim/object_pose", self._on_object_pose, qos_profile_sensor_data)

        if bool(self.get_parameter("use_mujoco_renderer").value):
            self._try_init_mujoco()

        self.create_timer(1.0 / self.rate, self._tick)
        mode = "MuJoCo renderer" if self._camera is not None else "synthetic fallback"
        self.get_logger().info(
            f"camera_bridge up ({self.w}x{self.h} @ {self.rate} Hz, {mode}, "
            f"color={color_topic})."
        )

    def _try_init_mujoco(self):
        if not _HAS_MUJOCO:
            self.get_logger().warn("MuJoCo renderer unavailable; using synthetic fallback.")
            return
        path = str(self.get_parameter("model_path").value)
        if path and not os.path.isabs(path):
            path = os.path.abspath(path)
        try:
            self._model = mujoco.MjModel.from_xml_path(path)
            self._data = mujoco.MjData(self._model)
            self._joint_qposadr = []
            self._gripper_qposadr = []
            for name in JOINT_NAMES:
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid < 0:
                    raise RuntimeError(f"MuJoCo joint '{name}' not found")
                self._joint_qposadr.append(int(self._model.jnt_qposadr[jid]))
            for name in FINGER_JOINT_NAMES:
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid >= 0:
                    self._gripper_qposadr.append(int(self._model.jnt_qposadr[jid]))
            target_jid = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, "target_object_joint")
            if target_jid >= 0:
                self._target_qposadr = int(self._model.jnt_qposadr[target_jid])
                self._target_qveladr = int(self._model.jnt_dofadr[target_jid])
            self._set_model_joints(self._q)

            camera = CameraModel(
                name=str(self.get_parameter("camera_name").value),
                width=self.w,
                height=self.h,
                fovy_deg=float(self.get_parameter("fovy_deg").value),
                frame_id=self.frame_id,
            )
            self._camera = VirtualCamera(mujoco, self._model, camera)
            self.get_logger().info(f"Loaded MuJoCo camera '{camera.name}' from {path}")
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"MuJoCo camera init failed ({exc}); using synthetic fallback.")
            self._model = None
            self._data = None
            self._camera = None

    def _on_joint_state(self, msg: JointState):
        if not msg.position:
            return
        by_name = dict(zip(msg.name, msg.position)) if msg.name else {}
        values = []
        for i, joint_name in enumerate(JOINT_NAMES):
            if joint_name in by_name:
                values.append(float(by_name[joint_name]))
            elif i < len(msg.position):
                values.append(float(msg.position[i]))
            else:
                values.append(float(self._q[i]))
        self._q = np.asarray(values, dtype=float)

    def _on_gripper_state(self, msg: Float64):
        self._gripper_opening = float(np.clip(msg.data, 0.0, 1.0))

    def _on_object_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        quat = np.array([o.w, o.x, o.y, o.z], dtype=float)
        norm = float(np.linalg.norm(quat))
        if norm < 1e-9 or not np.all(np.isfinite(quat)):
            quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        else:
            quat = quat / norm
        if np.all(np.isfinite(pos)):
            self._object_pose = (pos, quat)

    def _set_model_joints(self, q):
        if self._model is None or self._data is None:
            return
        for value, adr in zip(q, self._joint_qposadr):
            self._data.qpos[adr] = float(value)
        gripper_qpos = self._gripper_opening * MAX_GRIPPER_OPENING_M
        for adr in self._gripper_qposadr:
            self._data.qpos[adr] = gripper_qpos
        if self._target_qposadr is not None and self._object_pose is not None:
            pos, quat = self._object_pose
            adr = self._target_qposadr
            self._data.qpos[adr: adr + 3] = pos
            self._data.qpos[adr + 3: adr + 7] = quat
            if self._target_qveladr is not None:
                self._data.qvel[self._target_qveladr: self._target_qveladr + 6] = 0.0
        self._data.qvel[:] = 0.0
        mujoco.mj_forward(self._model, self._data)

    def _camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.w
        info.height = self.h
        if self._camera is not None:
            info.k = self._camera.camera.intrinsic_matrix
            info.p = self._camera.camera.projection_matrix
        else:
            fx = fy = float(self.w)
            cx, cy = self.w / 2.0, self.h / 2.0
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.distortion_model = "plumb_bob"
        return info

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        self._k += 1

        if self._camera is not None:
            self._set_model_joints(self._q)
            rgb, depth_arr = self._camera.render(self._data)
            rgb = np.ascontiguousarray(np.flipud(rgb))
            depth_arr = np.ascontiguousarray(np.flipud(depth_arr))
            if self.tactile_mode:
                rgb = self._simulate_gelsight(depth_arr)
        elif self.synthetic_fallback:
            rgb, depth_arr = self._synthetic_frame()
            if self.tactile_mode:
                rgb = self._simulate_gelsight(depth_arr)
        else:
            return

        self.pub_color.publish(self._image_msg(stamp, "rgb8", rgb))
        self.pub_depth.publish(self._image_msg(stamp, "32FC1", depth_arr.astype(np.float32)))
        self.pub_info.publish(self._camera_info(stamp))

    def _simulate_gelsight(self, depth_arr: np.ndarray) -> np.ndarray:
        # 1. Compute deformation from baseline
        deformation = np.maximum(0.0, self.gel_depth_baseline - depth_arr)

        # 2. Scale deformation for gradient calculation
        def_scaled = deformation * self.gel_scale

        # 3. Compute spatial gradients using central differences
        dy, dx = np.gradient(def_scaled)

        # 4. Compute normal vector
        nx = -dx
        ny = dy
        nz = np.ones_like(nx)

        norm = np.sqrt(nx**2 + ny**2 + nz**2)
        nx = nx / norm
        ny = ny / norm
        nz = nz / norm

        # 5. Define directional lights (Red from top/left, Green from top/right, Blue from bottom)
        lr = np.array([-0.5, 0.866, 0.3])
        lg = np.array([0.866, 0.5, 0.3])
        lb = np.array([-0.3, -0.866, 0.3])

        # Normalize lights
        lr = lr / np.linalg.norm(lr)
        lg = lg / np.linalg.norm(lg)
        lb = lb / np.linalg.norm(lb)

        # 6. Shading calculation (diffuse + ambient)
        dot_r = nx * lr[0] + ny * lr[1] + nz * lr[2]
        dot_g = nx * lg[0] + ny * lg[1] + nz * lg[2]
        dot_b = nx * lb[0] + ny * lb[1] + nz * lb[2]

        diffuse_r = np.maximum(0.0, dot_r)
        diffuse_g = np.maximum(0.0, dot_g)
        diffuse_b = np.maximum(0.0, dot_b)

        ambient = 0.3
        diffuse = 0.7

        r = ambient + diffuse * diffuse_r
        g = ambient + diffuse * diffuse_g
        b = ambient + diffuse * diffuse_b

        rgb_tactile = (np.clip(np.dstack([r, g, b]), 0.0, 1.0) * 255.0).astype(np.uint8)
        return rgb_tactile

    def _synthetic_frame(self):
        xs = np.linspace(0, 255, self.w, dtype=np.uint8)
        row = np.tile(xs, (self.h, 1))
        rgb = np.dstack([
            row,
            np.flipud(row),
            np.full((self.h, self.w), (self._k * 4) % 256, dtype=np.uint8),
        ])

        # Default depth
        depth = np.full((self.h, self.w), 0.8, dtype=np.float32)

        # If tactile mode, let's create a simulated indentation in the middle of the gel
        if self.tactile_mode:
            depth = np.full((self.h, self.w), self.gel_depth_baseline, dtype=np.float32)
            cx, cy = self.w // 2, self.h // 2
            r = min(self.w, self.h) // 6
            Y, X = np.ogrid[:self.h, :self.w]
            dist_sq = (X - cx)**2 + (Y - cy)**2
            mask = dist_sq < r**2

            # Oscillating indentation depth (max 2mm deep)
            depth_oscillation = 0.002 * (0.5 + 0.5 * math.sin(self._k * 0.1))
            # Sphere shape: depth is smaller in the middle
            sphere_depth = self.gel_depth_baseline - depth_oscillation * np.sqrt(np.maximum(0.0, 1.0 - dist_sq / (r**2)))
            depth[mask] = sphere_depth[mask]

        return rgb, depth

    def _image_msg(self, stamp, encoding: str, array: np.ndarray) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = int(array.shape[0])
        msg.width = int(array.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(array.strides[0])
        msg.data = np.ascontiguousarray(array).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
