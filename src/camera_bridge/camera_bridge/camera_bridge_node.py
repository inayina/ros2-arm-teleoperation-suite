#!/usr/bin/env python3
"""L6 camera bridge: MuJoCo virtual camera to ROS Image topics."""
import math
import os
import time

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
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

from camera_bridge.object_sync import MANIPULABLE_OBJECTS, MUJOCO_SIM_PARAM_NODE, object_joint_name

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
        self.declare_parameter("publish_depth", False)
        self.declare_parameter("synthetic_fallback", True)
        self.declare_parameter("tactile_mode", False)
        self.declare_parameter("gel_depth_baseline", 0.0155)
        self.declare_parameter("gel_scale", 300.0)
        self.declare_parameter("target_object_name", "object_red_box")
        self.declare_parameter("mujoco_sim_param_node", MUJOCO_SIM_PARAM_NODE)

        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.rate = float(self.get_parameter("rate").value)
        self._min_publish_period_s = 1.0 / self.rate
        self._last_publish_wall_s: float | None = None
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.synthetic_fallback = bool(self.get_parameter("synthetic_fallback").value)
        self.tactile_mode = bool(self.get_parameter("tactile_mode").value)
        self.publish_depth = bool(self.get_parameter("publish_depth").value)
        self.gel_depth_baseline = float(self.get_parameter("gel_depth_baseline").value)
        self.gel_scale = float(self.get_parameter("gel_scale").value)

        self._k = 0
        self._q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=float)
        self._gripper_opening = 1.0
        self._model = None
        self._data = None
        self._joint_qposadr: list[int] = []
        self._gripper_qposadr: list[int] = []
        self._object_joints: dict[str, dict[str, object]] = {}
        self._target_object_name = str(self.get_parameter("target_object_name").value)
        self._mujoco_sim_param_node = str(self.get_parameter("mujoco_sim_param_node").value)
        self._object_pose = None
        self._ee_pose = None
        self._camera = None
        self._params_poll_counter = 0
        self._get_params_future = None
        self._get_params_client = self.create_client(
            GetParameters, f"{self._mujoco_sim_param_node}/get_parameters")

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.pub_color = self.create_publisher(
            Image, color_topic, qos_profile_sensor_data)
        self.pub_depth = (
            self.create_publisher(Image, depth_topic, qos_profile_sensor_data)
            if self.publish_depth else None
        )
        self.pub_info = self.create_publisher(
            CameraInfo, camera_info_topic, qos_profile_sensor_data)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data)
        self.create_subscription(
            Float64, "/gripper/state", self._on_gripper_state, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/sim/object_pose", self._on_object_pose, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/ee_pose", self._on_ee_pose, qos_profile_sensor_data)

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
            self._object_joints = self._build_object_joint_map(mujoco)
            self._set_target_object_name(self._target_object_name, log=False)
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

    def _build_object_joint_map(self, mujoco_module) -> dict[str, dict[str, object]]:
        joints: dict[str, dict[str, object]] = {}
        for object_name in MANIPULABLE_OBJECTS:
            joint_name = object_joint_name(object_name)
            joint_id = mujoco_module.mj_name2id(
                self._model, mujoco_module.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                self.get_logger().warn(
                    f"MuJoCo joint '{joint_name}' not found for camera object sync.")
                continue
            qposadr = int(self._model.jnt_qposadr[joint_id])
            qveladr = int(self._model.jnt_dofadr[joint_id])
            joints[object_name] = {
                "qposadr": qposadr,
                "qveladr": qveladr,
                "initial_qpos": self._data.qpos[qposadr: qposadr + 7].copy(),
            }
        return joints

    def _set_target_object_name(self, target_name: str, *, log: bool = True) -> None:
        target_name = str(target_name).strip()
        if not target_name or target_name == self._target_object_name:
            return
        if target_name not in self._object_joints:
            self.get_logger().warn(
                f"Unknown target_object_name '{target_name}' for camera render sync.")
            return
        self._target_object_name = target_name
        if log:
            joint_name = object_joint_name(target_name)
            self.get_logger().info(
                f"Camera render target updated: name={target_name}, joint={joint_name}")

    def _poll_target_object_name(self) -> None:
        self._params_poll_counter += 1
        if self._params_poll_counter % 30 != 0:
            return
        if not self._get_params_client.service_is_ready():
            return
        if self._get_params_future is not None and not self._get_params_future.done():
            return
        request = GetParameters.Request()
        request.names = ["target_object_name"]
        self._get_params_future = self._get_params_client.call_async(request)
        self._get_params_future.add_done_callback(
            self._on_target_object_parameters)

    def _on_target_object_parameters(self, future) -> None:
        try:
            response = future.result()
        except Exception:
            self._get_params_future = None
            return
        self._get_params_future = None
        if not response or not response.values:
            return
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_STRING:
            return
        self._set_target_object_name(value.string_value)

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

    def _on_ee_pose(self, msg: PoseStamped):
        p = msg.pose.position
        pos = np.array([p.x, p.y, p.z], dtype=float)
        if np.all(np.isfinite(pos)):
            self._ee_pose = pos

    def _set_model_joints(self, q):
        if self._model is None or self._data is None:
            return
        for value, adr in zip(q, self._joint_qposadr):
            self._data.qpos[adr] = float(value)
        gripper_qpos = self._gripper_opening * MAX_GRIPPER_OPENING_M
        for adr in self._gripper_qposadr:
            self._data.qpos[adr] = gripper_qpos
        for object_name, joint_info in self._object_joints.items():
            qposadr = int(joint_info["qposadr"])
            qveladr = int(joint_info["qveladr"])
            if object_name == self._target_object_name and self._object_pose is not None:
                pos, quat = self._object_pose
                self._data.qpos[qposadr: qposadr + 3] = pos
                self._data.qpos[qposadr + 3: qposadr + 7] = quat
            else:
                self._data.qpos[qposadr: qposadr + 7] = joint_info["initial_qpos"]
            self._data.qvel[qveladr: qveladr + 6] = 0.0
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
        now_wall_s = time.monotonic()
        if (
            self._last_publish_wall_s is not None
            and now_wall_s - self._last_publish_wall_s
            < self._min_publish_period_s * 0.9
        ):
            return
        self._last_publish_wall_s = now_wall_s
        stamp = self.get_clock().now().to_msg()
        self._k += 1

        if self._camera is not None:
            self._poll_target_object_name()
            self._set_model_joints(self._q)
            needs_depth = self.publish_depth or self.tactile_mode
            if needs_depth:
                rgb, depth_arr = self._camera.render(self._data)
            else:
                rgb = self._camera.render_rgb(self._data)
                depth_arr = None
            # mujoco.Renderer already returns top-left-origin image arrays.
            # A legacy OpenGL flip here made recorded RGB/depth appear upside-down.
            rgb = np.ascontiguousarray(rgb)
            if depth_arr is not None:
                depth_arr = np.ascontiguousarray(depth_arr)
            if self.tactile_mode:
                assert depth_arr is not None
                rgb = self._simulate_gelsight(depth_arr)
        elif self.synthetic_fallback:
            rgb, depth_arr = self._synthetic_frame()
            if self.tactile_mode:
                rgb = self._simulate_gelsight(depth_arr)
        else:
            return

        self.pub_color.publish(self._image_msg(stamp, "rgb8", rgb))
        if self.pub_depth is not None:
            assert depth_arr is not None
            self.pub_depth.publish(
                self._image_msg(stamp, "32FC1", depth_arr.astype(np.float32)))
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
        depth = np.full((self.h, self.w), 0.8, dtype=np.float32)

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
            xs = np.linspace(0, 255, self.w, dtype=np.uint8)
            row = np.tile(xs, (self.h, 1))
            rgb = np.dstack([
                row,
                np.flipud(row),
                np.full((self.h, self.w), (self._k * 4) % 256, dtype=np.uint8),
            ])
            return rgb, depth

        rgb = np.full((self.h, self.w, 3), (236, 239, 241), dtype=np.uint8)
        self._draw_table_scene(rgb, depth)
        return rgb, depth

    def _world_to_px(self, pos: np.ndarray | tuple[float, float, float]) -> tuple[int, int]:
        x, y, z = (float(pos[0]), float(pos[1]), float(pos[2]))
        u = int(self.w * 0.50 + (y / 0.32) * self.w * 0.36)
        v = int(self.h * 0.84 - ((x - 0.20) / 0.35) * self.h * 0.54 - z * self.h * 1.35)
        return int(np.clip(u, 0, self.w - 1)), int(np.clip(v, 0, self.h - 1))

    @staticmethod
    def _draw_rect(img: np.ndarray, x0: int, y0: int, x1: int, y1: int, color) -> None:
        h, w = img.shape[:2]
        x0, x1 = sorted((max(0, x0), min(w, x1)))
        y0, y1 = sorted((max(0, y0), min(h, y1)))
        if x1 > x0 and y1 > y0:
            img[y0:y1, x0:x1] = color

    def _draw_box_outline(self, img: np.ndarray, center, half_w: int, half_h: int, color) -> None:
        u, v = center
        t = max(2, min(self.w, self.h) // 120)
        self._draw_rect(img, u - half_w, v - half_h, u + half_w, v - half_h + t, color)
        self._draw_rect(img, u - half_w, v + half_h - t, u + half_w, v + half_h, color)
        self._draw_rect(img, u - half_w, v - half_h, u - half_w + t, v + half_h, color)
        self._draw_rect(img, u + half_w - t, v - half_h, u + half_w, v + half_h, color)

    def _draw_circle(self, img: np.ndarray, center, radius: int, color) -> None:
        u, v = center
        y0 = max(0, v - radius)
        y1 = min(self.h, v + radius + 1)
        x0 = max(0, u - radius)
        x1 = min(self.w, u + radius + 1)
        if x1 <= x0 or y1 <= y0:
            return
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (xx - u) ** 2 + (yy - v) ** 2 <= radius ** 2
        img[y0:y1, x0:x1][mask] = color

    def _draw_table_scene(self, rgb: np.ndarray, depth: np.ndarray) -> None:
        table_color = (210, 216, 220)
        self._draw_rect(rgb, 0, int(self.h * 0.16), self.w, self.h, table_color)
        for y, color in [(-0.35, (128, 144, 154)), (0.35, (148, 158, 164))]:
            center = self._world_to_px((0.40, y, 0.02))
            self._draw_box_outline(
                rgb,
                center,
                max(18, self.w // 11),
                max(14, self.h // 13),
                color,
            )

        if self._ee_pose is not None:
            ee_u, ee_v = self._world_to_px(self._ee_pose)
            gap = int(8 + 24 * float(np.clip(self._gripper_opening, 0.0, 1.0)))
            finger_h = max(10, self.h // 24)
            finger_w = max(3, self.w // 90)
            color = (0, 132, 180)
            self._draw_rect(rgb, ee_u - gap, ee_v - finger_h, ee_u - gap + finger_w, ee_v + finger_h, color)
            self._draw_rect(rgb, ee_u + gap - finger_w, ee_v - finger_h, ee_u + gap, ee_v + finger_h, color)
            self._draw_rect(rgb, ee_u - gap, ee_v - finger_h, ee_u + gap, ee_v - finger_h + finger_w, color)

        if self._object_pose is None:
            return
        pos, _ = self._object_pose
        obj_u, obj_v = self._world_to_px(pos)
        shadow_u, shadow_v = self._world_to_px((pos[0], pos[1], 0.0))
        shadow_r = max(5, min(self.w, self.h) // 45)
        self._draw_circle(rgb, (shadow_u, shadow_v), shadow_r, (170, 176, 180))

        z = max(0.0, float(pos[2]))
        size = max(8, int(min(self.w, self.h) * (0.045 + z * 0.20)))
        color = (210, 40, 42)
        if "blue" in self._target_object_name:
            color = (36, 94, 190)
        elif "green" in self._target_object_name:
            color = (46, 150, 72)

        if "box" in self._target_object_name:
            self._draw_rect(rgb, obj_u - size, obj_v - size, obj_u + size, obj_v + size, color)
        else:
            self._draw_circle(rgb, (obj_u, obj_v), size, color)

        d = float(np.clip(0.55 - z, 0.2, 0.8))
        self._draw_rect(depth, obj_u - size, obj_v - size, obj_u + size, obj_v + size, d)

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
