#!/usr/bin/env python3
"""L6 camera bridge: MuJoCo virtual camera to ROS Image topics."""
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState

try:
    import mujoco
    from mujoco_sim.virtual_camera import CameraModel, VirtualCamera

    _HAS_MUJOCO = True
except Exception:  # pragma: no cover - exercised on systems without MuJoCo
    _HAS_MUJOCO = False

JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]


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
        self.declare_parameter("use_mujoco_renderer", True)
        self.declare_parameter("synthetic_fallback", True)

        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.rate = float(self.get_parameter("rate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.synthetic_fallback = bool(self.get_parameter("synthetic_fallback").value)

        self._k = 0
        self._q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=float)
        self._model = None
        self._data = None
        self._joint_qposadr: list[int] = []
        self._camera = None

        self.pub_color = self.create_publisher(
            Image, "/camera/color/image_raw", qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(
            Image, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(
            CameraInfo, "/camera/color/camera_info", qos_profile_sensor_data)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data)

        if bool(self.get_parameter("use_mujoco_renderer").value):
            self._try_init_mujoco()

        self.create_timer(1.0 / self.rate, self._tick)
        mode = "MuJoCo renderer" if self._camera is not None else "synthetic fallback"
        self.get_logger().info(f"camera_bridge up ({self.w}x{self.h} @ {self.rate} Hz, {mode}).")

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
            for name in JOINT_NAMES:
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid < 0:
                    raise RuntimeError(f"MuJoCo joint '{name}' not found")
                self._joint_qposadr.append(int(self._model.jnt_qposadr[jid]))
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

    def _set_model_joints(self, q):
        if self._model is None or self._data is None:
            return
        for value, adr in zip(q, self._joint_qposadr):
            self._data.qpos[adr] = float(value)
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
        elif self.synthetic_fallback:
            rgb, depth_arr = self._synthetic_frame()
        else:
            return

        self.pub_color.publish(self._image_msg(stamp, "rgb8", rgb))
        self.pub_depth.publish(self._image_msg(stamp, "32FC1", depth_arr.astype(np.float32)))
        self.pub_info.publish(self._camera_info(stamp))

    def _synthetic_frame(self):
        xs = np.linspace(0, 255, self.w, dtype=np.uint8)
        row = np.tile(xs, (self.h, 1))
        rgb = np.dstack([
            row,
            np.flipud(row),
            np.full((self.h, self.w), (self._k * 4) % 256, dtype=np.uint8),
        ])
        depth = np.full((self.h, self.w), 0.8, dtype=np.float32)
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
