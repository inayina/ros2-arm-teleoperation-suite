#!/usr/bin/env python3
"""L6 camera bridge.

Publishes (default 30 Hz):
  /camera/color/image_raw   sensor_msgs/Image  (rgb8)
  /camera/depth/image_raw   sensor_msgs/Image  (16UC1, millimeters)
  /camera/color/camera_info sensor_msgs/CameraInfo

SCAFFOLD: emits a synthetic gradient/test pattern so downstream (recorder, VLA)
can be developed without a renderer. M6 task: source frames from the MuJoCo
offscreen renderer (see mujoco_sim virtual cameras) instead of the pattern.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraBridgeNode(Node):
    def __init__(self):
        super().__init__("camera_bridge")
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.frame_id = self.get_parameter("frame_id").value
        rate = self.get_parameter("rate").value

        self.pub_color = self.create_publisher(
            Image, "/camera/color/image_raw", qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(
            Image, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(
            CameraInfo, "/camera/color/camera_info", qos_profile_sensor_data)

        self._k = 0
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"camera_bridge up ({self.w}x{self.h} @ {rate} Hz, scaffold).")

    def _camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.w
        info.height = self.h
        fx = fy = float(self.w)
        cx, cy = self.w / 2.0, self.h / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.distortion_model = "plumb_bob"
        return info

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        self._k += 1

        # Synthetic RGB gradient (rgb8)
        xs = np.linspace(0, 255, self.w, dtype=np.uint8)
        row = np.tile(xs, (self.h, 1))
        rgb = np.dstack([
            row,
            np.flipud(row),
            np.full((self.h, self.w), (self._k * 4) % 256, dtype=np.uint8),
        ])
        color = Image()
        color.header.stamp = stamp
        color.header.frame_id = self.frame_id
        color.height, color.width = self.h, self.w
        color.encoding = "rgb8"
        color.step = self.w * 3
        color.data = rgb.tobytes()
        self.pub_color.publish(color)

        # Synthetic depth (16UC1, mm)
        depth_arr = np.full((self.h, self.w), 800, dtype=np.uint16)
        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = self.frame_id
        depth.height, depth.width = self.h, self.w
        depth.encoding = "16UC1"
        depth.step = self.w * 2
        depth.data = depth_arr.tobytes()
        self.pub_depth.publish(depth)

        self.pub_info.publish(self._camera_info(stamp))


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
