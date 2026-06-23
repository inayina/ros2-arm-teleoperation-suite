#!/usr/bin/env python3
"""L7 multi-modal recorder.

Time-aligns the observation streams and records, per frame:
  observation.state    <- /joint_states (position)
  observation.ee_pose  <- /ee_pose
  observation.ft       <- /ft_sensor
  observation.gripper  <- /gripper/state
  observation.images.scene <- /camera/color/image_raw
  observation.depth.scene  <- /camera/depth/image_raw
  action               <- /safe_master_pose (+ gripper)
  timestamp / frame_index / episode_index

Control: /teleop/record_trigger (std_msgs/String) "start" | "stop".

SCAFFOLD: synchronizes joint_states + color + depth via message_filters; ee/ft/
gripper/action are cached as latest. M6 task: full LeRobotDataset v2 output and
richer sync. Output dir: `output_dir` param (default data/episodes).
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64, String

from .lerobot_writer import write_episode


def _img_to_np(msg: Image) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        return buf.reshape(msg.height, msg.width, 3)
    if msg.encoding == "16UC1":
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    return buf


class RecorderNode(Node):
    def __init__(self):
        super().__init__("lerobot_recorder")
        self.declare_parameter("output_dir", "data/episodes")
        self.declare_parameter("task", "teleop")
        self.out_dir = self.get_parameter("output_dir").value
        self.task = self.get_parameter("task").value

        self.recording = False
        self.episode_index = 0
        self.frames = []

        # Latest-value caches for non-synchronized streams.
        self._ee = None
        self._ft = None
        self._grip = 0.0
        self._action = None

        self.create_subscription(PoseStamped, "/ee_pose", self._on_ee, 10)
        self.create_subscription(WrenchStamped, "/ft_sensor", self._on_ft, 10)
        self.create_subscription(Float64, "/gripper/state", self._on_grip, 10)
        self.create_subscription(PoseStamped, "/safe_master_pose", self._on_action, 10)
        self.create_subscription(String, "/teleop/record_trigger", self._on_trigger, 10)

        # Synchronized observation streams.
        js_sub = message_filters.Subscriber(self, JointState, "/joint_states",
                                             qos_profile=qos_profile_sensor_data)
        color_sub = message_filters.Subscriber(self, Image, "/camera/color/image_raw",
                                                qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, "/camera/depth/image_raw",
                                                qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [js_sub, color_sub, depth_sub], queue_size=30, slop=0.05)
        self.sync.registerCallback(self._on_frame)

        self.get_logger().info(f"lerobot_recorder up (output={self.out_dir}).")

    def _on_ee(self, m): self._ee = m
    def _on_ft(self, m): self._ft = m
    def _on_grip(self, m): self._grip = m.data
    def _on_action(self, m): self._action = m

    def _on_trigger(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "start" and not self.recording:
            self.frames = []
            self.recording = True
            self.get_logger().info(f"recording episode {self.episode_index} ...")
        elif cmd == "stop" and self.recording:
            self.recording = False
            if self.frames:
                path = write_episode(self.out_dir, self.episode_index, self.frames, self.task)
                self.get_logger().info(f"saved {len(self.frames)} frames -> {path}")
                self.episode_index += 1

    @staticmethod
    def _pose_vec(p: PoseStamped):
        o = p.pose.orientation
        q = p.pose.position
        return [q.x, q.y, q.z, o.x, o.y, o.z, o.w]

    def _on_frame(self, js: JointState, color: Image, depth: Image):
        if not self.recording:
            return
        ee = self._pose_vec(self._ee) if self._ee else [0.0] * 7
        ft = ([self._ft.wrench.force.x, self._ft.wrench.force.y, self._ft.wrench.force.z,
               self._ft.wrench.torque.x, self._ft.wrench.torque.y, self._ft.wrench.torque.z]
              if self._ft else [0.0] * 6)
        action_pose = self._pose_vec(self._action) if self._action else [0.0] * 7
        frame = {
            "observation.state": list(js.position),
            "observation.ee_pose": ee,
            "observation.ft": ft,
            "observation.gripper": [self._grip],
            "observation.images.scene": _img_to_np(color),
            "observation.depth.scene": _img_to_np(depth),
            "action": action_pose + [self._grip],
            "timestamp": self.get_clock().now().nanoseconds * 1e-9,
            "frame_index": len(self.frames),
            "episode_index": self.episode_index,
        }
        self.frames.append(frame)


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
