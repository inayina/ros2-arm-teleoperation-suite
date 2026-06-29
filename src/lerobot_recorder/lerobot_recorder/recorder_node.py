#!/usr/bin/env python3
"""L7 multi-modal recorder for M6 LeRobot episodes."""
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64, String
from teleop_interfaces.msg import DriveStatus, DriveStatusArray, SafetyStatus

from .lerobot_writer import write_episode
from .time_sync import MultiModalSync


def _img_to_np(msg: Image) -> np.ndarray:
    if msg.encoding in ("rgb8", "bgr8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        return arr.copy()
    if msg.encoding == "16UC1":
        depth_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return (depth_mm.astype(np.float32) * 0.001).copy()
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def _pad(values, length: int) -> list[float]:
    out = [float(v) for v in values[:length]]
    out.extend([0.0] * (length - len(out)))
    return out


def _stamp_sec(msg) -> float:
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RecorderNode(Node):
    def __init__(self):
        super().__init__("lerobot_recorder")
        self.declare_parameter("output_dir", "data/episodes")
        self.declare_parameter("task", "teleop")
        self.declare_parameter("sync_queue_size", 30)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("auto_record_seconds", 0.0)
        self.declare_parameter("auto_record_delay_s", 0.0)
        self.out_dir = self.get_parameter("output_dir").value
        self.task = self.get_parameter("task").value

        self.recording = False
        self.episode_index = 0
        self.frames = []

        self._grip = 0.0
        self._action = None
        self._safety_estop = False
        self._drive_fault = False

        self.create_subscription(Float64, "/gripper/state", self._on_grip, 10)
        self.create_subscription(PoseStamped, "/teleop/cmd_pose", self._on_action, 10)
        self.create_subscription(SafetyStatus, "/safety/status", self._on_safety, 10)
        self.create_subscription(DriveStatusArray, "/servo_drive/status", self._on_drive_status, 10)
        self.create_subscription(String, "/teleop/record_trigger", self._on_trigger, 10)

        self.sync = MultiModalSync(
            self,
            self._on_frame,
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop").value),
        )

        self.get_logger().info(f"lerobot_recorder up (output={self.out_dir}).")
        auto_seconds = float(self.get_parameter("auto_record_seconds").value)
        auto_delay = float(self.get_parameter("auto_record_delay_s").value)
        self._auto_start_timer = None
        self._auto_stop_timer = None
        if auto_seconds > 0.0:
            self._auto_record_seconds = auto_seconds
            self._auto_start_timer = self.create_timer(
                max(0.1, auto_delay),
                self._auto_start_recording,
            )

    def _on_grip(self, m): self._grip = m.data
    def _on_action(self, m): self._action = m
    def _on_safety(self, m): self._safety_estop = bool(m.estop_active)

    def _on_drive_status(self, msg: DriveStatusArray):
        self._drive_fault = any(
            d.ds402_state == DriveStatus.STATE_FAULT or d.fault_code != 0
            for d in msg.drives
        )

    def _on_trigger(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "start" and not self.recording:
            self._start_recording()
        elif cmd == "stop" and self.recording:
            self._stop_recording()

    def _start_recording(self):
        self.frames = []
        self.recording = True
        self.get_logger().info(f"recording episode {self.episode_index} ...")

    def _stop_recording(self):
        self.recording = False
        if self.frames:
            path = write_episode(self.out_dir, self.episode_index, self.frames, self.task)
            self.get_logger().info(f"saved {len(self.frames)} frames -> {path}")
            self.episode_index += 1
        else:
            self.get_logger().warn("recording stopped without synchronized frames")

    def _auto_start_recording(self):
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self.destroy_timer(self._auto_start_timer)
            self._auto_start_timer = None
        if not self.recording:
            self._start_recording()
        self._auto_stop_timer = self.create_timer(
            max(0.1, self._auto_record_seconds),
            self._auto_stop_recording,
        )

    def _auto_stop_recording(self):
        if self._auto_stop_timer is not None:
            self._auto_stop_timer.cancel()
            self.destroy_timer(self._auto_stop_timer)
            self._auto_stop_timer = None
        if self.recording:
            self._stop_recording()

    @staticmethod
    def _pose_vec(p: PoseStamped):
        o = p.pose.orientation
        q = p.pose.position
        return [q.x, q.y, q.z, o.x, o.y, o.z, o.w]

    def _on_frame(
        self,
        js,
        ee_msg: PoseStamped,
        ft_msg: WrenchStamped,
        color: Image,
        depth: Image,
        wrist_color: Image,
        tactile_left: Image,
        tactile_right: Image,
        obj_msg: PoseStamped,
    ):
        if not self.recording:
            return
        ft = [
            ft_msg.wrench.force.x,
            ft_msg.wrench.force.y,
            ft_msg.wrench.force.z,
            ft_msg.wrench.torque.x,
            ft_msg.wrench.torque.y,
            ft_msg.wrench.torque.z,
        ]
        action_pose = self._pose_vec(self._action) if self._action else [0.0] * 7
        frame = {
            "observation.state": _pad(list(js.position), 7),
            "observation.ee_pose": self._pose_vec(ee_msg),
            "observation.object_pose": self._pose_vec(obj_msg),
            "observation.ft": ft,
            "observation.gripper": [float(self._grip)],
            "observation.images.scene": _img_to_np(color),
            "observation.images.wrist": _img_to_np(wrist_color),
            "observation.images.tactile_left": _img_to_np(tactile_left),
            "observation.images.tactile_right": _img_to_np(tactile_right),
            "observation.depth.scene": _img_to_np(depth),
            "action": action_pose + [float(self._grip)],
            "timestamp": _stamp_sec(color),
            "frame_index": len(self.frames),
            "episode_index": self.episode_index,
            "done": False,
            "task": self.task,
            "safety_estop": self._safety_estop,
            "drive_fault": self._drive_fault,
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
