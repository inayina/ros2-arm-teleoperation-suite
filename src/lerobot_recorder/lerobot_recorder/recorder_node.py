#!/usr/bin/env python3
"""L7 multi-modal recorder for M6 LeRobot episodes."""
from pathlib import Path
import time
import json

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from teleop_interfaces.msg import (
    DriveStatus,
    DriveStatusArray,
    SafetyStatus,
    TaskEvaluationStatus,
)
from teleop_interfaces.srv import EndEpisode

from .lerobot_writer import write_episode
from .time_sync import MultiModalSync


def _img_to_np(msg: Image) -> np.ndarray:
    if msg.encoding == "rgb8":
        # 仿真相机原生发布 rgb8 编码，避开非连续切片逆序重排。
        # 此时 NumPy copy() 执行的是底层 C 级别的连续内存 memcpy，效率极高。
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
    if msg.encoding == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return arr[:, :, ::-1].copy()  # 仅在确为 BGR 时执行非连续重排深拷贝
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


ACTION_SOURCE_TELEOP = "teleop_command"
ACTION_SOURCE_HOLD = "hold_from_ee"


def resolve_frame_action(
    *,
    command_pose: PoseStamped | None,
    grip_cmd: float | None,
    ee_pose: PoseStamped,
    grip_state: float,
    batch_gate: bool,
) -> tuple[list[float], str] | None:
    """Build action[8] for a synced frame.

    Training batch (`upstream_gate=batch_generator`) still requires a real
    `/teleop/cmd_pose` + `/teleop/gripper_cmd`. Portfolio / teleop camera
    capture may run with `start_teleop:=false`; dropping those frames silently
    produced 0-frame episodes while RGB bags succeeded. Hold fill uses EE pose
    + gripper *state* and must be tagged in episode metadata so it is not
    treated as expert command.
    """
    if command_pose is not None and grip_cmd is not None:
        return RecorderNode._pose_vec(command_pose) + [float(grip_cmd)], ACTION_SOURCE_TELEOP
    if batch_gate:
        return None
    return RecorderNode._pose_vec(ee_pose) + [float(grip_state)], ACTION_SOURCE_HOLD


class RecorderNode(Node):
    def __init__(self):
        super().__init__("lerobot_recorder")
        self.declare_parameter("output_dir", "data/episodes")
        self.declare_parameter("task", "teleop")
        self.declare_parameter("language_instruction", "pick up the target object")
        self.declare_parameter("upstream_gate", "teleop")
        self.declare_parameter("sync_queue_size", 30)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("auto_record_seconds", 0.0)
        self.declare_parameter("auto_record_delay_s", 0.0)
        self.declare_parameter("capture_mode", "portfolio")
        self.declare_parameter("enable_wrist_camera", True)
        self.declare_parameter("expected_frame_rate_hz", 10.0)
        self.declare_parameter("close_command_threshold", 0.12)
        self.declare_parameter("require_close_command_for_batch", True)
        self.declare_parameter("require_task_phase_for_batch", False)
        self.declare_parameter("simulator_backend", "")
        self.declare_parameter("simulator_version", "")
        self.declare_parameter("scene_id", "")
        self.out_dir = self.get_parameter("output_dir").value
        self.task = self.get_parameter("task").value
        self.capture_mode = str(self.get_parameter("capture_mode").value)
        if self.capture_mode not in {"training", "portfolio"}:
            raise ValueError("capture_mode must be training or portfolio")

        from .lerobot_v21_dataset import next_episode_index

        self.episode_index = next_episode_index(Path(self.out_dir))
        self.recording = False
        self.frames = []
        self._current_episode_metadata = {}
        self._frame_task_phases = []
        self._task_phase = "UNAVAILABLE"
        self._task_phase_valid = False

        self._grip = 0.0
        self._grip_cmd = None
        self._action = None
        self._safety_estop = False
        self._drive_fault = False
        self._close_command_seen = False
        self._last_writer_s = 0.0
        self._last_writer_error = ""
        self._command_missing_rejects = 0
        self._episode_used_hold_action = False
        self._last_command_warn_s = 0.0
        self._last_commit_error = ""
        self._record_started = time.perf_counter()

        self.create_subscription(Float64, "/gripper/state", self._on_grip, 10)
        self.create_subscription(Float64, "/teleop/gripper_cmd", self._on_grip_cmd, 10)
        self.create_subscription(PoseStamped, "/teleop/cmd_pose", self._on_action, 10)
        self.create_subscription(SafetyStatus, "/safety/status", self._on_safety, 10)
        self.create_subscription(DriveStatusArray, "/servo_drive/status", self._on_drive_status, 10)
        self.create_subscription(String, "/teleop/record_trigger", self._on_trigger, 10)
        self.create_subscription(
            TaskEvaluationStatus,
            "/task/evaluation_status",
            self._on_task_gt,
            10,
        )
        self.srv_end_episode = self.create_service(
            EndEpisode, "/lerobot_recorder/end_episode", self._on_end_episode
        )

        self.sync = MultiModalSync(
            self,
            self._on_frame,
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop").value),
            include_images=self.capture_mode == "portfolio",
            visual_keys=(
                (("color", "wrist") if bool(
                    self.get_parameter("enable_wrist_camera").value
                ) else ("color",))
                if self.capture_mode == "portfolio" else ()
            ),
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/recorder/diagnostics", 10)
        self._diagnostics_timer = self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f"lerobot_recorder up (output={self.out_dir}, capture_mode={self.capture_mode}).")
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
    def _on_grip_cmd(self, m):
        self._grip_cmd = float(m.data)
        if self.recording and self._grip_cmd <= float(
            self.get_parameter("close_command_threshold").value
        ):
            self._close_command_seen = True
    def _on_action(self, m): self._action = m
    def _on_safety(self, m): self._safety_estop = bool(m.estop_active)

    def _on_task_gt(self, msg: TaskEvaluationStatus):
        valid_source = (
            msg.validity == "VALID"
            and msg.gt_source == "upstream_continuous_task_evaluator"
            and bool(msg.phase)
        )
        self._task_phase_valid = valid_source
        self._task_phase = str(msg.phase) if valid_source else "UNAVAILABLE"

    def _on_drive_status(self, msg: DriveStatusArray):
        self._drive_fault = any(
            d.ds402_state == DriveStatus.STATE_FAULT or d.fault_code != 0
            for d in msg.drives
        )

    def _on_trigger(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "start" and not self.recording:
            self._start_recording()
        elif cmd in ("stop", "stop_success", "success") and self.recording:
            self._stop_recording(success=True)
        elif cmd in ("stop_failed", "failed") and self.recording:
            self._stop_recording(success=False)
        elif cmd in ("discard", "abort", "rollback") and self.recording:
            self._discard_recording(reason=cmd)

    def _start_recording(self):
        self.frames = []
        self._frame_task_phases = []
        self._current_episode_metadata = {}
        self._close_command_seen = False
        self._command_missing_rejects = 0
        self._episode_used_hold_action = False
        self._last_commit_error = ""
        self.recording = True
        self._record_started = time.perf_counter()
        self.get_logger().info(f"recording episode {self.episode_index} ...")

    def _stop_recording(self, success: bool = True):
        self.recording = False
        if not self.frames:
            self._last_commit_error = "no synchronized frames"
            self.get_logger().warn("recording stopped without synchronized frames")
            self._current_episode_metadata = {}
            return None
        if (
            str(self.get_parameter("upstream_gate").value) == "batch_generator"
            and bool(self.get_parameter("require_close_command_for_batch").value)
            and not self._close_command_seen
        ):
            self._last_commit_error = "no close gripper command captured"
            self.get_logger().error(
                "rejecting batch episode: no close gripper command captured")
            self.frames = []
            self._current_episode_metadata = {}
            return None
        if (
            str(self.get_parameter("upstream_gate").value) == "batch_generator"
            and bool(self.get_parameter("require_task_phase_for_batch").value)
            and (
                len(self._frame_task_phases) != len(self.frames)
                or any(phase == "UNAVAILABLE" for phase in self._frame_task_phases)
            )
        ):
            self._last_commit_error = "missing valid upstream Task GT phase"
            self.get_logger().error(
                "rejecting batch episode: frame-level Task GT phase incomplete"
            )
            self.frames = []
            self._frame_task_phases = []
            self._current_episode_metadata = {}
            return None
        metadata = dict(self._current_episode_metadata)
        metadata["stop_trigger_success"] = bool(success)
        metadata["upstream_gate"] = str(self.get_parameter("upstream_gate").value)
        enabled_visual_streams = ["scene"]
        if bool(self.get_parameter("enable_wrist_camera").value):
            enabled_visual_streams.append("wrist")
        expected_fps = float(self.get_parameter("expected_frame_rate_hz").value)
        metadata.update({
            "visual_streams": (
                enabled_visual_streams if self.capture_mode == "portfolio" else []
            ),
            "capture_fps": expected_fps,
            "action_type": "ee_pose_gripper",
            "action_semantics": "ee_pose_gripper_cmd_v1",
            "action_sources": {
                "pose": (
                    "/ee_pose" if self._episode_used_hold_action else "/teleop/cmd_pose"
                ),
                "gripper_command": (
                    "/gripper/state"
                    if self._episode_used_hold_action else "/teleop/gripper_cmd"
                ),
                "gripper_observation": "/gripper/state",
            },
            "command_missing": bool(self._episode_used_hold_action),
            "action_fill": (
                ACTION_SOURCE_HOLD if self._episode_used_hold_action else ACTION_SOURCE_TELEOP
            ),
            "task_phase_contract_version": "panda_train_frame_phase_v1",
            "task_phase_source": "upstream_continuous_task_evaluator",
            "task_phase_semantics": "continuous_gt_achieved_subgoal_frontier",
            "task_phases": list(self._frame_task_phases),
        })
        for provenance_key in (
            "simulator_backend",
            "simulator_version",
            "scene_id",
        ):
            provenance_value = str(
                self.get_parameter(provenance_key).value
            ).strip()
            if provenance_value:
                metadata[provenance_key] = provenance_value
        write_started = time.perf_counter()
        try:
            path = write_episode(
                self.out_dir,
                self.episode_index,
                self.frames,
                self.task,
                success=success,
                metadata=metadata,
                fps=expected_fps,
                prefer_video=self.capture_mode == "portfolio",
            )
            self._last_writer_error = ""
        except Exception as exc:
            self._last_commit_error = f"writer error: {exc}"
            self._last_writer_error = str(exc)
            self.get_logger().error(f"episode write failed: {exc}")
            self.frames = []
            self._current_episode_metadata = {}
            return None
        write_s = time.perf_counter() - write_started
        self._last_writer_s = write_s
        elapsed_s = time.perf_counter() - self._record_started
        self.get_logger().info(
            f"episode_profile frames={len(self.frames)} capture_mode={self.capture_mode} "
            f"record_wall_s={elapsed_s:.3f} write_s={write_s:.3f} "
            f"effective_hz={len(self.frames) / max(elapsed_s, 1e-9):.3f}")
        self.get_logger().info(f"saved {len(self.frames)} frames -> {path}")
        self.episode_index += 1
        self.frames = []
        self._frame_task_phases = []
        self._current_episode_metadata = {}
        self._last_commit_error = ""
        return path

    def _on_end_episode(self, request, response):
        frame_count = len(self.frames)
        if request.discard:
            self._discard_recording(reason="end_episode_service")
            response.success = True
            response.message = "episode discarded"
            response.dataset_path = ""
            response.frame_count = frame_count
            return response

        if not self.recording and frame_count == 0:
            response.success = False
            response.message = "no active recording to commit"
            response.dataset_path = ""
            response.frame_count = 0
            return response

        path = self._stop_recording(success=True)
        if path:
            response.success = True
            response.message = "episode committed"
            response.dataset_path = str(path)
            response.frame_count = frame_count
        else:
            response.success = False
            response.message = f"commit failed: {self._last_commit_error or 'unknown error'}"
            response.dataset_path = ""
            response.frame_count = frame_count
        return response

    def _discard_recording(self, reason: str = "discard"):
        frame_count = len(self.frames)
        self.frames = []
        self._frame_task_phases = []
        self.recording = False
        self._current_episode_metadata = {}
        self.get_logger().warn(
            f"discarded episode {self.episode_index} ({frame_count} buffered frames, reason={reason})"
        )

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
        wrist_color: Image,
        obj_msg: PoseStamped,
    ):
        if not self.recording:
            return
        batch_gate = str(self.get_parameter("upstream_gate").value) == "batch_generator"
        resolved = resolve_frame_action(
            command_pose=self._action,
            grip_cmd=self._grip_cmd,
            ee_pose=ee_msg,
            grip_state=float(self._grip),
            batch_gate=batch_gate,
        )
        if resolved is None:
            self._command_missing_rejects += 1
            self._warn_command_missing(dropped=True)
            return
        action_pose, action_source = resolved
        if action_source == ACTION_SOURCE_HOLD:
            self._command_missing_rejects += 1
            self._episode_used_hold_action = True
            self._warn_command_missing(dropped=False)
        ft = [
            ft_msg.wrench.force.x,
            ft_msg.wrench.force.y,
            ft_msg.wrench.force.z,
            ft_msg.wrench.torque.x,
            ft_msg.wrench.torque.y,
            ft_msg.wrench.torque.z,
        ]
        frame = {
            "observation.state": _pad(list(js.position), 7),
            "observation.ee_pose": self._pose_vec(ee_msg),
            "observation.object_pose": self._pose_vec(obj_msg),
            "observation.ft": ft,
            "observation.gripper": [float(self._grip)],
            "action": action_pose,
            "timestamp": _stamp_sec(color if color is not None else js),
            "frame_index": len(self.frames),
            "episode_index": self.episode_index,
            "done": False,
            "task": self.task,
            "language_instruction": str(self.get_parameter("language_instruction").value),
            "success": True,
            "safety_estop": self._safety_estop,
            "drive_fault": self._drive_fault,
            "task_phase": self._task_phase if self._task_phase_valid else "UNAVAILABLE",
        }
        if self.capture_mode == "portfolio":
            frame["observation.images.scene"] = _img_to_np(color)
            if bool(self.get_parameter("enable_wrist_camera").value) and wrist_color is not None:
                frame["observation.images.wrist"] = _img_to_np(wrist_color)
        self.frames.append(frame)
        self._frame_task_phases.append(frame["task_phase"])

    def _warn_command_missing(self, *, dropped: bool) -> None:
        now = time.perf_counter()
        last = float(getattr(self, "_last_command_warn_s", 0.0))
        if now - last < 5.0:
            return
        self._last_command_warn_s = now
        if dropped:
            self.get_logger().warn(
                "dropping synced frame: /teleop/cmd_pose or /teleop/gripper_cmd "
                "unseen (batch_generator requires expert command)"
            )
            return
        self.get_logger().warn(
            "teleop command unseen; filling action from /ee_pose + /gripper/state "
            "(hold_from_ee, not expert command)"
        )

    def _publish_diagnostics(self):
        elapsed = max(time.perf_counter() - self._record_started, 1e-9)
        sync_diag = self.sync.diagnostics_snapshot()
        status = DiagnosticStatus()
        status.name = "lerobot_recorder/health"
        status.hardware_id = "scene_only_capture"
        status.level = (
            DiagnosticStatus.ERROR if self._last_writer_error else DiagnosticStatus.OK
        )
        status.message = (
            self._last_writer_error or ("recording" if self.recording else "idle")
        )
        values = {
            "recording": self.recording,
            "buffered_frames": len(self.frames),
            "effective_hz": len(self.frames) / elapsed if self.recording else 0.0,
            "enabled_visual_streams": sync_diag["enabled_visual_streams"],
            "scene_age_s": sync_diag["ages_s"].get("color", -1.0),
            "missing_rejects": sync_diag["reject_counts"]["missing"],
            "stale_rejects": sync_diag["reject_counts"]["stale"],
            "reused_rejects": sync_diag["reject_counts"]["reused"],
            "command_missing_rejects": self._command_missing_rejects,
            "action_fill": (
                ACTION_SOURCE_HOLD if self._episode_used_hold_action else ACTION_SOURCE_TELEOP
            ),
            "gripper_command": (
                self._grip_cmd if self._grip_cmd is not None else "unseen"
            ),
            "gripper_state": self._grip,
            "close_command_seen": self._close_command_seen,
            "last_writer_s": self._last_writer_s,
            "last_writer_error": self._last_writer_error,
        }
        status.values = [
            KeyValue(
                key=str(key),
                value=(
                    json.dumps(value)
                    if isinstance(value, (bool, list, dict)) else str(value)
                ),
            )
            for key, value in values.items()
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diagnostics_pub.publish(msg)


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
