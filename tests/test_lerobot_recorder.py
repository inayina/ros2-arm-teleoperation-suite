import unittest
import numpy as np
import os
import json
import tempfile
from types import SimpleNamespace
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from lerobot_recorder.recorder_node import _pad, _img_to_np, RecorderNode
from lerobot_recorder.lerobot_writer import (
    _normalize_frame,
    write_episode,
    _HAS_DATASETS,
    _HAS_PYARROW,
    ffmpeg_available,
)
from lerobot_recorder.lerobot_v21_dataset import FORMAT_V21

class TestLeRobotRecorder(unittest.TestCase):
    def test_pad_helper(self):
        # Normal padding
        assert _pad([1.0, 2.0], 5) == [1.0, 2.0, 0.0, 0.0, 0.0]
        # Clipping if longer
        assert _pad([1.0, 2.0, 3.0, 4.0], 2) == [1.0, 2.0]
        # Equal length
        assert _pad([1.0, 2.0], 2) == [1.0, 2.0]
        # Empty input
        assert _pad([], 3) == [0.0, 0.0, 0.0]

    def test_pose_vec_serialization(self):
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 3.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        # Test RecorderNode._pose_vec without full initialization
        vec = RecorderNode._pose_vec(pose)
        assert vec == [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]

    def test_frame_separates_gripper_observation_and_command(self):
        node = RecorderNode.__new__(RecorderNode)
        node.recording = True
        node._action = PoseStamped()
        node._action.pose.orientation.w = 1.0
        node._grip = 0.75
        node._grip_cmd = 0.10
        node._safety_estop = False
        node._drive_fault = False
        node.frames = []
        node.episode_index = 3
        node.task = "test"
        node.capture_mode = "portfolio"
        values = {
            "enable_wrist_camera": False,
            "language_instruction": "pick the object",
        }
        node.get_parameter = lambda name: SimpleNamespace(value=values[name])

        joint = JointState()
        joint.position = [0.0] * 7
        ee = PoseStamped()
        ee.pose.orientation.w = 1.0
        wrench = WrenchStamped()
        obj = PoseStamped()
        obj.pose.orientation.w = 1.0
        image = Image()
        image.width = 2
        image.height = 2
        image.encoding = "rgb8"
        image.data = np.zeros((2, 2, 3), dtype=np.uint8).tobytes()

        node._on_frame(joint, ee, wrench, image, None, obj)

        assert len(node.frames) == 1
        assert node.frames[0]["observation.gripper"] == [0.75]
        assert node.frames[0]["action"][-1] == 0.10
        assert "observation.images.scene" in node.frames[0]
        assert "observation.images.wrist" not in node.frames[0]

    def test_batch_episode_without_close_command_is_rejected(self):
        node = RecorderNode.__new__(RecorderNode)
        node.recording = True
        node.frames = [{"frame_index": 0}]
        node._current_episode_metadata = {}
        node._close_command_seen = False
        node._last_commit_error = ""
        values = {
            "upstream_gate": "batch_generator",
            "require_close_command_for_batch": True,
        }
        node.get_parameter = lambda name: SimpleNamespace(value=values[name])
        node.get_logger = lambda: SimpleNamespace(
            error=lambda _text: None,
            warn=lambda _text: None,
        )

        assert node._stop_recording(success=True) is None
        assert node.frames == []
        assert node._last_commit_error == "no close gripper command captured"

    def test_img_to_np_rgb8(self):
        msg = Image()
        msg.encoding = "rgb8"
        msg.width = 4
        msg.height = 3
        # 4 * 3 * 3 = 36 bytes of data
        data = np.arange(36, dtype=np.uint8)
        msg.data = data.tobytes()
        
        arr = _img_to_np(msg)
        assert arr.shape == (3, 4, 3)
        assert np.array_equal(arr, data.reshape(3, 4, 3))

    def test_img_to_np_bgr8(self):
        msg = Image()
        msg.encoding = "bgr8"
        msg.width = 2
        msg.height = 2
        # 2 * 2 * 3 = 12 bytes
        # In BGR8, the channel order should be reversed (channel index 2 and 0 swapped)
        data = np.array([
            [[1, 2, 3], [4, 5, 6]],
            [[7, 8, 9], [10, 11, 12]]
        ], dtype=np.uint8)
        msg.data = data.tobytes()
        
        arr = _img_to_np(msg)
        # Expected BGR to RGB: swap channel 0 and 2
        expected = np.array([
            [[3, 2, 1], [6, 5, 4]],
            [[9, 8, 7], [12, 11, 10]]
        ], dtype=np.uint8)
        assert arr.shape == (2, 2, 3)
        assert np.array_equal(arr, expected)

    def test_img_to_np_depth_16uc1(self):
        msg = Image()
        msg.encoding = "16UC1"
        msg.width = 2
        msg.height = 2
        # 2 * 2 * 2 = 8 bytes of data (uint16)
        data = np.array([[1000, 2000], [3000, 4000]], dtype=np.uint16)
        msg.data = data.tobytes()
        
        arr = _img_to_np(msg)
        # 16UC1 scale: converts mm to meters (* 0.001)
        expected = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
        assert arr.shape == (2, 2)
        assert np.allclose(arr, expected)

    def test_img_to_np_depth_32fc1(self):
        msg = Image()
        msg.encoding = "32FC1"
        msg.width = 2
        msg.height = 2
        # float32 data
        data = np.array([[1.5, 2.5], [3.5, 4.5]], dtype=np.float32)
        msg.data = data.tobytes()
        
        arr = _img_to_np(msg)
        assert arr.shape == (2, 2)
        assert np.allclose(arr, data)

    def test_img_to_np_invalid(self):
        msg = Image()
        msg.encoding = "invalid"
        with self.assertRaises(ValueError):
            _img_to_np(msg)

    def test_normalize_frame(self):
        # Create a dummy raw frame with various data types
        raw_frame = {
            "observation.images.scene": np.zeros((10, 10, 3), dtype=np.uint8),
            "observation.images.wrist": np.zeros((5, 5, 3), dtype=np.uint8),
            "observation.state": [1, 2, 3, 4, 5, 6, 7],
            "observation.ee_pose": (1, 2, 3, 4, 5, 6, 7),
            "observation.object_pose": (0, 0, 0, 0, 0, 0, 1),
            "observation.ft": np.array([1, 2, 3, 4, 5, 6]),
            "observation.gripper": [0.5],
            "action": [0.1] * 8,
            "timestamp": 12345.678,
            "episode_index": "5",
            "frame_index": 10,
            "done": 0,
            "task": "pick_apple",
            "language_instruction": "pick up the red box and place it in the left bin",
            "safety_estop": False,
            "drive_fault": False
        }
        
        norm = _normalize_frame(raw_frame)
        
        assert isinstance(norm["observation.images.scene"], np.ndarray)
        assert norm["observation.images.scene"].dtype == np.uint8
        assert isinstance(norm["observation.images.wrist"], np.ndarray)
        assert norm["observation.images.wrist"].dtype == np.uint8

        assert isinstance(norm["observation.state"], list)
        assert isinstance(norm["observation.ee_pose"], list)
        assert isinstance(norm["observation.ft"], list)

        assert isinstance(norm["episode_index"], int)
        assert norm["episode_index"] == 5
        assert isinstance(norm["done"], bool)
        assert norm["done"] is False
        assert norm["success"] is True

    def test_write_episode(self):
        # Create 3 dummy frames
        frames = []
        for i in range(3):
            frames.append({
                "observation.images.scene": np.ones((6, 6, 3), dtype=np.uint8) * i,
                "observation.images.wrist": np.ones((4, 4, 3), dtype=np.uint8) * i,
                "observation.state": [0.0] * 7,
                "observation.ee_pose": [0.0] * 7,
                "observation.object_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "observation.ft": [0.0] * 6,
                "observation.gripper": [0.0],
                "action": [0.0] * 8,
                "timestamp": 1000.0 + i,
                "episode_index": 42,
                "frame_index": i,
                "done": False,
                "task": "test_task",
                "language_instruction": "pick up the red box and place it in the left bin",
                "success": True,
                "safety_estop": False,
                "drive_fault": False
            })
            
        with tempfile.TemporaryDirectory() as temp_dir:
            path = write_episode(
                temp_dir,
                42,
                frames,
                task="test_task",
                metadata={"upstream_gate": "batch_generator", "validation_mode": "place"},
                fps=1.0,
            )
            out_path = path
            ep_dir = os.path.join(temp_dir, "episode_000042")
            assert os.path.exists(ep_dir)

            meta_path = os.path.join(ep_dir, "meta.json")
            assert os.path.exists(meta_path)

            with open(meta_path, "r", encoding="utf-8") as f:
                meta = json.load(f)

            assert meta["task"] == "test_task"
            assert meta["frames"] == 3
            assert meta["episode_index"] == 42
            assert meta["success"] is True
            assert meta["upstream_gate"] == "batch_generator"
            assert meta["metadata"]["validation_mode"] == "place"

            if _HAS_PYARROW and ffmpeg_available():
                assert meta["format"] == FORMAT_V21
                assert os.path.exists(out_path)
                assert out_path.endswith(".parquet")
                assert os.path.exists(os.path.join(temp_dir, "meta", "info.json"))
            elif _HAS_DATASETS:
                assert meta["format"] == "huggingface_dataset"
                assert os.path.exists(out_path)
            else:
                assert meta["format"] == "npz_fallback"
                assert os.path.exists(os.path.join(out_path, "frames.npz"))

    def test_write_scene_only_episode_encodes_video_at_capture_fps(self):
        frames = []
        for i in range(3):
            frames.append({
                "observation.images.scene": np.full((240, 320, 3), i, dtype=np.uint8),
                "observation.state": [0.0] * 7,
                "observation.ee_pose": [0.0] * 7,
                "observation.object_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "observation.ft": [0.0] * 6,
                "observation.gripper": [0.8],
                "action": [0.0] * 7 + [0.1],
                "timestamp": 1000.0 + i / 10.0,
                "episode_index": 7,
                "frame_index": i,
                "done": False,
                "task": "scene_only",
                "language_instruction": "pick the object",
                "success": True,
                "safety_estop": False,
                "drive_fault": False,
            })
        with tempfile.TemporaryDirectory() as temp_dir:
            path = write_episode(
                temp_dir,
                7,
                frames,
                fps=10.0,
                metadata={
                    "visual_streams": ["scene"],
                    "capture_fps": 10.0,
                    "action_type": "ee_pose_gripper",
                    "action_semantics": "ee_pose_gripper_cmd_v1",
                    "simulator_backend": "isaac",
                    "simulator_version": "6.0.0.0",
                    "scene_id": "p3_single_panda_red_box_v1",
                },
            )
            meta = json.loads(
                open(os.path.join(temp_dir, "episode_000007", "meta.json")).read())
            assert meta["visual_streams"] == ["scene"]
            assert meta["capture_fps"] == 10.0
            assert meta["action_semantics"] == "ee_pose_gripper_cmd_v1"
            assert meta["simulator_backend"] == "isaac"
            assert meta["simulator_version"] == "6.0.0.0"
            assert meta["scene_id"] == "p3_single_panda_red_box_v1"
            if _HAS_PYARROW and ffmpeg_available():
                scene_video = os.path.join(
                    temp_dir, "videos", "chunk-000",
                    "observation.images.scene", "episode_000007.mp4")
                wrist_video = os.path.join(
                    temp_dir, "videos", "chunk-000",
                    "observation.images.wrist", "episode_000007.mp4")
                assert os.path.exists(path)
                assert os.path.exists(scene_video)
                assert not os.path.exists(wrist_video)
