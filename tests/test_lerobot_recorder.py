import unittest
import numpy as np
import os
import json
import tempfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from lerobot_recorder.recorder_node import _pad, _img_to_np, RecorderNode
from lerobot_recorder.lerobot_writer import _normalize_frame, write_episode, _HAS_DATASETS

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
            "observation.depth.scene": np.zeros((10, 10), dtype=np.float32),
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
            "safety_estop": False,
            "drive_fault": False
        }
        
        norm = _normalize_frame(raw_frame)
        
        assert isinstance(norm["observation.images.scene"], np.ndarray)
        assert norm["observation.images.scene"].dtype == np.uint8
        assert isinstance(norm["observation.images.wrist"], np.ndarray)
        assert norm["observation.images.wrist"].dtype == np.uint8
        assert isinstance(norm["observation.depth.scene"], np.ndarray)
        assert norm["observation.depth.scene"].dtype == np.float32
        
        assert isinstance(norm["observation.state"], list)
        assert isinstance(norm["observation.ee_pose"], list)
        assert isinstance(norm["observation.ft"], list)
        
        assert isinstance(norm["episode_index"], int)
        assert norm["episode_index"] == 5
        assert isinstance(norm["done"], bool)
        assert norm["done"] is False

    def test_write_episode(self):
        # Create 3 dummy frames
        frames = []
        for i in range(3):
            frames.append({
                "observation.images.scene": np.ones((5, 5, 3), dtype=np.uint8) * i,
                "observation.images.wrist": np.ones((3, 3, 3), dtype=np.uint8) * i,
                "observation.depth.scene": np.ones((5, 5), dtype=np.float32) * i,
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
                "safety_estop": False,
                "drive_fault": False
            })
            
        with tempfile.TemporaryDirectory() as temp_dir:
            out_path = write_episode(temp_dir, 42, frames, task="test_task")
            
            # Verify directory structure
            ep_dir = os.path.join(temp_dir, "episode_000042")
            assert os.path.exists(ep_dir)
            assert os.path.exists(out_path)
            
            # Verify the last frame was marked as done
            assert frames[-1]["done"] is True
            
            # Verify meta.json
            meta_path = os.path.join(ep_dir, "meta.json")
            assert os.path.exists(meta_path)
            
            with open(meta_path, "r", encoding="utf-8") as f:
                meta = json.load(f)
                
            assert meta["task"] == "test_task"
            assert meta["frames"] == 3
            assert meta["episode_index"] == 42
            assert "saved_unix_time" in meta
            
            if _HAS_DATASETS:
                assert meta["format"] == "huggingface_dataset"
                assert os.path.exists(os.path.join(out_path, "state.json")) or os.path.exists(os.path.join(out_path, "dataset_info.json"))
            else:
                assert meta["format"] == "npz_fallback"
                assert os.path.exists(os.path.join(out_path, "frames.npz"))
