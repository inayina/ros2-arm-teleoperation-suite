"""Write buffered M6 episodes in a LeRobot/HuggingFace-loadable layout."""
import json
import os
import time

import numpy as np

try:
    from datasets import Array2D, Array3D, Dataset, Features, Sequence, Value

    _HAS_DATASETS = True
except Exception:
    _HAS_DATASETS = False


def _episode_features(first_frame: dict) -> "Features":
    rgb = np.asarray(first_frame["observation.images.scene"])
    wrist_rgb = np.asarray(first_frame["observation.images.wrist"])
    depth = np.asarray(first_frame["observation.depth.scene"])
    return Features({
        "observation.state": Sequence(Value("float32"), length=7),
        "observation.ee_pose": Sequence(Value("float32"), length=7),
        "observation.object_pose": Sequence(Value("float32"), length=7),
        "observation.ft": Sequence(Value("float32"), length=6),
        "observation.gripper": Sequence(Value("float32"), length=1),
        "observation.images.scene": Array3D(
            dtype="uint8", shape=(int(rgb.shape[0]), int(rgb.shape[1]), 3)
        ),
        "observation.images.wrist": Array3D(
            dtype="uint8", shape=(int(wrist_rgb.shape[0]), int(wrist_rgb.shape[1]), 3)
        ),
        "observation.depth.scene": Array2D(
            dtype="float32", shape=(int(depth.shape[0]), int(depth.shape[1]))
        ),
        "action": Sequence(Value("float32"), length=8),
        "timestamp": Value("float64"),
        "episode_index": Value("int64"),
        "frame_index": Value("int64"),
        "done": Value("bool"),
        "task": Value("string"),
        "safety_estop": Value("bool"),
        "drive_fault": Value("bool"),
    })


def _normalize_frame(frame: dict) -> dict:
    normalized = dict(frame)
    normalized["observation.images.scene"] = np.asarray(
        frame["observation.images.scene"], dtype=np.uint8
    )
    normalized["observation.images.wrist"] = np.asarray(
        frame["observation.images.wrist"], dtype=np.uint8
    )
    normalized["observation.depth.scene"] = np.asarray(
        frame["observation.depth.scene"], dtype=np.float32
    )
    for key in (
        "observation.state",
        "observation.ee_pose",
        "observation.object_pose",
        "observation.ft",
        "observation.gripper",
        "action",
    ):
        normalized[key] = [float(v) for v in frame[key]]
    normalized["timestamp"] = float(frame["timestamp"])
    normalized["episode_index"] = int(frame["episode_index"])
    normalized["frame_index"] = int(frame["frame_index"])
    normalized["done"] = bool(frame["done"])
    normalized["task"] = str(frame["task"])
    normalized["safety_estop"] = bool(frame["safety_estop"])
    normalized["drive_fault"] = bool(frame["drive_fault"])
    return normalized


def write_episode(out_dir: str, episode_index: int, frames: list, task: str = "teleop") -> str:
    """Write one episode and return the loadable dataset path."""
    if not frames:
        raise ValueError("cannot write an empty episode")

    os.makedirs(out_dir, exist_ok=True)
    ep_dir = os.path.join(out_dir, f"episode_{episode_index:06d}")
    os.makedirs(ep_dir, exist_ok=True)
    train_dir = os.path.join(ep_dir, "train")

    frames[-1]["done"] = True
    normalized = [_normalize_frame(frame) for frame in frames]

    if _HAS_DATASETS:
        ds = Dataset.from_list(normalized, features=_episode_features(normalized[0]))
        ds.save_to_disk(train_dir)
    else:
        flat = {}
        for key in normalized[0].keys():
            try:
                flat[key] = np.asarray([f[key] for f in normalized])
            except Exception:
                flat[key] = np.asarray([f[key] for f in normalized], dtype=object)
        os.makedirs(train_dir, exist_ok=True)
        np.savez_compressed(os.path.join(train_dir, "frames.npz"), **flat)

    with open(os.path.join(ep_dir, "meta.json"), "w", encoding="utf-8") as fh:
        json.dump(
            {
                "task": task,
                "frames": len(normalized),
                "episode_index": episode_index,
                "dataset_path": train_dir,
                "saved_unix_time": time.time(),
                "format": "huggingface_dataset" if _HAS_DATASETS else "npz_fallback",
            },
            fh,
            indent=2,
        )
    return train_dir
