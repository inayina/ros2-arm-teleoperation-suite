"""Write buffered M6 episodes in LeRobot v2.1 (MP4 + parquet) layout."""
import json
import os
import time
from pathlib import Path

import numpy as np

from .lerobot_v21_dataset import (
    FORMAT_ARROW,
    FORMAT_V21,
    append_episode,
    ffmpeg_available,
)
from .video_encode import DEFAULT_FPS

try:
    import pyarrow as pa
    import pyarrow.parquet as pq

    _HAS_PYARROW = True
except Exception:
    pa = None
    pq = None
    _HAS_PYARROW = False

try:
    from datasets import Array2D, Array3D, Dataset, Features, Sequence, Value

    _HAS_DATASETS = True
except Exception:
    _HAS_DATASETS = False


def _episode_features(first_frame: dict) -> "Features":
    features = {
        "observation.state": Sequence(Value("float32"), length=7),
        "observation.ee_pose": Sequence(Value("float32"), length=7),
        "observation.object_pose": Sequence(Value("float32"), length=7),
        "observation.ft": Sequence(Value("float32"), length=6),
        "observation.gripper": Sequence(Value("float32"), length=1),
        "action": Sequence(Value("float32"), length=8),
        "timestamp": Value("float64"),
        "episode_index": Value("int64"),
        "frame_index": Value("int64"),
        "done": Value("bool"),
        "task": Value("string"),
        "language_instruction": Value("string"),
        "task_phase": Value("string"),
        "success": Value("bool"),
        "safety_estop": Value("bool"),
        "drive_fault": Value("bool"),
    }
    for key in ("observation.images.scene", "observation.images.wrist"):
        if key in first_frame:
            image = np.asarray(first_frame[key])
            features[key] = Array3D(
                dtype="uint8", shape=(int(image.shape[0]), int(image.shape[1]), 3)
            )
    return Features(features)


def _normalize_frame(frame: dict) -> dict:
    normalized = dict(frame)
    for key in ("observation.images.scene", "observation.images.wrist"):
        if key in frame:
            normalized[key] = np.asarray(frame[key], dtype=np.uint8)
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
    normalized["language_instruction"] = str(frame["language_instruction"])
    normalized["task_phase"] = str(frame.get("task_phase", "UNAVAILABLE"))
    normalized["success"] = bool(frame.get("success", True))
    normalized["safety_estop"] = bool(frame["safety_estop"])
    normalized["drive_fault"] = bool(frame["drive_fault"])
    return normalized


def _filter_shape_consistent_frames(frames: list[dict]) -> list[dict]:
    from collections import Counter

    image_keys = tuple(
        key for key in ("observation.images.scene", "observation.images.wrist")
        if key in frames[0]
    )
    if not image_keys:
        return frames
    most_common_shapes = {
        key: Counter(
            np.asarray(frame[key]).shape for frame in frames if key in frame
        ).most_common(1)[0][0]
        for key in image_keys
    }
    return [
        frame
        for frame in frames
        if all(
            key in frame and np.asarray(frame[key]).shape == most_common_shapes[key]
            for key in image_keys
        )
    ]


def _validate_capture_fps(frames: list[dict], fps: float, tolerance: float = 0.30) -> None:
    if fps <= 0.0:
        raise ValueError("fps must be positive")
    if len(frames) < 3:
        return
    stamps = np.asarray([float(frame["timestamp"]) for frame in frames], dtype=np.float64)
    deltas = np.diff(stamps)
    deltas = deltas[np.isfinite(deltas) & (deltas > 0.0)]
    if deltas.size < 2:
        raise ValueError("timestamps must be strictly increasing to validate capture fps")
    measured_fps = 1.0 / float(np.median(deltas))
    if abs(measured_fps - fps) / fps > tolerance:
        raise ValueError(
            f"timestamp median rate {measured_fps:.3f} Hz is inconsistent with fps={fps:.3f}"
        )


def _write_arrow_episode(train_dir: str, normalized: list[dict]) -> None:
    if _HAS_DATASETS:
        ds = Dataset.from_list(normalized, features=_episode_features(normalized[0]))
        ds.save_to_disk(train_dir)
        return

    flat = {}
    for key in normalized[0].keys():
        try:
            flat[key] = np.asarray([frame[key] for frame in normalized])
        except Exception:
            flat[key] = np.asarray([frame[key] for frame in normalized], dtype=object)
    os.makedirs(train_dir, exist_ok=True)
    np.savez_compressed(os.path.join(train_dir, "frames.npz"), **flat)


def write_episode(
    out_dir: str,
    episode_index: int,
    frames: list,
    task: str = "teleop",
    success: bool = True,
    metadata: dict | None = None,
    *,
    fps: float = DEFAULT_FPS,
    prefer_video: bool = True,
) -> str:
    """Write one episode and return the loadable dataset path."""
    if not frames:
        raise ValueError("cannot write an empty episode")

    filtered = _filter_shape_consistent_frames(frames)
    if not filtered:
        raise ValueError("no frames left after shape consistency filtering")

    for frame in filtered:
        frame["success"] = bool(success)
    filtered[-1]["done"] = True
    normalized = [_normalize_frame(frame) for frame in filtered]
    _validate_capture_fps(normalized, fps)

    dataset_root = Path(out_dir)
    use_v21 = _HAS_PYARROW and (not prefer_video or ffmpeg_available())
    episode_metadata = dict(metadata or {})
    if "upstream_gate" not in episode_metadata:
        episode_metadata["upstream_gate"] = "teleop"

    if use_v21:
        parquet_path = append_episode(
            dataset_root,
            episode_index,
            normalized,
            task=task,
            success=success,
            metadata=episode_metadata,
            fps=fps,
        )
        return str(parquet_path)

    ep_dir = os.path.join(out_dir, f"episode_{episode_index:06d}")
    train_dir = os.path.join(ep_dir, "train")
    _write_arrow_episode(train_dir, normalized)
    upstream_gate = str(episode_metadata.pop("upstream_gate", "teleop"))
    promoted = {
        key: episode_metadata.pop(key)
        for key in (
            "visual_streams",
            "capture_fps",
            "action_type",
            "action_semantics",
            "action_sources",
            "simulator_backend",
            "simulator_version",
            "scene_id",
        )
        if key in episode_metadata
    }
    with open(os.path.join(ep_dir, "meta.json"), "w", encoding="utf-8") as fh:
        json.dump(
            {
                "task": task,
                "frames": len(normalized),
                "episode_index": episode_index,
                "dataset_path": train_dir,
                "saved_unix_time": time.time(),
                "format": FORMAT_ARROW if _HAS_DATASETS else "npz_fallback",
                "success": bool(success),
                "upstream_gate": upstream_gate,
                **promoted,
                "metadata": episode_metadata,
            },
            fh,
            indent=2,
        )
    return train_dir
