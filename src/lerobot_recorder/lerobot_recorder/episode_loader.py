"""Load upstream Panda episodes (Arrow legacy or LeRobot v2.1)."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Any

import numpy as np

from .lerobot_v21_dataset import (
    DEPTH_KEY,
    FORMAT_ARROW,
    FORMAT_V21,
    RGB_IMAGE_KEYS,
    dataset_root,
    depth_episode_path,
    detect_dataset_format,
    list_episode_indices,
    load_dataset_parquet_rows,
    load_parquet_rows,
    sidecar_meta_path,
    validate_episode as validate_v21_episode,
    video_episode_path,
)
from .video_encode import ffprobe_frame_count

FORMAT_VIDEO = FORMAT_V21  # backward-compatible alias


def read_episode_meta(train_dir: Path) -> dict[str, Any]:
    root = dataset_root(train_dir)
    if train_dir.name.startswith("episode_") and (train_dir / "meta.json").is_file():
        return json.loads((train_dir / "meta.json").read_text(encoding="utf-8"))
    for episode_index in reversed(list_episode_indices(root)):
        sidecar = sidecar_meta_path(root, episode_index)
        if sidecar.is_file():
            return json.loads(sidecar.read_text(encoding="utf-8"))
    return {}


def detect_episode_format(train_dir: Path) -> str:
    return detect_dataset_format(train_dir)


def load_episode_rows(train_dir: Path, *, decode_videos: bool = False) -> list[dict[str, Any]]:
    root = dataset_root(train_dir)
    episode_format = detect_dataset_format(root)
    if episode_format == FORMAT_V21:
        if train_dir.name.startswith("episode_") and "_" in train_dir.name:
            episode_index = int(train_dir.name.split("_", 1)[1])
            rows = load_parquet_rows(root, episode_index)
        else:
            rows = load_dataset_parquet_rows(root)
        return attach_video_modalities(root, rows, decode_videos=decode_videos)

    try:
        from datasets import load_from_disk
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("HuggingFace datasets is required to read Arrow episodes") from exc

    path = train_dir
    if path.name == "train":
        path = path
    elif (path / "train").is_dir():
        path = path / "train"
    dataset = load_from_disk(str(path))
    return [dict(row) for row in dataset]


def attach_video_modalities(
    root: Path,
    rows: list[dict[str, Any]],
    *,
    decode_videos: bool,
) -> list[dict[str, Any]]:
    if not rows or not decode_videos:
        return rows

    try:
        import torchvision.io as io
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("torchvision is required to decode MP4 episodes") from exc

    root = dataset_root(root)
    episode_index = int(rows[0]["episode_index"])
    decoded: dict[str, list[np.ndarray]] = {}
    for key in RGB_IMAGE_KEYS:
        path = video_episode_path(root, key, episode_index)
        if path.is_file():
            tensor, _, _ = io.read_video(str(path), pts_unit="sec")
            decoded[key] = [np.asarray(frame, dtype=np.uint8) for frame in tensor]

    depth_stack: list[np.ndarray] | None = None
    depth_file = depth_episode_path(root, episode_index)
    if depth_file.is_file():
        payload = np.load(depth_file)
        depth_stack = [
            np.asarray(frame, dtype=np.float32) * 0.001 for frame in payload["depth_mm"]
        ]

    enriched: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        item = dict(row)
        for key, frames in decoded.items():
            if index < len(frames):
                item[key] = frames[index]
        if depth_stack is not None and index < len(depth_stack):
            item[DEPTH_KEY] = depth_stack[index]
        enriched.append(item)
    return enriched


def validate_video_episode(train_dir: Path, min_frames: int) -> list[str]:
    root = dataset_root(train_dir)
    if train_dir.name.startswith("episode_"):
        episode_index = int(train_dir.name.split("_", 1)[1])
        return validate_v21_episode(root, episode_index, min_frames)
    errors: list[str] = []
    for episode_index in list_episode_indices(root):
        errors.extend(validate_v21_episode(root, episode_index, min_frames))
    return errors
