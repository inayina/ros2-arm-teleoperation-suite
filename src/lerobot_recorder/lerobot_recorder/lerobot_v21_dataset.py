"""LeRobot v2.1 dataset layout helpers for upstream Panda episodes."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Any

import numpy as np

from .video_encode import DEFAULT_FPS, RGB_IMAGE_KEYS, encode_rgb_frames_to_mp4, ffmpeg_available

try:
    import pyarrow as pa
    import pyarrow.parquet as pq
except Exception:  # pragma: no cover
    pa = None
    pq = None

CODEBASE_VERSION = "v2.1"
FORMAT_V21 = "lerobot_v21"
FORMAT_ARROW = "huggingface_dataset"
CHUNK_ID = "chunk-000"
ROBOT_TYPE = "panda"

TABULAR_KEYS = (
    "observation.state",
    "observation.ee_pose",
    "observation.object_pose",
    "observation.ft",
    "observation.gripper",
    "action",
    "timestamp",
    "episode_index",
    "frame_index",
    "done",
    "task",
    "language_instruction",
    "success",
    "safety_estop",
    "drive_fault",
)

DEPTH_KEY = "observation.depth.scene"


def dataset_root(path: Path) -> Path:
    path = path.resolve()
    if (path / "meta" / "info.json").is_file():
        return path
    if path.name.startswith("episode_"):
        return path.parent
    if path.name == "train" and path.parent.name.startswith("episode_"):
        return path.parent.parent
    return path


def parquet_episode_path(root: Path, episode_index: int) -> Path:
    return root / "data" / CHUNK_ID / f"episode_{episode_index:06d}.parquet"


def video_episode_path(root: Path, camera_key: str, episode_index: int) -> Path:
    return root / "videos" / CHUNK_ID / camera_key / f"episode_{episode_index:06d}.mp4"


def depth_episode_path(root: Path, episode_index: int) -> Path:
    return root / "depth" / CHUNK_ID / DEPTH_KEY / f"episode_{episode_index:06d}.npz"




def sidecar_meta_path(root: Path, episode_index: int) -> Path:
    return root / f"episode_{episode_index:06d}" / "meta.json"


def detect_dataset_format(root: Path) -> str:
    root = dataset_root(root)
    info_path = root / "meta" / "info.json"
    if info_path.is_file():
        try:
            info = json.loads(info_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            return FORMAT_V21
        version = str(info.get("codebase_version", ""))
        if version.startswith("v2.1") or version.startswith("v2"):
            return FORMAT_V21
        return FORMAT_V21
    if list((root / "data" / CHUNK_ID).glob("episode_*.parquet")):
        return FORMAT_V21
    if list(root.glob("episode_*/train/data-*.arrow")):
        return FORMAT_ARROW
    if list(root.glob("episode_*/train/dataset_info.json")):
        return FORMAT_ARROW
    return FORMAT_ARROW


def list_episode_indices(root: Path) -> list[int]:
    root = dataset_root(root)
    indices: set[int] = set()
    data_dir = root / "data" / CHUNK_ID
    if data_dir.is_dir():
        for path in data_dir.glob("episode_*.parquet"):
            indices.add(int(path.stem.split("_", 1)[1]))
    episodes_jsonl = root / "meta" / "episodes.jsonl"
    if episodes_jsonl.is_file():
        for line in episodes_jsonl.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            payload = json.loads(line)
            indices.add(int(payload["episode_index"]))
    for path in root.glob("episode_*/meta.json"):
        try:
            indices.add(int(path.parent.name.split("_", 1)[1]))
        except (IndexError, ValueError):
            continue
    return sorted(indices)


def next_episode_index(root: Path) -> int:
    indices = list_episode_indices(root)
    if not indices:
        return 0
    return max(indices) + 1


def _task_index_for_instruction(root: Path, instruction: str) -> int:
    tasks_path = root / "meta" / "tasks.jsonl"
    tasks: list[str] = []
    if tasks_path.is_file():
        for line in tasks_path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                tasks.append(json.loads(line)["task"])
    if instruction in tasks:
        return tasks.index(instruction)
    return len(tasks)


def _global_index_start(root: Path, episode_index: int) -> int:
    total = 0
    for index in list_episode_indices(root):
        if index >= episode_index:
            break
        parquet_file = parquet_episode_path(root, index)
        if parquet_file.is_file() and pq is not None:
            total += pq.read_table(parquet_file).num_rows
    return total


def _build_video_feature(camera_key: str, shape: tuple[int, int, int], fps: float) -> dict[str, Any]:
    height, width, channels = shape
    return {
        "dtype": "video",
        "shape": [height, width, channels],
        "names": ["height", "width", "channel"],
        "info": {
            "video.height": height,
            "video.width": width,
            "video.codec": "h264",
            "video.pix_fmt": "yuv420p",
            "video.fps": fps,
            "video.channels": channels,
            "video.is_depth_map": False,
        },
    }


def build_info_features(normalized: list[dict], fps: float) -> dict[str, dict[str, Any]]:
    first = normalized[0]
    state_dim = len(first["observation.state"])
    action_dim = len(first["action"])
    features: dict[str, dict[str, Any]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": [state_dim],
            "names": [f"joint_{index}" for index in range(state_dim)],
        },
        "observation.ee_pose": {
            "dtype": "float32",
            "shape": [7],
            "names": ["x", "y", "z", "qx", "qy", "qz", "qw"],
        },
        "observation.object_pose": {
            "dtype": "float32",
            "shape": [7],
            "names": ["x", "y", "z", "qx", "qy", "qz", "qw"],
        },
        "observation.ft": {
            "dtype": "float32",
            "shape": [6],
            "names": ["fx", "fy", "fz", "tx", "ty", "tz"],
        },
        "observation.gripper": {
            "dtype": "float32",
            "shape": [1],
            "names": ["opening"],
        },
        "action": {
            "dtype": "float32",
            "shape": [action_dim],
            "names": [f"action_{index}" for index in range(action_dim)],
        },
        "language_instruction": {"dtype": "string", "shape": [1], "names": None},
        "success": {"dtype": "bool", "shape": [1], "names": None},
        "safety_estop": {"dtype": "bool", "shape": [1], "names": None},
        "drive_fault": {"dtype": "bool", "shape": [1], "names": None},
    }
    for key in RGB_IMAGE_KEYS:
        if key not in first:
            continue
        image = np.asarray(first[key], dtype=np.uint8)
        features[key] = _build_video_feature(key, (int(image.shape[0]), int(image.shape[1]), 3), fps)
    return features


def episode_to_parquet_table(
    normalized: list[dict],
    *,
    episode_index: int,
    global_index_start: int,
    fps: float,
    task_index: int,
) -> "pa.Table":
    if pa is None or pq is None:
        raise RuntimeError("pyarrow is required for LeRobot v2.1 episodes")

    num_frames = len(normalized)
    rows = {
        "index": list(range(global_index_start, global_index_start + num_frames)),
        "episode_index": [episode_index] * num_frames,
        "frame_index": [int(frame["frame_index"]) for frame in normalized],
        "timestamp": [float(frame["timestamp"]) for frame in normalized],
        "task_index": [task_index] * num_frames,
        "next.done": [False] * (num_frames - 1) + [True],
        "next.reward": [0.0] * num_frames,
        "task": [str(frame["task"]) for frame in normalized],
        "language_instruction": [str(frame["language_instruction"]) for frame in normalized],
        "success": [bool(frame["success"]) for frame in normalized],
        "safety_estop": [bool(frame["safety_estop"]) for frame in normalized],
        "drive_fault": [bool(frame["drive_fault"]) for frame in normalized],
    }
    for key in (
        "observation.state",
        "observation.ee_pose",
        "observation.object_pose",
        "observation.ft",
        "observation.gripper",
        "action",
    ):
        rows[key] = [list(frame[key]) for frame in normalized]
    return pa.Table.from_pydict(rows)


def _write_rgb_videos(root: Path, normalized: list[dict], episode_index: int, fps: float) -> None:
    for key in RGB_IMAGE_KEYS:
        if key not in normalized[0]:
            continue
        frames = [np.asarray(frame[key], dtype=np.uint8) for frame in normalized]
        output = video_episode_path(root, key, episode_index)
        output.parent.mkdir(parents=True, exist_ok=True)
        encode_rgb_frames_to_mp4(frames, output, fps=fps)





def _compute_stats(root: Path) -> dict[str, dict[str, list[float]]]:
    if pq is None:
        return {}
    keys = (
        "observation.state",
        "observation.ee_pose",
        "observation.object_pose",
        "observation.ft",
        "observation.gripper",
        "action",
    )
    stats: dict[str, dict[str, list[float]]] = {}
    for key in keys:
        chunks: list[np.ndarray] = []
        for episode_index in list_episode_indices(root):
            table = pq.read_table(parquet_episode_path(root, episode_index))
            if key not in table.column_names:
                continue
            values = np.asarray(table[key].to_pylist(), dtype=np.float32)
            chunks.append(values)
        if not chunks:
            continue
        stacked = np.concatenate(chunks, axis=0)
        stats[key] = {
            "mean": stacked.mean(axis=0).astype(np.float32).tolist(),
            "std": stacked.std(axis=0).astype(np.float32).tolist(),
            "min": stacked.min(axis=0).astype(np.float32).tolist(),
            "max": stacked.max(axis=0).astype(np.float32).tolist(),
        }
    return stats


def rebuild_dataset_meta(root: Path, *, fps: float, features: dict[str, dict[str, Any]] | None = None) -> None:
    root = dataset_root(root)
    meta_dir = root / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    episode_indices = list_episode_indices(root)
    episode_records: list[dict[str, Any]] = []
    total_frames = 0
    tasks: list[str] = []

    for episode_index in episode_indices:
        parquet_file = parquet_episode_path(root, episode_index)
        if not parquet_file.is_file():
            continue
        table = pq.read_table(parquet_file)
        num_frames = table.num_rows
        instruction = ""
        if "language_instruction" in table.column_names and num_frames > 0:
            instruction = str(table["language_instruction"][0].as_py())
        if instruction and instruction not in tasks:
            tasks.append(instruction)
        episode_records.append(
            {
                "episode_index": episode_index,
                "episode_id": f"episode_{episode_index:06d}",
                "tasks": [instruction] if instruction else [],
                "length": num_frames,
            }
        )
        total_frames += num_frames

    if not tasks:
        tasks = ["teleop"]

    if features is None and episode_indices:
        first_table = pq.read_table(parquet_episode_path(root, episode_indices[0]))
        pseudo_rows = []
        for index in range(min(1, first_table.num_rows)):
            pseudo_rows.append({column: first_table[column][index].as_py() for column in first_table.column_names})
        if pseudo_rows:
            features = build_info_features([pseudo_rows[0]], fps)
            sidecar = sidecar_meta_path(root, episode_indices[0])
            if sidecar.is_file():
                sidecar_meta = json.loads(sidecar.read_text(encoding="utf-8"))
                for key, value in (sidecar_meta.get("video_specs") or {}).items():
                    features[key] = _build_video_feature(key, tuple(value), fps)

    task_records = [{"task_index": index, "task": task} for index, task in enumerate(tasks)]
    info = {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": ROBOT_TYPE,
        "total_episodes": len(episode_records),
        "total_frames": total_frames,
        "total_tasks": len(task_records),
        "total_videos": sum(
            len((json.loads(sidecar_meta_path(root, index).read_text()).get("video_specs") or {}))
            for index in episode_indices if sidecar_meta_path(root, index).is_file()
        ),
        "total_chunks": 1,
        "chunks_size": 1000,
        "fps": fps,
        "splits": {"train": f"0:{len(episode_records)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": features or {},
    }
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
    (meta_dir / "stats.json").write_text(json.dumps(_compute_stats(root), indent=2) + "\n", encoding="utf-8")
    (meta_dir / "tasks.jsonl").write_text(
        "\n".join(json.dumps(record) for record in task_records) + ("\n" if task_records else ""),
        encoding="utf-8",
    )
    (meta_dir / "episodes.jsonl").write_text(
        "\n".join(json.dumps(record) for record in episode_records) + ("\n" if episode_records else ""),
        encoding="utf-8",
    )


def append_episode(
    root: Path,
    episode_index: int,
    normalized: list[dict],
    *,
    task: str,
    success: bool,
    metadata: dict[str, Any] | None,
    fps: float = DEFAULT_FPS,
) -> Path:
    has_videos = any(key in normalized[0] for key in RGB_IMAGE_KEYS)
    if has_videos and not ffmpeg_available():
        raise RuntimeError("ffmpeg is required to write LeRobot v2.1 video episodes")
    if pa is None or pq is None:
        raise RuntimeError("pyarrow is required to write LeRobot v2.1 episodes")

    root = dataset_root(root)
    root.mkdir(parents=True, exist_ok=True)
    instruction = str(normalized[0].get("language_instruction", task))
    task_index = _task_index_for_instruction(root, instruction)
    global_index_start = _global_index_start(root, episode_index)

    parquet_file = parquet_episode_path(root, episode_index)
    parquet_file.parent.mkdir(parents=True, exist_ok=True)
    table = episode_to_parquet_table(
        normalized,
        episode_index=episode_index,
        global_index_start=global_index_start,
        fps=fps,
        task_index=task_index,
    )
    pq.write_table(table, parquet_file)
    if has_videos:
        _write_rgb_videos(root, normalized, episode_index, fps)

    episode_metadata = dict(metadata or {})
    upstream_gate = str(episode_metadata.pop("upstream_gate", "teleop"))
    promoted = {
        key: episode_metadata.pop(key)
        for key in (
            "visual_streams",
            "capture_fps",
            "action_type",
            "action_semantics",
            "action_sources",
        )
        if key in episode_metadata
    }
    video_specs = {
        key: list(np.asarray(normalized[0][key], dtype=np.uint8).shape)
        for key in RGB_IMAGE_KEYS if key in normalized[0]
    }
    sidecar = sidecar_meta_path(root, episode_index)
    sidecar.parent.mkdir(parents=True, exist_ok=True)
    sidecar.write_text(
        json.dumps(
            {
                "task": task,
                "frames": len(normalized),
                "episode_index": episode_index,
                "dataset_path": str(parquet_file),
                "format": FORMAT_V21,
                "success": bool(success),
                "upstream_gate": upstream_gate,
                **promoted,
                "metadata": episode_metadata,
                "video_specs": video_specs,
                "fps": fps,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    rebuild_dataset_meta(root, fps=fps, features=build_info_features(normalized, fps))
    return parquet_file


def load_parquet_rows(root: Path, episode_index: int) -> list[dict[str, Any]]:
    table = pq.read_table(parquet_episode_path(dataset_root(root), episode_index))
    return table.to_pylist()


def load_dataset_parquet_rows(root: Path) -> list[dict[str, Any]]:
    root = dataset_root(root)
    rows: list[dict[str, Any]] = []
    for episode_index in list_episode_indices(root):
        rows.extend(load_parquet_rows(root, episode_index))
    return rows


def ffprobe_frame_count(video_path: Path) -> int:
    output = subprocess.check_output(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-count_packets",
            "-show_entries",
            "stream=nb_read_packets",
            "-of",
            "csv=p=0",
            str(video_path),
        ],
        text=True,
    ).strip()
    return int(output)


def validate_episode(root: Path, episode_index: int, min_frames: int) -> list[str]:
    root = dataset_root(root)
    errors: list[str] = []
    parquet_file = parquet_episode_path(root, episode_index)
    if not parquet_file.is_file():
        return [f"{root}: missing {parquet_file}"]
    rows = load_parquet_rows(root, episode_index)
    if len(rows) < min_frames:
        errors.append(f"{parquet_file}: too few frames ({len(rows)} < {min_frames})")
    sidecar = sidecar_meta_path(root, episode_index)
    video_specs = {}
    if sidecar.is_file():
        video_specs = json.loads(sidecar.read_text(encoding="utf-8")).get("video_specs") or {}
    for key in video_specs:
        video_file = video_episode_path(root, key, episode_index)
        if not video_file.is_file():
            errors.append(f"{parquet_file}: missing video {video_file}")
            continue
        try:
            frame_count = ffprobe_frame_count(video_file)
        except (RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
            errors.append(f"{parquet_file}: invalid video {video_file}: {exc}")
            continue
        if frame_count != len(rows):
            errors.append(
                f"{parquet_file}: {key} video frames ({frame_count}) != parquet rows ({len(rows)})"
            )
    depth_file = depth_episode_path(root, episode_index)
    if depth_file.is_file():
        payload = np.load(depth_file)
        if int(payload["depth_mm"].shape[0]) != len(rows):
            errors.append(f"{parquet_file}: depth frames != parquet rows")
    if rows and not all(str(row.get("language_instruction", "")).strip() for row in rows):
        errors.append(f"{parquet_file}: language_instruction contains empty values")
    if rows and not all(bool(row.get("success")) for row in rows):
        errors.append(f"{parquet_file}: success is not true for every frame")
    if rows and any(bool(row.get("safety_estop")) for row in rows):
        errors.append(f"{parquet_file}: contains safety_estop=true frames")
    if rows and any(bool(row.get("drive_fault")) for row in rows):
        errors.append(f"{parquet_file}: contains drive_fault=true frames")
    return errors
