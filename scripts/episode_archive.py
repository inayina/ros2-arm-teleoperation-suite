#!/usr/bin/env python3
"""Manage persistent upstream episode archives under data/episodes/."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RECORDER_ROOT = REPO_ROOT / "src" / "lerobot_recorder"
if str(RECORDER_ROOT) not in sys.path:
    sys.path.insert(0, str(RECORDER_ROOT))

from lerobot_recorder.episode_loader import load_episode_rows  # noqa: E402
from lerobot_recorder.lerobot_v21_dataset import (  # noqa: E402
    FORMAT_V21,
    RGB_IMAGE_KEYS,
    append_episode,
    dataset_root,
    depth_episode_path,
    detect_dataset_format,
    list_episode_indices,
    load_parquet_rows,
    next_episode_index,
    parquet_episode_path,
    rebuild_dataset_meta,
    sidecar_meta_path,
    video_episode_path,
)
from lerobot_recorder.lerobot_writer import _normalize_frame  # noqa: E402

try:
    import pyarrow as pa
    import pyarrow.parquet as pq
except Exception:  # pragma: no cover
    pa = None
    pq = None

DEFAULT_ARCHIVE_ROOT = REPO_ROOT / "data" / "episodes"
COLLECTION_LOG = "collection_log.jsonl"


def _legacy_episode_dirs(root: Path) -> list[tuple[int, Path]]:
    episodes: list[tuple[int, Path]] = []
    if not root.is_dir():
        return episodes
    for path in sorted(root.glob("episode_*")):
        if not path.is_dir() or not (path / "train").is_dir():
            continue
        try:
            index = int(path.name.split("_", 1)[1])
        except (IndexError, ValueError):
            continue
        episodes.append((index, path))
    return sorted(episodes, key=lambda item: item[0])


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


def _global_index_start(root: Path) -> int:
    if pq is None:
        raise RuntimeError("pyarrow is required to import LeRobot v2.1 episodes")
    total = 0
    for index in list_episode_indices(root):
        parquet_file = parquet_episode_path(root, index)
        if parquet_file.is_file():
            total += pq.read_table(parquet_file).num_rows
    return total


def _write_v21_rows(dest_root: Path, dest_index: int, rows: list[dict]) -> Path:
    if pa is None or pq is None:
        raise RuntimeError("pyarrow is required to import LeRobot v2.1 episodes")
    if not rows:
        raise ValueError("cannot import an empty LeRobot v2.1 episode")

    global_start = _global_index_start(dest_root)
    instruction = str(rows[0].get("language_instruction") or rows[0].get("task") or "teleop")
    task_index = _task_index_for_instruction(dest_root, instruction)
    rewritten: list[dict] = []
    for frame_index, row in enumerate(rows):
        item = dict(row)
        item["index"] = global_start + frame_index
        item["episode_index"] = dest_index
        item["frame_index"] = frame_index
        item["task_index"] = task_index
        if "next.done" in item:
            item["next.done"] = frame_index == len(rows) - 1
        rewritten.append(item)

    parquet_file = parquet_episode_path(dest_root, dest_index)
    parquet_file.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(pa.Table.from_pylist(rewritten), parquet_file)
    return parquet_file


def _declared_rgb_video_keys(root: Path) -> tuple[str, ...]:
    """Return RGB streams declared by the source LeRobot v2.1 dataset.

    Scene-only collection intentionally omits the wrist stream. Requiring every
    camera supported by the recorder turns that valid dataset into an import
    failure, so archive import follows ``meta/info.json`` instead.
    """
    info_path = dataset_root(root) / "meta" / "info.json"
    if not info_path.is_file():
        return ()
    info = json.loads(info_path.read_text(encoding="utf-8"))
    features = info.get("features") or {}
    return tuple(
        key for key in RGB_IMAGE_KEYS
        if isinstance(features.get(key), dict)
        and features[key].get("dtype") == "video"
    )


def _copy_v21_media(source_root: Path, source_index: int, dest_root: Path, dest_index: int) -> None:
    for key in _declared_rgb_video_keys(source_root):
        source_video = video_episode_path(source_root, key, source_index)
        if not source_video.is_file():
            raise FileNotFoundError(f"missing v2.1 video: {source_video}")
        dest_video = video_episode_path(dest_root, key, dest_index)
        dest_video.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_video, dest_video)

    source_depth = depth_episode_path(source_root, source_index)
    if source_depth.is_file():
        dest_depth = depth_episode_path(dest_root, dest_index)
        dest_depth.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_depth, dest_depth)


def _import_v21_episode(
    source_root: Path,
    source_index: int,
    dest_root: Path,
    dest_index: int,
) -> Path:
    source_root = dataset_root(source_root)
    dest_root = dataset_root(dest_root)
    dest_root.mkdir(parents=True, exist_ok=True)

    rows = load_parquet_rows(source_root, source_index)
    parquet_file = _write_v21_rows(dest_root, dest_index, rows)
    _copy_v21_media(source_root, source_index, dest_root, dest_index)

    source_sidecar = sidecar_meta_path(source_root, source_index)
    sidecar_payload = (
        json.loads(source_sidecar.read_text(encoding="utf-8"))
        if source_sidecar.is_file()
        else {}
    )
    sidecar_payload.update(
        {
            "episode_index": dest_index,
            "dataset_path": str(parquet_file),
            "format": FORMAT_V21,
            "frames": len(rows),
        }
    )
    dest_sidecar = sidecar_meta_path(dest_root, dest_index)
    dest_sidecar.parent.mkdir(parents=True, exist_ok=True)
    dest_sidecar.write_text(json.dumps(sidecar_payload, indent=2) + "\n", encoding="utf-8")
    return parquet_file


def cmd_status(args: argparse.Namespace) -> int:
    root = dataset_root(args.root)
    indices = list_episode_indices(root)
    if not indices:
        print(f"No episodes under {root}")
        return 0
    print(f"Archive root: {root}")
    print(f"Format: {detect_dataset_format(root)}")
    print(f"Episodes: {len(indices)}")
    print(f"Next index: {next_episode_index(root)}")
    print("")
    for index in indices:
        sidecar = root / f"episode_{index:06d}" / "meta.json"
        frames = "?"
        task = "?"
        upstream_gate = "?"
        success = "?"
        if sidecar.is_file():
            meta = json.loads(sidecar.read_text(encoding="utf-8"))
            frames = meta.get("frames", "?")
            task = meta.get("task", "?")
            upstream_gate = meta.get("upstream_gate", "?")
            success = meta.get("success", "?")
        print(
            f"  episode_{index:06d}  frames={frames}  success={success}  "
            f"gate={upstream_gate}  task={task}"
        )
    return 0


def _import_legacy_episode(
    source_episode: Path,
    dest_root: Path,
    dest_index: int,
    *,
    move: bool,
) -> Path:
    dest_root = dataset_root(dest_root)
    train_dir = source_episode / "train"
    meta_path = source_episode / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}
    rows = load_episode_rows(train_dir, decode_videos=True)
    normalized = [_normalize_frame(row) for row in rows]
    episode_metadata = dict(meta.get("metadata") or {})
    if meta.get("upstream_gate"):
        episode_metadata["upstream_gate"] = meta["upstream_gate"]
    parquet_path = append_episode(
        dest_root,
        dest_index,
        normalized,
        task=str(meta.get("task", "teleop")),
        success=bool(meta.get("success", True)),
        metadata=episode_metadata,
        fps=float(meta.get("fps", 30.0)),
    )
    if move:
        shutil.rmtree(source_episode)
    return parquet_episode_path(dest_root, dest_index)


def import_batch(
    source_root: Path,
    dest_root: Path = DEFAULT_ARCHIVE_ROOT,
    *,
    move: bool = False,
    dry_run: bool = False,
) -> list[dict]:
    source_root = source_root.resolve()
    dest_root = dataset_root(dest_root)
    source_format = detect_dataset_format(source_root)
    source_indices = list_episode_indices(source_root) if source_format == FORMAT_V21 else []
    if source_indices:
        source_episodes = [(index, sidecar_meta_path(source_root, index).parent) for index in source_indices]
    else:
        source_episodes = _legacy_episode_dirs(source_root)
    if not source_episodes:
        raise FileNotFoundError(f"No importable episodes found under {source_root}")

    next_index = next_episode_index(dest_root)
    imported: list[dict] = []
    for source_index, source_episode in source_episodes:
        dest_index = next_index
        next_index += 1
        record = {
            "source_root": str(source_root),
            "source_episode": source_episode.name,
            "source_index": source_index,
            "dest_index": dest_index,
            "dest_episode": f"episode_{dest_index:06d}",
            "move": move,
        }
        if dry_run:
            imported.append(record)
            continue
        if source_indices:
            dest_path = _import_v21_episode(source_root, source_index, dest_root, dest_index)
        else:
            dest_path = _import_legacy_episode(
                source_episode,
                dest_root,
                dest_index,
                move=move,
            )
        record["dest_path"] = str(dest_path)
        imported.append(record)
    return imported


def append_collection_log(dest_root: Path, records: list[dict], *, valid: bool | None = None) -> None:
    if not records:
        return
    dest_root.mkdir(parents=True, exist_ok=True)
    log_path = dest_root / COLLECTION_LOG
    payload = {
        "imported_at": time.time(),
        "records": records,
        "valid": valid,
    }
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")


def cmd_import(args: argparse.Namespace) -> int:
    records = import_batch(
        args.source.resolve(),
        args.dest.resolve(),
        move=args.move,
        dry_run=args.dry_run,
    )
    if args.dry_run:
        print(json.dumps(records, indent=2))
        return 0

    valid = None
    if args.validate:
        import subprocess

        result = subprocess.run(
            [
                sys.executable,
                str(REPO_ROOT / "scripts" / "validate_dataset.py"),
                str(args.dest.resolve()),
                "--min-frames",
                str(args.min_frames),
            ],
            capture_output=True,
            text=True,
        )
        valid = result.returncode == 0
        if not valid:
            print(result.stdout, file=sys.stderr)
            print(result.stderr, file=sys.stderr)
            return 1
        print(f"[PASS] imported {len(records)} episode(s) into {args.dest}")

    append_collection_log(args.dest.resolve(), records, valid=valid)
    rebuild_dataset_meta(args.dest.resolve(), fps=30.0)
    for record in records:
        print(
            f"imported {record['source_episode']} -> {record['dest_episode']} "
            f"({record['dest_path']})"
        )
    return 0


def cmd_next_index(args: argparse.Namespace) -> int:
    print(next_episode_index(dataset_root(args.root)))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    status = subparsers.add_parser("status", help="List archived episodes.")
    status.add_argument("--root", type=Path, default=DEFAULT_ARCHIVE_ROOT)
    status.set_defaults(func=cmd_status)

    next_index = subparsers.add_parser("next-index", help="Print the next episode index.")
    next_index.add_argument("--root", type=Path, default=DEFAULT_ARCHIVE_ROOT)
    next_index.set_defaults(func=cmd_next_index)

    import_cmd = subparsers.add_parser(
        "import",
        help="Import LeRobot v2.1 or legacy episode_*/ directories into the archive.",
    )
    import_cmd.add_argument("source", type=Path, help="Batch output root with episode_*/train.")
    import_cmd.add_argument("--dest", type=Path, default=DEFAULT_ARCHIVE_ROOT)
    import_cmd.add_argument("--move", action="store_true", help="Move instead of copy.")
    import_cmd.add_argument("--dry-run", action="store_true")
    import_cmd.add_argument(
        "--validate",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    import_cmd.add_argument("--min-frames", type=int, default=5)
    import_cmd.set_defaults(func=cmd_import)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
