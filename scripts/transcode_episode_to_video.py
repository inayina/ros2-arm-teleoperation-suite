#!/usr/bin/env python3
"""Migrate legacy Arrow upstream episodes to LeRobot v2.1 layout."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RECORDER_ROOT = ROOT / "src" / "lerobot_recorder"
if str(RECORDER_ROOT) not in sys.path:
    sys.path.insert(0, str(RECORDER_ROOT))

from lerobot_recorder.episode_loader import FORMAT_ARROW, detect_episode_format, load_episode_rows  # noqa: E402
from lerobot_recorder.lerobot_v21_dataset import (  # noqa: E402
    FORMAT_V21,
    append_episode,
    dataset_root,
    list_episode_indices,
    next_episode_index,
)
from lerobot_recorder.lerobot_writer import _normalize_frame  # noqa: E402


def discover_legacy_train_dirs(dataset: Path) -> list[Path]:
    if dataset.name == "train" and (dataset / "dataset_info.json").is_file():
        return [dataset]
    if dataset.name == "train" and list(dataset.glob("data-*.arrow")):
        return [dataset]
    episodes = sorted(path for path in dataset.glob("episode_*/train") if path.is_dir())
    if episodes:
        return episodes
    if (dataset / "train").is_dir():
        return [dataset / "train"]
    return []


def transcode_train_dir(train_dir: Path, dataset_root_path: Path, *, fps: float, dry_run: bool) -> dict:
    train_dir = train_dir.resolve()
    root = dataset_root(dataset_root_path)
    if detect_episode_format(train_dir) == FORMAT_V21 and not discover_legacy_train_dirs(train_dir):
        return {"train_dir": str(train_dir), "status": "skipped", "reason": "already v2.1"}

    meta_path = train_dir.parent / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}
    rows = load_episode_rows(train_dir, decode_videos=True)
    normalized = [_normalize_frame(row) for row in rows]
    episode_index = int(meta.get("episode_index", next_episode_index(root)))
    task = str(meta.get("task", "teleop"))
    success = bool(meta.get("success", True))
    episode_metadata = dict(meta.get("metadata") or {})
    if meta.get("upstream_gate"):
        episode_metadata["upstream_gate"] = meta["upstream_gate"]

    if dry_run:
        return {
            "train_dir": str(train_dir),
            "status": "dry_run",
            "frames": len(normalized),
            "episode_index": episode_index,
            "target_root": str(root),
        }

    append_episode(
        root,
        episode_index,
        normalized,
        task=task,
        success=success,
        metadata=episode_metadata,
        fps=fps,
    )

    legacy_episode_dir = train_dir.parent if train_dir.name == "train" else train_dir
    if legacy_episode_dir.name.startswith("episode_") and train_dir.is_dir():
        backup_root = root / ".legacy_arrow_backup" / legacy_episode_dir.name
        backup_train = backup_root / "train"
        backup_root.mkdir(parents=True, exist_ok=True)
        if backup_train.exists():
            shutil.rmtree(backup_train)
        shutil.move(str(train_dir), str(backup_train))

    return {
        "train_dir": str(train_dir),
        "status": "transcoded",
        "frames": len(normalized),
        "episode_index": episode_index,
        "format": FORMAT_V21,
        "dataset_root": str(root),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "dataset",
        type=Path,
        help="Legacy episode train dir, episode dir, or archive root (data/episodes).",
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=None,
        help="LeRobot v2.1 dataset root. Defaults to dataset when it is an archive root.",
    )
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    target_root = args.dataset_root or dataset_root(args.dataset)
    train_dirs = discover_legacy_train_dirs(args.dataset)
    if not train_dirs and detect_episode_format(args.dataset) == FORMAT_V21:
        print(f"Already LeRobot v2.1 under {target_root}")
        return 0
    if not train_dirs:
        print(f"No legacy Arrow train directories found under {args.dataset}", file=sys.stderr)
        return 2

    results = [
        transcode_train_dir(path, target_root, fps=args.fps, dry_run=args.dry_run)
        for path in train_dirs
    ]
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        for item in results:
            print(item)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
