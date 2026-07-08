#!/usr/bin/env python3
"""Validate recorded episodes before ACT/Diffusion training import."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


REQUIRED_FIELDS = (
    "observation.state",
    "observation.ee_pose",
    "observation.object_pose",
    "observation.ft",
    "observation.gripper",
    "observation.images.scene",
    "observation.images.wrist",
    "observation.images.tactile_left",
    "observation.images.tactile_right",
    "observation.depth.scene",
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


def _episode_train_dirs(root: Path) -> list[Path]:
    if root.name == "train" and root.is_dir():
        return [root]
    if (root / "train").is_dir():
        return [root / "train"]
    episode_dirs = sorted(path for path in root.glob("episode_*/train") if path.is_dir())
    if episode_dirs:
        return episode_dirs
    return sorted(
        path
        for path in root.rglob("train")
        if path.is_dir() and path.parent.name.startswith("episode_")
    )


def _episode_meta_path(train_dir: Path) -> Path | None:
    if train_dir.name == "train":
        meta_path = train_dir.parent / "meta.json"
        return meta_path if meta_path.is_file() else None
    return None


def _read_upstream_gate(train_dir: Path) -> str | None:
    meta_path = _episode_meta_path(train_dir)
    if meta_path is None:
        return None
    try:
        payload = json.loads(meta_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    gate = payload.get("upstream_gate")
    if gate:
        return str(gate)
    nested = payload.get("metadata")
    if isinstance(nested, dict) and nested.get("upstream_gate"):
        return str(nested["upstream_gate"])
    return None


def _load_dataset(path: Path):
    try:
        from datasets import load_from_disk
    except Exception as exc:  # pragma: no cover - environment guard
        raise RuntimeError("HuggingFace datasets is required to validate Arrow episodes") from exc
    return load_from_disk(str(path))


def _bool_column_has_true_only(ds, key: str) -> bool:
    values = list(ds[key])
    return bool(values) and all(bool(value) for value in values)


def _bool_column_has_false_only(ds, key: str) -> bool:
    values = list(ds[key])
    return all(not bool(value) for value in values)


def _nonempty_language(ds) -> bool:
    return all(isinstance(value, str) and value.strip() for value in ds["language_instruction"])


def _shape_ok(ds, key: str, expected_last_dim: int | None = None) -> bool:
    if len(ds) == 0:
        return False
    arr = np.asarray(ds[0][key])
    if arr.size == 0:
        return False
    if expected_last_dim is not None and (arr.ndim == 0 or arr.shape[-1] != expected_last_dim):
        return False
    return True


def validate_episode(train_dir: Path, min_frames: int, allow_failed: bool) -> tuple[list[str], dict]:
    errors: list[str] = []
    episode_info = {
        "train_dir": str(train_dir),
        "upstream_gate": _read_upstream_gate(train_dir),
    }
    try:
        ds = _load_dataset(train_dir)
    except Exception as exc:
        return [f"{train_dir}: failed to load dataset: {exc}"], episode_info

    if len(ds) < min_frames:
        errors.append(f"{train_dir}: too few frames ({len(ds)} < {min_frames})")

    missing = [field for field in REQUIRED_FIELDS if field not in ds.features]
    if missing:
        errors.append(f"{train_dir}: missing required fields: {', '.join(missing)}")
        return errors, episode_info

    if not _nonempty_language(ds):
        errors.append(f"{train_dir}: language_instruction contains empty values")
    if not allow_failed and not _bool_column_has_true_only(ds, "success"):
        errors.append(f"{train_dir}: success is not true for every frame")
    if not _bool_column_has_false_only(ds, "safety_estop"):
        errors.append(f"{train_dir}: contains safety_estop=true frames")
    if not _bool_column_has_false_only(ds, "drive_fault"):
        errors.append(f"{train_dir}: contains drive_fault=true frames")

    if not _shape_ok(ds, "observation.state"):
        errors.append(f"{train_dir}: observation.state is empty")
    if not _shape_ok(ds, "action"):
        errors.append(f"{train_dir}: action is empty")
    for key in (
        "observation.images.scene",
        "observation.images.wrist",
        "observation.images.tactile_left",
        "observation.images.tactile_right",
    ):
        if not _shape_ok(ds, key, expected_last_dim=3):
            errors.append(f"{train_dir}: {key} is not an RGB image tensor")

    return errors, episode_info


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path, help="episode root, episode dir, or train dir")
    parser.add_argument("--min-frames", type=int, default=5)
    parser.add_argument("--allow-failed", action="store_true")
    parser.add_argument("--json", action="store_true", help="emit a machine-readable summary")
    args = parser.parse_args()

    train_dirs = _episode_train_dirs(args.dataset)
    if not train_dirs:
        print(f"No episode train directories found under {args.dataset}", file=sys.stderr)
        return 2

    all_errors = []
    episode_summaries = []
    for train_dir in train_dirs:
        errors, episode_info = validate_episode(train_dir, args.min_frames, args.allow_failed)
        all_errors.extend(errors)
        episode_info["valid"] = not errors
        episode_summaries.append(episode_info)

    upstream_gates = sorted(
        {item["upstream_gate"] for item in episode_summaries if item.get("upstream_gate")}
    )
    summary = {
        "dataset": str(args.dataset),
        "episodes": len(train_dirs),
        "valid": not all_errors,
        "upstream_gates": upstream_gates,
        "episode_summaries": episode_summaries,
        "errors": all_errors,
    }
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    elif all_errors:
        for error in all_errors:
            print(f"[FAIL] {error}", file=sys.stderr)
    else:
        print(f"[PASS] {len(train_dirs)} episode(s) are ACT-ready.")
    return 0 if not all_errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
