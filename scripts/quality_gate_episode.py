#!/usr/bin/env python3
"""Reject episodes that pass schema checks but fail obvious grasp/place physics."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RECORDER_ROOT = ROOT / "src" / "lerobot_recorder"
if str(RECORDER_ROOT) not in sys.path:
    sys.path.insert(0, str(RECORDER_ROOT))

from lerobot_recorder.episode_loader import detect_episode_format, load_episode_rows  # noqa: E402
from lerobot_recorder.lerobot_v21_dataset import FORMAT_V21, dataset_root, list_episode_indices  # noqa: E402


def _gripper_values(rows: list[dict]) -> list[float]:
    values: list[float] = []
    for row in rows:
        gripper = row.get("observation.gripper")
        if isinstance(gripper, (list, tuple)):
            values.append(float(gripper[0]))
        elif gripper is not None:
            values.append(float(gripper))
    return values


def _object_xyz(rows: list[dict]) -> list[tuple[float, float, float]]:
    coords: list[tuple[float, float, float]] = []
    for row in rows:
        pose = row.get("observation.object_pose")
        if isinstance(pose, (list, tuple)) and len(pose) >= 3:
            coords.append((float(pose[0]), float(pose[1]), float(pose[2])))
    return coords


def check_episode_rows(
    rows: list[dict],
    *,
    min_lift_m: float,
    min_xy_move_m: float,
    gripper_close_max: float,
    require_gripper_close: bool = False,
) -> list[str]:
    errors: list[str] = []
    if not rows:
        return ["empty episode"]

    gripper_error = None
    gripper = _gripper_values(rows)
    if gripper and min(gripper) > gripper_close_max:
        gripper_error = f"gripper never closed (min={min(gripper):.3f}, need <= {gripper_close_max:.3f})"

    lift_ok = False
    xy_ok = False
    coords = _object_xyz(rows)
    if len(coords) >= 2:
        zs = [item[2] for item in coords]
        lift_delta = max(zs) - min(zs)
        lift_ok = lift_delta >= min_lift_m
        if not lift_ok:
            errors.append(f"lift_delta too small ({lift_delta:.3f}m < {min_lift_m:.3f}m)")
        start_xy = coords[0][:2]
        end_xy = coords[-1][:2]
        xy_move = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
        xy_ok = xy_move >= min_xy_move_m
        if not xy_ok:
            errors.append(f"xy_move too small ({xy_move:.3f}m < {min_xy_move_m:.3f}m)")
    if gripper_error and (require_gripper_close or not (lift_ok and xy_ok)):
        errors.append(gripper_error)

    return errors


def list_episode_dirs(root: Path) -> list[int]:
    root = dataset_root(root)
    indices = list_episode_indices(root)
    if indices:
        return indices
    legacy: list[int] = []
    for path in sorted(root.glob("episode_*")):
        if not path.is_dir():
            continue
        if not (path / "train").is_dir() and not (path / "meta.json").is_file():
            continue
        try:
            legacy.append(int(path.name.split("_", 1)[1]))
        except (IndexError, ValueError):
            continue
    return legacy


def load_rows_for_episode(root: Path, episode_index: int) -> list[dict]:
    root = dataset_root(root)
    if detect_episode_format(root) == FORMAT_V21:
        from lerobot_recorder.lerobot_v21_dataset import load_parquet_rows

        return load_parquet_rows(root, episode_index)
    train_dir = root / f"episode_{episode_index:06d}" / "train"
    return load_episode_rows(train_dir, decode_videos=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path, help="Archive root or episode dir")
    parser.add_argument("--episode-index", type=int, default=None)
    parser.add_argument("--min-lift-m", type=float, default=0.025)
    parser.add_argument("--min-xy-move-m", type=float, default=0.05)
    parser.add_argument("--gripper-close-max", type=float, default=0.12)
    parser.add_argument(
        "--require-gripper-close",
        action="store_true",
        help="also fail episodes whose gripper signal never crosses --gripper-close-max",
    )
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    root = dataset_root(args.dataset)
    indices = (
        [args.episode_index]
        if args.episode_index is not None
        else list_episode_dirs(root)
    )
    if not indices:
        print(f"[FAIL] no episodes found under {root}", file=sys.stderr)
        return 2

    all_errors: list[str] = []
    summaries: list[dict] = []
    for episode_index in indices:
        rows = load_rows_for_episode(root, episode_index)
        errors = check_episode_rows(
            rows,
            min_lift_m=args.min_lift_m,
            min_xy_move_m=args.min_xy_move_m,
            gripper_close_max=args.gripper_close_max,
            require_gripper_close=args.require_gripper_close,
        )
        summaries.append(
            {
                "episode_index": episode_index,
                "frames": len(rows),
                "valid": not errors,
                "errors": errors,
            }
        )
        all_errors.extend(f"episode_{episode_index:06d}: {error}" for error in errors)

    payload = {"dataset": str(root), "episodes": summaries, "valid": not all_errors}
    if args.json:
        print(json.dumps(payload, indent=2))
    elif all_errors:
        for error in all_errors:
            print(f"[FAIL] {error}", file=sys.stderr)
    else:
        print(f"[PASS] physics quality gate for {len(summaries)} episode(s)")
    return 0 if not all_errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
