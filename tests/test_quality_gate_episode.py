"""Tests for physics quality gate heuristics."""

from __future__ import annotations

import importlib.util
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "quality_gate_episode.py"
SPEC = importlib.util.spec_from_file_location("quality_gate_episode", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
quality_gate_episode = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(quality_gate_episode)
check_episode_rows = quality_gate_episode.check_episode_rows


def _row(x: float, y: float, z: float, gripper: float = 0.5) -> dict:
    return {
        "observation.object_pose": [x, y, z, 0.0, 0.0, 0.0, 1.0],
        "observation.gripper": [gripper],
    }


def test_assisted_grasp_passes_when_object_lifts_and_moves() -> None:
    rows = [
        _row(0.35, -0.08, 0.02),
        _row(0.36, -0.09, 0.08),
        _row(0.40, -0.20, 0.05),
    ]

    assert not check_episode_rows(
        rows,
        min_lift_m=0.025,
        min_xy_move_m=0.05,
        gripper_close_max=0.12,
    )


def test_gripper_proxy_still_fails_when_object_does_not_move() -> None:
    rows = [
        _row(0.35, -0.08, 0.02),
        _row(0.35, -0.08, 0.02),
    ]

    errors = check_episode_rows(
        rows,
        min_lift_m=0.025,
        min_xy_move_m=0.05,
        gripper_close_max=0.12,
    )

    assert any("gripper never closed" in error for error in errors)


def test_require_gripper_close_keeps_strict_mode() -> None:
    rows = [
        _row(0.35, -0.08, 0.02),
        _row(0.36, -0.09, 0.08),
        _row(0.40, -0.20, 0.05),
    ]

    errors = check_episode_rows(
        rows,
        min_lift_m=0.025,
        min_xy_move_m=0.05,
        gripper_close_max=0.12,
        require_gripper_close=True,
    )

    assert any("gripper never closed" in error for error in errors)
