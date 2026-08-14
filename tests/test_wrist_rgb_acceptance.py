# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Wrist RGB acceptance tests (REPORT_ONLY; no PASS / no hand-eye)."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from teleop_diagnostics.wrist_rgb import MIN_RED_PIXELS, red_pixel_stats, red_target_mask

REPO = Path(__file__).resolve().parents[1]


def test_red_mask_finds_synthetic_cube_and_ignores_blue():
    rgb = np.zeros((48, 64, 3), dtype=np.uint8)
    rgb[10:20, 10:20] = (200, 20, 20)
    rgb[30:40, 40:50] = (20, 20, 200)
    mask = red_target_mask(rgb)
    stats = red_pixel_stats(rgb)
    assert int(np.count_nonzero(mask)) == 100
    assert stats["rgb_target_visible"] is True
    assert stats["red_pixel_count"] >= MIN_RED_PIXELS


def test_red_mask_empty_on_gray():
    rgb = np.full((24, 24, 3), 80, dtype=np.uint8)
    assert red_pixel_stats(rgb)["rgb_target_visible"] is False


def test_wrist_rgb_cli_renders_or_fail_closed(tmp_path):
    from teleop_diagnostics.wrist_rgb_cli import run_wrist_rgb

    manifest = run_wrist_rgb(tmp_path / "wrist_rgb")
    assert manifest["stage"] == "wrist_rgb"
    assert manifest["models_merged"] is False
    assert manifest["result_semantics"] == "REPORT_ONLY"
    assert "PASS" not in json_dump(manifest)
    diag = (tmp_path / "wrist_rgb" / "wrist_rgb_diagnostics.json").read_text()
    assert "PHYSICAL_CALIBRATED" in diag
    assert "hand-eye" in diag
    gate = manifest["exit_gate_observations"]
    if gate["render_available"]:
        assert gate["gt_visible_all_four"] is True
        assert gate["rgb_visible_all_four"] is True
        assert (tmp_path / "wrist_rgb" / "png" / "wrist_grasp.png").is_file()
    else:
        assert manifest["exit_gate_observations"]["rgb_visible_all_four"] is False


def json_dump(obj) -> str:
    import json

    return json.dumps(obj)
