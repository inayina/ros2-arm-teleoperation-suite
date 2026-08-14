# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Wrist RGB pixel checks (simulation render, not hand-eye / not PASS).

Red-box mask is a diagnostic heuristic on MuJoCo RGB. Lighting and
self-occlusion can suppress counts even when GT projection is on-screen.
"""

from __future__ import annotations

import numpy as np

# Loose RGB heuristic for object_red_box rgba="0.9 0.1 0.1" under MuJoCo lights.
MIN_RED = 70
RED_MARGIN = 25
MIN_RED_PIXELS = 40


def red_target_mask(rgb: np.ndarray) -> np.ndarray:
    """Boolean HxW mask of reddish pixels. Input uint8 RGB."""
    arr = np.asarray(rgb)
    if arr.ndim != 3 or arr.shape[2] < 3:
        raise ValueError("rgb must be HxWx3")
    r = arr[:, :, 0].astype(np.int16)
    g = arr[:, :, 1].astype(np.int16)
    b = arr[:, :, 2].astype(np.int16)
    return (r >= MIN_RED) & (r >= g + RED_MARGIN) & (r >= b + RED_MARGIN)


def red_pixel_stats(rgb: np.ndarray) -> dict:
    mask = red_target_mask(rgb)
    count = int(np.count_nonzero(mask))
    h, w = mask.shape
    total = int(h * w)
    return {
        "red_pixel_count": count,
        "red_pixel_ratio": count / total if total else 0.0,
        "rgb_target_visible": count >= MIN_RED_PIXELS,
        "min_red_pixels": MIN_RED_PIXELS,
        "image_hw": [int(h), int(w)],
    }


def sample_rgb_at(rgb: np.ndarray, u: float, v: float) -> list[int] | None:
    """Nearest-neighbour RGB at pixel (u, v); None if out of bounds."""
    arr = np.asarray(rgb)
    h, w = arr.shape[:2]
    x = int(round(float(u)))
    y = int(round(float(v)))
    if x < 0 or y < 0 or x >= w or y >= h:
        return None
    return [int(arr[y, x, 0]), int(arr[y, x, 1]), int(arr[y, x, 2])]
