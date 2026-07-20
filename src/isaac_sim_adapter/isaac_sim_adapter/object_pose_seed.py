"""Deterministic red-box XY sampling for Isaac nominal evaluation seeds.

Matches MuJoCo training distribution in config/randomization.yaml:
  object.initial_pos_range.x: [0.36, 0.44]
  object.initial_pos_range.y: [-0.15, 0.15]
"""

from __future__ import annotations

import math
import random
from typing import Optional, Sequence, Tuple

# Training-distribution pick workspace (single red box, Isaac P3 scene).
OBJECT_X_RANGE = (0.36, 0.44)
OBJECT_Y_RANGE = (-0.15, 0.15)
OBJECT_Z_NOMINAL = 0.025
OBJECT_YAW_RANGE_DEG = (-15.0, 15.0)


def sample_red_box_pose(
    seed: int,
    *,
    x_range: Sequence[float] = OBJECT_X_RANGE,
    y_range: Sequence[float] = OBJECT_Y_RANGE,
    z: float = OBJECT_Z_NOMINAL,
    yaw_range_deg: Sequence[float] = OBJECT_YAW_RANGE_DEG,
) -> Tuple[float, float, float, float]:
    """Return (x, y, z, yaw_rad) for a seeded red-box placement."""
    if int(seed) < 0:
        raise ValueError('object seed must be non-negative')
    rng = random.Random(int(seed))
    x = rng.uniform(float(x_range[0]), float(x_range[1]))
    y = rng.uniform(float(y_range[0]), float(y_range[1]))
    yaw_deg = rng.uniform(float(yaw_range_deg[0]), float(yaw_range_deg[1]))
    return (float(x), float(y), float(z), math.radians(float(yaw_deg)))


def parse_object_xy(value: Optional[str]) -> Optional[Tuple[float, float]]:
    """Parse 'x,y' override; empty/None → None."""
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    parts = [p.strip() for p in text.split(',')]
    if len(parts) != 2:
        raise ValueError(f'object-xy must be x,y got {value!r}')
    return (float(parts[0]), float(parts[1]))


def resolve_red_box_pose(
    *,
    object_seed: Optional[int],
    object_xy: Optional[Tuple[float, float]] = None,
    nominal_xyz: Tuple[float, float, float] = (0.35, -0.07, 0.025),
) -> Tuple[float, float, float, float]:
    """Resolve placement: explicit XY override > seed sample > nominal (yaw=0)."""
    if object_xy is not None:
        return (float(object_xy[0]), float(object_xy[1]), float(nominal_xyz[2]), 0.0)
    if object_seed is not None:
        return sample_red_box_pose(int(object_seed))
    return (float(nominal_xyz[0]), float(nominal_xyz[1]), float(nominal_xyz[2]), 0.0)


def yaw_to_quat_wxyz(yaw_rad: float) -> Tuple[float, float, float, float]:
    """Isaac DynamicCuboid orientation is wxyz."""
    half = 0.5 * float(yaw_rad)
    return (math.cos(half), 0.0, 0.0, math.sin(half))
