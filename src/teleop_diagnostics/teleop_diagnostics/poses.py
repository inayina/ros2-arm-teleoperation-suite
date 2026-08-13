# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Nominal pose sets for Stage-1 geometry reports."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterator

import numpy as np

from teleop_diagnostics import PANDA_ARM_JOINTS

# Franka "ready" / home configuration used by controller GTest and MuJoCo fallback tests.
READY_Q = (
    0.0,
    -math.pi / 4.0,
    0.0,
    -3.0 * math.pi / 4.0,
    0.0,
    math.pi / 2.0,
    math.pi / 4.0,
)

# URDF joint limits (panda.urdf.xacro)
JOINT_LIMITS = {
    "panda_joint1": (-2.8973, 2.8973),
    "panda_joint2": (-1.7628, 1.7628),
    "panda_joint3": (-2.8973, 2.8973),
    "panda_joint4": (-3.0718, -0.0698),
    "panda_joint5": (-2.8973, 2.8973),
    "panda_joint6": (-0.0175, 3.7525),
    "panda_joint7": (-2.8973, 2.8973),
}


@dataclass(frozen=True)
class NamedPose:
    name: str
    q: tuple[float, ...]


def near_limit_pose(margin: float = 0.02) -> tuple[float, ...]:
    """Approach but do not cross each joint limit (toward upper, except j4 lower-ish mid-high)."""
    q = []
    for name in PANDA_ARM_JOINTS:
        lo, hi = JOINT_LIMITS[name]
        # Stay inside by margin from the upper bound (j4 upper is -0.0698).
        q.append(float(min(hi - margin, max(lo + margin, hi - margin))))
    return tuple(q)


def fixed_seed_random_poses(n: int = 5, seed: int = 20260813) -> list[tuple[float, ...]]:
    rng = np.random.default_rng(seed)
    poses: list[tuple[float, ...]] = []
    for _ in range(n):
        q = []
        for name in PANDA_ARM_JOINTS:
            lo, hi = JOINT_LIMITS[name]
            # Keep a small interior margin so poses remain legal.
            q.append(float(rng.uniform(lo + 0.05, hi - 0.05)))
        poses.append(tuple(q))
    return poses


def nominal_pose_set(random_count: int = 5, seed: int = 20260813) -> list[NamedPose]:
    poses = [
        NamedPose("zero", tuple(0.0 for _ in PANDA_ARM_JOINTS)),
        NamedPose("ready", READY_Q),
        NamedPose("near_joint_limit", near_limit_pose()),
    ]
    for i, q in enumerate(fixed_seed_random_poses(random_count, seed)):
        poses.append(NamedPose(f"random_seed{seed}_{i}", q))
    return poses


def iter_nominal_poses(**kwargs) -> Iterator[NamedPose]:
    yield from nominal_pose_set(**kwargs)
