"""Community-aligned motion helpers for batch Servo control.

References:
- MoveIt Servo teleop: incremental commands at fixed rate (Black Coffee Robotics)
- Claru dataset guide: cap linear velocity (~0.3 m/s teleop, lower for batch)
- UMI: track open-loop EE error before accepting episodes
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def xyz_from_pose_msg(pose_msg) -> list[float] | None:
    if pose_msg is None:
        return None
    p = pose_msg.pose.position
    if not all(math.isfinite(float(v)) for v in (p.x, p.y, p.z)):
        return None
    return [float(p.x), float(p.y), float(p.z)]


def position_error(current: Sequence[float], target: Sequence[float]) -> tuple[float, float, float]:
    err_x = float(target[0]) - float(current[0])
    err_y = float(target[1]) - float(current[1])
    err_z = float(target[2]) - float(current[2])
    xy_err = math.hypot(err_x, err_y)
    z_err = abs(err_z)
    total = math.sqrt(err_x * err_x + err_y * err_y + err_z * err_z)
    return xy_err, z_err, total


def acceleration_limited_step(
    distance_m: float,
    current_speed_mps: float,
    max_speed_mps: float,
    max_acceleration_mps2: float,
    dt_s: float,
) -> tuple[float, float]:
    """Return a bounded path step and next speed with acceleration/braking limits."""
    distance = max(0.0, float(distance_m))
    if distance <= 1e-9:
        return 0.0, 0.0
    dt = max(1e-4, float(dt_s))
    max_speed = max(1e-4, float(max_speed_mps))
    max_acceleration = max(1e-4, float(max_acceleration_mps2))
    current_speed = clamp(float(current_speed_mps), 0.0, max_speed)
    braking_speed = math.sqrt(2.0 * max_acceleration * distance)
    target_speed = min(max_speed, braking_speed)
    max_speed_delta = max_acceleration * dt
    if target_speed >= current_speed:
        next_speed = min(target_speed, current_speed + max_speed_delta)
    else:
        next_speed = max(target_speed, current_speed - max_speed_delta)
    step = min(distance, 0.5 * (current_speed + next_speed) * dt)
    return step, next_speed


def compute_twist_linear(
    current: Sequence[float],
    target: Sequence[float],
    max_linear_mps: float,
    z_scale: float = 1.0,
) -> tuple[float, float, float, bool]:
    """One twist linear velocity step toward target (planning frame, m/s)."""
    err = [float(target[i]) - float(current[i]) for i in range(3)]
    xy_err = math.hypot(err[0], err[1])
    z_err = abs(err[2])
    reached = xy_err < 1e-4 and z_err < 1e-4
    if reached:
        return 0.0, 0.0, 0.0, True

    max_linear = max(0.001, float(max_linear_mps))
    move_xy = min(max_linear, xy_err) if xy_err > 1e-9 else 0.0
    z_cap = max(0.001, max_linear * max(0.1, z_scale))
    move_z = min(z_cap, z_err) if z_err > 1e-9 else 0.0

    vx = (err[0] / xy_err * move_xy) if xy_err > 1e-9 else 0.0
    vy = (err[1] / xy_err * move_xy) if xy_err > 1e-9 else 0.0
    vz = math.copysign(move_z, err[2]) if z_err > 1e-9 else 0.0
    return vx, vy, vz, False


def update_max_tracking_error(
    current: Iterable[float] | None,
    target: Sequence[float],
    running_max: float,
) -> float:
    if current is None:
        return running_max
    _, _, total = position_error(current, target)
    return max(running_max, total)
