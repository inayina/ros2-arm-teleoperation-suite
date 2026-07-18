"""Pure safety helpers for bounded ``ee_delta_gripper`` execution."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


PANDA_JOINT_LOWER = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
PANDA_JOINT_UPPER = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)


@dataclass(frozen=True)
class BoundedAction:
    """Validated policy action after the local execution envelope is applied."""

    values: tuple[float, ...]
    clipped: bool


@dataclass(frozen=True)
class TargetPose:
    """Absolute Panda pose generated from a relative policy action."""

    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    workspace_clipped: bool


def offset_pose_in_local_frame(
    position: Sequence[float],
    orientation_xyzw: Sequence[float],
    local_offset: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Translate a pose by ``local_offset`` while preserving orientation."""
    if len(position) != 3 or len(local_offset) != 3:
        raise ValueError('position and local offset must have three components')
    origin = tuple(float(value) for value in position)
    offset = tuple(float(value) for value in local_offset)
    if not all(math.isfinite(value) for value in (*origin, *offset)):
        raise ValueError('pose translation contains NaN or infinity')
    x, y, z, w = normalize_quaternion(orientation_xyzw)
    # Rotate the local offset with q * v * q^-1. Expanded here so this helper
    # stays dependency-free in the isolated Isaac Python environment.
    rotation = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)),
    )
    translated = tuple(
        origin[row] + sum(rotation[row][column] * offset[column]
                          for column in range(3))
        for row in range(3)
    )
    return translated, (x, y, z, w)


def bound_gripper_command(value: float) -> tuple[float, bool]:
    """Validate and clamp a normalized Panda gripper command."""
    command = float(value)
    if not math.isfinite(command):
        raise ValueError('gripper command contains NaN or infinity')
    bounded = max(0.0, min(1.0, command))
    return bounded, bounded != command


def validate_panda_joint_positions(values: Sequence[float]) -> tuple[float, ...]:
    """Fail closed when a live Panda state is outside the URDF hard limits."""
    if len(values) != 7:
        raise ValueError(f'expected Panda joint state[7], got [{len(values)}]')
    positions = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in positions):
        raise ValueError('Panda joint state contains NaN or infinity')
    for index, (value, lower, upper) in enumerate(zip(
        positions, PANDA_JOINT_LOWER, PANDA_JOINT_UPPER
    ), start=1):
        if value < lower or value > upper:
            raise ValueError(
                f'panda_joint{index} outside hard limits: '
                f'{value:.6f} not in [{lower:.6f}, {upper:.6f}]'
            )
    return positions


def bound_ee_delta_gripper(
    values: Sequence[float],
    *,
    max_translation_m: float,
    max_rotation_rad: float,
) -> BoundedAction:
    """Validate action[7] and clamp each Cartesian/RPY component."""
    if len(values) != 7:
        raise ValueError(f'expected ee_delta_gripper[7], got [{len(values)}]')
    action = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in action):
        raise ValueError('policy action contains NaN or infinity')
    if max_translation_m <= 0.0 or max_rotation_rad <= 0.0:
        raise ValueError('policy action limits must be positive')

    limits = (max_translation_m,) * 3 + (max_rotation_rad,) * 3
    bounded = [
        max(-limit, min(limit, value))
        for value, limit in zip(action[:6], limits)
    ]
    bounded.append(max(0.0, min(1.0, action[6])))
    result = tuple(bounded)
    return BoundedAction(values=result, clipped=result != action)


def normalize_quaternion(
    quaternion_xyzw: Sequence[float],
) -> tuple[float, float, float, float]:
    if len(quaternion_xyzw) != 4:
        raise ValueError('quaternion must have four components')
    values = tuple(float(value) for value in quaternion_xyzw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion contains NaN or infinity')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise ValueError('quaternion norm must be positive')
    return tuple(value / norm for value in values)


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float,
) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def quaternion_multiply(
    left_xyzw: Sequence[float], right_xyzw: Sequence[float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = normalize_quaternion(left_xyzw)
    rx, ry, rz, rw = normalize_quaternion(right_xyzw)
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def action_to_target_pose(
    current_position: Sequence[float],
    current_orientation_xyzw: Sequence[float],
    bounded_action: Sequence[float],
    *,
    workspace_min: Sequence[float],
    workspace_max: Sequence[float],
) -> TargetPose:
    """Apply the training adapter's ``delta = target * inverse(current)`` contract."""
    if len(current_position) != 3:
        raise ValueError('current position must have three components')
    if len(bounded_action) != 7:
        raise ValueError('bounded action must have seven components')
    if len(workspace_min) != 3 or len(workspace_max) != 3:
        raise ValueError('workspace bounds must have three components')
    position = tuple(float(value) for value in current_position)
    if not all(math.isfinite(value) for value in position):
        raise ValueError('current position contains NaN or infinity')
    minimum = tuple(float(value) for value in workspace_min)
    maximum = tuple(float(value) for value in workspace_max)
    if any(low >= high for low, high in zip(minimum, maximum)):
        raise ValueError('workspace min must be below workspace max')

    requested = tuple(position[index] + float(bounded_action[index]) for index in range(3))
    target_position = tuple(
        max(minimum[index], min(maximum[index], requested[index]))
        for index in range(3)
    )
    delta_quaternion = quaternion_from_rpy(*bounded_action[3:6])
    # The dataset uses delta_q = target_q * inverse(current_q), therefore
    # target_q = delta_q * current_q.
    target_orientation = quaternion_multiply(
        delta_quaternion, current_orientation_xyzw
    )
    return TargetPose(
        position=target_position,
        orientation_xyzw=target_orientation,
        workspace_clipped=target_position != requested,
    )
