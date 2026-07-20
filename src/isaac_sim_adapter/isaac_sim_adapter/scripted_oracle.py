"""Pure planning helpers for Isaac scripted-oracle grasp trajectories.

Object-relative waypoints only. Physical lift/place success is owned by
ContinuousTaskEvaluator at runtime — this module never claims task success.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Mapping, Sequence


ORACLE_PHASES: tuple[str, ...] = (
    'approach_xy',
    'hover',
    'descend',
    'close',
    'grasp_pause',
    'lift',
    'hold',
)

DEFAULT_HOVER_Z = 0.12
# Match MuJoCo batch box offset (~0.012). Keep EE near cube mid-height so
# fingers pinch the sides; 0.04 closed on air above the top, 0.015 still
# tracked ~1 cm high and kicked the cube on hard close.
DEFAULT_PICK_Z_OFFSET = 0.010
DEFAULT_LIFT_Z = 0.12
DEFAULT_GRIPPER_OPEN = 1.0
# Normalized finger opening. Cube width 0.05 m ⇒ snug ≈ 0.625; command a
# mild squeeze without teleporting to fully closed (0.0 ejects the box).
DEFAULT_GRIPPER_CLOSE_TARGET = 0.40
DEFAULT_APPROACH_XY_TOL = 0.04
DEFAULT_DESCEND_XY_TOL = 0.05
DEFAULT_DESCEND_Z_TOL = 0.015
DEFAULT_GRASP_PAUSE_S = 2.0
MIN_PICK_Z = 0.02
# 5 cm cube side-grasp: finger opening ≈ 0.025 m → normalized ≈ 0.625.
# GT closed threshold for oracle must be above that (see runner).
ORACLE_GT_GRIPPER_CLOSE_MAX = 0.70


@dataclass(frozen=True)
class OracleTargets:
    """Object-relative waypoint set for one scripted grasp."""

    object_xyz: tuple[float, float, float]
    approach_xy: tuple[float, float, float]
    hover: tuple[float, float, float]
    pick: tuple[float, float, float]
    lift: tuple[float, float, float]
    gripper_open: float = DEFAULT_GRIPPER_OPEN
    gripper_close_target: float = DEFAULT_GRIPPER_CLOSE_TARGET

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class PhaseRecord:
    name: str
    target_xyz: tuple[float, float, float] | None
    ee_xyz: tuple[float, float, float] | None
    object_xyz: tuple[float, float, float] | None
    ee_object_xy_m: float | None
    gripper_cmd: float | None
    ok: bool
    detail: str = ''


@dataclass
class OracleReport:
    """Machine-readable oracle run summary (not a task-success verdict)."""

    status: str
    phases_completed: list[str] = field(default_factory=list)
    phases: list[dict[str, Any]] = field(default_factory=list)
    targets: dict[str, Any] = field(default_factory=dict)
    initial_object_xyz: tuple[float, float, float] | None = None
    final_ee_xyz: tuple[float, float, float] | None = None
    final_object_xyz: tuple[float, float, float] | None = None
    min_gripper_cmd: float | None = None
    all_phases_completed: bool = False
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            'artifact_type': 'isaac_scripted_oracle_report',
            'status': self.status,
            'phases_completed': list(self.phases_completed),
            'phases': list(self.phases),
            'targets': dict(self.targets),
            'initial_object_xyz': self.initial_object_xyz,
            'final_ee_xyz': self.final_ee_xyz,
            'final_object_xyz': self.final_object_xyz,
            'min_gripper_cmd': self.min_gripper_cmd,
            'all_phases_completed': self.all_phases_completed,
            'notes': list(self.notes),
            'task_success_claimed': False,
        }


def _xyz(values: Sequence[float]) -> tuple[float, float, float]:
    if len(values) < 3:
        raise ValueError(f'expected xyz, got {values!r}')
    out = (float(values[0]), float(values[1]), float(values[2]))
    if not all(math.isfinite(v) for v in out):
        raise ValueError(f'non-finite xyz: {out}')
    return out


def xy_distance(
    a: Sequence[float],
    b: Sequence[float],
) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def compute_oracle_targets(
    object_xyz: Sequence[float],
    ee_xyz: Sequence[float],
    *,
    hover_z: float = DEFAULT_HOVER_Z,
    pick_z_offset: float = DEFAULT_PICK_Z_OFFSET,
    lift_z: float | None = None,
    gripper_open: float = DEFAULT_GRIPPER_OPEN,
    gripper_close_target: float = DEFAULT_GRIPPER_CLOSE_TARGET,
    min_pick_z: float = MIN_PICK_Z,
) -> OracleTargets:
    """Build approach → hover → pick → lift waypoints from current poses."""
    obj = _xyz(object_xyz)
    ee = _xyz(ee_xyz)
    hover_height = float(hover_z)
    if not math.isfinite(hover_height) or hover_height <= 0.0:
        raise ValueError(f'hover_z must be > 0, got {hover_z}')
    offset = float(pick_z_offset)
    if not math.isfinite(offset):
        raise ValueError(f'pick_z_offset must be finite, got {pick_z_offset}')
    lift_height = float(hover_height if lift_z is None else lift_z)
    if not math.isfinite(lift_height) or lift_height <= 0.0:
        raise ValueError(f'lift_z must be > 0, got {lift_z}')

    approach = (obj[0], obj[1], ee[2])
    hover = (obj[0], obj[1], hover_height)
    pick = (obj[0], obj[1], max(float(min_pick_z), obj[2] + offset))
    lift = (obj[0], obj[1], lift_height)
    return OracleTargets(
        object_xyz=obj,
        approach_xy=approach,
        hover=hover,
        pick=pick,
        lift=lift,
        gripper_open=max(0.0, min(1.0, float(gripper_open))),
        gripper_close_target=max(0.0, min(1.0, float(gripper_close_target))),
    )


def phase_plan(targets: OracleTargets) -> list[tuple[str, tuple[float, float, float] | None, float]]:
    """Ordered (phase_name, xyz_or_None, gripper_cmd) for the scripted FSM."""
    return [
        ('approach_xy', targets.approach_xy, targets.gripper_open),
        ('hover', targets.hover, targets.gripper_open),
        ('descend', targets.pick, targets.gripper_open),
        ('close', targets.pick, targets.gripper_close_target),
        ('grasp_pause', targets.pick, targets.gripper_close_target),
        ('lift', targets.lift, targets.gripper_close_target),
        ('hold', targets.lift, targets.gripper_close_target),
    ]


def gate_xy(
    ee_xyz: Sequence[float],
    target_xyz: Sequence[float],
    *,
    tolerance_m: float,
) -> tuple[bool, float]:
    dist = xy_distance(ee_xyz, target_xyz)
    return dist <= float(tolerance_m), dist


def interpolate_pose(
    start: Sequence[float],
    target: Sequence[float],
    alpha: float,
) -> tuple[float, float, float]:
    a = max(0.0, min(1.0, float(alpha)))
    return (
        float(start[0]) + (float(target[0]) - float(start[0])) * a,
        float(start[1]) + (float(target[1]) - float(start[1])) * a,
        float(start[2]) + (float(target[2]) - float(start[2])) * a,
    )


def interpolate_gripper(start: float, target: float, alpha: float) -> float:
    a = max(0.0, min(1.0, float(alpha)))
    value = float(start) + (float(target) - float(start)) * a
    return max(0.0, min(1.0, value))


def default_phase_durations_s(
    *,
    approach_s: float = 4.0,
    hover_s: float = 2.5,
    descend_s: float = 4.0,
    close_s: float = 3.0,
    grasp_pause_s: float = DEFAULT_GRASP_PAUSE_S,
    lift_s: float = 3.5,
    hold_s: float = 2.0,
) -> Mapping[str, float]:
    return {
        'approach_xy': float(approach_s),
        'hover': float(hover_s),
        'descend': float(descend_s),
        'close': float(close_s),
        'grasp_pause': float(grasp_pause_s),
        'lift': float(lift_s),
        'hold': float(hold_s),
    }


def build_phase_record(
    name: str,
    *,
    target_xyz: tuple[float, float, float] | None,
    ee_xyz: tuple[float, float, float] | None,
    object_xyz: tuple[float, float, float] | None,
    gripper_cmd: float | None,
    ok: bool,
    detail: str = '',
) -> PhaseRecord:
    xy = None
    if ee_xyz is not None and object_xyz is not None:
        xy = xy_distance(ee_xyz, object_xyz)
    return PhaseRecord(
        name=name,
        target_xyz=target_xyz,
        ee_xyz=ee_xyz,
        object_xyz=object_xyz,
        ee_object_xy_m=xy,
        gripper_cmd=gripper_cmd,
        ok=ok,
        detail=detail,
    )
