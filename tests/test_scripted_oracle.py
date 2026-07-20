"""Unit tests for Isaac scripted-oracle pure planning helpers (no Isaac/ROS)."""

from __future__ import annotations

import math

import pytest

from isaac_sim_adapter.scripted_oracle import (
    DEFAULT_GRIPPER_CLOSE_TARGET,
    DEFAULT_HOVER_Z,
    DEFAULT_PICK_Z_OFFSET,
    ORACLE_PHASES,
    OracleReport,
    compute_oracle_targets,
    default_phase_durations_s,
    gate_xy,
    interpolate_gripper,
    interpolate_pose,
    phase_plan,
    xy_distance,
)


def test_oracle_phases_order() -> None:
    assert ORACLE_PHASES == (
        'approach_xy',
        'hover',
        'descend',
        'close',
        'grasp_pause',
        'lift',
        'hold',
    )


def test_compute_oracle_targets_object_relative() -> None:
    obj = (0.35, -0.07, 0.025)
    ee = (0.40, 0.10, 0.45)
    targets = compute_oracle_targets(obj, ee)
    assert targets.object_xyz == obj
    assert targets.approach_xy == (0.35, -0.07, 0.45)
    assert targets.hover == (0.35, -0.07, DEFAULT_HOVER_Z)
    assert targets.pick == (
        0.35,
        -0.07,
        max(0.02, 0.025 + DEFAULT_PICK_Z_OFFSET),
    )
    assert targets.pick[2] == pytest.approx(0.035)
    assert targets.lift == (0.35, -0.07, DEFAULT_HOVER_Z)
    assert targets.gripper_close_target == DEFAULT_GRIPPER_CLOSE_TARGET


def test_pick_z_is_below_cube_top_for_side_grasp() -> None:
    """5 cm cube top is at z=0.05; pick must sit near mid-height, not above top."""
    targets = compute_oracle_targets((0.35, -0.07, 0.025), (0.35, -0.07, 0.4))
    cube_top_z = 0.025 + 0.025
    assert targets.pick[2] < cube_top_z
    assert targets.pick[2] == pytest.approx(0.035)
    assert targets.gripper_close_target == pytest.approx(0.40)


def test_compute_oracle_targets_respects_min_pick_z() -> None:
    obj = (0.40, 0.0, 0.01)
    ee = (0.40, 0.0, 0.30)
    targets = compute_oracle_targets(
        obj, ee, pick_z_offset=0.01, min_pick_z=0.03
    )
    assert targets.pick[2] == pytest.approx(0.03)


def test_compute_oracle_targets_custom_lift_z() -> None:
    targets = compute_oracle_targets(
        (0.35, -0.07, 0.025),
        (0.35, -0.07, 0.40),
        hover_z=0.15,
        lift_z=0.20,
    )
    assert targets.hover[2] == pytest.approx(0.15)
    assert targets.lift[2] == pytest.approx(0.20)


def test_compute_oracle_targets_rejects_bad_hover() -> None:
    with pytest.raises(ValueError, match='hover_z'):
        compute_oracle_targets((0.3, 0.0, 0.02), (0.3, 0.0, 0.4), hover_z=0.0)


def test_phase_plan_matches_oracle_phases() -> None:
    targets = compute_oracle_targets((0.35, -0.07, 0.025), (0.5, 0.0, 0.4))
    plan = phase_plan(targets)
    assert [name for name, _, _ in plan] == list(ORACLE_PHASES)
    assert plan[0][1] == targets.approach_xy
    assert plan[2][1] == targets.pick
    assert plan[3][2] == targets.gripper_close_target
    assert plan[0][2] == targets.gripper_open


def test_gate_xy_and_distance() -> None:
    ok, dist = gate_xy((0.35, -0.07, 0.1), (0.35, -0.07, 0.5), tolerance_m=0.04)
    assert ok
    assert dist == pytest.approx(0.0)
    ok, dist = gate_xy((0.40, -0.07, 0.1), (0.35, -0.07, 0.5), tolerance_m=0.04)
    assert not ok
    assert dist == pytest.approx(0.05)
    assert xy_distance((0.0, 0.0, 1.0), (3.0, 4.0, 9.0)) == pytest.approx(5.0)


def test_interpolate_pose_and_gripper() -> None:
    mid = interpolate_pose((0.0, 0.0, 0.0), (1.0, 2.0, 4.0), 0.5)
    assert mid == pytest.approx((0.5, 1.0, 2.0))
    assert interpolate_gripper(1.0, 0.0, 0.25) == pytest.approx(0.75)
    assert interpolate_gripper(1.0, 0.0, 2.0) == pytest.approx(0.0)


def test_default_phase_durations_cover_all_phases() -> None:
    durations = default_phase_durations_s()
    assert set(durations) == set(ORACLE_PHASES)
    assert all(v > 0.0 for v in durations.values())


def test_oracle_report_never_claims_task_success() -> None:
    report = OracleReport(status='PASS', all_phases_completed=True)
    payload = report.to_dict()
    assert payload['artifact_type'] == 'isaac_scripted_oracle_report'
    assert payload['task_success_claimed'] is False
    assert payload['status'] == 'PASS'
