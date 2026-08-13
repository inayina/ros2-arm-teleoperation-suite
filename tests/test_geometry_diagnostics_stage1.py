# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage-1 geometry diagnostics tests (REPORT_ONLY, fail-closed provenance)."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from teleop_diagnostics.compare import CrossModelComparator
from teleop_diagnostics.controller_fk import CONTROLLER_REFERENCE_POINT, controller_analytic_fk
from teleop_diagnostics.mujoco_ee import MujocoEeSource, unavailable_mujoco_sample
from teleop_diagnostics.poses import READY_Q, fixed_seed_random_poses, near_limit_pose, nominal_pose_set
from teleop_diagnostics.report import assert_no_pass_semantics, git_commit, write_geometry_samples_csv
from teleop_diagnostics.tf_source import RobotStatePublisherTfSource
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    ResultSemantics,
    rotation_geodesic,
    validate_homogeneous,
    validate_rotation_matrix,
)
from teleop_diagnostics.urdf_fk import (
    IndependentKdlFk,
    IndependentUrdfFk,
    expand_xacro_to_urdf,
    load_urdf_model,
)

REPO = Path(__file__).resolve().parents[1]
XACRO = REPO / "src/teleop_description/urdf/panda.urdf.xacro"
MUJOCO_XML = REPO / "config/models/franka_panda.xml"


@pytest.fixture(scope="module")
def robot():
    xml = expand_xacro_to_urdf(XACRO)
    return load_urdf_model(xml)


@pytest.fixture(scope="module")
def comparator(robot):
    return CrossModelComparator(
        urdf_fk=IndependentUrdfFk(robot),
        kdl_fk=IndependentKdlFk(robot),
        mujoco=MujocoEeSource(MUJOCO_XML),
        tf_source=RobotStatePublisherTfSource(),
        commit=git_commit(REPO),
        backend_label="test",
    )


def test_zero_pose_urdf_mujoco_and_controller_reference(comparator):
    q = (0.0,) * 7
    urdf_ee = comparator.urdf_fk.forward(q, target_link="panda_ee")
    sim = comparator.mujoco.forward(q)
    assert sim.evidence_class == EvidenceClass.SIM_GT
    assert abs(urdf_ee.translation()[2] - 0.826) < 1e-6
    assert abs(sim.translation()[2] - 0.826) < 1e-6
    assert np.linalg.norm(urdf_ee.translation() - sim.translation()) < 1e-9

    audit = comparator.controller_reference_audit(q)
    assert audit["closest_urdf_frame_by_translation"] == "panda_link7"
    assert audit["closest_translation_error_m"] < 1e-9
    assert audit["jacobian_matches_fk_contract"] is True
    assert abs(audit["residuals_vs_urdf"]["panda_ee"]["translation_error_m"] - 0.207) < 1e-9
    assert abs(audit["residuals_vs_urdf"]["panda_hand"]["translation_error_m"] - 0.107) < 1e-9


def test_ready_pose_model_vs_sim(comparator):
    urdf_ee = comparator.urdf_fk.forward(READY_Q, target_link="panda_ee")
    sim = comparator.mujoco.forward(READY_Q)
    assert np.linalg.norm(urdf_ee.translation() - sim.translation()) < 1e-8
    assert rotation_geodesic(urdf_ee.rotation(), sim.rotation()) < 1e-6


def test_fixed_seed_random_poses_model_vs_sim_and_kdl(comparator):
    for q in fixed_seed_random_poses(5, seed=20260813):
        urdf = comparator.urdf_fk.forward(q)
        kdl = comparator.kdl_fk.forward(q)
        sim = comparator.mujoco.forward(q)
        assert np.linalg.norm(urdf.translation() - kdl.translation()) < 1e-8
        assert np.linalg.norm(urdf.translation() - sim.translation()) < 1e-7


def test_near_joint_limit_pose_is_finite(comparator):
    q = near_limit_pose()
    pose = comparator.urdf_fk.forward(q)
    assert np.all(np.isfinite(pose.matrix))


def test_invalid_joint_count_fail_closed(comparator):
    with pytest.raises(GeometryDiagnosticsError, match="invalid joint count"):
        comparator.urdf_fk.forward([0.0] * 6)
    with pytest.raises(GeometryDiagnosticsError, match="invalid joint count"):
        controller_analytic_fk([0.0] * 8)


def test_missing_joint_fail_closed(comparator):
    q = {f"panda_joint{i}": 0.0 for i in range(1, 7)}  # missing joint7
    with pytest.raises(GeometryDiagnosticsError, match="missing joint"):
        comparator.urdf_fk.forward(q)


def test_unknown_joint_fail_closed(comparator):
    q = {f"panda_joint{i}": 0.0 for i in range(1, 8)}
    q["panda_joint99"] = 0.1
    with pytest.raises(GeometryDiagnosticsError, match="unknown joint"):
        comparator.urdf_fk.forward(q)


def test_nan_fail_closed(comparator):
    q = [0.0] * 7
    q[3] = float("nan")
    with pytest.raises(GeometryDiagnosticsError, match="NaN"):
        comparator.urdf_fk.forward(q)
    with pytest.raises(GeometryDiagnosticsError, match="NaN"):
        controller_analytic_fk(q)


def test_backend_unavailable_insufficient_data():
    src = MujocoEeSource(Path("/nonexistent/model.xml"))
    sample = src.forward([0.0] * 7)
    assert sample.input_status == InputStatus.UNAVAILABLE
    assert sample.evidence_class == EvidenceClass.INSUFFICIENT_DATA
    assert "SIM_GT" not in sample.evidence_class.value


def test_fallback_backend_must_not_be_sim_gt():
    sample = unavailable_mujoco_sample(backend="fallback")
    assert sample.evidence_class == EvidenceClass.INSUFFICIENT_DATA
    assert sample.evidence_class != EvidenceClass.SIM_GT
    policy = {"mujoco": "SIM_GT", "fallback": "MODEL", "unknown": "INSUFFICIENT_DATA"}
    assert policy["fallback"] == "MODEL"
    assert policy["unknown"] == "INSUFFICIENT_DATA"


def test_unknown_provenance_must_not_pass(comparator):
    rows = comparator.compare_nominal("zero", (0.0,) * 7)
    assert_no_pass_semantics(rows)
    for row in rows:
        assert row.result_semantics in (
            ResultSemantics.REPORT_ONLY.value,
            ResultSemantics.INSUFFICIENT_DATA.value,
        )
        assert row.result_semantics != "PASS"
        assert row.physical == "NOT_RUN/UNAVAILABLE"


def test_tf_unavailable_is_insufficient(comparator):
    rows = comparator.compare_nominal("zero", (0.0,) * 7)
    tf_rows = [r for r in rows if "robot_state_publisher_tf" in (r.source_a, r.source_b)]
    assert tf_rows
    assert all(r.result_semantics == ResultSemantics.INSUFFICIENT_DATA.value for r in tf_rows)


def test_live_tf_lookup_path():
    T = np.eye(4)
    T[:3, 3] = [0.1, 0.2, 0.3]

    def lookup(frame_from, frame_to):
        assert frame_from == "panda_link0"
        assert frame_to == "panda_ee"
        return T

    src = RobotStatePublisherTfSource(lookup_fn=lookup)
    sample = src.forward()
    assert sample.input_status == InputStatus.AVAILABLE
    assert sample.evidence_class == EvidenceClass.MODEL
    assert sample.backend_provenance.startswith("tf2_buffer_lookup")


def test_rotation_quaternion_matrix_validity():
    R = np.eye(3)
    validate_rotation_matrix(R)
    R2 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    validate_rotation_matrix(R2)
    assert abs(rotation_geodesic(R, R2) - math.pi / 2) < 1e-9
    bad = np.eye(3)
    bad[0, 0] = 2.0
    with pytest.raises(GeometryDiagnosticsError):
        validate_rotation_matrix(bad)
    T = np.eye(4)
    T[:3, :3] = R2
    validate_homogeneous(T)


def test_controller_reference_point_comparison_multi_pose(comparator):
    assert CONTROLLER_REFERENCE_POINT == "panda_link7"
    for pose in nominal_pose_set(random_count=3, seed=7):
        audit = comparator.controller_reference_audit(pose.q)
        assert audit["closest_urdf_frame_by_translation"] == "panda_link7"
        assert audit["residuals_vs_urdf"]["panda_link7"]["translation_error_m"] < 1e-8
        assert (
            audit["residuals_vs_urdf"]["panda_ee"]["translation_error_m"]
            > audit["residuals_vs_urdf"]["panda_link7"]["translation_error_m"]
        )


def test_does_not_reuse_mujoco_fallback_constants():
    import teleop_diagnostics.urdf_fk as urdf_fk
    import teleop_diagnostics.mujoco_ee as mujoco_ee

    src = Path(urdf_fk.__file__).read_text(encoding="utf-8")
    src2 = Path(mujoco_ee.__file__).read_text(encoding="utf-8")
    assert "fallback_ee_transform" not in src
    assert "FALLBACK_JOINT_ORIGINS" not in src
    assert "fallback_ee_transform" not in src2
    assert "FALLBACK_HAND_ORIGIN" not in src2


def test_csv_writer_roundtrip(tmp_path, comparator):
    rows = comparator.compare_nominal("ready", READY_Q)
    path = tmp_path / "geometry_samples.csv"
    write_geometry_samples_csv(path, rows)
    text = path.read_text(encoding="utf-8")
    assert "translation_error_m" in text
    assert "REPORT_ONLY" in text or "INSUFFICIENT_DATA" in text
    assert "NOT_RUN/UNAVAILABLE" in text
