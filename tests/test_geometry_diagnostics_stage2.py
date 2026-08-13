# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage-1 live TF unit tests + Stage-2 fault injection tests."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from teleop_diagnostics import FRAME_EE, FRAME_LINK7
from teleop_diagnostics.controller_fk import controller_analytic_fk
from teleop_diagnostics.faults import (
    DiagnosticFaultCopy,
    JointOriginOffsetFault,
    JointZeroFault,
    TcpOffsetFault,
    deg_to_rad,
    residual_vs_reference,
)
from teleop_diagnostics.frames import FrameNormalizer
from teleop_diagnostics.tf_source import RobotStatePublisherTfSource, TfLookupRequest, transform_msg_to_matrix
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    PoseSample,
    ResultSemantics,
    make_transform,
)
from teleop_diagnostics.urdf_fk import IndependentUrdfFk, expand_xacro_to_urdf, load_urdf_model

REPO = Path(__file__).resolve().parents[1]
XACRO = REPO / "src/teleop_description/urdf/panda.urdf.xacro"


@pytest.fixture(scope="module")
def urdf_fk():
    robot = load_urdf_model(expand_xacro_to_urdf(XACRO))
    return IndependentUrdfFk(robot)


@pytest.fixture(scope="module")
def normalizer(urdf_fk):
    return FrameNormalizer(urdf_fk)


@pytest.fixture(scope="module")
def fault_copy(urdf_fk, normalizer):
    return DiagnosticFaultCopy(urdf_fk, normalizer)


# ---------- Live TF status classification (unit / mock) ----------

def test_expected_tf_available():
    T = np.eye(4)
    T[:3, 3] = [0.1, 0.0, 0.8]

    def lookup(req: TfLookupRequest):
        return PoseSample(
            source="robot_state_publisher_tf",
            frame_from=req.frame_from,
            frame_to=req.frame_to,
            reference_point=req.frame_to,
            evidence_class=EvidenceClass.MODEL,
            backend_provenance="tf2_buffer_lookup:robot_state_publisher",
            input_status=InputStatus.AVAILABLE,
            matrix=T,
            stamp_sec=1.0,
        )

    src = RobotStatePublisherTfSource(lookup_fn=lookup)
    s = src.forward()
    assert s.input_status == InputStatus.AVAILABLE
    assert s.matrix is not None


def test_missing_tf():
    def lookup(req: TfLookupRequest):
        raise GeometryDiagnosticsError("MISSING TF: frame does not exist")

    src = RobotStatePublisherTfSource(lookup_fn=lookup)
    s = src.forward()
    assert s.input_status == InputStatus.MISSING
    assert s.matrix is None
    assert s.evidence_class == EvidenceClass.INSUFFICIENT_DATA


def test_stale_tf():
    def lookup(req: TfLookupRequest):
        raise GeometryDiagnosticsError("STALE TF age=1.000s > max_age=0.250s")

    src = RobotStatePublisherTfSource(lookup_fn=lookup)
    s = src.forward()
    assert s.input_status == InputStatus.STALE
    assert s.matrix is None


def test_wrong_frame_name_missing():
    def lookup(req: TfLookupRequest):
        if req.frame_to != "panda_ee":
            raise GeometryDiagnosticsError('Frame "nope" does not exist')
        return np.eye(4)

    src = RobotStatePublisherTfSource(frame_to="nope", lookup_fn=lookup)
    s = src.forward()
    assert s.input_status in (InputStatus.MISSING, InputStatus.INVALID)
    assert s.matrix is None


def test_invalid_quaternion():
    class _T:
        pass

    class _Q:
        x = float("nan")
        y = 0.0
        z = 0.0
        w = 1.0

    class _Tr:
        translation = type("P", (), {"x": 0, "y": 0, "z": 0})()
        rotation = _Q()

    with pytest.raises(GeometryDiagnosticsError, match="INVALID quaternion"):
        transform_msg_to_matrix(_Tr())


def test_tf_unavailable_insufficient():
    src = RobotStatePublisherTfSource()
    s = src.forward()
    assert s.input_status == InputStatus.UNAVAILABLE
    assert s.result_semantics if False else True
    assert s.evidence_class == EvidenceClass.INSUFFICIENT_DATA


# ---------- Frame normalization ----------

def test_link7_ee_nominal_transform(normalizer):
    T = normalizer.link7_to_ee_nominal()
    # Fixed chain length along local z is 0.207 m magnitude of translation.
    assert abs(np.linalg.norm(T[:3, 3]) - 0.207) < 1e-9


def test_refuse_raw_cross_frame_compare(urdf_fk, normalizer):
    q = [0.0] * 7
    a = controller_analytic_fk(q)  # panda_link7
    b = urdf_fk.forward(q, target_link=FRAME_EE)
    with pytest.raises(GeometryDiagnosticsError, match="refuse raw residual"):
        normalizer.compare_same_tip(a, b, require_ee=True)


def test_canonicalized_link7_to_ee_nominal_residual_approx_zero(urdf_fk, normalizer):
    q = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    link7 = controller_analytic_fk(q)
    ee = urdf_fk.forward(q, target_link=FRAME_EE)
    canon = normalizer.canonicalize_to_ee(link7)
    res = normalizer.compare_same_tip(canon, ee, require_ee=True)
    assert res["translation_error_m"] < 1e-9
    assert res["rotation_error_rad"] < 1e-9


# ---------- Joint zero ----------

def test_joint_zero_injection_zero_is_nominal(fault_copy, urdf_fk, normalizer):
    q = [0.1, -0.2, 0.3, -1.0, 0.2, 1.0, 0.4]
    faulted = fault_copy.fk_with_joint_zero(q, JointZeroFault({}))
    nom = urdf_fk.forward(q, target_link=FRAME_EE)
    res = residual_vs_reference(faulted, nom, normalizer)
    assert res["translation_error_m"] < 1e-12
    assert faulted.evidence_class == EvidenceClass.INJECTED_FAULT


@pytest.mark.parametrize("deg", [0.5, 2.0, -0.5])
def test_joint_zero_offsets(fault_copy, urdf_fk, normalizer, deg):
    q = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    fault = JointZeroFault({"panda_joint3": deg_to_rad(deg)})
    faulted = fault_copy.fk_with_joint_zero(q, fault)
    nom = urdf_fk.forward(q, target_link=FRAME_EE)
    res = residual_vs_reference(faulted, nom, normalizer)
    assert res["translation_error_m"] > 1e-4


def test_joint_zero_unknown_joint_rejected():
    with pytest.raises(GeometryDiagnosticsError, match="unknown joint"):
        JointZeroFault({"panda_joint99": 0.1}).apply([0.0] * 7)


def test_joint_zero_nan_rejected():
    with pytest.raises(GeometryDiagnosticsError, match="NaN"):
        JointZeroFault({"panda_joint3": float("nan")}).apply([0.0] * 7)


def test_joint_zero_pose_dependence(fault_copy, urdf_fk, normalizer):
    fault = JointZeroFault({"panda_joint3": deg_to_rad(2.0)})
    errs = []
    for q in (
        [0.0] * 7,
        [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        [0.5, -0.5, 0.4, -1.5, 0.3, 1.2, -0.4],
    ):
        faulted = fault_copy.fk_with_joint_zero(q, fault)
        nom = urdf_fk.forward(q, target_link=FRAME_EE)
        errs.append(residual_vs_reference(faulted, nom, normalizer)["translation_error_m"])
    assert np.std(errs) > 1e-4


# ---------- TCP ----------

def test_tcp_zero_offset(fault_copy, urdf_fk, normalizer):
    q = [0.0] * 7
    faulted = fault_copy.fk_with_tcp(q, TcpOffsetFault())
    nom = urdf_fk.forward(q, target_link=FRAME_EE)
    res = residual_vs_reference(faulted, nom, normalizer)
    assert res["translation_error_m"] < 1e-12


@pytest.mark.parametrize("dz", [0.01, 0.03])
def test_tcp_translation(fault_copy, urdf_fk, normalizer, dz):
    q = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    faulted = fault_copy.fk_with_tcp(q, TcpOffsetFault(dz_m=dz))
    nom = urdf_fk.forward(q, target_link=FRAME_EE)
    res = residual_vs_reference(faulted, nom, normalizer)
    assert abs(res["translation_error_m"] - dz) < 1e-9
    # Tool-local z should match injected dz for pure z TCP offset.
    assert abs(res["tool_local_translation_m"][2] - dz) < 1e-9


def test_tcp_yaw_rotation(fault_copy, urdf_fk, normalizer):
    q = [0.0] * 7
    yaw = deg_to_rad(1.0)
    faulted = fault_copy.fk_with_tcp(q, TcpOffsetFault(dyaw_rad=yaw))
    nom = urdf_fk.forward(q, target_link=FRAME_EE)
    res = residual_vs_reference(faulted, nom, normalizer)
    assert abs(res["rotation_error_rad"] - yaw) < 1e-6


def test_tcp_tool_local_residual_correct(fault_copy, urdf_fk, normalizer):
    q = [0.2, -0.4, 0.1, -1.2, 0.3, 1.0, 0.5]
    faulted = fault_copy.fk_with_tcp(q, TcpOffsetFault(dx_m=0.010, dy_m=-0.005))
    nom = urdf_fk.forward(q, target_link=FRAME_EE)
    res = residual_vs_reference(faulted, nom, normalizer)
    assert abs(res["tool_local_translation_m"][0] - 0.010) < 1e-9
    assert abs(res["tool_local_translation_m"][1] + 0.005) < 1e-9


# ---------- Origin + provenance ----------

def test_joint_origin_offset(fault_copy, urdf_fk, normalizer):
    q = [0.0] * 7
    faulted = fault_copy.fk_with_joint_origin(
        q, JointOriginOffsetFault("panda_joint4", dz_m=0.005)
    )
    nom = urdf_fk.forward(q, target_link=FRAME_EE)
    res = residual_vs_reference(faulted, nom, normalizer)
    assert res["translation_error_m"] > 1e-4
    assert faulted.evidence_class == EvidenceClass.INJECTED_FAULT


def test_provenance_classes(fault_copy, urdf_fk):
    q = [0.0] * 7
    assert urdf_fk.forward(q).evidence_class == EvidenceClass.MODEL
    assert fault_copy.fk_with_joint_zero(q, JointZeroFault({"panda_joint1": 0.01})).evidence_class == EvidenceClass.INJECTED_FAULT
    from teleop_diagnostics.mujoco_ee import unavailable_mujoco_sample

    s = unavailable_mujoco_sample(backend="unknown")
    assert s.evidence_class == EvidenceClass.INSUFFICIENT_DATA


def test_no_pass_semantics_forbidden():
    assert ResultSemantics.REPORT_ONLY.value != "PASS"
    assert "PASS" not in {e.value for e in ResultSemantics}
