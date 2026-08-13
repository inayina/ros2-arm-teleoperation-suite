# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Diagnostic-only fault injection on model copies (never mutates runtime topics)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Optional, Sequence

import numpy as np

from teleop_diagnostics import FRAME_BASE, FRAME_EE, FRAME_LINK7, PANDA_ARM_JOINTS
from teleop_diagnostics.frames import FrameNormalizer
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    PoseSample,
    ResultSemantics,
    make_transform,
    rotation_geodesic,
    tool_local_translation_error,
    translation_error,
    validate_homogeneous,
)
from teleop_diagnostics.urdf_fk import IndependentUrdfFk


def deg_to_rad(deg: float) -> float:
    return float(deg) * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    return float(rad) * 180.0 / math.pi


@dataclass
class JointZeroFault:
    """Injected joint-zero offset (radians). Not a calibrated joint zero."""

    offsets_rad: dict[str, float] = field(default_factory=dict)

    def apply(self, q: Sequence[float] | Mapping[str, float]) -> list[float]:
        if isinstance(q, Mapping):
            missing = [n for n in PANDA_ARM_JOINTS if n not in q]
            if missing:
                raise GeometryDiagnosticsError(f"missing joint(s): {missing}")
            for n in q:
                if n.startswith("panda_joint") and n not in PANDA_ARM_JOINTS:
                    raise GeometryDiagnosticsError(f"unknown joint: {n}")
            base = {n: float(q[n]) for n in PANDA_ARM_JOINTS}
        else:
            vals = list(q)
            if len(vals) != 7:
                raise GeometryDiagnosticsError(
                    f"invalid joint count: got {len(vals)}, expected 7"
                )
            base = {n: float(v) for n, v in zip(PANDA_ARM_JOINTS, vals)}
        for name, off in self.offsets_rad.items():
            if name not in PANDA_ARM_JOINTS:
                raise GeometryDiagnosticsError(f"unknown joint for zero offset: {name}")
            if not math.isfinite(off):
                raise GeometryDiagnosticsError(f"NaN/Inf joint-zero offset: {name}")
            base[name] = base[name] + float(off)
        out = [base[n] for n in PANDA_ARM_JOINTS]
        if not all(math.isfinite(v) for v in out):
            raise GeometryDiagnosticsError("NaN/Inf after joint-zero injection")
        return out

    def describe(self) -> dict:
        return {
            "fault_type": "joint_zero_offset",
            "offsets_rad": {k: float(v) for k, v in self.offsets_rad.items()},
            "offsets_deg": {k: rad_to_deg(v) for k, v in self.offsets_rad.items()},
            "label": "injected joint-zero offset",
        }


@dataclass
class TcpOffsetFault:
    """ΔT applied on T_link7_ee in the diagnostics copy (does not modify URDF)."""

    dx_m: float = 0.0
    dy_m: float = 0.0
    dz_m: float = 0.0
    droll_rad: float = 0.0
    dpitch_rad: float = 0.0
    dyaw_rad: float = 0.0

    def delta_transform(self) -> np.ndarray:
        vals = [self.dx_m, self.dy_m, self.dz_m, self.droll_rad, self.dpitch_rad, self.dyaw_rad]
        if not all(math.isfinite(v) for v in vals):
            raise GeometryDiagnosticsError("NaN/Inf in TCP offset")
        return make_transform(
            (self.dx_m, self.dy_m, self.dz_m),
            (self.droll_rad, self.dpitch_rad, self.dyaw_rad),
        )

    def apply_to_link7_ee(self, T_link7_ee_nominal: np.ndarray) -> np.ndarray:
        # Fault in tool-local sense: nominal * delta (right-multiply).
        return validate_homogeneous(T_link7_ee_nominal @ self.delta_transform())

    def describe(self) -> dict:
        return {
            "fault_type": "tcp_offset",
            "tcp_dx_m": self.dx_m,
            "tcp_dy_m": self.dy_m,
            "tcp_dz_m": self.dz_m,
            "tcp_droll_rad": self.droll_rad,
            "tcp_dpitch_rad": self.dpitch_rad,
            "tcp_dyaw_rad": self.dyaw_rad,
            "label": "injected TCP offset on diagnostic T_link7_ee copy",
        }


@dataclass
class JointOriginOffsetFault:
    """Minimal injected joint-origin translation bias on a diagnostic URDF copy path.

    Implemented as a post-FK tip bias is insufficient; instead we apply an
    additional fixed transform after the named joint's child link FK when
    reconstructing EE. For Stage 2 we support a single joint origin xyz bias
    applied in that joint's child frame before continuing the chain — via
    composing an extra transform into the FK by re-running with a patched
    intermediate. Simpler Stage-2 approach: bias is applied as
    T_base_joint_child_faulted = T_base_joint_child * Trans(dx,dy,dz), then
    continue with remaining fixed chain to EE using nominal relative transforms.
    """

    joint_name: str
    dx_m: float = 0.0
    dy_m: float = 0.0
    dz_m: float = 0.0

    def describe(self) -> dict:
        return {
            "fault_type": "joint_origin_offset",
            "joint_name": self.joint_name,
            "origin_dx_m": self.dx_m,
            "origin_dy_m": self.dy_m,
            "origin_dz_m": self.dz_m,
            "label": "injected joint origin translation bias (diagnostic copy)",
        }


class DiagnosticFaultCopy:
    """Applies faults only to diagnostic FK copies. Never publishes /joint_states."""

    def __init__(self, urdf_fk: IndependentUrdfFk, normalizer: FrameNormalizer):
        self.urdf_fk = urdf_fk
        self.normalizer = normalizer

    def fk_with_joint_zero(
        self,
        q: Sequence[float],
        fault: JointZeroFault,
    ) -> PoseSample:
        q_f = fault.apply(q)
        sample = self.urdf_fk.forward(q_f, target_link=FRAME_EE)
        sample.evidence_class = EvidenceClass.INJECTED_FAULT
        sample.backend_provenance = (
            f"{sample.backend_provenance}|injected_joint_zero_offset"
        )
        sample.detail = str(fault.describe())
        sample.source = "diagnostic_fk_joint_zero"
        return sample

    def fk_with_tcp(
        self,
        q: Sequence[float],
        fault: TcpOffsetFault,
    ) -> PoseSample:
        link7 = self.urdf_fk.forward(q, target_link=FRAME_LINK7)
        assert link7.matrix is not None
        T_link7_ee_f = fault.apply_to_link7_ee(self.normalizer.T_link7_ee_nominal)
        T_ee = validate_homogeneous(link7.matrix @ T_link7_ee_f)
        return PoseSample(
            source="diagnostic_fk_tcp_offset",
            frame_from=FRAME_BASE,
            frame_to=FRAME_EE,
            reference_point=FRAME_EE,
            evidence_class=EvidenceClass.INJECTED_FAULT,
            backend_provenance="urdf_fk_link7_x_T_link7_ee_faulted",
            input_status=InputStatus.AVAILABLE,
            matrix=T_ee,
            detail=str(fault.describe()),
        )

    def fk_with_joint_origin(
        self,
        q: Sequence[float],
        fault: JointOriginOffsetFault,
    ) -> PoseSample:
        if fault.joint_name not in PANDA_ARM_JOINTS:
            raise GeometryDiagnosticsError(
                f"unknown joint for origin offset: {fault.joint_name}"
            )
        if not all(
            math.isfinite(v) for v in (fault.dx_m, fault.dy_m, fault.dz_m)
        ):
            raise GeometryDiagnosticsError("NaN/Inf joint origin offset")
        # Child link of joint i is panda_link{i}
        idx = PANDA_ARM_JOINTS.index(fault.joint_name) + 1
        child = f"panda_link{idx}"
        T_base_child = self.urdf_fk.forward(q, target_link=child).matrix
        assert T_base_child is not None
        bias = make_transform((fault.dx_m, fault.dy_m, fault.dz_m), (0.0, 0.0, 0.0))
        T_base_child_f = validate_homogeneous(T_base_child @ bias)
        # Remaining chain: child → ... → ee at this q via relative nominal transforms.
        T_base_ee_nom = self.urdf_fk.forward(q, target_link=FRAME_EE).matrix
        assert T_base_ee_nom is not None
        T_child_ee = validate_homogeneous(np.linalg.inv(T_base_child) @ T_base_ee_nom)
        T_ee = validate_homogeneous(T_base_child_f @ T_child_ee)
        return PoseSample(
            source="diagnostic_fk_joint_origin",
            frame_from=FRAME_BASE,
            frame_to=FRAME_EE,
            reference_point=FRAME_EE,
            evidence_class=EvidenceClass.INJECTED_FAULT,
            backend_provenance=f"urdf_fk_origin_bias:{fault.joint_name}",
            input_status=InputStatus.AVAILABLE,
            matrix=T_ee,
            detail=str(fault.describe()),
        )


def classify_fault_residual(
    *,
    fault_type: str,
    translation_error_m: float,
    tool_local_xyz: Sequence[float],
    pose_dependence_std_m: float,
    zero_injection_residual_m: float,
) -> dict:
    """Heuristic pattern label only — never PASS/CALIBRATED/ROOT_CAUSE_CONFIRMED."""
    if zero_injection_residual_m > 1e-6:
        return {
            "status": ResultSemantics.ERROR_INPUT.value,
            "suspected_cause": "NOMINAL_BASELINE_NOT_CLEAN",
            "result_semantics": ResultSemantics.ERROR_INPUT.value,
            "confidence": "diagnostic pattern only",
        }
    if translation_error_m < 1e-9 and abs(pose_dependence_std_m) < 1e-9:
        return {
            "status": ResultSemantics.REPORT_ONLY.value,
            "suspected_cause": "NONE_ZERO_INJECTION",
            "result_semantics": ResultSemantics.REPORT_ONLY.value,
            "confidence": "diagnostic pattern only",
        }
    if fault_type == "joint_zero_offset":
        if pose_dependence_std_m > 1e-4:
            return {
                "status": ResultSemantics.SUSPECTED.value,
                "suspected_cause": "JOINT_ZERO_OFFSET",
                "result_semantics": ResultSemantics.SUSPECTED.value,
                "confidence": "diagnostic pattern only",
                "pattern": "pose-dependent",
            }
        return {
            "status": ResultSemantics.AMBIGUOUS.value,
            "suspected_cause": "JOINT_ZERO_OR_OTHER",
            "result_semantics": ResultSemantics.AMBIGUOUS.value,
            "confidence": "diagnostic pattern only",
            "pattern": "weak pose-dependence",
        }
    if fault_type == "tcp_offset":
        # Tool-local residual should stay close to the injected local xyz for pure translation.
        return {
            "status": ResultSemantics.SUSPECTED.value,
            "suspected_cause": "TCP_OFFSET",
            "result_semantics": ResultSemantics.SUSPECTED.value,
            "confidence": "diagnostic pattern only",
            "pattern": "tool-local bias",
            "tool_local_translation_m": [float(x) for x in tool_local_xyz],
        }
    return {
        "status": ResultSemantics.SUSPECTED.value,
        "suspected_cause": fault_type.upper(),
        "result_semantics": ResultSemantics.SUSPECTED.value,
        "confidence": "diagnostic pattern only",
    }


def residual_vs_reference(
    faulted: PoseSample,
    reference: PoseSample,
    normalizer: FrameNormalizer,
) -> dict:
    """Normalize both to panda_ee before residual; refuse raw cross-frame subtract."""
    a = normalizer.canonicalize_to_ee(faulted) if faulted.frame_to != FRAME_EE else faulted
    b = (
        normalizer.canonicalize_to_ee(reference)
        if reference.frame_to != FRAME_EE
        else reference
    )
    return normalizer.compare_same_tip(b, a, require_ee=True)
