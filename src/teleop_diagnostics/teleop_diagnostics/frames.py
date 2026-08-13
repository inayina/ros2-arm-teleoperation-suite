# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Frame normalization: never residual-compare mismatched tip frames raw."""

from __future__ import annotations

from typing import Optional

import numpy as np

from teleop_diagnostics import FRAME_BASE, FRAME_EE, FRAME_HAND, FRAME_LINK7
from teleop_diagnostics.types import (
    GeometryDiagnosticsError,
    PoseSample,
    rotation_geodesic,
    tool_local_translation_error,
    translation_error,
    validate_homogeneous,
)
from teleop_diagnostics.urdf_fk import IndependentUrdfFk


class FrameNormalizer:
    """Canonical fixed transforms from URDF (MODEL), not MuJoCo XML constants."""

    def __init__(self, urdf_fk: IndependentUrdfFk):
        self.urdf_fk = urdf_fk
        # Fixed chain is configuration-independent; evaluate at q=0.
        q0 = [0.0] * 7
        T_base_link7 = urdf_fk.forward(q0, target_link=FRAME_LINK7).matrix
        T_base_hand = urdf_fk.forward(q0, target_link=FRAME_HAND).matrix
        T_base_ee = urdf_fk.forward(q0, target_link=FRAME_EE).matrix
        assert T_base_link7 is not None and T_base_hand is not None and T_base_ee is not None
        self.T_link7_hand = validate_homogeneous(
            np.linalg.inv(T_base_link7) @ T_base_hand
        )
        self.T_hand_ee = validate_homogeneous(np.linalg.inv(T_base_hand) @ T_base_ee)
        self.T_link7_ee_nominal = validate_homogeneous(
            self.T_link7_hand @ self.T_hand_ee
        )

    def link7_to_ee_nominal(self) -> np.ndarray:
        return self.T_link7_ee_nominal.copy()

    def canonicalize_to_ee(self, sample: PoseSample) -> PoseSample:
        """Map a pose sample onto panda_ee when possible; refuse unknown tips."""
        if sample.matrix is None:
            raise GeometryDiagnosticsError(
                f"cannot canonicalize {sample.source}: matrix unavailable "
                f"(status={sample.input_status.value})"
            )
        tip = sample.frame_to
        if tip == FRAME_EE:
            T = validate_homogeneous(sample.matrix)
        elif tip == FRAME_LINK7:
            T = validate_homogeneous(sample.matrix @ self.T_link7_ee_nominal)
        elif tip == FRAME_HAND:
            T = validate_homogeneous(sample.matrix @ self.T_hand_ee)
        else:
            raise GeometryDiagnosticsError(
                f"refuse canonicalize unknown tip frame_to={tip}"
            )
        return PoseSample(
            source=f"{sample.source}__canonical_ee",
            frame_from=sample.frame_from or FRAME_BASE,
            frame_to=FRAME_EE,
            reference_point=FRAME_EE,
            evidence_class=sample.evidence_class,
            backend_provenance=f"{sample.backend_provenance}|normalized_via_urdf_fixed_chain",
            input_status=sample.input_status,
            matrix=T,
            detail=f"canonicalized from {tip} using URDF fixed chain",
            stamp_sec=sample.stamp_sec,
        )

    def compare_same_tip(
        self,
        a: PoseSample,
        b: PoseSample,
        *,
        require_ee: bool = True,
    ) -> dict:
        """Compare only after both samples share the same tip (default panda_ee)."""
        if a.frame_to != b.frame_to:
            raise GeometryDiagnosticsError(
                f"refuse raw residual across different frames: "
                f"{a.source}:{a.frame_to} vs {b.source}:{b.frame_to}. "
                f"Normalize first (e.g. canonicalize_to_ee)."
            )
        if require_ee and a.frame_to != FRAME_EE:
            raise GeometryDiagnosticsError(
                f"refuse residual on non-ee tip without explicit allow: frame_to={a.frame_to}"
            )
        if a.matrix is None or b.matrix is None:
            raise GeometryDiagnosticsError("refuse residual: matrix missing")
        base_t = translation_error(a.matrix, b.matrix)
        base_r = rotation_geodesic(a.matrix[:3, :3], b.matrix[:3, :3])
        local = tool_local_translation_error(a.matrix, b.matrix)
        return {
            "frame_from": a.frame_from,
            "frame_to": a.frame_to,
            "translation_error_m": base_t,
            "rotation_error_rad": base_r,
            "tool_local_error_m": float(np.linalg.norm(local)),
            "tool_local_translation_m": [float(x) for x in local],
        }
