# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Replica of teleop_controllers analytic DH FK for cross-model comparison only.

Provenance is the C++ impedance_math.cpp modified-DH table. This is NOT an
independent authority and must never be labeled SIM_GT.
"""

from __future__ import annotations

import math
from typing import Mapping, Sequence

import numpy as np

from teleop_diagnostics import FRAME_BASE, FRAME_LINK7, PANDA_ARM_JOINTS
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    PoseSample,
    validate_homogeneous,
)

# Mirror of teleop_controllers/src/impedance_math.cpp kDH
# Rows: a, d, alpha, offset (modified DH / Craig).
_CONTROLLER_DH = (
    (0.0, 0.333, 0.0, 0.0),
    (0.0, 0.0, -math.pi / 2.0, 0.0),
    (0.0, 0.316, math.pi / 2.0, 0.0),
    (0.0825, 0.0, math.pi / 2.0, 0.0),
    (-0.0825, 0.384, -math.pi / 2.0, 0.0),
    (0.0, 0.0, math.pi / 2.0, 0.0),
    (0.088, 0.0, math.pi / 2.0, 0.0),
)

CONTROLLER_REFERENCE_POINT = FRAME_LINK7
CONTROLLER_FK_PROVENANCE = (
    "teleop_controllers/src/impedance_math.cpp:forward_kinematics "
    "(modified DH, stops after joint 7; Stage-1 contract = panda_link7)"
)


def _dh_link(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -sa * d],
            [st * sa, ct * sa, ca, ca * d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _q_vector(q: Sequence[float] | Mapping[str, float]) -> np.ndarray:
    if isinstance(q, Mapping):
        missing = [n for n in PANDA_ARM_JOINTS if n not in q]
        if missing:
            raise GeometryDiagnosticsError(f"missing joint(s): {missing}")
        for n in q:
            if n.startswith("panda_joint") and n not in PANDA_ARM_JOINTS:
                raise GeometryDiagnosticsError(f"unknown joint: {n}")
        vals = [float(q[n]) for n in PANDA_ARM_JOINTS]
    else:
        vals = [float(v) for v in q]
        if len(vals) != 7:
            raise GeometryDiagnosticsError(
                f"invalid joint count: got {len(vals)}, expected 7"
            )
    if not all(math.isfinite(v) for v in vals):
        raise GeometryDiagnosticsError("NaN/Inf in controller FK joint vector")
    return np.asarray(vals, dtype=float)


def controller_analytic_fk(q: Sequence[float] | Mapping[str, float]) -> PoseSample:
    qv = _q_vector(q)
    T = np.eye(4)
    for i, (a, d, alpha, offset) in enumerate(_CONTROLLER_DH):
        T = T @ _dh_link(a, d, alpha, float(qv[i]) + offset)
    T = validate_homogeneous(T)
    return PoseSample(
        source="controller_analytic_fk",
        frame_from=FRAME_BASE,
        frame_to=CONTROLLER_REFERENCE_POINT,
        reference_point=CONTROLLER_REFERENCE_POINT,
        evidence_class=EvidenceClass.MODEL,
        backend_provenance=CONTROLLER_FK_PROVENANCE,
        input_status=InputStatus.AVAILABLE,
        matrix=T,
        detail="Jacobian in impedance_math.cpp uses the same FK origin as p_ee",
    )


def controller_jacobian_reference_point() -> str:
    """Jacobian p_ee is the origin after 7 DH joints (= FK translation)."""
    return CONTROLLER_REFERENCE_POINT
