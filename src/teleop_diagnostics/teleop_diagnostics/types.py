# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Evidence classes, input status, and pose sample types."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Optional

import numpy as np


class EvidenceClass(str, Enum):
    MODEL = "MODEL"
    SIM_GT = "SIM_GT"
    INJECTED_FAULT = "INJECTED_FAULT"
    ESTIMATED = "ESTIMATED"
    PHYSICAL = "PHYSICAL"
    INSUFFICIENT_DATA = "INSUFFICIENT_DATA"


class InputStatus(str, Enum):
    AVAILABLE = "AVAILABLE"
    STALE = "STALE"
    MISSING = "MISSING"
    INVALID = "INVALID"
    UNAVAILABLE = "UNAVAILABLE"


class ResultSemantics(str, Enum):
    REPORT_ONLY = "REPORT_ONLY"
    INSUFFICIENT_DATA = "INSUFFICIENT_DATA"
    SUSPECTED = "SUSPECTED"
    AMBIGUOUS = "AMBIGUOUS"
    ERROR_INPUT = "ERROR_INPUT"


class GeometryDiagnosticsError(ValueError):
    """Fail-closed geometry diagnostics error."""


@dataclass
class PoseSample:
    source: str
    frame_from: str
    frame_to: str
    reference_point: str
    evidence_class: EvidenceClass
    backend_provenance: str
    input_status: InputStatus
    matrix: Optional[np.ndarray] = None  # 4x4
    detail: str = ""
    stamp_sec: Optional[float] = None

    def translation(self) -> Optional[np.ndarray]:
        if self.matrix is None:
            return None
        return np.asarray(self.matrix[:3, 3], dtype=float)

    def rotation(self) -> Optional[np.ndarray]:
        if self.matrix is None:
            return None
        return np.asarray(self.matrix[:3, :3], dtype=float)

    def to_dict(self) -> dict[str, Any]:
        out = {
            "source": self.source,
            "frame_from": self.frame_from,
            "frame_to": self.frame_to,
            "reference_point": self.reference_point,
            "evidence_class": self.evidence_class.value,
            "backend_provenance": self.backend_provenance,
            "input_status": self.input_status.value,
            "detail": self.detail,
            "stamp_sec": self.stamp_sec,
        }
        t = self.translation()
        if t is not None:
            out["translation_m"] = [float(x) for x in t]
        return out


@dataclass
class ResidualRow:
    scenario: str
    q: list[float]
    source_a: str
    source_b: str
    frame_from: str
    frame_to: str
    reference_point: str
    translation_error_m: Optional[float]
    rotation_error_rad: Optional[float]
    evidence_class_a: str
    evidence_class_b: str
    input_status: str
    result_semantics: str
    backend: str
    commit: str
    physical: str = "NOT_RUN/UNAVAILABLE"
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        extra = d.pop("extra", {})
        d.update(extra)
        return d


def validate_rotation_matrix(R: np.ndarray, tol: float = 1e-6) -> None:
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3) or not np.all(np.isfinite(R)):
        raise GeometryDiagnosticsError("rotation matrix invalid: shape/NaN")
    orth = R.T @ R
    if np.linalg.norm(orth - np.eye(3)) > tol:
        raise GeometryDiagnosticsError("rotation matrix not orthonormal")
    if abs(np.linalg.det(R) - 1.0) > tol:
        raise GeometryDiagnosticsError("rotation matrix determinant != 1")


def validate_homogeneous(T: np.ndarray, tol: float = 1e-6) -> np.ndarray:
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise GeometryDiagnosticsError("transform invalid: shape/NaN")
    if abs(T[3, 3] - 1.0) > tol or np.linalg.norm(T[3, :3]) > tol:
        raise GeometryDiagnosticsError("transform bottom row invalid")
    validate_rotation_matrix(T[:3, :3], tol=tol)
    return T


def rotation_geodesic(Ra: np.ndarray, Rb: np.ndarray) -> float:
    validate_rotation_matrix(Ra)
    validate_rotation_matrix(Rb)
    R = Ra.T @ Rb
    c = float((np.trace(R) - 1.0) * 0.5)
    c = max(-1.0, min(1.0, c))
    return float(np.arccos(c))


def translation_error(Ta: np.ndarray, Tb: np.ndarray) -> float:
    validate_homogeneous(Ta)
    validate_homogeneous(Tb)
    return float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]))


def tool_local_translation_error(T_ref: np.ndarray, T_meas: np.ndarray) -> np.ndarray:
    """Express translation residual in the reference tool frame: R_ref^T (p_meas - p_ref)."""
    validate_homogeneous(T_ref)
    validate_homogeneous(T_meas)
    dp = T_meas[:3, 3] - T_ref[:3, 3]
    return T_ref[:3, :3].T @ dp


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    import math

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def make_transform(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return validate_homogeneous(T)
