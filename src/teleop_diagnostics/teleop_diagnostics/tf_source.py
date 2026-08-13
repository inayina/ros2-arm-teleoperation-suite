# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""robot_state_publisher TF pose source with fail-closed status classification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Union

import numpy as np

from teleop_diagnostics import FRAME_BASE, FRAME_EE
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    PoseSample,
    validate_homogeneous,
)


@dataclass
class TfLookupRequest:
    frame_from: str = FRAME_BASE
    frame_to: str = FRAME_EE
    # Absolute stamp seconds, or None for "latest".
    stamp_sec: Optional[float] = None
    max_age_sec: float = 0.25


class RobotStatePublisherTfSource:
    """Lookup T_frame_from_frame_to from a TF buffer when available.

    Offline / no-buffer: input_status=UNAVAILABLE, evidence_class=INSUFFICIENT_DATA.
    Topic name alone is never used as provenance.
    Never returns identity on failure.
    """

    def __init__(
        self,
        *,
        frame_from: str = FRAME_BASE,
        frame_to: str = FRAME_EE,
        lookup_fn: Optional[
            Callable[[TfLookupRequest], Union[PoseSample, np.ndarray]]
        ] = None,
        backend_provenance: str = "tf2_buffer_lookup:robot_state_publisher",
        max_age_sec: float = 0.25,
    ):
        self.frame_from = frame_from
        self.frame_to = frame_to
        self.lookup_fn = lookup_fn
        self.backend_provenance = backend_provenance
        self.max_age_sec = max_age_sec

    def forward(
        self,
        q=None,
        *,
        stamp_sec: Optional[float] = None,
        max_age_sec: Optional[float] = None,
    ) -> PoseSample:
        del q
        if self.lookup_fn is None:
            return PoseSample(
                source="robot_state_publisher_tf",
                frame_from=self.frame_from,
                frame_to=self.frame_to,
                reference_point=self.frame_to,
                evidence_class=EvidenceClass.INSUFFICIENT_DATA,
                backend_provenance="tf_backend=unavailable",
                input_status=InputStatus.UNAVAILABLE,
                matrix=None,
                detail="No TF buffer; live robot_state_publisher not attached",
            )
        req = TfLookupRequest(
            frame_from=self.frame_from,
            frame_to=self.frame_to,
            stamp_sec=stamp_sec,
            max_age_sec=self.max_age_sec if max_age_sec is None else max_age_sec,
        )
        try:
            # Support both TfLookupRequest callables and legacy (frame_from, frame_to).
            try:
                result = self.lookup_fn(req)
            except TypeError:
                result = self.lookup_fn(req.frame_from, req.frame_to)
            if isinstance(result, PoseSample):
                if result.matrix is None and result.input_status == InputStatus.AVAILABLE:
                    raise GeometryDiagnosticsError(
                        "TF lookup returned AVAILABLE without matrix"
                    )
                return result
            T = validate_homogeneous(np.asarray(result))
            return PoseSample(
                source="robot_state_publisher_tf",
                frame_from=self.frame_from,
                frame_to=self.frame_to,
                reference_point=self.frame_to,
                evidence_class=EvidenceClass.MODEL,
                backend_provenance=self.backend_provenance,
                input_status=InputStatus.AVAILABLE,
                matrix=T,
                stamp_sec=stamp_sec,
            )
        except GeometryDiagnosticsError as exc:
            status = InputStatus.INVALID
            msg = str(exc)
            if "MISSING" in msg.upper() or "does not exist" in msg.lower():
                status = InputStatus.MISSING
            elif "STALE" in msg.upper():
                status = InputStatus.STALE
            return PoseSample(
                source="robot_state_publisher_tf",
                frame_from=self.frame_from,
                frame_to=self.frame_to,
                reference_point=self.frame_to,
                evidence_class=EvidenceClass.INSUFFICIENT_DATA,
                backend_provenance="tf_backend=lookup_failed",
                input_status=status,
                matrix=None,
                detail=msg,
            )
        except Exception as exc:  # noqa: BLE001
            msg = str(exc)
            status = InputStatus.INVALID
            low = msg.lower()
            if "does not exist" in low or "frame" in low and "not" in low:
                status = InputStatus.MISSING
            elif "extrapolation" in low or "past" in low:
                status = InputStatus.STALE
            return PoseSample(
                source="robot_state_publisher_tf",
                frame_from=self.frame_from,
                frame_to=self.frame_to,
                reference_point=self.frame_to,
                evidence_class=EvidenceClass.INSUFFICIENT_DATA,
                backend_provenance="tf_backend=lookup_failed",
                input_status=status,
                matrix=None,
                detail=msg,
            )


def transform_msg_to_matrix(transform) -> np.ndarray:
    """Convert geometry_msgs Transform to 4x4; reject invalid quaternions."""
    t = transform.translation
    q = transform.rotation
    n = float(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not np.isfinite(n) or n < 1e-12:
        raise GeometryDiagnosticsError("INVALID quaternion (zero/NaN)")
    # Normalize and build rotation
    s = 1.0 / np.sqrt(n)
    x, y, z, w = q.x * s, q.y * s, q.z * s, q.w * s
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return validate_homogeneous(T)
