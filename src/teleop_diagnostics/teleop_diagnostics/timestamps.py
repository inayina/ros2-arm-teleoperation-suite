# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Publication-time stamp skew diagnostics (Stage 4, REPORT_ONLY).

v1 measures **Header publication time** deltas on diagnostic copies.

It does **not** measure source/acquisition time. There is no `/clock`, no
shared physics sample ID, and several modalities (gripper Float64) have no
Header. SOURCE_TIME_SKEW is always UNAVAILABLE.

Signed convention::

    signed_delta_s = t_modality - t_anchor

Positive means the modality Header is later than the anchor (typically scene
image). Recorder ``MultiModalSync`` currently gates on ``abs(delta)``; this
module reports the signed value without changing that gate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Iterable, Optional, Sequence

from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    ResultSemantics,
)

# Recorder default slop; diagnostics report against it but do not change it.
DEFAULT_RECORDER_SLOP_S = 0.05

CONTROLLED_DELAYS_S = (-0.100, -0.050, -0.030, -0.010, 0.0, 0.010, 0.030, 0.050, 0.100)

ANCHOR_MODALITY = "color"
PAIRED_MODALITIES = ("joint", "ee", "object", "ft", "wrist")


class StampSemantics(str, Enum):
    PUBLICATION_HEADER = "PUBLICATION_HEADER"
    NO_HEADER = "NO_HEADER"
    UNKNOWN = "UNKNOWN"


class SkewClass(str, Enum):
    PUBLISH_TIME_SKEW = "PUBLISH_TIME_SKEW"
    SOURCE_TIME_SKEW = "SOURCE_TIME_SKEW"


class SequenceFlag(str, Enum):
    FIRST = "FIRST"
    IN_ORDER = "IN_ORDER"
    DUPLICATE = "DUPLICATE"
    OUT_OF_ORDER = "OUT_OF_ORDER"


@dataclass
class StampSample:
    modality: str
    topic: str
    stamp_sec: Optional[float]
    stamp_semantics: StampSemantics
    input_status: InputStatus
    evidence_class: EvidenceClass = EvidenceClass.MODEL
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "modality": self.modality,
            "topic": self.topic,
            "stamp_sec": self.stamp_sec,
            "stamp_semantics": self.stamp_semantics.value,
            "input_status": self.input_status.value,
            "evidence_class": self.evidence_class.value,
            "detail": self.detail,
        }


@dataclass
class SkewRow:
    scenario: str
    anchor_modality: str
    other_modality: str
    signed_delta_s: Optional[float]
    abs_delta_s: Optional[float]
    injected_delay_s: float
    recovered_delay_s: Optional[float]
    skew_class: str
    source_time_status: str
    sequence_flag: str
    input_status: str
    result_semantics: str
    evidence_class: str
    physical: str = "NOT_RUN/UNAVAILABLE"
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d = {
            "scenario": self.scenario,
            "anchor_modality": self.anchor_modality,
            "other_modality": self.other_modality,
            "signed_delta_s": self.signed_delta_s,
            "abs_delta_s": self.abs_delta_s,
            "injected_delay_s": self.injected_delay_s,
            "recovered_delay_s": self.recovered_delay_s,
            "skew_class": self.skew_class,
            "source_time_status": self.source_time_status,
            "sequence_flag": self.sequence_flag,
            "input_status": self.input_status,
            "result_semantics": self.result_semantics,
            "evidence_class": self.evidence_class,
            "physical": self.physical,
        }
        d.update(self.extra)
        return d


def header_stamp_sec(msg) -> StampSample:
    """Extract ROS Header stamp; fail-closed on missing/invalid values."""
    modality = getattr(msg, "_diag_modality", type(msg).__name__)
    topic = getattr(msg, "_diag_topic", "")
    header = getattr(msg, "header", None)
    if header is None:
        return StampSample(
            modality=modality,
            topic=topic,
            stamp_sec=None,
            stamp_semantics=StampSemantics.NO_HEADER,
            input_status=InputStatus.UNAVAILABLE,
            evidence_class=EvidenceClass.INSUFFICIENT_DATA,
            detail="message has no Header (cannot form PUBLISH_TIME_SKEW)",
        )
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return StampSample(
            modality=modality,
            topic=topic,
            stamp_sec=None,
            stamp_semantics=StampSemantics.PUBLICATION_HEADER,
            input_status=InputStatus.MISSING,
            evidence_class=EvidenceClass.INSUFFICIENT_DATA,
            detail="Header.stamp missing",
        )
    try:
        sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (TypeError, ValueError, AttributeError) as exc:
        return StampSample(
            modality=modality,
            topic=topic,
            stamp_sec=None,
            stamp_semantics=StampSemantics.PUBLICATION_HEADER,
            input_status=InputStatus.INVALID,
            evidence_class=EvidenceClass.INSUFFICIENT_DATA,
            detail=f"Header.stamp unreadable: {exc}",
        )
    if not math.isfinite(sec):
        return StampSample(
            modality=modality,
            topic=topic,
            stamp_sec=None,
            stamp_semantics=StampSemantics.PUBLICATION_HEADER,
            input_status=InputStatus.INVALID,
            evidence_class=EvidenceClass.INSUFFICIENT_DATA,
            detail="Header.stamp is NaN/Inf",
        )
    if sec < 0.0:
        return StampSample(
            modality=modality,
            topic=topic,
            stamp_sec=sec,
            stamp_semantics=StampSemantics.PUBLICATION_HEADER,
            input_status=InputStatus.INVALID,
            evidence_class=EvidenceClass.INSUFFICIENT_DATA,
            detail="Header.stamp is negative",
        )
    return StampSample(
        modality=modality,
        topic=topic,
        stamp_sec=sec,
        stamp_semantics=StampSemantics.PUBLICATION_HEADER,
        input_status=InputStatus.AVAILABLE,
        evidence_class=EvidenceClass.MODEL,
    )


def make_stamp(
    modality: str,
    stamp_sec: Optional[float],
    *,
    topic: str = "",
    semantics: StampSemantics = StampSemantics.PUBLICATION_HEADER,
    status: InputStatus | None = None,
) -> StampSample:
    if semantics == StampSemantics.NO_HEADER:
        return StampSample(
            modality=modality,
            topic=topic,
            stamp_sec=None,
            stamp_semantics=semantics,
            input_status=InputStatus.UNAVAILABLE,
            evidence_class=EvidenceClass.INSUFFICIENT_DATA,
            detail="no Header",
        )
    if stamp_sec is None:
        return StampSample(
            modality=modality,
            topic=topic,
            stamp_sec=None,
            stamp_semantics=semantics,
            input_status=status or InputStatus.MISSING,
            evidence_class=EvidenceClass.INSUFFICIENT_DATA,
        )
    if not math.isfinite(stamp_sec):
        return StampSample(
            modality=modality,
            topic=topic,
            stamp_sec=None,
            stamp_semantics=semantics,
            input_status=InputStatus.INVALID,
            evidence_class=EvidenceClass.INSUFFICIENT_DATA,
            detail="non-finite stamp",
        )
    return StampSample(
        modality=modality,
        topic=topic,
        stamp_sec=float(stamp_sec),
        stamp_semantics=semantics,
        input_status=status or InputStatus.AVAILABLE,
        evidence_class=EvidenceClass.MODEL,
    )


def apply_publication_delay(sample: StampSample, delay_s: float) -> StampSample:
    """Diagnostic copy only — never publish the delayed stamp to live topics."""
    if sample.stamp_sec is None or sample.input_status != InputStatus.AVAILABLE:
        raise GeometryDiagnosticsError(
            f"refuse delay injection on {sample.input_status.value} stamp"
        )
    return StampSample(
        modality=sample.modality,
        topic=sample.topic,
        stamp_sec=sample.stamp_sec + float(delay_s),
        stamp_semantics=sample.stamp_semantics,
        input_status=InputStatus.AVAILABLE,
        evidence_class=EvidenceClass.INJECTED_FAULT,
        detail=f"injected_publication_delay_s={delay_s}",
    )


def signed_delta(anchor: StampSample, other: StampSample) -> tuple[Optional[float], InputStatus, ResultSemantics]:
    """signed_delta_s = t_other - t_anchor. Fail-closed if either stamp is unusable."""
    if other.stamp_semantics == StampSemantics.NO_HEADER:
        return None, InputStatus.UNAVAILABLE, ResultSemantics.INSUFFICIENT_DATA
    if anchor.input_status != InputStatus.AVAILABLE or anchor.stamp_sec is None:
        return None, InputStatus.UNAVAILABLE, ResultSemantics.INSUFFICIENT_DATA
    if other.input_status == InputStatus.INVALID:
        return None, InputStatus.INVALID, ResultSemantics.ERROR_INPUT
    if other.input_status in (InputStatus.MISSING, InputStatus.UNAVAILABLE) or other.stamp_sec is None:
        return None, other.input_status, ResultSemantics.INSUFFICIENT_DATA
    delta = float(other.stamp_sec) - float(anchor.stamp_sec)
    return delta, InputStatus.AVAILABLE, ResultSemantics.REPORT_ONLY


def classify_sequence(previous_sec: Optional[float], current_sec: Optional[float]) -> SequenceFlag:
    if current_sec is None:
        return SequenceFlag.FIRST
    if previous_sec is None:
        return SequenceFlag.FIRST
    if current_sec == previous_sec:
        return SequenceFlag.DUPLICATE
    if current_sec < previous_sec:
        return SequenceFlag.OUT_OF_ORDER
    return SequenceFlag.IN_ORDER


def abs_slop_would_reject(abs_delta_s: float, slop_s: float = DEFAULT_RECORDER_SLOP_S) -> bool:
    """Mirrors MultiModalSync._stale_keys: reject iff abs(delta) > slop."""
    if slop_s <= 0.0:
        return False
    return float(abs_delta_s) > float(slop_s)


def motion_spatial_lag_m(speed_m_s: float, delay_s: float) -> float:
    """MODEL: latest-cache time error can become spatial lag ≈ |v| · |Δt|."""
    return abs(float(speed_m_s)) * abs(float(delay_s))


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    if not values:
        return None
    xs = sorted(float(v) for v in values)
    if len(xs) == 1:
        return xs[0]
    q = min(1.0, max(0.0, float(q)))
    idx = q * (len(xs) - 1)
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return xs[lo]
    w = idx - lo
    return xs[lo] * (1.0 - w) + xs[hi] * w


def summarize_signed(values: Sequence[float]) -> dict[str, Optional[float]]:
    xs = [float(v) for v in values]
    if not xs:
        return {"count": 0, "p50": None, "p95": None, "max": None, "min": None, "mean": None}
    return {
        "count": len(xs),
        "p50": percentile(xs, 0.50),
        "p95": percentile(xs, 0.95),
        "max": max(xs),
        "min": min(xs),
        "mean": sum(xs) / len(xs),
    }


def source_time_unavailable_row(scenario: str) -> SkewRow:
    return SkewRow(
        scenario=scenario,
        anchor_modality=ANCHOR_MODALITY,
        other_modality="*",
        signed_delta_s=None,
        abs_delta_s=None,
        injected_delay_s=0.0,
        recovered_delay_s=None,
        skew_class=SkewClass.SOURCE_TIME_SKEW.value,
        source_time_status=InputStatus.UNAVAILABLE.value,
        sequence_flag=SequenceFlag.FIRST.value,
        input_status=InputStatus.UNAVAILABLE.value,
        result_semantics=ResultSemantics.INSUFFICIENT_DATA.value,
        evidence_class=EvidenceClass.INSUFFICIENT_DATA.value,
        extra={
            "reason": (
                "no /clock, no physics sample ID, Header is publisher now(); "
                "SOURCE_TIME_SKEW is UNAVAILABLE"
            )
        },
    )


def skew_row_from_pair(
    *,
    scenario: str,
    anchor: StampSample,
    other: StampSample,
    injected_delay_s: float = 0.0,
    sequence_flag: SequenceFlag = SequenceFlag.FIRST,
    slop_s: float = DEFAULT_RECORDER_SLOP_S,
) -> SkewRow:
    delta, status, semantics = signed_delta(anchor, other)
    recovered = None
    if delta is not None and other.evidence_class == EvidenceClass.INJECTED_FAULT:
        recovered = delta
    extra: dict[str, Any] = {
        "anchor_stamp_sec": anchor.stamp_sec,
        "other_stamp_sec": other.stamp_sec,
        "anchor_semantics": anchor.stamp_semantics.value,
        "other_semantics": other.stamp_semantics.value,
        "recorder_slop_s": slop_s,
        "abs_slop_would_reject": (
            abs_slop_would_reject(abs(delta), slop_s) if delta is not None else None
        ),
        "stale_vs_slop": (
            InputStatus.STALE.value
            if delta is not None and abs_slop_would_reject(abs(delta), slop_s)
            else status.value
        ),
    }
    if sequence_flag in (SequenceFlag.DUPLICATE, SequenceFlag.OUT_OF_ORDER):
        semantics = ResultSemantics.ERROR_INPUT
        status = InputStatus.INVALID
    if status == InputStatus.STALE:
        # Stale is a report against the existing abs-slop gate, not a pass/fail.
        semantics = ResultSemantics.REPORT_ONLY
    return SkewRow(
        scenario=scenario,
        anchor_modality=anchor.modality,
        other_modality=other.modality,
        signed_delta_s=delta,
        abs_delta_s=None if delta is None else abs(delta),
        injected_delay_s=float(injected_delay_s),
        recovered_delay_s=recovered,
        skew_class=SkewClass.PUBLISH_TIME_SKEW.value,
        source_time_status=InputStatus.UNAVAILABLE.value,
        sequence_flag=sequence_flag.value,
        input_status=status.value,
        result_semantics=semantics.value,
        evidence_class=(
            EvidenceClass.INJECTED_FAULT.value
            if other.evidence_class == EvidenceClass.INJECTED_FAULT
            else EvidenceClass.MODEL.value
        ),
        extra=extra,
    )


def assert_no_pass_or_source_time(rows: Iterable[SkewRow]) -> None:
    forbidden = {
        "PASS",
        "FAIL",
        "CALIBRATED",
        "ROOT_CAUSE_CONFIRMED",
        "SOURCE_TIME_CONFIRMED",
        "ACQUISITION_SKEW",
    }
    allowed = {
        ResultSemantics.REPORT_ONLY.value,
        ResultSemantics.INSUFFICIENT_DATA.value,
        ResultSemantics.SUSPECTED.value,
        ResultSemantics.AMBIGUOUS.value,
        ResultSemantics.ERROR_INPUT.value,
    }
    for row in rows:
        if row.result_semantics in forbidden:
            raise ValueError(f"illegal result_semantics: {row.result_semantics}")
        if row.result_semantics not in allowed:
            raise ValueError(f"unknown result_semantics: {row.result_semantics}")
        if (
            row.skew_class == SkewClass.SOURCE_TIME_SKEW.value
            and row.source_time_status != InputStatus.UNAVAILABLE.value
        ):
            raise ValueError("Stage 4 v1 must not claim SOURCE_TIME_SKEW available")
        if row.physical != "NOT_RUN/UNAVAILABLE":
            raise ValueError(f"illegal physical claim: {row.physical}")
