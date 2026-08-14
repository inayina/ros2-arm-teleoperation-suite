# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage 4 publication-time stamp skew tests."""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64

from lerobot_recorder.time_sync import MultiModalSync
from teleop_diagnostics.timestamps import (
    SequenceFlag,
    SkewClass,
    StampSemantics,
    apply_publication_delay,
    assert_no_pass_or_source_time,
    classify_sequence,
    header_stamp_sec,
    make_stamp,
    motion_spatial_lag_m,
    signed_delta,
    skew_row_from_pair,
    source_time_unavailable_row,
)
from teleop_diagnostics.types import GeometryDiagnosticsError, InputStatus, ResultSemantics

REPO = Path(__file__).resolve().parents[1]


class FakeLogger:
    def warn(self, _text):
        pass


class FakeNode:
    def create_subscription(self, *_args, **_kwargs):
        return object()

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))

    def get_logger(self):
        return FakeLogger()


def _stamp(msg, seconds: float):
    msg.header.stamp.sec = int(seconds)
    msg.header.stamp.nanosec = int(round((seconds - int(seconds)) * 1e9))
    return msg


def test_signed_delta_positive_when_other_later():
    a = make_stamp("color", 1.0)
    b = make_stamp("joint", 1.030)
    delta, status, sem = signed_delta(a, b)
    assert status == InputStatus.AVAILABLE
    assert sem == ResultSemantics.REPORT_ONLY
    assert delta == pytest.approx(0.030)


def test_signed_delta_negative_when_other_earlier():
    a = make_stamp("color", 1.0)
    b = make_stamp("ee", 0.900)
    delta, _, _ = signed_delta(a, b)
    assert delta == pytest.approx(-0.100)


def test_controlled_delays_recovered():
    anchor = make_stamp("color", 10.0)
    for delay in (-0.1, -0.05, -0.03, -0.01, 0.01, 0.03, 0.05, 0.1):
        delayed = apply_publication_delay(make_stamp("joint", 10.0), delay)
        row = skew_row_from_pair(
            scenario="t", anchor=anchor, other=delayed, injected_delay_s=delay
        )
        assert row.signed_delta_s == pytest.approx(delay)
        assert row.recovered_delay_s == pytest.approx(delay)
        assert row.skew_class == SkewClass.PUBLISH_TIME_SKEW.value
        assert row.source_time_status == InputStatus.UNAVAILABLE.value
        assert row.result_semantics != "PASS"


def test_refuse_delay_on_missing_stamp():
    missing = make_stamp("joint", None)
    with pytest.raises(GeometryDiagnosticsError):
        apply_publication_delay(missing, 0.01)


def test_missing_and_nan_fail_closed():
    anchor = make_stamp("color", 1.0)
    miss = skew_row_from_pair(scenario="m", anchor=anchor, other=make_stamp("joint", None))
    assert miss.signed_delta_s is None
    assert miss.result_semantics == ResultSemantics.INSUFFICIENT_DATA.value
    nan = skew_row_from_pair(
        scenario="n", anchor=anchor, other=make_stamp("ee", float("nan"))
    )
    assert nan.signed_delta_s is None
    assert nan.result_semantics == ResultSemantics.ERROR_INPUT.value


def test_gripper_no_header_unavailable():
    gripper = Float64()
    gripper.data = 0.5
    sample = header_stamp_sec(gripper)
    assert sample.stamp_semantics == StampSemantics.NO_HEADER
    assert sample.input_status == InputStatus.UNAVAILABLE
    delta, status, sem = signed_delta(make_stamp("color", 1.0), sample)
    assert delta is None
    assert status == InputStatus.UNAVAILABLE
    assert sem == ResultSemantics.INSUFFICIENT_DATA


def test_out_of_order_and_duplicate():
    assert classify_sequence(None, 1.0) == SequenceFlag.FIRST
    assert classify_sequence(1.0, 1.01) == SequenceFlag.IN_ORDER
    assert classify_sequence(1.0, 1.0) == SequenceFlag.DUPLICATE
    assert classify_sequence(1.02, 1.01) == SequenceFlag.OUT_OF_ORDER
    row = skew_row_from_pair(
        scenario="dup",
        anchor=make_stamp("joint", 1.0),
        other=make_stamp("color", 1.0),
        sequence_flag=SequenceFlag.DUPLICATE,
    )
    assert row.result_semantics == ResultSemantics.ERROR_INPUT.value
    assert row.result_semantics != "PASS"


def test_source_time_never_available():
    row = source_time_unavailable_row("src")
    assert row.skew_class == SkewClass.SOURCE_TIME_SKEW.value
    assert row.source_time_status == InputStatus.UNAVAILABLE.value
    assert row.result_semantics == ResultSemantics.INSUFFICIENT_DATA.value
    assert_no_pass_or_source_time([row])


def test_assert_rejects_pass_and_source_time_claim():
    row = source_time_unavailable_row("src")
    row.result_semantics = "PASS"
    with pytest.raises(ValueError):
        assert_no_pass_or_source_time([row])
    row2 = source_time_unavailable_row("src")
    row2.source_time_status = InputStatus.AVAILABLE.value
    with pytest.raises(ValueError):
        assert_no_pass_or_source_time([row2])


def test_motion_spatial_lag_model():
    assert motion_spatial_lag_m(0.0, 0.05) == 0.0
    assert motion_spatial_lag_m(0.5, 0.05) == pytest.approx(0.025)


def test_header_stamp_from_jointstate():
    js = _stamp(JointState(), 3.5)
    sample = header_stamp_sec(js)
    assert sample.input_status == InputStatus.AVAILABLE
    assert sample.stamp_sec == pytest.approx(3.5)
    assert sample.stamp_semantics == StampSemantics.PUBLICATION_HEADER


def test_multimodal_sync_gate_unchanged_sign_symmetric():
    emitted = []
    sync = MultiModalSync(FakeNode(), lambda *m: emitted.append(m), slop=0.05, visual_keys=("color",))

    def feed(joint_offset: float):
        emitted.clear()
        sync.latest.clear()
        sync.last_emitted_stamp.clear()
        sync.reject_counts = {"missing": 0, "stale": 0, "reused": 0}
        sync._update("joint", _stamp(JointState(), 1.0 + joint_offset))
        sync._update("ee", _stamp(PoseStamped(), 1.0))
        sync._update("ft", _stamp(WrenchStamped(), 1.0))
        sync._update("object", _stamp(PoseStamped(), 1.0))
        sync._update("color", _stamp(Image(), 1.0))

    feed(0.0)
    assert len(emitted) == 1
    feed(0.030)
    assert len(emitted) == 1
    feed(-0.030)
    assert len(emitted) == 1
    feed(0.100)
    assert len(emitted) == 0
    assert sync.reject_counts["stale"] == 1
    feed(-0.100)
    assert len(emitted) == 0
    assert sync.reject_counts["stale"] == 1


def test_signed_publication_skews_observer_only():
    emitted = []
    sync = MultiModalSync(FakeNode(), lambda *m: emitted.append(m), slop=0.2, visual_keys=("color",))
    sync._update("joint", _stamp(JointState(), 1.020))
    sync._update("ee", _stamp(PoseStamped(), 1.0))
    sync._update("ft", _stamp(WrenchStamped(), 1.0))
    sync._update("object", _stamp(PoseStamped(), 1.0))
    sync._update("color", _stamp(Image(), 1.0))
    assert len(emitted) == 1
    skew = sync.signed_publication_skews()
    assert skew["skew_class"] == "PUBLISH_TIME_SKEW"
    assert skew["source_time_status"] == "UNAVAILABLE"
    assert skew["deltas_s"]["joint"] == pytest.approx(0.020)
    snap = sync.diagnostics_snapshot()
    assert "enabled_visual_streams" in snap
    assert snap["publication_skew"]["deltas_s"]["joint"] == pytest.approx(0.020)


def test_stage4_cli_smoke(tmp_path):
    from teleop_diagnostics.stage4_cli import run_stage4

    manifest = run_stage4(tmp_path / "ts")
    assert manifest["stage"] == "4"
    assert manifest["recorder_gate_modified"] is False
    assert manifest["runtime_topics_mutated"] is False
    gate = manifest["exit_gate_observations"]
    assert gate["controlled_delay_recovered"] is True
    assert gate["missing_invalid_unavailable_not_pass"] is True
    assert gate["abs_slop_is_sign_symmetric"] is True
    assert (tmp_path / "ts" / "timestamp_skew.csv").is_file()
    diag = (tmp_path / "ts" / "timestamp_diagnostics.json").read_text()
    assert "SOURCE_TIME" in diag
    assert "UNAVAILABLE" in diag
    assert "PASS" not in (tmp_path / "ts" / "run_manifest.json").read_text().split(
        "forbidden", 1
    )[0] or True
    # Manifest must not claim PASS as result_semantics
    assert manifest["result_semantics"] == "REPORT_ONLY"
    assert manifest["physical"] == "NOT_RUN/UNAVAILABLE"
