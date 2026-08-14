# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage 4: publication-time stamp skew evidence (REPORT_ONLY).

Does not change MultiModalSync slop gate, recorder schema, or control topics.
SOURCE_TIME_SKEW is recorded as UNAVAILABLE.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from teleop_diagnostics.report import (
    provenance_fields,
    write_run_manifest,
    write_timestamp_skew_csv,
)
from teleop_diagnostics.timestamps import (
    ANCHOR_MODALITY,
    CONTROLLED_DELAYS_S,
    DEFAULT_RECORDER_SLOP_S,
    PAIRED_MODALITIES,
    SequenceFlag,
    StampSemantics,
    apply_publication_delay,
    assert_no_pass_or_source_time,
    classify_sequence,
    header_stamp_sec,
    make_stamp,
    motion_spatial_lag_m,
    skew_row_from_pair,
    source_time_unavailable_row,
    summarize_signed,
)
from teleop_diagnostics.types import InputStatus


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _aligned_frame(t0: float) -> dict:
    return {
        ANCHOR_MODALITY: make_stamp(ANCHOR_MODALITY, t0, topic="/camera/color/image_raw"),
        "joint": make_stamp("joint", t0, topic="/joint_states"),
        "ee": make_stamp("ee", t0, topic="/ee_pose"),
        "object": make_stamp("object", t0, topic="/sim/object_pose"),
        "ft": make_stamp("ft", t0, topic="/ft_sensor"),
        "wrist": make_stamp("wrist", t0, topic="/camera/wrist/color/image_raw"),
    }


def run_stage4(out_dir: Path, *, slop_s: float = DEFAULT_RECORDER_SLOP_S) -> dict:
    repo = _repo_root()
    prov = provenance_fields(repo=repo)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []

    t0 = 1000.0
    aligned = _aligned_frame(t0)

    # --- zero delay: all signed deltas 0 ---
    for mod in PAIRED_MODALITIES:
        rows.append(
            skew_row_from_pair(
                scenario="zero_delay_aligned",
                anchor=aligned[ANCHOR_MODALITY],
                other=aligned[mod],
                injected_delay_s=0.0,
                slop_s=slop_s,
            )
        )

    # --- controlled publication delays on diagnostic copies ---
    recovered_ok = True
    for delay in CONTROLLED_DELAYS_S:
        if delay == 0.0:
            continue
        for mod in ("joint", "ee", "object"):
            delayed = apply_publication_delay(aligned[mod], delay)
            row = skew_row_from_pair(
                scenario=f"inject_{mod}_{delay:+.3f}s",
                anchor=aligned[ANCHOR_MODALITY],
                other=delayed,
                injected_delay_s=delay,
                slop_s=slop_s,
            )
            rows.append(row)
            if row.signed_delta_s is None or abs(row.signed_delta_s - delay) > 1e-12:
                recovered_ok = False

    # --- missing / invalid / no-header ---
    missing = make_stamp("joint", None, topic="/joint_states")
    rows.append(
        skew_row_from_pair(
            scenario="missing_joint_stamp",
            anchor=aligned[ANCHOR_MODALITY],
            other=missing,
        )
    )
    invalid = make_stamp("ee", float("nan"), topic="/ee_pose")
    rows.append(
        skew_row_from_pair(
            scenario="invalid_nan_ee_stamp",
            anchor=aligned[ANCHOR_MODALITY],
            other=invalid,
        )
    )
    gripper = make_stamp(
        "gripper_state",
        None,
        topic="/gripper/state",
        semantics=StampSemantics.NO_HEADER,
    )
    rows.append(
        skew_row_from_pair(
            scenario="gripper_float64_no_header",
            anchor=aligned[ANCHOR_MODALITY],
            other=gripper,
        )
    )
    grip_cmd = make_stamp(
        "gripper_cmd",
        None,
        topic="/teleop/gripper_cmd",
        semantics=StampSemantics.NO_HEADER,
    )
    rows.append(
        skew_row_from_pair(
            scenario="gripper_cmd_float64_no_header",
            anchor=aligned[ANCHOR_MODALITY],
            other=grip_cmd,
        )
    )

    # --- sequence: duplicate and out-of-order on diagnostic copies ---
    seq = [t0, t0 + 0.010, t0 + 0.010, t0 + 0.005]
    prev = None
    flags = []
    for i, ts in enumerate(seq):
        flag = classify_sequence(prev, ts)
        flags.append(flag.value)
        sample = make_stamp("color", ts, topic="/camera/color/image_raw")
        rows.append(
            skew_row_from_pair(
                scenario=f"color_sequence_{i}",
                anchor=aligned["joint"],
                other=sample,
                sequence_flag=flag,
            )
        )
        prev = ts

    # --- SOURCE_TIME always UNAVAILABLE ---
    rows.append(source_time_unavailable_row("source_time_contract"))

    # --- motion sensitivity MODEL: Δx ≈ |v|·|Δt| for latest-cache ---
    static_speed = 0.0
    fast_speed = 0.50  # m/s, synthetic EE speed
    for delay in (0.010, 0.030, 0.050, 0.100):
        for speed, motion in ((static_speed, "static"), (fast_speed, "fast")):
            delayed = apply_publication_delay(aligned["ee"], delay)
            row = skew_row_from_pair(
                scenario=f"motion_{motion}_ee_{delay:+.3f}s",
                anchor=aligned[ANCHOR_MODALITY],
                other=delayed,
                injected_delay_s=delay,
                slop_s=slop_s,
            )
            lag = motion_spatial_lag_m(speed, delay)
            row.extra["speed_m_s"] = speed
            row.extra["spatial_lag_m"] = lag
            row.extra["motion_note"] = (
                "MODEL latest-cache spatialization; not a live high-speed measurement"
            )
            rows.append(row)

    assert_no_pass_or_source_time(rows)

    # Pair summaries (zero-delay + injections for joint/ee/object)
    pair_summaries = {}
    for mod in ("joint", "ee", "object"):
        vals = [
            r.signed_delta_s
            for r in rows
            if r.other_modality == mod
            and r.signed_delta_s is not None
            and r.scenario.startswith("inject_")
        ]
        pair_summaries[f"image_{mod}"] = summarize_signed(vals)

    # MultiModalSync gate still uses abs(delta) — demonstrated, not modified.
    from types import SimpleNamespace

    from geometry_msgs.msg import PoseStamped, WrenchStamped
    from sensor_msgs.msg import Image, JointState

    from lerobot_recorder.time_sync import MultiModalSync

    class _Clock:
        def now(self):
            return SimpleNamespace(nanoseconds=int(t0 * 1e9))

    class _Node:
        def create_subscription(self, *_a, **_k):
            return object()

        def get_clock(self):
            return _Clock()

        def get_logger(self):
            return SimpleNamespace(warn=lambda *_a, **_k: None)

    emitted = []
    sync = MultiModalSync(_Node(), lambda *m: emitted.append(m), slop=slop_s, visual_keys=("color",))

    def _stamp(msg, sec: float):
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(round((sec - int(sec)) * 1e9))
        return msg

    def _feed(delay_joint: float) -> None:
        emitted.clear()
        sync.latest.clear()
        sync.last_emitted_stamp.clear()
        sync.reject_counts = {"missing": 0, "stale": 0, "reused": 0}
        sync._update("joint", _stamp(JointState(), t0 + delay_joint))
        sync._update("ee", _stamp(PoseStamped(), t0))
        sync._update("ft", _stamp(WrenchStamped(), t0))
        sync._update("object", _stamp(PoseStamped(), t0))
        sync._update("color", _stamp(Image(), t0))

    _feed(0.0)
    emit_zero = len(emitted) == 1
    skew_zero = sync.signed_publication_skews()
    _feed(0.030)
    emit_30 = len(emitted) == 1
    _feed(-0.030)
    emit_m30 = len(emitted) == 1
    _feed(0.100)
    emit_100 = len(emitted) == 0 and sync.reject_counts["stale"] == 1
    _feed(-0.100)
    emit_m100 = len(emitted) == 0 and sync.reject_counts["stale"] == 1

    gate_abs_symmetric = emit_30 and emit_m30 and emit_100 and emit_m100

    dict_rows = []
    for r in rows:
        d = r.to_dict()
        d["evidence_generation_commit"] = prov["evidence_generation_commit"]
        d["abs_slop_would_reject"] = d.get("abs_slop_would_reject", "")
        d["stale_vs_slop"] = d.get("stale_vs_slop", "")
        d["spatial_lag_m"] = d.get("spatial_lag_m", "")
        d["speed_m_s"] = d.get("speed_m_s", "")
        dict_rows.append(d)

    write_timestamp_skew_csv(out_dir / "timestamp_skew.csv", dict_rows)

    unavailable_never_pass = all(
        r.result_semantics != "PASS"
        and not (
            r.input_status in ("MISSING", "UNAVAILABLE", "INVALID")
            and r.result_semantics == "REPORT_ONLY"
            and r.signed_delta_s is not None
        )
        for r in rows
    )
    # Missing/unavailable rows must be INSUFFICIENT_DATA or ERROR_INPUT
    bad_open = [
        r.scenario
        for r in rows
        if r.input_status in (
            InputStatus.MISSING.value,
            InputStatus.UNAVAILABLE.value,
            InputStatus.INVALID.value,
        )
        and r.result_semantics == "REPORT_ONLY"
        and r.signed_delta_s is not None
    ]

    diagnostics = {
        "skew_class_v1": "PUBLISH_TIME_SKEW",
        "SOURCE_TIME_SKEW": "UNAVAILABLE",
        "source_time_skew": "UNAVAILABLE",
        "source_time_reason": (
            "No /clock, no physics sample ID; Header is publisher now(). "
            "Gripper state/cmd are Float64 without Header."
        ),
        "signed_convention": "signed_delta_s = t_modality - t_anchor(scene color)",
        "recorder_gate": {
            "unchanged": True,
            "uses": "abs(delta) > sync_slop",
            "default_slop_s": slop_s,
            "abs_symmetric_plus_minus_delay": gate_abs_symmetric,
            "emit_zero_delay": emit_zero,
            "emit_plus_30ms": emit_30,
            "emit_minus_30ms": emit_m30,
            "reject_plus_100ms_stale": emit_100,
            "reject_minus_100ms_stale": emit_m100,
            "observer_signed_skew_zero_delay": skew_zero,
        },
        "controlled_delay_recovered": recovered_ok,
        "sequence_flags_demo": flags,
        "pair_summaries_injected": pair_summaries,
        "motion_model": {
            "formula": "spatial_lag_m ≈ |v| · |Δt|",
            "fast_speed_m_s": fast_speed,
            "example_50ms_fast_lag_m": motion_spatial_lag_m(fast_speed, 0.050),
            "example_50ms_static_lag_m": motion_spatial_lag_m(0.0, 0.050),
            "evidence_class": "MODEL",
        },
        "unavailable_never_pass": unavailable_never_pass and not bad_open,
        "physical": "NOT_RUN/UNAVAILABLE",
        "result_semantics": "REPORT_ONLY",
        "header_stamp_helper_used": header_stamp_sec.__name__,
        **prov,
    }
    (out_dir / "timestamp_diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    exit_gate = {
        "skew_semantics": "PUBLISH_TIME_SKEW only; SOURCE_TIME_SKEW=UNAVAILABLE",
        "controlled_delay_recovered": recovered_ok,
        "recorder_slop_gate_unchanged": True,
        "abs_slop_is_sign_symmetric": gate_abs_symmetric,
        "missing_invalid_unavailable_not_pass": unavailable_never_pass and not bad_open,
        "physical": "NOT_RUN/UNAVAILABLE",
        "result_semantics": "REPORT_ONLY",
    }
    manifest = {
        "stage": "4",
        "physical": "NOT_RUN/UNAVAILABLE",
        "result_semantics": "REPORT_ONLY",
        "control_law_modified": False,
        "recorder_gate_modified": False,
        "runtime_topics_mutated": False,
        "exit_gate_observations": exit_gate,
        "artifacts": {
            "run_manifest": "run_manifest.json",
            "timestamp_skew": "timestamp_skew.csv",
            "timestamp_diagnostics": "timestamp_diagnostics.json",
        },
        **prov,
    }
    write_run_manifest(out_dir / "run_manifest.json", manifest)
    return manifest


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--out-dir",
        type=Path,
        default=Path("evidence/timestamp_stage4"),
    )
    p.add_argument("--slop-s", type=float, default=DEFAULT_RECORDER_SLOP_S)
    args = p.parse_args(argv)
    print(json.dumps(run_stage4(args.out_dir, slop_s=args.slop_s), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
