"""Unit tests for ContinuousTaskEvaluator and episode_results.jsonl writer."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from synth_data_gen.continuous_evaluator import (
    EVALUATOR_ID,
    OWNER_REPOSITORY,
    ContinuousTaskEvaluator,
    EvaluatorSample,
    append_episode_result,
)


def _trace_success_place(ev: ContinuousTaskEvaluator) -> None:
    """Synthetic continuous pick → lift → place → release."""
    obj0 = (0.40, -0.05, 0.025)
    ev.reset(initial_object_xyz=obj0, bin_xy=(0.40, -0.35), reset_monotonic_s=1.0)
    t = 1.0
    # Approach / reach
    for i in range(5):
        t += 0.1
        ev.observe(
            EvaluatorSample(
                t_monotonic=t,
                object_xyz=obj0,
                ee_xyz=(0.40, -0.05 + 0.01, 0.12),
                gripper=1.0,
                phase_hint="approach_xy",
            )
        )
    # Descend + grasp
    for i in range(5):
        t += 0.1
        z = 0.12 - i * 0.015
        ev.observe(
            EvaluatorSample(
                t_monotonic=t,
                object_xyz=obj0,
                ee_xyz=(0.40, -0.05, z),
                gripper=0.0 if i >= 2 else 1.0,
                phase_hint="close",
            )
        )
    # Continuous lift while held
    for i in range(8):
        t += 0.1
        oz = 0.025 + i * 0.012
        ev.observe(
            EvaluatorSample(
                t_monotonic=t,
                object_xyz=(0.40, -0.05, oz),
                ee_xyz=(0.40, -0.05, oz + 0.02),
                gripper=0.0,
                ee_cmd_xyz=(0.40, -0.05, oz + 0.02),
                phase_hint="lift",
            )
        )
    # Transport toward bin
    for i in range(6):
        t += 0.1
        y = -0.05 + i * (-0.05)
        ev.observe(
            EvaluatorSample(
                t_monotonic=t,
                object_xyz=(0.40, y, 0.12),
                ee_xyz=(0.40, y, 0.14),
                gripper=0.0,
                phase_hint="transport",
            )
        )
    # Place + release
    t += 0.1
    ev.observe(
        EvaluatorSample(
            t_monotonic=t,
            object_xyz=(0.40, -0.35, 0.05),
            ee_xyz=(0.40, -0.35, 0.08),
            gripper=0.0,
            phase_hint="place",
        )
    )
    t += 0.1
    ev.observe(
        EvaluatorSample(
            t_monotonic=t,
            object_xyz=(0.40, -0.35, 0.05),
            ee_xyz=(0.40, -0.35, 0.08),
            gripper=1.0,
            phase_hint="release",
        )
    )


def _evidence(tmp_path: Path, idx: int = 0) -> dict:
    runtime = tmp_path / f"runtime_{idx}.log"
    events = tmp_path / f"events_{idx}.jsonl"
    nfr = tmp_path / f"nfr_{idx}.json"
    runtime.write_text("ok\n", encoding="utf-8")
    events.write_text("", encoding="utf-8")
    nfr.write_text("{}\n", encoding="utf-8")
    return {
        "raw_episode_path": str(tmp_path / f"raw_{idx}"),
        "video_path": None,
        "runtime_log_path": str(runtime),
        "event_log_path": str(events),
        "nfr_sample_path": str(nfr),
    }


def test_continuous_lift_requires_held_peak_not_endpoint_only():
    ev = ContinuousTaskEvaluator(lift_success_delta=0.03, validation_mode="lift")
    obj0 = (0.35, 0.0, 0.025)
    ev.reset(initial_object_xyz=obj0, bin_xy=(0.4, -0.35), reset_monotonic_s=0.0)
    # Gripper never closes; object z rises somehow — must not count as lift.
    for i in range(5):
        ev.observe(
            EvaluatorSample(
                t_monotonic=float(i),
                object_xyz=(0.35, 0.0, 0.025 + i * 0.02),
                ee_xyz=(0.50, 0.0, 0.40),
                gripper=1.0,
            )
        )
    ok, reason = ev.evaluate_success()
    assert ok is False
    assert "gripper" in reason or "lift" in reason


def test_continuous_place_success_and_jsonl_schema_shape(tmp_path: Path):
    ev = ContinuousTaskEvaluator(
        lift_success_delta=0.03, bin_xy_tolerance=0.08, validation_mode="place"
    )
    _trace_success_place(ev)
    ok, reason = ev.evaluate_success()
    assert ok is True, reason
    assert ev._lift is True
    assert ev._place is True
    assert ev._release is True

    row = ev.finalize(
        evaluation_run_id="unit_test_run",
        identity={
            "model_id": "unit_expert",
            "backend": "mujoco",
            "scene_id": "panda_pick_place_v1",
            "suite_id": "nominal",
            "seed": 7,
            "episode_index": 0,
        },
        evidence=_evidence(tmp_path),
        execution_status="completed",
    )
    assert row["contract_version"] == "evaluation_contract_v0"
    assert row["artifact_type"] == "episode_result"
    assert row["execution_status"] == "completed"
    assert row["evidence_level"] == "runtime_observed"
    assert row["outcome"]["runtime_evaluated"] is True
    assert row["outcome"]["success"] is True
    assert row["outcome"]["evaluator"]["owner_repository"] == OWNER_REPOSITORY
    assert row["outcome"]["evaluator"]["evaluator_id"] == EVALUATOR_ID
    assert row["outcome"]["evaluator"]["ground_truth_source"] == "runtime_ground_truth"
    assert row["subgoals"]["lift"] is True
    assert row["subgoals"]["place"] is True
    assert row["motion"]["path_length_m"] is not None
    assert row["motion"]["path_length_m"] > 0.0

    out = tmp_path / "episode_results.jsonl"
    append_episode_result(str(out), row)
    append_episode_result(str(out), row)
    lines = [ln for ln in out.read_text(encoding="utf-8").splitlines() if ln.strip()]
    assert len(lines) == 2
    loaded = json.loads(lines[0])
    assert loaded["identity"]["seed"] == 7


def test_drop_after_lift_is_detected():
    ev = ContinuousTaskEvaluator(lift_success_delta=0.03, validation_mode="lift")
    obj0 = (0.35, 0.0, 0.025)
    ev.reset(initial_object_xyz=obj0, bin_xy=(0.4, -0.35), reset_monotonic_s=0.0)
    # Grasp + lift
    for i in range(6):
        oz = 0.025 + i * 0.015
        ev.observe(
            EvaluatorSample(
                t_monotonic=float(i),
                object_xyz=(0.35, 0.0, oz),
                ee_xyz=(0.35, 0.0, oz + 0.02),
                gripper=0.0,
            )
        )
    assert ev._lift is True
    # Open gripper and object falls back to table
    ev.observe(
        EvaluatorSample(
            t_monotonic=10.0,
            object_xyz=(0.35, 0.0, 0.028),
            ee_xyz=(0.35, 0.0, 0.20),
            gripper=1.0,
        )
    )
    ok, reason = ev.evaluate_success()
    assert ok is False
    assert "drop" in reason


def test_transport_does_not_count_as_slip_but_relative_drift_does():
    ev = ContinuousTaskEvaluator(
        lift_success_delta=0.03, slip_xy_tolerance=0.03, validation_mode="lift"
    )
    obj0 = (0.35, 0.0, 0.025)
    ev.reset(initial_object_xyz=obj0, bin_xy=(0.4, -0.35), reset_monotonic_s=0.0)
    for i in range(6):
        oz = 0.025 + i * 0.015
        ev.observe(
            EvaluatorSample(
                t_monotonic=float(i),
                object_xyz=(0.35, 0.0, oz),
                ee_xyz=(0.35, 0.0, oz + 0.02),
                gripper=0.0,
            )
        )
    assert ev._lift is True
    # Intentional transport: object and EE move together → not slip.
    for i in range(5):
        y = -0.02 * i
        ev.observe(
            EvaluatorSample(
                t_monotonic=10.0 + i,
                object_xyz=(0.35, y, 0.10),
                ee_xyz=(0.35, y, 0.12),
                gripper=0.0,
            )
        )
    assert ev._slip_detected is False
    # Relative hold drift while still "held" near EE → slip.
    ev.observe(
        EvaluatorSample(
            t_monotonic=20.0,
            object_xyz=(0.35, -0.12, 0.10),
            ee_xyz=(0.35, -0.08, 0.12),
            gripper=0.0,
        )
    )
    assert ev._slip_detected is True
    ok, reason = ev.evaluate_success()
    assert ok is False
    assert "slip" in reason


def test_episode_result_validates_against_midstream_schema_when_available(tmp_path: Path):
    schema_path = Path(
        "/home/ina/robot-sim-lab/robot-arm-episode-data-lab/evaluation/schemas/"
        "episode_result.schema.json"
    )
    if not schema_path.exists():
        pytest.skip("midstream episode_result schema not present")
    jsonschema = pytest.importorskip("jsonschema")
    from jsonschema import Draft202012Validator, FormatChecker

    ev = ContinuousTaskEvaluator(validation_mode="place")
    _trace_success_place(ev)
    row = ev.finalize(
        evaluation_run_id="unit_schema_check",
        identity={
            "model_id": "unit_expert",
            "backend": "mujoco",
            "scene_id": "panda_pick_place_v1",
            "suite_id": "nominal",
            "seed": 11,
            "episode_index": 0,
        },
        evidence=_evidence(tmp_path, idx=1),
    )
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    Draft202012Validator(schema, format_checker=FormatChecker()).validate(row)
