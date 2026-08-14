# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Evidence serialization for geometry / camera diagnostics."""

from __future__ import annotations

import csv
import json
import subprocess
from pathlib import Path
from typing import Any, Iterable, Sequence

from teleop_diagnostics.types import ResidualRow, ResultSemantics

# Audit baseline: pre–Stage-1/2 geometry diagnostics snapshot (docs/GEOMETRY_TIMING…).
AUDIT_BASELINE_COMMIT = "f3a760774d02aabf6a6bdd2993a53e1738b867b5"
# First commit that landed Stage 1/2 diagnostics implementation.
STAGE12_IMPLEMENTATION_COMMIT = "a131e180a77709f60d8b3a2bfb1a8cb0762b64e0"
# Stage 3 camera extrinsic contract.
STAGE3_IMPLEMENTATION_COMMIT = "4f71a494988fbe59b280cc3b99c2e4502eb52556"


def git_commit(repo: Path | None = None) -> str:
    try:
        proc = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(repo) if repo else None,
            check=False,
            capture_output=True,
            text=True,
        )
        if proc.returncode == 0:
            return proc.stdout.strip()
    except OSError:
        pass
    return "UNKNOWN"


def git_working_tree_dirty(repo: Path | None = None) -> bool:
    try:
        proc = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=str(repo) if repo else None,
            check=False,
            capture_output=True,
            text=True,
        )
        return proc.returncode == 0 and bool(proc.stdout.strip())
    except OSError:
        return False


def provenance_fields(
    *,
    repo: Path | None = None,
    audit_baseline_commit: str = AUDIT_BASELINE_COMMIT,
    implementation_commit: str | None = None,
) -> dict[str, Any]:
    """Distinct commit semantics to avoid Stage-2-style provenance ambiguity."""
    head = git_commit(repo)
    dirty = git_working_tree_dirty(repo)
    return {
        "audit_baseline_commit": audit_baseline_commit,
        "stage12_implementation_commit": STAGE12_IMPLEMENTATION_COMMIT,
        "stage3_implementation_commit": STAGE3_IMPLEMENTATION_COMMIT,
        "implementation_commit": implementation_commit or head,
        "evidence_generation_commit": head,
        "evidence_working_tree_dirty": dirty,
        "evidence_generation_note": (
            f"HEAD={head} with uncommitted working tree; SHA is not the dirty tree"
            if dirty
            else f"HEAD={head} clean"
        ),
    }


def write_run_manifest(path: Path, manifest: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_geometry_diagnostics_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


_CSV_FIELDS = [
    "commit",
    "evidence_generation_commit",
    "backend",
    "scenario",
    "q",
    "source_a",
    "source_b",
    "frame_from",
    "frame_to",
    "reference_point",
    "translation_error_m",
    "rotation_error_rad",
    "evidence_class_a",
    "evidence_class_b",
    "input_status",
    "result_semantics",
    "physical",
]


def write_geometry_samples_csv(path: Path, rows: Sequence[ResidualRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=_CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            d = row.to_dict()
            writer.writerow(
                {
                    "commit": d.get("evidence_generation_commit", d.get("commit", "")),
                    "evidence_generation_commit": d.get(
                        "evidence_generation_commit", d.get("commit", "")
                    ),
                    "backend": d.get("backend", ""),
                    "scenario": d.get("scenario", ""),
                    "q": json.dumps(d.get("q", [])),
                    "source_a": d.get("source_a", ""),
                    "source_b": d.get("source_b", ""),
                    "frame_from": d.get("frame_from", ""),
                    "frame_to": d.get("frame_to", ""),
                    "reference_point": d.get("reference_point", ""),
                    "translation_error_m": d.get("translation_error_m", ""),
                    "rotation_error_rad": d.get("rotation_error_rad", ""),
                    "evidence_class_a": d.get("evidence_class_a", ""),
                    "evidence_class_b": d.get("evidence_class_b", ""),
                    "input_status": d.get("input_status", ""),
                    "result_semantics": d.get(
                        "result_semantics", ResultSemantics.REPORT_ONLY.value
                    ),
                    "physical": d.get("physical", "NOT_RUN/UNAVAILABLE"),
                }
            )


def assert_no_pass_semantics(rows: Iterable[ResidualRow]) -> None:
    allowed = {
        ResultSemantics.REPORT_ONLY.value,
        ResultSemantics.INSUFFICIENT_DATA.value,
        ResultSemantics.SUSPECTED.value,
        ResultSemantics.AMBIGUOUS.value,
        ResultSemantics.ERROR_INPUT.value,
    }
    forbidden = {"PASS", "FAIL", "CALIBRATED", "ROOT_CAUSE_CONFIRMED"}
    for row in rows:
        if row.result_semantics in forbidden:
            raise ValueError(f"illegal result_semantics: {row.result_semantics}")
        if row.result_semantics not in allowed:
            raise ValueError(f"unknown result_semantics: {row.result_semantics}")


_FAULT_FIELDS = [
    "scenario",
    "fault_type",
    "joint_name",
    "offset_rad",
    "offset_deg",
    "tcp_dx_m",
    "tcp_dy_m",
    "tcp_dz_m",
    "tcp_droll_rad",
    "tcp_dpitch_rad",
    "tcp_dyaw_rad",
    "pose_name",
    "translation_error_m",
    "rotation_error_rad",
    "tool_local_error_m",
    "tool_local_xyz_m",
    "status",
    "suspected_cause",
    "evidence_class",
    "result_semantics",
    "physical",
    "commit",
    "evidence_generation_commit",
]


def write_fault_matrix_csv(path: Path, rows: Sequence[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=_FAULT_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            out = {k: row.get(k, "") for k in _FAULT_FIELDS}
            if not out.get("evidence_generation_commit"):
                out["evidence_generation_commit"] = out.get("commit", "")
            writer.writerow(out)


_CAMERA_SAMPLE_FIELDS = [
    "camera_name",
    "camera_type",
    "parent_frame",
    "optical_frame",
    "nominal_translation",
    "nominal_rotation",
    "injected_translation",
    "injected_rotation",
    "effective_translation",
    "effective_rotation",
    "renderer_translation",
    "renderer_rotation",
    "tf_translation",
    "tf_rotation",
    "renderer_tf_translation_residual_m",
    "renderer_tf_rotation_residual_rad",
    "nominal_effective_translation_delta_m",
    "nominal_effective_rotation_delta_rad",
    "seed",
    "evidence_class",
    "status",
    "effective_extrinsic_id",
    "input_status",
    "result_semantics",
    "evidence_generation_commit",
]


def write_camera_samples_csv(path: Path, rows: Sequence[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=_CAMERA_SAMPLE_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in _CAMERA_SAMPLE_FIELDS})


_TIMESTAMP_FIELDS = [
    "scenario",
    "anchor_modality",
    "other_modality",
    "signed_delta_s",
    "abs_delta_s",
    "injected_delay_s",
    "recovered_delay_s",
    "skew_class",
    "source_time_status",
    "sequence_flag",
    "input_status",
    "result_semantics",
    "evidence_class",
    "physical",
    "abs_slop_would_reject",
    "stale_vs_slop",
    "spatial_lag_m",
    "speed_m_s",
    "evidence_generation_commit",
]


def write_timestamp_skew_csv(path: Path, rows: Sequence[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=_TIMESTAMP_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in _TIMESTAMP_FIELDS})
