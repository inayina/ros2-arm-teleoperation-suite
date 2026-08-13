# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Evidence serialization for Stage-1 geometry diagnostics."""

from __future__ import annotations

import csv
import json
import subprocess
from pathlib import Path
from typing import Any, Iterable, Sequence

from teleop_diagnostics.types import ResidualRow, ResultSemantics


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


def write_run_manifest(path: Path, manifest: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_geometry_diagnostics_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


_CSV_FIELDS = [
    "commit",
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
        writer = csv.DictWriter(fh, fieldnames=_CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            d = row.to_dict()
            writer.writerow(
                {
                    "commit": d.get("commit", ""),
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
                    "result_semantics": d.get("result_semantics", ResultSemantics.REPORT_ONLY.value),
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
]


def write_fault_matrix_csv(path: Path, rows: Sequence[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=_FAULT_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in _FAULT_FIELDS})
