# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage-2 geometry fault-injection report (diagnostic copies only)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from teleop_diagnostics import FRAME_EE, FRAME_LINK7
from teleop_diagnostics.controller_fk import controller_analytic_fk
from teleop_diagnostics.faults import (
    DiagnosticFaultCopy,
    JointOriginOffsetFault,
    JointZeroFault,
    TcpOffsetFault,
    classify_fault_residual,
    deg_to_rad,
    residual_vs_reference,
)
from teleop_diagnostics.frames import FrameNormalizer
from teleop_diagnostics.mujoco_ee import MujocoEeSource
from teleop_diagnostics.poses import nominal_pose_set
from teleop_diagnostics.report import (
    git_commit,
    provenance_fields,
    write_fault_matrix_csv,
    write_geometry_diagnostics_json,
    write_geometry_samples_csv,
    write_run_manifest,
)
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    ResidualRow,
    ResultSemantics,
)
from teleop_diagnostics.urdf_fk import IndependentUrdfFk, expand_xacro_to_urdf, load_urdf_model


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def run_stage2_report(out_dir: Path, *, seed: int = 20260813) -> dict:
    repo = _repo_root()
    commit = git_commit(repo)
    xacro = repo / "src/teleop_description/urdf/panda.urdf.xacro"
    mujoco_model = repo / "config/models/franka_panda.xml"
    out_dir.mkdir(parents=True, exist_ok=True)

    robot = load_urdf_model(expand_xacro_to_urdf(xacro))
    urdf_fk = IndependentUrdfFk(robot)
    normalizer = FrameNormalizer(urdf_fk)
    faults = DiagnosticFaultCopy(urdf_fk, normalizer)
    mujoco = MujocoEeSource(mujoco_model)
    poses = nominal_pose_set(random_count=3, seed=seed)

    fault_rows = []
    sample_rows: list[ResidualRow] = []
    pattern_summaries = []

    # --- Prove frame normalization: raw link7 vs ee must be rejected ---
    q0 = poses[0].q
    link7 = controller_analytic_fk(q0)
    ee = urdf_fk.forward(q0, target_link=FRAME_EE)
    raw_compare_rejected = False
    try:
        normalizer.compare_same_tip(link7, ee, require_ee=True)
    except GeometryDiagnosticsError:
        raw_compare_rejected = True
    canon = normalizer.canonicalize_to_ee(link7)
    canon_vs_ee = normalizer.compare_same_tip(canon, ee, require_ee=True)

    joint_zero_cases = [
        JointZeroFault({}),
        JointZeroFault({"panda_joint3": deg_to_rad(0.5)}),
        JointZeroFault({"panda_joint3": deg_to_rad(-0.5)}),
        JointZeroFault({"panda_joint3": deg_to_rad(2.0)}),
        JointZeroFault({"panda_joint1": deg_to_rad(0.5), "panda_joint5": deg_to_rad(-1.0)}),
    ]
    tcp_cases = [
        TcpOffsetFault(),
        TcpOffsetFault(dx_m=0.010),
        TcpOffsetFault(dz_m=0.010),
        TcpOffsetFault(dz_m=0.030),
        TcpOffsetFault(dyaw_rad=deg_to_rad(1.0)),
    ]
    origin_cases = [
        JointOriginOffsetFault("panda_joint4", dz_m=0.005),
    ]

    # Joint-zero: measure pose dependence of residual magnitude
    for fault in joint_zero_cases:
        residuals = []
        zero_baseline = None
        for pose in poses:
            ref = mujoco.forward(pose.q)
            if ref.evidence_class != EvidenceClass.SIM_GT:
                continue
            faulted = faults.fk_with_joint_zero(pose.q, fault)
            # Also compare vs nominal URDF EE (same tip)
            nom = urdf_fk.forward(pose.q, target_link=FRAME_EE)
            res = residual_vs_reference(faulted, nom, normalizer)
            residuals.append(res["translation_error_m"])
            if pose.name == "zero":
                zero_baseline = res["translation_error_m"]
            desc = fault.describe()
            joint_name = next(iter(desc["offsets_rad"]), "")
            offset_rad = desc["offsets_rad"].get(joint_name, 0.0)
            classification = classify_fault_residual(
                fault_type="joint_zero_offset",
                translation_error_m=res["translation_error_m"],
                tool_local_xyz=res["tool_local_translation_m"],
                pose_dependence_std_m=0.0,  # filled after loop
                zero_injection_residual_m=0.0 if fault.offsets_rad else res["translation_error_m"],
            )
            fault_rows.append(
                {
                    "scenario": pose.name,
                    "fault_type": "joint_zero_offset",
                    "joint_name": joint_name,
                    "offset_rad": offset_rad,
                    "offset_deg": desc["offsets_deg"].get(joint_name, 0.0),
                    "tcp_dx_m": "",
                    "tcp_dy_m": "",
                    "tcp_dz_m": "",
                    "tcp_droll_rad": "",
                    "tcp_dpitch_rad": "",
                    "tcp_dyaw_rad": "",
                    "pose_name": pose.name,
                    "translation_error_m": res["translation_error_m"],
                    "rotation_error_rad": res["rotation_error_rad"],
                    "tool_local_error_m": res["tool_local_error_m"],
                    "tool_local_xyz_m": json.dumps(res["tool_local_translation_m"]),
                    "status": classification["status"],
                    "suspected_cause": classification["suspected_cause"],
                    "evidence_class": EvidenceClass.INJECTED_FAULT.value,
                    "result_semantics": classification["result_semantics"],
                    "physical": "NOT_RUN/UNAVAILABLE",
                    "commit": commit,
                }
            )
            sample_rows.append(
                ResidualRow(
                    scenario=f"joint_zero__{pose.name}__{joint_name or 'none'}",
                    q=list(pose.q),
                    source_a="diagnostic_fk_joint_zero",
                    source_b="independent_urdf_fk",
                    frame_from="panda_link0",
                    frame_to=FRAME_EE,
                    reference_point=FRAME_EE,
                    translation_error_m=res["translation_error_m"],
                    rotation_error_rad=res["rotation_error_rad"],
                    evidence_class_a=EvidenceClass.INJECTED_FAULT.value,
                    evidence_class_b=EvidenceClass.MODEL.value,
                    input_status="AVAILABLE",
                    result_semantics=classification["result_semantics"],
                    backend=f"mujoco={mujoco.backend}",
                    commit=commit,
                )
            )
        std = float(np.std(residuals)) if residuals else 0.0
        # Update pose-dependence classification on summary
        mean_t = float(np.mean(residuals)) if residuals else 0.0
        summary_class = classify_fault_residual(
            fault_type="joint_zero_offset",
            translation_error_m=mean_t,
            tool_local_xyz=[0, 0, 0],
            pose_dependence_std_m=std,
            zero_injection_residual_m=0.0 if fault.offsets_rad else (zero_baseline or 0.0),
        )
        pattern_summaries.append(
            {
                "fault": fault.describe(),
                "pose_residuals_m": residuals,
                "pose_dependence_std_m": std,
                "classification": summary_class,
            }
        )

    for fault in tcp_cases:
        for pose in poses:
            nom = urdf_fk.forward(pose.q, target_link=FRAME_EE)
            faulted = faults.fk_with_tcp(pose.q, fault)
            res = residual_vs_reference(faulted, nom, normalizer)
            desc = fault.describe()
            classification = classify_fault_residual(
                fault_type="tcp_offset",
                translation_error_m=res["translation_error_m"],
                tool_local_xyz=res["tool_local_translation_m"],
                pose_dependence_std_m=0.0,
                zero_injection_residual_m=0.0
                if any(
                    abs(v) > 0
                    for v in (
                        fault.dx_m,
                        fault.dy_m,
                        fault.dz_m,
                        fault.droll_rad,
                        fault.dpitch_rad,
                        fault.dyaw_rad,
                    )
                )
                else res["translation_error_m"],
            )
            fault_rows.append(
                {
                    "scenario": pose.name,
                    "fault_type": "tcp_offset",
                    "joint_name": "",
                    "offset_rad": "",
                    "offset_deg": "",
                    "tcp_dx_m": desc["tcp_dx_m"],
                    "tcp_dy_m": desc["tcp_dy_m"],
                    "tcp_dz_m": desc["tcp_dz_m"],
                    "tcp_droll_rad": desc["tcp_droll_rad"],
                    "tcp_dpitch_rad": desc["tcp_dpitch_rad"],
                    "tcp_dyaw_rad": desc["tcp_dyaw_rad"],
                    "pose_name": pose.name,
                    "translation_error_m": res["translation_error_m"],
                    "rotation_error_rad": res["rotation_error_rad"],
                    "tool_local_error_m": res["tool_local_error_m"],
                    "tool_local_xyz_m": json.dumps(res["tool_local_translation_m"]),
                    "status": classification["status"],
                    "suspected_cause": classification["suspected_cause"],
                    "evidence_class": EvidenceClass.INJECTED_FAULT.value,
                    "result_semantics": classification["result_semantics"],
                    "physical": "NOT_RUN/UNAVAILABLE",
                    "commit": commit,
                }
            )

    for fault in origin_cases:
        for pose in poses:
            nom = urdf_fk.forward(pose.q, target_link=FRAME_EE)
            faulted = faults.fk_with_joint_origin(pose.q, fault)
            res = residual_vs_reference(faulted, nom, normalizer)
            desc = fault.describe()
            fault_rows.append(
                {
                    "scenario": pose.name,
                    "fault_type": "joint_origin_offset",
                    "joint_name": desc["joint_name"],
                    "offset_rad": "",
                    "offset_deg": "",
                    "tcp_dx_m": desc["origin_dx_m"],
                    "tcp_dy_m": desc["origin_dy_m"],
                    "tcp_dz_m": desc["origin_dz_m"],
                    "tcp_droll_rad": "",
                    "tcp_dpitch_rad": "",
                    "tcp_dyaw_rad": "",
                    "pose_name": pose.name,
                    "translation_error_m": res["translation_error_m"],
                    "rotation_error_rad": res["rotation_error_rad"],
                    "tool_local_error_m": res["tool_local_error_m"],
                    "tool_local_xyz_m": json.dumps(res["tool_local_translation_m"]),
                    "status": ResultSemantics.SUSPECTED.value,
                    "suspected_cause": "JOINT_ORIGIN_OFFSET",
                    "evidence_class": EvidenceClass.INJECTED_FAULT.value,
                    "result_semantics": ResultSemantics.SUSPECTED.value,
                    "physical": "NOT_RUN/UNAVAILABLE",
                    "commit": commit,
                }
            )

    # Ambiguity note: single-pose translation residual alone cannot separate joint-zero vs TCP.
    ambiguity_note = {
        "statement": (
            "On a single posture, a base-frame translation residual of similar magnitude "
            "can be produced by either joint-zero or TCP injection; multi-pose dependence "
            "and tool-local residual are required to reduce ambiguity."
        ),
        "when_ambiguous": [
            "single posture only",
            "comparing residuals without tool-local frame",
            "mixing panda_link7 and panda_ee without canonicalize_to_ee",
        ],
    }

    manifest = {
        "stage": 2,
        "commit": commit,
        **provenance_fields(repo=repo),
        "result_semantics_allowed": [
            "REPORT_ONLY",
            "SUSPECTED",
            "AMBIGUOUS",
            "INSUFFICIENT_DATA",
            "ERROR_INPUT",
        ],
        "forbidden_semantics": ["PASS", "FAIL", "CALIBRATED", "ROOT_CAUSE_CONFIRMED"],
        "physical": "NOT_RUN/UNAVAILABLE",
        "control_law_modified": False,
        "runtime_topics_mutated": False,
        "frame_normalization": {
            "raw_link7_vs_ee_rejected": raw_compare_rejected,
            "canonicalized_link7_vs_ee": canon_vs_ee,
            "controller_reference": FRAME_LINK7,
            "tool_frame": FRAME_EE,
        },
        "artifacts": {
            "run_manifest": "run_manifest.json",
            "geometry_diagnostics": "geometry_diagnostics.json",
            "geometry_samples": "geometry_samples.csv",
            "fault_matrix": "fault_matrix.csv",
        },
    }
    diagnostics = {
        "commit": commit,
        "physical": "NOT_RUN/UNAVAILABLE",
        "joint_zero_pattern_summaries": pattern_summaries,
        "ambiguity_note": ambiguity_note,
        "frame_normalization": manifest["frame_normalization"],
        "current_contract": {
            "impedance_controller_reference": "panda_link7",
            "moveit_servo_tip": "panda_ee",
            "note": "Not a bug unless product contract requires identical tips.",
        },
    }
    write_run_manifest(out_dir / "run_manifest.json", manifest)
    write_geometry_diagnostics_json(out_dir / "geometry_diagnostics.json", diagnostics)
    write_geometry_samples_csv(out_dir / "geometry_samples.csv", sample_rows)
    write_fault_matrix_csv(out_dir / "fault_matrix.csv", fault_rows)
    return manifest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Stage-2 geometry fault injection report")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=_repo_root() / "evidence" / "geometry_stage2",
    )
    parser.add_argument("--seed", type=int, default=20260813)
    args = parser.parse_args(argv)
    manifest = run_stage2_report(args.out_dir, seed=args.seed)
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
