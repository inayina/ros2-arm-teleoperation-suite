# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""CLI: generate Stage-1 geometry consistency evidence (REPORT_ONLY)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from teleop_diagnostics.compare import CrossModelComparator
from teleop_diagnostics.mujoco_ee import MujocoEeSource
from teleop_diagnostics.poses import nominal_pose_set
from teleop_diagnostics.report import (
    assert_no_pass_semantics,
    git_commit,
    write_geometry_diagnostics_json,
    write_geometry_samples_csv,
    write_run_manifest,
)
from teleop_diagnostics.tf_source import RobotStatePublisherTfSource
from teleop_diagnostics.urdf_fk import IndependentKdlFk, IndependentUrdfFk, expand_xacro_to_urdf, load_urdf_model


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def build_comparator(
    *,
    repo: Path,
    xacro: Path,
    mujoco_model: Path,
    commit: str,
) -> CrossModelComparator:
    xml = expand_xacro_to_urdf(xacro)
    robot = load_urdf_model(xml)
    urdf_fk = IndependentUrdfFk(robot)
    kdl_fk = IndependentKdlFk(robot)
    mujoco = MujocoEeSource(mujoco_model)
    tf = RobotStatePublisherTfSource()  # offline default: UNAVAILABLE
    backend = f"mujoco={mujoco.backend};tf=unavailable_offline"
    return CrossModelComparator(
        urdf_fk=urdf_fk,
        kdl_fk=kdl_fk,
        mujoco=mujoco,
        tf_source=tf,
        commit=commit,
        backend_label=backend,
    )


def run_report(out_dir: Path, *, random_count: int = 5, seed: int = 20260813) -> dict:
    repo = _repo_root()
    commit = git_commit(repo)
    xacro = repo / "src/teleop_description/urdf/panda.urdf.xacro"
    mujoco_model = repo / "config/models/franka_panda.xml"
    out_dir.mkdir(parents=True, exist_ok=True)

    cmp = build_comparator(repo=repo, xacro=xacro, mujoco_model=mujoco_model, commit=commit)
    poses = nominal_pose_set(random_count=random_count, seed=seed)
    rows = []
    controller_audits = {}
    source_snapshots = {}
    for pose in poses:
        rows.extend(cmp.compare_nominal(pose.name, pose.q))
        controller_audits[pose.name] = cmp.controller_reference_audit(pose.q)
        samples = cmp.sample_sources(pose.q)
        source_snapshots[pose.name] = {k: v.to_dict() for k, v in samples.items()}

    assert_no_pass_semantics(rows)

    zero_audit = controller_audits.get("zero", {})
    manifest = {
        "stage": 1,
        "commit": commit,
        "backend": cmp.backend_label,
        "scenario_set": [p.name for p in poses],
        "result_semantics": "REPORT_ONLY",
        "physical": "NOT_RUN/UNAVAILABLE",
        "mujoco_backend": cmp.mujoco.backend,
        "mujoco_evidence_class_policy": {
            "mujoco": "SIM_GT",
            "fallback": "MODEL",
            "unknown": "INSUFFICIENT_DATA",
        },
        "controller_reference_frame_contract": {
            "reference_point": "panda_link7",
            "jacobian_reference_point": "panda_link7",
            "zero_pose_closest_urdf_frame": zero_audit.get("closest_urdf_frame_by_translation"),
            "zero_pose_closest_translation_error_m": zero_audit.get("closest_translation_error_m"),
            "control_law_modified": False,
            "tcp_0_207_compensation_applied": False,
        },
        "artifacts": {
            "run_manifest": "run_manifest.json",
            "geometry_diagnostics": "geometry_diagnostics.json",
            "geometry_samples": "geometry_samples.csv",
        },
    }
    diagnostics = {
        "commit": commit,
        "physical": "NOT_RUN/UNAVAILABLE",
        "controller_reference_audits": controller_audits,
        "source_snapshots": source_snapshots,
        "residual_count": len(rows),
        "notes": [
            "Stage 1 is REPORT_ONLY; no PASS threshold is applied.",
            "Live robot_state_publisher TF is UNAVAILABLE in offline CLI mode.",
            "MuJoCo path never uses mujoco_sim.fallback_ee_transform().",
        ],
    }

    write_run_manifest(out_dir / "run_manifest.json", manifest)
    write_geometry_diagnostics_json(out_dir / "geometry_diagnostics.json", diagnostics)
    write_geometry_samples_csv(out_dir / "geometry_samples.csv", rows)
    return manifest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Stage-1 geometry consistency report")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=_repo_root() / "evidence" / "geometry_stage1",
    )
    parser.add_argument("--random-count", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260813)
    args = parser.parse_args(argv)
    manifest = run_report(args.out_dir, random_count=args.random_count, seed=args.seed)
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
