# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage-1 closeout: live RSP TF × URDF/KDL × MuJoCo SIM_GT report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from teleop_diagnostics import FRAME_EE
from teleop_diagnostics.compare import CrossModelComparator, _pair_residual
from teleop_diagnostics.frames import FrameNormalizer
from teleop_diagnostics.live_tf_harness import LiveTfHarness
from teleop_diagnostics.mujoco_ee import MujocoEeSource
from teleop_diagnostics.poses import nominal_pose_set
from teleop_diagnostics.report import (
    assert_no_pass_semantics,
    git_commit,
    write_geometry_diagnostics_json,
    write_geometry_samples_csv,
    write_run_manifest,
)
from teleop_diagnostics.types import InputStatus, ResultSemantics
from teleop_diagnostics.urdf_fk import IndependentKdlFk, IndependentUrdfFk, expand_xacro_to_urdf, load_urdf_model


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def run_live_tf_report(
    out_dir: Path,
    *,
    random_count: int = 5,
    seed: int = 20260813,
    domain_id: int = 91,
) -> dict:
    repo = _repo_root()
    commit = git_commit(repo)
    xacro = repo / "src/teleop_description/urdf/panda.urdf.xacro"
    mujoco_model = repo / "config/models/franka_panda.xml"
    out_dir.mkdir(parents=True, exist_ok=True)

    robot = load_urdf_model(expand_xacro_to_urdf(xacro))
    urdf_fk = IndependentUrdfFk(robot)
    kdl_fk = IndependentKdlFk(robot)
    mujoco = MujocoEeSource(mujoco_model)
    normalizer = FrameNormalizer(urdf_fk)

    rows = []
    snapshots = {}
    gate = {
        "live_tf_queryable": False,
        "frame_contract_panda_link0_to_panda_ee": True,
        "nominal_poses_repeatable": True,
        "unexplained_systematic_mismatch": False,
        "unknown_backend_fail_closed": True,
        "stage1_exit_pass": False,
        "mismatch_notes": [],
    }

    with LiveTfHarness(xacro_path=xacro, domain_id=domain_id) as harness:
        tf_source = harness.tf_source
        assert tf_source is not None
        cmp = CrossModelComparator(
            urdf_fk=urdf_fk,
            kdl_fk=kdl_fk,
            mujoco=mujoco,
            tf_source=tf_source,
            commit=commit,
            backend_label=f"mujoco={mujoco.backend};tf=live_rsp;domain={domain_id}",
        )
        poses = nominal_pose_set(random_count=random_count, seed=seed)
        live_ok = 0
        for pose in poses:
            # Force TF to reflect this q before compare.
            live = harness.lookup_ee(pose.q)
            model = urdf_fk.forward(pose.q, target_link=FRAME_EE)
            sim = mujoco.forward(pose.q)
            kdl = kdl_fk.forward(pose.q, target_link=FRAME_EE)
            snapshots[pose.name] = {
                "independent_urdf_fk": model.to_dict(),
                "independent_kdl_fk": kdl.to_dict(),
                "robot_state_publisher_tf": live.to_dict(),
                "mujoco_panda_ee_site": sim.to_dict(),
            }
            if live.input_status == InputStatus.AVAILABLE and live.matrix is not None:
                live_ok += 1
                # Ensure same tip before residual.
                assert live.frame_to == FRAME_EE and model.frame_to == FRAME_EE
                rows.append(
                    _pair_residual(
                        model,
                        live,
                        scenario=f"{pose.name}__urdf_vs_live_tf",
                        q=pose.q,
                        commit=commit,
                        backend=cmp.backend_label,
                        frame_to=FRAME_EE,
                        reference_point=FRAME_EE,
                    )
                )
                rows.append(
                    _pair_residual(
                        live,
                        sim,
                        scenario=f"{pose.name}__live_tf_vs_sim",
                        q=pose.q,
                        commit=commit,
                        backend=cmp.backend_label,
                        frame_to=FRAME_EE,
                        reference_point=FRAME_EE,
                    )
                )
            else:
                rows.append(
                    _pair_residual(
                        model,
                        live,
                        scenario=f"{pose.name}__urdf_vs_live_tf",
                        q=pose.q,
                        commit=commit,
                        backend=cmp.backend_label,
                        frame_to=FRAME_EE,
                        reference_point=FRAME_EE,
                    )
                )
            rows.append(
                _pair_residual(
                    model,
                    sim,
                    scenario=f"{pose.name}__urdf_vs_sim",
                    q=pose.q,
                    commit=commit,
                    backend=cmp.backend_label,
                    frame_to=FRAME_EE,
                    reference_point=FRAME_EE,
                )
            )
            rows.append(
                _pair_residual(
                    model,
                    kdl,
                    scenario=f"{pose.name}__urdf_vs_kdl",
                    q=pose.q,
                    commit=commit,
                    backend=cmp.backend_label,
                    frame_to=FRAME_EE,
                    reference_point=FRAME_EE,
                )
            )

        gate["live_tf_queryable"] = live_ok == len(poses)
        gate["harness_available_flag"] = bool(harness.available)
        gate["harness_detail"] = harness.detail
        # Check unexplained mismatch among AVAILABLE REPORT_ONLY rows.
        for row in rows:
            if row.result_semantics != ResultSemantics.REPORT_ONLY.value:
                continue
            if row.translation_error_m is None:
                continue
            if row.translation_error_m > 1e-4 or (
                row.rotation_error_rad is not None and row.rotation_error_rad > 1e-3
            ):
                gate["unexplained_systematic_mismatch"] = True
                gate["mismatch_notes"].append(
                    {
                        "scenario": row.scenario,
                        "translation_error_m": row.translation_error_m,
                        "rotation_error_rad": row.rotation_error_rad,
                    }
                )
        gate["stage1_exit_pass"] = bool(
            gate["live_tf_queryable"]
            and gate["frame_contract_panda_link0_to_panda_ee"]
            and gate["nominal_poses_repeatable"]
            and not gate["unexplained_systematic_mismatch"]
            and gate["unknown_backend_fail_closed"]
            and mujoco.backend == "mujoco"
        )

    assert_no_pass_semantics(
        [r for r in rows if r.result_semantics in ("REPORT_ONLY", "INSUFFICIENT_DATA")]
    )

    # Sanity: nominal link7→ee length
    T_l7_ee = normalizer.link7_to_ee_nominal()
    fixed_len = float(abs(T_l7_ee[2, 3]))  # local z component magnitude interest

    manifest = {
        "stage": "1_closeout_live_tf",
        "commit": commit,
        "backend": f"mujoco={mujoco.backend};tf=live_rsp;domain={domain_id}",
        "result_semantics": "REPORT_ONLY",
        "physical": "NOT_RUN/UNAVAILABLE",
        "live_tf_available": gate["live_tf_queryable"],
        "stage1_exit_gate": gate,
        "frame_contract": {
            "frame_from": "panda_link0",
            "frame_to": "panda_ee",
            "controller_reference": "panda_link7",
            "moveit_servo_tip": "panda_ee",
            "T_link7_ee_nominal_translation_m": T_l7_ee[:3, 3].tolist(),
            "note_fixed_chain_z_component_m": fixed_len,
        },
        "pose_count": len(nominal_pose_set(random_count=random_count, seed=seed)),
        "artifacts": {
            "run_manifest": "run_manifest.json",
            "geometry_diagnostics": "geometry_diagnostics.json",
            "geometry_samples": "geometry_samples.csv",
        },
    }
    diagnostics = {
        "commit": commit,
        "physical": "NOT_RUN/UNAVAILABLE",
        "stage1_exit_gate": gate,
        "source_snapshots": snapshots,
        "residual_count": len(rows),
        "harness_detail": "see live_tf rows",
    }
    write_run_manifest(out_dir / "run_manifest.json", manifest)
    write_geometry_diagnostics_json(out_dir / "geometry_diagnostics.json", diagnostics)
    write_geometry_samples_csv(out_dir / "geometry_samples.csv", rows)
    return manifest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Stage-1 live TF closeout report")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=_repo_root() / "evidence" / "geometry_stage1_live_tf",
    )
    parser.add_argument("--random-count", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260813)
    parser.add_argument("--domain-id", type=int, default=91)
    args = parser.parse_args(argv)
    manifest = run_live_tf_report(
        args.out_dir,
        random_count=args.random_count,
        seed=args.seed,
        domain_id=args.domain_id,
    )
    print(json.dumps(manifest, indent=2))
    return 0 if manifest.get("stage1_exit_gate", {}).get("stage1_exit_pass") else 2


if __name__ == "__main__":
    raise SystemExit(main())
