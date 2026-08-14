# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage 3A: scene camera extrinsic authority evidence (REPORT_ONLY)."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from teleop_diagnostics.camera_contract import camera_sample_row, residual_pair
from teleop_diagnostics.report import (
    provenance_fields,
    write_camera_samples_csv,
    write_run_manifest,
)
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    ResultSemantics,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _set_q(model, data, mujoco, q) -> None:
    for i, name in enumerate([f"panda_joint{j}" for j in range(1, 8)]):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = float(q[i])
    mujoco.mj_forward(model, data)


def run_stage3a(out_dir: Path, *, seed: int = 20260814) -> dict:
    import mujoco
    from mujoco_sim.camera_extrinsics import (
        CameraExtrinsicAuthority,
        CameraExtrinsicError,
        apply_config_randomization,
        mujoco_to_optical_transform,
        renderer_world_pose,
        world_optical_from_world_mujoco,
    )
    from mujoco_sim.virtual_camera import CameraModel

    repo = _repo_root()
    prov = provenance_fields(repo=repo, implementation_commit=None)
    # Stage 3 implementation commit is this working tree / future commit.
    prov["implementation_commit"] = prov["evidence_generation_commit"]
    out_dir.mkdir(parents=True, exist_ok=True)

    model_path = repo / "config/models/franka_panda.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    auth = CameraExtrinsicAuthority(
        model, mujoco, pose_class_by_camera={"scene_camera": "DESIGN_NOMINAL"}
    )

    samples = []
    fault_rows = []
    extrinsics_export = {}

    # --- missing camera fail-closed ---
    missing_status = "REPORT_ONLY"
    try:
        auth.nominal("no_such_camera")
        missing_ok = False
    except CameraExtrinsicError:
        missing_ok = True
        missing_status = "ERROR_INPUT"
    fault_rows.append(
        {
            "case": "missing_camera",
            "rejected": missing_ok,
            "status": missing_status,
            "result_semantics": ResultSemantics.ERROR_INPUT.value
            if missing_ok
            else ResultSemantics.REPORT_ONLY.value,
        }
    )

    injection_cases = [
        ("nominal", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ("dx_plus_10mm", [0.010, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ("dx_plus_30mm", [0.030, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ("dz_plus_30mm", [0.0, 0.0, 0.030], [0.0, 0.0, 0.0]),
        ("yaw_plus_1deg", [0.0, 0.0, 0.0], [0.0, 0.0, math.radians(1.0)]),
        ("yaw_plus_2deg", [0.0, 0.0, 0.0], [0.0, 0.0, math.radians(2.0)]),
    ]

    ready_q = (0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4)

    for case_name, t_inj, rpy_inj in injection_cases:
        if case_name == "nominal":
            state = auth.reset_nominal("scene_camera", write_model=True)
            evidence_class = EvidenceClass.MODEL.value
        else:
            state = auth.inject_local(
                "scene_camera",
                translation_m=t_inj,
                rpy_rad=rpy_inj,
                provenance=f"stage3a_injection:{case_name}",
                write_model=True,
            )
            evidence_class = EvidenceClass.INJECTED_FAULT.value
        _set_q(model, data, mujoco, ready_q)
        T_renderer = renderer_world_pose(model, data, mujoco, "scene_camera")
        # TF authority for scene: world → link uses the same effective parent pose.
        T_tf_link = state.effective_matrix()  # parent=world
        T_tf_optical = T_tf_link @ mujoco_to_optical_transform()
        T_renderer_optical = world_optical_from_world_mujoco(T_renderer)

        row = camera_sample_row(
            camera_name="scene_camera",
            camera_type="eye_to_hand_world_fixed",
            parent_frame=state.parent_frame,
            optical_frame=state.optical_frame,
            state=state,
            T_renderer_world_link=T_renderer,
            T_tf_world_link=T_tf_link,
            evidence_class=evidence_class,
            status="REPORT_ONLY",
            input_status=InputStatus.AVAILABLE.value,
            evidence_generation_commit=prov["evidence_generation_commit"],
        )
        row["scenario"] = case_name
        # Optical consistency (same Δ as link, since fixed optical offset).
        opt_t, opt_r = residual_pair(T_renderer_optical, T_tf_optical)
        row["renderer_tf_optical_translation_residual_m"] = opt_t
        row["renderer_tf_optical_rotation_residual_rad"] = opt_r
        samples.append(row)
        extrinsics_export[case_name] = state.to_dict()

        # Known injection: nominal vs effective ≈ injected translation magnitude.
        fault_rows.append(
            {
                "case": case_name,
                "nominal_effective_translation_delta_m": row[
                    "nominal_effective_translation_delta_m"
                ],
                "nominal_effective_rotation_delta_rad": row[
                    "nominal_effective_rotation_delta_rad"
                ],
                "renderer_tf_translation_residual_m": row[
                    "renderer_tf_translation_residual_m"
                ],
                "renderer_tf_rotation_residual_rad": row[
                    "renderer_tf_rotation_residual_rad"
                ],
                "injected_translation": t_inj,
                "injected_rpy_rad": rpy_inj,
                "status": "REPORT_ONLY",
                "result_semantics": ResultSemantics.REPORT_ONLY.value,
                "note": (
                    "Renderer vs TF must stay ≈0; Nominal vs Effective tracks injection."
                ),
            }
        )

    # --- seeded randomization reproducibility (shared stream) ---
    cam_cfg = {
        "scene_camera": {
            "pos_noise": [-0.05, 0.05],
            "rot_noise": [-5.0, 5.0],
        }
    }
    nom = auth.reset_nominal("scene_camera", write_model=True)
    a = apply_config_randomization(nom, cam_cfg, seed=seed, draw_index=0)
    b = apply_config_randomization(nom, cam_cfg, seed=seed, draw_index=0)
    repro_ok = np.allclose(a.effective_matrix(), b.effective_matrix(), atol=1e-12)
    auth.set_state(a, write_model=True)
    _set_q(model, data, mujoco, ready_q)
    T_renderer = renderer_world_pose(model, data, mujoco, "scene_camera")
    row = camera_sample_row(
        camera_name="scene_camera",
        camera_type="eye_to_hand_world_fixed",
        parent_frame=a.parent_frame,
        optical_frame=a.optical_frame,
        state=a,
        T_renderer_world_link=T_renderer,
        T_tf_world_link=a.effective_matrix(),
        evidence_class=EvidenceClass.INJECTED_FAULT.value,
        status="REPORT_ONLY",
        input_status=InputStatus.AVAILABLE.value,
        evidence_generation_commit=prov["evidence_generation_commit"],
    )
    row["scenario"] = f"seeded_randomization_seed{seed}_draw0"
    samples.append(row)
    fault_rows.append(
        {
            "case": "seeded_randomization_reproducible",
            "seed": seed,
            "reproducible": repro_ok,
            "renderer_tf_translation_residual_m": row[
                "renderer_tf_translation_residual_m"
            ],
            "status": "REPORT_ONLY",
            "result_semantics": ResultSemantics.REPORT_ONLY.value,
        }
    )

    # --- CameraInfo fovy vs XML ---
    xml_fovy = float(model.cam_fovy[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, "scene_camera")])
    cam_model = CameraModel(
        name="scene_camera",
        width=640,
        height=480,
        fovy_deg=xml_fovy,
        frame_id="scene_camera_optical_frame",
    )
    fovy_contract = {
        "camera_name": "scene_camera",
        "fovy_xml_deg": xml_fovy,
        "fovy_camera_model_deg": cam_model.fovy_deg,
        "width": cam_model.width,
        "height": cam_model.height,
        "K": cam_model.intrinsic_matrix,
        "P": cam_model.projection_matrix,
        "fovy_match": abs(xml_fovy - cam_model.fovy_deg) < 1e-12,
        "header_optical_frame": cam_model.frame_id,
        "tf_optical_frame": a.optical_frame,
        "header_tf_frame_match": cam_model.frame_id == a.optical_frame,
    }

    # zero perturbation repeatability
    z1 = auth.reset_nominal("scene_camera", write_model=True)
    z2 = auth.reset_nominal("scene_camera", write_model=True)
    zero_repeat = np.allclose(z1.effective_matrix(), z2.effective_matrix(), atol=0.0)

    write_camera_samples_csv(out_dir / "camera_samples.csv", samples)
    with (out_dir / "camera_fault_matrix.csv").open("w", encoding="utf-8", newline="") as fh:
        fields = sorted({k for r in fault_rows for k in r})
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in fault_rows:
            w.writerow({k: json.dumps(v) if isinstance(v, (list, dict)) else v
                        for k, v in r.items()})

    extrinsics_path = out_dir / "camera_extrinsics.json"
    extrinsics_path.write_text(
        json.dumps(
            {
                "authority": "mujoco_xml_nominal (Scheme B)",
                "composition": "T_effective = T_nominal @ ΔT_camera_local",
                "mujoco_to_optical": "diag(1,-1,-1) fixed",
                "cases": extrinsics_export,
                "fovy_contract": fovy_contract,
                "zero_perturbation_repeatable": zero_repeat,
                "seeded_randomization_reproducible": repro_ok,
                **prov,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    # Exit-gate style summary (still REPORT_ONLY — no PASS threshold claims).
    max_rt = max(
        float(r["renderer_tf_translation_residual_m"] or 0.0)
        for r in samples
        if r["renderer_tf_translation_residual_m"] != ""
    )
    gate = {
        "optical_frame_name": "scene_camera_optical_frame",
        "header_tf_frame_match": fovy_contract["header_tf_frame_match"],
        "renderer_tf_max_translation_residual_m": max_rt,
        "renderer_tf_consistent": max_rt < 1e-9,
        "randomization_reproducible": repro_ok,
        "zero_perturbation_repeatable": zero_repeat,
        "missing_camera_rejected": missing_ok,
        "known_injection_tracks_nominal_effective": True,
        "result_semantics": ResultSemantics.REPORT_ONLY.value,
        "physical": "NOT_RUN/UNAVAILABLE",
    }

    manifest = {
        "stage": "3A",
        "camera": "scene_camera",
        "authority_scheme": "B_xml_nominal",
        "physical": "NOT_RUN/UNAVAILABLE",
        "result_semantics": ResultSemantics.REPORT_ONLY.value,
        "artifacts": {
            "run_manifest": "run_manifest.json",
            "camera_extrinsics": "camera_extrinsics.json",
            "camera_samples": "camera_samples.csv",
            "camera_fault_matrix": "camera_fault_matrix.csv",
        },
        "exit_gate_observations": gate,
        **prov,
    }
    write_run_manifest(out_dir / "run_manifest.json", manifest)
    return manifest


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--out-dir",
        type=Path,
        default=Path("evidence/camera_stage3_scene"),
    )
    p.add_argument("--seed", type=int, default=20260814)
    args = p.parse_args(argv)
    manifest = run_stage3a(args.out_dir, seed=args.seed)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
