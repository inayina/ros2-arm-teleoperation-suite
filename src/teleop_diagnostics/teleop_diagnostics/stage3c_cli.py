# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage 3C: freeze wrist DESIGN_NOMINAL + eye-in-hand extrinsic contract."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

from teleop_diagnostics.camera_contract import camera_sample_row, residual_pair
from teleop_diagnostics.poses import READY_Q, fixed_seed_random_poses
from teleop_diagnostics.report import (
    provenance_fields,
    write_camera_samples_csv,
    write_run_manifest,
)
from teleop_diagnostics.types import EvidenceClass, InputStatus, ResultSemantics
from teleop_diagnostics.wrist_pose_candidates import all_wrist_pose_candidates


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _set_q(model, data, mujoco, q) -> None:
    for i, name in enumerate([f"panda_joint{j}" for j in range(1, 8)]):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = float(q[i])
    mujoco.mj_forward(model, data)


def freeze_wrist_xml(model_xml: Path, candidate_id: str) -> dict:
    """Replace wrist_camera pose in franka_panda.xml with selected DESIGN_NOMINAL."""
    cand = next(c for c in all_wrist_pose_candidates() if c.candidate_id == candidate_id)
    R = np.asarray(cand.rotation_matrix)
    xyaxes = list(R[:, 0]) + list(R[:, 1])
    pos = cand.translation_xyz
    new_cam = (
        f'<camera name="wrist_camera" pos="{pos[0]} {pos[1]} {pos[2]}" '
        f'xyaxes="{" ".join(f"{v:.6g}" for v in xyaxes)}" '
        f'fovy="{cand.fovy_deg}" />'
    )
    text = model_xml.read_text(encoding="utf-8")
    pattern = re.compile(r'<camera name="wrist_camera"[^/]*/>')
    if not pattern.search(text):
        raise RuntimeError("wrist_camera not found in XML")
    updated = pattern.sub(new_cam, text, count=1)
    model_xml.write_text(updated, encoding="utf-8")
    return {
        "candidate_id": candidate_id,
        "xml_camera": new_cam,
        "pose_class": "DESIGN_NOMINAL",
        "physical": "NOT_RUN/UNAVAILABLE",
    }


def run_stage3c(
    out_dir: Path,
    *,
    selected_candidate_id: str | None = None,
    apply_xml_freeze: bool = True,
    seed: int = 20260814,
) -> dict:
    import mujoco
    from mujoco_sim.camera_extrinsics import (
        CameraExtrinsicAuthority,
        mujoco_to_optical_transform,
        renderer_world_pose,
    )

    repo = _repo_root()
    prov = provenance_fields(repo=repo)
    prov["implementation_commit"] = prov["evidence_generation_commit"]
    out_dir.mkdir(parents=True, exist_ok=True)

    # Resolve selection from Stage 3B evidence if present.
    tune_manifest = repo / "evidence/wrist_camera_pose_tuning/run_manifest.json"
    if selected_candidate_id is None and tune_manifest.is_file():
        selected_candidate_id = json.loads(tune_manifest.read_text())["selected_candidate_id"]
    if selected_candidate_id is None:
        selected_candidate_id = "C_higher_pitch_down"

    model_xml = repo / "config/models/franka_panda.xml"
    freeze_info = None
    if apply_xml_freeze:
        freeze_info = freeze_wrist_xml(model_xml, selected_candidate_id)

    model = mujoco.MjModel.from_xml_path(str(model_xml))
    data = mujoco.MjData(model)
    auth = CameraExtrinsicAuthority(
        model,
        mujoco,
        pose_class_by_camera={"wrist_camera": "DESIGN_NOMINAL"},
    )
    nom = auth.reset_nominal("wrist_camera", write_model=True)

    samples = []
    hand_rel = []

    poses = [("ready", READY_Q)] + [
        (f"random_{i}", q) for i, q in enumerate(fixed_seed_random_poses(3, seed=seed))
    ]

    for pose_name, q in poses:
        _set_q(model, data, mujoco, q)
        # Parent body pose
        hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
        T_world_hand = np.eye(4)
        T_world_hand[:3, :3] = np.asarray(data.xmat[hid], dtype=float).reshape(3, 3)
        T_world_hand[:3, 3] = np.asarray(data.xpos[hid], dtype=float)
        T_world_cam = renderer_world_pose(model, data, mujoco, "wrist_camera")
        T_hand_cam = np.linalg.inv(T_world_hand) @ T_world_cam
        T_tf_hand_cam = nom.effective_matrix()  # parent = panda_hand local
        row = camera_sample_row(
            camera_name="wrist_camera",
            camera_type="eye_in_hand",
            parent_frame=nom.parent_frame,
            optical_frame=nom.optical_frame,
            state=nom,
            T_renderer_world_link=T_hand_cam,  # compare in hand frame
            T_tf_world_link=T_tf_hand_cam,
            evidence_class=EvidenceClass.MODEL.value,
            status="REPORT_ONLY",
            input_status=InputStatus.AVAILABLE.value,
            evidence_generation_commit=prov["evidence_generation_commit"],
        )
        row["scenario"] = pose_name
        row["world_camera_translation"] = [float(x) for x in T_world_cam[:3, 3]]
        samples.append(row)
        hand_rel.append(T_hand_cam.copy())

    # Eye-in-hand: T_hand_camera stable across robot motion; T_world_camera changes.
    rel_stack = np.stack(hand_rel, axis=0)
    rel_spread = float(np.max(np.linalg.norm(rel_stack[:, :3, 3] - rel_stack[0, :3, 3], axis=1)))
    world_translations = np.array([r["world_camera_translation"] for r in samples])
    world_spread = float(
        np.max(np.linalg.norm(world_translations - world_translations[0], axis=1))
    )

    # Known perturbations in panda_hand / camera-local frame
    for case, t_inj, rpy in [
        ("dx_plus_10mm", [0.01, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ("dz_plus_10mm", [0.0, 0.0, 0.01], [0.0, 0.0, 0.0]),
        ("yaw_plus_1deg", [0.0, 0.0, 0.0], [0.0, 0.0, math.radians(1.0)]),
    ]:
        state = auth.inject_local(
            "wrist_camera",
            translation_m=t_inj,
            rpy_rad=rpy,
            provenance=f"stage3c_injection:{case}",
            write_model=True,
        )
        _set_q(model, data, mujoco, READY_Q)
        hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
        T_world_hand = np.eye(4)
        T_world_hand[:3, :3] = np.asarray(data.xmat[hid], dtype=float).reshape(3, 3)
        T_world_hand[:3, 3] = np.asarray(data.xpos[hid], dtype=float)
        T_world_cam = renderer_world_pose(model, data, mujoco, "wrist_camera")
        T_hand_cam = np.linalg.inv(T_world_hand) @ T_world_cam
        row = camera_sample_row(
            camera_name="wrist_camera",
            camera_type="eye_in_hand",
            parent_frame=state.parent_frame,
            optical_frame=state.optical_frame,
            state=state,
            T_renderer_world_link=T_hand_cam,
            T_tf_world_link=state.effective_matrix(),
            evidence_class=EvidenceClass.INJECTED_FAULT.value,
            status="REPORT_ONLY",
            input_status=InputStatus.AVAILABLE.value,
            evidence_generation_commit=prov["evidence_generation_commit"],
        )
        row["scenario"] = case
        samples.append(row)
        auth.reset_nominal("wrist_camera", write_model=True)

    write_camera_samples_csv(out_dir / "camera_samples.csv", samples)
    contract = {
        "selected_candidate_id": selected_candidate_id,
        "pose_class": "DESIGN_NOMINAL",
        "physical": "NOT_RUN/UNAVAILABLE",
        "not": ["PHYSICAL_CALIBRATED", "hand-eye solver"],
        "freeze": freeze_info,
        "eye_in_hand": {
            "T_hand_camera_spread_m": rel_spread,
            "T_world_camera_spread_m": world_spread,
            "hand_relative_stable": rel_spread < 1e-9,
            "world_pose_changes_with_motion": world_spread > 1e-3,
            "perturbation_parent_frame": "panda_hand (camera-local ΔT on nominal)",
            "composition": "T_effective = T_nominal @ ΔT_camera_local",
        },
        "optical_convention": {
            "link_frame": "wrist_camera_link = MuJoCo camera axes",
            "optical_frame": "wrist_camera_optical_frame = link @ diag(1,-1,-1)",
            "ros_optical": "+x right, +y down, +z forward",
        },
        **prov,
    }
    (out_dir / "camera_extrinsics.json").write_text(
        json.dumps({"nominal": nom.to_dict(), "contract": contract}, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    # Minimal fault matrix for wrist injections
    fault = [r for r in samples if r["scenario"].startswith(("dx_", "dz_", "yaw_"))]
    (out_dir / "camera_fault_matrix.csv").write_text(
        "scenario,nominal_effective_translation_delta_m,"
        "renderer_tf_translation_residual_m,result_semantics\n"
        + "\n".join(
            f"{r['scenario']},{r['nominal_effective_translation_delta_m']},"
            f"{r['renderer_tf_translation_residual_m']},REPORT_ONLY"
            for r in fault
        )
        + "\n",
        encoding="utf-8",
    )

    manifest = {
        "stage": "3C",
        "camera": "wrist_camera",
        "selected_candidate_id": selected_candidate_id,
        "pose_class": "DESIGN_NOMINAL",
        "physical": "NOT_RUN/UNAVAILABLE",
        "result_semantics": ResultSemantics.REPORT_ONLY.value,
        "eye_in_hand": contract["eye_in_hand"],
        "artifacts": {
            "run_manifest": "run_manifest.json",
            "camera_extrinsics": "camera_extrinsics.json",
            "camera_samples": "camera_samples.csv",
            "camera_fault_matrix": "camera_fault_matrix.csv",
        },
        **prov,
    }
    write_run_manifest(out_dir / "run_manifest.json", manifest)
    return manifest


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out-dir", type=Path, default=Path("evidence/camera_stage3_wrist"))
    p.add_argument("--candidate-id", type=str, default=None)
    p.add_argument("--no-xml-freeze", action="store_true")
    p.add_argument("--seed", type=int, default=20260814)
    args = p.parse_args(argv)
    m = run_stage3c(
        args.out_dir,
        selected_candidate_id=args.candidate_id,
        apply_xml_freeze=not args.no_xml_freeze,
        seed=args.seed,
    )
    print(json.dumps(m, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
