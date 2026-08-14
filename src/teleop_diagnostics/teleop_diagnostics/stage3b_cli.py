# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage 3B: wrist camera nominal pose tuning (DESIGN placement, not hand-eye)."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from teleop_diagnostics.camera_contract import project_point_to_camera
from teleop_diagnostics.poses import READY_Q
from teleop_diagnostics.report import provenance_fields, write_run_manifest
from teleop_diagnostics.wrist_pose_candidates import wrist_pose_candidates


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


# Approximate task postures (joint space) for projection metrics — not teleop trajectories.
SCENARIOS = {
    "pregrasp": READY_Q,
    "approach": (
        0.0,
        -0.55,
        0.0,
        -2.2,
        0.0,
        1.65,
        0.785,
    ),
    "grasp": (
        0.0,
        -0.40,
        0.0,
        -2.05,
        0.0,
        1.75,
        0.785,
    ),
    "lift": (
        0.0,
        -0.70,
        0.0,
        -2.15,
        0.0,
        1.55,
        0.785,
    ),
}


def _set_q(model, data, mujoco, q) -> None:
    for i, name in enumerate([f"panda_joint{j}" for j in range(1, 8)]):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = float(q[i])
    mujoco.mj_forward(model, data)


def _object_xy_default() -> np.ndarray:
    return np.array([0.35, -0.07, 0.025], dtype=float)


def run_stage3b(out_dir: Path, *, seed: int = 20260814, write_png: bool = False) -> dict:
    import mujoco
    from mujoco_sim.camera_extrinsics import apply_state_to_model, extract_nominal_from_model
    from mujoco_sim.camera_extrinsics import CameraExtrinsicState, renderer_world_pose
    from mujoco_sim.virtual_camera import CameraModel, VirtualCamera

    repo = _repo_root()
    prov = provenance_fields(repo=repo)
    prov["implementation_commit"] = prov["evidence_generation_commit"]
    out_dir.mkdir(parents=True, exist_ok=True)

    model_path = repo / "config/models/franka_panda.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    candidates = wrist_pose_candidates(seed=seed)
    target = _object_xy_default()
    width, height = 320, 240

    metrics_rows = []
    scores = {}

    for cand in candidates:
        state = CameraExtrinsicState(
            camera_name="wrist_camera",
            parent_frame="panda_hand",
            nominal_translation=list(cand.translation_xyz),
            nominal_quat_wxyz=cand.quat_wxyz(),
            provenance=f"stage3b_candidate:{cand.candidate_id}",
            pose_class="CANDIDATE_POSE",
            fovy_deg=cand.fovy_deg,
        )
        apply_state_to_model(model, mujoco, state)
        # Keep XML fovy for evaluation of candidate FOV via CameraModel, but
        # MuJoCo render uses model.cam_fovy — update it for this candidate.
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_camera")
        model.cam_fovy[cid] = cand.fovy_deg

        vis_count = 0
        margin_sum = 0.0
        depth_ok = 0
        for scen_name, q in SCENARIOS.items():
            _set_q(model, data, mujoco, q)
            T_cam = renderer_world_pose(model, data, mujoco, "wrist_camera")
            proj = project_point_to_camera(
                T_cam,
                target,
                width=width,
                height=height,
                fovy_deg=cand.fovy_deg,
            )
            if proj["target_visible"]:
                vis_count += 1
                margin_sum += float(proj["border_margin"])
            depth = proj["depth_m"]
            if depth is not None and 0.08 <= depth <= 0.55:
                depth_ok += 1
            metrics_rows.append(
                {
                    "candidate_id": cand.candidate_id,
                    "pose": json.dumps(
                        {"xyz": cand.translation_xyz, "quat_wxyz": cand.quat_wxyz()}
                    ),
                    "fovy": cand.fovy_deg,
                    "scenario": scen_name,
                    "target_visible": proj["target_visible"],
                    "target_center_x_norm": proj["target_center_x_norm"],
                    "target_center_y_norm": proj["target_center_y_norm"],
                    "target_bbox_area_ratio": "",
                    "border_margin": proj["border_margin"],
                    "occlusion_ratio_if_available": "",
                    "depth_m": depth,
                }
            )
            if write_png:
                try:
                    cam = CameraModel(
                        name="wrist_camera",
                        width=width,
                        height=height,
                        fovy_deg=cand.fovy_deg,
                        frame_id="wrist_camera_optical_frame",
                    )
                    vc = VirtualCamera(mujoco, model, cam)
                    rgb = vc.render_rgb(data)
                    from PIL import Image

                    Image.fromarray(rgb).save(
                        out_dir / f"{cand.candidate_id}_{scen_name}.png"
                    )
                except Exception:
                    pass

        scores[cand.candidate_id] = {
            "visible_scenarios": vis_count,
            "mean_border_margin_when_visible": (
                margin_sum / vis_count if vis_count else -1.0
            ),
            "depth_in_range_count": depth_ok,
            "rationale": cand.rationale,
        }

    # Selection: maximize visibility, then margin, then depth-in-range.
    # Explicitly not "looks better".
    def sort_key(cid: str):
        s = scores[cid]
        return (
            s["visible_scenarios"],
            s["mean_border_margin_when_visible"],
            s["depth_in_range_count"],
        )

    ranked = sorted(scores.keys(), key=sort_key, reverse=True)
    selected_id = ranked[0]
    selected = next(c for c in candidates if c.candidate_id == selected_id)

    # Prefer C if it ties or beats B on visibility — documented trade-off.
    selection_rationale = {
        "selected_candidate_id": selected_id,
        "ranking": ranked,
        "scores": scores,
        "why": (
            f"{selected_id} selected because it maximizes "
            f"visible_scenarios={scores[selected_id]['visible_scenarios']}/4, "
            f"mean_border_margin_when_visible="
            f"{scores[selected_id]['mean_border_margin_when_visible']:.3f}, "
            f"depth_in_range_count={scores[selected_id]['depth_in_range_count']}/4 "
            f"on GT object projection (no segmentation). "
            "This freezes a DESIGN_NOMINAL simulation mount, not PHYSICAL_CALIBRATED."
        ),
        "pose_class_after_freeze": "DESIGN_NOMINAL",
        "physical": "NOT_RUN/UNAVAILABLE",
        "evidence_class": "MODEL / SIM_GT supported design choice",
        "not": ["PHYSICAL_CALIBRATED", "hand-eye", "looks better"],
    }

    (out_dir / "pose_candidates.json").write_text(
        json.dumps(
            {
                "seed": seed,
                "parent_frame": "panda_hand",
                "target_object_xy_default": target.tolist(),
                "scenarios": {k: list(v) for k, v in SCENARIOS.items()},
                "candidates": [c.to_dict() for c in candidates],
                "selection": selection_rationale,
                **prov,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    with (out_dir / "pose_metrics.csv").open("w", encoding="utf-8", newline="") as fh:
        fields = [
            "candidate_id",
            "pose",
            "fovy",
            "scenario",
            "target_visible",
            "target_center_x_norm",
            "target_center_y_norm",
            "target_bbox_area_ratio",
            "border_margin",
            "occlusion_ratio_if_available",
            "depth_m",
        ]
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in metrics_rows:
            w.writerow(r)

    # Emit recommended XML snippet for Stage 3C freeze (applied by stage3c).
    R = np.asarray(selected.rotation_matrix)
    xyaxes = list(R[:, 0]) + list(R[:, 1])
    xml_snippet = (
        f'<camera name="wrist_camera" pos="'
        f'{selected.translation_xyz[0]} {selected.translation_xyz[1]} {selected.translation_xyz[2]}" '
        f'xyaxes="{" ".join(f"{v:.6g}" for v in xyaxes)}" '
        f'fovy="{selected.fovy_deg}" />'
    )
    (out_dir / "selected_wrist_camera.xml.snippet").write_text(xml_snippet + "\n", encoding="utf-8")

    manifest = {
        "stage": "3B",
        "selected_candidate_id": selected_id,
        "pose_class": "DESIGN_NOMINAL (after freeze) / was CANDIDATE_POSE",
        "physical": "NOT_RUN/UNAVAILABLE",
        "xml_snippet": xml_snippet,
        "artifacts": {
            "pose_candidates": "pose_candidates.json",
            "pose_metrics": "pose_metrics.csv",
            "selected_snippet": "selected_wrist_camera.xml.snippet",
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
        default=Path("evidence/wrist_camera_pose_tuning"),
    )
    p.add_argument("--seed", type=int, default=20260814)
    p.add_argument("--write-png", action="store_true")
    args = p.parse_args(argv)
    print(json.dumps(run_stage3b(args.out_dir, seed=args.seed, write_png=args.write_png), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
