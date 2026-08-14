# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Wrist RGB acceptance: frozen XML pose, actual MuJoCo pixels (REPORT_ONLY).

Does not merge sim/camera models, does not change control law, does not claim
hand-eye or PASS. Live ROS is optional; this CLI uses the same VirtualCamera
path as camera_bridge.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from teleop_diagnostics.camera_contract import project_point_to_camera
from teleop_diagnostics.report import provenance_fields, write_run_manifest
from teleop_diagnostics.stage3b_cli import SCENARIOS, _object_xy_default
from teleop_diagnostics.wrist_pose_candidates import (
    WristPoseCandidate,
    wrist_pose_candidates_outside_palm,
)
from teleop_diagnostics.wrist_rgb import MIN_RED_PIXELS, red_pixel_stats, sample_rgb_at

FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")
MAX_GRIPPER_OPENING_M = 0.04
WIDTH, HEIGHT = 320, 240

# Open for approach; partly closed at grasp/lift so fingers are in frame
# without fully swallowing the cube.
GRIPPER_OPENING = {
    "pregrasp": 1.0,
    "approach": 1.0,
    "grasp": 0.40,
    "lift": 0.40,
}


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _hide_hand_visuals(model, mujoco) -> list[str]:
    """Zero alpha on panda_hand visual meshes (diagnostic; does not write XML)."""
    hidden = []
    for gid in range(model.ngeom):
        mesh_id = int(model.geom_dataid[gid])
        mesh_name = ""
        if mesh_id >= 0:
            mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id) or ""
        if mesh_name.startswith("hand_"):
            model.geom_rgba[gid, 3] = 0.0
            hidden.append(mesh_name)
    return hidden


def _set_arm_and_gripper(model, data, mujoco, q, opening: float) -> None:
    for i, name in enumerate([f"panda_joint{j}" for j in range(1, 8)]):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = float(q[i])
    finger = float(np.clip(opening, 0.0, 1.0)) * MAX_GRIPPER_OPENING_M
    for name in FINGER_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = finger
    mujoco.mj_forward(model, data)


def run_wrist_rgb(out_dir: Path) -> dict:
    import mujoco
    from mujoco_sim.camera_extrinsics import extract_nominal_from_model, renderer_world_pose
    from mujoco_sim.virtual_camera import CameraModel, VirtualCamera

    repo = _repo_root()
    prov = provenance_fields(repo=repo)
    out_dir.mkdir(parents=True, exist_ok=True)
    png_dir = out_dir / "png"
    png_dir.mkdir(exist_ok=True)

    model_path = repo / "config/models/franka_panda.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    state = extract_nominal_from_model(
        model, mujoco, "wrist_camera", pose_class="DESIGN_NOMINAL"
    )
    fovy = float(state.fovy_deg if state.fovy_deg is not None else 70.0)
    target = _object_xy_default()

    cam = CameraModel(
        name="wrist_camera",
        width=WIDTH,
        height=HEIGHT,
        fovy_deg=fovy,
        frame_id="wrist_camera_optical_frame",
    )
    try:
        vc = VirtualCamera(mujoco, model, cam)
        render_status = "AVAILABLE"
        render_error = ""
    except Exception as exc:
        vc = None
        render_status = "UNAVAILABLE"
        render_error = str(exc)

    rows = []
    pngs = []
    for scen_name, q in SCENARIOS.items():
        opening = GRIPPER_OPENING[scen_name]
        _set_arm_and_gripper(model, data, mujoco, q, opening)
        T_cam = renderer_world_pose(model, data, mujoco, "wrist_camera")
        proj = project_point_to_camera(
            T_cam, target, width=WIDTH, height=HEIGHT, fovy_deg=fovy
        )
        row = {
            "scenario": scen_name,
            "gripper_opening": opening,
            "gt_target_visible": bool(proj["target_visible"]),
            "gt_center_x_norm": proj["target_center_x_norm"],
            "gt_center_y_norm": proj["target_center_y_norm"],
            "gt_depth_m": proj["depth_m"],
            "gt_border_margin": proj["border_margin"],
            "render_input_status": render_status,
            "red_pixel_count": None,
            "red_pixel_ratio": None,
            "rgb_target_visible": False,
            "proj_pixel_rgb": None,
            "png": "",
            "result_semantics": "REPORT_ONLY",
            "physical": "NOT_RUN/UNAVAILABLE",
            "pose_class": "DESIGN_NOMINAL",
        }
        if vc is None:
            row["result_semantics"] = "INSUFFICIENT_DATA"
            rows.append(row)
            continue
        rgb = vc.render_rgb(data)
        stats = red_pixel_stats(rgb)
        row.update(stats)
        if proj["target_center_x_norm"] is not None:
            u = float(proj["target_center_x_norm"]) * WIDTH
            v = float(proj["target_center_y_norm"]) * HEIGHT
            row["proj_pixel_rgb"] = sample_rgb_at(rgb, u, v)
        png_name = f"wrist_{scen_name}.png"
        try:
            from PIL import Image

            Image.fromarray(rgb).save(png_dir / png_name)
            row["png"] = f"png/{png_name}"
            pngs.append(row["png"])
        except Exception as exc:
            row["png"] = f"save_failed:{exc}"
        rows.append(row)

    hand_hidden = {"render_input_status": render_status}
    if vc is not None:
        hidden = _hide_hand_visuals(model, mujoco)
        _set_arm_and_gripper(
            model, data, mujoco, SCENARIOS["grasp"], GRIPPER_OPENING["grasp"]
        )
        rgb = vc.render_rgb(data)
        stats = red_pixel_stats(rgb)
        png_name = "wrist_grasp_hand_visuals_hidden.png"
        hidden_png = ""
        try:
            from PIL import Image

            Image.fromarray(rgb).save(png_dir / png_name)
            hidden_png = f"png/{png_name}"
            pngs.append(hidden_png)
        except Exception:
            hidden_png = ""
        hand_hidden = {
            "scenario": "grasp_hand_visuals_hidden",
            "geoms_hidden": hidden,
            "png": hidden_png,
            "note": "Diagnostic only: palm visual geom alpha=0. Not a launch config.",
            **stats,
        }

    frozen_rows = [r for r in rows if r["scenario"] in SCENARIOS]
    visible_rgb = sum(1 for r in frozen_rows if r.get("rgb_target_visible"))
    visible_gt = sum(1 for r in frozen_rows if r.get("gt_target_visible"))
    rendered = sum(
        1 for r in frozen_rows if r.get("render_input_status") == "AVAILABLE"
    )

    diagnostics = {
        "camera": "wrist_camera",
        "pose_class": "DESIGN_NOMINAL",
        "xml_pose": {
            "pos": list(state.nominal_translation),
            "quat_wxyz": list(state.nominal_quat_wxyz),
            "fovy_deg": fovy,
            "parent_frame": state.parent_frame,
        },
        "target_object": "object_red_box",
        "target_xyz": target.tolist(),
        "image_size": [WIDTH, HEIGHT],
        "min_red_pixels": MIN_RED_PIXELS,
        "scenarios": rows,
        "hand_visuals_hidden_diagnostic": hand_hidden,
        "counts": {
            "gt_visible": visible_gt,
            "rgb_visible": visible_rgb,
            "rendered": rendered,
            "n_scenarios": len(SCENARIOS),
        },
        "render_error": render_error,
        "not": [
            "PHYSICAL_CALIBRATED",
            "hand-eye",
            "PASS",
            "live ROS bag",
            "merged sim/camera models",
        ],
        "physical": "NOT_RUN/UNAVAILABLE",
        "result_semantics": "REPORT_ONLY",
        **prov,
    }
    (out_dir / "wrist_rgb_diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    fields = [
        "scenario",
        "gripper_opening",
        "gt_target_visible",
        "gt_center_x_norm",
        "gt_center_y_norm",
        "gt_depth_m",
        "gt_border_margin",
        "red_pixel_count",
        "red_pixel_ratio",
        "rgb_target_visible",
        "proj_pixel_rgb",
        "render_input_status",
        "png",
        "result_semantics",
        "physical",
    ]
    with (out_dir / "wrist_rgb_metrics.csv").open("w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in fields})

    exit_gate = {
        "render_available": rendered == len(SCENARIOS),
        "gt_visible_all_four": visible_gt == len(SCENARIOS),
        "rgb_visible_all_four": visible_rgb == len(SCENARIOS),
        "launch_default_changed": False,
        "physical": "NOT_RUN/UNAVAILABLE",
        "result_semantics": "REPORT_ONLY",
        "live_ros": False,
        "launch_default_not_modified_by_this_cli": True,
    }
    manifest = {
        "stage": "wrist_rgb",
        "physical": "NOT_RUN/UNAVAILABLE",
        "result_semantics": "REPORT_ONLY",
        "control_law_modified": False,
        "models_merged": False,
        "exit_gate_observations": exit_gate,
        "artifacts": {
            "run_manifest": "run_manifest.json",
            "wrist_rgb_diagnostics": "wrist_rgb_diagnostics.json",
            "wrist_rgb_metrics": "wrist_rgb_metrics.csv",
            "png": pngs,
        },
        **prov,
        **exit_gate,
    }
    write_run_manifest(out_dir / "run_manifest.json", manifest)
    return manifest


def _score_candidate(
    *,
    mujoco,
    model,
    data,
    vc,
    cand: WristPoseCandidate,
    target,
    png_dir: Path,
) -> dict:
    from mujoco_sim.camera_extrinsics import (
        CameraExtrinsicState,
        apply_state_to_model,
        renderer_world_pose,
    )

    state = CameraExtrinsicState(
        camera_name="wrist_camera",
        parent_frame="panda_hand",
        nominal_translation=list(cand.translation_xyz),
        nominal_quat_wxyz=cand.quat_wxyz(),
        provenance=f"rgb_tune:{cand.candidate_id}",
        pose_class="CANDIDATE_POSE",
        fovy_deg=cand.fovy_deg,
    )
    apply_state_to_model(model, mujoco, state)
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_camera")
    model.cam_fovy[cid] = cand.fovy_deg

    scen_rows = []
    rgb_n = 0
    gt_n = 0
    red_sum = 0
    for scen_name, q in SCENARIOS.items():
        _set_arm_and_gripper(model, data, mujoco, q, GRIPPER_OPENING[scen_name])
        T_cam = renderer_world_pose(model, data, mujoco, "wrist_camera")
        proj = project_point_to_camera(
            T_cam, target, width=WIDTH, height=HEIGHT, fovy_deg=cand.fovy_deg
        )
        rgb = vc.render_rgb(data)
        stats = red_pixel_stats(rgb)
        if proj["target_visible"]:
            gt_n += 1
        if stats["rgb_target_visible"]:
            rgb_n += 1
        red_sum += int(stats["red_pixel_count"])
        png_rel = f"png/{cand.candidate_id}_{scen_name}.png"
        try:
            from PIL import Image

            Image.fromarray(rgb).save(png_dir / f"{cand.candidate_id}_{scen_name}.png")
        except Exception:
            png_rel = ""
        scen_rows.append(
            {
                "candidate_id": cand.candidate_id,
                "scenario": scen_name,
                "gt_target_visible": bool(proj["target_visible"]),
                "gt_depth_m": proj["depth_m"],
                "png": png_rel,
                **stats,
            }
        )
    return {
        "candidate_id": cand.candidate_id,
        "translation_xyz": list(cand.translation_xyz),
        "fovy_deg": cand.fovy_deg,
        "rationale": cand.rationale,
        "rgb_visible": rgb_n,
        "gt_visible": gt_n,
        "red_pixel_sum": red_sum,
        "scenarios": scen_rows,
    }


def run_wrist_rgb_tune(out_dir: Path) -> dict:
    """Score B (buried baseline) + outside-palm candidates on RGB. Does not freeze XML."""
    import mujoco
    from mujoco_sim.virtual_camera import CameraModel, VirtualCamera
    from teleop_diagnostics.wrist_pose_candidates import wrist_pose_candidates

    repo = _repo_root()
    prov = provenance_fields(repo=repo)
    out_dir.mkdir(parents=True, exist_ok=True)
    png_dir = out_dir / "png"
    png_dir.mkdir(exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(repo / "config/models/franka_panda.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    target = _object_xy_default()
    cam = CameraModel(
        name="wrist_camera",
        width=WIDTH,
        height=HEIGHT,
        fovy_deg=70.0,
        frame_id="wrist_camera_optical_frame",
    )
    vc = VirtualCamera(mujoco, model, cam)

    baseline_b = next(c for c in wrist_pose_candidates() if c.candidate_id == "B_look_fingers")
    cands = [baseline_b] + wrist_pose_candidates_outside_palm()
    scores = []
    for cand in cands:
        scores.append(
            _score_candidate(
                mujoco=mujoco,
                model=model,
                data=data,
                vc=vc,
                cand=cand,
                target=target,
                png_dir=png_dir,
            )
        )

    ranked = sorted(
        scores,
        key=lambda s: (s["rgb_visible"], s["red_pixel_sum"], s["gt_visible"]),
        reverse=True,
    )
    selected = ranked[0]
    eligible = selected["rgb_visible"] == len(SCENARIOS)
    (out_dir / "rgb_tune_scores.json").write_text(
        json.dumps(
            {
                "selection_metric": "rgb_visible then red_pixel_sum then gt_visible",
                "not": ["PHYSICAL_CALIBRATED", "hand-eye", "GT-only", "looks better"],
                "ranked": ranked,
                "selected_candidate_id": selected["candidate_id"],
                "rgb_four_of_four": eligible,
                "xml_frozen": False,
                **prov,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    fields = [
        "candidate_id",
        "scenario",
        "gt_target_visible",
        "gt_depth_m",
        "red_pixel_count",
        "red_pixel_ratio",
        "rgb_target_visible",
        "png",
    ]
    with (out_dir / "rgb_tune_metrics.csv").open("w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for s in scores:
            for row in s["scenarios"]:
                w.writerow({k: row.get(k, "") for k in fields})

    manifest = {
        "stage": "wrist_rgb_tune",
        "selected_candidate_id": selected["candidate_id"],
        "rgb_four_of_four": eligible,
        "xml_frozen": False,
        "ranked_ids": [s["candidate_id"] for s in ranked],
        "scores_summary": [
            {
                "id": s["candidate_id"],
                "rgb_visible": s["rgb_visible"],
                "gt_visible": s["gt_visible"],
                "red_pixel_sum": s["red_pixel_sum"],
            }
            for s in ranked
        ],
        "physical": "NOT_RUN/UNAVAILABLE",
        "result_semantics": "REPORT_ONLY",
        **prov,
    }
    write_run_manifest(out_dir / "run_manifest.json", manifest)
    return manifest


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--out-dir",
        type=Path,
        default=Path("evidence/wrist_rgb_acceptance"),
    )
    p.add_argument(
        "--tune-outside-palm",
        action="store_true",
        help="Score outside-palm candidates on RGB; do not freeze XML.",
    )
    args = p.parse_args(argv)
    if args.tune_outside_palm:
        result = run_wrist_rgb_tune(args.out_dir)
    else:
        result = run_wrist_rgb(args.out_dir)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
