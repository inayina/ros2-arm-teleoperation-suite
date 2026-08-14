# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage 3 camera extrinsic contract tests."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]


def test_camera_extrinsic_nominal_from_xml():
    import mujoco
    from mujoco_sim.camera_extrinsics import CameraExtrinsicAuthority, CameraExtrinsicError

    model = mujoco.MjModel.from_xml_path(str(REPO / "config/models/franka_panda.xml"))
    auth = CameraExtrinsicAuthority(model, mujoco)
    scene = auth.nominal("scene_camera")
    assert scene.parent_frame == "world"
    assert scene.optical_frame == "scene_camera_optical_frame"
    assert scene.link_frame == "scene_camera_link"
    assert np.allclose(scene.nominal_translation, [1.45, -0.55, 1.25])
    with pytest.raises(CameraExtrinsicError):
        auth.nominal("missing_cam")


def test_scene_renderer_tf_nominal_consistency():
    import mujoco
    from mujoco_sim.camera_extrinsics import (
        CameraExtrinsicAuthority,
        renderer_world_pose,
    )

    model = mujoco.MjModel.from_xml_path(str(REPO / "config/models/franka_panda.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    auth = CameraExtrinsicAuthority(model, mujoco)
    state = auth.reset_nominal("scene_camera", write_model=True)
    mujoco.mj_forward(model, data)
    T_r = renderer_world_pose(model, data, mujoco, "scene_camera")
    T_tf = state.effective_matrix()
    assert np.linalg.norm(T_r[:3, 3] - T_tf[:3, 3]) < 1e-12
    assert np.linalg.norm(T_r[:3, :3] - T_tf[:3, :3]) < 1e-12


def test_scene_injection_30mm_and_renderer_tf_stay_matched():
    import mujoco
    from mujoco_sim.camera_extrinsics import CameraExtrinsicAuthority, renderer_world_pose

    model = mujoco.MjModel.from_xml_path(str(REPO / "config/models/franka_panda.xml"))
    data = mujoco.MjData(model)
    auth = CameraExtrinsicAuthority(model, mujoco)
    nom = auth.reset_nominal("scene_camera")
    state = auth.inject_local(
        "scene_camera", translation_m=[0.03, 0.0, 0.0], provenance="test"
    )
    mujoco.mj_forward(model, data)
    delta = np.linalg.norm(
        np.asarray(state.effective_translation()) - np.asarray(nom.nominal_translation)
    )
    assert abs(delta - 0.03) < 1e-12
    T_r = renderer_world_pose(model, data, mujoco, "scene_camera")
    assert np.linalg.norm(T_r[:3, 3] - state.effective_matrix()[:3, 3]) < 1e-12


def test_scene_yaw_injection_2deg():
    import mujoco
    from mujoco_sim.camera_extrinsics import CameraExtrinsicAuthority
    from teleop_diagnostics.types import rotation_geodesic

    model = mujoco.MjModel.from_xml_path(str(REPO / "config/models/franka_panda.xml"))
    auth = CameraExtrinsicAuthority(model, mujoco)
    nom = auth.reset_nominal("scene_camera")
    state = auth.inject_local(
        "scene_camera", rpy_rad=[0.0, 0.0, math.radians(2.0)], provenance="test"
    )
    geod = rotation_geodesic(nom.nominal_matrix()[:3, :3], state.effective_matrix()[:3, :3])
    assert abs(geod - math.radians(2.0)) < 1e-9


def test_randomization_seed_reproducible_across_authorities():
    import mujoco
    from mujoco_sim.camera_extrinsics import (
        CameraExtrinsicAuthority,
        apply_config_randomization,
    )

    cfg = {"scene_camera": {"pos_noise": [-0.05, 0.05], "rot_noise": [-5.0, 5.0]}}
    m1 = mujoco.MjModel.from_xml_path(str(REPO / "config/models/franka_panda.xml"))
    m2 = mujoco.MjModel.from_xml_path(str(REPO / "config/models/franka_panda.xml"))
    a = apply_config_randomization(
        CameraExtrinsicAuthority(m1, mujoco).nominal("scene_camera"), cfg, seed=7, draw_index=0
    )
    b = apply_config_randomization(
        CameraExtrinsicAuthority(m2, mujoco).nominal("scene_camera"), cfg, seed=7, draw_index=0
    )
    assert np.allclose(a.effective_matrix(), b.effective_matrix())


def test_header_optical_frame_matches_contract():
    from mujoco_sim.camera_extrinsics import optical_frame_name

    assert optical_frame_name("scene_camera") == "scene_camera_optical_frame"
    assert optical_frame_name("wrist_camera") == "wrist_camera_optical_frame"


def test_mujoco_to_optical_convention():
    from mujoco_sim.camera_extrinsics import R_MUJOCO_TO_OPTICAL

    # Optical +Z (forward) = MuJoCo −Z (look direction)
    assert np.allclose(R_MUJOCO_TO_OPTICAL @ np.array([0, 0, 1.0]), [0, 0, -1.0])
    # Optical +Y (down) = MuJoCo −Y
    assert np.allclose(R_MUJOCO_TO_OPTICAL @ np.array([0, 1.0, 0]), [0, -1.0, 0])


def test_wrist_candidates_deterministic_and_parseable():
    from teleop_diagnostics.wrist_pose_candidates import wrist_pose_candidates

    a = wrist_pose_candidates(seed=1)
    b = wrist_pose_candidates(seed=999)
    assert [c.candidate_id for c in a] == [c.candidate_id for c in b]
    assert len(a) >= 4
    for c in a:
        assert len(c.quat_wxyz()) == 4
        assert abs(np.linalg.norm(c.quat_wxyz()) - 1.0) < 1e-9
        assert c.pose_class == "CANDIDATE_POSE"


def test_invalid_wrist_pose_rejected():
    from mujoco_sim.camera_extrinsics import CameraExtrinsicError, make_transform

    with pytest.raises(CameraExtrinsicError):
        make_transform([0, 0, 0], quat_wxyz=[0, 0, 0, 0])


def test_target_projection_valid():
    from teleop_diagnostics.camera_contract import project_point_to_camera

    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, 1.0]  # camera at z=1 looking −Z toward origin
    # Point in front of camera at origin
    out = project_point_to_camera(T, [0.0, 0.0, 0.0], width=320, height=240, fovy_deg=70.0)
    assert out["depth_m"] == pytest.approx(1.0)
    assert out["target_visible"] is True


def test_fovy_camera_info_matches_xml():
    import mujoco
    from mujoco_sim.virtual_camera import CameraModel

    model = mujoco.MjModel.from_xml_path(str(REPO / "config/models/franka_panda.xml"))
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "scene_camera")
    fovy = float(model.cam_fovy[cid])
    cam = CameraModel("scene_camera", 640, 480, fovy, "scene_camera_optical_frame")
    assert cam.fovy_deg == fovy
    assert len(cam.intrinsic_matrix) == 9
    assert len(cam.projection_matrix) == 12


def test_stage3a_cli_smoke(tmp_path):
    from teleop_diagnostics.stage3a_cli import run_stage3a

    manifest = run_stage3a(tmp_path / "scene")
    assert manifest["stage"] == "3A"
    assert (tmp_path / "scene" / "camera_samples.csv").is_file()
    assert manifest["exit_gate_observations"]["renderer_tf_consistent"] is True
    assert manifest["exit_gate_observations"]["missing_camera_rejected"] is True


def test_stage3b_cli_records_selection(tmp_path):
    from teleop_diagnostics.stage3b_cli import run_stage3b

    manifest = run_stage3b(tmp_path / "tune", write_png=False)
    assert manifest["stage"] == "3B"
    assert "selected_candidate_id" in manifest
    assert (tmp_path / "tune" / "pose_candidates.json").is_file()
    data = (tmp_path / "tune" / "pose_candidates.json").read_text()
    assert "PHYSICAL_CALIBRATED" in data
    assert "DESIGN_NOMINAL" in data


def test_stage3c_eye_in_hand_without_xml_write(tmp_path):
    from teleop_diagnostics.stage3c_cli import run_stage3c

    # Don't mutate repo XML during unit test — evaluate relative contract on current model.
    manifest = run_stage3c(
        tmp_path / "wrist",
        selected_candidate_id="B_look_fingers",
        apply_xml_freeze=False,
    )
    assert manifest["stage"] == "3C"
    assert manifest["eye_in_hand"]["hand_relative_stable"] is True
    assert manifest["eye_in_hand"]["world_pose_changes_with_motion"] is True


def test_camera_bridge_publishes_tf_and_extrinsic_hooks():
    source = (REPO / "src/camera_bridge/camera_bridge/camera_bridge_node.py").read_text()
    assert "StaticTransformBroadcaster" in source
    assert "/sim/camera_extrinsic" in source
    assert "optical_frame_name" in source
    assert "apply_state_to_model" in source


def test_domain_randomizer_exposes_last_camera_states():
    from unittest.mock import MagicMock

    import numpy as np
    from mujoco_sim.domain_randomizer import DomainRandomizer

    class _Mj:
        class mjtObj:
            mjOBJ_BODY = 1
            mjOBJ_GEOM = 2
            mjOBJ_JOINT = 3
            mjOBJ_CAMERA = 4
            mjOBJ_LIGHT = 5

        def mj_name2id(self, model, obj_type, name):
            return {"scene_camera": 0, "object_red_box": -1}.get(name, -1)

        def mj_id2name(self, model, obj_type, obj_id):
            return "world"

    class _Model:
        def __init__(self):
            self.cam_pos = [np.array([1.0, 0.0, 1.0], dtype=float)]
            self.cam_quat = [np.array([1.0, 0.0, 0.0, 0.0], dtype=float)]
            self.cam_bodyid = [0]
            self.cam_fovy = [45.0]
            self.body_mass = []
            self.geom_friction = []
            self.jnt_qposadr = []
            self.jnt_dofadr = []
            self.light_diffuse = []

    class _Data:
        def __init__(self):
            self.qpos = np.zeros(1)
            self.qvel = np.zeros(1)

    cfg = {
        "domain_randomization": {
            "enabled": True,
            "seed": 3,
            "camera": {
                "scene_camera": {"pos_noise": [-0.01, 0.01], "rot_noise": [0.0, 0.0]}
            },
            "object": {},
        }
    }
    rnd = DomainRandomizer(cfg)
    mj = _Mj()
    mj.mj_forward = MagicMock()
    rnd.apply(_Model(), _Data(), mj)
    assert "scene_camera" in rnd.last_camera_states
