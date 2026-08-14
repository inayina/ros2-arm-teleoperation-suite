# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Independent camera model must copy object poses and light diffuse from physics."""

from __future__ import annotations

from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]


def test_scene_visual_copies_nontarget_pose_and_light_between_models():
    import mujoco
    from camera_bridge.object_sync import MANIPULABLE_OBJECTS, object_joint_name
    from mujoco_sim.scene_visual import (
        apply_lights,
        apply_object_poses,
        snapshot_scene_visual,
    )

    xml = str(REPO / "config/models/franka_panda.xml")
    phys = mujoco.MjModel.from_xml_path(xml)
    phys_data = mujoco.MjData(phys)
    rend = mujoco.MjModel.from_xml_path(xml)
    rend_data = mujoco.MjData(rend)
    mujoco.mj_forward(phys, phys_data)
    mujoco.mj_forward(rend, rend_data)

    jid = mujoco.mj_name2id(phys, mujoco.mjtObj.mjOBJ_JOINT, "blue_cylinder_joint")
    adr = int(phys.jnt_qposadr[jid])
    phys_data.qpos[adr: adr + 3] = [0.41, 0.12, 0.03]
    mujoco.mj_forward(phys, phys_data)

    lid = mujoco.mj_name2id(phys, mujoco.mjtObj.mjOBJ_LIGHT, "top")
    assert lid >= 0
    phys.light_diffuse[lid][:] = [0.2, 0.3, 0.4]

    snap = snapshot_scene_visual(phys, phys_data, mujoco)
    assert "object_blue_cylinder" in snap["objects"]
    assert "top" in snap["lights"]

    joints = {}
    for name in MANIPULABLE_OBJECTS:
        joint = object_joint_name(name)
        jj = mujoco.mj_name2id(rend, mujoco.mjtObj.mjOBJ_JOINT, joint)
        joints[name] = {
            "qposadr": int(rend.jnt_qposadr[jj]),
            "qveladr": int(rend.jnt_dofadr[jj]),
        }
    apply_object_poses(rend, rend_data, mujoco, snap["objects"], object_joints=joints)
    apply_lights(rend, mujoco, snap["lights"])
    mujoco.mj_forward(rend, rend_data)

    bid_p = mujoco.mj_name2id(phys, mujoco.mjtObj.mjOBJ_BODY, "object_blue_cylinder")
    bid_r = mujoco.mj_name2id(rend, mujoco.mjtObj.mjOBJ_BODY, "object_blue_cylinder")
    assert np.allclose(phys_data.xpos[bid_p], rend_data.xpos[bid_r], atol=1e-9)
    lid_r = mujoco.mj_name2id(rend, mujoco.mjtObj.mjOBJ_LIGHT, "top")
    assert np.allclose(phys.light_diffuse[lid], rend.light_diffuse[lid_r], atol=1e-12)


def test_light_name_key_aliases_xml_top():
    import mujoco
    from mujoco_sim.scene_visual import resolve_light_id

    model = mujoco.MjModel.from_xml_path(str(REPO / "config/models/franka_panda.xml"))
    assert resolve_light_id(model, mujoco, "top") >= 0
    assert resolve_light_id(model, mujoco, "key") == resolve_light_id(model, mujoco, "top")


def test_camera_bridge_subscribes_scene_visual():
    source = (
        REPO / "src/camera_bridge/camera_bridge/camera_bridge_node.py"
    ).read_text(encoding="utf-8")
    assert "SCENE_VISUAL_TOPIC" in source
    assert "_on_scene_visual" in source
    assert "_object_poses" in source


def test_mujoco_sim_publishes_scene_visual():
    source = (REPO / "src/mujoco_sim/mujoco_sim/mujoco_sim_node.py").read_text(
        encoding="utf-8"
    )
    assert "SCENE_VISUAL_TOPIC" in source
    assert "_publish_scene_visual" in source
    # Load runs before create_publisher; a missing-pub AttributeError used to
    # throw away a valid MjModel and leave cameras on XML rest with no object pose.
    load_fn = source.split("def _try_load_model", 1)[1].split("def _initial_positions", 1)[0]
    assert "_publish_camera_extrinsics" not in load_fn
    assert "_publish_scene_visual" not in load_fn
    assert 'getattr(self, "pub_cam_extrinsic_json"' in source
    assert 'getattr(self, "pub_scene_visual"' in source


def test_live_randomization_yaml_targets_xml_light_top():
    text = (REPO / "config/randomization.yaml").read_text(encoding="utf-8")
    assert "top:" in text.split("lighting:")[-1]


def test_object_pose_target_contract_unchanged():
    sim = (REPO / "src/mujoco_sim/mujoco_sim/mujoco_sim_node.py").read_text(
        encoding="utf-8"
    )
    bridge = (
        REPO / "src/camera_bridge/camera_bridge/camera_bridge_node.py"
    ).read_text(encoding="utf-8")
    assert 'PoseStamped, "/sim/object_pose"' in sim
    assert 'PoseStamped, "/sim/object_pose"' in bridge
    assert "_target_object_pose" in sim
