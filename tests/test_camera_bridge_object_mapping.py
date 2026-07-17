"""Tests for camera_bridge target-object joint mapping."""

from camera_bridge.object_sync import MANIPULABLE_OBJECTS, object_joint_name
from pathlib import Path
import xml.etree.ElementTree as ET


def test_object_joint_name_matches_mujoco_sim_convention() -> None:
    assert object_joint_name("object_red_box") == "red_box_joint"
    assert object_joint_name("object_blue_cylinder") == "blue_cylinder_joint"
    assert object_joint_name("object_green_sphere") == "green_sphere_joint"


def test_manipulable_objects_cover_sorting_targets() -> None:
    assert MANIPULABLE_OBJECTS == (
        "object_red_box",
        "object_blue_cylinder",
        "object_green_sphere",
    )


def test_scene_camera_is_fixed_in_worldbody() -> None:
    root = ET.parse(Path("config/models/franka_panda.xml")).getroot()
    worldbody = root.find("worldbody")
    assert worldbody is not None
    assert worldbody.find("./camera[@name='scene_camera']") is not None
    nested_scene = worldbody.findall(".//body//camera[@name='scene_camera']")
    assert nested_scene == []


def test_camera_parameter_poll_is_async() -> None:
    source = Path(
        "src/camera_bridge/camera_bridge/camera_bridge_node.py"
    ).read_text(encoding="utf-8")
    assert "call_async(request)" in source
    assert "_get_params_client.call(request)" not in source
