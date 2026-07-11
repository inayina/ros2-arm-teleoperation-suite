"""Tests for camera_bridge target-object joint mapping."""

from camera_bridge.object_sync import MANIPULABLE_OBJECTS, object_joint_name


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
