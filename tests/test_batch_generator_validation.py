import math

from geometry_msgs.msg import PoseStamped

from synth_data_gen.batch_generator import BatchGenerator


def _pose(x, y, z):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


def _validator(final_pose, mode="place"):
    node = BatchGenerator.__new__(BatchGenerator)
    node.validation_mode = mode
    node.lift_success_delta = 0.02
    node.bin_xy_tolerance = 0.14
    node._trial_initial_object_z = 0.025
    node._trial_initial_object_xyz = (0.35, -0.05, 0.025)
    node._trial_max_object_z = 0.12
    node._trial_max_ee_tracking_error = 0.0
    node._trial_gripper_was_closed = True
    node.require_gripper_close = True
    node.gripper_close_max = 0.12
    node.ee_tracking_tolerance_m = 0.08
    node.motion_mode = "pose"
    node._object_workspace_xy = (0.20, 0.60, -0.25, 0.25)
    node._object_workspace_z = (0.00, 0.20)
    node._wait_for_object_pose = lambda timeout=1.0: final_pose
    node.get_logger = lambda: type("L", (), {"warn": lambda *a, **k: None})()
    return node


def test_object_xyz_helper_handles_missing_pose():
    assert BatchGenerator._object_xyz(None) is None
    assert BatchGenerator._object_xyz(_pose(0.1, 0.2, 0.3)) == (0.1, 0.2, 0.3)


def test_place_validation_requires_lift_and_bin_proximity():
    node = _validator(_pose(0.41, -0.21, 0.05))
    result = node._validate_episode(
        target_obj="object_red_box",
        bin_x=0.4,
        bin_y=-0.2,
        target_set=True,
        language_set=True,
        gate_set=True,
        reset_ok=True,
    )
    assert result["success"] is True
    assert "place validation passed" in result["reason"]


def test_place_validation_rejects_wrong_bin():
    node = _validator(_pose(0.4, 0.2, 0.05))
    result = node._validate_episode(
        target_obj="object_red_box",
        bin_x=0.4,
        bin_y=-0.2,
        target_set=True,
        language_set=True,
        gate_set=True,
        reset_ok=True,
    )
    assert result["success"] is False
    assert "place validation failed" in result["reason"]


def test_lift_validation_accepts_without_bin_proximity():
    node = _validator(_pose(0.4, 0.2, 0.05), mode="lift")
    node._trial_max_ee_tracking_error = 0.01
    node.ee_tracking_tolerance_m = 0.08
    result = node._validate_episode(
        target_obj="object_red_box",
        bin_x=0.4,
        bin_y=-0.2,
        target_set=True,
        language_set=True,
        gate_set=True,
        reset_ok=True,
    )
    assert result["success"] is True
    assert math.isfinite(node._trial_max_object_z)


def test_pick_z_offset_uses_shape_default_when_param_is_default():
    node = BatchGenerator.__new__(BatchGenerator)
    node.pick_height_offset = 0.05
    node._pick_height_offset_default = 0.05
    node._pick_offset_by_shape = {"sphere": 0.008, "cylinder": 0.010, "box": 0.012}
    assert node._pick_z_offset("object_red_box") == 0.012
    assert node._pick_z_offset("object_blue_cylinder") == 0.010
    assert node._pick_z_offset("object_green_sphere") == 0.008


def test_pick_z_offset_honors_explicit_override():
    node = BatchGenerator.__new__(BatchGenerator)
    node.pick_height_offset = 0.012
    node._pick_height_offset_default = 0.05
    node._pick_offset_by_shape = {"sphere": 0.008, "cylinder": 0.010, "box": 0.018}
    assert node._pick_z_offset("object_red_box") == 0.012


def test_language_timeout_does_not_fail_lift_gate():
    node = _validator(_pose(0.41, -0.21, 0.05), mode="lift")
    result = node._validate_episode(
        target_obj="object_red_box",
        bin_x=0.4,
        bin_y=-0.2,
        target_set=True,
        language_set=False,
        gate_set=True,
        reset_ok=True,
        motion_ok=True,
    )
    assert result["success"] is True
    assert "lift validation passed" in result["reason"]


def test_out_of_workspace_initial_pose_rejected():
    node = _validator(_pose(0.41, -0.21, 0.05), mode="lift")
    node._trial_initial_object_xyz = (1.84, -0.03, 0.02)
    result = node._validate_episode(
        target_obj="object_green_sphere",
        bin_x=0.4,
        bin_y=-0.2,
        target_set=True,
        language_set=True,
        gate_set=True,
        reset_ok=True,
        motion_ok=True,
    )
    assert result["success"] is False
    assert "out of workspace" in result["reason"]


def test_validate_episode_rejects_failed_motion():
    node = _validator(_pose(0.41, -0.21, 0.05))
    result = node._validate_episode(
        target_obj="object_red_box",
        bin_x=0.4,
        bin_y=-0.2,
        target_set=True,
        language_set=True,
        gate_set=True,
        reset_ok=True,
        motion_ok=False,
    )
    assert result["success"] is False
    assert "motion phase did not converge" in result["reason"]


def test_validate_episode_rejects_excessive_ee_tracking_error():
    node = _validator(_pose(0.41, -0.21, 0.05))
    node.motion_mode = "twist"
    node._trial_max_ee_tracking_error = 0.10
    node.ee_tracking_tolerance_m = 0.08
    result = node._validate_episode(
        target_obj="object_red_box",
        bin_x=0.4,
        bin_y=-0.2,
        target_set=True,
        language_set=True,
        gate_set=True,
        reset_ok=True,
    )
    assert result["success"] is False
    assert "ee tracking error" in result["reason"]
