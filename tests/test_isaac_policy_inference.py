"""Pure tests for the bounded ACT-to-Isaac execution boundary."""

import math

from isaac_sim_adapter.policy_control import action_to_target_pose
from isaac_sim_adapter.policy_control import bound_ee_delta_gripper
from isaac_sim_adapter.policy_control import bound_gripper_command
from isaac_sim_adapter.policy_control import offset_pose_in_local_frame
from isaac_sim_adapter.policy_control import validate_panda_joint_positions
from isaac_sim_adapter.policy_inference_node import image_message_to_rgb
import numpy as np
import pytest
from sensor_msgs.msg import Image


def test_policy_action_rejects_bad_shape_and_nonfinite_values():
    with pytest.raises(ValueError, match='expected ee_delta_gripper'):
        bound_ee_delta_gripper(
            [0.0] * 6, max_translation_m=0.005, max_rotation_rad=0.03
        )
    with pytest.raises(ValueError, match='NaN or infinity'):
        bound_ee_delta_gripper(
            [0.0] * 6 + [math.nan],
            max_translation_m=0.005,
            max_rotation_rad=0.03,
        )


def test_policy_action_clamps_translation_rotation_and_gripper():
    result = bound_ee_delta_gripper(
        [0.2, -0.2, 0.001, 1.0, -1.0, 0.01, 2.0],
        max_translation_m=0.005,
        max_rotation_rad=0.03,
    )
    assert result.clipped is True
    assert result.values == pytest.approx([
        0.005, -0.005, 0.001, 0.03, -0.03, 0.01, 1.0
    ])


def test_gripper_boundary_rejects_nonfinite_and_clamps():
    assert bound_gripper_command(-0.2) == (0.0, True)
    assert bound_gripper_command(0.4) == (0.4, False)
    with pytest.raises(ValueError, match='NaN or infinity'):
        bound_gripper_command(math.nan)


def test_live_joint_state_must_be_inside_panda_hard_limits():
    ready = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    assert validate_panda_joint_positions(ready) == pytest.approx(ready)
    with pytest.raises(ValueError, match='panda_joint7 outside hard limits'):
        validate_panda_joint_positions([*ready[:6], 149.0])


def test_target_pose_uses_delta_times_current_quaternion_and_workspace():
    result = action_to_target_pose(
        [0.649, 0.0, 0.2],
        [0.0, 0.0, 0.0, 1.0],
        [0.005, 0.0, -0.06, 0.0, 0.0, math.pi / 2, 0.5],
        workspace_min=[0.2, -0.4, 0.15],
        workspace_max=[0.65, 0.4, 0.75],
    )
    assert result.position == pytest.approx([0.65, 0.0, 0.15])
    assert result.workspace_clipped is True
    assert result.orientation_xyzw == pytest.approx([
        0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)
    ])


def test_target_pose_allows_pregrasp_height_when_workspace_floor_is_low():
    result = action_to_target_pose(
        [0.35, -0.07, 0.12],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, -0.06, 0.0, 0.0, 0.0, 1.0],
        workspace_min=[0.2, -0.4, 0.02],
        workspace_max=[0.65, 0.4, 0.75],
    )
    assert result.position == pytest.approx([0.35, -0.07, 0.06])
    assert result.workspace_clipped is False


def test_local_tool_offset_rotates_with_hand_orientation():
    position, orientation = offset_pose_in_local_frame(
        [1.0, 2.0, 3.0],
        [0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)],
        [0.0, 0.0, 0.10],
    )
    assert position == pytest.approx([1.10, 2.0, 3.0])
    assert orientation == pytest.approx([
        0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)
    ])


def test_ros_image_decoder_handles_padding_and_bgr():
    message = Image()
    message.height = 1
    message.width = 2
    message.encoding = 'bgr8'
    message.step = 8
    message.data = bytes([3, 2, 1, 6, 5, 4, 99, 99])
    rgb = image_message_to_rgb(message)
    assert rgb.dtype == np.uint8
    assert rgb.shape == (1, 2, 3)
    assert rgb.tolist() == [[[1, 2, 3], [4, 5, 6]]]
