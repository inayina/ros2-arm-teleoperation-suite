"""Pure adapter tests that do not require Isaac Sim or a GPU."""

import math
from pathlib import Path

from isaac_sim_adapter.adapter_node import filter_arm_joint_state
from isaac_sim_adapter.adapter_node import extract_arm_joint_target
from isaac_sim_adapter.adapter_node import normalize_namespace
from isaac_sim_adapter.adapter_node import PANDA_ARM_JOINTS
from isaac_sim_adapter.e1_action_sequence import trajectory_rmse
from isaac_sim_adapter.effort_control import LatestEffortCommand
from isaac_sim_adapter.effort_control import PANDA_TORQUE_LIMITS_NM
from isaac_sim_adapter.effort_control import validate_effort_command
from isaac_sim_adapter.effort_control import ZERO_EFFORT
import pytest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def test_namespace_normalization():
    assert normalize_namespace(' /isaac/ ') == '/isaac'
    with pytest.raises(ValueError, match='must not be empty'):
        normalize_namespace('///')


def test_joint_filter_reorders_and_removes_fingers():
    names = ['panda_finger_joint1', *reversed(PANDA_ARM_JOINTS)]
    message = JointState()
    message.name = names
    message.position = [float(index) for index in range(len(names))]
    message.velocity = [float(index + 10) for index in range(len(names))]
    message.effort = []

    filtered = filter_arm_joint_state(message)

    assert filtered is not None
    assert filtered.name == list(PANDA_ARM_JOINTS)
    assert list(filtered.position) == [7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0]
    assert list(filtered.velocity) == [17.0, 16.0, 15.0, 14.0, 13.0, 12.0, 11.0]
    assert list(filtered.effort) == []


def test_joint_filter_rejects_incomplete_arm_state():
    message = JointState()
    message.name = list(PANDA_ARM_JOINTS[:-1])
    message.position = [0.0] * len(message.name)
    assert filter_arm_joint_state(message) is None


def test_joint_filter_drops_truncated_optional_arrays():
    message = JointState()
    message.name = list(PANDA_ARM_JOINTS)
    message.position = [0.0] * 6
    assert list(filter_arm_joint_state(message).position) == []


def test_servo_target_extracts_final_point_in_canonical_order():
    message = JointTrajectory()
    message.joint_names = list(reversed(PANDA_ARM_JOINTS))
    message.points = [
        JointTrajectoryPoint(positions=[0.0] * 7),
        JointTrajectoryPoint(positions=[float(index) for index in range(7)]),
    ]
    assert extract_arm_joint_target(message) == [6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0]


def test_effort_validation_rejects_bad_shape_and_nonfinite_values():
    with pytest.raises(ValueError, match='expected 7'):
        validate_effort_command([0.0] * 6)
    with pytest.raises(ValueError, match='NaN or infinity'):
        validate_effort_command([0.0] * 6 + [math.nan])


def test_effort_validation_clamps_to_panda_limits():
    command, clipped = validate_effort_command([100.0] * 7)

    assert clipped is True
    assert command == PANDA_TORQUE_LIMITS_NM


def test_latest_effort_requires_fresh_state_and_times_out_to_zero():
    gate = LatestEffortCommand(command_timeout_s=0.1, state_timeout_s=0.2)

    rejected = gate.accept([1.0] * 7, now=1.0)
    assert rejected.status == 'state_unavailable'
    assert rejected.should_publish is False

    gate.update_state(now=1.01)
    active = gate.accept([1.0] * 7, now=1.02)
    assert active.status == 'active'
    assert active.efforts == (1.0,) * 7

    stale = gate.output(now=1.13)
    assert stale.status == 'command_stale'
    assert stale.efforts == ZERO_EFFORT
    assert stale.should_publish is True


def test_reset_clears_history_and_requires_new_epoch_state():
    gate = LatestEffortCommand(command_timeout_s=0.1, state_timeout_s=0.2)
    gate.update_state(now=1.0)
    assert gate.accept([1.0] * 7, now=1.01).status == 'active'

    gate.begin_reset()
    assert gate.output(now=1.02).status == 'reset_in_progress'
    gate.complete_reset()
    assert gate.accept([1.0] * 7, now=1.03).status == 'state_unavailable'
    gate.update_state(now=1.04)
    assert gate.accept([1.0] * 7, now=1.05).status == 'active'


def test_repeatability_rmse_is_zero_for_identical_trajectory():
    trajectory = [[0.0] * 7, [0.1] * 7, [0.2] * 7]
    assert trajectory_rmse(trajectory, trajectory) == pytest.approx(0.0)


def test_backend_source_applies_bounded_effort_and_local_watchdog():
    root = Path(__file__).resolve().parents[1]
    backend = (
        root / 'src/isaac_sim_adapter/scripts/isaac_panda_backend.py'
    ).read_text(encoding='utf-8')
    adapter = (
        root / 'src/isaac_sim_adapter/isaac_sim_adapter/adapter_node.py'
    ).read_text(encoding='utf-8')

    assert "'/isaac/joint_effort_cmd'" in backend
    assert 'ArticulationAction(' in backend
    assert 'LatestEffortCommand(' in backend
    assert "switch_dof_control_mode(int(joint_index), 'effort')" in backend
    assert "'/sim/joint_effort_cmd'" in adapter
    assert "f'{self._source}/gripper_cmd'" in adapter
    assert "'/teleop/gripper_cmd'" in adapter
    assert "'/isaac/gripper_cmd'" in backend
    assert "'/isaac/joint_position_cmd'" in backend
    assert "f'{self._source}/joint_position_cmd'" in adapter
    assert "'/joint_target'" in adapter
    assert 'franka.gripper.set_joint_positions' in backend
    assert 'franka.disable_gravity()' in backend
    assert 'NOMINAL_ARM_HOME = (0.0, -0.785' in backend
    assert 'NOMINAL_RED_BOX_POSITION = (0.35, -0.07, 0.025)' in backend
    assert "add_bin('bin_left', NOMINAL_BIN_Y[0])" in backend
    assert 'position=NOMINAL_CAMERA_POSITION' in backend
    assert '--object-seed' in backend
    assert 'resolve_red_box_pose' in backend
    assert 'qos_profile_sensor_data' in adapter
    assert 'self._effort.begin_reset()' in adapter


def test_object_pose_seed_is_deterministic_in_training_distribution():
    from isaac_sim_adapter.object_pose_seed import (
        OBJECT_X_RANGE,
        OBJECT_Y_RANGE,
        resolve_red_box_pose,
        sample_red_box_pose,
    )

    a = sample_red_box_pose(2000)
    b = sample_red_box_pose(2000)
    c = sample_red_box_pose(2001)
    assert a == b
    assert a != c
    assert OBJECT_X_RANGE[0] <= a[0] <= OBJECT_X_RANGE[1]
    assert OBJECT_Y_RANGE[0] <= a[1] <= OBJECT_Y_RANGE[1]
    assert abs(a[2] - 0.025) < 1e-9

    nominal = resolve_red_box_pose(object_seed=None)
    assert nominal[:3] == (0.35, -0.07, 0.025)
    override = resolve_red_box_pose(object_seed=2000, object_xy=(0.40, 0.0))
    assert override[:2] == (0.40, 0.0)
