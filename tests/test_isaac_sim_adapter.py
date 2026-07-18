"""Pure adapter tests that do not require Isaac Sim or a GPU."""

from isaac_sim_adapter.adapter_node import filter_arm_joint_state
from isaac_sim_adapter.adapter_node import normalize_namespace
from isaac_sim_adapter.adapter_node import PANDA_ARM_JOINTS
import pytest
from sensor_msgs.msg import JointState


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
