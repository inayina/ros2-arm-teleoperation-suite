"""Static contract tests for the Isaac ROS freshness gate."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'isaac_ros_topic_gate.py'


def _module():
    spec = importlib.util.spec_from_file_location('isaac_ros_topic_gate', SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_base_gate_matches_smolvla_policy_inputs() -> None:
    module = _module()
    requirements = module.required_topics(False)
    assert [(item.key, item.topic) for item in requirements] == [
        ('raw_joint_state', '/isaac/joint_states'),
        ('encoder_state', '/sim/encoder_state'),
        ('ee_pose', '/ee_pose'),
        ('gripper_state', '/gripper/state'),
        ('scene_rgb', '/camera/color/image_raw'),
        ('wrist_rgb', '/camera/wrist/color/image_raw'),
    ]


def test_full_system_gate_adds_moveit_control_state() -> None:
    module = _module()
    requirements = module.required_topics(True)
    assert requirements[-1].key == 'control_joint_state'
    assert requirements[-1].topic == '/joint_states'
