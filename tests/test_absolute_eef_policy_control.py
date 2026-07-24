"""Pure tests for absolute-EEF execution helpers (no ROS runtime import)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "isaac_sim_adapter"))

from isaac_sim_adapter.policy_control import (  # noqa: E402
    absolute_action_to_target_pose,
    bound_absolute_eef_gripper,
)


def test_absolute_eef_clamps_workspace_and_gripper_for_smolvla_s4():
    result = bound_absolute_eef_gripper(
        [0.80, 0.0, 0.05, 0.0, 0.0, 0.0, 2.0, 1.22],
        workspace_min=[0.20, -0.40, 0.02],
        workspace_max=[0.65, 0.40, 0.75],
    )
    assert result.clipped is True
    assert result.values[:3] == pytest.approx([0.65, 0.0, 0.05])
    assert result.values[7] == pytest.approx(1.0)
    pose = absolute_action_to_target_pose(result.values)
    assert pose.position == pytest.approx([0.65, 0.0, 0.05])
    with pytest.raises(ValueError, match="expected absolute ee_pose_gripper"):
        bound_absolute_eef_gripper(
            [0.0] * 7,
            workspace_min=[0.2, -0.4, 0.02],
            workspace_max=[0.65, 0.4, 0.75],
        )


def test_absolute_eef_rejects_nonfinite():
    with pytest.raises(ValueError, match="NaN or infinity"):
        bound_absolute_eef_gripper(
            [0.4, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0, math.nan],
            workspace_min=[0.2, -0.4, 0.02],
            workspace_max=[0.65, 0.4, 0.75],
        )
