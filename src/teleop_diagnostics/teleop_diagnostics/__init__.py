# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Observer-only geometry diagnostics (Stage 1: TF/FK authority)."""

__version__ = "0.1.0"

PANDA_ARM_JOINTS = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)

FRAME_BASE = "panda_link0"
FRAME_LINK7 = "panda_link7"
FRAME_HAND = "panda_hand"
FRAME_EE = "panda_ee"
