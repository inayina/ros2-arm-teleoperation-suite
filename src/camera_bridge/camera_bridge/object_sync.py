"""Shared object-name to MuJoCo joint mapping for camera render sync."""

from mujoco_sim.scene_visual import MANIPULABLE_OBJECTS

MUJOCO_SIM_PARAM_NODE = "/mujoco_sim"


def object_joint_name(target_object_name: str) -> str:
    """Match mujoco_sim_node._update_target_ids joint naming."""
    return target_object_name.replace("object_", "") + "_joint"
