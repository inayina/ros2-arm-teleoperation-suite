# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Snapshot/apply visual scene state for the independent camera MjModel.

Physics (`mujoco_sim`) and renderer (`camera_bridge`) load separate models.
This module copies **object free-joint poses** and **light diffuse** so RGB
sees the same table layout and lighting as the physics reset/tick.

Does not copy mass/friction (not visible). Does not merge the two models.
"""

from __future__ import annotations

from typing import Any, Iterable, Mapping, Optional

import numpy as np

SCENE_VISUAL_TOPIC = "/sim/scene_visual"

MANIPULABLE_OBJECTS = (
    "object_red_box",
    "object_blue_cylinder",
    "object_green_sphere",
)

# randomization.yaml historically used lighting.key; XML light is named "top".
LIGHT_NAME_ALIASES = {"key": "top", "top": "key"}


def resolve_light_id(model, mujoco_module, light_name: str) -> int:
    lid = int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_LIGHT, light_name))
    if lid >= 0:
        return lid
    alias = LIGHT_NAME_ALIASES.get(light_name)
    if alias:
        return int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_LIGHT, alias))
    return -1


def snapshot_scene_visual(
    model,
    data,
    mujoco_module,
    *,
    object_names: Iterable[str] = MANIPULABLE_OBJECTS,
) -> dict[str, Any]:
    objects: dict[str, Any] = {}
    for name in object_names:
        bid = int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_BODY, name))
        if bid < 0:
            continue
        objects[name] = {
            "pos": [float(x) for x in np.asarray(data.xpos[bid], dtype=float)],
            "quat_wxyz": [float(x) for x in np.asarray(data.xquat[bid], dtype=float)],
        }
    lights: dict[str, Any] = {}
    nlight = int(getattr(model, "nlight", 0))
    for lid in range(nlight):
        name = mujoco_module.mj_id2name(model, mujoco_module.mjtObj.mjOBJ_LIGHT, lid)
        if not name:
            name = f"light_{lid}"
        lights[str(name)] = {
            "diffuse": [float(x) for x in np.asarray(model.light_diffuse[lid], dtype=float)],
        }
    return {
        "source": "mujoco_sim",
        "objects": objects,
        "lights": lights,
    }


def apply_object_poses(
    model,
    data,
    mujoco_module,
    objects: Mapping[str, Mapping[str, Any]],
    *,
    object_joints: Mapping[str, Mapping[str, Any]],
) -> list[str]:
    """Write free-joint qpos from snapshot. Returns applied body names."""
    applied = []
    for name, pose in objects.items():
        info = object_joints.get(name)
        if info is None:
            continue
        pos = pose.get("pos")
        quat = pose.get("quat_wxyz")
        if pos is None or quat is None or len(pos) != 3 or len(quat) != 4:
            continue
        qposadr = int(info["qposadr"])
        data.qpos[qposadr: qposadr + 3] = np.asarray(pos, dtype=float)
        q = np.asarray(quat, dtype=float)
        n = float(np.linalg.norm(q))
        if n < 1e-9 or not np.all(np.isfinite(q)):
            q = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            q = q / n
        data.qpos[qposadr + 3: qposadr + 7] = q
        applied.append(name)
    return applied


def apply_lights(model, mujoco_module, lights: Mapping[str, Mapping[str, Any]]) -> list[str]:
    applied = []
    for name, spec in lights.items():
        lid = resolve_light_id(model, mujoco_module, name)
        if lid < 0:
            continue
        diffuse = spec.get("diffuse")
        if diffuse is None or len(diffuse) < 3:
            continue
        for i in range(3):
            model.light_diffuse[lid][i] = max(0.0, min(1.0, float(diffuse[i])))
        applied.append(name)
    return applied
