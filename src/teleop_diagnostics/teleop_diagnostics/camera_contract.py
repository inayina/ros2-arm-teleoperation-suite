# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Stage 3A/3C camera extrinsic contract helpers (offline REPORT_ONLY)."""

from __future__ import annotations

import math
from typing import Any, Optional, Sequence

import numpy as np

from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    ResultSemantics,
    rotation_geodesic,
    translation_error,
    validate_homogeneous,
)


def quat_wxyz_list(q: Sequence[float]) -> list[float]:
    return [float(x) for x in q]


def mat_to_rpy(R: np.ndarray) -> list[float]:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    pitch = math.asin(max(-1.0, min(1.0, float(-R[2, 0]))))
    if abs(math.cos(pitch)) < 1e-8:
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return [roll, pitch, yaw]


def residual_pair(Ta: np.ndarray, Tb: np.ndarray) -> tuple[float, float]:
    return translation_error(Ta, Tb), rotation_geodesic(Ta[:3, :3], Tb[:3, :3])


def camera_sample_row(
    *,
    camera_name: str,
    camera_type: str,
    parent_frame: str,
    optical_frame: str,
    state,
    T_renderer_world_link: Optional[np.ndarray],
    T_tf_world_link: Optional[np.ndarray],
    evidence_class: str,
    status: str,
    input_status: str,
    evidence_generation_commit: str,
    result_semantics: str = ResultSemantics.REPORT_ONLY.value,
) -> dict[str, Any]:
    T_nom = state.nominal_matrix()
    T_eff = state.effective_matrix()
    nom_eff_t, nom_eff_r = residual_pair(T_nom, T_eff)

    if T_renderer_world_link is None or T_tf_world_link is None:
        rt_t = None
        rt_r = None
        if status not in ("ERROR_INPUT", "MISSING"):
            status = "INSUFFICIENT_DATA"
            input_status = InputStatus.MISSING.value
            result_semantics = ResultSemantics.INSUFFICIENT_DATA.value
    else:
        validate_homogeneous(T_renderer_world_link)
        validate_homogeneous(T_tf_world_link)
        rt_t, rt_r = residual_pair(T_renderer_world_link, T_tf_world_link)

    def _xyz(T):
        return [float(x) for x in T[:3, 3]]

    def _quat(T):
        from mujoco_sim.camera_extrinsics import matrix_to_quat_wxyz

        return quat_wxyz_list(matrix_to_quat_wxyz(T[:3, :3]))

    return {
        "camera_name": camera_name,
        "camera_type": camera_type,
        "parent_frame": parent_frame,
        "optical_frame": optical_frame,
        "nominal_translation": state.nominal_translation,
        "nominal_rotation": state.nominal_quat_wxyz,
        "injected_translation": state.perturbation_translation,
        "injected_rotation": state.perturbation_quat_wxyz,
        "effective_translation": state.effective_translation(),
        "effective_rotation": state.effective_quat_wxyz(),
        "renderer_translation": (
            _xyz(T_renderer_world_link) if T_renderer_world_link is not None else ""
        ),
        "renderer_rotation": (
            _quat(T_renderer_world_link) if T_renderer_world_link is not None else ""
        ),
        "tf_translation": _xyz(T_tf_world_link) if T_tf_world_link is not None else "",
        "tf_rotation": _quat(T_tf_world_link) if T_tf_world_link is not None else "",
        "renderer_tf_translation_residual_m": rt_t if rt_t is not None else "",
        "renderer_tf_rotation_residual_rad": rt_r if rt_r is not None else "",
        "nominal_effective_translation_delta_m": nom_eff_t,
        "nominal_effective_rotation_delta_rad": nom_eff_r,
        "seed": state.seed if state.seed is not None else "",
        "evidence_class": evidence_class,
        "status": status,
        "effective_extrinsic_id": state.effective_id(),
        "input_status": input_status,
        "result_semantics": result_semantics,
        "evidence_generation_commit": evidence_generation_commit,
        "frame_from": parent_frame,
        "frame_to": optical_frame,
        "provenance": state.provenance,
    }


def project_point_to_camera(
    T_world_cam_mujoco: np.ndarray,
    p_world: Sequence[float],
    *,
    width: int,
    height: int,
    fovy_deg: float,
) -> dict[str, Any]:
    """Project a world point into MuJoCo camera image (OpenGL-style, no distortion)."""
    T = validate_homogeneous(T_world_cam_mujoco)
    R = T[:3, :3]
    t = T[:3, 3]
    p = np.asarray(p_world, dtype=float).reshape(3)
    p_cam = R.T @ (p - t)  # world → camera
    # MuJoCo camera looks along −Z; depth positive in front of camera.
    depth = float(-p_cam[2])
    if depth <= 1e-6 or not np.isfinite(depth):
        return {
            "target_visible": False,
            "target_center_x_norm": None,
            "target_center_y_norm": None,
            "depth_m": depth,
            "border_margin": None,
        }
    f = (height / 2.0) / math.tan(math.radians(fovy_deg / 2.0))
    u = f * (p_cam[0] / depth) + width / 2.0
    v = -f * (p_cam[1] / depth) + height / 2.0  # image y down
    x_norm = float(u / width)
    y_norm = float(v / height)
    visible = 0.0 <= x_norm <= 1.0 and 0.0 <= y_norm <= 1.0 and depth > 0.0
    margin = min(x_norm, 1.0 - x_norm, y_norm, 1.0 - y_norm) if visible else -1.0
    return {
        "target_visible": bool(visible),
        "target_center_x_norm": x_norm,
        "target_center_y_norm": y_norm,
        "depth_m": depth,
        "border_margin": float(margin),
        "target_bbox_area_ratio": None,  # no segmentation available — do not invent
        "occlusion_ratio_if_available": None,
    }


def require_camera(model, mujoco_module, name: str) -> int:
    cid = int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_CAMERA, name))
    if cid < 0:
        raise GeometryDiagnosticsError(f"missing camera '{name}'")
    return cid
