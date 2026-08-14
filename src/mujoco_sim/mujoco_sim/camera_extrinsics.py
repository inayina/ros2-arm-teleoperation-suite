# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Camera extrinsic authority: MuJoCo XML nominal + camera-local ΔT.

Scheme B (Stage 3): the MuJoCo model XML is the sole *nominal* extrinsic
authority. Runtime consumers (camera_bridge renderer, TF, diagnostics) derive
effective poses from::

    T_parent_camera_effective = T_parent_camera_nominal @ ΔT_camera_local

Composition convention (v1, exclusive):
  * ΔT is expressed in the **nominal camera frame** (right-multiply / local).
  * World-frame additive perturbations are NOT supported in v1.

Frame naming:
  * ``{camera}_link``     — MuJoCo camera axes (look = −Z, up = +Y, right = +X)
  * ``{camera}_optical_frame`` — ROS REP-103 optical (+X right, +Y down, +Z forward)

MuJoCo camera → ROS optical (fixed)::

    R_mujoco_to_optical = diag(1, -1, -1)
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import asdict, dataclass, field
from typing import Any, Mapping, Optional, Sequence

import numpy as np

# MuJoCo camera axes → ROS optical: flip Y and Z (180° about X).
R_MUJOCO_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])

# Parent body name in MuJoCo XML → ROS TF parent frame.
BODY_TO_ROS_FRAME = {
    "world": "world",
    "hand": "panda_hand",
    "left_finger": "panda_leftfinger",
    "right_finger": "panda_rightfinger",
}


class CameraExtrinsicError(ValueError):
    """Fail-closed extrinsic contract error."""


def _normalize_quat_wxyz(q: Sequence[float]) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12 or not np.all(np.isfinite(q)):
        raise CameraExtrinsicError("invalid quaternion")
    return q / n


def quat_wxyz_to_matrix(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = _normalize_quat_wxyz(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return _normalize_quat_wxyz([w, x, y, z])


def quat_multiply_wxyz(q1: Sequence[float], q2: Sequence[float]) -> np.ndarray:
    w1, x1, y1, z1 = _normalize_quat_wxyz(q1)
    w2, x2, y2, z2 = _normalize_quat_wxyz(q2)
    return _normalize_quat_wxyz(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def make_transform(xyz: Sequence[float], rpy: Sequence[float] | None = None,
                   quat_wxyz: Sequence[float] | None = None) -> np.ndarray:
    T = np.eye(4)
    if quat_wxyz is not None:
        T[:3, :3] = quat_wxyz_to_matrix(quat_wxyz)
    elif rpy is not None:
        T[:3, :3] = rpy_to_matrix(*rpy)
    else:
        raise CameraExtrinsicError("rpy or quat_wxyz required")
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def transform_to_xyz_quat(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    T = np.asarray(T, dtype=float).reshape(4, 4)
    return T[:3, 3].copy(), matrix_to_quat_wxyz(T[:3, :3])


def mujoco_to_optical_transform() -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R_MUJOCO_TO_OPTICAL
    return T


def optical_frame_name(camera_name: str) -> str:
    return f"{camera_name}_optical_frame"


def link_frame_name(camera_name: str) -> str:
    return f"{camera_name}_link"


def effective_id_for(state: "CameraExtrinsicState") -> str:
    payload = (
        f"{state.camera_name}|{state.parent_frame}|"
        f"{np.asarray(state.nominal_translation).round(9).tolist()}|"
        f"{np.asarray(state.nominal_quat_wxyz).round(9).tolist()}|"
        f"{np.asarray(state.perturbation_translation).round(9).tolist()}|"
        f"{np.asarray(state.perturbation_quat_wxyz).round(9).tolist()}|"
        f"{state.seed}|{state.provenance}"
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


@dataclass
class CameraExtrinsicState:
    """Narrow extrinsic contract shared by simulator, renderer, TF, diagnostics."""

    camera_name: str
    parent_frame: str
    nominal_translation: list[float]
    nominal_quat_wxyz: list[float]
    perturbation_translation: list[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0]
    )
    perturbation_quat_wxyz: list[float] = field(
        default_factory=lambda: [1.0, 0.0, 0.0, 0.0]
    )
    seed: Optional[int] = None
    provenance: str = "xml_nominal"
    composition: str = "T_effective = T_nominal @ ΔT_camera_local"
    status: str = "AVAILABLE"  # AVAILABLE | MISSING | INVALID
    pose_class: str = "DESIGN_NOMINAL"  # or CANDIDATE_POSE for untuned wrist
    fovy_deg: Optional[float] = None

    @property
    def link_frame(self) -> str:
        return link_frame_name(self.camera_name)

    @property
    def optical_frame(self) -> str:
        return optical_frame_name(self.camera_name)

    def nominal_matrix(self) -> np.ndarray:
        return make_transform(self.nominal_translation, quat_wxyz=self.nominal_quat_wxyz)

    def perturbation_matrix(self) -> np.ndarray:
        return make_transform(
            self.perturbation_translation, quat_wxyz=self.perturbation_quat_wxyz
        )

    def effective_matrix(self) -> np.ndarray:
        return self.nominal_matrix() @ self.perturbation_matrix()

    def effective_translation(self) -> list[float]:
        return [float(x) for x in self.effective_matrix()[:3, 3]]

    def effective_quat_wxyz(self) -> list[float]:
        return [float(x) for x in matrix_to_quat_wxyz(self.effective_matrix()[:3, :3])]

    def effective_id(self) -> str:
        return effective_id_for(self)

    def with_local_perturbation(
        self,
        *,
        translation_m: Sequence[float] | None = None,
        rpy_rad: Sequence[float] | None = None,
        quat_wxyz: Sequence[float] | None = None,
        provenance: str | None = None,
        seed: Optional[int] = None,
    ) -> "CameraExtrinsicState":
        if translation_m is None:
            translation_m = [0.0, 0.0, 0.0]
        if quat_wxyz is None:
            if rpy_rad is None:
                rpy_rad = [0.0, 0.0, 0.0]
            quat_wxyz = matrix_to_quat_wxyz(rpy_to_matrix(*rpy_rad))
        return CameraExtrinsicState(
            camera_name=self.camera_name,
            parent_frame=self.parent_frame,
            nominal_translation=list(self.nominal_translation),
            nominal_quat_wxyz=list(self.nominal_quat_wxyz),
            perturbation_translation=[float(x) for x in translation_m],
            perturbation_quat_wxyz=[float(x) for x in _normalize_quat_wxyz(quat_wxyz)],
            seed=self.seed if seed is None else seed,
            provenance=provenance or self.provenance,
            composition=self.composition,
            status=self.status,
            pose_class=self.pose_class,
            fovy_deg=self.fovy_deg,
        )

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["link_frame"] = self.link_frame
        d["optical_frame"] = self.optical_frame
        d["effective_translation"] = self.effective_translation()
        d["effective_quat_wxyz"] = self.effective_quat_wxyz()
        d["effective_id"] = self.effective_id()
        return d


def camera_id(model, mujoco_module, camera_name: str) -> int:
    cid = int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_CAMERA, camera_name))
    if cid < 0:
        raise CameraExtrinsicError(f"missing camera '{camera_name}' in MuJoCo model")
    return cid


def parent_ros_frame(model, mujoco_module, camera_name: str) -> str:
    cid = camera_id(model, mujoco_module, camera_name)
    body_id = int(model.cam_bodyid[cid])
    if body_id <= 0:
        return "world"
    body_name = mujoco_module.mj_id2name(model, mujoco_module.mjtObj.mjOBJ_BODY, body_id)
    if not body_name:
        raise CameraExtrinsicError(f"camera '{camera_name}' parent body unnamed")
    if body_name not in BODY_TO_ROS_FRAME:
        raise CameraExtrinsicError(
            f"camera '{camera_name}' parent body '{body_name}' has no ROS frame mapping"
        )
    return BODY_TO_ROS_FRAME[body_name]


def extract_nominal_from_model(
    model,
    mujoco_module,
    camera_name: str,
    *,
    pose_class: str = "DESIGN_NOMINAL",
) -> CameraExtrinsicState:
    cid = camera_id(model, mujoco_module, camera_name)
    pos = [float(x) for x in np.asarray(model.cam_pos[cid], dtype=float)]
    quat = [float(x) for x in _normalize_quat_wxyz(model.cam_quat[cid])]
    fovy = float(model.cam_fovy[cid])
    return CameraExtrinsicState(
        camera_name=camera_name,
        parent_frame=parent_ros_frame(model, mujoco_module, camera_name),
        nominal_translation=pos,
        nominal_quat_wxyz=quat,
        provenance="mujoco_xml_nominal",
        pose_class=pose_class,
        fovy_deg=fovy,
        status="AVAILABLE",
    )


def apply_state_to_model(model, mujoco_module, state: CameraExtrinsicState) -> None:
    """Write *effective* parent→camera pose into the MuJoCo model buffers."""
    if state.status != "AVAILABLE":
        raise CameraExtrinsicError(
            f"refuse to apply extrinsic with status={state.status}"
        )
    cid = camera_id(model, mujoco_module, state.camera_name)
    xyz, quat = transform_to_xyz_quat(state.effective_matrix())
    model.cam_pos[cid][:] = xyz
    model.cam_quat[cid][:] = quat


def renderer_world_pose(model, data, mujoco_module, camera_name: str) -> np.ndarray:
    """4x4 world←MuJoCo-camera after mj_forward (SIM_GT renderer pose)."""
    cid = camera_id(model, mujoco_module, camera_name)
    T = np.eye(4)
    T[:3, :3] = np.asarray(data.cam_xmat[cid], dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(data.cam_xpos[cid], dtype=float)
    return T


def world_optical_from_world_mujoco(T_world_mujoco: np.ndarray) -> np.ndarray:
    return np.asarray(T_world_mujoco, dtype=float) @ mujoco_to_optical_transform()


def seeded_local_rpy_noise(
    seed: int,
    camera_name: str,
    *,
    pos_noise: Sequence[float],
    rot_noise_deg: Sequence[float],
    draw_index: int = 0,
) -> tuple[np.ndarray, np.ndarray]:
    """Deterministic camera-local noise matching DomainRandomizer ranges.

    Uses an independent RNG stream keyed by (seed, camera_name, draw_index) so
    the main simulator and camera_bridge never draw different samples.
    """
    key = f"{seed}:{camera_name}:{draw_index}:camera_local"
    digest = hashlib.sha256(key.encode("utf-8")).digest()
    seed_u32 = int.from_bytes(digest[:4], "little")
    rng = np.random.default_rng(seed_u32)
    lo_p, hi_p = float(pos_noise[0]), float(pos_noise[1])
    lo_r, hi_r = float(rot_noise_deg[0]), float(rot_noise_deg[1])
    dx = float(rng.uniform(lo_p, hi_p))
    dy = float(rng.uniform(lo_p, hi_p))
    dz = float(rng.uniform(lo_p, hi_p))
    roll = math.radians(float(rng.uniform(lo_r, hi_r)))
    pitch = math.radians(float(rng.uniform(lo_r, hi_r)))
    yaw = math.radians(float(rng.uniform(lo_r, hi_r)))
    return np.array([dx, dy, dz], dtype=float), np.array([roll, pitch, yaw], dtype=float)


def apply_config_randomization(
    nominal: CameraExtrinsicState,
    camera_cfg: Mapping[str, Any],
    *,
    seed: int,
    draw_index: int = 0,
) -> CameraExtrinsicState:
    params = camera_cfg.get(nominal.camera_name)
    if not params:
        return nominal
    pos_noise = params.get("pos_noise")
    rot_noise = params.get("rot_noise")
    if not pos_noise and not rot_noise:
        return nominal
    pos_noise = pos_noise or [0.0, 0.0]
    rot_noise = rot_noise or [0.0, 0.0]
    t, rpy = seeded_local_rpy_noise(
        seed,
        nominal.camera_name,
        pos_noise=pos_noise,
        rot_noise_deg=rot_noise,
        draw_index=draw_index,
    )
    return nominal.with_local_perturbation(
        translation_m=t,
        rpy_rad=rpy,
        provenance=f"seeded_local_randomization:seed={seed}:draw={draw_index}",
        seed=seed,
    )


class CameraExtrinsicAuthority:
    """Extract / perturb / apply camera extrinsics from a MuJoCo model."""

    def __init__(self, model, mujoco_module, *, pose_class_by_camera: Optional[dict] = None):
        self.model = model
        self.mujoco = mujoco_module
        self.pose_class_by_camera = pose_class_by_camera or {}
        self._nominal_cache: dict[str, CameraExtrinsicState] = {}
        self._states: dict[str, CameraExtrinsicState] = {}

    def nominal(self, camera_name: str) -> CameraExtrinsicState:
        if camera_name not in self._nominal_cache:
            pose_class = self.pose_class_by_camera.get(camera_name, "DESIGN_NOMINAL")
            self._nominal_cache[camera_name] = extract_nominal_from_model(
                self.model, self.mujoco, camera_name, pose_class=pose_class
            )
        return self._nominal_cache[camera_name]

    def get(self, camera_name: str) -> CameraExtrinsicState:
        if camera_name not in self._states:
            self._states[camera_name] = self.nominal(camera_name)
        return self._states[camera_name]

    def set_state(self, state: CameraExtrinsicState, *, write_model: bool = True) -> None:
        self._states[state.camera_name] = state
        if write_model:
            apply_state_to_model(self.model, self.mujoco, state)

    def reset_nominal(self, camera_name: str, *, write_model: bool = True) -> CameraExtrinsicState:
        state = self.nominal(camera_name)
        self.set_state(state, write_model=write_model)
        return state

    def inject_local(
        self,
        camera_name: str,
        *,
        translation_m: Sequence[float] | None = None,
        rpy_rad: Sequence[float] | None = None,
        provenance: str = "diagnostic_injection",
        write_model: bool = True,
    ) -> CameraExtrinsicState:
        state = self.nominal(camera_name).with_local_perturbation(
            translation_m=translation_m,
            rpy_rad=rpy_rad,
            provenance=provenance,
        )
        self.set_state(state, write_model=write_model)
        return state

    def apply_randomization_config(
        self,
        camera_cfg: Mapping[str, Any],
        *,
        seed: int,
        draw_index: int = 0,
        write_model: bool = True,
    ) -> dict[str, CameraExtrinsicState]:
        out = {}
        for camera_name in camera_cfg:
            try:
                nom = self.nominal(camera_name)
            except CameraExtrinsicError:
                continue
            state = apply_config_randomization(
                nom, camera_cfg, seed=seed, draw_index=draw_index
            )
            self.set_state(state, write_model=write_model)
            out[camera_name] = state
        return out
