# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Deterministic wrist-camera pose candidates (simulation sensor placement)."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from mujoco_sim.camera_extrinsics import matrix_to_quat_wxyz


@dataclass(frozen=True)
class WristPoseCandidate:
    candidate_id: str
    translation_xyz: tuple[float, float, float]
    # MuJoCo camera axes in parent (hand) frame: columns of R are camera X,Y,Z.
    # Camera looks along −Z.
    rotation_matrix: tuple[tuple[float, float, float], ...]
    fovy_deg: float
    rationale: str
    pose_class: str = "CANDIDATE_POSE"

    def quat_wxyz(self) -> list[float]:
        R = np.asarray(self.rotation_matrix, dtype=float)
        return [float(x) for x in matrix_to_quat_wxyz(R)]

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["quat_wxyz"] = self.quat_wxyz()
        return d


def _axes_to_R(x: tuple[float, float, float], y: tuple[float, float, float]) -> np.ndarray:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    x = x / np.linalg.norm(x)
    y = y / np.linalg.norm(y)
    z = np.cross(x, y)
    z = z / np.linalg.norm(z)
    y = np.cross(z, x)  # re-orthogonalize
    return np.column_stack([x, y, z])


def wrist_pose_candidates(*, seed: int = 20260814) -> list[WristPoseCandidate]:
    """Finite deterministic candidate set around panda_hand (not random search).

    Seed is recorded for provenance; candidate geometry is fixed (not RNG-drawn).
    """
    _ = seed  # provenance only — set is deterministic by design.
    # A: current XML (side-looking). Known poor for grasp workspace.
    R_a = _axes_to_R((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))  # look = −Y_hand
    # B: look along +Z_hand (toward fingers / EE), slight back along −Z.
    R_b = _axes_to_R((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))  # look = +Z_hand
    # C: look +Z with mild pitch (camera Y tilted) for table coverage.
    # Rotate B by +25° about camera X (hand X): pitch look toward table when hand faces down.
    pitch = np.deg2rad(25.0)
    cx, sx = np.cos(pitch), np.sin(pitch)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    R_c = R_b @ Rx
    # D: closer to palm centerline, look +Z, slight lateral offset to reduce finger self-occlusion.
    R_d = R_b.copy()

    return [
        WristPoseCandidate(
            candidate_id="A_current_xml",
            translation_xyz=(0.0, -0.08, 0.03),
            rotation_matrix=tuple(map(tuple, R_a.tolist())),
            fovy_deg=70.0,
            rationale="Current XML pose; looks along −Y_hand (side view).",
        ),
        WristPoseCandidate(
            candidate_id="B_look_fingers",
            translation_xyz=(0.0, 0.0, -0.02),
            rotation_matrix=tuple(map(tuple, R_b.tolist())),
            fovy_deg=70.0,
            rationale="Look along +Z_hand toward fingers/EE; seated slightly behind palm.",
        ),
        WristPoseCandidate(
            candidate_id="C_higher_pitch_down",
            translation_xyz=(0.0, -0.03, 0.04),
            rotation_matrix=tuple(map(tuple, R_c.tolist())),
            fovy_deg=70.0,
            rationale="Higher mount + 25° pitch for table/workspace coverage at pre-grasp.",
        ),
        WristPoseCandidate(
            candidate_id="D_centerline_offset",
            translation_xyz=(0.02, 0.0, 0.0),
            rotation_matrix=tuple(map(tuple, R_d.tolist())),
            fovy_deg=65.0,
            rationale="Palm centerline with small +X offset; narrower FOV for grasp focus.",
        ),
    ]


def wrist_pose_candidates_outside_palm(*, seed: int = 20260814) -> list[WristPoseCandidate]:
    """Finite mounts intended to sit outside hand_0, still looking toward fingers.

    Same look as B (+Z_hand). RGB, not GT projection, is the selection metric.
    """
    _ = seed
    R_b = _axes_to_R((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
    pitch = np.deg2rad(15.0)
    cx, sx = np.cos(pitch), np.sin(pitch)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    R_pitch = R_b @ Rx
    R = tuple(map(tuple, R_b.tolist()))
    Rp = tuple(map(tuple, R_pitch.tolist()))
    return [
        WristPoseCandidate(
            candidate_id="E_behind_z08",
            translation_xyz=(0.0, 0.0, -0.08),
            rotation_matrix=R,
            fovy_deg=70.0,
            rationale="Same look as B; 80 mm behind hand origin, toward wrist.",
        ),
        WristPoseCandidate(
            candidate_id="F_behind_z12",
            translation_xyz=(0.0, 0.0, -0.12),
            rotation_matrix=R,
            fovy_deg=70.0,
            rationale="Further behind palm (120 mm) to clear hand_0 volume.",
        ),
        WristPoseCandidate(
            candidate_id="G_dorsal_ym05_z06",
            translation_xyz=(0.0, -0.05, -0.06),
            rotation_matrix=R,
            fovy_deg=70.0,
            rationale="Dorsal −Y offset plus behind-palm; outside the shell.",
        ),
        WristPoseCandidate(
            candidate_id="H_knuckle_z05",
            translation_xyz=(0.0, 0.0, 0.05),
            rotation_matrix=R,
            fovy_deg=70.0,
            rationale="Forward of palm at knuckle height, looking along fingers.",
        ),
        WristPoseCandidate(
            candidate_id="I_dorsal_knuckle",
            translation_xyz=(0.0, -0.04, 0.03),
            rotation_matrix=Rp,
            fovy_deg=70.0,
            rationale="Dorsal knuckle mount with 15° pitch toward the table.",
        ),
    ]


def all_wrist_pose_candidates(*, seed: int = 20260814) -> list[WristPoseCandidate]:
    return wrist_pose_candidates(seed=seed) + wrist_pose_candidates_outside_palm(seed=seed)
