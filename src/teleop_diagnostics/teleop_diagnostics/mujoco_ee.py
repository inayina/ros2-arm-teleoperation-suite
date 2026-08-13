# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""MuJoCo panda_ee site pose with fail-closed provenance."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping, Optional, Sequence

import numpy as np

from teleop_diagnostics import FRAME_BASE, FRAME_EE, PANDA_ARM_JOINTS
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    PoseSample,
    validate_homogeneous,
)


def _q_vector(q: Sequence[float] | Mapping[str, float]) -> np.ndarray:
    if isinstance(q, Mapping):
        missing = [n for n in PANDA_ARM_JOINTS if n not in q]
        if missing:
            raise GeometryDiagnosticsError(f"missing joint(s): {missing}")
        for n in q:
            if n.startswith("panda_joint") and n not in PANDA_ARM_JOINTS:
                raise GeometryDiagnosticsError(f"unknown joint: {n}")
        vals = [float(q[n]) for n in PANDA_ARM_JOINTS]
    else:
        vals = [float(v) for v in q]
        if len(vals) != 7:
            raise GeometryDiagnosticsError(
                f"invalid joint count: got {len(vals)}, expected 7"
            )
    arr = np.asarray(vals, dtype=float)
    if not np.all(np.isfinite(arr)):
        raise GeometryDiagnosticsError("NaN/Inf in MuJoCo joint vector")
    return arr


class MujocoEeSource:
    """Read panda_ee site pose from a real MuJoCo model (never fallback FK)."""

    def __init__(
        self,
        model_path: str | Path,
        *,
        site_name: str = FRAME_EE,
        base_frame: str = FRAME_BASE,
    ):
        self.model_path = Path(model_path)
        self.site_name = site_name
        self.base_frame = base_frame
        self._mujoco = None
        self.model = None
        self.data = None
        self.backend = "unknown"
        self.available = False
        self._site_id = -1
        self._qpos_addrs: list[int] = []
        self._load()

    def _load(self) -> None:
        if not self.model_path.is_file():
            self.backend = "unavailable"
            self.available = False
            return
        try:
            import mujoco

            self._mujoco = mujoco
            self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
            self.data = mujoco.MjData(self.model)
            self._site_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name
            )
            if self._site_id < 0:
                raise GeometryDiagnosticsError(f"MuJoCo site missing: {self.site_name}")
            for name in PANDA_ARM_JOINTS:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid < 0:
                    raise GeometryDiagnosticsError(f"MuJoCo joint missing: {name}")
                self._qpos_addrs.append(int(self.model.jnt_qposadr[jid]))
            self.backend = "mujoco"
            self.available = True
        except GeometryDiagnosticsError:
            self.backend = "invalid_model"
            self.available = False
            self.model = None
            self.data = None
            raise
        except Exception:  # noqa: BLE001
            self.backend = "unavailable"
            self.available = False
            self.model = None
            self.data = None

    def evidence_class_for_backend(self) -> EvidenceClass:
        if self.backend == "mujoco" and self.available:
            return EvidenceClass.SIM_GT
        if self.backend in ("fallback",):
            # Explicit guard: this class never uses fallback, but keep mapping.
            return EvidenceClass.MODEL
        return EvidenceClass.INSUFFICIENT_DATA

    def forward(self, q: Sequence[float] | Mapping[str, float]) -> PoseSample:
        qv = _q_vector(q)
        if not self.available or self.model is None or self.data is None:
            return PoseSample(
                source="mujoco_panda_ee_site",
                frame_from=self.base_frame,
                frame_to=self.site_name,
                reference_point=self.site_name,
                evidence_class=EvidenceClass.INSUFFICIENT_DATA,
                backend_provenance=f"mujoco_backend={self.backend}",
                input_status=InputStatus.UNAVAILABLE,
                matrix=None,
                detail="MuJoCo backend unavailable; not labeled SIM_GT",
            )
        mj = self._mujoco
        assert mj is not None
        for addr, qi in zip(self._qpos_addrs, qv):
            self.data.qpos[addr] = qi
        mj.mj_forward(self.model, self.data)
        xpos = np.array(self.data.site_xpos[self._site_id], dtype=float)
        xmat = np.array(self.data.site_xmat[self._site_id], dtype=float).reshape(3, 3)
        T = np.eye(4)
        T[:3, :3] = xmat
        T[:3, 3] = xpos
        # MuJoCo world ≡ panda_link0 under current identity world→link0 contract.
        T = validate_homogeneous(T)
        return PoseSample(
            source="mujoco_panda_ee_site",
            frame_from=self.base_frame,
            frame_to=self.site_name,
            reference_point=self.site_name,
            evidence_class=EvidenceClass.SIM_GT,
            backend_provenance=(
                f"mujoco_backend=mujoco;model={self.model_path.name};"
                f"site={self.site_name};api=site_xpos/site_xmat"
            ),
            input_status=InputStatus.AVAILABLE,
            matrix=T,
        )


def unavailable_mujoco_sample(
    *,
    backend: str = "unknown",
    detail: str = "",
) -> PoseSample:
    return PoseSample(
        source="mujoco_panda_ee_site",
        frame_from=FRAME_BASE,
        frame_to=FRAME_EE,
        reference_point=FRAME_EE,
        evidence_class=EvidenceClass.INSUFFICIENT_DATA,
        backend_provenance=f"mujoco_backend={backend}",
        input_status=InputStatus.UNAVAILABLE,
        matrix=None,
        detail=detail or "backend unavailable",
    )
