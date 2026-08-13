# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Independent URDF and PyKDL forward kinematics (MODEL evidence)."""

from __future__ import annotations

import math
import subprocess
from pathlib import Path
from typing import Iterable, Mapping, Optional, Sequence

import numpy as np
from urdf_parser_py.urdf import URDF

from teleop_diagnostics import FRAME_BASE, FRAME_EE, PANDA_ARM_JOINTS
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    PoseSample,
    validate_homogeneous,
)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def expand_xacro_to_urdf(xacro_path: str | Path, output_path: Optional[str | Path] = None) -> str:
    xacro_path = Path(xacro_path)
    if not xacro_path.is_file():
        raise GeometryDiagnosticsError(f"xacro not found: {xacro_path}")
    cmd = ["xacro", str(xacro_path)]
    proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if proc.returncode != 0:
        raise GeometryDiagnosticsError(f"xacro failed: {proc.stderr.strip()}")
    xml = proc.stdout
    if output_path is not None:
        Path(output_path).write_text(xml, encoding="utf-8")
    return xml


def load_urdf_model(urdf_xml_or_path: str | Path) -> URDF:
    if isinstance(urdf_xml_or_path, Path):
        if not urdf_xml_or_path.is_file():
            raise GeometryDiagnosticsError(f"URDF file not found: {urdf_xml_or_path}")
        return URDF.from_xml_file(str(urdf_xml_or_path))
    text = str(urdf_xml_or_path)
    stripped = text.lstrip()
    if stripped.startswith("<?xml") or stripped.startswith("<robot"):
        return URDF.from_xml_string(text)
    path = Path(text)
    if path.is_file():
        return URDF.from_xml_file(str(path))
    raise GeometryDiagnosticsError("URDF input is neither XML nor an existing file path")


def _validate_q(
    q: Sequence[float] | Mapping[str, float],
    joint_names: Sequence[str] = PANDA_ARM_JOINTS,
) -> dict[str, float]:
    if isinstance(q, Mapping):
        missing = [n for n in joint_names if n not in q]
        if missing:
            raise GeometryDiagnosticsError(f"missing joint(s): {missing}")
        unknown = [n for n in q if n not in joint_names and n.startswith("panda_joint")]
        # Allow extra non-arm keys, but unknown arm joints fail closed.
        extra_arm = [n for n in q if n not in joint_names and n in set(PANDA_ARM_JOINTS)]
        if extra_arm:
            raise GeometryDiagnosticsError(f"unknown arm joint(s): {extra_arm}")
        out = {n: float(q[n]) for n in joint_names}
    else:
        vals = list(q)
        if len(vals) != len(joint_names):
            raise GeometryDiagnosticsError(
                f"invalid joint count: got {len(vals)}, expected {len(joint_names)}"
            )
        out = {n: float(v) for n, v in zip(joint_names, vals)}
    for name, val in out.items():
        if not math.isfinite(val):
            raise GeometryDiagnosticsError(f"NaN/Inf joint value: {name}={val}")
    return out


def _joint_origin_transform(joint, q: float = 0.0) -> np.ndarray:
    xyz = list(joint.origin.xyz) if joint.origin is not None else [0.0, 0.0, 0.0]
    rpy = list(joint.origin.rpy) if joint.origin is not None else [0.0, 0.0, 0.0]
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    if joint.type in ("revolute", "continuous"):
        axis = np.asarray(joint.axis if joint.axis is not None else [0.0, 0.0, 1.0], dtype=float)
        n = np.linalg.norm(axis)
        if n < 1e-12:
            raise GeometryDiagnosticsError(f"invalid joint axis: {joint.name}")
        axis = axis / n
        c, s = math.cos(q), math.sin(q)
        kx, ky, kz = axis
        K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
        R = np.eye(3) + s * K + (1.0 - c) * (K @ K)
        Tj = np.eye(4)
        Tj[:3, :3] = R
        T = T @ Tj
    elif joint.type not in ("fixed", "floating", "planar", "prismatic"):
        # prismatic unused on Panda arm chain; fixed handled above
        if joint.type == "prismatic":
            axis = np.asarray(joint.axis if joint.axis is not None else [0.0, 0.0, 1.0], dtype=float)
            axis = axis / np.linalg.norm(axis)
            T[:3, 3] = T[:3, 3] + axis * q
    return T


def _chain_joints(robot: URDF, target_link: str, base_link: str = FRAME_BASE):
    if target_link not in robot.link_map and target_link != robot.get_root():
        raise GeometryDiagnosticsError(f"unknown link: {target_link}")
    chain = []
    link = target_link
    while link != base_link:
        if link not in robot.parent_map:
            raise GeometryDiagnosticsError(
                f"no path from {base_link} to {target_link} (stopped at {link})"
            )
        joint_name, parent = robot.parent_map[link]
        chain.append(robot.joint_map[joint_name])
        link = parent
    chain.reverse()
    return chain


class IndependentUrdfFk:
    """Parse robot_description / URDF and compute FK without MuJoCo constants."""

    def __init__(
        self,
        robot: URDF,
        *,
        base_link: str = FRAME_BASE,
        joint_names: Sequence[str] = PANDA_ARM_JOINTS,
        provenance: str = "urdf_parser_py.chain_fk",
    ):
        self.robot = robot
        self.base_link = base_link
        self.joint_names = tuple(joint_names)
        self.provenance = provenance

    @classmethod
    def from_xacro(cls, xacro_path: str | Path, **kwargs) -> "IndependentUrdfFk":
        xml = expand_xacro_to_urdf(xacro_path)
        return cls(load_urdf_model(xml), **kwargs)

    @classmethod
    def from_urdf_file(cls, urdf_path: str | Path, **kwargs) -> "IndependentUrdfFk":
        return cls(load_urdf_model(urdf_path), **kwargs)

    def forward(
        self,
        q: Sequence[float] | Mapping[str, float],
        target_link: str = FRAME_EE,
    ) -> PoseSample:
        try:
            q_map = _validate_q(q, self.joint_names)
            # Reject unknown required joints already handled; also reject if target needs
            # an arm joint not in map (covered). Fail on unknown joint names passed as list
            # is count check. For Mapping with unexpected arm joint names:
            if isinstance(q, Mapping):
                unknown = [n for n in q.keys() if n not in self.joint_names]
                # non-arm extras ignored unless they look like required panda joints with typo
                for n in unknown:
                    if n.startswith("panda_joint"):
                        raise GeometryDiagnosticsError(f"unknown joint: {n}")
            chain = _chain_joints(self.robot, target_link, self.base_link)
            T = np.eye(4)
            for joint in chain:
                qi = 0.0
                if joint.type in ("revolute", "continuous", "prismatic"):
                    if joint.name not in q_map and joint.name in self.joint_names:
                        raise GeometryDiagnosticsError(f"missing joint: {joint.name}")
                    if joint.name in q_map:
                        qi = q_map[joint.name]
                    elif joint.name.startswith("panda_joint"):
                        raise GeometryDiagnosticsError(f"unknown/missing joint: {joint.name}")
                T = T @ _joint_origin_transform(joint, qi)
            T = validate_homogeneous(T)
            return PoseSample(
                source="independent_urdf_fk",
                frame_from=self.base_link,
                frame_to=target_link,
                reference_point=target_link,
                evidence_class=EvidenceClass.MODEL,
                backend_provenance=self.provenance,
                input_status=InputStatus.AVAILABLE,
                matrix=T,
            )
        except GeometryDiagnosticsError:
            raise
        except Exception as exc:  # noqa: BLE001
            raise GeometryDiagnosticsError(str(exc)) from exc


class IndependentKdlFk:
    """Second MODEL path via PyKDL chain built from the same URDF (not MuJoCo)."""

    def __init__(
        self,
        robot: URDF,
        *,
        base_link: str = FRAME_BASE,
        tip_link: str = FRAME_EE,
        joint_names: Sequence[str] = PANDA_ARM_JOINTS,
    ):
        import PyKDL as kdl

        self._kdl = kdl
        self.robot = robot
        self.base_link = base_link
        self.tip_link = tip_link
        self.joint_names = tuple(joint_names)
        self.chain, self._q_joint_names = self._build_chain(robot, base_link, tip_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.provenance = (
            "PyKDL.ChainFkSolverPos_recursive_from_urdf"
            "(fixed_origin_then_revolute; URDF T_origin*R(q) order)"
        )

    def _build_chain(self, robot: URDF, base_link: str, tip_link: str):
        """Build a PyKDL chain with URDF joint order: T_origin * R(q).

        Standard ``Joint(0) * f_tip(origin)`` is Rot(q)*T_origin and does **not**
        match URDF when the joint origin has nonzero translation. We therefore
        emit a Fixed segment for the origin, then a RotAxis segment (identity tip).
        """
        kdl = self._kdl
        joints = _chain_joints(robot, tip_link, base_link)
        chain = kdl.Chain()
        q_names: list[str] = []
        for joint in joints:
            xyz = list(joint.origin.xyz) if joint.origin is not None else [0.0, 0.0, 0.0]
            rpy = list(joint.origin.rpy) if joint.origin is not None else [0.0, 0.0, 0.0]
            origin = kdl.Frame(
                kdl.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
                kdl.Vector(xyz[0], xyz[1], xyz[2]),
            )
            chain.addSegment(
                kdl.Segment(
                    f"{joint.name}__origin",
                    kdl.Joint(f"{joint.name}__fixed", kdl.Joint.Fixed),
                    origin,
                )
            )
            if joint.type in ("revolute", "continuous"):
                axis = joint.axis if joint.axis is not None else [0.0, 0.0, 1.0]
                jnt = kdl.Joint(
                    joint.name,
                    kdl.Vector(),
                    kdl.Vector(axis[0], axis[1], axis[2]),
                    kdl.Joint.RotAxis,
                )
                q_names.append(joint.name)
                chain.addSegment(
                    kdl.Segment(joint.child, jnt, kdl.Frame.Identity())
                )
            else:
                chain.addSegment(
                    kdl.Segment(
                        joint.child,
                        kdl.Joint(joint.name, kdl.Joint.Fixed),
                        kdl.Frame.Identity(),
                    )
                )
        return chain, q_names

    def forward(
        self,
        q: Sequence[float] | Mapping[str, float],
        target_link: Optional[str] = None,
    ) -> PoseSample:
        if target_link is not None and target_link != self.tip_link:
            # Rebuild for alternate tip (link7/hand/ee comparisons).
            other = IndependentKdlFk(
                self.robot,
                base_link=self.base_link,
                tip_link=target_link,
                joint_names=self.joint_names,
            )
            return other.forward(q)
        q_map = _validate_q(q, self.joint_names)
        if isinstance(q, Mapping):
            for n in q.keys():
                if n.startswith("panda_joint") and n not in self.joint_names:
                    raise GeometryDiagnosticsError(f"unknown joint: {n}")
        kdl = self._kdl
        jnt = kdl.JntArray(len(self._q_joint_names))
        for i, name in enumerate(self._q_joint_names):
            jnt[i] = q_map[name]
        frame = kdl.Frame()
        rc = self.fk_solver.JntToCart(jnt, frame)
        if rc < 0:
            raise GeometryDiagnosticsError(f"KDL FK failed with code {rc}")
        T = np.eye(4)
        for r in range(3):
            for c in range(3):
                T[r, c] = frame.M[r, c]
            T[r, 3] = frame.p[r]
        T = validate_homogeneous(T)
        return PoseSample(
            source="independent_kdl_fk",
            frame_from=self.base_link,
            frame_to=self.tip_link,
            reference_point=self.tip_link,
            evidence_class=EvidenceClass.MODEL,
            backend_provenance=self.provenance,
            input_status=InputStatus.AVAILABLE,
            matrix=T,
        )
