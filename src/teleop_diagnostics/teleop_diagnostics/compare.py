# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Cross-model FK/TF/SIM_GT comparator (observer-only, REPORT_ONLY)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Optional, Sequence

import numpy as np

from teleop_diagnostics import FRAME_BASE, FRAME_EE, FRAME_HAND, FRAME_LINK7, PANDA_ARM_JOINTS
from teleop_diagnostics.controller_fk import (
    CONTROLLER_REFERENCE_POINT,
    controller_analytic_fk,
    controller_jacobian_reference_point,
)
from teleop_diagnostics.mujoco_ee import MujocoEeSource
from teleop_diagnostics.tf_source import RobotStatePublisherTfSource
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    PoseSample,
    ResidualRow,
    ResultSemantics,
    rotation_geodesic,
    translation_error,
)
from teleop_diagnostics.urdf_fk import IndependentKdlFk, IndependentUrdfFk


def _pair_residual(
    a: PoseSample,
    b: PoseSample,
    *,
    scenario: str,
    q: Sequence[float],
    commit: str,
    backend: str,
    frame_to: str,
    reference_point: str,
) -> ResidualRow:
    statuses = {a.input_status, b.input_status}
    if (
        a.matrix is None
        or b.matrix is None
        or a.input_status != InputStatus.AVAILABLE
        or b.input_status != InputStatus.AVAILABLE
        or a.evidence_class == EvidenceClass.INSUFFICIENT_DATA
        or b.evidence_class == EvidenceClass.INSUFFICIENT_DATA
    ):
        return ResidualRow(
            scenario=scenario,
            q=list(map(float, q)),
            source_a=a.source,
            source_b=b.source,
            frame_from=FRAME_BASE,
            frame_to=frame_to,
            reference_point=reference_point,
            translation_error_m=None,
            rotation_error_rad=None,
            evidence_class_a=a.evidence_class.value,
            evidence_class_b=b.evidence_class.value,
            input_status="|".join(sorted(s.value for s in statuses)),
            result_semantics=ResultSemantics.INSUFFICIENT_DATA.value,
            backend=backend,
            commit=commit,
        )
    # Unknown provenance must not PASS — Stage 1 never emits PASS anyway.
    if "unknown" in a.backend_provenance or "unknown" in b.backend_provenance:
        sem = ResultSemantics.INSUFFICIENT_DATA.value
    else:
        sem = ResultSemantics.REPORT_ONLY.value
    return ResidualRow(
        scenario=scenario,
        q=list(map(float, q)),
        source_a=a.source,
        source_b=b.source,
        frame_from=FRAME_BASE,
        frame_to=frame_to,
        reference_point=reference_point,
        translation_error_m=translation_error(a.matrix, b.matrix),
        rotation_error_rad=rotation_geodesic(a.matrix[:3, :3], b.matrix[:3, :3]),
        evidence_class_a=a.evidence_class.value,
        evidence_class_b=b.evidence_class.value,
        input_status=InputStatus.AVAILABLE.value,
        result_semantics=sem,
        backend=backend,
        commit=commit,
    )


@dataclass
class CrossModelComparator:
    urdf_fk: IndependentUrdfFk
    kdl_fk: IndependentKdlFk
    mujoco: MujocoEeSource
    tf_source: RobotStatePublisherTfSource
    commit: str = "UNKNOWN"
    backend_label: str = "offline"

    def sample_sources(
        self,
        q: Sequence[float] | Mapping[str, float],
        *,
        ee_target: str = FRAME_EE,
    ) -> dict[str, PoseSample]:
        q_list = (
            [float(q[n]) for n in PANDA_ARM_JOINTS]
            if isinstance(q, Mapping)
            else list(map(float, q))
        )
        samples = {
            "independent_urdf_fk": self.urdf_fk.forward(q_list, target_link=ee_target),
            "independent_kdl_fk": self.kdl_fk.forward(q_list, target_link=ee_target),
            "robot_state_publisher_tf": self.tf_source.forward(q_list),
            "mujoco_panda_ee_site": self.mujoco.forward(q_list),
            "controller_analytic_fk": controller_analytic_fk(q_list),
        }
        return samples

    def compare_nominal(
        self,
        scenario: str,
        q: Sequence[float],
    ) -> list[ResidualRow]:
        samples = self.sample_sources(q, ee_target=FRAME_EE)
        model = samples["independent_urdf_fk"]
        tf = samples["robot_state_publisher_tf"]
        sim = samples["mujoco_panda_ee_site"]
        ctrl = samples["controller_analytic_fk"]
        kdl = samples["independent_kdl_fk"]
        rows = [
            _pair_residual(
                model,
                kdl,
                scenario=scenario,
                q=q,
                commit=self.commit,
                backend=self.backend_label,
                frame_to=FRAME_EE,
                reference_point=FRAME_EE,
            ),
            _pair_residual(
                model,
                tf,
                scenario=scenario,
                q=q,
                commit=self.commit,
                backend=self.backend_label,
                frame_to=FRAME_EE,
                reference_point=FRAME_EE,
            ),
            _pair_residual(
                model,
                sim,
                scenario=scenario,
                q=q,
                commit=self.commit,
                backend=self.backend_label,
                frame_to=FRAME_EE,
                reference_point=FRAME_EE,
            ),
            _pair_residual(
                tf,
                sim,
                scenario=scenario,
                q=q,
                commit=self.commit,
                backend=self.backend_label,
                frame_to=FRAME_EE,
                reference_point=FRAME_EE,
            ),
        ]
        # Controller vs its contracted reference (panda_link7) and vs hand/ee for audit.
        for tip in (FRAME_LINK7, FRAME_HAND, FRAME_EE):
            tip_pose = self.urdf_fk.forward(q, target_link=tip)
            rows.append(
                _pair_residual(
                    ctrl,
                    tip_pose,
                    scenario=f"{scenario}__controller_vs_{tip}",
                    q=q,
                    commit=self.commit,
                    backend=self.backend_label,
                    frame_to=tip,
                    reference_point=CONTROLLER_REFERENCE_POINT,
                )
            )
        return rows

    def controller_reference_audit(self, q: Sequence[float]) -> dict[str, Any]:
        ctrl = controller_analytic_fk(q)
        out: dict[str, Any] = {
            "controller_reference_point_contract": CONTROLLER_REFERENCE_POINT,
            "jacobian_reference_point": controller_jacobian_reference_point(),
            "jacobian_matches_fk_contract": (
                controller_jacobian_reference_point() == CONTROLLER_REFERENCE_POINT
            ),
            "controller_pose": ctrl.to_dict(),
            "residuals_vs_urdf": {},
        }
        for tip in (FRAME_LINK7, FRAME_HAND, FRAME_EE):
            tip_pose = self.urdf_fk.forward(q, target_link=tip)
            out["residuals_vs_urdf"][tip] = {
                "translation_error_m": translation_error(ctrl.matrix, tip_pose.matrix),
                "rotation_error_rad": rotation_geodesic(
                    ctrl.matrix[:3, :3], tip_pose.matrix[:3, :3]
                ),
                "urdf_translation_m": tip_pose.translation().tolist(),
                "controller_translation_m": ctrl.translation().tolist(),
            }
        # Select closest translation match
        best = min(
            out["residuals_vs_urdf"].items(),
            key=lambda kv: kv[1]["translation_error_m"],
        )
        out["closest_urdf_frame_by_translation"] = best[0]
        out["closest_translation_error_m"] = best[1]["translation_error_m"]
        return out
