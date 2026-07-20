"""Continuous runtime ground-truth evaluator for Panda pick/place episodes.

Ownership: ros2-arm-teleoperation-suite (Evaluation Contract runtime_ground_truth).
Produces episode_result rows compatible with midstream
`evaluation/schemas/episode_result.schema.json`.

This module is ROS-free so unit tests can drive synthetic traces without a stack.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Mapping, MutableMapping, Optional, Sequence, Tuple

EVALUATOR_ID = "panda_continuous_gt_v0"
EVALUATOR_VERSION = "0.1.0"
OWNER_REPOSITORY = "ros2-arm-teleoperation-suite"

XYZ = Tuple[float, float, float]


@dataclass
class EvaluatorSample:
    """One streaming observation used by the continuous evaluator."""

    t_monotonic: float
    object_xyz: Optional[XYZ] = None
    ee_xyz: Optional[XYZ] = None
    gripper: Optional[float] = None  # 0=closed, 1=open
    grasp_active: Optional[bool] = None
    contact_force_n: Optional[float] = None
    estop: Optional[bool] = None
    ee_cmd_xyz: Optional[XYZ] = None
    phase_hint: Optional[str] = None


@dataclass
class ContinuousTaskEvaluator:
    """Streaming lift/place/contact evaluator with subgoal funnel."""

    lift_success_delta: float = 0.03
    bin_xy_tolerance: float = 0.08
    gripper_close_max: float = 0.12
    reach_xy_tolerance: float = 0.05
    grasp_xy_tolerance: float = 0.04
    ee_object_hold_xy_tolerance: float = 0.06
    drop_z_tolerance: float = 0.015
    slip_xy_tolerance: float = 0.05
    validation_mode: str = "place"  # lift | place

    _reset_monotonic_s: Optional[float] = field(default=None, init=False, repr=False)
    _first_valid_state_monotonic_s: Optional[float] = field(
        default=None, init=False, repr=False
    )
    _start_monotonic_s: Optional[float] = field(default=None, init=False, repr=False)
    _end_monotonic_s: Optional[float] = field(default=None, init=False, repr=False)
    _initial_object_xyz: Optional[XYZ] = field(default=None, init=False, repr=False)
    _bin_xy: Optional[Tuple[float, float]] = field(default=None, init=False, repr=False)
    _phase: Optional[str] = field(default=None, init=False, repr=False)

    _peak_object_z: Optional[float] = field(default=None, init=False, repr=False)
    _peak_object_z_while_held: Optional[float] = field(
        default=None, init=False, repr=False
    )
    _gripper_was_closed: bool = field(default=False, init=False, repr=False)
    _reach: bool = field(default=False, init=False, repr=False)
    _grasp: bool = field(default=False, init=False, repr=False)
    _lift: bool = field(default=False, init=False, repr=False)
    _transport: bool = field(default=False, init=False, repr=False)
    _place: bool = field(default=False, init=False, repr=False)
    _release: bool = field(default=False, init=False, repr=False)
    _drop_detected: bool = field(default=False, init=False, repr=False)
    _slip_detected: bool = field(default=False, init=False, repr=False)
    _estop_triggered: bool = field(default=False, init=False, repr=False)
    _collision_count: int = field(default=0, init=False, repr=False)
    _peak_force_n: float = field(default=0.0, init=False, repr=False)
    _ee_track_errors: list[float] = field(default_factory=list, init=False, repr=False)
    _ee_path_length_m: float = field(default=0.0, init=False, repr=False)
    _prev_ee_xyz: Optional[XYZ] = field(default=None, init=False, repr=False)
    _sample_count: int = field(default=0, init=False, repr=False)
    # Grasp-time object−EE XY offset; slip = drift of that relative hold, not
    # absolute object travel during intentional transport.
    _held_obj_ee_xy_offset: Optional[Tuple[float, float]] = field(
        default=None, init=False, repr=False
    )
    _timeout: bool = field(default=False, init=False, repr=False)
    _abort_reason: Optional[str] = field(default=None, init=False, repr=False)
    _fail_safe_events: list[dict[str, Any]] = field(
        default_factory=list, init=False, repr=False
    )

    def reset(
        self,
        *,
        initial_object_xyz: Optional[XYZ],
        bin_xy: Tuple[float, float],
        reset_monotonic_s: Optional[float] = None,
    ) -> None:
        now = time.monotonic()
        self._reset_monotonic_s = (
            float(reset_monotonic_s) if reset_monotonic_s is not None else now
        )
        self._first_valid_state_monotonic_s = None
        self._start_monotonic_s = self._reset_monotonic_s
        self._end_monotonic_s = None
        self._initial_object_xyz = (
            tuple(float(v) for v in initial_object_xyz)  # type: ignore[assignment]
            if initial_object_xyz is not None
            else None
        )
        self._bin_xy = (float(bin_xy[0]), float(bin_xy[1]))
        self._phase = "reset"
        self._peak_object_z = (
            float(initial_object_xyz[2]) if initial_object_xyz is not None else None
        )
        self._peak_object_z_while_held = None
        self._gripper_was_closed = False
        self._reach = False
        self._grasp = False
        self._lift = False
        self._transport = False
        self._place = False
        self._release = False
        self._drop_detected = False
        self._slip_detected = False
        self._estop_triggered = False
        self._collision_count = 0
        self._peak_force_n = 0.0
        self._ee_track_errors.clear()
        self._ee_path_length_m = 0.0
        self._prev_ee_xyz = None
        self._sample_count = 0
        self._held_obj_ee_xy_offset = None
        self._timeout = False
        self._abort_reason = None
        self._fail_safe_events.clear()

    def set_phase(self, phase: str) -> None:
        self._phase = str(phase)

    def mark_timeout(self, reason: str = "policy_timeout") -> None:
        self._timeout = True
        self._abort_reason = reason
        self._fail_safe_events.append(
            {
                "condition": "policy_timeout",
                "response": "hold_then_abort",
                "monotonic_timestamp_s": time.monotonic(),
                "details": reason,
            }
        )

    def observe(self, sample: EvaluatorSample) -> None:
        self._sample_count += 1
        self._end_monotonic_s = float(sample.t_monotonic)
        if sample.phase_hint:
            self._phase = sample.phase_hint

        if sample.estop:
            self._estop_triggered = True

        if sample.contact_force_n is not None and math.isfinite(sample.contact_force_n):
            force = float(sample.contact_force_n)
            self._peak_force_n = max(self._peak_force_n, force)
            if force > 80.0:
                self._collision_count += 1

        gripper = sample.gripper
        closed = False
        if sample.grasp_active is True:
            closed = True
        elif gripper is not None and math.isfinite(gripper):
            closed = float(gripper) <= self.gripper_close_max
        if closed:
            self._gripper_was_closed = True

        obj = sample.object_xyz
        ee = sample.ee_xyz
        if obj is not None and all(math.isfinite(v) for v in obj):
            if self._first_valid_state_monotonic_s is None:
                self._first_valid_state_monotonic_s = float(sample.t_monotonic)
            z = float(obj[2])
            self._peak_object_z = (
                z if self._peak_object_z is None else max(self._peak_object_z, z)
            )
            if closed and ee is not None and all(math.isfinite(v) for v in ee):
                hold_xy = math.hypot(ee[0] - obj[0], ee[1] - obj[1])
                if hold_xy <= self.ee_object_hold_xy_tolerance:
                    self._peak_object_z_while_held = (
                        z
                        if self._peak_object_z_while_held is None
                        else max(self._peak_object_z_while_held, z)
                    )
                    offset = (float(obj[0] - ee[0]), float(obj[1] - ee[1]))
                    if self._held_obj_ee_xy_offset is None:
                        self._held_obj_ee_xy_offset = offset
                    else:
                        slip = math.hypot(
                            offset[0] - self._held_obj_ee_xy_offset[0],
                            offset[1] - self._held_obj_ee_xy_offset[1],
                        )
                        if slip > self.slip_xy_tolerance and self._lift:
                            self._slip_detected = True

        if ee is not None and all(math.isfinite(v) for v in ee):
            if self._prev_ee_xyz is not None:
                self._ee_path_length_m += math.dist(self._prev_ee_xyz, ee)
            self._prev_ee_xyz = ee
            if sample.ee_cmd_xyz is not None and all(
                math.isfinite(v) for v in sample.ee_cmd_xyz
            ):
                self._ee_track_errors.append(math.dist(ee, sample.ee_cmd_xyz))

        self._update_subgoals(obj=obj, ee=ee, closed=closed, gripper=gripper)

    def _update_subgoals(
        self,
        *,
        obj: Optional[XYZ],
        ee: Optional[XYZ],
        closed: bool,
        gripper: Optional[float],
    ) -> None:
        if obj is None or self._initial_object_xyz is None or self._bin_xy is None:
            return
        init = self._initial_object_xyz
        if ee is not None and all(math.isfinite(v) for v in ee):
            ee_xy = math.hypot(ee[0] - init[0], ee[1] - init[1])
            if ee_xy <= self.reach_xy_tolerance:
                self._reach = True
            obj_ee_xy = math.hypot(ee[0] - obj[0], ee[1] - obj[1])
            if closed and obj_ee_xy <= self.grasp_xy_tolerance:
                self._grasp = True

        # Continuous lift: peak height while held, not endpoint-only.
        held_peak = self._peak_object_z_while_held
        if held_peak is None and self._gripper_was_closed:
            # Fallback: overall peak after a close (batch gate parity).
            held_peak = self._peak_object_z
        if (
            held_peak is not None
            and self._gripper_was_closed
            and (held_peak - init[2]) + 1e-3 >= self.lift_success_delta
        ):
            self._lift = True

        if self._lift:
            # Drop: after a successful lift, object falls near table while open.
            if (
                gripper is not None
                and float(gripper) > self.gripper_close_max
                and obj[2] <= init[2] + self.drop_z_tolerance
                and held_peak is not None
                and held_peak - obj[2] >= self.lift_success_delta * 0.5
            ):
                self._drop_detected = True

            moved_toward_bin = math.hypot(
                obj[0] - self._bin_xy[0], obj[1] - self._bin_xy[1]
            ) < math.hypot(init[0] - self._bin_xy[0], init[1] - self._bin_xy[1]) - 0.02
            if moved_toward_bin or self._place:
                self._transport = True

        bin_dist = math.hypot(obj[0] - self._bin_xy[0], obj[1] - self._bin_xy[1])
        if self._lift and bin_dist <= self.bin_xy_tolerance:
            self._place = True
            self._transport = True

        if self._place and gripper is not None and float(gripper) > 0.5:
            self._release = True

    def _motion_stats(self) -> dict[str, Optional[float]]:
        errs = self._ee_track_errors
        if not errs:
            return {
                "completion_time_s": self._completion_time_s(),
                "ee_tracking_rmse_m": None,
                "ee_tracking_p95_m": None,
                "ee_tracking_max_m": None,
                "path_length_m": self._ee_path_length_m if self._sample_count else None,
                "smoothness_jerk_rms": None,
            }
        rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
        ordered = sorted(errs)
        p95 = ordered[min(len(ordered) - 1, int(math.ceil(0.95 * len(ordered)) - 1))]
        return {
            "completion_time_s": self._completion_time_s(),
            "ee_tracking_rmse_m": rmse,
            "ee_tracking_p95_m": p95,
            "ee_tracking_max_m": max(errs),
            "path_length_m": self._ee_path_length_m,
            "smoothness_jerk_rms": None,
        }

    def _completion_time_s(self) -> Optional[float]:
        if self._start_monotonic_s is None or self._end_monotonic_s is None:
            return None
        return max(0.0, self._end_monotonic_s - self._start_monotonic_s)

    def _infer_failure_stage(self, success: bool) -> Optional[str]:
        if success:
            return None
        if self._estop_triggered or self._abort_reason:
            return "system"
        if not self._reach:
            return "reach"
        if not self._grasp:
            return "grasp"
        if not self._lift or self._drop_detected or self._slip_detected:
            return "lift"
        mode = self.validation_mode.lower()
        if mode == "lift":
            return "lift"
        if not self._transport:
            return "transport"
        if not self._place:
            return "place"
        if not self._release:
            return "release"
        return "place"

    def evaluate_success(self) -> tuple[bool, str]:
        if self._initial_object_xyz is None:
            return False, "missing privileged object pose"
        if self._estop_triggered:
            return False, "estop_triggered"
        if self._drop_detected:
            return False, "drop_detected_after_lift"
        if self._slip_detected:
            return False, "slip_detected_during_hold"
        if not self._gripper_was_closed:
            return False, f"gripper never closed below {self.gripper_close_max:.3f}"
        if not self._lift:
            peak = self._peak_object_z_while_held or self._peak_object_z
            delta = (
                None
                if peak is None
                else float(peak) - float(self._initial_object_xyz[2])
            )
            return False, f"lift_failed delta={delta}"
        mode = self.validation_mode.lower()
        if mode == "lift":
            return True, "lift validation passed"
        if mode == "place":
            if not self._place:
                return False, "place_failed bin proximity"
            return True, "place validation passed"
        return False, f"unknown validation_mode={self.validation_mode!r}"

    def finalize(
        self,
        *,
        evaluation_run_id: str,
        identity: Mapping[str, Any],
        evidence: Mapping[str, Optional[str]],
        execution_status: str = "completed",
        ros_start_s: Optional[float] = None,
        ros_end_s: Optional[float] = None,
        simulation_start_s: Optional[float] = None,
        simulation_end_s: Optional[float] = None,
        system_performance: Optional[Mapping[str, Any]] = None,
        data_health: Optional[Mapping[str, Any]] = None,
    ) -> dict[str, Any]:
        if execution_status == "infrastructure_failure":
            success: Optional[bool] = None
            timeout: Optional[bool] = None
            reason: Optional[str] = self._abort_reason or "infrastructure_failure"
            runtime_evaluated = False
            ground_truth_source = "not_evaluated"
            evidence_level = "runtime_observed"
        elif execution_status == "aborted":
            success = False
            timeout = bool(self._timeout)
            reason = self._abort_reason or "aborted"
            runtime_evaluated = True
            ground_truth_source = "runtime_ground_truth"
            evidence_level = "runtime_observed"
        else:
            execution_status = "completed"
            ok, reason = self.evaluate_success()
            success = bool(ok)
            timeout = bool(self._timeout)
            runtime_evaluated = True
            ground_truth_source = "runtime_ground_truth"
            evidence_level = "runtime_observed"

        motion = self._motion_stats()
        null_perf = {k: None for k in _SYSTEM_PERFORMANCE_KEYS}
        if system_performance:
            null_perf.update(dict(system_performance))
        null_health = {k: None for k in _DATA_HEALTH_KEYS}
        if data_health:
            null_health.update(dict(data_health))

        row: dict[str, Any] = {
            "contract_version": "evaluation_contract_v0",
            "artifact_type": "episode_result",
            "evaluation_run_id": str(evaluation_run_id),
            "execution_status": execution_status,
            "evidence_level": evidence_level,
            "identity": {
                "model_id": str(identity["model_id"]),
                "backend": str(identity["backend"]),
                "scene_id": str(identity["scene_id"]),
                "suite_id": str(identity["suite_id"]),
                "seed": int(identity["seed"]),
                "episode_index": int(identity["episode_index"]),
            },
            "timestamps": {
                "simulation_start_s": simulation_start_s,
                "simulation_end_s": simulation_end_s,
                "ros_start_s": ros_start_s,
                "ros_end_s": ros_end_s,
                "monotonic_start_s": self._start_monotonic_s,
                "monotonic_end_s": self._end_monotonic_s,
                "reset_completed_monotonic_s": self._reset_monotonic_s,
                "first_valid_state_monotonic_s": self._first_valid_state_monotonic_s,
            },
            "outcome": {
                "runtime_evaluated": runtime_evaluated,
                "evaluator": {
                    "owner_repository": OWNER_REPOSITORY,
                    "evaluator_id": EVALUATOR_ID if runtime_evaluated else None,
                    "evaluator_version": EVALUATOR_VERSION if runtime_evaluated else None,
                    "ground_truth_source": ground_truth_source,
                },
                "success": success,
                "timeout": timeout,
                "failure_stage": self._infer_failure_stage(bool(success))
                if success is not None
                else None,
                "failure_reason": None if success else reason,
            },
            "subgoals": {
                "reach": self._reach if runtime_evaluated else None,
                "grasp": self._grasp if runtime_evaluated else None,
                "lift": self._lift if runtime_evaluated else None,
                "transport": self._transport if runtime_evaluated else None,
                "place": self._place if runtime_evaluated else None,
                "release": self._release if runtime_evaluated else None,
            },
            "motion": motion,
            "contact_safety": {
                "collision_count": self._collision_count if runtime_evaluated else None,
                "drop_detected": self._drop_detected if runtime_evaluated else None,
                "slip_detected": self._slip_detected if runtime_evaluated else None,
                "peak_force_n": self._peak_force_n if runtime_evaluated else None,
                "peak_torque_nm": None,
                "joint_limit_event_count": None,
                "estop_triggered": self._estop_triggered if runtime_evaluated else None,
            },
            "data_health": null_health,
            "system_performance": null_perf,
            "fail_safe_events": list(self._fail_safe_events),
            "evidence": {
                "raw_episode_path": evidence.get("raw_episode_path"),
                "video_path": evidence.get("video_path"),
                "runtime_log_path": evidence.get("runtime_log_path"),
                "event_log_path": evidence.get("event_log_path"),
                "nfr_sample_path": evidence.get("nfr_sample_path"),
            },
        }
        return row


_SYSTEM_PERFORMANCE_KEYS = (
    "physics_fps",
    "real_time_factor",
    "cpu_percent",
    "rss_mb",
    "gpu_percent",
    "vram_mb",
    "frame_time_p95_ms",
    "command_age_p50_ms",
    "command_age_p95_ms",
    "command_age_max_ms",
    "state_age_p50_ms",
    "state_age_p95_ms",
    "state_age_max_ms",
    "control_frequency_hz",
    "state_frequency_hz",
    "watchdog_latency_ms",
    "reset_recovery_ms",
)

_DATA_HEALTH_KEYS = (
    "missing_frame_count",
    "stale_state_count",
    "stale_command_count",
    "reused_frame_count",
    "dropped_frame_count",
    "recorder_hz",
)


def append_episode_result(path: str, row: Mapping[str, Any]) -> None:
    """Append one episode_result JSON object as a JSONL line."""
    import json
    from pathlib import Path

    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, sort_keys=False, ensure_ascii=True) + "\n")
