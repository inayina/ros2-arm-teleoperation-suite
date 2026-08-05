"""Fail-closed phase and failure-onset timeline for continuous Task GT.

The timeline records what the upstream evaluator knew at each monotonic time.
Terminal non-achievement is explicitly interval/right-censored: its behavioral
onset is not retroactively invented at episode finalization.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from synth_data_gen.continuous_evaluator import ContinuousTaskEvaluator
from synth_data_gen.task_gt_live import TaskGtSnapshot


CONTRACT_VERSION = 'panda_task_timeline_v1'
PHASE_SEMANTICS = 'continuous_gt_achieved_subgoal_frontier'


@dataclass
class TaskTimelineTracker:
    """Stateful transition/onset encoder; one instance is scoped to one episode."""

    trace_run_id: str
    episode_id: str
    _last_phase: Optional[str] = None
    _failure: Optional[dict[str, Any]] = None

    @staticmethod
    def _observable_failure(evaluator: ContinuousTaskEvaluator) -> Optional[str]:
        for active, event_type in (
            (evaluator._estop_triggered, 'estop'),
            (evaluator._drop_detected, 'drop_after_lift'),
            (evaluator._slip_detected, 'slip_during_hold'),
            (evaluator._timeout, 'policy_timeout'),
        ):
            if active:
                return event_type
        return None

    def encode(
        self,
        snapshot: TaskGtSnapshot,
        evaluator: ContinuousTaskEvaluator,
        *,
        event_sequence: int,
        monotonic_ns: int,
        ros_time_ns: int,
        final_failure_stage: Optional[str] = None,
        final_failure_reason: Optional[str] = None,
    ) -> dict[str, Any]:
        phase_from = self._last_phase
        phase_changed = phase_from is not None and phase_from != snapshot.phase
        self._last_phase = snapshot.phase

        first_onset = False
        observed = self._observable_failure(evaluator)
        if self._failure is None and observed is not None:
            first_onset = True
            self._failure = {
                'kind': 'observed_event',
                'event_type': observed,
                'stage': 'system' if observed in {'estop', 'policy_timeout'} else 'lift',
                'reason': observed,
                'onset_monotonic_ns': int(monotonic_ns),
                'onset_event_sequence': int(event_sequence),
                'onset_is_exact': True,
            }
        elif (
            self._failure is None
            and snapshot.task_status == 'FAIL'
            and snapshot.validity == 'VALID'
        ):
            # A missing subgoal is only known to have failed at the terminal
            # deadline. Do not backdate it to an arbitrary earlier frame.
            first_onset = True
            self._failure = {
                'kind': 'terminal_nonachievement',
                'event_type': 'terminal_nonachievement',
                'stage': final_failure_stage,
                'reason': final_failure_reason,
                'onset_monotonic_ns': int(monotonic_ns),
                'onset_event_sequence': int(event_sequence),
                'onset_is_exact': False,
            }

        if snapshot.validity != 'VALID':
            failure = {
                'kind': 'unavailable',
                'event_type': None,
                'stage': None,
                'reason': snapshot.reason_code,
                'onset_monotonic_ns': None,
                'onset_event_sequence': None,
                'onset_is_exact': False,
                'first_onset_in_row': False,
            }
        elif self._failure is None:
            failure = {
                'kind': 'none_observed',
                'event_type': None,
                'stage': None,
                'reason': None,
                'onset_monotonic_ns': None,
                'onset_event_sequence': None,
                'onset_is_exact': False,
                'first_onset_in_row': False,
            }
        else:
            failure = dict(self._failure)
            failure['first_onset_in_row'] = first_onset

        return {
            'contract_version': CONTRACT_VERSION,
            'artifact_type': 'task_timeline_sample',
            'trace_run_id': self.trace_run_id,
            'episode_id': self.episode_id,
            'event_sequence': int(event_sequence),
            'monotonic_ns': int(monotonic_ns),
            'ros_time_ns': int(ros_time_ns),
            'validity': snapshot.validity,
            'reason_code': snapshot.reason_code,
            'task_status': snapshot.task_status,
            'phase': snapshot.phase,
            'phase_source': 'upstream_continuous_task_evaluator',
            'phase_semantics': PHASE_SEMANTICS,
            'phase_transition': {
                'changed': phase_changed,
                'from': phase_from,
                'to': snapshot.phase,
            },
            'subgoals': {
                'reach': snapshot.reach,
                'grasp': snapshot.grasp,
                'lift': snapshot.lift,
                'place': snapshot.place,
            },
            'object_delta_m': (
                None if snapshot.object_delta_m is None else list(snapshot.object_delta_m)
            ),
            'failure_onset': failure,
            'claims_task_success': False,
            'claims_causal_proof': False,
        }
