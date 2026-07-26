"""M2/M4 Panda policy execution adapter shared by shadow and authority."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from isaac_sim_adapter.policy_control import absolute_action_to_target_pose
from isaac_sim_adapter.policy_control import action_to_target_pose
from isaac_sim_adapter.policy_control import bound_absolute_eef_gripper
from isaac_sim_adapter.policy_control import bound_ee_delta_gripper
from isaac_sim_adapter.policy_runtime import ABSOLUTE_ACTION_SCHEMA
from isaac_sim_adapter.policy_runtime import ACTION_DIMENSIONS
from isaac_sim_adapter.policy_runtime import classify_runtime_error
from isaac_sim_adapter.policy_runtime import CONTRACT_VERSION
from isaac_sim_adapter.policy_runtime import DELTA_ACTION_SCHEMA
from isaac_sim_adapter.policy_runtime import ShadowPolicyCommand


EXECUTION_ACTION_SCHEMA = 'panda_bounded_pose_gripper_v0'
ADAPTER_NAME = 'panda_policy_execution_adapter_shadow'
ADAPTER_VERSION = 'm2_v1'
AUTHORITATIVE_ADAPTER_NAME = 'panda_policy_execution_adapter_authoritative'
AUTHORITATIVE_ADAPTER_VERSION = 'm4_v1'


def resolve_execution_adapter_mode(
    requested_mode: str, *, shadow_enabled: bool, dry_run: bool
) -> str:
    """Resolve M1 compatibility flag into the explicit M4 mode contract."""
    mode = str(requested_mode).strip().lower()
    if mode not in {'legacy', 'shadow', 'authoritative'}:
        raise ValueError('execution_adapter_mode must be legacy, shadow, or authoritative')
    if shadow_enabled:
        if mode == 'authoritative':
            raise ValueError('shadow compatibility flag conflicts with authoritative mode')
        mode = 'shadow'
    if mode == 'shadow' and not dry_run:
        raise ValueError('shadow execution adapter requires dry_run=true')
    if mode == 'authoritative' and dry_run:
        raise ValueError('authoritative execution adapter requires dry_run=false')
    return mode


def validate_authoritative_publisher_counts(
    pose_publishers: int, gripper_publishers: int
) -> None:
    """Fail closed unless this process is the sole command authority."""
    if int(pose_publishers) != 1 or int(gripper_publishers) != 1:
        raise RuntimeError(
            'authoritative publisher identity mismatch: '
            f'pose={pose_publishers}, gripper={gripper_publishers}'
        )


@dataclass(frozen=True)
class ExecutionState:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class PolicyExecutionDecision:
    event_id: str
    parent_event_id: str
    trace_run_id: str
    episode_id: str
    command_sequence: int
    validity: str
    reason_code: str
    accepted: bool
    decision: str
    source_action_schema_version: str
    source_action: tuple[float, ...]
    bounded_action: tuple[float, ...] | None
    clipped: bool
    clip_axes: tuple[str, ...]
    hold_active: bool
    estop_active: bool
    adapter_name: str = ADAPTER_NAME
    adapter_version: str = ADAPTER_VERSION
    execution_action_schema_version: str = EXECUTION_ACTION_SCHEMA
    claims_task_success: bool = False


class PandaPolicyExecutionAdapter:
    """Validate and convert PolicyCommand into one bounded pose decision."""

    def __init__(
        self,
        *,
        workspace_min: Sequence[float],
        workspace_max: Sequence[float],
        max_translation_m: float = 0.05,
        max_rotation_rad: float = 0.25,
        execution_mode: str = 'shadow',
    ) -> None:
        self.workspace_min = tuple(float(value) for value in workspace_min)
        self.workspace_max = tuple(float(value) for value in workspace_max)
        if len(self.workspace_min) != 3 or len(self.workspace_max) != 3:
            raise ValueError('workspace bounds must have three components')
        self.max_translation_m = float(max_translation_m)
        self.max_rotation_rad = float(max_rotation_rad)
        if execution_mode not in {'shadow', 'authoritative'}:
            raise ValueError('adapter execution_mode must be shadow or authoritative')
        self.execution_mode = execution_mode
        self.adapter_name = (
            ADAPTER_NAME if execution_mode == 'shadow'
            else AUTHORITATIVE_ADAPTER_NAME
        )
        self.adapter_version = (
            ADAPTER_VERSION if execution_mode == 'shadow'
            else AUTHORITATIVE_ADAPTER_VERSION
        )
        self._last_sequence: int | None = None
        self._hold_active = False
        self._estop_active = False

    def reset(self) -> None:
        self._last_sequence = None
        self._hold_active = False
        self._estop_active = False

    def set_hold(self, active: bool) -> None:
        self._hold_active = bool(active)

    def set_estop(self, active: bool) -> None:
        # Clearing is allowed only when the safety monitor publishes reset.
        self._estop_active = bool(active)

    def evaluate(
        self,
        command: ShadowPolicyCommand,
        state: ExecutionState,
        *,
        now_monotonic_ns: int,
    ) -> PolicyExecutionDecision:
        source = tuple(float(value) for value in command.action)
        base = {
            'event_id': (
                f'execution:{command.episode_id}:{command.command_sequence}'
            ),
            'parent_event_id': command.event_id,
            'trace_run_id': command.trace_run_id,
            'episode_id': command.episode_id,
            'command_sequence': command.command_sequence,
            'source_action_schema_version': command.action_schema_version,
            'source_action': source,
            'hold_active': self._hold_active,
            'estop_active': self._estop_active,
            'adapter_name': self.adapter_name,
            'adapter_version': self.adapter_version,
        }
        try:
            self._validate_command(command, source, now_monotonic_ns)
            self._last_sequence = command.command_sequence
            bounded, clipped, clip_axes = self._convert(command, source, state)
        except (ValueError, RuntimeError) as error:
            reason, _lane = classify_runtime_error(error)
            if 'contract version' in str(error).lower():
                reason = 'contract_mismatch'
            elif 'sequence' in str(error).lower():
                reason = 'command_sequence_regression'
            elif 'ttl' in str(error).lower():
                reason = 'command_ttl_expired'
            return PolicyExecutionDecision(
                **base,
                validity='ERROR',
                reason_code=reason,
                accepted=False,
                decision='REJECTED',
                bounded_action=None,
                clipped=False,
                clip_axes=(),
            )

        if self._estop_active:
            return PolicyExecutionDecision(
                **base,
                validity='VALID',
                reason_code='risk_r3_estop',
                accepted=False,
                decision='ESTOPPED',
                bounded_action=bounded,
                clipped=clipped,
                clip_axes=clip_axes,
            )
        if self._hold_active:
            return PolicyExecutionDecision(
                **base,
                validity='VALID',
                reason_code='risk_r2_hold',
                accepted=False,
                decision='HELD',
                bounded_action=bounded,
                clipped=clipped,
                clip_axes=clip_axes,
            )
        return PolicyExecutionDecision(
            **base,
            validity='VALID',
            reason_code=_clip_reason(
                command.action_schema_version, clip_axes
            ),
            accepted=True,
            decision='EXECUTED',
            bounded_action=bounded,
            clipped=clipped,
            clip_axes=clip_axes,
        )

    def _validate_command(
        self,
        command: ShadowPolicyCommand,
        source: tuple[float, ...],
        now_monotonic_ns: int,
    ) -> None:
        if command.contract_version != CONTRACT_VERSION:
            raise ValueError('contract version mismatch')
        expected_dim = ACTION_DIMENSIONS.get(command.action_schema_version)
        if expected_dim is None:
            raise ValueError(
                f'unknown action schema: {command.action_schema_version}'
            )
        if len(source) != expected_dim:
            raise ValueError(
                f'{command.action_schema_version} requires action[{expected_dim}]'
            )
        if not all(math.isfinite(value) for value in source):
            raise ValueError('action contains NaN or infinity')
        if (
            self._last_sequence is not None
            and command.command_sequence <= self._last_sequence
        ):
            raise ValueError('command sequence regression')
        if now_monotonic_ns > command.valid_until_monotonic_ns:
            raise ValueError('command TTL expired')

    def _convert(
        self,
        command: ShadowPolicyCommand,
        source: tuple[float, ...],
        state: ExecutionState,
    ) -> tuple[tuple[float, ...], bool, tuple[str, ...]]:
        if command.action_schema_version == ABSOLUTE_ACTION_SCHEMA:
            result = bound_absolute_eef_gripper(
                source,
                workspace_min=self.workspace_min,
                workspace_max=self.workspace_max,
            )
            axes = _absolute_clip_axes(source, result.values)
            return result.values, result.clipped, axes
        if command.action_schema_version == DELTA_ACTION_SCHEMA:
            delta = bound_ee_delta_gripper(
                source,
                max_translation_m=self.max_translation_m,
                max_rotation_rad=self.max_rotation_rad,
            )
            target = action_to_target_pose(
                state.position,
                state.orientation_xyzw,
                delta.values,
                workspace_min=self.workspace_min,
                workspace_max=self.workspace_max,
            )
            bounded = (*target.position, *target.orientation_xyzw, delta.values[6])
            axes = list(_delta_clip_axes(source, delta.values))
            if target.workspace_clipped:
                axes.append('workspace')
            return bounded, bool(axes), tuple(dict.fromkeys(axes))
        raise ValueError(f'unknown action schema: {command.action_schema_version}')


def _absolute_clip_axes(
    source: Sequence[float], bounded: Sequence[float]
) -> tuple[str, ...]:
    names = ('x', 'y', 'z')
    axes = [names[index] for index in range(3) if source[index] != bounded[index]]
    if source[7] != bounded[7]:
        axes.append('gripper')
    return tuple(axes)


def _delta_clip_axes(
    source: Sequence[float], bounded: Sequence[float]
) -> tuple[str, ...]:
    names = ('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'gripper')
    return tuple(
        name
        for name, raw, limited in zip(names, source, bounded)
        if raw != limited
    )


def _clip_reason(
    action_schema_version: str, clip_axes: Sequence[str]
) -> str:
    """Map bounded dimensions to the most specific frozen M0 reason code."""
    axes = tuple(clip_axes)
    if not axes:
        return 'none'
    if axes == ('gripper',):
        return 'gripper_clipped'
    if action_schema_version == ABSOLUTE_ACTION_SCHEMA:
        return 'workspace_clipped'
    return 'soft_limit'


def legacy_absolute_result(
    action: Sequence[float],
    *,
    workspace_min: Sequence[float],
    workspace_max: Sequence[float],
) -> tuple[tuple[float, ...], tuple[float, ...], float, bool]:
    """Return the unchanged legacy absolute result for parity tests."""
    bounded = bound_absolute_eef_gripper(
        action, workspace_min=workspace_min, workspace_max=workspace_max
    )
    target = absolute_action_to_target_pose(bounded.values)
    return (
        target.position,
        target.orientation_xyzw,
        bounded.values[7],
        bounded.clipped,
    )
