"""Thread-safe latest-effort command gate shared by the Isaac ROS boundary."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Iterable


PANDA_ARM_JOINTS = tuple(f'panda_joint{i}' for i in range(1, 8))
PANDA_TORQUE_LIMITS_NM = (87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0)
ZERO_EFFORT = (0.0,) * len(PANDA_ARM_JOINTS)


@dataclass(frozen=True)
class EffortDecision:
    """One deterministic command-gate decision."""

    efforts: tuple[float, ...]
    status: str
    should_publish: bool
    command_age_s: float | None
    state_age_s: float | None
    clipped: bool = False


def validate_effort_command(
    values: Iterable[float],
    limits: tuple[float, ...] = PANDA_TORQUE_LIMITS_NM,
) -> tuple[tuple[float, ...], bool]:
    """Validate, finite-check and clamp one Panda arm effort command."""
    command = tuple(float(value) for value in values)
    if len(command) != len(limits):
        raise ValueError(
            f'expected {len(limits)} Panda joint efforts, got {len(command)}'
        )
    if not all(math.isfinite(value) for value in command):
        raise ValueError('joint effort command contains NaN or infinity')
    clamped = tuple(
        max(-limit, min(limit, value))
        for value, limit in zip(command, limits)
    )
    return clamped, clamped != command


class LatestEffortCommand:
    """Latest-value command buffer with state, reset and timeout interlocks."""

    def __init__(self, *, command_timeout_s: float, state_timeout_s: float) -> None:
        if command_timeout_s <= 0.0:
            raise ValueError('command_timeout_s must be positive')
        if state_timeout_s <= 0.0:
            raise ValueError('state_timeout_s must be positive')
        self.command_timeout_s = float(command_timeout_s)
        self.state_timeout_s = float(state_timeout_s)
        self._lock = threading.RLock()
        self._latest = ZERO_EFFORT
        self._command_time: float | None = None
        self._state_time: float | None = None
        self._reset_in_progress = False
        self._had_valid_command = False
        self.accepted_count = 0
        self.rejected_count = 0
        self.clipped_count = 0

    def update_state(self, now: float | None = None) -> None:
        """Mark a state sample in the current reset epoch."""
        timestamp = time.monotonic() if now is None else float(now)
        with self._lock:
            if not self._reset_in_progress:
                self._state_time = timestamp

    def begin_reset(self) -> None:
        """Enter the safe reset epoch and discard all prior history."""
        with self._lock:
            self._reset_in_progress = True
            self._clear_history()

    def complete_reset(self) -> None:
        """Leave reset but require a newer state before accepting commands."""
        with self._lock:
            self._clear_history()
            self._reset_in_progress = False

    def reject(self) -> None:
        """Invalidate any prior command after malformed or unsafe input."""
        with self._lock:
            self.rejected_count += 1
            self._latest = ZERO_EFFORT
            self._command_time = None

    def accept(
        self,
        values: Iterable[float],
        now: float | None = None,
    ) -> EffortDecision:
        """Accept only finite, bounded input paired with a fresh post-reset state."""
        timestamp = time.monotonic() if now is None else float(now)
        try:
            command, clipped = validate_effort_command(values)
        except (TypeError, ValueError, OverflowError):
            self.reject()
            raise

        with self._lock:
            state_age = self._age(timestamp, self._state_time)
            if self._reset_in_progress:
                self.reject()
                return self._decision('reset_in_progress', timestamp)
            if state_age is None:
                self.reject()
                return self._decision('state_unavailable', timestamp)
            if state_age > self.state_timeout_s:
                self.reject()
                return self._decision('state_stale', timestamp)

            self._latest = command
            self._command_time = timestamp
            self._had_valid_command = True
            self.accepted_count += 1
            if clipped:
                self.clipped_count += 1
            return EffortDecision(
                efforts=command,
                status='active',
                should_publish=True,
                command_age_s=0.0,
                state_age_s=state_age,
                clipped=clipped,
            )

    def output(self, now: float | None = None) -> EffortDecision:
        """Return latest valid effort or a zero-effort fail-safe decision."""
        timestamp = time.monotonic() if now is None else float(now)
        with self._lock:
            if self._reset_in_progress:
                return self._decision('reset_in_progress', timestamp)
            state_age = self._age(timestamp, self._state_time)
            if state_age is None:
                return self._decision('state_unavailable', timestamp)
            if state_age > self.state_timeout_s:
                self._latest = ZERO_EFFORT
                self._command_time = None
                return self._decision('state_stale', timestamp)
            command_age = self._age(timestamp, self._command_time)
            if command_age is None:
                return self._decision('no_command', timestamp)
            if command_age > self.command_timeout_s:
                self._latest = ZERO_EFFORT
                self._command_time = None
                return EffortDecision(
                    efforts=ZERO_EFFORT,
                    status='command_stale',
                    should_publish=self._had_valid_command,
                    command_age_s=command_age,
                    state_age_s=state_age,
                )
            return EffortDecision(
                efforts=self._latest,
                status='active',
                should_publish=True,
                command_age_s=command_age,
                state_age_s=state_age,
            )

    def snapshot(self, now: float | None = None) -> dict[str, object]:
        """Expose bounded health counters without leaking mutable state."""
        decision = self.output(now)
        with self._lock:
            return {
                'status': decision.status,
                'command_age_s': decision.command_age_s,
                'state_age_s': decision.state_age_s,
                'reset_in_progress': self._reset_in_progress,
                'accepted_count': self.accepted_count,
                'rejected_count': self.rejected_count,
                'clipped_count': self.clipped_count,
            }

    def _clear_history(self) -> None:
        self._latest = ZERO_EFFORT
        self._command_time = None
        self._state_time = None
        self._had_valid_command = False

    @staticmethod
    def _age(now: float, timestamp: float | None) -> float | None:
        if timestamp is None:
            return None
        return max(0.0, now - timestamp)

    def _decision(self, status: str, now: float) -> EffortDecision:
        return EffortDecision(
            efforts=ZERO_EFFORT,
            status=status,
            should_publish=self._had_valid_command,
            command_age_s=self._age(now, self._command_time),
            state_age_s=self._age(now, self._state_time),
        )
