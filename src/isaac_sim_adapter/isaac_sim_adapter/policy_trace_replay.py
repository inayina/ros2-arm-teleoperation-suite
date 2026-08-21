"""Validated replay contract for recorded authoritative absolute-EEF traces.

The helpers are ROS-free.  They intentionally replay only ``bounded_action``
rows that were actually ``EXECUTED``; raw model output, rejected commands, and
task-success claims are never accepted as control input.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Sequence


TRACE_CONTRACT_VERSION = 'policy_trace_prefix_replay_v1'


@dataclass(frozen=True)
class ReplayAction:
    index: int
    emitted_monotonic_ns: int
    relative_s: float
    bounded_action: tuple[float, ...]
    source_observation_sequence: int
    source_chunk_index: int
    clipped: bool


@dataclass(frozen=True)
class ReplayTrace:
    source_path: str
    source_sha256: str
    actions: tuple[ReplayAction, ...]

    @property
    def duration_s(self) -> float:
        return 0.0 if not self.actions else self.actions[-1].relative_s

    @property
    def clipped_count(self) -> int:
        return sum(int(action.clipped) for action in self.actions)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_executed_prefix(path: Path, count: int) -> ReplayTrace:
    """Load exactly ``count`` sequential EXECUTED absolute-EEF8 commands."""
    source = path.resolve()
    if count <= 0:
        raise ValueError('prefix count must be positive')
    rows: list[dict] = []
    try:
        for line_number, line in enumerate(
            source.read_text(encoding='utf-8').splitlines(), start=1
        ):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f'{source}:{line_number}: invalid JSON: {exc}'
                ) from exc
            if not isinstance(row, dict):
                raise ValueError(f'{source}:{line_number}: JSON object required')
            rows.append(row)
    except OSError as exc:
        raise ValueError(f'cannot read action trace {source}: {exc}') from exc
    if len(rows) < count:
        raise ValueError(f'trace has {len(rows)} rows, need prefix count {count}')

    parsed: list[ReplayAction] = []
    first_ns: int | None = None
    previous_ns: int | None = None
    for expected_index, row in enumerate(rows[:count]):
        if row.get('decision') != 'EXECUTED':
            raise ValueError(f'action {expected_index} was not EXECUTED')
        if row.get('shadow_only') is True:
            raise ValueError(f'action {expected_index} is shadow-only')
        if row.get('claims_task_success') is not False:
            raise ValueError(f'action {expected_index} has invalid task-success claim')
        if int(row.get('index', -1)) != expected_index:
            raise ValueError(
                f'action index mismatch: expected {expected_index}, got {row.get("index")}'
            )
        values = row.get('bounded_action')
        if not isinstance(values, list) or len(values) != 8:
            raise ValueError(f'action {expected_index} requires bounded_action[8]')
        action = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in action):
            raise ValueError(f'action {expected_index} contains non-finite value')
        if not 0.0 <= action[7] <= 1.0:
            raise ValueError(f'action {expected_index} bounded gripper outside [0,1]')
        emitted_ns = int(row.get('command_emitted_monotonic_ns', 0))
        if emitted_ns <= 0:
            raise ValueError(f'action {expected_index} has invalid emission timestamp')
        if previous_ns is not None and emitted_ns <= previous_ns:
            raise ValueError(f'action {expected_index} emission timestamp regressed')
        if first_ns is None:
            first_ns = emitted_ns
        parsed.append(
            ReplayAction(
                index=expected_index,
                emitted_monotonic_ns=emitted_ns,
                relative_s=(emitted_ns - first_ns) / 1_000_000_000.0,
                bounded_action=action,
                source_observation_sequence=int(row.get('observation_sequence', -1)),
                source_chunk_index=int(row.get('chunk_index', -1)),
                clipped=bool(row.get('action_clipped', False)),
            )
        )
        previous_ns = emitted_ns
    return ReplayTrace(
        source_path=str(source),
        source_sha256=_sha256(source),
        actions=tuple(parsed),
    )


def quantile_prefix_counts(total: int, quantiles: Sequence[float]) -> tuple[int, ...]:
    """Return unique 1-based prefix counts for policy-path coverage."""
    if total <= 0:
        raise ValueError('total must be positive')
    counts: list[int] = []
    for quantile in quantiles:
        value = float(quantile)
        if not 0.0 < value <= 1.0:
            raise ValueError('quantiles must be in (0,1]')
        count = max(1, min(total, int(math.ceil(total * value))))
        if count not in counts:
            counts.append(count)
    return tuple(counts)
