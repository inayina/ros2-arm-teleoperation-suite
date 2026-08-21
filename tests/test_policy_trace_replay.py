from __future__ import annotations

import json
from pathlib import Path

import pytest

from isaac_sim_adapter.policy_trace_replay import (
    load_executed_prefix,
    quantile_prefix_counts,
)


def _write_trace(path: Path, *, rows: int = 5) -> None:
    payload = []
    for index in range(rows):
        payload.append(
            {
                'index': index,
                'decision': 'EXECUTED',
                'shadow_only': False,
                'claims_task_success': False,
                'bounded_action': [0.3 + index * 0.01, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0, 1.0],
                'command_emitted_monotonic_ns': 1_000_000_000 + index * 125_000_000,
                'observation_sequence': index // 2,
                'chunk_index': index % 5,
                'action_clipped': False,
            }
        )
    path.write_text(
        ''.join(json.dumps(row) + '\n' for row in payload), encoding='utf-8'
    )


def test_load_executed_prefix_preserves_recorded_timing(tmp_path: Path) -> None:
    path = tmp_path / 'actions.jsonl'
    _write_trace(path)
    trace = load_executed_prefix(path, 4)
    assert len(trace.actions) == 4
    assert trace.actions[0].relative_s == 0.0
    assert trace.actions[-1].relative_s == pytest.approx(0.375)
    assert trace.actions[-1].bounded_action[0] == pytest.approx(0.33)
    assert trace.source_sha256


def test_load_executed_prefix_rejects_nonexecuted_row(tmp_path: Path) -> None:
    path = tmp_path / 'actions.jsonl'
    _write_trace(path)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[2]['decision'] = 'HELD'
    path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
    with pytest.raises(ValueError, match='not EXECUTED'):
        load_executed_prefix(path, 4)


def test_quantile_prefix_counts() -> None:
    assert quantile_prefix_counts(100, (0.25, 0.5, 0.75, 1.0)) == (25, 50, 75, 100)
