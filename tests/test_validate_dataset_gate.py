"""Tests for upstream_gate reporting in validate_dataset.py."""

from __future__ import annotations

import json
from pathlib import Path

from scripts import validate_dataset as validator


def test_read_upstream_gate_from_episode_meta(tmp_path: Path) -> None:
    episode_dir = tmp_path / "episode_000001"
    train_dir = episode_dir / "train"
    train_dir.mkdir(parents=True)
    (episode_dir / "meta.json").write_text(
        json.dumps({"upstream_gate": "batch_generator", "success": True}),
        encoding="utf-8",
    )

    assert validator._read_upstream_gate(train_dir) == "batch_generator"


def test_episode_train_dirs_finds_nested_episode_layout(tmp_path: Path) -> None:
    train_dir = tmp_path / "episode_000000" / "train"
    train_dir.mkdir(parents=True)

    found = validator._episode_train_dirs(tmp_path)

    assert found == [train_dir]
