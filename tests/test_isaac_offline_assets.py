"""Pure tests for offline Isaac asset path resolution (no Isaac runtime)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'isaac_sim_adapter'))

from isaac_sim_adapter.offline_assets import (  # noqa: E402
    FRANKA_USD_RELATIVE,
    FRANKA_USD_REMOTE,
    franka_offline_download_hint,
    resolve_franka_usd_path,
    validate_franka_usd_path,
)


def test_resolve_prefers_cli_over_env(monkeypatch, tmp_path):
    usd = tmp_path / 'franka.usd'
    usd.write_text('stub')
    other = tmp_path / 'other.usd'
    other.write_text('stub')
    monkeypatch.setenv('ISAAC_FRANKA_USD', str(other))
    assert resolve_franka_usd_path(str(usd)) == str(usd.resolve())


def test_resolve_uses_env_when_cli_empty(monkeypatch, tmp_path):
    usd = tmp_path / 'franka.usd'
    usd.write_text('stub')
    monkeypatch.setenv('ISAAC_FRANKA_USD', str(usd))
    assert resolve_franka_usd_path('') == str(usd.resolve())


def test_resolve_none_when_unset(monkeypatch):
    monkeypatch.delenv('ISAAC_FRANKA_USD', raising=False)
    assert resolve_franka_usd_path('') is None
    assert resolve_franka_usd_path(None, env_value='') is None


def test_resolve_keeps_remote_urls():
    url = 'https://example.com/franka.usd'
    assert resolve_franka_usd_path(url) == url


def test_validate_local_missing_raises(tmp_path):
    missing = tmp_path / 'missing.usd'
    with pytest.raises(FileNotFoundError, match='Franka USD not found'):
        validate_franka_usd_path(str(missing))


def test_validate_local_ok(tmp_path):
    usd = tmp_path / 'franka.usd'
    usd.write_text('stub')
    assert validate_franka_usd_path(str(usd)) == str(usd.resolve())


def test_download_hint_mentions_nucleus_relative_path():
    hint = franka_offline_download_hint('/tmp/isaac_assets')
    assert FRANKA_USD_RELATIVE in hint
    assert FRANKA_USD_REMOTE in hint
    assert 'ISAAC_FRANKA_USD=' in hint


def test_offline_scene_light_spec_frozen():
    from isaac_sim_adapter.offline_assets import (
        OFFLINE_DISTANT_INTENSITY,
        OFFLINE_DOME_INTENSITY,
        offline_scene_light_spec,
    )

    spec = offline_scene_light_spec()
    assert spec['dome_path'] == '/World/OfflineDomeLight'
    assert spec['distant_path'] == '/World/OfflineDistantLight'
    assert spec['dome_intensity'] == OFFLINE_DOME_INTENSITY
    assert spec['distant_intensity'] == OFFLINE_DISTANT_INTENSITY
    assert len(spec['distant_rotate_xyz_deg']) == 3
    assert 'FixedCuboid' in str(spec['reason'])
    assert OFFLINE_DOME_INTENSITY > 0
    assert OFFLINE_DISTANT_INTENSITY > 0
