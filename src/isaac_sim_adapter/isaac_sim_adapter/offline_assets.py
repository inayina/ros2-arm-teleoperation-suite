"""Resolve local Isaac assets without Nucleus for offline scene bootstrap.

``Franka()`` and ``add_default_ground_plane()`` both call
``get_assets_root_path()``, which blocks on Nucleus ``check_server``.
Ground can be replaced with a local ``FixedCuboid``. The Franka robot still
needs a USD: set ``ISAAC_FRANKA_USD`` / ``--franka-usd`` to a local file
(preferred) or a direct URL. Hashed OV client cache entries under
``~/.cache/ov/client/https/`` are *not* a usable Franka tree (relative
payloads).
"""

from __future__ import annotations

import os
from pathlib import Path

# Matches isaacsim.storage.native extension.toml for this Isaac 6.0 install.
DEFAULT_ISAAC_ASSETS_ROOT = (
    'https://omniverse-content-staging.s3-us-west-2.amazonaws.com/'
    'Assets/Isaac/6.0'
)
FRANKA_USD_RELATIVE = 'Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd'
FRANKA_USD_REMOTE = f'{DEFAULT_ISAAC_ASSETS_ROOT}/{FRANKA_USD_RELATIVE}'
FRANKA_FOLDER_REMOTE = (
    f'{DEFAULT_ISAAC_ASSETS_ROOT}/Isaac/Robots/FrankaRobotics/FrankaPanda'
)


def resolve_franka_usd_path(
    cli_value: str | None = None,
    env_value: str | None = None,
) -> str | None:
    """Return Franka USD path from CLI, else env, else None (Nucleus fallback).

    Empty strings are ignored. Local paths are expanded; ``http(s)://`` /
    ``omniverse://`` / ``file://`` URLs are returned unchanged.
    """
    if cli_value is None:
        cli_value = ''
    if env_value is None:
        env_value = os.environ.get('ISAAC_FRANKA_USD', '')
    raw = str(cli_value).strip() or str(env_value).strip()
    if not raw:
        return None
    if '://' in raw:
        return raw
    return str(Path(raw).expanduser().resolve())


def validate_franka_usd_path(path: str) -> str:
    """Ensure a local filesystem USD exists; leave remote URLs unchecked."""
    if path.startswith(('http://', 'https://', 'omniverse://')):
        return path
    local = path
    if path.startswith('file://'):
        local = path[len('file://'):]
    resolved = Path(local).expanduser()
    if not resolved.is_file():
        raise FileNotFoundError(
            f'Franka USD not found: {resolved}. '
            f'Download the FrankaPanda folder once (see franka_offline_download_hint) '
            f'and set ISAAC_FRANKA_USD / --franka-usd.'
        )
    return str(resolved.resolve())


def franka_offline_download_hint(dest_root: str | Path | None = None) -> str:
    """One-shot download instructions when the network is briefly available."""
    dest = Path(dest_root or Path.home() / 'isaac_assets').expanduser()
    panda = dest / 'Isaac' / 'Robots' / 'FrankaRobotics' / 'FrankaPanda'
    return (
        'One-time Franka USD download (Isaac 6.0 staging; needs network):\n'
        f'  mkdir -p {panda}\n'
        f'  # Prefer the whole FrankaPanda folder (root USD has relative payloads):\n'
        f'  aws s3 sync --no-sign-request \\\n'
        f'    s3://omniverse-content-staging/Assets/Isaac/6.0/'
        f'Isaac/Robots/FrankaRobotics/FrankaPanda/ \\\n'
        f'    {panda}/\n'
        f'  # Fallback if aws CLI unavailable (root only; may still need payloads):\n'
        f'  curl -fsSL -o {panda}/franka.usd \\\n'
        f'    {FRANKA_USD_REMOTE}\n'
        f'  export ISAAC_FRANKA_USD={panda}/franka.usd\n'
        f'Nucleus-relative path used by Franka(): /{FRANKA_USD_RELATIVE}\n'
        f'Remote folder: {FRANKA_FOLDER_REMOTE}'
    )
