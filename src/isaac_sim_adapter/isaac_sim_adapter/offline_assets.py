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

# FixedCuboid offline ground replaces Nucleus default_environment.usd, which
# normally supplies dome lighting. Without explicit lights the ROS camera feed
# is nearly black (S4 telemetry 2026-07-24: policy JPEG mean ≈ 0.3/255).
# Intensities tuned toward oracle/MuJoCo usable range (not washed-out white):
# lightfix smoke1 @ 2000/3000 → mean ≈233 (too bright); target ≈80–130.
OFFLINE_DOME_LIGHT_PATH = '/World/OfflineDomeLight'
OFFLINE_DISTANT_LIGHT_PATH = '/World/OfflineDistantLight'
OFFLINE_DOME_INTENSITY = 450.0
OFFLINE_DISTANT_INTENSITY = 900.0
# Degrees; upper-left key light toward workspace (matches common Isaac examples).
OFFLINE_DISTANT_ROTATE_XYZ_DEG = (-45.0, -35.0, 0.0)


def offline_scene_light_spec() -> dict[str, object]:
    """Return the frozen offline lighting contract (no Isaac import)."""
    return {
        'dome_path': OFFLINE_DOME_LIGHT_PATH,
        'distant_path': OFFLINE_DISTANT_LIGHT_PATH,
        'dome_intensity': OFFLINE_DOME_INTENSITY,
        'distant_intensity': OFFLINE_DISTANT_INTENSITY,
        'distant_rotate_xyz_deg': list(OFFLINE_DISTANT_ROTATE_XYZ_DEG),
        'reason': (
            'FixedCuboid offline ground has no Nucleus default_environment '
            'dome light; explicit lights required for usable scene RGB.'
        ),
    }


def add_offline_scene_lights(stage) -> dict[str, object]:
    """Create DomeLight + DistantLight on an offline FixedCuboid stage.

    Idempotent: redefines the same prim paths. Returns the applied spec for
    READY/logging. Requires ``pxr.UsdLux`` (Isaac / Omniverse runtime).
    """
    from pxr import Gf, Sdf, UsdLux

    spec = offline_scene_light_spec()
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path(str(spec['dome_path'])))
    dome.CreateIntensityAttr(float(spec['dome_intensity']))
    dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    distant = UsdLux.DistantLight.Define(
        stage, Sdf.Path(str(spec['distant_path']))
    )
    distant.CreateIntensityAttr(float(spec['distant_intensity']))
    distant.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    rotate = list(spec['distant_rotate_xyz_deg'])  # type: ignore[arg-type]
    distant.AddRotateXYZOp().Set(
        Gf.Vec3f(float(rotate[0]), float(rotate[1]), float(rotate[2]))
    )
    return spec


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
