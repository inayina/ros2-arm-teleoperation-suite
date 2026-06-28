# Copyright 2026 ros2-arm-teleoperation-suite contributors
# SPDX-License-Identifier: MIT

"""Run the workspace teleop_input tests under colcon."""
from pathlib import Path
import runpy


_ROOT = Path(__file__).resolve().parents[3]
_namespace = runpy.run_path(str(_ROOT / 'tests' / 'test_teleop_input.py'))

for _name, _value in _namespace.items():
    if _name.startswith('test_') or _name.startswith('Test'):
        globals()[_name] = _value
