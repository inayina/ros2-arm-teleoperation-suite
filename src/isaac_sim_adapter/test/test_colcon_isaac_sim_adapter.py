# Copyright 2026 ros2-arm-teleoperation-suite contributors
# SPDX-License-Identifier: MIT

"""Expose repository-level Isaac adapter tests to colcon."""

from pathlib import Path
import runpy


_namespace = runpy.run_path(
    str(Path(__file__).resolve().parents[3] / 'tests/test_isaac_sim_adapter.py')
)

for _name, _value in _namespace.items():
    if _name.startswith('test_') or _name.startswith('Test'):
        globals()[_name] = _value

_policy_namespace = runpy.run_path(
    str(Path(__file__).resolve().parents[3] / 'tests/test_isaac_policy_inference.py')
)

for _name, _value in _policy_namespace.items():
    if _name.startswith('test_') or _name.startswith('Test'):
        globals()[_name] = _value
