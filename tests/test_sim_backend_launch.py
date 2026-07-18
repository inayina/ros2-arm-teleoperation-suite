"""Static and pure-function tests for the simulator launch boundary."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
FULL_SYSTEM = ROOT / "src/teleop_bringup/launch/full_system.launch.py"
SELECTOR = ROOT / "src/teleop_bringup/launch/simulation.launch.py"
MUJOCO_BACKEND = ROOT / "src/teleop_bringup/launch/backends/mujoco.launch.py"
ISAAC_BACKEND = ROOT / "src/teleop_bringup/launch/backends/isaac.launch.py"


def _load_selector():
    spec = importlib.util.spec_from_file_location("simulation_launch", SELECTOR)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_backend_name_validation_is_explicit():
    module = _load_selector()
    assert module.validate_backend_name(" MuJoCo ") == "mujoco"
    assert module.validate_backend_name(" ISAAC ") == "isaac"
    with pytest.raises(ValueError, match="sim_backend must be one of"):
        module.validate_backend_name("pybullet")


def test_full_system_defaults_to_mujoco_and_forwards_backend():
    source = FULL_SYSTEM.read_text(encoding="utf-8")
    assert 'default_value="mujoco"' in source
    assert '"sim_backend": sim_backend' in source
    assert 'choices=["mujoco", "isaac"]' in source


def test_selector_delegates_mujoco_details_to_backend_launch():
    selector = SELECTOR.read_text(encoding="utf-8")
    backend = MUJOCO_BACKEND.read_text(encoding="utf-8")
    assert 'backends",\n        "mujoco.launch.py"' in selector
    assert 'package="mujoco_sim"' not in selector
    assert 'package="mujoco_sim"' in backend
    assert 'name="mujoco_sim"' in backend
    assert 'package="camera_bridge"' in backend


def test_selector_delegates_isaac_to_ros_only_adapter():
    selector = SELECTOR.read_text(encoding="utf-8")
    backend = ISAAC_BACKEND.read_text(encoding="utf-8")
    assert 'backends",\n        "isaac.launch.py"' in selector
    assert 'package="isaac_sim_adapter"' in backend
    assert "isaac_panda_backend.py" in backend
    assert "SimulationApp" not in backend


def test_batch_generator_has_no_fixed_mujoco_parameter_call():
    source = (
        ROOT / "src/synth_data_gen/synth_data_gen/batch_generator.py"
    ).read_text(encoding="utf-8")
    assert "_set_node_parameter('/mujoco_sim'" not in source
    assert "self.simulator_node_name" in source
