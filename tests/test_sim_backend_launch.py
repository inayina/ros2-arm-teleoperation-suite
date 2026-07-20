"""Static and pure-function tests for the simulator launch boundary."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
FULL_SYSTEM = ROOT / "src/teleop_bringup/launch/full_system.launch.py"
SELECTOR = ROOT / "src/teleop_bringup/launch/simulation.launch.py"
ROS2_CONTROL = ROOT / "src/teleop_bringup/launch/ros2_control.launch.py"
SERVO = ROOT / "src/teleop_moveit_config/launch/servo.launch.py"
SIM_CONTROL_RATE = ROOT / "src/teleop_bringup/config/control_rate_sim.yaml"
REAL_CONTROL_RATE = ROOT / "src/teleop_bringup/config/control_rate_real.yaml"
CANOPEN_SYSTEM = ROOT / "src/canopen_hw_interface/src/canopen_system.cpp"
MUJOCO_NODE = ROOT / "src/mujoco_sim/mujoco_sim/mujoco_sim_node.py"
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


def test_ros2_control_disables_fifo_for_dds_sim_backplane_only():
    source = ROS2_CONTROL.read_text(encoding="utf-8")
    assert '"controller_thread_priority"' in source
    assert '"thread_priority": ParameterValue(' in source
    assert '"\'0\' if \'", use_sim, "\' == \'true\' else \'50\'"' in source
    assert '"\'control_rate_sim.yaml\' if \'", use_sim' in source
    assert '"\' == \'true\' else \'control_rate_real.yaml\'"' in source
    assert "update_rate: 500" in SIM_CONTROL_RATE.read_text(encoding="utf-8")
    assert "update_rate: 1000" in REAL_CONTROL_RATE.read_text(encoding="utf-8")


def test_moveit_servo_disables_fifo_for_dds_sim_backplane_only():
    source = SERVO.read_text(encoding="utf-8")
    assert "'servo_thread_priority'" in source
    assert "'thread_priority'] = ParameterValue(" in source
    assert '"\'0\' if \'", use_sim, "\' == \'true\' else \'40\'"' in source
    assert "'prlimit --rtprio=0:0 --' if '" in source


def test_sim_effort_dds_publish_is_off_the_controller_write_path():
    source = CANOPEN_SYSTEM.read_text(encoding="utf-8")
    write_body = source.split("CanopenSystem::write(", 1)[1].split(
        "}  // namespace canopen_hw_interface", 1
    )[0]
    assert "sim_effort_command_[i].store" in write_body
    assert "pub_sim_effort_->publish" not in write_body
    assert "CanopenSystem::sim_effort_publish_loop()" in source
    assert "KeepLast(1)" in source


def test_mujoco_encoder_feedback_is_500hz_without_raising_all_observations():
    source = MUJOCO_NODE.read_text(encoding="utf-8")
    assert 'declare_parameter("encoder_publish_rate", 500.0)' in source
    assert "self._encoder_pub_decim" in source
    assert "self._observation_pub_decim" in source
    assert "def _publish_encoder(self):" in source
    assert "def _publish_observations(self):" in source


def test_selector_delegates_mujoco_details_to_backend_launch():
    selector = SELECTOR.read_text(encoding="utf-8")
    backend = MUJOCO_BACKEND.read_text(encoding="utf-8")
    assert 'backends",\n        "mujoco.launch.py"' in selector
    assert 'package="mujoco_sim"' not in selector
    assert 'package="mujoco_sim"' in backend
    assert 'name="mujoco_sim"' in backend
    assert 'package="camera_bridge"' in backend
    assert backend.count('"synthetic_fallback": False') == 4


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
