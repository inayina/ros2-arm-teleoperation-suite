"""Integration test for M1 control simulation launch."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


class _JointStateCollector(Node):
    def __init__(self) -> None:
        super().__init__('m1_launch_test_collector')
        self.joint_states: list[JointState] = []
        self.create_subscription(
            JointState, '/joint_states', self._on_joint, 10)

    def _on_joint(self, msg: JointState) -> None:
        self.joint_states.append(msg)


@pytest.mark.launch_test
def test_m1_control_sim_launches_and_publishes() -> None:
    """Launch m1_control_sim and verify it publishes joint states."""
    old_domain_id = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = old_domain_id or '89'
    env = os.environ.copy()
    
    # Launch in headless mode
    proc = subprocess.Popen(
        [
            'ros2',
            'launch',
            'teleop_bringup',
            'm1_control_sim.launch.py',
            'headless:=true',
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    rclpy.init()
    collector = _JointStateCollector()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Wait up to 15 seconds for joint states to publish
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if len(collector.joint_states) > 0:
                break
            time.sleep(0.2)

        assert collector.joint_states, 'm1_control_sim should publish /joint_states'
        
        # Verify joint state contains Franka Panda joint names
        latest = collector.joint_states[-1]
        assert len(latest.name) >= 7, 'Panda should have at least 7 joints'
        assert any('panda_joint' in name for name in latest.name), 'Should contain panda joint names'
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
        # Clean teardown of ROS 2 launch subprocess
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except PermissionError:
                proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except PermissionError:
                    proc.kill()
                proc.wait(timeout=5.0)
                
        if old_domain_id is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = old_domain_id
