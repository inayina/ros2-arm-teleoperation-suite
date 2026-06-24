#!/usr/bin/env bash
# stop_stack.sh — tear down the V2 teleop stack launched via teleop_bringup.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

stop_pattern() {
  local pattern="$1"
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    pkill -f "$pattern" 2>/dev/null || true
  fi
}

echo "[stop_stack] stopping teleop_bringup launch tree..."
stop_pattern 'ros2 launch teleop_bringup'
stop_pattern 'ros2 launch teleop_moveit_config'
stop_pattern 'ros2_control_node'
stop_pattern 'controller_manager'
stop_pattern 'mujoco_sim_node'
stop_pattern 'virtual_servo_driver'
stop_pattern 'servo_node'
stop_pattern 'teleop_input_node'
stop_pattern 'safety_monitor_node'
stop_pattern 'camera_bridge_node'
stop_pattern 'gripper_driver_node'
stop_pattern 'lerobot_recorder'

sleep 1

if pgrep -af 'teleop_bringup|ros2_control_node|mujoco_sim|servo_node|teleop_input' >/dev/null 2>&1; then
  echo "[stop_stack] some processes still running:"
  pgrep -af 'teleop_bringup|ros2_control_node|mujoco_sim|servo_node|teleop_input' || true
  exit 1
fi

echo "[stop_stack] stack stopped."
