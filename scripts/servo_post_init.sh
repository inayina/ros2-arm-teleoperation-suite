#!/usr/bin/env bash
# servo_post_init.sh — unpause MoveIt Servo and switch to pose-tracking mode.
set -euo pipefail

WAIT_S="${1:-6}"
CALL_TIMEOUT="${2:-20}"

sleep "$WAIT_S"

echo "[servo_post_init] unpausing servo_node..."
timeout "${CALL_TIMEOUT}s" ros2 service call /servo_node/pause_servo \
  std_srvs/srv/SetBool "{data: false}"

echo "[servo_post_init] switching to pose-tracking mode..."
timeout "${CALL_TIMEOUT}s" ros2 service call /servo_node/switch_command_type \
  moveit_msgs/srv/ServoCommandType "{command_type: 2}"

echo "[servo_post_init] done."
