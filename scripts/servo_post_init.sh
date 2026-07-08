#!/usr/bin/env bash
# servo_post_init.sh — unpause MoveIt Servo and switch command type.
set -euo pipefail

WAIT_S="${1:-6}"
CALL_TIMEOUT="${2:-20}"
SERVO_MODE="${SERVO_POST_INIT_MODE:-pose}"
case "${SERVO_MODE}" in
  twist) COMMAND_TYPE=1 ;;
  pose) COMMAND_TYPE=2 ;;
  *)
    echo "[servo_post_init] unknown SERVO_POST_INIT_MODE=${SERVO_MODE} (use pose|twist)" >&2
    exit 1
    ;;
esac

sleep "$WAIT_S"

echo "[servo_post_init] unpausing servo_node..."
timeout "${CALL_TIMEOUT}s" ros2 service call /servo_node/pause_servo \
  std_srvs/srv/SetBool "{data: false}"

echo "[servo_post_init] switching to ${SERVO_MODE} mode..."
timeout "${CALL_TIMEOUT}s" ros2 service call /servo_node/switch_command_type \
  moveit_msgs/srv/ServoCommandType "{command_type: ${COMMAND_TYPE}}"

echo "[servo_post_init] done."
