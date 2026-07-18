#!/usr/bin/env bash
# stop_stack.sh — tear down the V2 teleop stack launched via teleop_bringup.
set -euo pipefail

readonly TERM_WAIT_STEPS="${STOP_STACK_TERM_WAIT_STEPS:-20}"
readonly TERM_WAIT_INTERVAL_S="${STOP_STACK_TERM_WAIT_INTERVAL_S:-0.1}"

readonly -a STACK_PATTERNS=(
  '/opt/ros/.*/bin/ros2 launch teleop_bringup'
  '/opt/ros/.*/bin/ros2 launch teleop_moveit_config'
  'ros2_control_node'
  'controller_manager'
  'robot_state_publisher'
  'static_transform_publisher'
  'aggregator_node'
  'mujoco_sim_node'
  'virtual_servo_driver'
  'servo_node'
  'teleop_input_node'
  'safety_monitor_node'
  'camera_bridge_node'
  'gripper_driver_node'
  'lerobot_recorder_node'
  'system_telemetry_node'
  'grasp_monitor'
  'batch_generator'
)

collect_stack_pids() {
  local pattern pid cmdline
  declare -A seen=()
  for pattern in "${STACK_PATTERNS[@]}"; do
    while read -r pid cmdline; do
      [[ -n "${pid:-}" ]] || continue
      [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || continue
      case "${cmdline}" in
        *stop_stack.sh*|*pgrep*|*pkill*) continue ;;
      esac
      seen["${pid}"]=1
    done < <(pgrep -af "${pattern}" 2>/dev/null || true)
  done
  if (( ${#seen[@]} > 0 )); then
    printf '%s\n' "${!seen[@]}" | sort -n
  fi
}

mapfile -t target_pids < <(collect_stack_pids)
if (( ${#target_pids[@]} == 0 )); then
  echo "[stop_stack] stack already stopped."
  exit 0
fi

echo "[stop_stack] stopping ${#target_pids[@]} process(es)..."
kill -TERM "${target_pids[@]}" 2>/dev/null || true

survivors=()
for ((step = 0; step < TERM_WAIT_STEPS; step++)); do
  survivors=()
  for pid in "${target_pids[@]}"; do
    kill -0 "${pid}" 2>/dev/null && survivors+=("${pid}")
  done
  (( ${#survivors[@]} == 0 )) && break
  sleep "${TERM_WAIT_INTERVAL_S}"
done

if (( ${#survivors[@]} > 0 )); then
  echo "[stop_stack] force-killing ${#survivors[@]} unresponsive process(es)..."
  kill -KILL "${survivors[@]}" 2>/dev/null || true
  sleep 0.2
fi

mapfile -t remaining_pids < <(collect_stack_pids)
if (( ${#remaining_pids[@]} > 0 )); then
  echo "[stop_stack] failed; process(es) still running: ${remaining_pids[*]}" >&2
  exit 1
fi

echo "[stop_stack] stack stopped."
