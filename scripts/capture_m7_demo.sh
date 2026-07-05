#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -euo pipefail

HEADLESS="${M7_HEADLESS:-true}"
CONTACT_DEBUG="${M7_CONTACT_DEBUG:-true}"
GRASP_ASSIST="${M7_GRASP_ASSIST:-false}"
RECORD_GIF="${M7_RECORD_GIF:-true}"
WATCHDOG_TIMEOUT="${M7_WATCHDOG_TIMEOUT:-2.0}"
CONTROLLER_TIMEOUT_S="${M7_CONTROLLER_TIMEOUT_S:-45}"
BATCH_TIMEOUT_S="${M7_BATCH_TIMEOUT_S:-105}"
RECORDER_TIMEOUT_S="${M7_RECORDER_TIMEOUT_S:-60}"
GRASP_GIF_SECONDS="${M7_GRASP_GIF_SECONDS:-40}"
WRIST_GIF_SECONDS="${M7_WRIST_GIF_SECONDS:-40}"
HOVER_DURATION="${M7_HOVER_DURATION:-4.0}"
DESCEND_DURATION="${M7_DESCEND_DURATION:-4.0}"
CLOSE_DURATION="${M7_CLOSE_DURATION:-3.0}"
GRASP_PAUSE="${M7_GRASP_PAUSE:-3.0}"
PICK_HEIGHT_OFFSET="${M7_PICK_HEIGHT_OFFSET:-0.015}"
LIFT_DURATION="${M7_LIFT_DURATION:-10.0}"
LIFT_TARGET_Z="${M7_LIFT_TARGET_Z:-0.075}"
POST_LIFT_HOLD="${M7_POST_LIFT_HOLD:-8.0}"

LAUNCH_PID=""
HB_PID=""
ROS_HB_PID=""
REC_PID=""
REC_WRIST_PID=""
BATCH_PID=""

cleanup() {
  if [[ -n "${BATCH_PID}" ]]; then kill "${BATCH_PID}" 2>/dev/null || true; fi
  if [[ -n "${REC_WRIST_PID}" ]]; then kill "${REC_WRIST_PID}" 2>/dev/null || true; fi
  if [[ -n "${REC_PID}" ]]; then kill "${REC_PID}" 2>/dev/null || true; fi
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait_with_timeout "${LAUNCH_PID}" 10 "ROS 2 launch shutdown" || true
  fi
  if [[ -n "${ROS_HB_PID}" ]]; then kill "${ROS_HB_PID}" 2>/dev/null || true; fi
  if [[ -n "${HB_PID}" ]]; then kill "${HB_PID}" 2>/dev/null || true; fi
  sleep 2
  pkill -f "ros2 launch teleop_bringup full_system.launch.py" 2>/dev/null || true
  pkill -f "mujoco_sim_node" 2>/dev/null || true
  pkill -f "servo_node" 2>/dev/null || true
  pkill -f "controller_manager" 2>/dev/null || true
}
trap cleanup EXIT

wait_for_controller() {
  local deadline=$((SECONDS + CONTROLLER_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if ros2 control list_controllers 2>/dev/null | grep -A 1 cartesian_impedance_controller | grep active > /dev/null; then
      return 0
    fi
    sleep 0.5
  done
  echo "Timed out waiting for cartesian_impedance_controller to become active." >&2
  return 1
}

wait_with_timeout() {
  local pid="$1"
  local timeout_s="$2"
  local label="$3"
  local deadline=$((SECONDS + timeout_s))
  while kill -0 "${pid}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      echo "Timed out waiting for ${label}." >&2
      kill "${pid}" 2>/dev/null || true
      return 1
    fi
    sleep 0.5
  done
  wait "${pid}"
}

request_safety_reset() {
  echo "Requesting safety latch reset..."
  ros2 service wait /safety/reset --timeout 5 >/dev/null 2>&1 || true
  local attempt
  for attempt in 1 2 3 4 5; do
    local output
    output="$(ros2 service call /safety/reset std_srvs/srv/Trigger "{}" 2>/dev/null || true)"
    if [[ "${output}" == *"success=True"* || "${output}" == *"success: true"* || "${output}" == *"E-Stop reset"* ]]; then
      echo "Safety reset succeeded."
      return 0
    fi
    sleep 0.5
  done
  echo "Safety reset was not needed or is currently blocked; continuing so diagnostics can capture the state."
  return 0
}

echo "Starting full system (headless=${HEADLESS}, grasp_assist_enabled=${GRASP_ASSIST})..."
echo "Mode: use_sim:=true (sim-direct). Use M2/M5 validation for CANopen fieldbus evidence."
ros2 launch teleop_bringup full_system.launch.py \
  use_sim:=true \
  headless:="${HEADLESS}" \
  contact_debug_enabled:="${CONTACT_DEBUG}" \
  contact_debug_period_s:=0.5 \
  grasp_assist_enabled:="${GRASP_ASSIST}" \
  watchdog_timeout:="${WATCHDOG_TIMEOUT}" &
LAUNCH_PID=$!

echo "Waiting for controllers to activate..."
wait_for_controller
echo "Controllers active! Stabilizing physics..."
sleep 1

echo "Switching command type to POSE tracking..."
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 2}"
sleep 1

echo "Starting dummy heartbeat for safety monitor..."
python3 scripts/publish_dummy_heartbeat.py --rate 50 &
HB_PID=$!
ros2 topic pub -r 30 /teleop/heartbeat std_msgs/msg/Header "{frame_id: 'm7_capture_heartbeat'}" >/dev/null 2>&1 &
ROS_HB_PID=$!

# Wait for ros2 topic pub to initialize so the heartbeat isn't interrupted
sleep 2
request_safety_reset

echo "Stopping manual teleop to allow batch generator to take over..."
pkill -f "teleop_input_node" || true
sleep 1
request_safety_reset

if [[ "${RECORD_GIF}" == "true" ]]; then
  echo "Starting GIF recorder..."
  python3 scripts/record_demo_gif.py media/m7/grasp_demo.gif --seconds "${GRASP_GIF_SECONDS}" --fps 12 &
  REC_PID=$!
  python3 scripts/record_demo_gif.py media/m7/gripper_closeup.gif --seconds "${WRIST_GIF_SECONDS}" --fps 12 --topic /camera/wrist/color/image_raw &
  REC_WRIST_PID=$!

  # Give recorder a moment to subscribe.
  sleep 2
else
  echo "Skipping GIF recorder (M7_RECORD_GIF=${RECORD_GIF})."
fi

echo "Starting batch generator..."
ros2 run synth_data_gen batch_generator --ros-args \
  -p episodes:=1 \
  -p hover_duration:="${HOVER_DURATION}" \
  -p descend_duration:="${DESCEND_DURATION}" \
  -p close_duration:="${CLOSE_DURATION}" \
  -p grasp_pause:="${GRASP_PAUSE}" \
  -p pick_height_offset:="${PICK_HEIGHT_OFFSET}" \
  -p lift_duration:="${LIFT_DURATION}" \
  -p lift_target_z:="${LIFT_TARGET_Z}" \
  -p post_lift_hold:="${POST_LIFT_HOLD}" &
BATCH_PID=$!

echo "Waiting for batch generator and recorder to finish..."
wait_with_timeout "${BATCH_PID}" "${BATCH_TIMEOUT_S}" "batch generator"
BATCH_PID=""
if [[ -n "${REC_PID}" ]]; then
  wait_with_timeout "${REC_PID}" "${RECORDER_TIMEOUT_S}" "GIF recorder"
  REC_PID=""
fi
if [[ -n "${REC_WRIST_PID}" ]]; then
  wait_with_timeout "${REC_WRIST_PID}" "${RECORDER_TIMEOUT_S}" "wrist GIF recorder"
  REC_WRIST_PID=""
fi

echo "Killing ROS 2 launch..."
kill -INT "$LAUNCH_PID"
wait_with_timeout "${LAUNCH_PID}" 10 "ROS 2 launch shutdown" || true
kill "$ROS_HB_PID" 2>/dev/null || true
kill "$HB_PID" 2>/dev/null || true
LAUNCH_PID=""
ROS_HB_PID=""
HB_PID=""

echo "Done capturing M7 demo!"
