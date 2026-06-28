#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -euo pipefail

HEADLESS="${M7_HEADLESS:-true}"
CONTROLLER_TIMEOUT_S="${M7_CONTROLLER_TIMEOUT_S:-45}"
BATCH_TIMEOUT_S="${M7_BATCH_TIMEOUT_S:-45}"
RECORDER_TIMEOUT_S="${M7_RECORDER_TIMEOUT_S:-35}"

LAUNCH_PID=""
HB_PID=""
REC_PID=""
BATCH_PID=""

cleanup() {
  if [[ -n "${BATCH_PID}" ]]; then kill "${BATCH_PID}" 2>/dev/null || true; fi
  if [[ -n "${REC_PID}" ]]; then kill "${REC_PID}" 2>/dev/null || true; fi
  if [[ -n "${HB_PID}" ]]; then kill "${HB_PID}" 2>/dev/null || true; fi
  if [[ -n "${LAUNCH_PID}" ]]; then kill -INT "${LAUNCH_PID}" 2>/dev/null || true; fi
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
  echo "Safety reset was not needed or is currently blocked; continuing so diagnostics/GIF can capture the state."
  return 0
}

echo "Starting full system (headless=${HEADLESS})..."
echo "Mode: use_sim:=true (sim-direct). Use M2/M5 validation for CANopen fieldbus evidence."
ros2 launch teleop_bringup full_system.launch.py use_sim:=true headless:="${HEADLESS}" &
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

# Wait for ros2 topic pub to initialize so the heartbeat isn't interrupted
sleep 2
request_safety_reset

echo "Stopping manual teleop to allow batch generator to take over..."
pkill -f "teleop_input_node" || true
sleep 1
request_safety_reset

echo "Starting GIF recorder..."
python3 scripts/record_demo_gif.py media/m7/grasp_demo.gif --seconds 20 --fps 15 &
REC_PID=$!

# Give recorder a moment to subscribe
sleep 2

echo "Starting batch generator..."
ros2 run synth_data_gen batch_generator --ros-args \
  -p episodes:=1 \
  -p hover_duration:=4.0 \
  -p descend_duration:=4.0 \
  -p grasp_pause:=2.0 \
  -p lift_duration:=4.0 &
BATCH_PID=$!

echo "Waiting for batch generator and recorder to finish..."
wait_with_timeout "${BATCH_PID}" "${BATCH_TIMEOUT_S}" "batch generator"
BATCH_PID=""
wait_with_timeout "${REC_PID}" "${RECORDER_TIMEOUT_S}" "GIF recorder"
REC_PID=""

echo "Killing ROS 2 launch..."
kill -INT $LAUNCH_PID
kill $HB_PID 2>/dev/null || true
LAUNCH_PID=""
HB_PID=""

echo "Done capturing M7 demo!"
