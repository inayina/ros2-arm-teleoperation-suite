#!/usr/bin/env bash
# validate_m7_grasp_monitor.sh - M7 grasp monitor topic/state validation.

set -euo pipefail

CYAN='\033[0;36m'; GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
PASS=0; WARN=0; FAIL=0
LAUNCH_PID=""; HB_PID=""; BATCH_PID=""; STATUS_PID=""; ADVICE_PID=""

log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS + 1)); }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; WARN=$((WARN + 1)); }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL + 1)); }
log_info() { echo -e "${CYAN}[INFO]${NC} $1"; }

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/.m7_validation"
REPORT="${LOG_DIR}/grasp_monitor_report.json"
STATUS_LOG="${LOG_DIR}/grasp_status.txt"
ADVICE_LOG="${LOG_DIR}/grasp_advice.txt"
CONTACT_LOG="${LOG_DIR}/contact_debug_once.txt"
LAUNCH_LOG="${LOG_DIR}/full_system_grasp_monitor.log"
OBSERVE_SECONDS="${M7_GRASP_MONITOR_OBSERVE_SECONDS:-60}"
CONTROLLER_TIMEOUT_S="${M7_CONTROLLER_TIMEOUT_S:-45}"
BATCH_TIMEOUT_S="${M7_BATCH_TIMEOUT_S:-105}"

mkdir -p "${LOG_DIR}"
: > "${STATUS_LOG}"
: > "${ADVICE_LOG}"
: > "${CONTACT_LOG}"

export HOME="${M7_HOME:-${LOG_DIR}/home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${LOG_DIR}/ros_logs}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export ROS2CLI_DISABLE_DAEMON=1
mkdir -p "${HOME}/.ros/locks" "${HOME}/.ros/log" "${ROS_LOG_DIR}"

set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f "${ROOT_DIR}/install/setup.bash" ]] && source "${ROOT_DIR}/install/setup.bash"
set -u

cleanup() {
  for pid in "${STATUS_PID}" "${ADVICE_PID}" "${BATCH_PID}" "${HB_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        break
      fi
      sleep 1
    done
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

wait_for_controller() {
  local deadline=$((SECONDS + CONTROLLER_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if timeout 4s ros2 control list_controllers 2>/dev/null \
      | grep -A 1 cartesian_impedance_controller \
      | grep active >/dev/null; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

wait_with_timeout() {
  local pid="$1"
  local timeout_s="$2"
  local label="$3"
  local deadline=$((SECONDS + timeout_s))
  while kill -0 "${pid}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      log_warn "Timed out waiting for ${label}; continuing with collected monitor logs."
      kill "${pid}" 2>/dev/null || true
      return 1
    fi
    sleep 0.5
  done
  wait "${pid}" 2>/dev/null || true
}

request_safety_reset() {
  timeout 7s ros2 service wait /safety/reset --timeout 5 >/dev/null 2>&1 || true
  for _ in 1 2 3 4 5; do
    local output
    output="$(timeout 5s ros2 service call /safety/reset std_srvs/srv/Trigger "{}" 2>/dev/null || true)"
    if [[ "${output}" == *"success=True"* || "${output}" == *"success: true"* ]]; then
      return 0
    fi
    sleep 0.5
  done
  return 0
}

bool_for() {
  local file="$1"
  local pattern="$2"
  if grep -Eq "${pattern}" "${file}" 2>/dev/null; then
    echo "true"
  else
    echo "false"
  fi
}

echo ""
echo "================================"
echo "  M7 Grasp Monitor Validation"
echo "================================"

bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true

log_info "Launching full_system.launch.py with enable_grasp_monitor:=true ..."
setsid ros2 launch teleop_bringup full_system.launch.py \
  use_sim:=true \
  headless:=true \
  enable_grasp_monitor:=true \
  contact_debug_enabled:=false \
  grasp_assist_enabled:=false \
  watchdog_timeout:=2.0 \
  > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

log_info "Waiting for controller activation ..."
if wait_for_controller; then
  log_pass "cartesian_impedance_controller active"
else
  log_fail "controller did not become active"
fi

log_info "Switching Servo to pose command mode and resetting safety latch ..."
timeout 8s ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType \
  "{command_type: 2}" >/dev/null 2>&1 || true
python3 "${ROOT_DIR}/scripts/publish_dummy_heartbeat.py" --rate 50 >/dev/null 2>&1 &
HB_PID=$!
sleep 2
request_safety_reset
pkill -f "teleop_input_node" 2>/dev/null || true
sleep 1
request_safety_reset

log_info "Capturing grasp monitor topics for ${OBSERVE_SECONDS}s ..."
timeout "${OBSERVE_SECONDS}s" ros2 topic echo --no-daemon --field data \
  /grasp/status std_msgs/msg/String \
  > "${STATUS_LOG}" 2>&1 &
STATUS_PID=$!
timeout "${OBSERVE_SECONDS}s" ros2 topic echo --no-daemon --field data \
  /grasp/advice std_msgs/msg/String \
  > "${ADVICE_LOG}" 2>&1 &
ADVICE_PID=$!
timeout 8s ros2 topic echo --no-daemon --field data --once \
  /grasp/contact_debug std_msgs/msg/String \
  > "${CONTACT_LOG}" 2>&1 || true

log_info "Running one synthetic M7 grasp episode ..."
ros2 run synth_data_gen batch_generator --ros-args \
  -p episodes:=1 \
  -p hover_duration:=4.0 \
  -p descend_duration:=4.0 \
  -p close_duration:=3.0 \
  -p grasp_pause:=3.0 \
  -p pick_height_offset:=0.015 \
  -p lift_duration:=10.0 \
  -p lift_target_z:=0.075 \
  -p post_lift_hold:=8.0 \
  > "${LOG_DIR}/batch_generator.log" 2>&1 &
BATCH_PID=$!
wait_with_timeout "${BATCH_PID}" "${BATCH_TIMEOUT_S}" "batch generator" || true
BATCH_PID=""

sleep 3
kill "${STATUS_PID}" "${ADVICE_PID}" 2>/dev/null || true
wait "${STATUS_PID}" "${ADVICE_PID}" 2>/dev/null || true
STATUS_PID=""; ADVICE_PID=""

CONTACT_DEBUG_RECEIVED="$(bool_for "${CONTACT_LOG}" "ee_object_dist|finger_object_contacts")"
STATUS_RECEIVED="$(bool_for "${STATUS_LOG}" "\"state\"")"
ADVICE_RECEIVED="$(bool_for "${ADVICE_LOG}" "\"advice\"")"
CONTACT_DETECTED_OBSERVED="$(bool_for "${STATUS_LOG}" "CONTACT_DETECTED|LIFTING|GRASP_SUCCESS|SLIP_AFTER_LIFT")"
CLASSIFICATION_OBSERVED="$(bool_for "${STATUS_LOG}" "GRASP_SUCCESS|SLIP_AFTER_LIFT|GRASP_FAILED|RELEASED_BY_COMMAND")"
ASSISTED_OBSERVED="$(bool_for "${STATUS_LOG}" "ASSISTED_GRASP|\"grasp_assist_attached\": true")"

if [[ "${CONTACT_DEBUG_RECEIVED}" == "true" ]]; then
  log_pass "/grasp/contact_debug received"
else
  log_fail "/grasp/contact_debug not received"
fi
if [[ "${STATUS_RECEIVED}" == "true" ]]; then
  log_pass "/grasp/status received"
else
  log_fail "/grasp/status not received"
fi
if [[ "${ADVICE_RECEIVED}" == "true" ]]; then
  log_pass "/grasp/advice received"
else
  log_fail "/grasp/advice not received"
fi
if [[ "${CONTACT_DETECTED_OBSERVED}" == "true" ]]; then
  log_pass "CONTACT_DETECTED/LIFTING observed"
else
  log_warn "No contact/lift state observed; inspect ${STATUS_LOG}"
fi
if [[ "${CLASSIFICATION_OBSERVED}" == "true" ]]; then
  log_pass "Terminal success/failure classification observed"
else
  log_warn "No terminal classification observed; try a longer OBSERVE_SECONDS window"
fi
if [[ "${ASSISTED_OBSERVED}" == "true" ]]; then
  log_fail "Synthetic assist was observed during physical grasp validation"
else
  log_pass "No synthetic grasp assist observed"
fi

cat > "${REPORT}" <<JSON
{
  "contact_debug_received": ${CONTACT_DEBUG_RECEIVED},
  "status_received": ${STATUS_RECEIVED},
  "advice_received": ${ADVICE_RECEIVED},
  "contact_detected_observed": ${CONTACT_DETECTED_OBSERVED},
  "classification_observed": ${CLASSIFICATION_OBSERVED},
  "assisted_observed": ${ASSISTED_OBSERVED},
  "status_log": "${STATUS_LOG}",
  "advice_log": "${ADVICE_LOG}",
  "contact_debug_log": "${CONTACT_LOG}",
  "launch_log": "${LAUNCH_LOG}",
  "pass": ${PASS},
  "warn": ${WARN},
  "fail": ${FAIL}
}
JSON

echo "Report saved to ${REPORT}"
if (( FAIL > 0 )); then
  exit 1
fi
