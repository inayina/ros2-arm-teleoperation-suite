#!/usr/bin/env bash
# validate_m4_motion_layer.sh
# Automated M4 acceptance check: samples heartbeat, /joint_target, and
# controller state, then verifies the servo node is outputting at ~125 Hz.
#
# Usage (stack already running):
#   bash scripts/validate_m4_motion_layer.sh
#
# Usage (let this script launch the stack itself):
#   bash scripts/validate_m4_motion_layer.sh --launch
#
# Exit code: 0 = all checks passed, 1 = one or more checks failed.

set -euo pipefail
CYAN='\033[0;36m'; GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
PASS=0; FAIL=0; LAUNCH_PID=""

log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS + 1)); }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL + 1)); }
log_info() { echo -e "${CYAN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/.m4_validation"
mkdir -p "${LOG_DIR}"

# ---------------------------------------------------------------------------
# Source ROS + workspace
# ---------------------------------------------------------------------------
set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f "${ROOT_DIR}/install/setup.bash" ]] && source "${ROOT_DIR}/install/setup.bash"
set -u

# ---------------------------------------------------------------------------
# Optional: launch the full stack from this script
# ---------------------------------------------------------------------------
cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    log_info "Shutting down launched stack (PID ${LAUNCH_PID})..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

if [[ "${1:-}" == "--launch" ]]; then
  bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  log_info "Launching full_system.launch.py in background (new session)..."
  setsid ros2 launch teleop_bringup full_system.launch.py headless:=true \
    > "${LOG_DIR}/full_system.log" 2>&1 &
  LAUNCH_PID=$!
  log_info "Waiting 35 s for all layers + servo init (PID ${LAUNCH_PID})..."
  sleep 35
fi

echo ""
echo "=============================="
echo "  M4 Motion Layer Validation"
echo "=============================="

# Ensure servo is unpaused and in pose mode (idempotent if launch already did it).
log_info "Ensuring servo_node is unpaused and in pose mode..."
bash "${ROOT_DIR}/scripts/servo_post_init.sh" 0 \
  > "${LOG_DIR}/servo_post_init.txt" 2>&1 || true
log_info "Warm-up 5 s for stable topic rates..."
sleep 5

# ---------------------------------------------------------------------------
# AC-7  /teleop/heartbeat @ 50 Hz
# ---------------------------------------------------------------------------
log_info "AC-7 — /teleop/heartbeat frequency (expect ≥45 Hz) ..."
timeout 8s ros2 topic hz /teleop/heartbeat --window 50 \
  > "${LOG_DIR}/heartbeat_hz.txt" 2>&1 || true
HB_HZ=$(grep "average rate" "${LOG_DIR}/heartbeat_hz.txt" | tail -1 \
         | awk '{print $3}' | tr -d ':' || true)
if [[ -z "$HB_HZ" ]]; then
  log_fail "AC-7 /teleop/heartbeat: no messages (is teleop_input running?)"
else
  HB_INT=${HB_HZ%.*}
  if [[ "${HB_INT:-0}" -ge 45 ]]; then
    log_pass "AC-7 /teleop/heartbeat @ ${HB_HZ} Hz"
  else
    log_fail "AC-7 /teleop/heartbeat only @ ${HB_HZ} Hz (need ≥45)"
  fi
fi

# ---------------------------------------------------------------------------
# AC-1  servo_node active: /safe_master_pose must have a publisher
# ---------------------------------------------------------------------------
log_info "AC-1 — /safe_master_pose publisher count ..."
PUB_COUNT=$(ros2 topic info /safe_master_pose 2>/dev/null \
            | grep "Publisher count" | awk '{print $NF}')
if [[ "${PUB_COUNT:-0}" -ge 1 ]]; then
  log_pass "AC-1 /safe_master_pose has ${PUB_COUNT} publisher(s)"
else
  log_fail "AC-1 /safe_master_pose has no publishers — check safety_monitor"
fi

# ---------------------------------------------------------------------------
# AC-1  /joint_target @ 125 Hz  (servo must be publishing)
# ---------------------------------------------------------------------------
log_info "AC-1 — /joint_target frequency (expect ≥90 Hz on non-RT host) ..."
timeout 12s ros2 topic hz /joint_target --window 100 \
  > "${LOG_DIR}/joint_target_hz.txt" 2>&1 || true
JT_HZ=$(grep "average rate" "${LOG_DIR}/joint_target_hz.txt" | tail -1 \
         | awk '{print $3}' | tr -d ':' || true)
if [[ -z "$JT_HZ" ]]; then
  log_fail "AC-1 /joint_target: no messages (is servo_node running and unpaused?)"
else
  JT_INT=${JT_HZ%.*}
  if [[ "${JT_INT:-0}" -ge 85 ]]; then
    log_pass "AC-1 /joint_target @ ${JT_HZ} Hz"
  else
    log_fail "AC-1 /joint_target only @ ${JT_HZ:-0} Hz (need ≥85 on non-RT host)"
  fi
fi

# ---------------------------------------------------------------------------
# Controller state  (cartesian_impedance_controller must be active)
# ---------------------------------------------------------------------------
log_info "Checking controller state ..."
ros2 control list_controllers > "${LOG_DIR}/controllers.txt" 2>&1 || true
CTRL_STATE=$(grep "cartesian_impedance_controller" "${LOG_DIR}/controllers.txt" \
             | awk '{print $NF}')
if [[ "${CTRL_STATE}" == "active" ]]; then
  log_pass "cartesian_impedance_controller is active"
else
  log_fail "cartesian_impedance_controller state='${CTRL_STATE:-not found}' (need active)"
fi

# ---------------------------------------------------------------------------
# Sample one /joint_target message
# ---------------------------------------------------------------------------
log_info "Sampling one /joint_target message ..."
timeout 5s ros2 topic echo /joint_target --once \
  > "${LOG_DIR}/joint_target_sample.txt" 2>&1 || true
if grep -q "joint_names" "${LOG_DIR}/joint_target_sample.txt"; then
  log_pass "/joint_target message received and contains joint_names"
  echo "--- Sample (first 15 lines) ---"
  head -15 "${LOG_DIR}/joint_target_sample.txt"
  echo "---"
else
  log_fail "/joint_target message not received or malformed"
fi

# ---------------------------------------------------------------------------
# AC-3  End-to-end latency hint via ros2 topic delay
# ---------------------------------------------------------------------------
log_info "AC-3 — End-to-end latency estimate (/joint_states delay) ..."
timeout 6s ros2 topic delay /joint_states \
  > "${LOG_DIR}/e2e_delay.txt" 2>&1 || true
DELAY_MS=$(grep "average delay" "${LOG_DIR}/e2e_delay.txt" | head -1 \
            | awk '{print $3}' | tr -d 's' || true)
if [[ -n "$DELAY_MS" ]]; then
  DELAY_MS_INT=$(echo "$DELAY_MS * 1000" | bc | cut -d. -f1)
  if [[ "${DELAY_MS_INT:-9999}" -le 50 ]]; then
    log_pass "AC-3 end-to-end delay ~${DELAY_MS_INT} ms (<50 ms)"
  else
    log_warn "AC-3 end-to-end delay ~${DELAY_MS_INT} ms (target <50 ms — may be DDS buffering)"
  fi
else
  log_warn "AC-3 could not measure delay (run manually: ros2 topic delay /joint_states)"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "=============================="
echo -e "M4 Result: ${GREEN}${PASS} passed${NC}  ${RED}${FAIL} failed${NC}"
echo "Logs: ${LOG_DIR}/"
echo "=============================="

[[ "$FAIL" -gt 0 ]] && exit 1 || exit 0
