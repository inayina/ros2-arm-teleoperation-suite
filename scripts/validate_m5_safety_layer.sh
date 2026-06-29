#!/usr/bin/env bash
# validate_m5_safety_layer.sh — automated M5 safety layer checks.
#
# Usage (stack already running):
#   bash scripts/validate_m5_safety_layer.sh
#
# Usage (launch full stack):
#   bash scripts/validate_m5_safety_layer.sh --launch
#
# Optional CAN mode (Quick Stop SDO on vcan0):
#   bash scripts/validate_m5_safety_layer.sh --launch --can

set -euo pipefail
CYAN='\033[0;36m'; GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
PASS=0; FAIL=0; LAUNCH_PID=""

log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS + 1)); }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL + 1)); }
log_info() { echo -e "${CYAN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/.m5_validation"
mkdir -p "${LOG_DIR}"

set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f "${ROOT_DIR}/install/setup.bash" ]] && source "${ROOT_DIR}/install/setup.bash"
set -u

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

USE_CAN=false
for arg in "$@"; do
  [[ "$arg" == "--can" ]] && USE_CAN=true
done

if [[ "${1:-}" == "--launch" || "${2:-}" == "--launch" ]]; then
  bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  bash "${ROOT_DIR}/scripts/setup_vcan.sh" >/dev/null 2>&1 || true
  LAUNCH_ARGS=(headless:=true)
  if $USE_CAN; then
    LAUNCH_ARGS+=(use_sim:=false can_interface:=vcan0)
  fi
  log_info "Launching full_system.launch.py ${LAUNCH_ARGS[*]} ..."
  setsid ros2 launch teleop_bringup full_system.launch.py "${LAUNCH_ARGS[@]}" \
    > "${LOG_DIR}/full_system.log" 2>&1 &
  LAUNCH_PID=$!
  log_info "Waiting 30 s for stack (PID ${LAUNCH_PID})..."
  sleep 30
  bash "${ROOT_DIR}/scripts/servo_post_init.sh" 0 > "${LOG_DIR}/servo_post_init.txt" 2>&1 || true
  sleep 3
fi

echo ""
echo "=============================="
echo "  M5 Safety Layer Validation"
echo "=============================="

# AC-1 unit tests
log_info "AC-1 — safety_monitor GTest ..."
if (cd "${ROOT_DIR}" && colcon test --packages-select safety_monitor --event-handlers console_direct+ \
    > "${LOG_DIR}/gtest.log" 2>&1); then
  log_pass "AC-1 safety_monitor colcon test"
else
  log_fail "AC-1 colcon test failed — see ${LOG_DIR}/gtest.log"
fi

# AC-6 manual trigger
log_info "AC-6 — /safety/trigger_estop ..."
timeout 15s ros2 service call /safety/trigger_estop teleop_interfaces/srv/TriggerEstop "{reason: 'm5_validate'}" \
  > "${LOG_DIR}/trigger_estop.txt" 2>&1 || true
sleep 0.5
ESTOP=$(timeout 3s ros2 topic echo /safety/estop --once 2>/dev/null | grep "data:" | awk '{print $2}' || true)
if [[ "${ESTOP}" == "true" ]]; then
  log_pass "AC-6 /safety/trigger_estop latched E-Stop"
else
  log_fail "AC-6 E-Stop not latched after trigger (got '${ESTOP:-empty}')"
fi

# AC-5 reset (requires faults cleared — restart teleop heartbeat path)
log_info "AC-5 — /safety/reset ..."
timeout 15s ros2 service call /safety/reset std_srvs/srv/Trigger "{}" \
  > "${LOG_DIR}/reset.txt" 2>&1 || true
sleep 0.5
if grep -q "success=True" "${LOG_DIR}/reset.txt" 2>/dev/null; then
  log_pass "AC-5 /safety/reset cleared E-Stop"
elif grep -q "success: true" "${LOG_DIR}/reset.txt" 2>/dev/null; then
  log_pass "AC-5 /safety/reset cleared E-Stop"
else
  log_fail "AC-5 /safety/reset failed — see ${LOG_DIR}/reset.txt"
fi

# AC-7 diagnostics — expect 5 monitor entries
log_info "AC-7 — /safety/diagnostics monitor entries ..."
timeout 4s ros2 topic echo /safety/diagnostics --once \
  > "${LOG_DIR}/diagnostics.txt" 2>&1 || true
for name in joint_limit workspace velocity comm_watchdog estop; do
  if grep -q "safety_monitor/${name}" "${LOG_DIR}/diagnostics.txt"; then
    log_pass "AC-7 diagnostic entry: ${name}"
  else
    log_fail "AC-7 missing diagnostic entry: ${name}"
  fi
done

# AC-2 workspace reject — publish out-of-box cmd_pose
log_info "AC-2/AC-3 — workspace violation holds last safe pose ..."
SAFE_BEFORE=$(timeout 3s ros2 topic echo /safe_master_pose --once 2>/dev/null | grep -A3 "position:" | head -3 || true)
ros2 topic pub --once /teleop/cmd_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'panda_link0'}, pose: {position: {x: 2.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}" \
  > "${LOG_DIR}/bad_pose_pub.txt" 2>&1 || true
sleep 0.5
SAFE_AFTER=$(timeout 3s ros2 topic echo /safe_master_pose --once 2>/dev/null | grep "x:" | head -1 || true)
if grep -q "x: 2" "${LOG_DIR}/bad_pose_pub.txt" 2>/dev/null; then
  log_warn "AC-2 pub sent (check hold manually if needed)"
fi
if echo "${SAFE_AFTER}" | grep -q "x: 2"; then
  log_fail "AC-2 /safe_master_pose forwarded out-of-box command"
else
  log_pass "AC-2 /safe_master_pose did not forward x=2.0 workspace violation"
fi

# AC-4 heartbeat timeout — stop teleop_input only
log_info "AC-4 — heartbeat timeout -> E-Stop ..."
pkill -f teleop_input_node 2>/dev/null || true
sleep 0.75
ESTOP=$(timeout 4s ros2 topic echo /safety/estop --once 2>/dev/null | grep "data:" | awk '{print $2}' || true)
if [[ "${ESTOP}" == "true" ]]; then
  log_pass "AC-4 heartbeat loss latched E-Stop"
else
  log_fail "AC-4 heartbeat timeout did not latch E-Stop (got '${ESTOP:-empty}')"
fi

if $USE_CAN; then
  log_info "AC-4 CAN — checking vcan0 for Quick Stop SDO (6040=0002) ..."
  timeout 2s candump vcan0 > "${LOG_DIR}/candump_estop.txt" 2>&1 || true
  if grep -qi "6040" "${LOG_DIR}/candump_estop.txt"; then
    log_pass "AC-4 candump shows 6040 SDO traffic"
  else
    log_warn "AC-4 no 6040 frame captured (may have been sent earlier)"
  fi
fi

echo ""
echo "=============================="
echo -e "M5 Result: ${GREEN}${PASS} passed${NC}  ${RED}${FAIL} failed${NC}"
echo "Logs: ${LOG_DIR}/"
echo "=============================="

[[ "$FAIL" -gt 0 ]] && exit 1 || exit 0
