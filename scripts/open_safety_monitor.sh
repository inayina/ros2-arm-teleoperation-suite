#!/usr/bin/env bash
# Open the safety diagnostics GUI with the correct topic and environment.
#
# Usage:
#   bash scripts/open_safety_monitor.sh              # launch full stack, then open GUI
#   bash scripts/open_safety_monitor.sh --no-launch  # use an already-running stack

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/.m5_validation"
mkdir -p "${LOG_DIR}"

LAUNCH_STACK=true
if [[ "${1:-}" == "--no-launch" ]]; then
  LAUNCH_STACK=false
fi

set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f "${ROOT_DIR}/install/setup.bash" ]] && source "${ROOT_DIR}/install/setup.bash"
set -u

if ! ros2 pkg prefix rqt_robot_monitor >/dev/null 2>&1; then
  echo "[open_safety_monitor] Missing package: ros-jazzy-rqt-robot-monitor"
  echo "Install it with: sudo apt install -y ros-jazzy-rqt-robot-monitor"
  exit 1
fi

LAUNCH_PID=""
cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
    bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

if ${LAUNCH_STACK}; then
  bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  echo "[open_safety_monitor] Launching full stack (headless)..."
  setsid ros2 launch teleop_bringup full_system.launch.py headless:=true     > "${LOG_DIR}/safety_monitor_gui_stack.log" 2>&1 &
  LAUNCH_PID=$!
fi

echo "[open_safety_monitor] Waiting for /diagnostics_agg..."
for _ in {1..80}; do
  if timeout 5s ros2 topic echo /diagnostics_agg --once >/tmp/safety_monitor_diag_once.txt 2>/dev/null; then
    break
  fi
  sleep 0.5
done

if ! timeout 5s ros2 topic echo /diagnostics_agg --once >/tmp/safety_monitor_diag_once.txt 2>/dev/null; then
  echo "[open_safety_monitor] No /diagnostics_agg data yet."
  echo "Launch log: ${LOG_DIR}/safety_monitor_gui_stack.log"
  echo "If you already have the stack running, use: bash scripts/open_safety_monitor.sh --no-launch"
  exit 1
fi

echo "[open_safety_monitor] Diagnostics are live. Opening rqt_robot_monitor."
echo "[open_safety_monitor] In the GUI, use topic: /diagnostics_agg and expand /Safety."
exec ros2 run rqt_robot_monitor rqt_robot_monitor --clear-config
