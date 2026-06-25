#!/usr/bin/env bash
# validate_m6_perception_recorder.sh — M6 perception + LeRobot recorder checks.
#
# Usage (stack already running with recorder):
#   bash scripts/validate_m6_perception_recorder.sh
#
# Usage (launch full stack with recorder):
#   bash scripts/validate_m6_perception_recorder.sh --launch

set -euo pipefail

CYAN='\033[0;36m'; GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
PASS=0; FAIL=0; LAUNCH_PID=""

log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS + 1)); }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL + 1)); }
log_info() { echo -e "${CYAN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/.m6_validation"
OUT_DIR="${M6_OUTPUT_DIR:-${LOG_DIR}/episodes}"
mkdir -p "${LOG_DIR}"

set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f "${ROOT_DIR}/install/setup.bash" ]] && source "${ROOT_DIR}/install/setup.bash"
set -u

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    log_info "Shutting down launched stack (PID ${LAUNCH_PID})..."
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ "${1:-}" == "--launch" ]]; then
    bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  fi
}
trap 'cleanup "$@"' EXIT

if [[ "${1:-}" == "--launch" ]]; then
  bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  bash "${ROOT_DIR}/scripts/setup_vcan.sh" >/dev/null 2>&1 || true
  rm -rf "${OUT_DIR}"
  mkdir -p "${OUT_DIR}"
  log_info "Launching full_system.launch.py with record:=true ..."
  setsid ros2 launch teleop_bringup full_system.launch.py \
    headless:=true record:=true output_dir:="${OUT_DIR}" task:="m6_validation" \
    camera_width:="${M6_CAMERA_WIDTH:-320}" camera_height:="${M6_CAMERA_HEIGHT:-240}" \
    > "${LOG_DIR}/full_system.log" 2>&1 &
  LAUNCH_PID=$!
  log_info "Waiting 35 s for stack and recorder startup (PID ${LAUNCH_PID})..."
  sleep 35
fi

echo ""
echo "===================================="
echo "  M6 Perception + Recorder Validation"
echo "===================================="

log_info "AC-1 — /camera/color/image_raw frequency (expect >= 25 Hz) ..."
timeout 8s ros2 topic hz /camera/color/image_raw --window 50 \
  > "${LOG_DIR}/color_hz.txt" 2>&1 || true
COLOR_HZ=$(grep "average rate" "${LOG_DIR}/color_hz.txt" | tail -1 | awk '{print $3}' | tr -d ':' || true)
if [[ -n "${COLOR_HZ}" && "${COLOR_HZ%.*}" -ge 25 ]]; then
  log_pass "RGB camera @ ${COLOR_HZ} Hz"
else
  log_fail "RGB camera rate '${COLOR_HZ:-none}' (need >= 25 Hz)"
fi

log_info "AC-1 — /camera/depth/image_raw frequency (expect >= 25 Hz) ..."
timeout 8s ros2 topic hz /camera/depth/image_raw --window 50 \
  > "${LOG_DIR}/depth_hz.txt" 2>&1 || true
DEPTH_HZ=$(grep "average rate" "${LOG_DIR}/depth_hz.txt" | tail -1 | awk '{print $3}' | tr -d ':' || true)
if [[ -n "${DEPTH_HZ}" && "${DEPTH_HZ%.*}" -ge 25 ]]; then
  log_pass "Depth camera @ ${DEPTH_HZ} Hz"
else
  log_fail "Depth camera rate '${DEPTH_HZ:-none}' (need >= 25 Hz)"
fi

log_info "AC-3 — record a short episode ..."
timeout 6s ros2 topic pub --wait-matching-subscriptions 1 -r 5 \
  /teleop/record_trigger std_msgs/msg/String "{data: 'start'}" \
  > "${LOG_DIR}/record_start.txt" 2>&1 || true
sleep "${M6_RECORD_SECONDS:-10}"
timeout 6s ros2 topic pub --wait-matching-subscriptions 1 -r 5 \
  /teleop/record_trigger std_msgs/msg/String "{data: 'stop'}" \
  > "${LOG_DIR}/record_stop.txt" 2>&1 || true
sleep 5

DATASET_PATH="${OUT_DIR}/episode_000000/train"
if [[ -d "${DATASET_PATH}" ]]; then
  log_pass "Episode dataset created: ${DATASET_PATH}"
else
  log_fail "Episode dataset missing: ${DATASET_PATH}"
fi

log_info "AC-4/AC-5 — load dataset and check fields ..."
python3 - "${DATASET_PATH}" > "${LOG_DIR}/dataset_check.txt" 2>&1 <<'PY' || true
import sys
from datasets import load_from_disk

path = sys.argv[1]
ds = load_from_disk(path)
required = {
    "observation.state",
    "observation.ee_pose",
    "observation.ft",
    "observation.gripper",
    "observation.images.scene",
    "observation.depth.scene",
    "action",
    "timestamp",
    "episode_index",
    "frame_index",
    "done",
    "task",
    "safety_estop",
    "drive_fault",
}
missing = sorted(required.difference(ds.features.keys()))
print(f"frames={len(ds)}")
print(f"features={list(ds.features.keys())}")
if missing:
    raise SystemExit(f"missing fields: {missing}")
if len(ds) < 200:
    raise SystemExit(f"too few synchronized frames: {len(ds)}")
PY

if grep -q "frames=" "${LOG_DIR}/dataset_check.txt" && ! grep -q "missing fields\\|too few" "${LOG_DIR}/dataset_check.txt"; then
  log_pass "Dataset loads and contains required M6 fields"
else
  log_fail "Dataset load/field check failed — see ${LOG_DIR}/dataset_check.txt"
fi

echo ""
echo "===================================="
echo -e "M6 Result: ${GREEN}${PASS} passed${NC}  ${RED}${FAIL} failed${NC}"
echo "Logs: ${LOG_DIR}/"
echo "===================================="

[[ "$FAIL" -gt 0 ]] && exit 1 || exit 0
