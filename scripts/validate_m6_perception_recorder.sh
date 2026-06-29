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
PASS=0; FAIL=0; LAUNCH_PID=""; LAUNCHED=false

log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS + 1)); }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL + 1)); }
log_info() { echo -e "${CYAN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/.m6_validation"
OUT_DIR="${M6_OUTPUT_DIR:-${LOG_DIR}/episodes}"
MIN_CAMERA_HZ="${M6_MIN_CAMERA_HZ:-25}"
MIN_FRAMES="${M6_MIN_FRAMES:-200}"
ORIGINAL_HOME="${HOME:-}"
PY_VER="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
USER_SITE="${ORIGINAL_HOME}/.local/lib/python${PY_VER}/site-packages"
mkdir -p "${LOG_DIR}"
export HOME="${M6_HOME:-${LOG_DIR}/home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${LOG_DIR}/ros_logs}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
if [[ -d "${USER_SITE}" ]]; then
  export PYTHONPATH="${USER_SITE}${PYTHONPATH:+:${PYTHONPATH}}"
fi
mkdir -p "${HOME}/.ros/locks" "${HOME}/.ros/log" "${ROS_LOG_DIR}"

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
  LAUNCHED=true
  AUTO_RECORD_SECONDS="0.0"
  AUTO_RECORD_DELAY_S="0.0"
  if [[ "${M6_AUTO_RECORD:-true}" == "true" ]]; then
    AUTO_RECORD_SECONDS="${M6_RECORD_SECONDS:-10}"
    AUTO_RECORD_DELAY_S="${M6_AUTO_RECORD_DELAY_S:-12}"
  fi
  bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  rm -rf "${OUT_DIR}"
  mkdir -p "${OUT_DIR}"
  log_info "Launching full_system.launch.py with record:=true ..."
  setsid ros2 launch teleop_bringup full_system.launch.py \
    headless:=true record:=true output_dir:="${OUT_DIR}" task:="m6_validation" \
    sync_slop:="${M6_SYNC_SLOP:-0.12}" sync_queue_size:="${M6_SYNC_QUEUE_SIZE:-120}" \
    auto_record_seconds:="${AUTO_RECORD_SECONDS}" auto_record_delay_s:="${AUTO_RECORD_DELAY_S}" \
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

log_info "AC-1 — /camera/color/image_raw frequency (expect >= ${MIN_CAMERA_HZ} Hz) ..."
timeout 8s ros2 topic hz /camera/color/image_raw --window 50 \
  > "${LOG_DIR}/color_hz.txt" 2>&1 || true
COLOR_HZ=$(grep "average rate" "${LOG_DIR}/color_hz.txt" | tail -1 | awk '{print $3}' | tr -d ':' || true)
if [[ -n "${COLOR_HZ}" && "${COLOR_HZ%.*}" -ge "${MIN_CAMERA_HZ%.*}" ]]; then
  log_pass "RGB camera @ ${COLOR_HZ} Hz"
else
  log_fail "RGB camera rate '${COLOR_HZ:-none}' (need >= ${MIN_CAMERA_HZ} Hz)"
fi

log_info "AC-1 — /camera/depth/image_raw frequency (expect >= ${MIN_CAMERA_HZ} Hz) ..."
timeout 8s ros2 topic hz /camera/depth/image_raw --window 50 \
  > "${LOG_DIR}/depth_hz.txt" 2>&1 || true
DEPTH_HZ=$(grep "average rate" "${LOG_DIR}/depth_hz.txt" | tail -1 | awk '{print $3}' | tr -d ':' || true)
if [[ -n "${DEPTH_HZ}" && "${DEPTH_HZ%.*}" -ge "${MIN_CAMERA_HZ%.*}" ]]; then
  log_pass "Depth camera @ ${DEPTH_HZ} Hz"
else
  log_fail "Depth camera rate '${DEPTH_HZ:-none}' (need >= ${MIN_CAMERA_HZ} Hz)"
fi

log_info "AC-1 — /camera/wrist/color/image_raw frequency (expect >= ${MIN_CAMERA_HZ} Hz) ..."
timeout 8s ros2 topic hz /camera/wrist/color/image_raw --window 50 \
  > "${LOG_DIR}/wrist_color_hz.txt" 2>&1 || true
WRIST_HZ=$(grep "average rate" "${LOG_DIR}/wrist_color_hz.txt" | tail -1 | awk '{print $3}' | tr -d ':' || true)
if [[ -n "${WRIST_HZ}" && "${WRIST_HZ%.*}" -ge "${MIN_CAMERA_HZ%.*}" ]]; then
  log_pass "Wrist RGB camera @ ${WRIST_HZ} Hz"
else
  log_fail "Wrist RGB camera rate '${WRIST_HZ:-none}' (need >= ${MIN_CAMERA_HZ} Hz)"
fi

log_info "AC-1 — /camera/tactile_left/image_raw frequency (expect >= ${MIN_CAMERA_HZ} Hz) ..."
timeout 8s ros2 topic hz /camera/tactile_left/image_raw --window 50 \
  > "${LOG_DIR}/tactile_left_hz.txt" 2>&1 || true
TACTILE_LEFT_HZ=$(grep "average rate" "${LOG_DIR}/tactile_left_hz.txt" | tail -1 | awk '{print $3}' | tr -d ':' || true)
if [[ -n "${TACTILE_LEFT_HZ}" && "${TACTILE_LEFT_HZ%.*}" -ge "${MIN_CAMERA_HZ%.*}" ]]; then
  log_pass "Left tactile camera @ ${TACTILE_LEFT_HZ} Hz"
else
  log_fail "Left tactile camera rate '${TACTILE_LEFT_HZ:-none}' (need >= ${MIN_CAMERA_HZ} Hz)"
fi

log_info "AC-1 — /camera/tactile_right/image_raw frequency (expect >= ${MIN_CAMERA_HZ} Hz) ..."
timeout 8s ros2 topic hz /camera/tactile_right/image_raw --window 50 \
  > "${LOG_DIR}/tactile_right_hz.txt" 2>&1 || true
TACTILE_RIGHT_HZ=$(grep "average rate" "${LOG_DIR}/tactile_right_hz.txt" | tail -1 | awk '{print $3}' | tr -d ':' || true)
if [[ -n "${TACTILE_RIGHT_HZ}" && "${TACTILE_RIGHT_HZ%.*}" -ge "${MIN_CAMERA_HZ%.*}" ]]; then
  log_pass "Right tactile camera @ ${TACTILE_RIGHT_HZ} Hz"
else
  log_fail "Right tactile camera rate '${TACTILE_RIGHT_HZ:-none}' (need >= ${MIN_CAMERA_HZ} Hz)"
fi

log_info "AC-3 — record a short episode ..."
if [[ "${LAUNCHED}" == "true" && "${M6_AUTO_RECORD:-true}" == "true" ]]; then
  echo "auto_record_seconds=${AUTO_RECORD_SECONDS}" > "${LOG_DIR}/record_start.txt"
  echo "auto_record_delay_s=${AUTO_RECORD_DELAY_S}" > "${LOG_DIR}/record_stop.txt"
  sleep 3
else
  timeout 6s ros2 topic pub --wait-matching-subscriptions 1 -r 5 \
    /teleop/record_trigger std_msgs/msg/String "{data: 'start'}" \
    > "${LOG_DIR}/record_start.txt" 2>&1 || true
  sleep "${M6_RECORD_SECONDS:-10}"
  timeout 6s ros2 topic pub --wait-matching-subscriptions 1 -r 5 \
    /teleop/record_trigger std_msgs/msg/String "{data: 'stop'}" \
    > "${LOG_DIR}/record_stop.txt" 2>&1 || true
  sleep 5
fi

DATASET_PATH="${OUT_DIR}/episode_000000/train"
for _ in $(seq 1 "${M6_DATASET_WAIT_SECONDS:-20}"); do
  [[ -d "${DATASET_PATH}" ]] && break
  sleep 1
done
if [[ -d "${DATASET_PATH}" ]]; then
  log_pass "Episode dataset created: ${DATASET_PATH}"
else
  log_fail "Episode dataset missing: ${DATASET_PATH}"
fi

log_info "AC-4/AC-5 — load dataset and check fields ..."
python3 - "${DATASET_PATH}" "${MIN_FRAMES}" > "${LOG_DIR}/dataset_check.txt" 2>&1 <<'PY' || true
import sys
from datasets import load_from_disk

path = sys.argv[1]
min_frames = int(sys.argv[2])
ds = load_from_disk(path)
required = {
    "observation.state",
    "observation.ee_pose",
    "observation.ft",
    "observation.gripper",
    "observation.images.scene",
    "observation.images.wrist",
    "observation.images.tactile_left",
    "observation.images.tactile_right",
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
if len(ds) < min_frames:
    raise SystemExit(f"too few synchronized frames: {len(ds)} < {min_frames}")
PY

if grep -q "frames=" "${LOG_DIR}/dataset_check.txt" && ! grep -q "missing fields\\|too few" "${LOG_DIR}/dataset_check.txt"; then
  log_pass "Dataset loads and contains required M6 fields"
else
  log_fail "Dataset load/field check failed — see ${LOG_DIR}/dataset_check.txt"
fi

if [[ "${M6_CAPTURE_MEDIA:-true}" == "true" ]]; then
  log_info "M6 media — capture fresh scene/wrist/tactile PNGs and dataset schema ..."
  if python3 "${ROOT_DIR}/scripts/capture_m6_media.py" \
      --output "${ROOT_DIR}/media/m6" \
      --dataset "${DATASET_PATH}" \
      --timeout "${M6_CAPTURE_TIMEOUT:-8}" \
      > "${LOG_DIR}/capture_m6_media.txt" 2>&1; then
    log_pass "Fresh M6 media captured in media/m6/"
  else
    log_fail "Fresh M6 media capture failed — see ${LOG_DIR}/capture_m6_media.txt"
  fi
fi

echo ""
echo "===================================="
echo -e "M6 Result: ${GREEN}${PASS} passed${NC}  ${RED}${FAIL} failed${NC}"
echo "Logs: ${LOG_DIR}/"
echo "===================================="

[[ "$FAIL" -gt 0 ]] && exit 1 || exit 0
