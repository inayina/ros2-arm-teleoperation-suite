#!/usr/bin/env bash
# Run the first pre-batch collection gate: one accepted episode per target.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_ROOT="${BATCH_PREFLIGHT_OUTPUT_ROOT:-/tmp/ros2_arm_batch_preflight_${STAMP}_$$}"
LOG_DIR="${BATCH_PREFLIGHT_LOG_DIR:-${OUT_ROOT}/logs}"
OBJECTS="${BATCH_PREFLIGHT_OBJECTS:-object_red_box object_blue_cylinder object_green_sphere}"
EPISODES="${BATCH_PREFLIGHT_EPISODES:-1}"
MAX_ATTEMPTS="${BATCH_PREFLIGHT_MAX_ATTEMPTS:-8}"
RANDOMIZE="${BATCH_PREFLIGHT_RANDOMIZE:-false}"
HEADLESS="${BATCH_PREFLIGHT_HEADLESS:-true}"
WATCHDOG_TIMEOUT="${BATCH_PREFLIGHT_WATCHDOG_TIMEOUT:-2.0}"
SYNC_SLOP="${BATCH_PREFLIGHT_SYNC_SLOP:-2.5}"
SYNC_QUEUE_SIZE="${BATCH_PREFLIGHT_SYNC_QUEUE_SIZE:-120}"
VALIDATION_MODE="${BATCH_PREFLIGHT_VALIDATION_MODE:-place}"
ENABLE_GRASP_MONITOR="${BATCH_PREFLIGHT_ENABLE_GRASP_MONITOR:-true}"
ENABLE_TACTILE="${BATCH_PREFLIGHT_ENABLE_TACTILE:-false}"
SCENE_USE_MUJOCO_RENDERER="${BATCH_PREFLIGHT_SCENE_USE_MUJOCO_RENDERER:-false}"
WRIST_USE_MUJOCO_RENDERER="${BATCH_PREFLIGHT_WRIST_USE_MUJOCO_RENDERER:-false}"
#
# IMPORTANT: downstream Panda schema expects scene RGB shape [240, 320, 3].
# The recorder writes image shapes as-is, so we must make the scene camera
# publish exactly 320x240 (height=240, width=320).
#
CAMERA_WIDTH="${BATCH_PREFLIGHT_CAMERA_WIDTH:-320}"
CAMERA_HEIGHT="${BATCH_PREFLIGHT_CAMERA_HEIGHT:-240}"
WRIST_CAMERA_WIDTH="${BATCH_PREFLIGHT_WRIST_CAMERA_WIDTH:-320}"
WRIST_CAMERA_HEIGHT="${BATCH_PREFLIGHT_WRIST_CAMERA_HEIGHT:-240}"
CAMERA_RATE="${BATCH_PREFLIGHT_CAMERA_RATE:-10.0}"
CONTROLLER_TIMEOUT_S="${BATCH_PREFLIGHT_CONTROLLER_TIMEOUT_S:-45}"
BATCH_TIMEOUT_S="${BATCH_PREFLIGHT_BATCH_TIMEOUT_S:-450}"
DATASET_WAIT_S="${BATCH_PREFLIGHT_DATASET_WAIT_S:-45}"
LAUNCH_SHUTDOWN_TIMEOUT_S="${BATCH_PREFLIGHT_LAUNCH_SHUTDOWN_TIMEOUT_S:-5}"
# Image flush on commit can exceed 5s for long RGBD episodes; keep headroom.
RECORDER_SETTLE_S="${BATCH_PREFLIGHT_RECORDER_SETTLE_S:-45.0}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-92}"
MUJOCO_GL="${MUJOCO_GL:-egl}"

HOVER_DURATION="${BATCH_PREFLIGHT_HOVER_DURATION:-4.0}"
HOVER_HEIGHT="${BATCH_PREFLIGHT_HOVER_HEIGHT:-0.12}"
DESCEND_DURATION="${BATCH_PREFLIGHT_DESCEND_DURATION:-4.0}"
CLOSE_DURATION="${BATCH_PREFLIGHT_CLOSE_DURATION:-3.0}"
GRASP_PAUSE="${BATCH_PREFLIGHT_GRASP_PAUSE:-3.0}"
# Use M7-proven absolute pick offset (docs/M7_GRASP_DEBUGGING.md), not shape sentinel.
PICK_HEIGHT_OFFSET="${BATCH_PREFLIGHT_PICK_HEIGHT_OFFSET:-0.015}"
GRIPPER_CLOSE_TARGET="${BATCH_PREFLIGHT_GRIPPER_CLOSE_TARGET:-0.0}"
POSE_STEP_M="${BATCH_PREFLIGHT_POSE_STEP_M:-0.008}"
POSE_CMD_RATE_HZ="${BATCH_PREFLIGHT_POSE_CMD_RATE_HZ:-100.0}"
LIFT_DURATION="${BATCH_PREFLIGHT_LIFT_DURATION:-10.0}"
LIFT_TARGET_Z="${BATCH_PREFLIGHT_LIFT_TARGET_Z:-0.12}"
POST_LIFT_HOLD="${BATCH_PREFLIGHT_POST_LIFT_HOLD:-8.0}"
LIFT_SUCCESS_DELTA="${BATCH_PREFLIGHT_LIFT_SUCCESS_DELTA:-0.03}"
BIN_XY_TOLERANCE="${BATCH_PREFLIGHT_BIN_XY_TOLERANCE:-0.08}"
REQUIRE_GRIPPER_CLOSE="${BATCH_PREFLIGHT_REQUIRE_GRIPPER_CLOSE:-true}"
GRIPPER_CLOSE_MAX="${BATCH_PREFLIGHT_GRIPPER_CLOSE_MAX:-0.12}"
# Portfolio fallback only: physics grasp may still fail without assist.
GRASP_ASSIST="${BATCH_PREFLIGHT_GRASP_ASSIST:-false}"
GRIPPER_FORCE_MAX_N="${BATCH_PREFLIGHT_GRIPPER_FORCE_MAX_N:-30.0}"
GRIPPER_CONTACT_HOLD_MARGIN="${BATCH_PREFLIGHT_GRIPPER_CONTACT_HOLD_MARGIN:-0.006}"
GRIPPER_FORCE_SQUEEZE_MARGIN_MAX="${BATCH_PREFLIGHT_GRIPPER_FORCE_SQUEEZE_MARGIN_MAX:-0.008}"
SERVO_MODE="${BATCH_PREFLIGHT_SERVO_MODE:-pose}"
MOTION_MODE="${BATCH_PREFLIGHT_MOTION_MODE:-pose}"
TWIST_MAX_LINEAR_MPS="${BATCH_PREFLIGHT_TWIST_MAX_LINEAR_MPS:-0.05}"
TWIST_DESCEND_LINEAR_MPS="${BATCH_PREFLIGHT_TWIST_DESCEND_LINEAR_MPS:-0.04}"
EE_TRACKING_TOLERANCE_M="${BATCH_PREFLIGHT_EE_TRACKING_TOLERANCE_M:-0.08}"
# 0.05 is too tight leaving Panda home singularity; 0.08 is enough for G0 without
# the old 0.12 false-reach problem near home.
EE_XY_TOLERANCE="${BATCH_PREFLIGHT_EE_XY_TOLERANCE:-0.08}"
EE_Z_TOLERANCE="${BATCH_PREFLIGHT_EE_Z_TOLERANCE:-0.01}"
RECORD_WARMUP_S="${BATCH_PREFLIGHT_RECORD_WARMUP_S:-3.0}"

LAUNCH_PID=""
HB_PID=""
ROS_HB_PID=""

instruction_for() {
  case "$1" in
    object_red_box) echo "pick up the red box and place it in the left bin" ;;
    object_blue_cylinder) echo "pick up the blue cylinder and place it in the right bin" ;;
    object_green_sphere) echo "pick up the green sphere and place it in the left bin" ;;
    *) echo "pick up $1" ;;
  esac
}

cleanup() {
  if [[ -n "${ROS_HB_PID}" ]]; then kill "${ROS_HB_PID}" 2>/dev/null || true; fi
  if [[ -n "${HB_PID}" ]]; then kill "${HB_PID}" 2>/dev/null || true; fi
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait_with_timeout "${LAUNCH_PID}" "${LAUNCH_SHUTDOWN_TIMEOUT_S}" "ROS 2 launch shutdown" || true
    LAUNCH_PID=""
  fi
  bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

wait_with_timeout() {
  local pid="$1"
  local timeout_s="$2"
  local label="$3"
  local deadline=$((SECONDS + timeout_s))
  while kill -0 "${pid}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      echo "[preflight] timed out waiting for ${label}" >&2
      kill "${pid}" 2>/dev/null || true
      return 1
    fi
    sleep 0.5
  done
  wait "${pid}"
}

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

episode_accepted() {
  local log_file="$1"
  local expected="$2"
  if grep -q "Accepted Episode ${expected}/${expected}" "${log_file}"; then
    return 0
  fi
  if grep -Eq "Batch progress: ${expected}/${expected} accepted" "${log_file}"; then
    return 0
  fi
  return 1
}

count_episode_train_dirs() {
  local root="$1"
  local legacy_count
  legacy_count="$(find "${root}" -mindepth 2 -maxdepth 2 -type d -name train 2>/dev/null | wc -l)"
  if (( legacy_count > 0 )); then
    echo "${legacy_count}"
  else
    find "${root}" -mindepth 1 -maxdepth 2 -type d -name "episode_*" 2>/dev/null | wc -l
  fi
}

wait_for_episode_dirs() {
  local root="$1"
  local min_episodes="$2"
  local timeout_s="$3"
  local deadline=$((SECONDS + timeout_s))
  local found=0
  while (( SECONDS < deadline )); do
    found="$(count_episode_train_dirs "${root}")"
    if (( found >= min_episodes )); then
      echo "[preflight] found ${found} episode train dir(s) under ${root}"
      return 0
    fi
    sleep 0.5
  done
  echo "[preflight] timed out waiting for ${min_episodes} episode train dir(s); found ${found} under ${root}" >&2
  return 1
}

request_safety_reset() {
  echo "[preflight] waiting for /safety/reset service..."
  timeout 15s ros2 service wait /safety/reset --timeout 12 >/dev/null 2>&1 || true
  echo "[preflight] calling reset..."
  for i in {1..15}; do
    local output
    output="$(timeout 5s ros2 service call /safety/reset std_srvs/srv/Trigger "{}" 2>&1 || true)"
    echo "[preflight] reset attempt $i: ${output}"
    if [[ "${output}" == *"success=True"* || "${output}" == *"success: true"* || "${output}" == *"E-Stop reset"* || "${output}" == *"already clear"* ]]; then
      echo "[preflight] safety reset successful!"
      return 0
    fi
    sleep 0.5
  done
  echo "[preflight] safety reset failed after 15 attempts!" >&2
  return 1
}

mkdir -p "${OUT_ROOT}" "${LOG_DIR}"
export ROS_DOMAIN_ID MUJOCO_GL ROS2CLI_DISABLE_DAEMON=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-${LOG_DIR}/ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/jazzy/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

echo "[preflight] output root: ${OUT_ROOT}"
echo "[preflight] objects: ${OBJECTS}"

bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true

setsid ros2 launch teleop_bringup full_system.launch.py \
  use_sim:=true \
  start_teleop:=false \
  headless:="${HEADLESS}" \
  record:=true \
  output_dir:="${OUT_ROOT}" \
  capture_mode:="${BATCH_PREFLIGHT_CAPTURE_MODE:-portfolio}" \
  servo_mode:="${SERVO_MODE}" \
  sync_slop:="${SYNC_SLOP}" \
  sync_queue_size:="${SYNC_QUEUE_SIZE}" \
  randomize:="${RANDOMIZE}" \
  contact_debug_enabled:=true \
  contact_debug_period_s:=1.0 \
  camera_width:="${CAMERA_WIDTH}" \
  camera_height:="${CAMERA_HEIGHT}" \
  camera_rate:="${CAMERA_RATE}" \
  publish_depth:=false \
  enable_wrist_camera:=false \
  scene_use_mujoco_renderer:="${SCENE_USE_MUJOCO_RENDERER}" \
  wrist_camera_width:="${WRIST_CAMERA_WIDTH}" \
  wrist_camera_height:="${WRIST_CAMERA_HEIGHT}" \
  wrist_use_mujoco_renderer:="${WRIST_USE_MUJOCO_RENDERER}" \
  enable_tactile:="${ENABLE_TACTILE}" \
  grasp_assist_enabled:="${GRASP_ASSIST}" \
  enable_grasp_monitor:="${ENABLE_GRASP_MONITOR}" \
  gripper_force_max_n:="${GRIPPER_FORCE_MAX_N}" \
  gripper_contact_hold_margin:="${GRIPPER_CONTACT_HOLD_MARGIN}" \
  gripper_force_squeeze_margin_max:="${GRIPPER_FORCE_SQUEEZE_MARGIN_MAX}" \
  watchdog_timeout:="${WATCHDOG_TIMEOUT}" \
  > "${LOG_DIR}/full_system.log" 2>&1 &
LAUNCH_PID=$!

echo "[preflight] waiting for cartesian_impedance_controller..."
sleep 3
if ! wait_for_controller; then
  echo "[preflight] controller did not become active; see ${LOG_DIR}/full_system.log" >&2
  exit 1
fi
echo "[preflight] waiting 12s for late-running nodes to settle..."
sleep 12

timeout 8s ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType \
  "{command_type: $([ "${SERVO_MODE}" = twist ] && echo 1 || echo 2)}" >/dev/null 2>&1 || true
nice -n 19 ionice -c 3 python3 "${ROOT_DIR}/scripts/publish_dummy_heartbeat.py" --rate 50 >/dev/null 2>&1 &
HB_PID=$!
nice -n 19 ionice -c 3 ros2 topic pub -r 30 /teleop/heartbeat std_msgs/msg/Header \
  "{frame_id: 'batch_preflight_heartbeat'}" >/dev/null 2>&1 &
ROS_HB_PID=$!
request_safety_reset
pkill -f "teleop_input_node" 2>/dev/null || true
sleep 1.0
request_safety_reset
export SERVO_POST_INIT_MODE="${SERVO_MODE}"
bash "${ROOT_DIR}/scripts/servo_post_init.sh" 2 15 || true
sleep 0.5

# Publish nominal home pose once to initialize the controller target and lift the arm
echo "[preflight] initializing controller target to home pose..."
timeout 5s ros2 topic pub -1 /teleop/cmd_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'panda_link0'}, pose: {position: {x: 0.307, y: 0.0, z: 0.490}, orientation: {w: 0.0, x: 1.0, y: 0.0, z: 0.0}}}" >/dev/null 2>&1 || true
sleep 1.0

for object_name in ${OBJECTS}; do
  instruction="$(instruction_for "${object_name}")"
  log_file="${LOG_DIR}/batch_${object_name}.log"
  echo "[preflight] running ${object_name} -> ${instruction}"
  request_safety_reset
  timeout 4s ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
  nice -n 19 ionice -c 3 ros2 run synth_data_gen batch_generator --ros-args \
    -p target_object_name:="${object_name}" \
    -p language_instruction:="${instruction}" \
    -p validation_mode:="${VALIDATION_MODE}" \
    -p episodes:="${EPISODES}" \
    -p max_attempts_per_episode:="${MAX_ATTEMPTS}" \
    -p fail_on_max_attempts:=true \
    -p lift_success_delta:="${LIFT_SUCCESS_DELTA}" \
    -p bin_xy_tolerance:="${BIN_XY_TOLERANCE}" \
    -p require_gripper_close:="${REQUIRE_GRIPPER_CLOSE}" \
    -p gripper_close_max:="${GRIPPER_CLOSE_MAX}" \
    -p hover_duration:="${HOVER_DURATION}" \
    -p hover_height:="${HOVER_HEIGHT}" \
    -p pose_step_m:="${POSE_STEP_M}" \
    -p pose_cmd_rate_hz:="${POSE_CMD_RATE_HZ}" \
    -p descend_duration:="${DESCEND_DURATION}" \
    -p close_duration:="${CLOSE_DURATION}" \
    -p grasp_pause:="${GRASP_PAUSE}" \
    -p pick_height_offset:="${PICK_HEIGHT_OFFSET}" \
    -p gripper_close_target:="${GRIPPER_CLOSE_TARGET}" \
    -p lift_duration:="${LIFT_DURATION}" \
    -p lift_target_z:="${LIFT_TARGET_Z}" \
    -p post_lift_hold:="${POST_LIFT_HOLD}" \
    -p recorder_settle_s:="${RECORDER_SETTLE_S}" \
    -p record_warmup_s:="${RECORD_WARMUP_S}" \
    -p ee_xy_tolerance:="${EE_XY_TOLERANCE}" \
    -p ee_z_tolerance:="${EE_Z_TOLERANCE}" \
    -p motion_mode:="${MOTION_MODE}" \
    -p twist_max_linear_mps:="${TWIST_MAX_LINEAR_MPS}" \
    -p twist_descend_linear_mps:="${TWIST_DESCEND_LINEAR_MPS}" \
    -p ee_tracking_tolerance_m:="${EE_TRACKING_TOLERANCE_M}" \
    > "${log_file}" 2>&1 &
  batch_pid=$!
  if ! wait_with_timeout "${batch_pid}" "${BATCH_TIMEOUT_S}" "batch_generator ${object_name}"; then
    echo "[preflight] ${object_name} failed; see ${log_file}" >&2
    exit 1
  fi
  if ! episode_accepted "${log_file}" "${EPISODES}"; then
    echo "[preflight] ${object_name} was not accepted; see ${log_file}" >&2
    exit 1
  fi
  grep -E "Accepted Episode|Batch progress" "${log_file}" | tail -1 || true
done

expected_train_dirs=$(( $(echo "${OBJECTS}" | wc -w) * EPISODES ))
if ! wait_for_episode_dirs "${OUT_ROOT}" "${expected_train_dirs}" "${DATASET_WAIT_S}"; then
  echo "[preflight] recorder did not flush episodes; check ${LOG_DIR}/full_system.log for lerobot_recorder" >&2
  exit 1
fi

python3 "${ROOT_DIR}/scripts/validate_dataset.py" "${OUT_ROOT}" --min-frames 5 --json \
  > "${LOG_DIR}/dataset_validation.json"
cat "${LOG_DIR}/dataset_validation.json"

# Stop background stack before the EXIT trap to avoid shutdown hangs.
if [[ -n "${ROS_HB_PID}" ]]; then kill "${ROS_HB_PID}" 2>/dev/null || true; ROS_HB_PID=""; fi
if [[ -n "${HB_PID}" ]]; then kill "${HB_PID}" 2>/dev/null || true; HB_PID=""; fi
if [[ -n "${LAUNCH_PID}" ]]; then
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  wait_with_timeout "${LAUNCH_PID}" "${LAUNCH_SHUTDOWN_TIMEOUT_S}" "ROS 2 launch shutdown" || true
  LAUNCH_PID=""
fi
bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true

echo "[preflight] PASS: smoke dataset is ACT-ready at ${OUT_ROOT}"
