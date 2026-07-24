#!/usr/bin/env bash
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly OUTPUT_DIR="${1:-/tmp/isaac_act_smoke}"
readonly CHECKPOINT="${CHECKPOINT:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab/data/e2_rendered_act_1epoch/checkpoint.pt}"
readonly ISAAC_PYTHON="${ISAAC_PYTHON:-/home/ina/isaacsim/.venv/bin/python}"
readonly POLICY_PYTHON="${POLICY_PYTHON:-/home/ina/miniforge3/envs/lerobot/bin/python}"
readonly MAX_ACTIONS="${MAX_ACTIONS:-1}"
readonly DRY_RUN="${DRY_RUN:-false}"
readonly ARM_COMMAND_MODE="${ARM_COMMAND_MODE:-position}"
readonly MAX_JOINT_EXCURSION_RAD="${MAX_JOINT_EXCURSION_RAD:-0.25}"
readonly MAX_EE_EXCURSION_M="${MAX_EE_EXCURSION_M:-0.03}"
readonly MAX_TRANSLATION_M="${MAX_TRANSLATION_M:-0.005}"
readonly MAX_ROTATION_RAD="${MAX_ROTATION_RAD:-0.034906585}"
readonly INFERENCE_RATE_HZ="${INFERENCE_RATE_HZ:-1.0}"
readonly POLICY_STARTUP_TIMEOUT_S="${POLICY_STARTUP_TIMEOUT_S:-30.0}"
readonly POLICY_RUNTIME_TIMEOUT_S="${POLICY_RUNTIME_TIMEOUT_S:-50}"
readonly PREGRASP_WARMSTART="${PREGRASP_WARMSTART:-false}"
readonly PREGRASP_DURATION_S="${PREGRASP_DURATION_S:-8.0}"
# 0 = checkpoint chunk_size; smaller values replan more often (closed-loop).
readonly N_ACTION_STEPS="${N_ACTION_STEPS:-0}"
# Grasp needs z below the old 0.15 m floor (object ~0.025 m).
readonly WORKSPACE_MIN="${WORKSPACE_MIN:-0.20,-0.40,0.02}"
readonly WORKSPACE_MAX="${WORKSPACE_MAX:-0.65,0.40,0.75}"
# Isaac backend wall time; must cover bringup + policy horizon for long smoke.
readonly BACKEND_DURATION_SEC="${BACKEND_DURATION_SEC:-100}"
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-83}"
readonly RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Optional seeded object placement (training distribution when set).
readonly OBJECT_SEED="${OBJECT_SEED:-}"
readonly OBJECT_XY="${OBJECT_XY:-}"
# Continuous GT / video (suite path); empty disables.
readonly EPISODE_RESULTS_PATH="${EPISODE_RESULTS_PATH:-}"
readonly EVALUATION_RUN_ID="${EVALUATION_RUN_ID:-}"
readonly EVALUATION_MODEL_ID="${EVALUATION_MODEL_ID:-isaac_act_policy}"
readonly EVALUATION_SEED="${EVALUATION_SEED:-${OBJECT_SEED:-0}}"
readonly EVALUATION_EPISODE_INDEX="${EVALUATION_EPISODE_INDEX:-0}"
readonly RECORD_SCENE_VIDEO="${RECORD_SCENE_VIDEO:-false}"
readonly REQUIRE_REPORT_PASS="${REQUIRE_REPORT_PASS:-true}"

export ROS_DOMAIN_ID RMW_IMPLEMENTATION
mkdir -p "${OUTPUT_DIR}"

backend_pid=""
stack_pid=""
gpu_pid=""
video_pid=""
gt_pid=""

cleanup() {
  [[ -z "${video_pid}" ]] || kill "${video_pid}" 2>/dev/null || true
  [[ -z "${gt_pid}" ]] || kill "${gt_pid}" 2>/dev/null || true
  [[ -z "${gpu_pid}" ]] || kill "${gpu_pid}" 2>/dev/null || true
  [[ -z "${stack_pid}" ]] || kill "${stack_pid}" 2>/dev/null || true
  [[ -z "${backend_pid}" ]] || kill "${backend_pid}" 2>/dev/null || true
  [[ -z "${video_pid}" ]] || wait "${video_pid}" 2>/dev/null || true
  [[ -z "${gt_pid}" ]] || wait "${gt_pid}" 2>/dev/null || true
  [[ -z "${gpu_pid}" ]] || wait "${gpu_pid}" 2>/dev/null || true
  [[ -z "${stack_pid}" ]] || wait "${stack_pid}" 2>/dev/null || true
  [[ -z "${backend_pid}" ]] || wait "${backend_pid}" 2>/dev/null || true
  "${REPO_ROOT}/scripts/stop_stack.sh" > "${OUTPUT_DIR}/cleanup.log" 2>&1 || true
  pkill -9 -f "isaac_panda_backend.py" 2>/dev/null || true
  pkill -9 -f "isaac_scene_video_recorder.py" 2>/dev/null || true
  pkill -9 -f "isaac_continuous_gt_recorder.py" 2>/dev/null || true
  pkill -9 -f "policy_inference_node" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -f "${CHECKPOINT}" ]]; then
  echo "ACT checkpoint not found: ${CHECKPOINT}" >&2
  exit 2
fi
if [[ ! -x "${ISAAC_PYTHON}" || ! -x "${POLICY_PYTHON}" ]]; then
  echo "Isaac or ACT Python is not executable" >&2
  exit 2
fi
if [[ "${ARM_COMMAND_MODE}" != "effort" \
  && "${ARM_COMMAND_MODE}" != "position" ]]; then
  echo "ARM_COMMAND_MODE must be effort or position" >&2
  exit 2
fi
if [[ "${ARM_COMMAND_MODE}" == "position" ]]; then
  controller_profile="forward"
else
  controller_profile="impedance"
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u

backend_args=(
  --duration-sec "${BACKEND_DURATION_SEC}"
  --camera-rate 10
  --command-timeout-s 0.1
  --arm-command-mode "${ARM_COMMAND_MODE}"
)
if [[ -n "${OBJECT_SEED}" ]]; then
  backend_args+=(--object-seed "${OBJECT_SEED}")
fi
if [[ -n "${OBJECT_XY}" ]]; then
  backend_args+=(--object-xy "${OBJECT_XY}")
fi
if [[ -n "${ISAAC_FRANKA_USD:-}" ]]; then
  backend_args+=(--franka-usd "${ISAAC_FRANKA_USD}")
fi

timeout "$((BACKEND_DURATION_SEC + 30))s" "${ISAAC_PYTHON}" \
  "${REPO_ROOT}/src/isaac_sim_adapter/scripts/isaac_panda_backend.py" \
  "${backend_args[@]}" \
  > "${OUTPUT_DIR}/backend.log" 2>&1 &
backend_pid=$!

ready=false
for _ in $(seq 1 140); do
  if grep -q "ISAAC_E1_READY=" "${OUTPUT_DIR}/backend.log" 2>/dev/null; then
    ready=true
    break
  fi
  if ! kill -0 "${backend_pid}" 2>/dev/null; then
    tail -n 80 "${OUTPUT_DIR}/backend.log" >&2
    exit 3
  fi
  sleep 0.5
done
if [[ "${ready}" != "true" ]]; then
  echo "Isaac backend READY timeout" >&2
  exit 3
fi

timeout 85s ros2 launch teleop_bringup full_system.launch.py \
  sim_backend:=isaac record:=false start_teleop:=false \
  controller:="${controller_profile}" \
  enable_grasp_monitor:=false camera_rate:=10.0 watchdog_timeout:=30.0 \
  > "${OUTPUT_DIR}/full_system.log" 2>&1 &
stack_pid=$!

graph_ready=false
for _ in $(seq 1 180); do
  if ros2 topic list 2>/dev/null | grep -qx "/sim/encoder_state" \
    && ros2 topic list 2>/dev/null | grep -qx "/ee_pose" \
    && ros2 topic list 2>/dev/null | grep -qx "/safety/status"; then
    graph_ready=true
    break
  fi
  if ! kill -0 "${stack_pid}" 2>/dev/null; then
    tail -n 100 "${OUTPUT_DIR}/full_system.log" >&2
    exit 4
  fi
  sleep 0.25
done
if [[ "${graph_ready}" != "true" ]]; then
  echo "Isaac control graph discovery timeout" >&2
  exit 4
fi

sleep 6
timeout 8s ros2 topic echo /sim/encoder_state --once \
  > "${OUTPUT_DIR}/initial_encoder.txt"
timeout 8s ros2 topic echo /ee_pose --once \
  > "${OUTPUT_DIR}/initial_ee_pose.txt"
timeout 8s ros2 topic echo /sim/object_pose --once \
  > "${OUTPUT_DIR}/initial_object_pose.txt"
timeout 8s ros2 topic echo /safety/status --once \
  > "${OUTPUT_DIR}/safety_pre.txt"
if ! grep -q "ok: true" "${OUTPUT_DIR}/safety_pre.txt"; then
  echo "Safety preflight is not OK" >&2
  exit 5
fi

warm_hb_pid=""
if [[ "${PREGRASP_WARMSTART}" == "true" ]]; then
  echo "[isaac-act] pregrasp warmstart enabled"
  export SERVO_POST_INIT_MODE=pose
  bash "${REPO_ROOT}/scripts/servo_post_init.sh" 2 15 \
    > "${OUTPUT_DIR}/servo_post_init.txt" 2>&1 || true
  timeout 8s ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool \
    "{data: false}" >/dev/null 2>&1 || true
  nice -n 19 ros2 topic pub -r 30 /teleop/heartbeat std_msgs/msg/Header \
    "{frame_id: 'isaac_act_warmstart_hb'}" >/dev/null 2>&1 &
  warm_hb_pid=$!
  set +e
  timeout 60s "${POLICY_PYTHON}" \
    "${REPO_ROOT}/scripts/isaac_pregrasp_warmstart.py" \
    --duration-s "${PREGRASP_DURATION_S}" \
    > "${OUTPUT_DIR}/pregrasp_warmstart.log" 2>&1
  warm_status=$?
  set -e
  if [[ "${warm_status}" -ne 0 ]]; then
    kill "${warm_hb_pid}" 2>/dev/null || true
    wait "${warm_hb_pid}" 2>/dev/null || true
    warm_hb_pid=""
    echo "Pregrasp warmstart failed; see ${OUTPUT_DIR}/pregrasp_warmstart.log" >&2
    exit 6
  fi
  timeout 8s ros2 topic echo /ee_pose --once \
    > "${OUTPUT_DIR}/post_warmstart_ee_pose.txt" || true
fi

if [[ "${RECORD_SCENE_VIDEO}" == "true" ]]; then
  /usr/bin/python3 "${REPO_ROOT}/scripts/isaac_scene_video_recorder.py" \
    --output "${OUTPUT_DIR}/scene.mp4" \
    --max-duration-s "${POLICY_RUNTIME_TIMEOUT_S}" \
    --max-frames 500 \
    > "${OUTPUT_DIR}/video_recorder.log" 2>&1 &
  video_pid=$!
fi

if [[ -n "${EPISODE_RESULTS_PATH}" ]]; then
  run_id="${EVALUATION_RUN_ID:-isaac_act_$(date +%Y%m%d_%H%M%S)}"
  export PYTHONPATH="${REPO_ROOT}/src/synth_data_gen:${REPO_ROOT}/install/synth_data_gen/lib/python3.12/site-packages:${PYTHONPATH:-}"
  /usr/bin/python3 "${REPO_ROOT}/scripts/isaac_continuous_gt_recorder.py" \
    --episode-results-path "${EPISODE_RESULTS_PATH}" \
    --evaluation-run-id "${run_id}" \
    --seed "${EVALUATION_SEED}" \
    --episode-index "${EVALUATION_EPISODE_INDEX}" \
    --model-id "${EVALUATION_MODEL_ID}" \
    --wait-for-report "${OUTPUT_DIR}/report.json" \
    --raw-episode-path "${OUTPUT_DIR}" \
    --video-path "${OUTPUT_DIR}/scene.mp4" \
    --runtime-log-path "${OUTPUT_DIR}/gt_runtime.log" \
    --event-log-path "${OUTPUT_DIR}/gt_events.jsonl" \
    --nfr-sample-path "${OUTPUT_DIR}/gt_nfr.json" \
    --max-duration-s "$((POLICY_RUNTIME_TIMEOUT_S + 60))" \
    > "${OUTPUT_DIR}/gt_recorder.log" 2>&1 &
  gt_pid=$!
fi

timeout 45s nvidia-smi \
  --query-gpu=timestamp,utilization.gpu,memory.used,power.draw \
  --format=csv,noheader,nounits --loop-ms=200 \
  > "${OUTPUT_DIR}/gpu_during_policy.csv" 2>&1 &
gpu_pid=$!

set +e
echo "[isaac-act] POLICY_RUNTIME_TIMEOUT_S=${POLICY_RUNTIME_TIMEOUT_S} MAX_ACTIONS=${MAX_ACTIONS} OBJECT_SEED=${OBJECT_SEED:-none} PREGRASP_WARMSTART=${PREGRASP_WARMSTART}"
timeout "${POLICY_RUNTIME_TIMEOUT_S}s" "${POLICY_PYTHON}" -m \
  isaac_sim_adapter.policy_inference_node --ros-args \
  -p checkpoint:="${CHECKPOINT}" \
  -p device:=cuda \
  -p dry_run:="${DRY_RUN}" \
  -p max_actions:="${MAX_ACTIONS}" \
  -p n_action_steps:="${N_ACTION_STEPS}" \
  -p inference_rate_hz:="$(python3 -c "print(float('${INFERENCE_RATE_HZ}'))")" \
  -p startup_timeout_s:="$(python3 -c "print(float('${POLICY_STARTUP_TIMEOUT_S}'))")" \
  -p post_action_hold_s:=3.0 \
  -p max_joint_excursion_rad:="$(python3 -c "print(float('${MAX_JOINT_EXCURSION_RAD}'))")" \
  -p max_ee_excursion_m:="$(python3 -c "print(float('${MAX_EE_EXCURSION_M}'))")" \
  -p max_translation_m:="$(python3 -c "print(float('${MAX_TRANSLATION_M}'))")" \
  -p max_rotation_rad:="$(python3 -c "print(float('${MAX_ROTATION_RAD}'))")" \
  -p workspace_min:="[${WORKSPACE_MIN}]" \
  -p workspace_max:="[${WORKSPACE_MAX}]" \
  -p output_path:="${OUTPUT_DIR}/report.json" \
  > "${OUTPUT_DIR}/policy.log" 2>&1
policy_status=$?
set -e

if [[ -n "${warm_hb_pid}" ]]; then
  kill "${warm_hb_pid}" 2>/dev/null || true
  wait "${warm_hb_pid}" 2>/dev/null || true
  warm_hb_pid=""
fi
kill "${gpu_pid}" 2>/dev/null || true
wait "${gpu_pid}" 2>/dev/null || true
gpu_pid=""

# After policy: wait briefly then SIGTERM GT so it covers the full episode
# (do not exit-on-report; mid-episode FAIL flushes create report.json early).
if [[ -n "${gt_pid}" ]]; then
  sleep 2
  kill -TERM "${gt_pid}" 2>/dev/null || true
  for _ in $(seq 1 40); do
    if ! kill -0 "${gt_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.25
  done
  kill -9 "${gt_pid}" 2>/dev/null || true
  wait "${gt_pid}" 2>/dev/null || true
  gt_pid=""
fi
if [[ -n "${video_pid}" ]]; then
  kill -TERM "${video_pid}" 2>/dev/null || true
  wait "${video_pid}" 2>/dev/null || true
  video_pid=""
fi

timeout 8s ros2 topic echo /sim/encoder_state --once \
  > "${OUTPUT_DIR}/final_encoder.txt" || true
timeout 8s ros2 topic echo /ee_pose --once \
  > "${OUTPUT_DIR}/final_ee_pose.txt" || true
timeout 8s ros2 topic echo /sim/object_pose --once \
  > "${OUTPUT_DIR}/final_object_pose.txt" || true
timeout 8s ros2 topic echo /safety/status --once \
  > "${OUTPUT_DIR}/safety_final.txt" || true

if [[ ! -f "${OUTPUT_DIR}/report.json" ]]; then
  tail -n 100 "${OUTPUT_DIR}/policy.log" >&2
  exit "${policy_status}"
fi
cat "${OUTPUT_DIR}/report.json"
if [[ "${REQUIRE_REPORT_PASS}" == "true" ]]; then
  "${POLICY_PYTHON}" - "${OUTPUT_DIR}/report.json" <<'PY'
import json
import pathlib
import sys

report = json.loads(pathlib.Path(sys.argv[1]).read_text())
if report["status"] != "PASS":
    raise SystemExit(1)
PY
fi

echo "ISAAC_ACT_SMOKE_EVIDENCE=${OUTPUT_DIR}"
