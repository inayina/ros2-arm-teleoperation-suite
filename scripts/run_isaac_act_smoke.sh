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
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-83}"
readonly RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

export ROS_DOMAIN_ID RMW_IMPLEMENTATION
mkdir -p "${OUTPUT_DIR}"

backend_pid=""
stack_pid=""
gpu_pid=""

cleanup() {
  [[ -z "${gpu_pid}" ]] || kill "${gpu_pid}" 2>/dev/null || true
  [[ -z "${stack_pid}" ]] || kill "${stack_pid}" 2>/dev/null || true
  [[ -z "${backend_pid}" ]] || kill "${backend_pid}" 2>/dev/null || true
  [[ -z "${gpu_pid}" ]] || wait "${gpu_pid}" 2>/dev/null || true
  [[ -z "${stack_pid}" ]] || wait "${stack_pid}" 2>/dev/null || true
  [[ -z "${backend_pid}" ]] || wait "${backend_pid}" 2>/dev/null || true
  "${REPO_ROOT}/scripts/stop_stack.sh" > "${OUTPUT_DIR}/cleanup.log" 2>&1 || true
  pkill -9 -f "isaac_panda_backend.py" 2>/dev/null || true
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

timeout 110s "${ISAAC_PYTHON}" \
  "${REPO_ROOT}/src/isaac_sim_adapter/scripts/isaac_panda_backend.py" \
  --duration-sec 100 --camera-rate 10 --command-timeout-s 0.1 \
  --arm-command-mode "${ARM_COMMAND_MODE}" \
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
timeout 8s ros2 topic echo /safety/status --once \
  > "${OUTPUT_DIR}/safety_pre.txt"
if ! grep -q "ok: true" "${OUTPUT_DIR}/safety_pre.txt"; then
  echo "Safety preflight is not OK" >&2
  exit 5
fi

timeout 45s nvidia-smi \
  --query-gpu=timestamp,utilization.gpu,memory.used,power.draw \
  --format=csv,noheader,nounits --loop-ms=200 \
  > "${OUTPUT_DIR}/gpu_during_policy.csv" 2>&1 &
gpu_pid=$!

set +e
timeout 50s "${POLICY_PYTHON}" -m \
  isaac_sim_adapter.policy_inference_node --ros-args \
  -p checkpoint:="${CHECKPOINT}" \
  -p device:=cuda \
  -p dry_run:="${DRY_RUN}" \
  -p max_actions:="${MAX_ACTIONS}" \
  -p inference_rate_hz:=1.0 \
  -p startup_timeout_s:=30.0 \
  -p post_action_hold_s:=3.0 \
  -p max_joint_excursion_rad:=0.25 \
  -p max_ee_excursion_m:=0.03 \
  -p output_path:="${OUTPUT_DIR}/report.json" \
  > "${OUTPUT_DIR}/policy.log" 2>&1
policy_status=$?
set -e

kill "${gpu_pid}" 2>/dev/null || true
wait "${gpu_pid}" 2>/dev/null || true
gpu_pid=""
timeout 8s ros2 topic echo /sim/encoder_state --once \
  > "${OUTPUT_DIR}/final_encoder.txt" || true
timeout 8s ros2 topic echo /ee_pose --once \
  > "${OUTPUT_DIR}/final_ee_pose.txt" || true
timeout 8s ros2 topic echo /safety/status --once \
  > "${OUTPUT_DIR}/safety_final.txt" || true

if [[ ! -f "${OUTPUT_DIR}/report.json" ]]; then
  tail -n 100 "${OUTPUT_DIR}/policy.log" >&2
  exit "${policy_status}"
fi
cat "${OUTPUT_DIR}/report.json"
"${POLICY_PYTHON}" - "${OUTPUT_DIR}/report.json" <<'PY'
import json
import pathlib
import sys

report = json.loads(pathlib.Path(sys.argv[1]).read_text())
if report["status"] != "PASS":
    raise SystemExit(1)
PY

echo "ISAAC_ACT_SMOKE_EVIDENCE=${OUTPUT_DIR}"
