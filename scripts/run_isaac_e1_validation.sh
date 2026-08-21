#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_PYTHON="${ISAAC_PYTHON:-/home/ina/isaacsim/.venv/bin/python}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${REPO_ROOT}/install/setup.bash}"
OUTPUT_DIR="${1:-/tmp/isaac_e1_validation}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

export ROS_DOMAIN_ID RMW_IMPLEMENTATION
mkdir -p "${OUTPUT_DIR}"

backend_pid=""
adapter_pid=""
runner_pid=""

cleanup() {
  if [[ -n "${adapter_pid}" ]]; then
    kill "${adapter_pid}" 2>/dev/null || true
  fi
  if [[ -n "${runner_pid}" ]]; then
    kill "${runner_pid}" 2>/dev/null || true
  fi
  if [[ -n "${backend_pid}" ]]; then
    kill "${backend_pid}" 2>/dev/null || true
  fi
  wait "${adapter_pid}" 2>/dev/null || true
  wait "${runner_pid}" 2>/dev/null || true
  wait "${backend_pid}" 2>/dev/null || true
  pkill -9 -f "isaac_panda_backend.py" 2>/dev/null || true
  pkill -9 -f "isaac_sim_adapter" 2>/dev/null || true
  pkill -9 -f "isaac_e1_action_sequence" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "Isaac Python not executable: ${ISAAC_PYTHON}" >&2
  exit 2
fi
if [[ ! -f "${ROS_SETUP}" || ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "ROS or workspace setup missing; build the adapter first" >&2
  exit 2
fi

set +u
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"
set -u

uname -a > "${OUTPUT_DIR}/kernel.txt"
ps -eLo pid,tid,psr,cls,rtprio,pri,pcpu,comm --sort=-pcpu \
  > "${OUTPUT_DIR}/scheduler_snapshot.txt"
nvidia-smi --query-gpu=name,driver_version,memory.total \
  --format=csv,noheader > "${OUTPUT_DIR}/gpu.txt" 2>&1 || true

timeout 65s "${ISAAC_PYTHON}" \
  "${REPO_ROOT}/src/isaac_sim_adapter/scripts/isaac_panda_backend.py" \
  --duration-sec 25 --camera-rate 30 --command-timeout-s 0.1 \
  > "${OUTPUT_DIR}/backend.log" 2>&1 &
backend_pid=$!

ready=false
for _ in $(seq 1 120); do
  if grep -q "ISAAC_E1_READY=" "${OUTPUT_DIR}/backend.log" 2>/dev/null; then
    ready=true
    break
  fi
  if ! kill -0 "${backend_pid}" 2>/dev/null; then
    echo "Isaac backend exited before READY" >&2
    tail -n 80 "${OUTPUT_DIR}/backend.log" >&2
    exit 3
  fi
  sleep 0.5
done
if [[ "${ready}" != "true" ]]; then
  echo "Isaac backend READY timeout" >&2
  exit 3
fi

timeout 70s ros2 run isaac_sim_adapter isaac_sim_adapter --ros-args \
  -p startup_timeout_s:=20.0 \
  -p reset_timeout_s:=8.0 \
  -p command_timeout_s:=0.1 \
  -p state_timeout_s:=0.1 \
  -p command_forward_rate_hz:=250.0 \
  > "${OUTPUT_DIR}/adapter.log" 2>&1 &
adapter_pid=$!

graph_ready=false
for _ in $(seq 1 80); do
  if ros2 topic list 2>/dev/null | grep -qx "/sim/encoder_state" \
    && ros2 service list 2>/dev/null | grep -qx "/sim/reset_scene"; then
    graph_ready=true
    break
  fi
  if ! kill -0 "${adapter_pid}" 2>/dev/null; then
    echo "Isaac adapter exited before graph preflight" >&2
    tail -n 80 "${OUTPUT_DIR}/adapter.log" >&2
    exit 4
  fi
  sleep 0.25
done
if [[ "${graph_ready}" != "true" ]]; then
  echo "Isaac adapter graph discovery timed out" >&2
  exit 4
fi

# Prove raw Isaac -> canonical policy-input topics before any bridge action.
timeout 20s /usr/bin/python3 "${REPO_ROOT}/scripts/isaac_ros_topic_gate.py" \
  --timeout-s 12.0 \
  --output "${OUTPUT_DIR}/bridge_preflight.json"

timeout 20s ros2 topic echo /sim/encoder_state --once \
  > "${OUTPUT_DIR}/first_encoder_state.txt"
ros2 topic info -v /sim/encoder_state \
  > "${OUTPUT_DIR}/state_qos.txt"

timeout 55s ros2 run isaac_sim_adapter isaac_e1_action_sequence \
  --repeats 5 \
  --command-rate-hz 100 \
  --reset-timeout-s 8 \
  --graph-timeout-s 20 \
  --renderer-pressure-source scene_camera_30hz \
  --output "${OUTPUT_DIR}/action_sequence_report.json" \
  > "${OUTPUT_DIR}/runner.log" 2>&1 &
runner_pid=$!

publisher_ready=false
for _ in $(seq 1 40); do
  effort_topic_info="$(
    ros2 topic info /sim/joint_effort_cmd 2>/dev/null || true
  )"
  if grep -q "Publisher count: 1" <<< "${effort_topic_info}"; then
    publisher_ready=true
    break
  fi
  sleep 0.1
done
if [[ "${publisher_ready}" != "true" ]]; then
  echo "E1 runner publisher did not appear" >&2
  exit 4
fi
ros2 topic info -v /sim/joint_effort_cmd \
  > "${OUTPUT_DIR}/effort_qos.txt"
wait "${runner_pid}"
runner_pid=""

ros2 topic echo /sim/backend_status --once \
  > "${OUTPUT_DIR}/final_backend_status.txt" || true

# Prove the remote-failure boundary: once the ROS adapter disappears, the
# simulator-local watchdog must stop replaying the last forwarded effort.
kill "${adapter_pid}" 2>/dev/null || true
wait "${adapter_pid}" 2>/dev/null || true
adapter_pid=""
sleep 0.4
wait "${backend_pid}"
backend_pid=""

grep "ISAAC_E1_" "${OUTPUT_DIR}/backend.log" \
  > "${OUTPUT_DIR}/backend_e1_events.log" || true

echo "ISAAC_E1_EVIDENCE=${OUTPUT_DIR}"
