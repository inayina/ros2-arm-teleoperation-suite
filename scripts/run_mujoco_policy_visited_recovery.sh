#!/usr/bin/env bash
# Bounded local-only policy-prefix replay -> scripted recovery capture.
# Never loads a model, starts a remote GPU, enters Isaac, or trains.
set -euo pipefail

readonly ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly MIDSTREAM="${MIDSTREAM:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab}"
readonly CASE_ID="${CASE_ID:-P0}"
readonly PREFIX_COUNT="${PREFIX_COUNT:-25}"
readonly REPEAT_ID="${REPEAT_ID:-0}"
readonly STAMP="${DATE_TAG:-$(date -u +%Y%m%dT%H%M%SZ)}"
readonly DATASET_ROOT="${RECOVERY_DATASET_ROOT:-${MIDSTREAM}/data/policy_visited_recovery/reach_v1_raw}"
readonly OUT="${1:-${MIDSTREAM}/evidence/smolvla_reach_recovery_${CASE_ID}_k${PREFIX_COUNT}_r${REPEAT_ID}_${STAMP}}"
readonly BASE_DOMAIN_ID="${BASE_DOMAIN_ID:-176}"
readonly TASK_TEXT='pick up the red box and place it in the left bin'
readonly RECOVERY_APPROACH_S="${RECOVERY_APPROACH_S:-5.0}"
export RECOVERY_APPROACH_S
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${BASE_DOMAIN_ID}}"

case "${CASE_ID}" in
  P0|P1|P2|P3) ;;
  *) echo 'CASE_ID must be P0, P1, P2, or P3' >&2; exit 2 ;;
esac
if [[ ! "${PREFIX_COUNT}" =~ ^[0-9]+$ ]] \
  || (( PREFIX_COUNT < 1 || PREFIX_COUNT > 100 )); then
  echo 'PREFIX_COUNT must be an integer in [1,100]' >&2
  exit 2
fi

readonly SOURCE_EVIDENCE="${RECOVERY_SOURCE_EVIDENCE:-${MIDSTREAM}/evidence/smolvla_dualcam_timergroup_${CASE_ID}_20260821}"
readonly ACTIONS="${SOURCE_EVIDENCE}/trial/actions.jsonl"
readonly RANDOMIZATION="${RECOVERY_RANDOMIZATION_PATH:-${SOURCE_EVIDENCE}/trial/randomization.yaml}"
[[ -s "${ACTIONS}" && -s "${RANDOMIZATION}" ]] || {
  echo "missing frozen source trace for ${CASE_ID}" >&2
  exit 2
}

mkdir -p "${OUT}" "${DATASET_ROOT}"
cp -f "${RANDOMIZATION}" "${OUT}/randomization.yaml"
: > "${OUT}/episode_results.jsonl"

nuke() {
  "${ROOT}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  pkill -9 -f '[m]ujoco_policy_visited_recovery.py' 2>/dev/null || true
  pkill -9 -f '[i]saac_continuous_gt_recorder.py' 2>/dev/null || true
  pkill -9 -f '[t]eleop_bringup' 2>/dev/null || true
  pkill -9 -f '[m]ujoco_sim' 2>/dev/null || true
  pkill -9 -f '[l]erobot_recorder' 2>/dev/null || true
  pkill -9 -f '[s]ervo_node' 2>/dev/null || true
  pkill -9 -f '[r]os2_control' 2>/dev/null || true
}
trap nuke EXIT
nuke

set +u
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
set -u
export PYTHONPATH="${ROOT}/src/synth_data_gen:${ROOT}/src/isaac_sim_adapter:${ROOT}/install/synth_data_gen/lib/python3.12/site-packages:${ROOT}/install/isaac_sim_adapter/lib/python3.12/site-packages:${PYTHONPATH:-}"

setsid --fork --wait timeout 240s ros2 launch teleop_bringup full_system.launch.py \
  sim_backend:=mujoco \
  record:=true \
  start_teleop:=false \
  controller:=impedance \
  enable_grasp_monitor:=false \
  grasp_assist_enabled:=false \
  randomize:=true \
  randomization_path:="${OUT}/randomization.yaml" \
  camera_rate:=10.0 \
  camera_width:=320 \
  camera_height:=240 \
  sync_slop:=0.12 \
  sync_queue_size:=120 \
  scene_use_mujoco_renderer:=true \
  enable_wrist_camera:=true \
  watchdog_timeout:=30.0 \
  headless:=true \
  capture_mode:=portfolio \
  output_dir:="${DATASET_ROOT}" \
  task:="${TASK_TEXT}" \
  auto_record_seconds:=0.0 \
  auto_record_delay_s:=0.0 \
  > "${OUT}/full_system.log" 2>&1 &
stack_pid=$!

ready=false
for _ in $(seq 1 240); do
  if grep -q 'camera_bridge up .*color=/camera/color/image_raw' "${OUT}/full_system.log" \
    && grep -q 'camera_bridge up .*color=/camera/wrist/color/image_raw' "${OUT}/full_system.log" \
    && grep -q 'safety_monitor up' "${OUT}/full_system.log"; then
    ready=true
    break
  fi
  kill -0 "${stack_pid}" 2>/dev/null || break
  sleep .25
done
[[ "${ready}" == true ]] || { echo 'runtime launch readiness failed' > "${OUT}/runtime_blocker.txt"; exit 3; }
sleep 3

timeout 20s /usr/bin/python3 "${ROOT}/scripts/mujoco_dualcam_runtime_preflight.py" \
  --timeout-s 15 --output "${OUT}/runtime_preflight.json" \
  > "${OUT}/runtime_preflight.log" 2>&1 || {
    echo 'dual-camera freshness preflight failed' > "${OUT}/runtime_blocker.txt"
    exit 3
  }

timeout 8s ros2 service call /sim/reset_scene std_srvs/srv/Trigger '{}' \
  > "${OUT}/reset_scene.txt" 2>&1 || true

controller_ready=false
for _ in $(seq 1 60); do
  timeout 8s ros2 control list_controllers --spin-time 2.0 > "${OUT}/controllers.txt" 2>&1 || true
  if grep -q 'cartesian_impedance_controller.*active' "${OUT}/controllers.txt" \
    || (grep -q 'CartesianImpedanceController activated' "${OUT}/full_system.log" \
      && grep -q 'Successfully switched controllers' "${OUT}/full_system.log"); then
    controller_ready=true
    break
  fi
  sleep .5
done
[[ "${controller_ready}" == true ]] || {
  echo 'cartesian_impedance_controller not active' > "${OUT}/runtime_blocker.txt"
  exit 3
}

export SERVO_POST_INIT_MODE=pose
bash "${ROOT}/scripts/servo_post_init.sh" 4 20 > "${OUT}/servo_post_init.txt" 2>&1 || true
for _ in 1 2 3 4; do
  timeout 8s ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool \
    '{data: false}' >> "${OUT}/servo_unpause.txt" 2>&1 || true
  sleep 1
done

/usr/bin/python3 "${ROOT}/scripts/isaac_continuous_gt_recorder.py" \
  --episode-results-path "${OUT}/episode_results.jsonl" \
  --evaluation-run-id "smolvla_reach_recovery_${STAMP}" \
  --seed 42 \
  --episode-index 0 \
  --model-id scripted_policy_visited_recovery_expert \
  --suite-id smolvla_reach_recovery_v1 \
  --backend mujoco \
  --validation-mode lift \
  --lift-success-delta .03 \
  --gripper-close-max .70 \
  --wait-for-report "${OUT}/execution_report.json" \
  --exit-on-report \
  --raw-episode-path "${DATASET_ROOT}" \
  --runtime-log-path "${OUT}/gt_runtime.log" \
  --event-log-path "${OUT}/gt_events.jsonl" \
  --nfr-sample-path "${OUT}/gt_nfr.json" \
  --max-duration-s 150 \
  > "${OUT}/gt_recorder.log" 2>&1 &
gt_pid=$!

set +e
timeout 130s /usr/bin/python3 "${ROOT}/scripts/mujoco_policy_visited_recovery.py" \
  --actions "${ACTIONS}" \
  --prefix-count "${PREFIX_COUNT}" \
  --case-id "${CASE_ID}" \
  --approach-s "${RECOVERY_APPROACH_S}" \
  --output "${OUT}/execution_report.json" \
  > "${OUT}/coordinator.log" 2>&1
coordinator_exit=$?
set -e
echo "${coordinator_exit}" > "${OUT}/coordinator_exit_code.txt"

for _ in $(seq 1 40); do
  kill -0 "${gt_pid}" 2>/dev/null || break
  sleep .25
done
kill -TERM "${gt_pid}" 2>/dev/null || true
wait "${gt_pid}" 2>/dev/null || true

[[ -s "${OUT}/execution_report.json" ]] || exit 4
status="$(jq -r '.status' "${OUT}/execution_report.json")"
dataset_path="$(jq -r '.recorder.dataset_path // empty' "${OUT}/execution_report.json")"
if [[ "${status}" != CAPTURE_ACCEPTED_PENDING_MIDSTREAM_QA || -z "${dataset_path}" ]]; then
  echo "RECOVERY_STATUS=${status}"
  exit 5
fi

episode_name="$(basename "${dataset_path}" .parquet)"
episode_meta="${DATASET_ROOT}/${episode_name}/meta.json"
[[ -s "${episode_meta}" ]] || {
  echo "committed episode metadata missing: ${episode_meta}" >&2
  exit 6
}

source_completed_actions="$(python3 - "${ACTIONS}" <<'PY'
import json
import sys
count = 0
for line in open(sys.argv[1], encoding='utf-8'):
    row = json.loads(line)
    if row.get('decision') != 'EXECUTED':
        break
    count += 1
print(count)
PY
)"
python3 - "${OUT}" "${SOURCE_EVIDENCE}" "${CASE_ID}" "${PREFIX_COUNT}" "${episode_meta}" "${source_completed_actions}" <<'PY'
import json, os, sys
from pathlib import Path
out, evidence, case_id, count, episode_meta, completed_actions = sys.argv[1:]
payload = {
    'contract_version': 'policy_visited_recovery_capture_v1',
    'simulation_backend': 'mujoco',
    'claims_task_success': False,
    'expert_source': 'scripted_oracle_privileged_gt',
    'invariant': {
        'task': 'pick up the red box and place it in the left bin',
        'cameras': ['scene', 'wrist'],
        'state_contract': 'observation.state[15]',
        'action_semantics': 'ee_pose_gripper_cmd_v1',
        'grasp_assist_enabled': False,
        'object_pose_is_policy_input': False,
    },
    'recording_contract': {
        'expected_fps': 10.0,
        'sync_slop_s': 0.12,
        'sync_queue_size': 120,
    },
    'recovery_expert_contract': {
        'approach_s': float(os.environ['RECOVERY_APPROACH_S']),
        'approach_xy_tolerance_m': 0.02,
    },
    'policy_rollout': {
        'evidence_dir': str(Path(evidence).resolve()),
        'case': case_id,
        'failure_type': 'NO_MEANINGFUL_APPROACH',
        'failure_onset_action_index': int(count) - 1,
        'completed_actions': int(completed_actions),
    },
    'episodes': [{
        'episode_meta': str(Path(episode_meta).resolve()),
        'phase_buckets': ['ALIGN', 'DESCEND', 'CLOSE', 'LIFT'],
    }],
}
Path(out, 'capture_manifest.json').write_text(
    json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
PY

python3 "${MIDSTREAM}/training/scripts/validate_policy_visited_recovery_capture.py" \
  --capture-manifest "${OUT}/capture_manifest.json" \
  --output-report "${OUT}/capture_validation_report.json"

jq -n \
  --arg case_id "${CASE_ID}" \
  --argjson prefix_count "${PREFIX_COUNT}" \
  --arg status "${status}" \
  --arg episode_meta "${episode_meta}" \
  --arg execution_report "${OUT}/execution_report.json" \
  --arg validation_report "${OUT}/capture_validation_report.json" \
  '{contract_version:"smolvla_reach_recovery_trial_v1",simulation_backend:"mujoco",case_id:$case_id,prefix_count:$prefix_count,status:$status,episode_meta:$episode_meta,execution_report:$execution_report,validation_report:$validation_report,remote_gpu_used:false,trained:false,claims_task_success:false}' \
  > "${OUT}/trial_summary.json"

echo "RECOVERY_OUT=${OUT}"
echo "RECOVERY_STATUS=${status}"
echo "EPISODE_META=${episode_meta}"
