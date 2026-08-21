#!/usr/bin/env bash
# Independent scene+wrist SmolVLA MuJoCo smoke. Never starts Isaac or training.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MID="${MIDSTREAM:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab}"
STAMP="${DATE_TAG:-$(date -u +%Y%m%dT%H%M%SZ)}"
OUT="${1:-${MID}/evidence/smolvla_dualcam_mujoco_smoke_${STAMP}}"
POLICY_PYTHON="${POLICY_PYTHON:-/home/ina/miniforge3/envs/lerobot/bin/python}"
LORA_DIR="${LORA_DIR:-${MID}/runs/smolvla_wrist_ablation_v1_B}"
BASE_DIR="${BASE_DIR:-${MID}/checkpoints/smolvla_base_gate_s1}"
VLM_DIR="${VLM_DIR:-${MID}/checkpoints/SmolVLM2-500M-Video-Instruct}"
ENDPOINT="${POLICY_RUNTIME_ENDPOINT:-http://127.0.0.1:18081}"
POLICY_TIMEOUT_S="${POLICY_TIMEOUT_S:-180}"
MAX_ACTIONS="${MAX_ACTIONS:-100}"
RUNTIME_MAX_OBSERVATION_AGE_S="${RUNTIME_MAX_OBSERVATION_AGE_S:-2.5}"
CONTROL_STATE_TIMEOUT_S="${CONTROL_STATE_TIMEOUT_S:-2.5}"
RECORD_VIDEO="${RECORD_VIDEO:-true}"
CAMERA_RATE_HZ="${CAMERA_RATE_HZ:-10.0}"
NOMINAL_SEED="${NOMINAL_SEED:-42}"
OBJECT_X="${OBJECT_X:-}"
OBJECT_Y="${OBJECT_Y:-}"
OBJECT_YAW_DEG="${OBJECT_YAW_DEG:-}"
RANDOMIZATION_PATH="${RANDOMIZATION_PATH:-}"
BASE_DOMAIN="${BASE_DOMAIN_ID:-164}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"

[[ -x "${POLICY_PYTHON}" && -d "${LORA_DIR}" && -d "${BASE_DIR}" && -d "${VLM_DIR}" ]] || { echo 'required policy artifact missing' >&2; exit 2; }
mkdir -p "${OUT}/policy" "${OUT}/trial" "${OUT}/videos" "${OUT}/randomization"
: > "${OUT}/episode_results.jsonl"

nuke() {
  "${ROOT}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  pkill -9 -f '[s]molvla_policy_inference_node' 2>/dev/null || true
  pkill -9 -f '[m]ujoco_dualcam_runtime_preflight' 2>/dev/null || true
  pkill -9 -f '[i]saac_continuous_gt_recorder' 2>/dev/null || true
  pkill -9 -f '[i]saac_scene_video_recorder' 2>/dev/null || true
  pkill -9 -f '[t]eleop_bringup' 2>/dev/null || true
  pkill -9 -f '[m]ujoco_sim' 2>/dev/null || true
  pkill -9 -f '[s]ervo_node' 2>/dev/null || true
  pkill -9 -f '[r]os2_control' 2>/dev/null || true
}

write_randomization() {
  local path="$1" seed="$2"
  if [[ -n "${RANDOMIZATION_PATH}" ]]; then
    [[ -s "${RANDOMIZATION_PATH}" ]] || { echo "RANDOMIZATION_PATH missing: ${RANDOMIZATION_PATH}" >&2; return 2; }
    cp -f "${RANDOMIZATION_PATH}" "${path}"
    echo "randomization_source=${RANDOMIZATION_PATH}"
    return 0
  fi
  PYTHONPATH="${ROOT}/src/isaac_sim_adapter:${PYTHONPATH:-}" "${POLICY_PYTHON}" - "${path}" "${seed}" "${OBJECT_X}" "${OBJECT_Y}" "${OBJECT_YAW_DEG}" <<'PY'
import math, sys
from pathlib import Path
from isaac_sim_adapter.object_pose_seed import sample_red_box_pose
p, seed = Path(sys.argv[1]), int(sys.argv[2])
x_override, y_override, yaw_override = sys.argv[3:6]
x, y, _z, yaw = sample_red_box_pose(seed)
if bool(x_override) != bool(y_override):
    raise SystemExit('OBJECT_X and OBJECT_Y must be set together')
if x_override:
    x, y = float(x_override), float(y_override)
if yaw_override:
    yaw = math.radians(float(yaw_override))
p.write_text(f'''domain_randomization:
  enabled: true
  seed: {seed}
  camera:
    scene_camera:
      pos_noise: [0.0, 0.0]
      rot_noise: [0.0, 0.0]
  lighting:
    key:
      diffuse_noise: [0.0, 0.0]
  object:
    box_initial_z: 0.025
    mass_range: [0.04, 0.04]
    friction_range: [2.2, 2.2]
    initial_pos_by_object:
      object_red_box: [{x:.8f}, {y:.8f}]
    yaw_range_deg_by_object:
      object_red_box: [{math.degrees(yaw):.8f}, {math.degrees(yaw):.8f}]
''')
print(f'object_red_box=[{x:.8f},{y:.8f},0.02500000], yaw_deg={math.degrees(yaw):.8f}')
PY
}

write_metadata() {
  python3 - "${OUT}" "${STAMP}" "${LORA_DIR}" "${BASE_DIR}" "${VLM_DIR}" "${NOMINAL_SEED}" <<'PY'
import hashlib,json,subprocess,sys
from datetime import datetime,timezone
from pathlib import Path
o,stamp,lora,base,vlm,seed=map(str,sys.argv[1:]); o=Path(o); cfg=json.loads((Path(lora)/'train_config.json').read_text())
def git(p):
 try:return subprocess.check_output(['git','-C',p,'rev-parse','HEAD'],text=True).strip()
 except Exception:return None
ck={'path':lora,'adapter_sha256':'943633adfb0c8201e46a088507e4e9191843754617093024e12cdeb8c6be950a','training_run_id':'train_20260818_retry2','final_step':5460,'lora_complete':True,'policy_preprocessor':(Path(lora)/'policy_preprocessor.json').is_file(),'policy_postprocessor':(Path(lora)/'policy_postprocessor.json').is_file()}
m={'run_id':'smolvla_dualcam_mujoco_smoke_'+stamp,'created_at':datetime.now(timezone.utc).isoformat(),'backend':'mujoco','camera_variant':'scene_wrist','validation_mode':'reach_grasp_lift','simulation_only':True,'claims_real_robot':False,'claims_sim2real':False,'claims_task_success':False,'execution_status':'running','checkpoint':ck,'dataset_release':{'id':'smolvla_wrist_ablation_v1_panda_abs_eef_scene_wrist_phaseaware50','content_sha256':'258cfd7cb4a90c5caed15e717a83e6be435a716ac0a4f2d78acf084d03af5221'},'policy_input':{'state':'observation.state[15]','state_dim':15,'images':['observation.images.scene','observation.images.wrist'],'object_pose_is_policy_input':False},'action_contract':{'semantics':'absolute_eef_gripper_v0','action_dim':8,'chunk_size':10,'execute_k':5},'stage_a_seed':int(seed),'in_distribution_sampler':'sample_red_box_pose','artifacts':{'base_dir':base,'vlm_dir':vlm,'train_config_sha256':hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest()},'repositories':{'upstream':git('/home/ina/dev/ros2-arm-teleoperation-suite'),'midstream':git('/home/ina/robot-sim-lab/robot-arm-episode-data-lab')}}
(o/'run_manifest.json').write_text(json.dumps(m,indent=2,sort_keys=True)+'\n')
(o/'policy'/'checkpoint_metadata.json').write_text(json.dumps({'checkpoint':ck,'train_config':cfg},indent=2,sort_keys=True)+'\n')
PY
}

run_trial() {
  local seed="$1" index="$2" label="$3" dir="${OUT}/trial"
  [[ "${label}" == stage_a ]] || { dir="${OUT}/confirm_${index}"; mkdir -p "${dir}"; }
  local stack='' gt='' scene='' wrist='' heartbeat=''
  cleanup() { for p in "${scene}" "${wrist}" "${gt}" "${heartbeat}" "${stack}"; do [[ -z "$p" ]] || kill -TERM "$p" 2>/dev/null || true; done; nuke; }
  export ROS_DOMAIN_ID=$((BASE_DOMAIN + index))
  local randomization="${OUT}/randomization/${label}_${seed}.yaml"
  write_randomization "${randomization}" "${seed}" > "${dir}/placement.txt"; cp -f "${randomization}" "${dir}/randomization.yaml"
  set +u; source /opt/ros/jazzy/setup.bash; source "${ROOT}/install/setup.bash"; set -u
  # Keep this bounded launch in its own session.  Without setsid, a shell
  # supervising the smoke may close its background job before ROS has finished
  # bringing up the control graph; timeout remains the hard lifecycle limit.
  setsid --fork --wait timeout 240s ros2 launch teleop_bringup full_system.launch.py sim_backend:=mujoco record:=false start_teleop:=false controller:=impedance enable_grasp_monitor:=false grasp_assist_enabled:=false randomize:=true randomization_path:="${randomization}" camera_rate:="${CAMERA_RATE_HZ}" camera_width:=320 camera_height:=240 scene_use_mujoco_renderer:=true enable_wrist_camera:=true watchdog_timeout:=30.0 headless:=true > "${dir}/full_system.log" 2>&1 & stack=$!
  local ready=false
  for _ in $(seq 1 240); do
    # Do not trust ros2 topic list alone here: the local ROS daemon can retain
    # a just-stopped domain graph.  These current-launch log lines prove the
    # two live renderers and safety node have actually initialized.
    if grep -q 'camera_bridge up .*color=/camera/color/image_raw' "${dir}/full_system.log" \
      && grep -q 'camera_bridge up .*color=/camera/wrist/color/image_raw' "${dir}/full_system.log" \
      && grep -q 'safety_monitor up' "${dir}/full_system.log"; then ready=true; break; fi
    kill -0 "${stack}" 2>/dev/null || break; sleep .25
  done
  if [[ "${ready}" != true ]]; then echo 'RUNTIME_BLOCKER: required MuJoCo topic missing' > "${dir}/runtime_blocker.txt"; cleanup; return 0; fi
  sleep 3
  timeout 20s /usr/bin/python3 "${ROOT}/scripts/mujoco_dualcam_runtime_preflight.py" --timeout-s 15 --output "${dir}/runtime_preflight.json" > "${dir}/runtime_preflight.log" 2>&1 || { echo 'RUNTIME_BLOCKER: dual-camera freshness preflight failed' > "${dir}/runtime_blocker.txt"; cleanup; return 0; }
  timeout 8s ros2 service call /sim/reset_scene std_srvs/srv/Trigger '{}' > "${dir}/reset_scene.txt" 2>&1 || true
  local safety_ready=false
  : > "${dir}/safety_pre.txt"
  for _ in $(seq 1 10); do
    timeout 2s ros2 topic echo /safety/status --once >> "${dir}/safety_pre.txt" 2>&1 || true
    if grep -q 'ok: true' "${dir}/safety_pre.txt"; then safety_ready=true; break; fi
    sleep .25
  done
  if [[ "${safety_ready}" != true ]]; then echo 'RUNTIME_BLOCKER: safety preflight not OK' > "${dir}/runtime_blocker.txt"; cleanup; return 0; fi
  local controller_ready=false
  for _ in $(seq 1 60); do
    timeout 3s ros2 control list_controllers > "${dir}/controllers.txt" 2>&1 || true
    if grep -q 'cartesian_impedance_controller.*active' "${dir}/controllers.txt"; then
      controller_ready=true
      break
    fi
    sleep .5
  done
  if [[ "${controller_ready}" != true ]]; then echo 'RUNTIME_BLOCKER: cartesian_impedance_controller not active' > "${dir}/runtime_blocker.txt"; cleanup; return 0; fi
  export SERVO_POST_INIT_MODE=pose; bash "${ROOT}/scripts/servo_post_init.sh" 4 20 > "${dir}/servo_post_init.txt" 2>&1 || true
  for _ in 1 2 3 4; do
    timeout 8s ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool '{data: false}' >> "${dir}/servo_unpause.txt" 2>&1 || true
    sleep 1
  done
  ros2 topic pub -r 30 /teleop/heartbeat std_msgs/msg/Header "{frame_id: 'mujoco_dualcam_smoke'}" >/dev/null 2>&1 & heartbeat=$!
  if [[ "${RECORD_VIDEO}" == true ]]; then
    /usr/bin/python3 "${ROOT}/scripts/isaac_scene_video_recorder.py" --topic /camera/color/image_raw --output "${dir}/scene.mp4" --max-duration-s "${POLICY_TIMEOUT_S}" --max-frames 1200 > "${dir}/scene_video.log" 2>&1 & scene=$!
    /usr/bin/python3 "${ROOT}/scripts/isaac_scene_video_recorder.py" --topic /camera/wrist/color/image_raw --output "${dir}/wrist.mp4" --max-duration-s "${POLICY_TIMEOUT_S}" --max-frames 1200 > "${dir}/wrist_video.log" 2>&1 & wrist=$!
  fi
  export PYTHONPATH="${ROOT}/src/synth_data_gen:${ROOT}/src/isaac_sim_adapter:${ROOT}/install/synth_data_gen/lib/python3.12/site-packages:${ROOT}/install/isaac_sim_adapter/lib/python3.12/site-packages:${PYTHONPATH:-}"
  /usr/bin/python3 "${ROOT}/scripts/isaac_continuous_gt_recorder.py" --episode-results-path "${OUT}/episode_results.jsonl" --evaluation-run-id "smolvla_dualcam_mujoco_smoke_${STAMP}" --seed "${seed}" --episode-index "${index}" --model-id smolvla_wrist_ablation_v1_B --suite-id smolvla_dualcam_mujoco_smoke --backend mujoco --validation-mode lift --lift-success-delta .03 --gripper-close-max .70 --wait-for-report "${dir}/report.json" --exit-on-report --raw-episode-path "${dir}" --video-path "${dir}/scene.mp4" --runtime-log-path "${dir}/gt_runtime.log" --event-log-path "${dir}/gt_events.jsonl" --nfr-sample-path "${dir}/gt_nfr.json" --max-duration-s "$((POLICY_TIMEOUT_S+30))" > "${dir}/gt_recorder.log" 2>&1 & gt=$!
  mkdir -p "${dir}/telemetry/camera"
  set +e
  setsid --fork --wait timeout "${POLICY_TIMEOUT_S}s" "${POLICY_PYTHON}" -m isaac_sim_adapter.smolvla_policy_inference_node --ros-args -p lora_dir:="${LORA_DIR}" -p base_dir:="${BASE_DIR}" -p vlm_dir:="${VLM_DIR}" -p device:=cuda -p dry_run:=false -p simulation_backend:=mujoco -p policy_runtime_backend:=remote -p policy_runtime_remote_endpoint:="${ENDPOINT}" -p policy_runtime_remote_timeout_s:=20.0 -p execution_adapter_mode:=authoritative -p policy_runtime_async_chunk_enabled:=true -p policy_runtime_warmup_enabled:=true -p observation_timeout_s:=0.5 -p policy_runtime_control_state_timeout_s:="${CONTROL_STATE_TIMEOUT_S}" -p policy_runtime_max_observation_age_s:="${RUNTIME_MAX_OBSERVATION_AGE_S}" -p policy_runtime_command_ttl_s:=0.1 -p max_actions:="${MAX_ACTIONS}" -p n_action_steps:=5 -p inference_rate_hz:=10.0 -p startup_timeout_s:=60.0 -p post_action_hold_s:=2.0 -p task:='pick up the red box and place it in the left bin' -p output_path:="${dir}/report.json" -p telemetry_dir:="${dir}/telemetry" -p camera_dump_stride:=1 -p policy_runtime_trace_run_id:="smolvla_dualcam_mujoco_smoke_${STAMP}" -p policy_runtime_episode_id:="${label}_seed_${seed}" > "${dir}/policy.log" 2>&1 &
  local policy_pid=$!
  wait "${policy_pid}"; echo $? > "${dir}/policy_exit_code.txt"; set -e
  sleep 2; for p in "${scene}" "${wrist}" "${gt}"; do kill -TERM "$p" 2>/dev/null || true; done
  cp -f "${dir}/telemetry/observations.jsonl" "${dir}/observations.jsonl" 2>/dev/null || true
  python3 - "${dir}" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]); r=p/'report.json'
if r.is_file():
 with (p/'actions.jsonl').open('w') as h:
  for row in json.loads(r.read_text()).get('actions',[]): h.write(json.dumps(row,sort_keys=True)+'\n')
PY
  if [[ "${RECORD_VIDEO}" == true ]] && command -v ffmpeg >/dev/null 2>&1 && [[ -f "${dir}/scene.mp4" && -f "${dir}/wrist.mp4" ]]; then ffmpeg -y -i "${dir}/scene.mp4" -i "${dir}/wrist.mp4" -filter_complex hstack=inputs=2 -an "${dir}/side_by_side.mp4" > "${dir}/side_by_side.log" 2>&1 || true; fi
  cleanup
}

stage_outcome() {
  python3 - "${OUT}" <<'PY'
import json,sys
from pathlib import Path
o=Path(sys.argv[1]); rows=[]
for line in (o/'episode_results.jsonl').read_text().splitlines() if (o/'episode_results.jsonl').is_file() else []:
 try: rows.append(json.loads(line))
 except Exception: pass
def yes(r,k):
 v=(r.get('subgoals') or {}).get(k); return bool(v.get('success') if isinstance(v,dict) else v)
r=rows[0] if rows else {}; b=(o/'trial/runtime_blocker.txt').is_file(); report_path=o/'trial/report.json'; report=report_path.is_file()
report_data=json.loads(report_path.read_text()) if report else {}
ee_excursion=float(report_data.get('max_observed_ee_excursion_m') or 0.0)
control_moved=ee_excursion >= 0.001
out='E_RUNTIME_INTERFACE_FAILURE' if b or not report or not control_moved else ('A_TRUE_LIFT' if yes(r,'lift') else ('C_GRASP_NO_LIFT' if yes(r,'grasp') else ('B_REACH_ALIGN_NO_GRASP' if yes(r,'reach') else 'D_NO_MEANINGFUL_APPROACH')))
s={'backend':'mujoco','camera_variant':'scene_wrist','validation_mode':'reach_grasp_lift','simulation_only':True,'claims_real_robot':False,'claims_sim2real':False,'stage_a_outcome':out,'stage_a_gt':r,'control_execution':{'max_observed_ee_excursion_m':ee_excursion,'moved_at_least_1mm':control_moved},'confirmation':{'executed':max(0,len(rows)-1),'reach':sum(yes(x,'reach') for x in rows[1:]),'grasp':sum(yes(x,'grasp') for x in rows[1:]),'lift':sum(yes(x,'lift') for x in rows[1:])}}
(o/'summary.json').write_text(json.dumps(s,indent=2,sort_keys=True)+'\n'); m=json.loads((o/'run_manifest.json').read_text()); m.update({'execution_status':'completed','stage_a_outcome':out}); (o/'run_manifest.json').write_text(json.dumps(m,indent=2,sort_keys=True)+'\n'); print(out)
PY
}

nuke; write_metadata; run_trial "${NOMINAL_SEED}" 0 stage_a; OUTCOME="$(stage_outcome)"
if [[ "${OUTCOME}" == A_TRUE_LIFT ]]; then for i in 1 2 3; do run_trial "$((NOMINAL_SEED+i))" "$i" confirm; done; stage_outcome >/dev/null; fi
for f in scene.mp4 wrist.mp4 side_by_side.mp4; do cp -f "${OUT}/trial/${f}" "${OUT}/videos/${f}" 2>/dev/null || true; done
echo "SMOKE_OUT=${OUT}"; echo "STAGE_A_OUTCOME=$(jq -r .stage_a_outcome "${OUT}/summary.json")"; nuke
