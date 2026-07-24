#!/usr/bin/env bash
# Bounded SmolVLA Recovery-v3 → Isaac S4 (≤5 seeds).
# Closed-loop abs-EEF via smolvla_policy_inference_node + ActionChunk K=5.
# Lift success is ContinuousTaskEvaluator GT only; never claim Sim2Real.
#
# Required local Franka USD (offline / weak Nucleus):
#   export ISAAC_FRANKA_USD=$HOME/isaac_assets/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd
#   export ISAAC_REQUIRE_LOCAL_FRANKA=1
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly MIDSTREAM="${MIDSTREAM:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab}"
readonly DATE_TAG="${DATE_TAG:-$(date +%Y%m%dT%H%M%SZ)}"
readonly SUITE_OUT="${1:-${MIDSTREAM}/evidence/smolvla_s4_bounded5_${DATE_TAG}}"
readonly ISAAC_PYTHON="${ISAAC_PYTHON:-/home/ina/isaacsim/.venv/bin/python}"
readonly POLICY_PYTHON="${POLICY_PYTHON:-/home/ina/miniforge3/envs/lerobot/bin/python}"
readonly SEEDS="${SEEDS:-1 2 3 4 5}"
readonly ARM_COMMAND_MODE="${ARM_COMMAND_MODE:-position}"
readonly BACKEND_DURATION_SEC="${BACKEND_DURATION_SEC:-180}"
readonly POLICY_RUNTIME_TIMEOUT_S="${POLICY_RUNTIME_TIMEOUT_S:-140}"
readonly POLICY_STARTUP_TIMEOUT_S="${POLICY_STARTUP_TIMEOUT_S:-90}"
readonly MAX_ACTIONS="${MAX_ACTIONS:-150}"
readonly N_ACTION_STEPS="${N_ACTION_STEPS:-5}"
readonly INFERENCE_RATE_HZ="${INFERENCE_RATE_HZ:-10.0}"
readonly BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-130}"
readonly RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Default off on ~6GB laptop VRAM.
readonly RECORD_SCENE_VIDEO="${RECORD_SCENE_VIDEO:-false}"
readonly EVALUATION_RUN_ID="${EVALUATION_RUN_ID:-smolvla_s4_bounded5_${DATE_TAG}}"
readonly EVALUATION_MODEL_ID="${EVALUATION_MODEL_ID:-smolvla_recovery_v3}"
readonly LIFT_SUCCESS_DELTA="${LIFT_SUCCESS_DELTA:-0.03}"
readonly GRIPPER_CLOSE_MAX="${GRIPPER_CLOSE_MAX:-0.70}"
readonly PASS_THRESHOLD="${PASS_THRESHOLD:-1}"
readonly DEVICE="${DEVICE:-cuda}"
readonly DRY_RUN="${DRY_RUN:-false}"
readonly REQUIRE_REPORT_PASS="${REQUIRE_REPORT_PASS:-false}"
readonly WORKSPACE_MIN="${WORKSPACE_MIN:-0.20,-0.40,0.02}"
readonly WORKSPACE_MAX="${WORKSPACE_MAX:-0.65,0.40,0.75}"
readonly MAX_JOINT_EXCURSION_RAD="${MAX_JOINT_EXCURSION_RAD:-3.0}"
readonly MAX_EE_EXCURSION_M="${MAX_EE_EXCURSION_M:-0.55}"
readonly TASK="${TASK:-pick up the red box and place it in the left bin}"

readonly LORA_DIR="${LORA_DIR:-${MIDSTREAM}/runs/smolvla_s3/recovery_v3_lora_20260723T125632Z/lerobot_run/checkpoints/005705/pretrained_model}"
readonly BASE_DIR="${BASE_DIR:-${MIDSTREAM}/checkpoints/smolvla_base_gate_s1}"
readonly VLM_DIR="${VLM_DIR:-${MIDSTREAM}/checkpoints/SmolVLM2-500M-Video-Instruct}"

export RMW_IMPLEMENTATION
export ISAAC_REQUIRE_LOCAL_FRANKA="${ISAAC_REQUIRE_LOCAL_FRANKA:-1}"
if [[ -z "${ISAAC_FRANKA_USD:-}" ]]; then
  export ISAAC_FRANKA_USD="${HOME}/isaac_assets/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
fi

if [[ ! -x "${ISAAC_PYTHON}" || ! -x "${POLICY_PYTHON}" ]]; then
  echo "Isaac or policy Python is not executable" >&2
  exit 2
fi
if [[ ! -d "${LORA_DIR}" ]]; then
  echo "LoRA checkpoint missing: ${LORA_DIR}" >&2
  exit 2
fi
if [[ ! -f "${ISAAC_FRANKA_USD}" ]]; then
  echo "Local Franka USD missing: ${ISAAC_FRANKA_USD}" >&2
  exit 2
fi
if [[ "${ARM_COMMAND_MODE}" == "position" ]]; then
  controller_profile="forward"
elif [[ "${ARM_COMMAND_MODE}" == "effort" ]]; then
  controller_profile="impedance"
else
  echo "ARM_COMMAND_MODE must be effort or position" >&2
  exit 2
fi

# shellcheck disable=SC2206
SEED_LIST=(${SEEDS})
if [[ "${#SEED_LIST[@]}" -gt 5 ]]; then
  echo "Bounded S4 allows at most 5 seeds; got: ${SEEDS}" >&2
  exit 2
fi

mkdir -p "${SUITE_OUT}/trials" "${SUITE_OUT}/videos"
EPISODE_RESULTS_PATH="${SUITE_OUT}/episode_results.jsonl"
export EPISODE_RESULTS_PATH
: > "${EPISODE_RESULTS_PATH}"

nuke() {
  "${REPO_ROOT}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  pkill -9 -f '[i]saac_panda_backend.py' 2>/dev/null || true
  pkill -9 -f '[s]molvla_policy_inference_node' 2>/dev/null || true
  pkill -9 -f '[i]saac_scene_video_recorder' 2>/dev/null || true
  pkill -9 -f '[i]saac_continuous_gt_recorder' 2>/dev/null || true
  pkill -9 -f '[t]eleop_bringup' 2>/dev/null || true
  pkill -9 -f '[s]ervo_node' 2>/dev/null || true
  pkill -9 -f '[r]os2_control' 2>/dev/null || true
  sleep 2
}

python3 - "${SUITE_OUT}" "${EVALUATION_RUN_ID}" "${SEEDS}" \
  "${LORA_DIR}" "${BASE_DIR}" "${VLM_DIR}" \
  "${MIDSTREAM}/evaluation/examples/nominal_contract_fixture/run_manifest.json" <<'PY'
import hashlib, json, subprocess, sys
from datetime import datetime, timezone
from pathlib import Path

out, run_id, seeds_text, lora, base, vlm, fixture = sys.argv[1:8]
out = Path(out)
seeds = [int(x) for x in seeds_text.split()]
suite = {
    "suite_id": "smolvla_s4_bounded5",
    "suite_version": "0.1.0",
    "description": (
        "Bounded SmolVLA Recovery-v3 Isaac S4 closed-loop abs-EEF "
        "(≤5 seeds; lift GT via ContinuousTaskEvaluator)."
    ),
    "scene_id": "panda_pick_place_v1",
    "seeds": seeds,
    "protocol_id": "smolvla_s4_abs_eef_closed_loop",
    "status": "runtime_diagnostic",
    "validation_mode": "lift",
    "checkpoint": {
        "lora_dir": lora,
        "base_dir": base,
        "vlm_dir": vlm,
    },
    "runtime": {
        "control_rate_hz": 10.0,
        "chunk_size": 10,
        "execute_k": 5,
        "replan_period_s": 0.5,
        "gripper_command": "clip(raw, 0, 1)",
        "cameras": "scene-only",
        "state": "observation.state[15]",
    },
    "claims_task_success": False,
    "claims_sim2real": False,
}
(out / "s4_suite.json").write_text(json.dumps(suite, indent=2) + "\n")

manifest = json.loads(Path(fixture).read_text(encoding="utf-8"))
up = Path("/home/ina/dev/ros2-arm-teleoperation-suite")
mid = Path("/home/ina/robot-sim-lab/robot-arm-episode-data-lab")
down = Path("/home/ina/ros2_ws/src/ros2-moveit-pybullet-bridge")

def git_sha(path: Path):
    try:
        return subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
        ).strip()
    except Exception:
        return None

manifest["evaluation_run_id"] = run_id
manifest["created_at"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
manifest["execution_status"] = "running"
manifest["evidence_level"] = "runtime_observed"
manifest["provenance"]["model"] = {
    "model_id": "smolvla_recovery_v3",
    "model_commit": git_sha(up),
    "checkpoint_sha256": None,
    "checkpoint_path": lora,
}
manifest["provenance"]["dataset"] = {
    "release_id": "smolvla_s3_panda_abs_eef_scene_v3_phaseaware50",
    "manifest_path": None,
    "manifest_sha256": None,
}
manifest["provenance"]["repositories"] = {
    "upstream": {"repository": "ros2-arm-teleoperation-suite", "commit_sha": git_sha(up)},
    "midstream": {"repository": "robot-arm-episode-data-lab", "commit_sha": git_sha(mid)},
    "downstream": {"repository": "ros2-moveit-pybullet-bridge", "commit_sha": git_sha(down)},
}
manifest["scenario"]["seeds"] = seeds
manifest["scenario"]["suite"]["config_path"] = "s4_suite.json"
manifest["scenario"]["suite"]["config_sha256"] = hashlib.sha256(
    (out / "s4_suite.json").read_bytes()
).hexdigest()
manifest["simulator"]["backend"] = "isaac"
manifest["action_contract"]["policy_rate_hz"] = 10.0
manifest["action_contract"]["future_runtime_adapter"]["implementation_status"] = (
    "smolvla_abs_eef_online_v0"
)
manifest["evidence_paths"] = {
    "artifact_root": str(out),
    "raw_episode_pattern": "trials/seed_{seed}/",
    "video_pattern": "videos/seed_{seed}.mp4",
    "runtime_log_pattern": "trials/seed_{seed}/policy.log",
    "qos_preflight": "nfr/dds_qos.txt",
    "nfr_snapshot": "nfr/",
}
manifest["limitations"] = [
    "Bounded ≤5-seed Isaac S4 diagnostic; open-loop Pass ≠ task success.",
    "validation_mode=lift (not place).",
    "Does not claim Sim2Real or real-robot deployment.",
]
(out / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(f"Wrote run_manifest → {out}")
PY

run_one_seed() {
  local seed="$1"
  local episode_index="$2"
  local trial_dir="${SUITE_OUT}/trials/seed_${seed}"
  mkdir -p "${trial_dir}"

  local backend_pid="" stack_pid="" video_pid="" gt_pid="" hb_pid=""
  cleanup_trial() {
    [[ -z "${video_pid}" ]] || kill "${video_pid}" 2>/dev/null || true
    [[ -z "${gt_pid}" ]] || kill "${gt_pid}" 2>/dev/null || true
    [[ -z "${hb_pid}" ]] || kill "${hb_pid}" 2>/dev/null || true
    [[ -z "${stack_pid}" ]] || kill "${stack_pid}" 2>/dev/null || true
    [[ -z "${backend_pid}" ]] || kill "${backend_pid}" 2>/dev/null || true
    [[ -z "${video_pid}" ]] || wait "${video_pid}" 2>/dev/null || true
    [[ -z "${gt_pid}" ]] || wait "${gt_pid}" 2>/dev/null || true
    [[ -z "${hb_pid}" ]] || wait "${hb_pid}" 2>/dev/null || true
    [[ -z "${stack_pid}" ]] || wait "${stack_pid}" 2>/dev/null || true
    [[ -z "${backend_pid}" ]] || wait "${backend_pid}" 2>/dev/null || true
    "${REPO_ROOT}/scripts/stop_stack.sh" > "${trial_dir}/cleanup.log" 2>&1 || true
    pkill -9 -f '[i]saac_panda_backend.py' 2>/dev/null || true
    pkill -9 -f '[s]molvla_policy_inference_node' 2>/dev/null || true
    pkill -9 -f '[i]saac_scene_video_recorder' 2>/dev/null || true
    pkill -9 -f '[i]saac_continuous_gt_recorder' 2>/dev/null || true
  }
  trap cleanup_trial RETURN

  export ROS_DOMAIN_ID=$((BASE_ROS_DOMAIN_ID + seed))

  set +u
  source /opt/ros/jazzy/setup.bash
  source "${REPO_ROOT}/install/setup.bash"
  set -u

  local backend_args=(
    --duration-sec "${BACKEND_DURATION_SEC}"
    --camera-rate 10
    --command-timeout-s 0.1
    --arm-command-mode "${ARM_COMMAND_MODE}"
    --object-seed "${seed}"
  )
  if [[ -n "${ISAAC_FRANKA_USD:-}" ]]; then
    backend_args+=(--franka-usd "${ISAAC_FRANKA_USD}")
  fi

  timeout "$((BACKEND_DURATION_SEC + 30))s" "${ISAAC_PYTHON}" \
    "${REPO_ROOT}/src/isaac_sim_adapter/scripts/isaac_panda_backend.py" \
    "${backend_args[@]}" \
    > "${trial_dir}/backend.log" 2>&1 &
  backend_pid=$!

  local ready=false
  for _ in $(seq 1 160); do
    if grep -q "ISAAC_E1_READY=" "${trial_dir}/backend.log" 2>/dev/null; then
      ready=true
      break
    fi
    if ! kill -0 "${backend_pid}" 2>/dev/null; then
      tail -n 80 "${trial_dir}/backend.log" >&2
      echo 3 > "${trial_dir}/policy_exit_code.txt"
      return 3
    fi
    sleep 0.5
  done
  if [[ "${ready}" != "true" ]]; then
    echo "Isaac backend READY timeout (seed ${seed})" >&2
    echo 3 > "${trial_dir}/policy_exit_code.txt"
    return 3
  fi

  timeout 85s ros2 launch teleop_bringup full_system.launch.py \
    sim_backend:=isaac record:=false start_teleop:=false \
    controller:="${controller_profile}" \
    enable_grasp_monitor:=false camera_rate:=10.0 watchdog_timeout:=30.0 \
    > "${trial_dir}/full_system.log" 2>&1 &
  stack_pid=$!

  local graph_ready=false
  for _ in $(seq 1 180); do
    if ros2 topic list 2>/dev/null | grep -qx "/sim/encoder_state" \
      && ros2 topic list 2>/dev/null | grep -qx "/ee_pose" \
      && ros2 topic list 2>/dev/null | grep -qx "/safety/status"; then
      graph_ready=true
      break
    fi
    if ! kill -0 "${stack_pid}" 2>/dev/null; then
      tail -n 100 "${trial_dir}/full_system.log" >&2
      echo 4 > "${trial_dir}/policy_exit_code.txt"
      return 4
    fi
    sleep 0.25
  done
  if [[ "${graph_ready}" != "true" ]]; then
    echo "Isaac control graph discovery timeout (seed ${seed})" >&2
    echo 4 > "${trial_dir}/policy_exit_code.txt"
    return 4
  fi

  sleep 6
  timeout 8s ros2 topic echo /safety/status --once \
    > "${trial_dir}/safety_pre.txt" || true
  if ! grep -q "ok: true" "${trial_dir}/safety_pre.txt" 2>/dev/null; then
    echo "Safety preflight is not OK (seed ${seed})" >&2
    echo 5 > "${trial_dir}/policy_exit_code.txt"
    return 5
  fi

  export SERVO_POST_INIT_MODE=pose
  bash "${REPO_ROOT}/scripts/servo_post_init.sh" 4 20 \
    > "${trial_dir}/servo_post_init.txt" 2>&1 || true
  timeout 8s ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool \
    "{data: false}" >/dev/null 2>&1 || true
  sleep 2
  timeout 8s ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool \
    "{data: false}" >/dev/null 2>&1 || true
  nice -n 19 ros2 topic pub -r 30 /teleop/heartbeat std_msgs/msg/Header \
    "{frame_id: 'isaac_smolvla_s4_hb'}" >/dev/null 2>&1 &
  hb_pid=$!
  sleep 1

  if [[ "${RECORD_SCENE_VIDEO}" == "true" ]]; then
    /usr/bin/python3 "${REPO_ROOT}/scripts/isaac_scene_video_recorder.py" \
      --output "${trial_dir}/scene.mp4" \
      --max-duration-s "${POLICY_RUNTIME_TIMEOUT_S}" \
      --max-frames 600 \
      > "${trial_dir}/video_recorder.log" 2>&1 &
    video_pid=$!
  fi

  export PYTHONPATH="${REPO_ROOT}/src/synth_data_gen:${REPO_ROOT}/src/isaac_sim_adapter:${REPO_ROOT}/install/synth_data_gen/lib/python3.12/site-packages:${REPO_ROOT}/install/isaac_sim_adapter/lib/python3.12/site-packages:${PYTHONPATH:-}"
  /usr/bin/python3 "${REPO_ROOT}/scripts/isaac_continuous_gt_recorder.py" \
    --episode-results-path "${EPISODE_RESULTS_PATH}" \
    --evaluation-run-id "${EVALUATION_RUN_ID}" \
    --seed "${seed}" \
    --episode-index "${episode_index}" \
    --model-id "${EVALUATION_MODEL_ID}" \
    --suite-id smolvla_s4_bounded5 \
    --validation-mode lift \
    --lift-success-delta "${LIFT_SUCCESS_DELTA}" \
    --gripper-close-max "${GRIPPER_CLOSE_MAX}" \
    --wait-for-report "${trial_dir}/report.json" \
    --raw-episode-path "${trial_dir}" \
    --video-path "${trial_dir}/scene.mp4" \
    --runtime-log-path "${trial_dir}/gt_runtime.log" \
    --event-log-path "${trial_dir}/gt_events.jsonl" \
    --nfr-sample-path "${trial_dir}/gt_nfr.json" \
    --max-duration-s "$((POLICY_RUNTIME_TIMEOUT_S + 60))" \
    > "${trial_dir}/gt_recorder.log" 2>&1 &
  gt_pid=$!

  set +e
  echo "[smolvla-s4] seed=${seed} domain=${ROS_DOMAIN_ID} timeout=${POLICY_RUNTIME_TIMEOUT_S}s device=${DEVICE}"
  timeout "${POLICY_RUNTIME_TIMEOUT_S}s" "${POLICY_PYTHON}" -m \
    isaac_sim_adapter.smolvla_policy_inference_node --ros-args \
    -p lora_dir:="${LORA_DIR}" \
    -p base_dir:="${BASE_DIR}" \
    -p vlm_dir:="${VLM_DIR}" \
    -p device:="${DEVICE}" \
    -p dry_run:="${DRY_RUN}" \
    -p max_actions:="${MAX_ACTIONS}" \
    -p n_action_steps:="${N_ACTION_STEPS}" \
    -p inference_rate_hz:="$(python3 -c "print(float('${INFERENCE_RATE_HZ}'))")" \
    -p startup_timeout_s:="$(python3 -c "print(float('${POLICY_STARTUP_TIMEOUT_S}'))")" \
    -p post_action_hold_s:=2.0 \
    -p max_joint_excursion_rad:="$(python3 -c "print(float('${MAX_JOINT_EXCURSION_RAD}'))")" \
    -p max_ee_excursion_m:="$(python3 -c "print(float('${MAX_EE_EXCURSION_M}'))")" \
    -p workspace_min:="[${WORKSPACE_MIN}]" \
    -p workspace_max:="[${WORKSPACE_MAX}]" \
    -p task:="${TASK}" \
    -p output_path:="${trial_dir}/report.json" \
    > "${trial_dir}/policy.log" 2>&1
  local policy_status=$?
  set -e
  echo "${policy_status}" > "${trial_dir}/policy_exit_code.txt"

  sleep 2
  if [[ -n "${gt_pid}" ]]; then
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
    sleep 2
    kill -TERM "${video_pid}" 2>/dev/null || true
    for _ in $(seq 1 40); do
      if ! kill -0 "${video_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.25
    done
    kill -9 "${video_pid}" 2>/dev/null || true
    wait "${video_pid}" 2>/dev/null || true
    video_pid=""
  fi
  if [[ -n "${hb_pid}" ]]; then
    kill "${hb_pid}" 2>/dev/null || true
    wait "${hb_pid}" 2>/dev/null || true
    hb_pid=""
  fi

  if [[ -f "${trial_dir}/scene.mp4" ]]; then
    cp -f "${trial_dir}/scene.mp4" "${SUITE_OUT}/videos/seed_${seed}.mp4" || true
  fi

  if [[ "${REQUIRE_REPORT_PASS}" == "true" ]]; then
    if [[ ! -f "${trial_dir}/report.json" ]]; then
      return "${policy_status}"
    fi
    "${POLICY_PYTHON}" - "${trial_dir}/report.json" <<'PY'
import json, pathlib, sys
report = json.loads(pathlib.Path(sys.argv[1]).read_text())
if report.get("status") != "PASS":
    raise SystemExit(1)
PY
  fi
  return "${policy_status}"
}

nuke
episode_index=0
for seed in "${SEED_LIST[@]}"; do
  if [[ -f "${SUITE_OUT}/trials/seed_${seed}/report.json" ]] \
    && [[ -f "${SUITE_OUT}/trials/seed_${seed}/gt_runtime.log" ]]; then
    echo "[smolvla-s4] resume skip seed=${seed}"
    episode_index=$((episode_index + 1))
    continue
  fi
  echo "========== [smolvla-s4] seed=${seed} (${episode_index}/$(( ${#SEED_LIST[@]} - 1 ))) =========="
  nuke
  set +e
  run_one_seed "${seed}" "${episode_index}"
  status=$?
  set -e
  echo "[smolvla-s4] seed=${seed} exit=${status}"
  nuke
  episode_index=$((episode_index + 1))
done

python3 - "${SUITE_OUT}/run_manifest.json" <<'PY'
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
m = json.loads(p.read_text())
m["execution_status"] = "completed"
m["evidence_level"] = "runtime_observed"
m["simulator"].update({
    "version": m["simulator"].get("version") or "isaac-sim-local",
    "build_id": m["simulator"].get("build_id") or "local",
    "driver_version": m["simulator"].get("driver_version") or "unknown",
    "hardware_id": m["simulator"].get("hardware_id") or "local-gpu",
})
p.write_text(json.dumps(m, indent=2) + "\n")
PY

if [[ -f "${MIDSTREAM}/training/scripts/aggregate_evaluation_summary.py" ]]; then
  python3 "${MIDSTREAM}/training/scripts/aggregate_evaluation_summary.py" \
    --run-dir "${SUITE_OUT}" \
    || true
fi

python3 - "${SUITE_OUT}" "${PASS_THRESHOLD}" "${#SEED_LIST[@]}" <<'PY'
import json, sys
from pathlib import Path

out = Path(sys.argv[1])
need = int(sys.argv[2])
planned = int(sys.argv[3])
rows = []
path = out / "episode_results.jsonl"
if path.is_file():
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))

def _sg_ok(subgoals, key):
    value = (subgoals or {}).get(key)
    if isinstance(value, bool):
        return value
    if isinstance(value, dict):
        return bool(value.get("success"))
    return False

lift_n = reach_n = grasp_n = success_n = 0
for row in rows:
    sg = row.get("subgoals") or {}
    if _sg_ok(sg, "lift"):
        lift_n += 1
    if _sg_ok(sg, "reach"):
        reach_n += 1
    if _sg_ok(sg, "grasp"):
        grasp_n += 1
    if (row.get("outcome") or {}).get("success"):
        success_n += 1

policy_reports = list((out / "trials").glob("seed_*/report.json"))
policy_pass = 0
for report_path in policy_reports:
    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception:
        continue
    if payload.get("status") == "PASS":
        policy_pass += 1

gate = {
    "gate_id": "smolvla_s4_bounded5_lift",
    "seeds_planned": planned,
    "episodes_recorded": len(rows),
    "policy_reports": len(policy_reports),
    "policy_interface_pass": policy_pass,
    "reach": reach_n,
    "grasp": grasp_n,
    "lift": lift_n,
    "outcome_success": success_n,
    "pass_threshold": need,
    "gate_pass": lift_n >= need,
    "ran_isaac": True,
    "claims_task_success": False,
    "claims_sim2real": False,
    "interpretation": (
        "bounded_s4_lift_evidence"
        if lift_n >= need
        else "bounded_s4_hold_or_incomplete"
    ),
}
(out / "s4_gate.json").write_text(json.dumps(gate, indent=2) + "\n")
print(json.dumps(gate, indent=2))
PY

echo "SUITE_OUT=${SUITE_OUT}"
echo "EPISODE_RESULTS_PATH=${EPISODE_RESULTS_PATH}"
nuke
