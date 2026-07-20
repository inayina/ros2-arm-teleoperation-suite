#!/usr/bin/env bash
# Isaac scripted-oracle 5-repeat lift regression (physics-chain gate).
# Does NOT run ACT. Physical success comes only from ContinuousTaskEvaluator.
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly MIDSTREAM="${MIDSTREAM:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab}"
readonly DATE_TAG="${DATE_TAG:-$(date +%Y%m%d)}"
readonly SUITE_OUT="${1:-${MIDSTREAM}/evidence/e3p5_isaac_scripted_oracle_5x_lift_${DATE_TAG}}"
readonly ISAAC_PYTHON="${ISAAC_PYTHON:-/home/ina/isaacsim/.venv/bin/python}"
readonly ORACLE_PYTHON="${ORACLE_PYTHON:-/usr/bin/python3}"
readonly TRIALS="${TRIALS:-5}"
readonly ARM_COMMAND_MODE="${ARM_COMMAND_MODE:-position}"
readonly BACKEND_DURATION_SEC="${BACKEND_DURATION_SEC:-150}"
readonly ORACLE_RUNTIME_TIMEOUT_S="${ORACLE_RUNTIME_TIMEOUT_S:-110}"
readonly BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-120}"
readonly RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
readonly RECORD_SCENE_VIDEO="${RECORD_SCENE_VIDEO:-true}"
readonly EVALUATION_RUN_ID="${EVALUATION_RUN_ID:-e3p5_isaac_scripted_oracle_5x_lift_${DATE_TAG}}"
readonly EVALUATION_MODEL_ID="${EVALUATION_MODEL_ID:-scripted_oracle}"
# Fixed nominal red-box pose (physics repeatability). Override via OBJECT_XY if needed.
readonly OBJECT_SEED="${OBJECT_SEED:-}"
readonly OBJECT_XY="${OBJECT_XY:-}"
readonly LIFT_SUCCESS_DELTA="${LIFT_SUCCESS_DELTA:-0.03}"
readonly HOVER_Z="${HOVER_Z:-0.12}"
readonly PICK_Z_OFFSET="${PICK_Z_OFFSET:-0.010}"
readonly LIFT_Z="${LIFT_Z:-0.12}"
readonly GRIPPER_CLOSE_TARGET="${GRIPPER_CLOSE_TARGET:-0.40}"
readonly GRIPPER_CLOSE_MAX="${GRIPPER_CLOSE_MAX:-0.70}"
readonly PASS_THRESHOLD="${PASS_THRESHOLD:-4}"

export RMW_IMPLEMENTATION

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "ISAAC_PYTHON is not executable: ${ISAAC_PYTHON}" >&2
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

mkdir -p "${SUITE_OUT}/trials" "${SUITE_OUT}/videos"
EPISODE_RESULTS_PATH="${SUITE_OUT}/episode_results.jsonl"
export EPISODE_RESULTS_PATH
: > "${EPISODE_RESULTS_PATH}"

nuke() {
  "${REPO_ROOT}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  pkill -9 -f '[i]saac_panda_backend.py' 2>/dev/null || true
  pkill -9 -f '[i]saac_scripted_oracle.py' 2>/dev/null || true
  pkill -9 -f '[i]saac_scene_video_recorder' 2>/dev/null || true
  pkill -9 -f '[i]saac_continuous_gt_recorder' 2>/dev/null || true
  pkill -9 -f '[t]eleop_bringup' 2>/dev/null || true
  pkill -9 -f '[s]ervo_node' 2>/dev/null || true
  pkill -9 -f '[r]os2_control' 2>/dev/null || true
  sleep 2
}

# Suite config + minimal run_manifest for aggregate_evaluation_summary.
python3 - "${SUITE_OUT}" "${EVALUATION_RUN_ID}" "${TRIALS}" \
  "${MIDSTREAM}/evaluation/examples/nominal_contract_fixture/run_manifest.json" <<'PY'
import hashlib, json, subprocess, sys
from datetime import datetime, timezone
from pathlib import Path

out, run_id, trials, fixture = sys.argv[1:5]
out = Path(out)
n = int(trials)
seeds = list(range(n))  # trial indices as synthetic seeds (fixed nominal pose)

suite = {
    "suite_id": "scripted_oracle_nominal",
    "suite_version": "0.1.0",
    "description": (
        "E3.5 Isaac scripted-oracle 5-repeat lift gate "
        "(fixed nominal red-box; not learned-policy)."
    ),
    "scene_id": "panda_pick_place_v1",
    "seeds": seeds,
    "protocol_id": "scripted_oracle_lift",
    "status": "runtime_diagnostic",
    "validation_mode": "lift",
}
(out / "oracle_suite.json").write_text(json.dumps(suite, indent=2) + "\n")

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
    "model_id": "scripted_oracle",
    "model_commit": git_sha(up),
    "checkpoint_sha256": None,
    "checkpoint_path": None,
}
manifest["provenance"]["dataset"] = {
    "release_id": None,
    "manifest_path": None,
    "manifest_sha256": None,
}
manifest["provenance"]["repositories"] = {
    "upstream": {"repository": "ros2-arm-teleoperation-suite", "commit_sha": git_sha(up)},
    "midstream": {"repository": "robot-arm-episode-data-lab", "commit_sha": git_sha(mid)},
    "downstream": {"repository": "ros2-moveit-pybullet-bridge", "commit_sha": git_sha(down)},
}
manifest["scenario"]["seeds"] = seeds
manifest["scenario"]["suite"]["config_path"] = "oracle_suite.json"
manifest["scenario"]["suite"]["config_sha256"] = hashlib.sha256(
    (out / "oracle_suite.json").read_bytes()
).hexdigest()
manifest["simulator"]["backend"] = "isaac"
manifest["action_contract"]["policy_rate_hz"] = 20.0
manifest["action_contract"]["future_runtime_adapter"]["implementation_status"] = "n/a_scripted_oracle"
manifest["evidence_paths"] = {
    "artifact_root": str(out),
    "raw_episode_pattern": "trials/trial_{seed}/",
    "video_pattern": "videos/trial_{seed}.mp4",
    "runtime_log_pattern": "trials/trial_{seed}/oracle.log",
    "qos_preflight": "nfr/dds_qos.txt",
    "nfr_snapshot": "nfr/",
}
manifest["limitations"] = [
    "Scripted oracle physics-chain gate only; not learned-policy success.",
    "Fixed nominal red-box pose by default; not a generalization suite.",
    "validation_mode=lift (not place).",
]
(out / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(f"Wrote run_manifest → {out}")
PY

run_one_trial() {
  local trial_index="$1"
  local trial_dir="${SUITE_OUT}/trials/trial_${trial_index}"
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
    pkill -9 -f '[i]saac_scripted_oracle.py' 2>/dev/null || true
    pkill -9 -f '[i]saac_scene_video_recorder' 2>/dev/null || true
    pkill -9 -f '[i]saac_continuous_gt_recorder' 2>/dev/null || true
  }
  trap cleanup_trial RETURN

  export ROS_DOMAIN_ID=$((BASE_ROS_DOMAIN_ID + trial_index))

  set +u
  source /opt/ros/jazzy/setup.bash
  source "${REPO_ROOT}/install/setup.bash"
  set -u

  local backend_args=(
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

  timeout "$((BACKEND_DURATION_SEC + 30))s" "${ISAAC_PYTHON}" \
    "${REPO_ROOT}/src/isaac_sim_adapter/scripts/isaac_panda_backend.py" \
    "${backend_args[@]}" \
    > "${trial_dir}/backend.log" 2>&1 &
  backend_pid=$!

  local ready=false
  for _ in $(seq 1 140); do
    if grep -q "ISAAC_E1_READY=" "${trial_dir}/backend.log" 2>/dev/null; then
      ready=true
      break
    fi
    if ! kill -0 "${backend_pid}" 2>/dev/null; then
      tail -n 80 "${trial_dir}/backend.log" >&2
      echo 3 > "${trial_dir}/oracle_exit_code.txt"
      return 3
    fi
    sleep 0.5
  done
  if [[ "${ready}" != "true" ]]; then
    echo "Isaac backend READY timeout (trial ${trial_index})" >&2
    echo 3 > "${trial_dir}/oracle_exit_code.txt"
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
      echo 4 > "${trial_dir}/oracle_exit_code.txt"
      return 4
    fi
    sleep 0.25
  done
  if [[ "${graph_ready}" != "true" ]]; then
    echo "Isaac control graph discovery timeout (trial ${trial_index})" >&2
    echo 4 > "${trial_dir}/oracle_exit_code.txt"
    return 4
  fi

  sleep 6
  timeout 8s ros2 topic echo /sim/encoder_state --once \
    > "${trial_dir}/initial_encoder.txt" || true
  timeout 8s ros2 topic echo /ee_pose --once \
    > "${trial_dir}/initial_ee_pose.txt" || true
  timeout 8s ros2 topic echo /sim/object_pose --once \
    > "${trial_dir}/initial_object_pose.txt" || true
  timeout 8s ros2 topic echo /safety/status --once \
    > "${trial_dir}/safety_pre.txt" || true
  if ! grep -q "ok: true" "${trial_dir}/safety_pre.txt" 2>/dev/null; then
    echo "Safety preflight is not OK (trial ${trial_index})" >&2
    echo 5 > "${trial_dir}/oracle_exit_code.txt"
    return 5
  fi

  export SERVO_POST_INIT_MODE=pose
  bash "${REPO_ROOT}/scripts/servo_post_init.sh" 4 20 \
    > "${trial_dir}/servo_post_init.txt" 2>&1 || true
  timeout 8s ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool \
    "{data: false}" >/dev/null 2>&1 || true
  # Extra unpause after pose switch; MoveIt sometimes leaves pause latched.
  sleep 2
  timeout 8s ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool \
    "{data: false}" >/dev/null 2>&1 || true
  nice -n 19 ros2 topic pub -r 30 /teleop/heartbeat std_msgs/msg/Header \
    "{frame_id: 'isaac_scripted_oracle_hb'}" >/dev/null 2>&1 &
  hb_pid=$!
  sleep 1

  if [[ "${RECORD_SCENE_VIDEO}" == "true" ]]; then
    /usr/bin/python3 "${REPO_ROOT}/scripts/isaac_scene_video_recorder.py" \
      --output "${trial_dir}/scene.mp4" \
      --max-duration-s "${ORACLE_RUNTIME_TIMEOUT_S}" \
      --max-frames 600 \
      > "${trial_dir}/video_recorder.log" 2>&1 &
    video_pid=$!
  fi

  export PYTHONPATH="${REPO_ROOT}/src/synth_data_gen:${REPO_ROOT}/src/isaac_sim_adapter:${REPO_ROOT}/install/synth_data_gen/lib/python3.12/site-packages:${REPO_ROOT}/install/isaac_sim_adapter/lib/python3.12/site-packages:${PYTHONPATH:-}"
  /usr/bin/python3 "${REPO_ROOT}/scripts/isaac_continuous_gt_recorder.py" \
    --episode-results-path "${EPISODE_RESULTS_PATH}" \
    --evaluation-run-id "${EVALUATION_RUN_ID}" \
    --seed "${trial_index}" \
    --episode-index "${trial_index}" \
    --model-id "${EVALUATION_MODEL_ID}" \
    --suite-id scripted_oracle_nominal \
    --validation-mode lift \
    --lift-success-delta "${LIFT_SUCCESS_DELTA}" \
    --gripper-close-max "${GRIPPER_CLOSE_MAX}" \
    --wait-for-report "${trial_dir}/report.json" \
    --raw-episode-path "${trial_dir}" \
    --video-path "${trial_dir}/scene.mp4" \
    --runtime-log-path "${trial_dir}/gt_runtime.log" \
    --event-log-path "${trial_dir}/gt_events.jsonl" \
    --nfr-sample-path "${trial_dir}/gt_nfr.json" \
    --max-duration-s "$((ORACLE_RUNTIME_TIMEOUT_S + 60))" \
    > "${trial_dir}/gt_recorder.log" 2>&1 &
  gt_pid=$!

  set +e
  echo "[oracle] trial=${trial_index} domain=${ROS_DOMAIN_ID} timeout=${ORACLE_RUNTIME_TIMEOUT_S}s"
  timeout "${ORACLE_RUNTIME_TIMEOUT_S}s" "${ORACLE_PYTHON}" \
    "${REPO_ROOT}/scripts/isaac_scripted_oracle.py" \
    --hover-z "${HOVER_Z}" \
    --pick-z-offset "${PICK_Z_OFFSET}" \
    --lift-z "${LIFT_Z}" \
    --gripper-close-target "${GRIPPER_CLOSE_TARGET}" \
    --output "${trial_dir}/oracle_report.json" \
    > "${trial_dir}/oracle.log" 2>&1
  local oracle_status=$?
  set -e
  echo "${oracle_status}" > "${trial_dir}/oracle_exit_code.txt"

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
  # Give the video recorder time to flush frames + ffmpeg after SIGTERM.
  if [[ -n "${video_pid}" ]]; then
    sleep 2
    kill -TERM "${video_pid}" 2>/dev/null || true
    for _ in $(seq 1 80); do
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

  timeout 8s ros2 topic echo /ee_pose --once \
    > "${trial_dir}/final_ee_pose.txt" || true
  timeout 8s ros2 topic echo /sim/object_pose --once \
    > "${trial_dir}/final_object_pose.txt" || true
  timeout 8s ros2 topic echo /safety/status --once \
    > "${trial_dir}/safety_final.txt" || true

  if [[ -f "${trial_dir}/scene.mp4" ]]; then
    cp -f "${trial_dir}/scene.mp4" "${SUITE_OUT}/videos/trial_${trial_index}.mp4" || true
  else
    echo "[oracle] WARN trial=${trial_index} missing scene.mp4 (see video_recorder.log)" >&2
  fi
  return "${oracle_status}"
}

nuke
for trial in $(seq 0 $((TRIALS - 1))); do
  if [[ -f "${SUITE_OUT}/trials/trial_${trial}/oracle_report.json" ]] \
    && [[ -f "${SUITE_OUT}/trials/trial_${trial}/gt_runtime.log" ]]; then
    echo "[oracle] resume skip trial=${trial}"
    continue
  fi
  echo "========== [oracle] trial=${trial} / $((TRIALS - 1)) =========="
  nuke
  set +e
  run_one_trial "${trial}"
  status=$?
  set -e
  echo "[oracle] trial=${trial} exit=${status}"
  nuke
done

# Finalize manifest + aggregate summary.
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
ac = m["action_contract"]
ac["policy_rate_hz"] = ac.get("policy_rate_hz") or 20.0
ac["adapter_rate_hz"] = ac.get("adapter_rate_hz") or 50.0
ac["controller_rate_hz"] = ac.get("controller_rate_hz") or 500.0
m["clock_contract"]["use_sim_time"] = False
for item in m.get("fail_safe_contract", []):
    if item.get("threshold_source") == "runtime_config" and item.get("threshold_ms") is None:
        item["threshold_ms"] = 500.0
p.write_text(json.dumps(m, indent=2) + "\n")
PY

if [[ -f "${MIDSTREAM}/training/scripts/aggregate_evaluation_summary.py" ]]; then
  python3 "${MIDSTREAM}/training/scripts/aggregate_evaluation_summary.py" \
    --run-dir "${SUITE_OUT}" \
    || true
fi

# Gate summary: count lift subgoals / outcome.success from episode_results.
python3 - "${SUITE_OUT}" "${PASS_THRESHOLD}" "${TRIALS}" <<'PY'
import json, sys
from pathlib import Path

out = Path(sys.argv[1])
need = int(sys.argv[2])
trials = int(sys.argv[3])
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

lift_n = 0
success_n = 0
reach_n = 0
grasp_n = 0
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

gate = {
    "gate_id": "e3p5_scripted_oracle_lift",
    "trials_planned": trials,
    "episodes_recorded": len(rows),
    "reach": reach_n,
    "grasp": grasp_n,
    "lift": lift_n,
    "outcome_success": success_n,
    "pass_threshold": need,
    "gate_pass": lift_n >= need,
    "interpretation": (
        "physics_chain_ok_focus_on_policy"
        if lift_n >= need
        else "physics_or_tcp_gripper_contact_triage"
    ),
    "task_success_claimed": False,
}
(out / "oracle_gate.json").write_text(json.dumps(gate, indent=2) + "\n")
print(json.dumps(gate, indent=2))
if not gate["gate_pass"]:
    raise SystemExit(10)
PY

echo "SUITE_OUT=${SUITE_OUT}"
echo "EPISODE_RESULTS_PATH=${EPISODE_RESULTS_PATH}"
nuke
