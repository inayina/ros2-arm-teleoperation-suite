#!/usr/bin/env bash
# Run E3 nominal diagnostic suite: seeds 2000..2019, home envelope, resume-safe.
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly MIDSTREAM="${MIDSTREAM:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab}"
readonly CHECKPOINT="${CHECKPOINT:-${MIDSTREAM}/data/e2_500hz_act_random30_descend_conservative_5epoch_20260719/checkpoint.pt}"
readonly RELEASE="${RELEASE:-${MIDSTREAM}/data/releases/e2_500hz_random30_descend_20260719}"
readonly SUITE_OUT="${SUITE_OUT:-${MIDSTREAM}/evidence/e3_nominal20_home_30ep_gt_v1_20260719}"
readonly EVALUATION_RUN_ID="${EVALUATION_RUN_ID:-e3_nominal20_home_30ep_gt_v1_20260719}"
readonly EVALUATION_MODEL_ID="${EVALUATION_MODEL_ID:-e2_500hz_act_random30_descend_conservative_5epoch_20260719}"
readonly SEED_START="${SEED_START:-2000}"
readonly SEED_END="${SEED_END:-2019}"
readonly BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-90}"

# Model-card home control envelope.
export MAX_ACTIONS="${MAX_ACTIONS:-160}"
export N_ACTION_STEPS="${N_ACTION_STEPS:-8}"
export INFERENCE_RATE_HZ="${INFERENCE_RATE_HZ:-5.0}"
export MAX_JOINT_EXCURSION_RAD="${MAX_JOINT_EXCURSION_RAD:-3.0}"
export MAX_TRANSLATION_M="${MAX_TRANSLATION_M:-0.015}"
export MAX_EE_EXCURSION_M="${MAX_EE_EXCURSION_M:-0.55}"
export WORKSPACE_MIN="${WORKSPACE_MIN:-0.20,-0.40,0.02}"
export BACKEND_DURATION_SEC="${BACKEND_DURATION_SEC:-280}"
export POLICY_RUNTIME_TIMEOUT_S="${POLICY_RUNTIME_TIMEOUT_S:-220}"
export POLICY_STARTUP_TIMEOUT_S="${POLICY_STARTUP_TIMEOUT_S:-45.0}"
export PREGRASP_WARMSTART="${PREGRASP_WARMSTART:-false}"
export ARM_COMMAND_MODE="${ARM_COMMAND_MODE:-position}"
export DRY_RUN="${DRY_RUN:-false}"
export RECORD_SCENE_VIDEO="${RECORD_SCENE_VIDEO:-true}"
export REQUIRE_REPORT_PASS="${REQUIRE_REPORT_PASS:-false}"
export CHECKPOINT
export EVALUATION_RUN_ID
export EVALUATION_MODEL_ID

# Gate: refuse to start unless GT preflight passed (override with SKIP_GT_PREFLIGHT=1).
if [[ "${SKIP_GT_PREFLIGHT:-0}" != "1" ]]; then
  pref="${GT_PREFLIGHT_SUMMARY:-${MIDSTREAM}/evidence/e3_gt_preflight_v1_20260719/preflight_summary.json}"
  if [[ ! -f "${pref}" ]]; then
    echo "Missing GT preflight summary: ${pref}" >&2
    echo "Run scripts/run_isaac_gt_preflight.sh first, or SKIP_GT_PREFLIGHT=1" >&2
    exit 2
  fi
  python3 - "${pref}" <<'PY'
import json, sys
s = json.loads(open(sys.argv[1], encoding="utf-8").read())
if not s.get("ready_for_nominal20"):
    raise SystemExit(f"GT preflight not ready: {s}")
print(f"GT preflight gate OK: {s}")
PY
fi

mkdir -p "${SUITE_OUT}/seeds" "${SUITE_OUT}/videos"
EPISODE_RESULTS_PATH="${SUITE_OUT}/episode_results.jsonl"
export EPISODE_RESULTS_PATH

nuke() {
  "${REPO_ROOT}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  pkill -9 -f "isaac_panda_backend.py" 2>/dev/null || true
  pkill -9 -f "policy_inference_node" 2>/dev/null || true
  pkill -9 -f "isaac_scene_video_recorder.py" 2>/dev/null || true
  pkill -9 -f "isaac_continuous_gt_recorder.py" 2>/dev/null || true
  pkill -9 -f "teleop_bringup" 2>/dev/null || true
  pkill -9 -f "servo_node" 2>/dev/null || true
  pkill -9 -f "ros2_control" 2>/dev/null || true
  sleep 2
}

# Write / refresh run_manifest + suite config once (schema-aligned from E0 fixture).
python3 - "${SUITE_OUT}" "${EVALUATION_RUN_ID}" "${CHECKPOINT}" "${RELEASE}" "${SEED_START}" "${SEED_END}" \
  "${MIDSTREAM}/evaluation/examples/nominal_contract_fixture/run_manifest.json" <<'PY'
import hashlib, json, subprocess, sys
from datetime import datetime, timezone
from pathlib import Path

out, run_id, ckpt, release, s0, s1, fixture = sys.argv[1:8]
out = Path(out)
seeds = list(range(int(s0), int(s1) + 1))
ckpt_p = Path(ckpt)
rel_p = Path(release)
sha = hashlib.sha256(ckpt_p.read_bytes()).hexdigest() if ckpt_p.is_file() else None
rel_manifest = rel_p / "manifest.json"
rel_sha = None
release_id = rel_p.name
if rel_manifest.is_file():
    rel = json.loads(rel_manifest.read_text())
    release_id = rel.get("release_id", release_id)
    rel_sha = hashlib.sha256(rel_manifest.read_bytes()).hexdigest()

def git_sha(path: Path):
    try:
        return subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
        ).strip()
    except Exception:
        return None

suite = {
    "suite_id": "nominal",
    "suite_version": "0.1.0",
    "description": "E3 Isaac home diagnostic suite (bounded ACT, training-distribution XY seeds).",
    "scene_id": "panda_pick_place_v1",
    "seeds": seeds,
    "protocol_id": "home_start",
    "status": "runtime_diagnostic",
}
(out / "nominal_suite.json").write_text(json.dumps(suite, indent=2) + "\n")

manifest = json.loads(Path(fixture).read_text(encoding="utf-8"))
up = Path("/home/ina/dev/ros2-arm-teleoperation-suite")
mid = Path("/home/ina/robot-sim-lab/robot-arm-episode-data-lab")
down = Path("/home/ina/ros2_ws/src/ros2-moveit-pybullet-bridge")
manifest["evaluation_run_id"] = run_id
manifest["created_at"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
manifest["execution_status"] = "running"
manifest["evidence_level"] = "runtime_observed"
manifest["provenance"]["model"] = {
    "model_id": ckpt_p.parent.name,
    "model_commit": git_sha(mid),
    "checkpoint_sha256": sha,
    "checkpoint_path": str(ckpt_p),
}
manifest["provenance"]["dataset"] = {
    "release_id": release_id,
    "manifest_path": str(rel_manifest) if rel_manifest.is_file() else None,
    "manifest_sha256": rel_sha,
}
manifest["provenance"]["repositories"] = {
    "upstream": {"repository": "ros2-arm-teleoperation-suite", "commit_sha": git_sha(up)},
    "midstream": {"repository": "robot-arm-episode-data-lab", "commit_sha": git_sha(mid)},
    "downstream": {"repository": "ros2-moveit-pybullet-bridge", "commit_sha": git_sha(down)},
}
manifest["scenario"]["seeds"] = seeds
manifest["scenario"]["suite"]["config_path"] = "nominal_suite.json"
manifest["scenario"]["suite"]["config_sha256"] = hashlib.sha256(
    (out / "nominal_suite.json").read_bytes()
).hexdigest()
manifest["simulator"]["backend"] = "isaac"
manifest["action_contract"]["policy_rate_hz"] = 5.0
manifest["action_contract"]["future_runtime_adapter"]["implementation_status"] = "implemented"
manifest["evidence_paths"] = {
    "artifact_root": str(out),
    "raw_episode_pattern": "seeds/seed_{seed}/",
    "video_pattern": "videos/seed_{seed}.mp4",
    "runtime_log_pattern": "seeds/seed_{seed}/policy.log",
    "qos_preflight": "nfr/dds_qos.txt",
    "nfr_snapshot": "nfr/",
}
manifest["limitations"] = [
    "Bounded home_start diagnostic suite; not Sim2Real and not real-robot deployment.",
    "Task success uses ContinuousTaskEvaluator on Isaac privileged poses; place success may be near zero.",
    "Interface PASS must not be reported as pick/place success.",
]
(out / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(f"Wrote run_manifest with {len(seeds)} seeds → {out}")
PY

episode_index=0
for seed in $(seq "${SEED_START}" "${SEED_END}"); do
  seed_dir="${SUITE_OUT}/seeds/seed_${seed}"
  mkdir -p "${seed_dir}"
  if [[ -f "${seed_dir}/report.json" ]]; then
    echo "[suite] resume skip seed=${seed} (report.json exists)"
    episode_index=$((episode_index + 1))
    continue
  fi

  echo "========== [suite] seed=${seed} episode_index=${episode_index} =========="
  nuke
  export ROS_DOMAIN_ID=$((BASE_ROS_DOMAIN_ID + (seed % 20)))
  export OBJECT_SEED="${seed}"
  export EVALUATION_SEED="${seed}"
  export EVALUATION_EPISODE_INDEX="${episode_index}"

  set +e
  bash "${REPO_ROOT}/scripts/run_isaac_act_smoke.sh" "${seed_dir}"
  status=$?
  set -e

  if [[ -f "${seed_dir}/scene.mp4" ]]; then
    cp -f "${seed_dir}/scene.mp4" "${SUITE_OUT}/videos/seed_${seed}.mp4" || true
  fi
  echo "${status}" > "${seed_dir}/smoke_exit_code.txt"
  if [[ "${status}" -ne 0 ]]; then
    echo "[suite] seed=${seed} smoke_exit=${status} (continuing)"
  fi
  nuke
  episode_index=$((episode_index + 1))
done

# Mark manifest completed with schema-required runtime fields filled.
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
ac["policy_rate_hz"] = ac.get("policy_rate_hz") or 5.0
ac["adapter_rate_hz"] = ac.get("adapter_rate_hz") or 50.0
ac["controller_rate_hz"] = ac.get("controller_rate_hz") or 500.0
ac["future_runtime_adapter"]["implementation_status"] = "implemented"
m["clock_contract"]["use_sim_time"] = False
for item in m.get("fail_safe_contract", []):
    if item.get("threshold_source") == "runtime_config" and item.get("threshold_ms") is None:
        item["threshold_ms"] = 500.0
p.write_text(json.dumps(m, indent=2) + "\n")
PY

echo "SUITE_OUT=${SUITE_OUT}"
echo "EPISODE_RESULTS_PATH=${EPISODE_RESULTS_PATH}"
echo "Next: python3 ${MIDSTREAM}/training/scripts/aggregate_evaluation_summary.py --run-dir ${SUITE_OUT}"
