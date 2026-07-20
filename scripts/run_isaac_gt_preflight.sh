#!/usr/bin/env bash
# Preflight: run 2–3 Isaac seeds and verify GT gripper_state min ~= report state[7] min.
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly MIDSTREAM="${MIDSTREAM:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab}"
readonly CHECKPOINT="${CHECKPOINT:-${MIDSTREAM}/data/e2_500hz_act_random30_descend_conservative_5epoch_20260719/checkpoint.pt}"
readonly PREFLIGHT_OUT="${PREFLIGHT_OUT:-${MIDSTREAM}/evidence/e3_gt_preflight_v1_$(date +%Y%m%d_%H%M%S)}"
readonly SEEDS="${SEEDS:-2100 2101 2102}"
readonly TOL="${GRIPPER_MIN_TOL:-0.05}"

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
export PREGRASP_WARMSTART=false
export ARM_COMMAND_MODE=position
export DRY_RUN=false
export RECORD_SCENE_VIDEO=true
export REQUIRE_REPORT_PASS=false
export CHECKPOINT
export EVALUATION_RUN_ID="${EVALUATION_RUN_ID:-e3_gt_preflight_v1}"
export EVALUATION_MODEL_ID="${EVALUATION_MODEL_ID:-e2_500hz_act_random30_descend_conservative_5epoch_20260719}"
export EPISODE_RESULTS_PATH="${PREFLIGHT_OUT}/episode_results.jsonl"

mkdir -p "${PREFLIGHT_OUT}/seeds"
: > "${EPISODE_RESULTS_PATH}"

nuke() {
  "${REPO_ROOT}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
  pkill -9 -f '[i]saac_panda_backend.py' 2>/dev/null || true
  pkill -9 -f '[p]olicy_inference_node' 2>/dev/null || true
  pkill -9 -f '[i]saac_scene_video_recorder' 2>/dev/null || true
  pkill -9 -f '[i]saac_continuous_gt_recorder' 2>/dev/null || true
  sleep 2
}

episode_index=0
pass_count=0
fail_count=0
for seed in ${SEEDS}; do
  seed_dir="${PREFLIGHT_OUT}/seeds/seed_${seed}"
  mkdir -p "${seed_dir}"
  echo "========== [gt-preflight] seed=${seed} =========="
  nuke
  export ROS_DOMAIN_ID=$((100 + episode_index))
  export OBJECT_SEED="${seed}"
  export EVALUATION_SEED="${seed}"
  export EVALUATION_EPISODE_INDEX="${episode_index}"
  set +e
  bash "${REPO_ROOT}/scripts/run_isaac_act_smoke.sh" "${seed_dir}"
  status=$?
  set -e
  echo "${status}" > "${seed_dir}/smoke_exit_code.txt"

  set +e
  python3 - "${seed_dir}" "${TOL}" <<'PY'
import json, sys
from pathlib import Path
seed_dir = Path(sys.argv[1])
tol = float(sys.argv[2])
report_path = seed_dir / "report.json"
gt_log = seed_dir / "gt_runtime.log"
if not report_path.is_file():
    print(f"PREFLIGHT_FAIL missing report.json in {seed_dir}")
    raise SystemExit(2)
report = json.loads(report_path.read_text())
acts = report.get("actions") or []
state_min = min(
    (float(a["state"][7]) for a in acts if len(a.get("state") or []) > 7),
    default=None,
)
cmd_min = min((float(a["bounded_action"][6]) for a in acts), default=None)
gt = {}
if gt_log.is_file():
    for line in gt_log.read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            gt[k.strip()] = v.strip()
eval_state = gt.get("min_gripper_state")
eval_cmd = gt.get("min_gripper_command")
state_count = int(float(gt.get("gripper_state_count") or 0))
ft_count = int(float(gt.get("ft_count") or 0))
cmd_count = int(float(gt.get("gripper_cmd_count") or 0))

def as_float(x):
    if x is None or x == "None":
        return None
    return float(x)

eval_state_f = as_float(eval_state)
eval_cmd_f = as_float(eval_cmd)
ok = True
reasons = []
if state_min is None:
    ok = False
    reasons.append("report missing state[7]")
if eval_state_f is None:
    ok = False
    reasons.append("gt min_gripper_state missing")
if state_count <= 0:
    ok = False
    reasons.append("gripper_state_count==0")
if cmd_count <= 0:
    ok = False
    reasons.append("gripper_cmd_count==0")
if ft_count <= 0:
    ok = False
    reasons.append("ft_count==0 (no /ft_sensor samples)")
if state_min is not None and eval_state_f is not None:
    delta = abs(state_min - eval_state_f)
    if delta > tol:
        ok = False
        reasons.append(
            f"|min_gripper_state({eval_state_f})-report_state7({state_min})|={delta} > tol={tol}"
        )
if state_min is not None and state_min <= 0.12 and (eval_state_f is None or eval_state_f > 0.12):
    ok = False
    reasons.append("report closed but evaluator min_gripper_state did not")
if cmd_min is not None and cmd_min <= 0.12 and (eval_cmd_f is None or eval_cmd_f > 0.12):
    ok = False
    reasons.append("report cmd closed but evaluator min_gripper_command did not")

payload = {
    "seed_dir": str(seed_dir),
    "ok": ok,
    "report_status": report.get("status"),
    "report_state7_min": state_min,
    "report_cmd_gripper_min": cmd_min,
    "gt_min_gripper_state": eval_state_f,
    "gt_min_gripper_command": eval_cmd_f,
    "gripper_state_count": state_count,
    "gripper_cmd_count": cmd_count,
    "ft_count": ft_count,
    "reasons": reasons,
}
(seed_dir / "gt_preflight_check.json").write_text(json.dumps(payload, indent=2) + "\n")
print("PREFLIGHT_CHECK=" + json.dumps(payload))
raise SystemExit(0 if ok else 3)
PY
  check_status=$?
  set -e
  if [[ "${check_status}" -eq 0 ]]; then
    pass_count=$((pass_count + 1))
    echo "[gt-preflight] PASS seed=${seed}"
  else
    fail_count=$((fail_count + 1))
    echo "[gt-preflight] FAIL seed=${seed} code=${check_status}"
  fi
  nuke
  episode_index=$((episode_index + 1))
done

python3 - "${PREFLIGHT_OUT}" "${pass_count}" "${fail_count}" <<'PY'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
summary = {
    "artifact_type": "e3_gt_preflight_v1",
    "pass_count": int(sys.argv[2]),
    "fail_count": int(sys.argv[3]),
    "gate": "evaluator min_gripper_state ~= report state[7] min; ft_count>0",
    "ready_for_nominal20": int(sys.argv[2]) >= 2 and int(sys.argv[3]) == 0,
}
(out / "preflight_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
raise SystemExit(0 if summary["ready_for_nominal20"] else 1)
PY

echo "PREFLIGHT_OUT=${PREFLIGHT_OUT}"
