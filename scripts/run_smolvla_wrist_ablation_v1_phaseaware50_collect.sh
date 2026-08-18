#!/usr/bin/env bash
# Wrist ablation v1: matched dual-camera phaseaware50 collection (50 accepted).
# scene + H_knuckle_z05 wrist RGB. No tactile / depth / third camera.
# 36 train + 4 validation + 10 benchmark (P4 OOD). No train / no Isaac.
#
# Does not overwrite historical v3 trees (different output prefix).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="${WRIST_ABLATION_STAMP:-$(date +%Y%m%d)}"
MIDSTREAM="${MIDSTREAM_ROOT:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab}"
CFG="${MIDSTREAM}/configs/smolvla_s3/wrist_ablation_v1_phaseaware50.yaml"
RUN_TAG="${WRIST_ABLATION_RUN_TAG:-wrist_ablation_v1}"

PARK_BLUE_X=0.52
PARK_BLUE_Y=-0.14
PARK_GREEN_X=0.52
PARK_GREEN_Y=0.14

write_pos_yaml() {
  local out="$1" seed="$2" x="$3" y="$4"
  cat >"$out" <<YAML
domain_randomization:
  enabled: true
  seed: ${seed}
  camera:
    scene_camera:
      pos_noise: [0.0, 0.0]
      rot_noise: [0.0, 0.0]
    wrist_camera:
      pos_noise: [0.0, 0.0]
      rot_noise: [0.0, 0.0]
  object:
    sphere_initial_z: 0.03
    mass_range: [0.04, 0.04]
    friction_range: [2.2, 2.2]
    box_initial_z: 0.025
    cylinder_initial_z: 0.03
    sphere_friction_min: 4.0
    sphere_friction_max: 6.0
    initial_pos_by_object:
      object_red_box: [${x}, ${y}]
      object_blue_cylinder: [${PARK_BLUE_X}, ${PARK_BLUE_Y}]
      object_green_sphere: [${PARK_GREEN_X}, ${PARK_GREEN_Y}]
    yaw_range_deg_by_object:
      object_red_box: [0.0, 0.0]
      object_blue_cylinder: [0.0, 0.0]
      object_green_sphere: [0.0, 0.0]
  lighting:
    key:
      diffuse_noise: [0.0, 0.0]
YAML
}

run_position() {
  local pos_id="$1" seed="$2" x="$3" y="$4"
  local out="data/e2_red_500hz_seed${seed}_${RUN_TAG}_${pos_id}_phaseaware10_${STAMP}"
  local log="evidence/e2_red_500hz_seed${seed}_${RUN_TAG}_${pos_id}_phaseaware10_${STAMP}"
  local yaml="${ROOT_DIR}/config/randomization_${RUN_TAG}_${pos_id}_${STAMP}.yaml"

  if [[ -e "${ROOT_DIR}/${out}" ]]; then
    if [[ "${WRIST_ABLATION_FORCE_RECREATE:-0}" == "1" ]]; then
      rm -rf "${ROOT_DIR}/${out}"
    else
      echo "[wrist-ablation-collect] refuse overwrite: ${ROOT_DIR}/${out}" >&2
      exit 1
    fi
  fi
  write_pos_yaml "${yaml}" "${seed}" "${x}" "${y}"
  echo "[wrist-ablation-collect] ${pos_id} seed=${seed} xy=(${x},${y}) -> ${out}" >&2

  export BATCH_PREFLIGHT_OUTPUT_ROOT="${ROOT_DIR}/${out}"
  export BATCH_PREFLIGHT_LOG_DIR="${ROOT_DIR}/${log}"
  export BATCH_PREFLIGHT_SEED="${seed}"
  export BATCH_PREFLIGHT_OBJECTS=object_red_box
  export BATCH_PREFLIGHT_EPISODES=10
  export BATCH_PREFLIGHT_MAX_ATTEMPTS=14
  export BATCH_PREFLIGHT_RANDOMIZE=true
  export BATCH_PREFLIGHT_HEADLESS=true
  export BATCH_PREFLIGHT_CAPTURE_MODE=portfolio
  export BATCH_PREFLIGHT_SCENE_USE_MUJOCO_RENDERER=true
  export BATCH_PREFLIGHT_ENABLE_WRIST_CAMERA=true
  export BATCH_PREFLIGHT_WRIST_USE_MUJOCO_RENDERER=true
  export BATCH_PREFLIGHT_WRIST_CAMERA_WIDTH=320
  export BATCH_PREFLIGHT_WRIST_CAMERA_HEIGHT=240
  export BATCH_PREFLIGHT_ENABLE_TACTILE=false
  export BATCH_PREFLIGHT_RANDOMIZATION_PATH="${yaml}"
  export BATCH_PREFLIGHT_GRASP_ASSIST=false
  export BATCH_PREFLIGHT_ENABLE_GRASP_MONITOR=false
  export BATCH_PREFLIGHT_VALIDATION_MODE=lift
  export BATCH_PREFLIGHT_CAMERA_WIDTH=320
  export BATCH_PREFLIGHT_CAMERA_HEIGHT=240
  export BATCH_PREFLIGHT_CAMERA_RATE=10.0
  # Match v3 phase-aware timing (not Phase-1 smoke's longer Round-2 holds).
  export BATCH_PREFLIGHT_PRE_CLOSE_HOLD=0.4
  export BATCH_PREFLIGHT_CLOSE_DURATION=0.8
  export BATCH_PREFLIGHT_GRASP_PAUSE=0.5
  export BATCH_PREFLIGHT_HOVER_DURATION=3.0
  export BATCH_PREFLIGHT_HOVER_HEIGHT=0.20
  export BATCH_PREFLIGHT_DESCEND_DURATION=6.0
  export BATCH_PREFLIGHT_APPROACH_XY_DURATION=0.0
  export BATCH_PREFLIGHT_POSE_STEP_M=0.001
  export BATCH_PREFLIGHT_POSE_CMD_RATE_HZ=100.0
  export BATCH_PREFLIGHT_POSE_MAX_ACCELERATION_MPS2=0.5
  export BATCH_PREFLIGHT_LIFT_DURATION=8.0
  export BATCH_PREFLIGHT_POST_LIFT_HOLD=4.0
  export BATCH_PREFLIGHT_BATCH_TIMEOUT_S=3600
  export BATCH_PREFLIGHT_DATASET_WAIT_S=120
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-95}"

  # Preflight logs go to stderr so callers can capture only the dataset path.
  bash "${ROOT_DIR}/scripts/run_batch_preflight_smoke.sh" >&2
  echo "${ROOT_DIR}/${out}"
}

cd "${ROOT_DIR}"
mapfile -t POS_ROWS < <(python3 - "$CFG" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
assert cfg["enable_wrist_camera"] is True
assert cfg["enable_tactile"] is False
assert cfg["cameras"] == ["scene", "wrist"]
for pid, row in cfg["positions"].items():
    x, y = row["xy"]
    print(f"{pid} {row['seed']} {x} {y}")
PY
)

declare -A OUTS
POSITIONS_FILTER="${WRIST_ABLATION_POSITIONS:-P0,P1,P2,P3,P4}"
for row in "${POS_ROWS[@]}"; do
  read -r pid seed x y <<<"${row}"
  if [[ ",${POSITIONS_FILTER}," != *",${pid},"* ]]; then
    continue
  fi
  run_position "$pid" "$seed" "$x" "$y" >/dev/null
  OUTS["$pid"]="${ROOT_DIR}/data/e2_red_500hz_seed${seed}_${RUN_TAG}_${pid}_phaseaware10_${STAMP}"
  if [[ ! -d "${OUTS[$pid]}" ]]; then
    echo "[wrist-ablation-collect] missing dataset tree: ${OUTS[$pid]}" >&2
    exit 1
  fi
done

pkill -9 -f "teleop_bringup" || true
pkill -9 -f "mujoco_sim" || true
pkill -9 -f "lerobot_recorder" || true
pkill -9 -f "servo_node" || true
pkill -9 -f "ros2_control" || true
bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true

MAP_JSON="${ROOT_DIR}/evidence/${RUN_TAG}_phaseaware50_${STAMP}_position_map.json"
python3 - <<PY
import json
from pathlib import Path
outs = {
$(for pid in P0 P1 P2 P3 P4; do
  if [[ -n "${OUTS[$pid]:-}" ]]; then
    printf '  "%s": "%s",\n' "$(basename "${OUTS[$pid]}")" "$pid"
  fi
done)
}
path = Path("${MAP_JSON}")
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps(outs, indent=2) + "\n", encoding="utf-8")
print("wrote", path)
print(json.dumps(outs, indent=2))
PY

if [[ ! -d "${MIDSTREAM}" ]]; then
  echo "[wrist-ablation-collect] midstream missing: ${MIDSTREAM}" >&2
  exit 2
fi

SRC_ARGS=()
EXPECTED=0
for pid in P0 P1 P2 P3 P4; do
  if [[ -n "${OUTS[$pid]:-}" ]]; then
    SRC_ARGS+=(--source "${OUTS[$pid]}")
    EXPECTED=$((EXPECTED + 10))
  fi
done
if [[ ${#SRC_ARGS[@]} -eq 0 ]]; then
  echo "[wrist-ablation-collect] no sources collected" >&2
  exit 2
fi

QA_DIR="${MIDSTREAM}/runs/smolvla_wrist_ablation_v1/phaseaware50_${STAMP}"
mkdir -p "${QA_DIR}"
python3 "${MIDSTREAM}/training/scripts/audit_smolvla_s3_phaseaware_dataset.py" \
  "${SRC_ARGS[@]}" \
  --config "${CFG}" \
  --json-out "${QA_DIR}/phaseaware_qa.json"

python3 "${MIDSTREAM}/training/scripts/audit_smolvla_s3_phase1_wrist_smoke.py" \
  "${SRC_ARGS[@]}" \
  --expected-episodes "${EXPECTED}" \
  --json-out "${QA_DIR}/wrist_visual_qa.json"

echo "[wrist-ablation-collect] QA PASS. NO train / NO Isaac."
echo "[wrist-ablation-collect] Next: prepare immutable parent release if still authorized."
for pid in P0 P1 P2 P3 P4; do
  [[ -n "${OUTS[$pid]:-}" ]] && echo "  ${pid}: ${OUTS[$pid]}"
done
echo "  position_map: ${MAP_JSON}"
echo "  qa_dir: ${QA_DIR}"
