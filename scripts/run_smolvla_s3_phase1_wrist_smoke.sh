#!/usr/bin/env bash
# Phase-1 SmolVLA Recovery wrist-camera smoke (bounded accepted episodes).
# Collect + audit only: no release, no train, no Isaac.
#
# Re-run P1 only after a failed seed59 (keeps seed58):
#   PHASE1_POSITIONS=P1 PHASE1_EXISTING_P0=<seed58_root> \
#     bash scripts/run_smolvla_s3_phase1_wrist_smoke.sh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d)"
MIDSTREAM="${MIDSTREAM_ROOT:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab}"
RUN_TAG="${PHASE1_RUN_TAG:-wrist_smoke2}"
EPISODES_PER_POSITION="${PHASE1_EPISODES_PER_POSITION:-2}"

# Two fixed table positions for the red box (wrist FOV diversity).
# Blue/green are parked ≥0.10 m away via initial_pos_by_object — do NOT use a
# single degenerate initial_pos_range for all objects (collision ejects red).
P0_X="${PHASE1_P0_X:-0.38}"
P0_Y="${PHASE1_P0_Y:--0.10}"
P1_X="${PHASE1_P1_X:-0.42}"
P1_Y="${PHASE1_P1_Y:-0.10}"
SEED_P0="${PHASE1_SEED_P0:-58}"
SEED_P1="${PHASE1_SEED_P1:-59}"
PARK_BLUE_X="${PHASE1_PARK_BLUE_X:-0.52}"
PARK_BLUE_Y="${PHASE1_PARK_BLUE_Y:--0.14}"
PARK_GREEN_X="${PHASE1_PARK_GREEN_X:-0.52}"
PARK_GREEN_Y="${PHASE1_PARK_GREEN_Y:-0.14}"
# Comma-separated: P0,P1 (default) or P1 / P0 alone.
PHASE1_POSITIONS="${PHASE1_POSITIONS:-P0,P1}"

write_pos_yaml() {
  local out="$1"
  local seed="$2"
  local x="$3"
  local y="$4"
  cat >"$out" <<YAML
domain_randomization:
  enabled: true
  # Phase-1 wrist smoke only. Do not reuse for v3 formal collection.
  seed: ${seed}
  camera:
    scene_camera:
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
    # Per-object XY: red at target; distractors parked clear of grasp workspace.
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
  local pos_id="$1"
  local seed="$2"
  local x="$3"
  local y="$4"
  local out="data/e2_red_500hz_seed${seed}_${RUN_TAG}_${STAMP}"
  local log="evidence/e2_red_500hz_seed${seed}_${RUN_TAG}_${STAMP}"
  local yaml="${ROOT_DIR}/config/randomization_phase1_${pos_id}_${STAMP}.yaml"

  if [[ -e "${ROOT_DIR}/${out}" ]]; then
    if [[ "${PHASE1_FORCE_RECREATE:-0}" == "1" ]]; then
      echo "[phase1-wrist] PHASE1_FORCE_RECREATE=1 → removing ${out}" >&2
      rm -rf "${ROOT_DIR}/${out}"
    else
      echo "[phase1-wrist] refuse overwrite: ${ROOT_DIR}/${out} exists (set PHASE1_FORCE_RECREATE=1)" >&2
      exit 1
    fi
  fi
  mkdir -p "${ROOT_DIR}/$(dirname "${log}")"
  write_pos_yaml "${yaml}" "${seed}" "${x}" "${y}"

  echo "[phase1-wrist] position=${pos_id} seed=${seed} xy=(${x},${y}) out=${out}" >&2

  export BATCH_PREFLIGHT_OUTPUT_ROOT="${ROOT_DIR}/${out}"
  export BATCH_PREFLIGHT_LOG_DIR="${ROOT_DIR}/${log}"
  export BATCH_PREFLIGHT_SEED="${seed}"
  export BATCH_PREFLIGHT_OBJECTS=object_red_box
  export BATCH_PREFLIGHT_EPISODES="${EPISODES_PER_POSITION}"
  export BATCH_PREFLIGHT_MAX_ATTEMPTS=3
  export BATCH_PREFLIGHT_RANDOMIZE=true
  export BATCH_PREFLIGHT_HEADLESS=true
  export BATCH_PREFLIGHT_CAPTURE_MODE=portfolio
  export BATCH_PREFLIGHT_SCENE_USE_MUJOCO_RENDERER=true
  export BATCH_PREFLIGHT_ENABLE_WRIST_CAMERA=true
  export BATCH_PREFLIGHT_WRIST_USE_MUJOCO_RENDERER=true
  export BATCH_PREFLIGHT_WRIST_CAMERA_WIDTH=320
  export BATCH_PREFLIGHT_WRIST_CAMERA_HEIGHT=240
  export BATCH_PREFLIGHT_RANDOMIZATION_PATH="${yaml}"
  export BATCH_PREFLIGHT_GRASP_ASSIST=false
  export BATCH_PREFLIGHT_ENABLE_GRASP_MONITOR=false
  export BATCH_PREFLIGHT_VALIDATION_MODE=lift
  export BATCH_PREFLIGHT_CAMERA_WIDTH=320
  export BATCH_PREFLIGHT_CAMERA_HEIGHT=240
  export BATCH_PREFLIGHT_CAMERA_RATE=10.0
  # Match Round-2 grasp control recipe; only camera/wrist + fixed XY differ.
  export BATCH_PREFLIGHT_PRE_CLOSE_HOLD=3.0
  export BATCH_PREFLIGHT_CLOSE_DURATION=3.0
  export BATCH_PREFLIGHT_GRASP_PAUSE=3.0
  export BATCH_PREFLIGHT_HOVER_DURATION=4.0
  export BATCH_PREFLIGHT_HOVER_HEIGHT=0.20
  export BATCH_PREFLIGHT_DESCEND_DURATION=8.0
  export BATCH_PREFLIGHT_APPROACH_XY_DURATION=0.0
  export BATCH_PREFLIGHT_POSE_STEP_M=0.001
  export BATCH_PREFLIGHT_POSE_CMD_RATE_HZ=100.0
  export BATCH_PREFLIGHT_POSE_MAX_ACCELERATION_MPS2=0.5
  export BATCH_PREFLIGHT_LIFT_DURATION=10.0
  export BATCH_PREFLIGHT_POST_LIFT_HOLD=8.0
  export BATCH_PREFLIGHT_BATCH_TIMEOUT_S=900
  export BATCH_PREFLIGHT_DATASET_WAIT_S=90
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-93}"

  bash "${ROOT_DIR}/scripts/run_batch_preflight_smoke.sh" >&2
  echo "${ROOT_DIR}/${out}"
}

cd "${ROOT_DIR}"
echo "[phase1-wrist] positions=${PHASE1_POSITIONS} scene+wrist, no release/train"

OUT_P0="${PHASE1_EXISTING_P0:-}"
OUT_P1="${PHASE1_EXISTING_P1:-}"
IFS=',' read -r -a _POS_LIST <<< "${PHASE1_POSITIONS}"
for pos in "${_POS_LIST[@]}"; do
  pos="$(echo "${pos}" | tr -d '[:space:]')"
  case "${pos}" in
    P0) OUT_P0="$(run_position P0 "${SEED_P0}" "${P0_X}" "${P0_Y}")" ;;
    P1) OUT_P1="$(run_position P1 "${SEED_P1}" "${P1_X}" "${P1_Y}")" ;;
    *) echo "[phase1-wrist] unknown PHASE1_POSITIONS entry: ${pos}" >&2; exit 1 ;;
  esac
done

# Physical cleanup (AGENTS 8.7)
pkill -9 -f "teleop_bringup" || true
pkill -9 -f "mujoco_sim" || true
pkill -9 -f "lerobot_recorder" || true
pkill -9 -f "servo_node" || true
pkill -9 -f "ros2_control" || true
bash "${ROOT_DIR}/scripts/stop_stack.sh" >/dev/null 2>&1 || true

MANIFEST="${ROOT_DIR}/evidence/phase1_${RUN_TAG}_${STAMP}_manifest.json"
python3 - <<PY
import json
from pathlib import Path
positions = {}
if "${OUT_P0}":
    positions["P0"] = {"seed": ${SEED_P0}, "xy": [${P0_X}, ${P0_Y}], "root": "${OUT_P0}"}
if "${OUT_P1}":
    positions["P1"] = {"seed": ${SEED_P1}, "xy": [${P1_X}, ${P1_Y}], "root": "${OUT_P1}"}
manifest = {
    "phase": "smolvla_s3_recovery_phase1_wrist_smoke",
    "stamp": "${STAMP}",
    "accepted_target": len(positions) * ${EPISODES_PER_POSITION},
    "positions": positions,
    "cameras": ["observation.images.scene", "observation.images.wrist"],
    "grasp_assist_enabled": False,
    "builds_release": False,
    "triggers_train": False,
    "triggers_isaac": False,
    "park_distractors": {
        "object_blue_cylinder": [${PARK_BLUE_X}, ${PARK_BLUE_Y}],
        "object_green_sphere": [${PARK_GREEN_X}, ${PARK_GREEN_Y}],
    },
}
Path("${MANIFEST}").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
print("wrote", "${MANIFEST}")
PY

AUDIT_SOURCES=()
[[ -n "${OUT_P0}" ]] && AUDIT_SOURCES+=(--source "${OUT_P0}")
[[ -n "${OUT_P1}" ]] && AUDIT_SOURCES+=(--source "${OUT_P1}")
if [[ -d "${MIDSTREAM}" && ${#AUDIT_SOURCES[@]} -gt 0 ]]; then
  python3 "${MIDSTREAM}/training/scripts/audit_smolvla_s3_phase1_wrist_smoke.py" \
    "${AUDIT_SOURCES[@]}" \
    --expected-episodes "$(( ${#AUDIT_SOURCES[@]} / 2 * EPISODES_PER_POSITION ))" \
    --json-out "${MIDSTREAM}/runs/smolvla_s3/phase1_${RUN_TAG}_${STAMP}/wrist_smoke_audit.json" \
    || echo "[phase1-wrist] audit reported failures (see JSON); collection trees retained"
fi

echo "[phase1-wrist] done. Trees:"
[[ -n "${OUT_P0}" ]] && echo "  P0: ${OUT_P0}"
[[ -n "${OUT_P1}" ]] && echo "  P1: ${OUT_P1}"
echo "[phase1-wrist] NO release / NO train / NO Isaac"
