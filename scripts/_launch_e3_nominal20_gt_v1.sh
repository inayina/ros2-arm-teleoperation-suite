#!/usr/bin/env bash
set -euo pipefail
MID=/home/ina/robot-sim-lab/robot-arm-episode-data-lab
UP=/home/ina/dev/ros2-arm-teleoperation-suite
SUITE_OUT="${MID}/evidence/e3_nominal20_home_30ep_gt_v1_20260719"
mkdir -p "${SUITE_OUT}"
pkill -9 -f '[i]saac_panda_backend.py' 2>/dev/null || true
pkill -9 -f '[p]olicy_inference_node' 2>/dev/null || true
"${UP}/scripts/stop_stack.sh" >/dev/null 2>&1 || true
sleep 2
cd "${UP}"
export CHECKPOINT="${MID}/data/e2_500hz_act_random30_descend_conservative_5epoch_20260719/checkpoint.pt"
export RELEASE="${MID}/data/releases/e2_500hz_random30_descend_20260719"
export SUITE_OUT
export GT_PREFLIGHT_SUMMARY="${MID}/evidence/e3_gt_preflight_v1_20260719/preflight_summary.json"
export INFERENCE_RATE_HZ=5.0
exec bash scripts/run_isaac_nominal_suite.sh
