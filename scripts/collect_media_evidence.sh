#!/usr/bin/env bash
# Collect raw text evidence for README media recapture.
#
# Usage:
#   source /opt/ros/jazzy/setup.bash
#   source install/setup.bash
#   bash scripts/collect_media_evidence.sh
#
# Run this while the relevant milestone launch file is active. GUI screenshots
# still need to be captured manually from MuJoCo/rqt windows.

set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${ROOT_DIR}/.media_evidence/${STAMP}"

mkdir -p "${OUT_DIR}"

run_capture() {
  local name="$1"
  shift
  local file="${OUT_DIR}/${name}.txt"

  {
    echo "$ $*"
    echo
    "$@"
  } >"${file}" 2>&1

  echo "[collect_media_evidence] wrote ${file}"
}

run_shell_capture() {
  local name="$1"
  shift
  local file="${OUT_DIR}/${name}.txt"

  {
    echo "$ $*"
    echo
    bash -lc "$*"
  } >"${file}" 2>&1

  echo "[collect_media_evidence] wrote ${file}"
}

echo "[collect_media_evidence] output: ${OUT_DIR}"

run_capture "ros2_node_list" ros2 node list
run_capture "ros2_topic_list" ros2 topic list
run_capture "ros2_control_list_controllers" ros2 control list_controllers
run_capture "ros2_control_hardware_components" ros2 control list_hardware_components
run_capture "joint_states_once" timeout 5s ros2 topic echo /joint_states --once
run_capture "safety_estop_once" timeout 5s ros2 topic echo /safety/estop --once
run_capture "safety_status_once" timeout 5s ros2 topic echo /safety/status --once
run_capture "drive_status_once" timeout 5s ros2 topic echo /servo_drive/status --once
run_capture "camera_info_once" timeout 5s ros2 topic echo /camera/color/camera_info --once
run_capture "wrist_camera_info_once" timeout 5s ros2 topic echo /camera/wrist/color/camera_info --once
run_capture "tactile_left_camera_info_once" timeout 5s ros2 topic echo /camera/tactile_left/camera_info --once
run_capture "tactile_right_camera_info_once" timeout 5s ros2 topic echo /camera/tactile_right/camera_info --once
run_capture "gripper_state_once" timeout 5s ros2 topic echo /gripper/state --once

run_shell_capture "joint_states_hz" "timeout 8s ros2 topic hz /joint_states --window 100"
run_shell_capture "sim_encoder_state_hz" "timeout 8s ros2 topic hz /sim/encoder_state --window 100"
run_shell_capture "joint_target_hz" "timeout 8s ros2 topic hz /joint_target --window 100"
run_shell_capture "heartbeat_hz" "timeout 8s ros2 topic hz /teleop/heartbeat --window 100"
run_shell_capture "camera_color_hz" "timeout 8s ros2 topic hz /camera/color/image_raw --window 50"
run_shell_capture "camera_depth_hz" "timeout 8s ros2 topic hz /camera/depth/image_raw --window 50"
run_shell_capture "wrist_camera_color_hz" "timeout 8s ros2 topic hz /camera/wrist/color/image_raw --window 50"
run_shell_capture "tactile_left_hz" "timeout 8s ros2 topic hz /camera/tactile_left/image_raw --window 50"
run_shell_capture "tactile_right_hz" "timeout 8s ros2 topic hz /camera/tactile_right/image_raw --window 50"
run_shell_capture "joint_states_delay" "timeout 8s ros2 topic delay /joint_states"

if command -v candump >/dev/null 2>&1; then
  run_shell_capture "candump_vcan0" "timeout 5s candump vcan0"
else
  echo "candump not found" >"${OUT_DIR}/candump_vcan0.txt"
  echo "[collect_media_evidence] wrote ${OUT_DIR}/candump_vcan0.txt"
fi

if [[ -d "${ROOT_DIR}/.m6_validation/episodes/episode_000000/train" ]]; then
  run_shell_capture "m6_dataset_features" \
    "python3 -c \"from datasets import load_from_disk; ds=load_from_disk('${ROOT_DIR}/.m6_validation/episodes/episode_000000/train'); print(len(ds)); print(ds.features)\""
elif [[ -d "${ROOT_DIR}/data/episodes/episode_000000/train" ]]; then
  run_shell_capture "m6_dataset_features" \
    "python3 -c \"from datasets import load_from_disk; ds=load_from_disk('${ROOT_DIR}/data/episodes/episode_000000/train'); print(len(ds)); print(ds.features)\""
else
  echo "No episode dataset found at .m6_validation/episodes/episode_000000/train or data/episodes/episode_000000/train" \
    >"${OUT_DIR}/m6_dataset_features.txt"
  echo "[collect_media_evidence] wrote ${OUT_DIR}/m6_dataset_features.txt"
fi

cat >"${OUT_DIR}/README.txt" <<EOF
Raw evidence collected at ${STAMP}.

Use these files as source material when recapturing media:
- M1: ros2_control_list_controllers.txt, joint_states_hz.txt, sim_encoder_state_hz.txt
- M2: candump_vcan0.txt, drive_status_once.txt
- M3: ros2_control_list_controllers.txt, joint_states_hz.txt
- M4: joint_target_hz.txt, heartbeat_hz.txt, joint_states_delay.txt
- M5: safety_estop_once.txt, safety_status_once.txt, candump_vcan0.txt
- M6: camera_color_hz.txt, camera_depth_hz.txt, wrist_camera_color_hz.txt, tactile_left_hz.txt, tactile_right_hz.txt, camera_info_once.txt, wrist_camera_info_once.txt, tactile_left_camera_info_once.txt, tactile_right_camera_info_once.txt, m6_dataset_features.txt
- M7: use the MuJoCo viewer/recorder output plus policy inference logs; check gripper_state_once.txt and wrist_camera_color_hz.txt before recording

GUI captures still need manual screenshots:
- rqt_graph
- rqt_plot
- rqt_robot_monitor
- rqt_image_view
- MuJoCo viewer / grasp demo recording
EOF

echo "[collect_media_evidence] done"
