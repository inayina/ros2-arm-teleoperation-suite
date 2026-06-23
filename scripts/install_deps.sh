#!/bin/bash
# Install system + Python + ROS dependencies for the V2 teleop stack.
set -e

echo "[install_deps] ROS 2 packages (Jazzy)..."
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-forward-command-controller \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-moveit-servo \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-diagnostic-aggregator \
  ros-jazzy-realtime-tools \
  can-utils \
  linux-modules-extra-"$(uname -r)" || true

echo "[install_deps] Python packages (active environment)..."
if ! python3 -m pip install -r "$(dirname "$0")/../requirements.txt"; then
  echo "[install_deps] pip is externally managed; retrying user-site install..."
  python3 -m pip install --user --break-system-packages -r "$(dirname "$0")/../requirements.txt"
fi

echo "[install_deps] Done. Build with: colcon build && source install/setup.bash"
