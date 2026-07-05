#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [ -f "${ROS2_WS_ROOT:-/ws}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${ROS2_WS_ROOT:-/ws}/install/setup.bash"
fi

export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:${PATH}"
unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV

exec "$@"
