#!/usr/bin/env bash
set -eo pipefail
export HOME="${HOME:-/tmp/robotrace-home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/ws/results/ros}"
mkdir -p "$HOME" "$ROS_LOG_DIR"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
if [[ -f /ws/.build/devel/setup.bash ]]; then
  source /ws/.build/devel/setup.bash
fi
exec "$@"
