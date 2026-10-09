#!/usr/bin/env bash
# Internal helpers. Public scripts execute in their own shell, not through source.
set -euo pipefail
ROBOTRACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "Required command is missing: $1"; }

ros_underlay() {
  [[ -f /opt/ros/noetic/setup.bash ]] || die 'Use the Ubuntu 20.04 / ROS Noetic CUDA container.'
  # Discard inherited catkin overlay chains before establishing this workspace.
  unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
  export CMAKE_PREFIX_PATH=/opt/ros/noetic
}

ros_workspace() {
  ros_underlay
  [[ -f "$ROBOTRACE_ROOT/.build/devel/setup.bash" ]] || die 'Run ./scripts/build.sh first.'
  set +u
  source "$ROBOTRACE_ROOT/.build/devel/setup.bash"
  set -u
  export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROBOTRACE_ROOT/results/ros}"
  mkdir -p "$ROS_LOG_DIR"
}

resolve_file() {
  local candidate="$1"
  [[ -f "$candidate" ]] || die "File does not exist: $candidate"
  realpath -- "$candidate"
}
