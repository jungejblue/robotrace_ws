#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"
if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
  printf '%s\n' 'Usage: ./scripts/replay.sh [data/ochang2_localization.bag] [rosbag play options]' \
    'Starts paused with /clock. Start run.sh bag in another terminal first.'
  exit 0
fi
default_bag="$ROBOTRACE_ROOT/results/ochang2_localization.bag"
if [[ ! -f "$default_bag" ]]; then
  default_bag="$ROBOTRACE_ROOT/data/ochang2_localization.bag"
fi
bag_path="${1:-$default_bag}"
if (($#)); then shift; fi
bag_path="$(resolve_file "$bag_path")"
ros_workspace
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
rosnode list >/dev/null 2>&1 || die 'ROS master is unavailable. Start ./scripts/run.sh bag first.'
printf 'Replay: %s\nPress Space only after the prior map has loaded.\n' "$bag_path"
exec rosbag play "$bag_path" --clock --pause "$@"
