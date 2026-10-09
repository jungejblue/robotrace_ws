#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"

mode="${1:-bag}"
if (($#)); then shift; fi
case "$mode" in bag) sim_time=true ;; live) sim_time=false ;;
  -h|--help)
    printf '%s\n' 'Usage: ./scripts/run.sh [bag|live] [--map FILE.pcd] [--metrics FILE.csv] [--config FILE.yaml]'
    exit 0 ;;
  *) die 'First argument must be bag or live.' ;;
esac
map_path="$ROBOTRACE_ROOT/data/map_enu_v010.pcd"
metrics_path="$ROBOTRACE_ROOT/results/localization_metrics.csv"
config_path="$ROBOTRACE_ROOT/src/robotrace_localization/config/localization.yaml"
while (($#)); do
  case "$1" in
    --map|--metrics|--config)
      (($# >= 2)) || die "$1 needs a path."
      case "$1" in --map) map_path="$2" ;; --metrics) metrics_path="$2" ;; --config) config_path="$2" ;; esac
      shift 2 ;;
    *) die "Unknown argument: $1" ;;
  esac
done
map_path="$(resolve_file "$map_path")"
config_path="$(resolve_file "$config_path")"
mkdir -p -- "$(dirname -- "$metrics_path")"
metrics_path="$(realpath -m -- "$metrics_path")"
ros_workspace
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
printf 'Mode: %s\nMap: %s\nMetrics: %s\n' "$mode" "$map_path" "$metrics_path"
exec roslaunch robotrace_localization localization.launch \
  use_sim_time:="$sim_time" map_path:="$map_path" \
  config:="$config_path" metrics_path:="$metrics_path"
