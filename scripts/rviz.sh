#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"

build_only=false
software_rendering=false
dev_container="${ROBOTRACE_CONTAINER:-robotrace-ws}"
container_name=robotrace-rviz
image_name=robotrace:rviz
base_image=""
config_path="$ROBOTRACE_ROOT/src/robotrace_localization/rviz/localization.rviz"
while (($#)); do
  case "$1" in
    --build) build_only=true; shift ;;
    --software-rendering) software_rendering=true; shift ;;
    --dev-container|--name|--image|--base-image|--config)
      (($# >= 2)) || die "$1 needs a value."
      case "$1" in
        --dev-container) dev_container="$2" ;;
        --name) container_name="$2" ;;
        --image) image_name="$2" ;;
        --base-image) base_image="$2" ;;
        --config) config_path="$2" ;;
      esac
      shift 2 ;;
    -h|--help)
      printf '%s\n' \
        'Usage: ./scripts/rviz.sh [--build] [options]' \
        'Run on the Ubuntu host. --build builds the image and exits.' \
        'Options: --dev-container NAME --name NAME --image IMAGE --base-image IMAGE' \
        '         --config FILE.rviz --software-rendering' \
        'Default: robotrace-ws development container; robotrace:rviz GUI image.' \
        'Uses X11/XWayland and the supplied map-frame RViz configuration.'
      exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
done
need docker
docker_command=(docker)
if ! docker info >/dev/null 2>&1; then
  need sudo
  docker_command=(sudo docker)
fi
if [[ "$build_only" == true ]]; then
  if [[ -z "$base_image" ]]; then
    if "${docker_command[@]}" inspect "$dev_container" >/dev/null 2>&1; then
      base_image="$("${docker_command[@]}" inspect -f '{{.Config.Image}}' "$dev_container")"
    else
      base_image="${ROBOTRACE_IMAGE:-robotrace:localization-cuda}"
    fi
  fi
  exec "${docker_command[@]}" build \
    --build-arg ROBOTRACE_BASE="$base_image" \
    -f "$ROBOTRACE_ROOT/docker/Dockerfile.rviz" \
    -t "$image_name" "$ROBOTRACE_ROOT/docker"
fi
[[ -n "${DISPLAY:-}" ]] || die 'DISPLAY is unset. Run from the host graphical desktop (X11/XWayland).'
[[ -d /tmp/.X11-unix ]] || die 'The X11 socket directory /tmp/.X11-unix is missing.'
need xhost
config_path="$(resolve_file "$config_path")"
"${docker_command[@]}" image inspect "$image_name" >/dev/null 2>&1 || \
  die "RViz image is missing: $image_name. Run ./scripts/rviz.sh --build first."
if "${docker_command[@]}" inspect "$container_name" >/dev/null 2>&1; then
  die "Container $container_name already exists. Close that RViz session or choose another --name."
fi

# Preserve a pre-existing grant; only revoke the grant introduced by this run.
grant_added=false
if [[ "$(xhost)" != *'SI:localuser:root'* ]]; then
  xhost +si:localuser:root >/dev/null
  grant_added=true
fi
cleanup() {
  if [[ "$grant_added" == true ]]; then xhost -si:localuser:root >/dev/null 2>&1 || true; fi
}
trap cleanup EXIT
gpu_args=(--runtime=nvidia --gpus all)
render_args=()
if [[ "$software_rendering" == true ]]; then
  gpu_args=()
  render_args=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi
"${docker_command[@]}" run --rm -it \
  --name "$container_name" --network host \
  "${gpu_args[@]}" \
  -e DISPLAY="$DISPLAY" \
  -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
  -e ROS_IP="${ROS_IP:-127.0.0.1}" \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  "${render_args[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$config_path:/opt/robotrace/localization.rviz:ro" \
  --entrypoint /bin/bash "$image_name" -lc \
  'source /opt/ros/noetic/setup.bash && exec rviz -d /opt/robotrace/localization.rviz'
