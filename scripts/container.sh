#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"

action="${1:-help}"
if (($#)); then shift; fi
container_name="${ROBOTRACE_CONTAINER:-robotrace-localization}"
image_name="${ROBOTRACE_IMAGE:-robotrace:localization-cuda}"
data_dir=""
cuda_base="nvidia/cuda:12.8.1-devel-ubuntu20.04"
while (($#)); do
  case "$1" in
    --name|--image|--data-dir|--cuda-base)
      (($# >= 2)) || die "$1 needs a value."
      case "$1" in
        --name) container_name="$2" ;; --image) image_name="$2" ;;
        --data-dir) data_dir="$2" ;; --cuda-base) cuda_base="$2" ;;
      esac
      shift 2 ;;
    *) die "Unknown argument: $1" ;;
  esac
done
case "$action" in
  help|-h|--help)
    printf '%s\n' \
      'Usage: ./scripts/container.sh build|start|shell|stop [options]' \
      'Options: --image IMAGE --name NAME --data-dir HOST_DIRECTORY --cuda-base IMAGE' \
      'Default image: robotrace:localization-cuda; default container: robotrace-localization.' \
      'start reuses that container only when its /ws mount is this workspace.' \
      '--data-dir mounts existing map/bags read-only at /ws/data.' \
      'Example: ./scripts/container.sh start --image robotrace:cuda-dev --data-dir ../robotrace_ws/data'
    exit 0 ;;
  build|start|shell|stop) ;;
  *) die "Unknown action: $action" ;;
esac
need docker
docker_command=(docker)
if ! docker info >/dev/null 2>&1; then
  need sudo
  printf '%s\n' 'Docker needs elevated host permissions; invoking sudo docker.' >&2
  docker_command=(sudo docker)
fi

if [[ "$action" == build ]]; then
  exec "${docker_command[@]}" build --build-arg CUDA_BASE="$cuda_base" \
    -f "$ROBOTRACE_ROOT/docker/Dockerfile" -t "$image_name" "$ROBOTRACE_ROOT/docker"
fi

exists=false
if "${docker_command[@]}" inspect "$container_name" >/dev/null 2>&1; then exists=true; fi
if [[ "$exists" == true ]]; then
  mount_root="$("${docker_command[@]}" inspect -f '{{range .Mounts}}{{if eq .Destination "/ws"}}{{.Source}}{{end}}{{end}}' "$container_name")"
  [[ "$mount_root" == "$ROBOTRACE_ROOT" ]] || \
    die "Container $container_name mounts a different workspace. Choose another --name; no container was modified."
  if [[ -n "$data_dir" && "$action" == start ]]; then
    [[ -d "$data_dir" ]] || die "Data directory does not exist: $data_dir"
    data_dir="$(realpath -- "$data_dir")"
    mounted_data="$("${docker_command[@]}" inspect -f '{{range .Mounts}}{{if eq .Destination "/ws/data"}}{{.Source}}{{end}}{{end}}' "$container_name")"
    [[ "$mounted_data" == "$data_dir" ]] || \
      die 'Existing container has a different data mount. Choose a new --name; mounts cannot change on start.'
    mounted_data_rw="$("${docker_command[@]}" inspect -f '{{range .Mounts}}{{if eq .Destination "/ws/data"}}{{.RW}}{{end}}{{end}}' "$container_name")"
    [[ "$mounted_data_rw" == false ]] || die 'Existing data mount is writable. Choose a new --name for the required read-only mount.'
  fi
fi

case "$action" in
  start)
    mkdir -p "$ROBOTRACE_ROOT/results/ros"
    if [[ "$exists" == true ]]; then
      "${docker_command[@]}" start "$container_name" >/dev/null
    else
      mount_args=(-v "$ROBOTRACE_ROOT:/ws")
      if [[ -n "$data_dir" ]]; then
        [[ -d "$data_dir" ]] || die "Data directory does not exist: $data_dir"
        data_dir="$(realpath -- "$data_dir")"
        mount_args+=(-v "$data_dir:/ws/data:ro")
      fi
      "${docker_command[@]}" run -dit --name "$container_name" \
        --runtime=nvidia --gpus all --network host --ipc host \
        --user "$(id -u):$(id -g)" \
        -e HOME=/tmp/robotrace-home -e ROS_LOG_DIR=/ws/results/ros \
        "${mount_args[@]}" -w /ws \
        --entrypoint /ws/docker/entrypoint.sh \
        "$image_name" bash --rcfile /ws/docker/bashrc
    fi
    printf 'Container ready: %s\nEnter: ./scripts/container.sh shell --name %s\n' "$container_name" "$container_name"
    ;;
  shell)
    [[ "$exists" == true ]] || die 'Container does not exist. Run container.sh start first.'
    exec "${docker_command[@]}" exec -it --user "$(id -u):$(id -g)" \
      -e HOME=/tmp/robotrace-home -e ROS_LOG_DIR=/ws/results/ros \
      -w /ws "$container_name" bash --rcfile /ws/docker/bashrc
    ;;
  stop)
    [[ "$exists" == true ]] || die 'Container does not exist.'
    "${docker_command[@]}" stop "$container_name"
    ;;
esac
