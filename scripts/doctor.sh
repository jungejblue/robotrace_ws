#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"
ros_workspace
need nvidia-smi
need nvcc
printf 'Workspace: %s\n' "$ROBOTRACE_ROOT"
printf '%s\n' 'GPU:'
nvidia-smi --query-gpu=name,compute_cap,driver_version --format=csv
printf '%s\n' 'CUDA toolkit:'
nvcc --version | tail -n 2
printf '%s\n' 'ROS packages:'
for package_name in fast_lio fast_gicp robotrace_localization; do
  printf '%s: ' "$package_name"
  rospack find "$package_name"
done
for item in \
  "$ROBOTRACE_ROOT/.build/devel/lib/libfast_vgicp_cuda.so" \
  "$ROBOTRACE_ROOT/.build/devel/lib/fast_lio/fastlio_mapping" \
  "$ROBOTRACE_ROOT/.build/devel/lib/robotrace_localization/gnss_initializer_node" \
  "$ROBOTRACE_ROOT/.build/devel/lib/robotrace_localization/gpu_map_localizer_node"; do
  [[ -f "$item" ]] || die "Build output is missing: $item"
  printf 'Present: %s\n' "$item"
  deps="$(ldd "$item")"
  if grep -q 'not found' <<< "$deps"; then
    printf '%s\n' "$deps" >&2
    die 'A shared library is unavailable.'
  fi
done
if command -v cuobjdump >/dev/null 2>&1; then
  printf '%s\n' 'CUDA device binaries:'
  cuobjdump --list-elf "$ROBOTRACE_ROOT/.build/devel/lib/libfast_vgicp_cuda.so"
fi
printf '\nPreflight passed. This does not prove registration accuracy or runtime throughput.\n'
