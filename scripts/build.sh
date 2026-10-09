#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"

# CMake stores -D variables with STRING/UNINITIALIZED/FILEPATH types. Compiler
# identity depends on the value and resolved executable, not that cache type.
canonical_compiler_path() {
  local candidate="$1" cache_directory="${2:-$PWD}"
  local resolved
  [[ -n "$candidate" && "$candidate" != *-NOTFOUND ]] || return 1
  if [[ "$candidate" != */* ]]; then
    candidate="$(command -v -- "$candidate")" || return 1
  elif [[ "$candidate" != /* ]]; then
    candidate="$cache_directory/$candidate"
  fi
  resolved="$(realpath -e -- "$candidate" 2>/dev/null)" || return 1
  [[ -f "$resolved" && -x "$resolved" ]] || return 1
  printf '%s\n' "$resolved"
}

cache_can_retry_before_compiler_detection() {
  local directory="$(dirname -- "$1")" state_file
  [[ ! -f "$directory/Makefile" && ! -f "$directory/build.ninja" &&
     ! -f "$directory/CMakeFiles/Makefile.cmake" ]] || return 1
  # Missing compiler entries are safe to retry only before language detection.
  # Once compiler state exists, an absent cache entry is unexplained corruption.
  for state_file in "$directory"/CMakeFiles/*/CMakeCCompiler.cmake \
                    "$directory"/CMakeFiles/*/CMakeCXXCompiler.cmake; do
    [[ ! -f "$state_file" ]] || return 1
  done
  return 0
}

check_cached_compiler() {
  local cache_file="$1" key="$2" expected="$3" allow_missing="${4:-false}"
  local line value actual_real expected_real
  local entries=()
  while IFS= read -r line || [[ -n "$line" ]]; do
    case "$line" in "$key":*=*) entries+=("$line") ;; esac
  done < "$cache_file"
  if ((${#entries[@]} == 0)); then
    if [[ "$allow_missing" == true ]]; then
      printf 'Cache has no %s before compiler detection; retrying with %s.\n' "$key" "$expected"
      return 0
    fi
    die "Cache has no $key entry: $cache_file. Compiler state is not safely reusable; run ./scripts/clean.sh --yes."
  fi
  if ((${#entries[@]} != 1)); then
    printf 'Cached compiler entries:\n%s\n' "${entries[*]}" >&2
    die "Cache contains duplicate $key entries. Run ./scripts/clean.sh --yes."
  fi
  line="${entries[0]}"
  value="${line#*=}"
  expected_real="$(canonical_compiler_path "$expected")" || \
    die "Expected compiler is unavailable: $expected"
  if ! actual_real="$(canonical_compiler_path "$value" "$(dirname -- "$cache_file")")"; then
    printf 'Cached compiler entry: %s\nExpected compiler: %s -> %s\n' \
      "$line" "$expected" "$expected_real" >&2
    die "Cached $key does not resolve to an executable. Run ./scripts/clean.sh --yes."
  fi
  if [[ "$actual_real" != "$expected_real" ]]; then
    printf 'Cached compiler entry: %s\nCached resolved path: %s\nExpected compiler: %s -> %s\n' \
      "$line" "$actual_real" "$expected" "$expected_real" >&2
    die "Cached $key differs. Run ./scripts/clean.sh --yes."
  fi
}

# Unit checks may source the helper functions without invoking ROS or a build.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then return; fi

jobs=1
cuda_arch="${CUDA_ARCH:-120}"
while (($#)); do
  case "$1" in
    --jobs) (($# >= 2)) || die '--jobs needs an integer'; jobs="$2"; shift 2 ;;
    --cuda-arch) (($# >= 2)) || die '--cuda-arch needs an integer'; cuda_arch="$2"; shift 2 ;;
    -h|--help)
      printf '%s\n' 'Usage: ./scripts/build.sh [--jobs 1] [--cuda-arch 120]' \
        'CUDA_ARCH may also set the architecture. RTX 5060 Ti: 120; Orin: 87.' \
        'Changing the compiler, toolkit or architecture requires ./scripts/clean.sh --yes.'
      exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || die '--jobs must be a positive integer.'
[[ "$cuda_arch" =~ ^[1-9][0-9]{1,2}$ ]] || die '--cuda-arch must be an integer such as 120 or 87.'

ros_underlay
for command_name in catkin_make cmake nvcc gcc-10 g++-10; do need "$command_name"; done
[[ "$(gcc-10 -dumpversion | cut -d. -f1)" == 10 ]] || die 'gcc-10 is not GCC 10.'
[[ "$(g++-10 -dumpversion | cut -d. -f1)" == 10 ]] || die 'g++-10 is not GCC 10.'
[[ ! -L "$ROBOTRACE_ROOT/.build" ]] || die '.build must be a directory, not a symlink.'

build_dir="$ROBOTRACE_ROOT/.build/catkin"
devel_dir="$ROBOTRACE_ROOT/.build/devel"
cache="$build_dir/CMakeCache.txt"
toolchain="$(printf 'CC=%s\nCXX=%s\nCUDA_ARCH=%s\nCUDA=%s\n' \
  "$(realpath /usr/bin/gcc-10)" "$(realpath /usr/bin/g++-10)" "$cuda_arch" \
  "$(nvcc --version | sed -n 's/.*release \([^,]*\), \(V[^ ]*\).*/\1 \2/p')")"

if [[ -f "$cache" ]]; then
  [[ -f "$build_dir/robotrace-toolchain.txt" ]] || die 'Unknown cached build. Run ./scripts/clean.sh --yes.'
  if [[ "$(cat "$build_dir/robotrace-toolchain.txt")" != "$toolchain" ]]; then
    printf 'Cached toolchain:\n%s\nExpected toolchain:\n%s\n' \
      "$(cat "$build_dir/robotrace-toolchain.txt")" "$toolchain" >&2
    die 'Compiler, CUDA toolkit or architecture changed. Run ./scripts/clean.sh --yes, then rebuild.'
  fi
  allow_missing=false
  if cache_can_retry_before_compiler_detection "$cache"; then allow_missing=true; fi
  check_cached_compiler "$cache" CMAKE_C_COMPILER /usr/bin/gcc-10 "$allow_missing"
  check_cached_compiler "$cache" CMAKE_CXX_COMPILER /usr/bin/g++-10 "$allow_missing"
  check_cached_compiler "$cache" CUDA_HOST_COMPILER /usr/bin/gcc-10 "$allow_missing"
fi
mkdir -p "$build_dir" "$devel_dir"
printf '%s\n' "$toolchain" > "$build_dir/robotrace-toolchain.txt"
printf 'Workspace: %s\nBuild: %s\nDevel: %s\nCUDA architecture: sm_%s\nJobs: %s\n' \
  "$ROBOTRACE_ROOT" "$build_dir" "$devel_dir" "$cuda_arch" "$jobs"

cd "$ROBOTRACE_ROOT"
catkin_make --build "$build_dir" \
  -DCATKIN_DEVEL_PREFIX="$devel_dir" \
  -DCMAKE_INSTALL_PREFIX="$build_dir/install" \
  -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/gcc-10 \
  -DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/g++-10 \
  -DCUDA_HOST_COMPILER:FILEPATH=/usr/bin/gcc-10 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 \
  -DBUILD_VGICP_CUDA=ON -DBUILD_apps=OFF -DBUILD_test=OFF \
  -DROBOTRACE_CUDA_ARCH="$cuda_arch" \
  -j"$jobs" -l"$jobs" 2>&1 | tee "$ROBOTRACE_ROOT/.build/build.log"

printf '\nBuild complete. Next: ./scripts/doctor.sh\n'
