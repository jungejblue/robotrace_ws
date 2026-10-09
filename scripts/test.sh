#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"
if (($#)); then
  [[ $# == 1 && ( "$1" == -h || "$1" == --help ) ]] || die 'Unknown test option.'
  printf '%s\n' 'Usage: ./scripts/test.sh' \
    'Runs Python offline tests and the two standalone C++ geometry/GNSS tests.' \
    'Requires python3, g++-10 and Eigen3; does not require roscore or a GPU.'
  exit 0
fi
need python3
need g++-10
eigen_include="${EIGEN_INCLUDE:-/usr/include/eigen3}"
[[ -f "$eigen_include/Eigen/Core" ]] || die 'Install libeigen3-dev or set EIGEN_INCLUDE to an Eigen3 include directory.'
[[ ! -L "$ROBOTRACE_ROOT/.build" ]] || die '.build must be a directory, not a symlink.'
test_dir="$ROBOTRACE_ROOT/.build/tests"
mkdir -p "$test_dir"
cd "$ROBOTRACE_ROOT"
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
for test_name in gnss_seed_test pose_math_test; do
  g++-10 -std=c++14 -O2 -Wall -Wextra -pedantic \
    -I "$ROBOTRACE_ROOT/src/robotrace_localization/include" -I "$eigen_include" \
    "$ROBOTRACE_ROOT/tests/$test_name.cpp" -o "$test_dir/$test_name"
  "$test_dir/$test_name"
done
printf '\nOffline tests passed. Full CUDA/ROS build and bag validation are separate checks.\n'
