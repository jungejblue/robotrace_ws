#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"
if [[ "${1:-}" != --yes || $# != 1 ]]; then
  printf 'Removes only %s/.build (compiled files and build cache).\n' "$ROBOTRACE_ROOT"
  printf '%s\n' 'Source files, data, results and every other workspace remain intact.' \
    'Usage: ./scripts/clean.sh --yes'
  exit 0
fi
[[ ! -L "$ROBOTRACE_ROOT/.build" ]] || die 'Refusing to remove a symlink named .build.'
rm -rf -- "$ROBOTRACE_ROOT/.build"
printf 'Removed: %s/.build\n' "$ROBOTRACE_ROOT"
