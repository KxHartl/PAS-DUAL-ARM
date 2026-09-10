#!/usr/bin/env bash

# Reapply tracked local patches to the imported source dependencies.
#
# The source packages are pinned in ros2.repos and fetched clean with
# `vcs import src < ros2.repos`. Any intentional local modification to an
# upstream package lives as a patch under patches/ and is reapplied here, so a
# clean checkout reproduces the exact tree the workspace was built against.
#
# Each patch is applied from the root of the package it targets. Re-running this
# script is safe: an already-applied patch is detected and skipped.

set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_root"

# patch file -> package directory it applies in
declare -A patches=(
  ["patches/ros2_kortex-robotiq_2f_85-drop-isaac-args.patch"]="src/ros2_kortex"
)

for patch in "${!patches[@]}"; do
  target=${patches[$patch]}
  if [[ ! -d $target ]]; then
    printf 'skip: target %s is missing (run vcs import first)\n' "$target" >&2
    continue
  fi
  if git -C "$target" apply --check --reverse "$repo_root/$patch" 2>/dev/null; then
    printf 'already applied: %s\n' "$patch"
    continue
  fi
  if git -C "$target" apply --check "$repo_root/$patch" 2>/dev/null; then
    git -C "$target" apply "$repo_root/$patch"
    printf 'applied: %s -> %s\n' "$patch" "$target"
  else
    printf 'ERROR: %s does not apply cleanly to %s\n' "$patch" "$target" >&2
    exit 1
  fi
done

printf 'all patches reconciled\n'
