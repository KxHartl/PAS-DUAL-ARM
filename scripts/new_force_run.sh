#!/usr/bin/env bash
# Prepare a fresh force-grasp run: archive the previous logs and clear the partition.
#
# The four terminal commands always write the SAME file names, so they can be
# copy-pasted unchanged. This script is what makes that safe: it moves the
# previous run out of the way first. Without it the recorder would refuse to
# overwrite its own log (it opens with 'x' on purpose) and would die on startup
# while `ros2 launch` stayed alive - a run that looks fine and records nothing.
set -uo pipefail
cd "$(dirname -- "${BASH_SOURCE[0]}")/.."

bash scripts/stop_force_isolated.sh || exit 1

shopt -s nullglob
# Every entry must be a glob, so a missing file disappears instead of becoming a
# literal path that `mv` then complains about.
previous=(log/run-t*.log log/run-force*.jsonl log/run-qualification*.json)
if ((${#previous[@]})); then
  stamp=$(date +%Y%m%d-%H%M%S)
  mkdir -p "log/archive/$stamp"
  mv "${previous[@]}" "log/archive/$stamp/"
  echo "Prethodni run spremljen u log/archive/$stamp/"
fi

echo "Spremno. Logovi ovog runa:"
printf '  %s\n' log/run-t1-sim.log log/run-t2-estimator.log log/run-t3-task.log \
  log/run-t4-grasp.log log/run-force.jsonl
