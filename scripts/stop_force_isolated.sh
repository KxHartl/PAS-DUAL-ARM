#!/usr/bin/env bash
# Stop everything belonging to the isolated force-grasp experiment.
# With --check it only reports what is still alive and changes nothing.
#
# Killing `ros2 launch` is not enough: its node processes survive as orphans and
# keep talking on the same partition, so the next run sees every node twice and
# reports "more than one action server". Select by the partition each process
# was started with instead of by command name - that also guarantees the user's
# own GUI session on domain 75 is never touched.
set -uo pipefail

partition=${IGN_PARTITION:-pas_dual_arm_force_76}
check_only=false
[ "${1:-}" = "--check" ] && check_only=true

mine() {
  python3 - "$partition" <<'PY'
from pathlib import Path
import os, sys
needle = f'IGN_PARTITION={sys.argv[1]}'.encode()
for process in Path('/proc').iterdir():
    if not process.name.isdigit() or int(process.name) == os.getpid():
        continue
    try:
        if needle in (process / 'environ').read_bytes().split(b'\0'):
            print(process.name)
    except (PermissionError, FileNotFoundError, ProcessLookupError):
        pass
PY
}

if $check_only; then
  pids=$(mine)
  if [ -z "$pids" ]; then
    echo "Partition $partition is clear."
    exit 0
  fi
  echo "Still alive in partition $partition:"
  ps -o pid,etimes,args -p "$(echo "$pids" | tr '\n' ',' | sed 's/,$//')"
  exit 1
fi

for signal in INT INT TERM KILL; do
  pids=$(mine)
  [ -z "$pids" ] && break
  echo "$pids" | xargs -r kill -"$signal" 2>/dev/null || true
  for _ in $(seq 20); do
    [ -z "$(mine)" ] && break
    sleep 1
  done
done

remaining=$(mine)
if [ -n "$remaining" ]; then
  echo "Still alive in partition $partition:" >&2
  ps -o pid,args -p $(echo "$remaining" | tr '\n' ',' | sed 's/,$//') >&2
  exit 1
fi
echo "Partition $partition is clear."
