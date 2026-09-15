#!/usr/bin/env bash
# A separate partition from the user's existing cube GUI (domain 75).
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PAS_DUAL_ARM_ROS_DOMAIN_ID=76
export IGN_PARTITION=pas_dual_arm_force_76
export GZ_PARTITION=pas_dual_arm_force_76
export ROS_HOME="$script_dir/../log/force_ros_home"
mkdir -p "$ROS_HOME"
if [[ "$*" == *"sim.launch.py"* ]]; then
  # Gazebo's ruby launcher can leave its server orphaned on shutdown.
  python3 - <<'PY'
from pathlib import Path
import sys
for process in Path('/proc').iterdir():
    if not process.name.isdigit():
        continue
    try:
        cmd = (process / 'cmdline').read_bytes().replace(b'\0', b' ')
        env = (process / 'environ').read_bytes().split(b'\0')
        if cmd.startswith(b'ign gazebo ') and b'IGN_PARTITION=pas_dual_arm_force_76' in env:
            sys.exit(f'Force simulator already running (PID {process.name}); stop it before restarting.')
    except (PermissionError, FileNotFoundError, ProcessLookupError):
        pass
PY
fi
exec bash "$script_dir/run_native.sh" "$@"
