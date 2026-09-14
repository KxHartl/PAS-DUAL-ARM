#!/usr/bin/env bash

# Run this worktree without joining the main workspace's ROS or Gazebo traffic.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_root=$(cd -- "$script_dir/.." && pwd)

export PAS_DUAL_ARM_ROS_DOMAIN_ID=75
export IGN_PARTITION=pas_dual_arm_cube_75
export GZ_PARTITION=pas_dual_arm_cube_75
export ROS_HOME="$workspace_root/log/cube_ros_home"

mkdir -p "$ROS_HOME"

# Refuse to start a SECOND simulator in this partition. Two Gazebo servers on
# one ROS domain both publish /clock, so time jumps back and forth between
# them: RViz logs "Detected jump back in time" hundreds of times a second,
# resets on every one, and eventually dies on "Cannot create GL vertex buffer".
# The cause is invisible from the symptom, so catch it here instead.
if [[ "$*" == *sim.launch.py* ]]; then
  for pid_dir in /proc/[0-9]*; do
    pid=${pid_dir#/proc/}
    [[ -r "$pid_dir/environ" ]] || continue
    grep -qz "IGN_PARTITION=$IGN_PARTITION" "$pid_dir/environ" 2>/dev/null || continue
    grep -qa "ign gazebo" "$pid_dir/cmdline" 2>/dev/null || continue
    echo "GRESKA: simulacija vec radi u particiji $IGN_PARTITION (pid $pid)." >&2
    echo "Dvije simulacije na istoj domeni ruse RViz preko /clock-a." >&2
    echo "Ugasi je, ili pokreni: bash scripts/clean_ros.sh" >&2
    exit 1
  done
fi

exec "$script_dir/run_native.sh" "$@"
