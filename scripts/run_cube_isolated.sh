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
exec "$script_dir/run_native.sh" "$@"
