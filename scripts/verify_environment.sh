#!/usr/bin/env bash

# Read-only PAS-DUAL-ARM preflight. Enter through run_native.sh so an inherited
# BATRACS/other ROS overlay cannot affect the result.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_root=$(cd -- "$script_dir/.." && pwd)

if [[ ${1:-} != --inside ]]; then
  exec "$script_dir/run_native.sh" bash "$script_dir/verify_environment.sh" --inside "$@"
fi
shift

check_live=false
while (($#)); do
  case "$1" in
    --live) check_live=true ;;
    -h|--help)
      printf 'Usage: ./scripts/verify_environment.sh [--live]\n'
      printf 'Static checks are read-only; --live also checks an already running simulation.\n'
      exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
  shift
done

passes=0
failures=0
pass() { printf '[PASS] %s\n' "$1"; passes=$((passes + 1)); }
fail() { printf '[FAIL] %s\n' "$1" >&2; failures=$((failures + 1)); }

[[ ${ROS_DISTRO:-} == humble ]] && pass 'ROS 2 Humble' || fail 'ROS_DISTRO is not humble'
[[ ${RMW_IMPLEMENTATION:-} == rmw_fastrtps_cpp ]] && pass 'Fast DDS' || fail 'Fast DDS not selected'
[[ ${ROS_DOMAIN_ID:-} == 5 ]] && pass 'ROS domain 5' || fail "ROS domain is ${ROS_DOMAIN_ID:-unset}, expected 5"
[[ ${ROS_LOCALHOST_ONLY:-} == 1 ]] && pass 'localhost discovery' || fail 'localhost discovery not selected'
[[ ${PAS_DUAL_ARM_ROOT:-} == "$project_root" ]] && pass 'project root' || fail 'wrong project root'

IFS=: read -r -a prefixes <<< "${AMENT_PREFIX_PATH:-}"
foreign=()
for prefix in "${prefixes[@]}"; do
  case "$prefix" in
    /opt/ros/humble|"$project_root"/install|"$project_root"/install/*) ;;
    *) foreign+=("$prefix") ;;
  esac
done
if ((${#foreign[@]} == 0)); then
  pass 'no foreign ROS overlays'
else
  fail "foreign ROS overlays: ${foreign[*]}"
fi

for command_name in ros2 colcon xacro ign; do
  if command -v "$command_name" >/dev/null 2>&1; then
    pass "command: $command_name"
  else
    fail "missing command: $command_name"
  fi
done

for package_name in \
  pas_dual_arm_bringup pas_dual_arm_scripts pas_dual_arm_moveit_config \
  slam_toolbox nav2_bringup ros_gz_sim controller_manager; do
  if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    pass "package: $package_name"
  else
    fail "missing package: $package_name (build or install first)"
  fi
done

if $check_live; then
  for topic_name in /clock /joint_states /scan_filtered /base_controller/odom; do
    if timeout 8 ros2 topic echo "$topic_name" --once >/dev/null 2>&1; then
      pass "live topic: $topic_name"
    else
      fail "no message on live topic: $topic_name"
    fi
  done
fi

printf '\nSummary: %d passed, %d failed\n' "$passes" "$failures"
((failures == 0))
