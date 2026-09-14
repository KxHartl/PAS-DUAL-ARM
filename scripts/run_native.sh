#!/usr/bin/env bash

# Enter or run a command in the declared PAS-DUAL-ARM native ROS environment.
# A project shell is a process boundary: exit it before entering another ROS
# project instead of sourcing another workspace on top of this one.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_root=$(cd -- "$script_dir/.." && pwd)

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset \
  AMENT_PREFIX_PATH \
  CMAKE_MODULE_PATH \
  CMAKE_PREFIX_PATH \
  COLCON_PREFIX_PATH \
  CPATH \
  C_INCLUDE_PATH \
  CPLUS_INCLUDE_PATH \
  CYCLONEDDS_URI \
  FASTDDS_DEFAULT_PROFILES_FILE \
  FASTRTPS_DEFAULT_PROFILES_FILE \
  GAZEBO_MODEL_PATH \
  GAZEBO_PLUGIN_PATH \
  GAZEBO_RESOURCE_PATH \
  GZ_CONFIG_PATH \
  GZ_FILE_PATH \
  GZ_SIM_RESOURCE_PATH \
  IGN_CONFIG_PATH \
  IGN_GAZEBO_RESOURCE_PATH \
  IGN_GAZEBO_SYSTEM_PLUGIN_PATH \
  LD_LIBRARY_PATH \
  LD_PRELOAD \
  LIBRARY_PATH \
  PKG_CONFIG_PATH \
  PYTHONPATH \
  RMW_IMPLEMENTATION \
  ROS_AUTOMATIC_DISCOVERY_RANGE \
  ROS_DISTRO \
  ROS_DOMAIN_ID \
  ROS_LOCALHOST_ONLY \
  ROS_PACKAGE_PATH \
  ROS_PYTHON_VERSION \
  ROS_STATIC_PEERS \
  ROS_VERSION \
  ZENOH_CONFIG_OVERRIDE

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble underlay is unavailable: /opt/ros/humble/setup.bash\n' >&2
  exit 1
fi

# Humble's generated setup scripts are not safe under nounset.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -r $project_root/install/local_setup.bash ]]; then
  # local_setup does not recursively replay underlays recorded during a build.
  # shellcheck disable=SC1091
  source "$project_root/install/local_setup.bash"
fi
set -u
hash -r

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=${PAS_DUAL_ARM_ROS_DOMAIN_ID:-5}
export ROS_LOCALHOST_ONLY=${PAS_DUAL_ARM_ROS_LOCALHOST_ONLY:-1}
export PAS_DUAL_ARM_ROOT=$project_root
if [[ -r $project_root/src/pas_dual_arm_bringup/config/fastdds_profiles.xml ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$project_root/src/pas_dual_arm_bringup/config/fastdds_profiles.xml"
fi

IFS=: read -r -a prefixes <<< "${AMENT_PREFIX_PATH:-}"
for prefix in "${prefixes[@]}"; do
  case "$prefix" in
    /opt/ros/humble|"$project_root"/install|"$project_root"/install/*) ;;
    *)
      printf 'Foreign ROS overlay entered the PAS-DUAL-ARM environment: %s\n' "$prefix" >&2
      exit 1
      ;;
  esac
done

cd "$project_root"

if (($#)); then
  exec "$@"
fi

export PS1='(pas-dual-arm) \u@\h:\w\$ '
printf '%s\n' \
  'PAS-DUAL-ARM native environment' \
  "  ROS:        $ROS_DISTRO" \
  "  RMW:        $RMW_IMPLEMENTATION" \
  "  domain:     $ROS_DOMAIN_ID" \
  "  localhost:  $ROS_LOCALHOST_ONLY" \
  "  workspace:  $project_root" \
  'Exit this shell before entering another ROS project.'
exec bash --noprofile --norc -i
