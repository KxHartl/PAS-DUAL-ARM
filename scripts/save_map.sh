#!/usr/bin/env bash

# Save the map slam_toolbox is currently building.
#
# Writes both formats, because they answer different questions:
#   maps/<name>.yaml + .pgm   occupancy grid - what map_server loads, and the
#                             picture that goes into the seminar
#   maps/<name>.posegraph     slam_toolbox's own serialised graph - lets SLAM be
#                             resumed or re-run in localisation mode later
#
# Usage: ./scripts/save_map.sh [tag]     (default: teleop)
# Example: ./scripts/save_map.sh run43
# Produces: maps/map_20260913_205500_run43.{yaml,pgm,posegraph,data}
#      and: maps/seminar_map.{yaml,pgm,posegraph,data}

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_root=$(cd -- "$script_dir/.." && pwd)

tag=${1:-teleop}
# Normalize if user passes seminar_map directly
if [ "$tag" = "seminar_map" ]; then
  tag="teleop"
fi

timestamp=$(date +"%Y%m%d_%H%M%S")
name="map_${timestamp}_${tag}"
maps_dir="$project_root/src/pas_dual_arm_bringup/maps"
mkdir -p "$maps_dir"

run() { "$script_dir/run_native.sh" "$@"; }

if ! run ros2 service list 2>/dev/null | grep -q '/slam_toolbox/save_map'; then
  printf 'slam_toolbox is not running (no /slam_toolbox/save_map service).\n' >&2
  printf 'Start it first: ros2 launch pas_dual_arm_bringup mapping.launch.py\n' >&2
  exit 1
fi

printf 'Saving occupancy grid -> %s/%s.{yaml,pgm}\n' "$maps_dir" "$name"
run ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$maps_dir/$name'}}"

printf 'Saving pose graph      -> %s/%s.posegraph\n' "$maps_dir" "$name"
run ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$maps_dir/$name'}"

# Also mirror to seminar_map so Nav2 default works out of the box
printf 'Updating active map   -> %s/seminar_map.{yaml,pgm,posegraph,data}\n' "$maps_dir"
cp -f "$maps_dir/$name.pgm" "$maps_dir/seminar_map.pgm"
if [ -f "$maps_dir/$name.posegraph" ]; then
  cp -f "$maps_dir/$name.posegraph" "$maps_dir/seminar_map.posegraph"
fi
if [ -f "$maps_dir/$name.data" ]; then
  cp -f "$maps_dir/$name.data" "$maps_dir/seminar_map.data"
fi
sed "s/${name}\.pgm/seminar_map.pgm/g" "$maps_dir/$name.yaml" > "$maps_dir/seminar_map.yaml"

printf '\nVerifying coverage gate with check_map.py:\n'
run python3 "$script_dir/check_map.py" "$maps_dir/$name.yaml"

printf '\nWritten to %s:\n' "$maps_dir"
ls -la "$maps_dir" | sed 's/^/  /'
printf '\nRebuild so the installed share/ copy sees it:\n'
printf '  ./scripts/run_native.sh colcon build --packages-select pas_dual_arm_bringup\n'
