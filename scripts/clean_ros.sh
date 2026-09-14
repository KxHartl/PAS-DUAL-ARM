#!/usr/bin/env bash
# Terminate any leftover ROS 2, Gazebo, and project nodes to ensure a clean start.
set -e

echo "Zaustavljam zaostale ROS i Gazebo procese..."
PIDS=$(pgrep -f '(ros2 launch pas_dual_arm_bringup|gz sim|ign gazebo|room_navigator|cmd_vel_relay|scan_filter|static_transform_publisher|nav2_|controller_server|bt_navigator|amcl|map_server|rviz2|teleop_twist_keyboard|parameter_bridge|ros_gz_bridge)' || true)

if [ -n "$PIDS" ]; then
    echo "Pronađeni procesi: $PIDS"
    echo "$PIDS" | xargs kill -9 2>/dev/null || true
    sleep 1
    echo "Procesi uspješno ugašeni."
fi

# Zaustavi i ros2 daemon ako je pokrenut
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
"$DIR/run_native.sh" ros2 daemon stop >/dev/null 2>&1 || true
echo "ROS 2 okoliš je čist."
