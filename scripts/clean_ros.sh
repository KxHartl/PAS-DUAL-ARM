#!/usr/bin/env bash
# Terminate any leftover ROS 2, Gazebo, and project nodes to ensure a clean start.
set -e

echo "Zaustavljam zaostale ROS i Gazebo procese..."
# Everything the project starts, not just the loud half. The ArUco detectors,
# move_group and loc_error survived every previous cleanup and kept processing
# camera frames between runs: four of them left over cost about 70 % of a core
# each run, the simulation slowed to a third of real time, and a localiser that
# cannot keep up hands Nav2 a stale transform - which is how a leg gets reported
# as reached without the robot moving (P-49, P-50, 18 Sep).
PIDS=$(pgrep -f '(ros2 launch pas_dual_arm_bringup|gz sim|ign gazebo|room_navigator|cmd_vel_relay|scan_filter|static_transform_publisher|nav2_|controller_server|bt_navigator|amcl|map_server|rviz2|teleop_twist_keyboard|parameter_bridge|ros_gz_bridge|aruco_detector|loc_error|main_task|move_group|footprint_publisher|nav_gui|nav_zones|feature_registry|table_ready|map_handoff|room_sweeper|frontier_explorer|cloud_restamp|set_posture|robot_state_publisher|joint_state_publisher|spawner|ros2 bag record)' || true)

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
