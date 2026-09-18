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
# `pgrep -f` gleda cijelu naredbenu liniju, pa pogodi i shell koji je POKRENUO ovu skriptu ako
# ta naredba negdje sadrži jedan od uzoraka - npr.
#   bash scripts/clean_ros.sh && ros2 launch pas_dual_arm_bringup scenario_mission.launch.py
# Tada se `kill -9` pošalje vlastitom roditelju i pokretanje se **tiho preskoči**: nema greske,
# nema simulacije, a log ostane od prosloga runa. Dogodilo se dvaput 18. 9. prije nego je
# primijeceno. Zato se iz popisa izbacuju vlastiti PID i svi nasi preci.
#
# Iskljucuju se ANCESTORI, a ne cijela procesna grupa: simulator pokrenut iz istog terminala je u
# istoj grupi, a njega treba ugasiti.
ancestors() {
    local pid=$$
    while [ -n "$pid" ] && [ "$pid" != "0" ] && [ "$pid" != "1" ]; do
        printf '%s\n' "$pid"
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    done
}
SKIP=" $(ancestors | tr '\n' ' ')"

PIDS=$(pgrep -f '(ros2 launch pas_dual_arm_bringup|gz sim|ign gazebo|room_navigator|cmd_vel_relay|scan_filter|static_transform_publisher|nav2_|controller_server|bt_navigator|amcl|map_server|rviz2|teleop_twist_keyboard|parameter_bridge|ros_gz_bridge|aruco_detector|loc_error|main_task|move_group|footprint_publisher|nav_gui|nav_zones|feature_registry|table_ready|map_handoff|room_sweeper|frontier_explorer|cloud_restamp|set_posture|robot_state_publisher|joint_state_publisher|spawner|ros2 bag record)' || true)

# Makni sebe i svoje pretke iz popisa za gasenje.
if [ -n "$PIDS" ]; then
    KEEP=""
    for pid in $PIDS; do
        case "$SKIP" in *" $pid "*) continue ;; esac
        KEEP="$KEEP $pid"
    done
    PIDS=$(echo "$KEEP" | tr -s ' ' | sed 's/^ //;s/ $//')
fi

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
