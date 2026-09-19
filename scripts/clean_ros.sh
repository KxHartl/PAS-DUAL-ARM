#!/usr/bin/env bash
# Terminate any leftover ROS 2, Gazebo, and project nodes to ensure a clean start.
set -e

if [ -n "${IGN_PARTITION:-}" ]; then
    echo "Zaustavljam zaostale procese u particiji $IGN_PARTITION..."
else
    echo "Zaustavljam zaostale ROS i Gazebo procese..."
fi
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
#
# `laser_odometry` je u popisu nedostajao do 18. 9. navecer, i to nije bilo tiho. Cvor prezivi
# svako ciscenje, sljedeci run digne jos jedan, i za nekoliko pokusaja ih pet objavljuje na
# /laser_odom istovremeno. U bagu se vidi kao 247 Hz umjesto 50, a mjerenje ispadne besmisleno:
# 34 stupnja greske zakreta ondje gdje je isti pokus s jednim cvorom dao 1,3.
ancestors() {
    local pid=$$
    while [ -n "$pid" ] && [ "$pid" != "0" ] && [ "$pid" != "1" ]; do
        printf '%s\n' "$pid"
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    done
}
SKIP=" $(ancestors | tr '\n' ' ')"

# Kill only OUR partition, when we are in one.
#
# This script matches on the command line, which is fine for one simulation on a
# machine and fatal for several: run_batch calls it between runs, so two batches
# in parallel would take turns killing each other. That happened on 18 Sep with
# two batches and thousands of "Detected jump back in time".
#
# With IGN_PARTITION set, a process only counts if its own environment carries
# the same one - the test run_cube_isolated.sh already uses to refuse a second
# simulator in its partition. Without it, nothing changes and everything dies,
# which is what a person running this by hand means by it.
in_our_partition() {
    [ -z "${IGN_PARTITION:-}" ] && return 0
    grep -qz "IGN_PARTITION=$IGN_PARTITION" "/proc/$1/environ" 2>/dev/null
}

PIDS=$(pgrep -f '(ros2 launch pas_dual_arm_bringup|gz sim|ign gazebo|room_navigator|cmd_vel_relay|scan_filter|static_transform_publisher|nav2_|controller_server|bt_navigator|amcl|map_server|rviz2|teleop_twist_keyboard|parameter_bridge|ros_gz_bridge|aruco_detector|loc_error|main_task|move_group|footprint_publisher|nav_gui|nav_zones|feature_registry|table_ready|map_handoff|room_sweeper|frontier_explorer|cloud_restamp|set_posture|laser_odometry|yaw_drive|scan_watch|robot_state_publisher|joint_state_publisher|spawner|ros2 bag record)' || true)

# Makni sebe i svoje pretke iz popisa za gasenje.
if [ -n "$PIDS" ]; then
    KEEP=""
    for pid in $PIDS; do
        case "$SKIP" in *" $pid "*) continue ;; esac
        in_our_partition "$pid" || continue
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
