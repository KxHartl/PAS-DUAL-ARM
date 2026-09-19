#!/usr/bin/env bash
# Put the physics on the fast cores and the controllers on cores of their own.
#
# This machine is an i7-14650HX: cpu0-15 are the eight P-cores' threads at
# 5.0-5.2 GHz, cpu16-23 are eight E-cores at 3.7. The kernel scatters our
# processes across both and the result is measurable - controller_server misses
# its 20 Hz deadline 649 times in a run while sitting at 55 % of one core, which
# is not a busy process. It is a process waiting to be scheduled.
#
# So: Gazebo gets P-cores, because its physics loop is single-threaded and wants
# the fastest clock there is, and nothing else can help it. The ROS controllers
# get E-cores to themselves, because a whole slow core beats a shared fast one
# for anything with a deadline. If one of them turns out to be compute-bound
# rather than starved, `--promote <name>` moves it back to the P-cores.
#
#   bash scripts/pin_cores.sh            # pin what is running
#   bash scripts/pin_cores.sh --report   # who is where, and how busy each core is
#   bash scripts/pin_cores.sh --promote controller_server
set -u

# Measured, not assumed. Confined to one E-core each, controller_server ran at
# 100 % of it and laser_odometry at 95 - both compute-bound, not merely starved -
# while the sixteen P-threads sat at about 50 %, which is Gazebo using two of
# eight physical cores. So the two expensive ROS nodes go to P-cores of their
# own, and the E-cores take the cheap ones.
P_CORES=${P_CORES:-0-7}          # physics, four physical cores
ALONE_A_CORE=${ALONE_A_CORE:-8-11}    # the local controller: MPPI is not cheap
ALONE_B_CORE=${ALONE_B_CORE:-12-13}   # the scan matcher
REST_CORES=${REST_CORES:-16-23}       # everything else, on the E-cores
E_CORES=${E_CORES:-16-23}

# The physics. Everything else in a run is downstream of it.
HEAVY='ign gazebo|gz sim|ruby .*ign'
# One core each for the two that actually cost something and cannot be late.
# Handing them a RANGE is not enough: with 0-15 to choose from the kernel put
# controller_server and room_navigator on the same core, 66 % and 56 % of it,
# which is the starvation this script exists to remove.
ALONE_A='controller_server'
ALONE_B='laser_odometry'
# The rest are deadline-bound but cheap, and can share what is left.
TIMED='planner_server|bt_navigator|amcl|cmd_vel_relay|collision_monitor|velocity_smoother|room_navigator'

# Only our own partition, when we are in one. pgrep matches by command line, so
# three parallel workers would each pin the others' processes onto their own
# slice and all three would end up crowded into one third of the machine.
in_our_partition() {
    [ -z "${IGN_PARTITION:-}" ] && return 0
    grep -qz "IGN_PARTITION=$IGN_PARTITION" "/proc/$1/environ" 2>/dev/null
}

pin() {   # pattern, cores, label
    local found=0
    for pid in $(pgrep -f "$1" 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        in_our_partition "$pid" || continue
        taskset -acp "$2" "$pid" >/dev/null 2>&1 && found=$((found + 1))
    done
    printf '%-28s -> cpu %-8s %d procesa\n' "$3" "$2" "$found"
}

report() {
    printf '\n%-22s %6s %5s %7s\n' 'proces' 'cpu' 'tip' '%CPU'
    for pattern in $(echo "$HEAVY|$ALONE_A|$ALONE_B|$TIMED" | tr '|' ' '); do
        for pid in $(pgrep -f "$pattern" 2>/dev/null | head -2); do
            [ "$pid" = "$$" ] && continue
            local cpu type name use
            cpu=$(ps -o psr= -p "$pid" 2>/dev/null | tr -d ' ') || continue
            [ -z "$cpu" ] && continue
            [ "$cpu" -ge 16 ] && type=E || type=P
            name=$(ps -o comm= -p "$pid" | cut -c1-22)
            use=$(ps -o pcpu= -p "$pid" | tr -d ' ')
            printf '%-22s %6s %5s %7s\n' "$name" "$cpu" "$type" "$use"
        done
    done
    echo
    echo 'zauzece po jezgri (1 s), iz /proc/stat:'
    awk '/^cpu[0-9]/ {idle[$1]=$5; tot[$1]=$2+$3+$4+$5+$6+$7+$8} END {
        for (c in idle) printf "%s %d %d\n", c, idle[c], tot[c]
    }' /proc/stat | sort > /tmp/.pin_a
    sleep 1
    awk '/^cpu[0-9]/ {idle[$1]=$5; tot[$1]=$2+$3+$4+$5+$6+$7+$8} END {
        for (c in idle) printf "%s %d %d\n", c, idle[c], tot[c]
    }' /proc/stat | sort > /tmp/.pin_b
    join /tmp/.pin_a /tmp/.pin_b | awk '{
        di = $4 - $2; dt = $5 - $3
        n = substr($1, 4) + 0
        type = (n >= 16) ? "E" : "P"
        busy = (dt > 0) ? 100 * (1 - di / dt) : 0
        bar = ""
        for (i = 0; i < int(busy / 5); i++) bar = bar "#"
        printf "  cpu%-3s %s %5.1f %%  %s\n", n, type, busy, bar
    }' | sort -t u -k2 -n
    rm -f /tmp/.pin_a /tmp/.pin_b
}

case "${1:-}" in
    --report)
        report
        ;;
    --promote)
        pin "${2:?ime procesa}" "$P_CORES" "promaknut ${2}"
        ;;
    *)
        echo "Zakucavam na jezgre (P=$P_CORES, E=$E_CORES):"
        pin "$HEAVY" "$P_CORES" 'fizika (Gazebo)'
        pin "$ALONE_A" "${ALONE_A_CORE:-16}" 'regulator putanje (sam)'
        pin "$ALONE_B" "${ALONE_B_CORE:-17}" 'laserska odometrija (sam)'
        pin "$TIMED" "${REST_CORES:-18-23}" 'ostali cvorovi'
        echo
        echo 'Provjera: bash scripts/pin_cores.sh --report'
        ;;
esac
