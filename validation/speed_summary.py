#!/usr/bin/env python3
"""How long did the mission take, in SIMULATED seconds, and how often did it work?

Wall-clock duration is useless for comparing setups once runs go in parallel:
three simulations share one machine, the real-time factor drops, and a run gets
"slower" without the robot doing anything differently. The robot lives in sim
time, so that is what gets measured - from the moment it sets off for the cube
(step 3/8) to MISSION COMPLETE. Startup and the harness's wait for a button that
only it presses are left out; they are not the mission.

The log is stamped with the wall clock and the bag carries /clock with its own
wall stamp, so the two are matched through the bag's sqlite index directly:
reading a whole 1 kHz /clock through rosbag2_py takes a minute per run, and this
needs two messages.

    ./scripts/run_native.sh python3 validation/speed_summary.py <batch> [<batch>...]
"""
import csv
import os
import re
import sqlite3
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
START = re.compile(r'\[(\d+\.\d+)\].*\[3/8\] ')
END = re.compile(r'\[(\d+\.\d+)\].*MISSION COMPLETE')


def sim_at(db, topic_id, wall, deserialize, msg_type):
    """Simulation time of the /clock message recorded nearest after `wall`."""
    row = db.execute('SELECT data FROM messages WHERE topic_id = ? AND timestamp >= ? '
                     'ORDER BY timestamp LIMIT 1', (topic_id, int(wall * 1e9))).fetchone()
    if row is None:
        return None
    clock = deserialize(row[0], msg_type).clock
    return clock.sec + clock.nanosec * 1e-9


def mission_seconds(folder):
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    log = os.path.join(folder, 'run.log')
    if not os.path.isfile(log):
        return None
    text = open(log, errors='replace').read()
    start, end = START.search(text), END.search(text)
    if not (start and end):
        return None
    files = [f for f in os.listdir(os.path.join(folder, 'bag')) if f.endswith('.db3')] \
        if os.path.isdir(os.path.join(folder, 'bag')) else []
    if not files:
        return None
    db = sqlite3.connect(os.path.join(folder, 'bag', files[0]))
    topic = db.execute("SELECT id FROM topics WHERE name = '/clock'").fetchone()
    if topic is None:
        return None
    a = sim_at(db, topic[0], float(start.group(1)), deserialize_message, Clock)
    b = sim_at(db, topic[0], float(end.group(1)), deserialize_message, Clock)
    wall = float(end.group(1)) - float(start.group(1))
    return None if a is None or b is None else (b - a, wall)


def main():
    for batch in sys.argv[1:]:
        root = os.path.join(HERE, 'results', batch)
        rows = list(csv.DictReader(open(os.path.join(root, 'results.csv'))))
        ok = [r for r in rows if r['outcome'] == 'success']
        sims, walls = [], []
        for r in ok:
            got = mission_seconds(os.path.join(root, r['dir']))
            if got:
                sims.append(got[0])
                walls.append(got[1])
        rtf = st.median([s / w for s, w in zip(sims, walls)]) if sims else float('nan')
        line = f'{batch:<22} {len(ok):>2}/{len(rows):<2} uspjeh'
        if sims:
            line += (f'   misija sim: medijan {st.median(sims):6.1f} s  '
                     f'min {min(sims):6.1f}  max {max(sims):6.1f}   RTF {rtf:.2f}')
        print(line)
        reasons = {}
        for r in rows:
            if r['outcome'] != 'success':
                key = r['reason'][:60]
                reasons[key] = reasons.get(key, 0) + 1
        for k, v in reasons.items():
            print(f'    {v}x {k}')


if __name__ == '__main__':
    main()
