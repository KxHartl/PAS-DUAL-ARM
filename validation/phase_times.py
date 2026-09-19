#!/usr/bin/env python3
"""Where does the mission's simulated time go, phase by phase?

Uses the mission's own step banners ([n/8] ...) and converts their wall stamps
to simulation time through the run's bag, as speed_summary.py does. The median
over the successful runs of a batch is what is printed.

    ./scripts/run_native.sh python3 validation/phase_times.py <batch>
"""
import csv
import os
import re
import sqlite3
import statistics as st
import sys

from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock

HERE = os.path.dirname(os.path.abspath(__file__))
STEP = re.compile(r'\[(\d+\.\d+)\].*\[(\d)/8\] ([^:]+):')
DONE = re.compile(r'\[(\d+\.\d+)\].*MISSION COMPLETE')


def main():
    batch = sys.argv[1]
    root = os.path.join(HERE, 'results', batch)
    rows = [r for r in csv.DictReader(open(os.path.join(root, 'results.csv')))
            if r['outcome'] == 'success']
    phases = {}
    for r in rows:
        folder = os.path.join(root, r['dir'])
        text = open(os.path.join(folder, 'run.log'), errors='replace').read()
        marks, seen = [], set()
        for m in STEP.finditer(text):
            n = int(m.group(2))
            if n not in seen:
                seen.add(n)
                marks.append((float(m.group(1)), f'{n}/8 {m.group(3).strip()[:38]}'))
        done = DONE.search(text)
        if done:
            marks.append((float(done.group(1)), 'kraj'))
        db3 = [f for f in os.listdir(os.path.join(folder, 'bag')) if f.endswith('.db3')][0]
        db = sqlite3.connect(os.path.join(folder, 'bag', db3))
        tid = db.execute("SELECT id FROM topics WHERE name='/clock'").fetchone()[0]

        def sim(wall):
            row = db.execute('SELECT data FROM messages WHERE topic_id=? AND timestamp>=? '
                             'ORDER BY timestamp LIMIT 1', (tid, int(wall * 1e9))).fetchone()
            c = deserialize_message(row[0], Clock).clock
            return c.sec + c.nanosec * 1e-9
        for (w0, name), (w1, _) in zip(marks, marks[1:]):
            phases.setdefault(name, []).append(sim(w1) - sim(w0))
    print(f'{batch}: {len(rows)} uspjesnih runova, medijan po fazi (sim s)')
    total = 0.0
    for name, values in phases.items():
        if name.startswith(('1/8', '2/8')):
            continue
        med = st.median(values)
        total += med
        print(f'  {name:<48} {med:7.1f}')
    print(f'  {"misija (3/8 do kraja)":<48} {total:7.1f}')


if __name__ == '__main__':
    main()
