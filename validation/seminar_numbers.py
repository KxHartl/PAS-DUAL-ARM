#!/usr/bin/env python3
"""Every measured figure the seminar quotes, written as LaTeX macros.

The seminar used to carry numbers typed in by hand from one run. With a series
behind it, each figure has a median and a range, and retyping twenty of them
after every batch is how a paper ends up claiming something the data no longer
says. So the batch writes them: this reads a batch's results.csv, metrics.csv
and bags, and emits seminar/mjerenja.tex, which seminar.tex \input's.

Refresh after a series with:

    ./scripts/run_native.sh python3 validation/seminar_numbers.py <batch> [<batch>...]

Several batches are pooled - they must be the same code and the same number of
parallel workers, or the localisation figures are not comparable (a third worker
starves AMCL: 2.95 cm against 4.60).
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
# Half the published footprint, from nav2_params.yaml; the robot is 0.854 m wide
# there because the footprint covers every protrusion, not just the base.
FOOTPRINT_HALF_WIDTH_M = 0.427
REPO = os.path.dirname(HERE)
STEP = re.compile(r'\[(\d+\.\d+)\].*\[(\d)/8\] ([^:]+):')
START = re.compile(r'\[(\d+\.\d+)\].*\[3/8\] ')
DONE = re.compile(r'\[(\d+\.\d+)\].*MISSION COMPLETE')


def hr(value, digits=1):
    """A number the way the text sets it: decimal comma, fixed digits."""
    return f'{value:.{digits}f}'.replace('.', ',')


def sim_clock(folder):
    """A function turning this run's wall stamps into simulated seconds."""
    bag = os.path.join(folder, 'bag')
    files = [f for f in os.listdir(bag) if f.endswith('.db3')] if os.path.isdir(bag) else []
    if not files:
        return None
    db = sqlite3.connect(os.path.join(bag, files[0]))
    topic = db.execute("SELECT id FROM topics WHERE name='/clock'").fetchone()
    if topic is None:
        return None

    def at(wall):
        row = db.execute('SELECT data FROM messages WHERE topic_id=? AND timestamp>=? '
                         'ORDER BY timestamp LIMIT 1', (topic[0], int(wall * 1e9))).fetchone()
        if row is None:
            return None
        clock = deserialize_message(row[0], Clock).clock
        return clock.sec + clock.nanosec * 1e-9
    return at


def collect(batches):
    runs, metrics, missions, legs = [], [], [], {}
    for batch in batches:
        root = os.path.join(HERE, 'results', batch)
        rows = list(csv.DictReader(open(os.path.join(root, 'results.csv'))))
        runs += rows
        path = os.path.join(root, 'metrics.csv')
        if os.path.isfile(path):
            metrics += list(csv.DictReader(open(path)))
        for r in rows:
            if r['outcome'] != 'success':
                continue
            folder = os.path.join(root, r['dir'])
            log = os.path.join(folder, 'run.log')
            if not os.path.isfile(log):
                continue
            text = open(log, errors='replace').read()
            at = sim_clock(folder)
            if at is None:
                continue
            begin, end = START.search(text), DONE.search(text)
            if begin and end:
                a, b = at(float(begin.group(1))), at(float(end.group(1)))
                if a is not None and b is not None:
                    missions.append(b - a)
            marks, seen = [], set()
            for m in STEP.finditer(text):
                n = int(m.group(2))
                if n not in seen:
                    seen.add(n)
                    marks.append((float(m.group(1)), n))
            if end:
                marks.append((float(end.group(1)), 9))
            for (w0, n), (w1, _) in zip(marks, marks[1:]):
                a, b = at(w0), at(w1)
                if a is not None and b is not None:
                    legs.setdefault(n, []).append(b - a)
    return runs, metrics, missions, legs


def column(metrics, name):
    out = []
    for row in metrics:
        value = row.get(name)
        if value not in (None, '', 'nan'):
            out.append(float(value))
    return out


def main():
    batches = sys.argv[1:]
    if not batches:
        sys.exit(__doc__)
    runs, metrics, missions, legs = collect(batches)
    good = [r for r in runs if r['outcome'] == 'success']
    n, ok = len(runs), len(good)

    lines = [
        '% Generirano: validation/seminar_numbers.py ' + ' '.join(batches),
        '% Ne uređivati ručno - svaka izmjena nestaje pri sljedecem osvjezavanju.',
        '',
        f'\\newcommand{{\\Nrunova}}{{{n}}}',
        f'\\newcommand{{\\Nuspjeha}}{{{ok}}}',
        f'\\newcommand{{\\Uspjesnost}}{{{hr(100.0 * ok / n, 1) if n else "--"}}}',
    ]
    # Rule of three: with no failure in n runs, the 95 % lower bound on the
    # success rate is 1 - 3/n. It is the honest way to put a number on "it
    # worked every time" without claiming the sample proves more than it does.
    if n and ok == n:
        lines.append(f'\\newcommand{{\\Donjagranica}}{{{hr(100.0 * (1 - 3.0 / n), 1)}}}')

    def stat(macro, name, digits=1, scale=1.0):
        values = [v * scale for v in column(metrics, name)]
        if not values:
            return
        lines.append(f'\\newcommand{{\\{macro}Med}}{{{hr(st.median(values), digits)}}}')
        lines.append(f'\\newcommand{{\\{macro}Min}}{{{hr(min(values), digits)}}}')
        lines.append(f'\\newcommand{{\\{macro}Max}}{{{hr(max(values), digits)}}}')

    lines.append('')
    stat('Odlaganje', 'place_err_truth_mm')
    stat('Odlaganjex', 'place_dx_mm')
    stat('Odlaganjey', 'place_dy_mm')
    stat('Dizanje', 'cube_lift_m', digits=3)
    lines.append('')
    stat('Amcl', 'amcl_err_max_cm')
    stat('Amclrms', 'amcl_err_rms_cm')
    stat('Amclzakret', 'amcl_yaw_max_deg')
    lines.append('')
    stat('Vrata', 'door_clear_min_m', digits=1, scale=100.0)   # cm
    stat('Stol', 'table_clear_transit_min_m', digits=3)
    stat('Zid', 'wall_clear_open_min_m', digits=3)
    stat('Putanja', 'path_len_m', digits=2)
    # The text quotes the gap between the published footprint and the jamb, not
    # the distance from the doorway's axis, so half the footprint width comes off.
    gaps = [(v - FOOTPRINT_HALF_WIDTH_M) * 100.0 for v in column(metrics, 'door_clear_min_m')]
    if gaps:
        lines.append(f'\\newcommand{{\\ZazorMed}}{{{hr(st.median(gaps))}}}')
        lines.append(f'\\newcommand{{\\ZazorMin}}{{{hr(min(gaps))}}}')
        lines.append(f'\\newcommand{{\\ZazorMax}}{{{hr(max(gaps))}}}')
    passes = column(metrics, 'door_passes')
    if passes:
        lines.append(f'\\newcommand{{\\Prolaza}}{{{int(sum(passes))}}}')

    # What the robot itself reported, from results.csv: tool tips against their
    # targets, the carriage height, and its own claim about the placement. These
    # are the system's own measurements, and the table says so.
    def from_runs(macro, *names, digits=1):
        values = []
        for row in good:
            for name in names:
                value = row.get(name)
                if value not in (None, '', 'nan'):
                    values.append(float(value))
        if values:
            lines.append(f'\\newcommand{{\\{macro}Med}}{{{hr(st.median(values), digits)}}}')
            lines.append(f'\\newcommand{{\\{macro}Min}}{{{hr(min(values), digits)}}}')
            lines.append(f'\\newcommand{{\\{macro}Max}}{{{hr(max(values), digits)}}}')

    lines.append('')
    from_runs('Vrhalata', 'tip_left_mm', 'tip_right_mm')
    from_runs('Klizac', 'lift_left_m', 'lift_right_m', digits=4)
    from_runs('Odlaganjezapis', 'place_error_mm')

    lines.append('')
    if missions:
        lines.append(f'\\newcommand{{\\MisijaMed}}{{{hr(st.median(missions))}}}')
        lines.append(f'\\newcommand{{\\MisijaMin}}{{{hr(min(missions))}}}')
        lines.append(f'\\newcommand{{\\MisijaMax}}{{{hr(max(missions))}}}')
    # Leg 3 is the drive to the room holding the cube, leg 6 the carry to the
    # room it is placed in; those are the two the text quotes by name.
    for step, macro in ((3, 'Voznjaplava'), (6, 'Voznjacrvena')):
        if legs.get(step):
            lines.append(f'\\newcommand{{\\{macro}}}{{{hr(st.median(legs[step]))}}}')

    out = os.path.join(REPO, 'seminar', 'mjerenja.tex')
    open(out, 'w').write('\n'.join(lines) + '\n')
    print(f'{ok}/{n} uspjesnih iz: {", ".join(batches)}')
    print(f'-> {out}')
    print('\n'.join(l for l in lines if l.startswith('\\newcommand')))


if __name__ == '__main__':
    main()
