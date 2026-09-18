#!/usr/bin/env python3
"""One table for the whole question of where odometry should come from.

Every calibration drive in a directory, scored against ground truth and grouped
by the configuration it ran, so the rows can be compared instead of read one at
a time. The names come from the run folders, which yaw_run.sh writes as
yaw-<heading>-<translation>-<time>.

A word on those names, because they mislead if taken at face value: the laser is
correcting in EVERY configuration. `translation=wheels` means the laser corrects
a dead reckoning the wheels carry between matches; `translation=laser` means
there is nothing to carry it and the pose stands still until the next match.

    ./scripts/run_native.sh python3 validation/odometry_matrix.py <dir>
"""
import argparse
import os
import statistics as st
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def score(bag):
    """(yaw mean, yaw max, pos mean, pos max) for one drive, or None."""
    out = subprocess.run(
        [sys.executable, os.path.join(HERE, 'evaluate_imu_yaw.py'), bag],
        capture_output=True, text=True).stdout
    rows = [line.split() for line in out.splitlines() if line.startswith('fuzija')]
    if len(rows) != 2:
        return None
    # fuzija (/laser_odom)   <n> <max> <mean> <final>, degrees then millimetres
    return (float(rows[0][-2]), float(rows[0][-3]),
            float(rows[1][-2]), float(rows[1][-3]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('folder')
    args = parser.parse_args()

    groups = {}
    for name in sorted(os.listdir(args.folder)):
        bag = os.path.join(args.folder, name, 'bag')
        if not name.startswith('yaw-') or not os.path.isdir(bag):
            continue
        parts = name.split('-')
        if len(parts) < 4:
            continue                            # an older run, before the matrix
        got = score(bag)
        if got:
            groups.setdefault((parts[1], parts[2]), []).append(got)

    print(f'\n{"smjer":<9}{"pomak":<9}{"n":>3}'
          f'{"zakret sred":>13}{"zakret max":>12}'
          f'{"polozaj sred":>14}{"polozaj max":>13}')
    print(f'{"":<21}{"[deg]":>13}{"[deg]":>12}{"[mm]":>14}{"[mm]":>13}')
    for (heading, translation), rows in sorted(groups.items()):
        print(f'{heading:<9}{translation:<9}{len(rows):>3}'
              f'{st.median(r[0] for r in rows):>13.3f}'
              f'{st.median(r[1] for r in rows):>12.2f}'
              f'{st.median(r[2] for r in rows):>14.2f}'
              f'{st.median(r[3] for r in rows):>13.1f}')
    print('\nLaser ispravlja u SVAKOM retku. "pomak=wheels" znaci da kotaci nose '
          'mrtvi racun\nizmedu podudaranja, "pomak=laser" da ga nista ne nosi.\n')


if __name__ == '__main__':
    main()
