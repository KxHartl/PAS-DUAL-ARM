#!/usr/bin/env python3
"""Drive several batches at once, so a series is minutes instead of hours.

A run takes about ten minutes and tells you almost nothing on its own; three
runs cannot tell 80 % apart from 95 %. That arithmetic has shaped the whole of
this work - changes get judged on three runs because thirty would take five
hours - and it does not have to.

This machine has 24 threads and one simulation uses about five. What stopped
several running at once was never the hardware:

  * every simulation published on the same ROS domain and the same Ignition
    partition, so they saw each other's topics and each other's /clock;
  * run_batch calls clean_ros.sh between runs, and clean_ros.sh matched on the
    command line, so two batches took turns killing each other. That happened
    on 18 Sep, with thousands of "Detected jump back in time" to show for it.

Both are fixed rather than worked around: each worker gets its own domain,
partition and ROS_HOME, and clean_ros.sh now kills only processes whose own
environment carries its partition.

    ./scripts/run_native.sh python3 validation/run_parallel.py \
        --n 12 --workers 3 --batch mppi-par -- --laser-odometry
"""
import argparse
import csv
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
RESULTS = os.path.join(HERE, 'results')

# Cores per worker, as pin_cores.sh reads them. cpu0-15 are the P-cores'
# threads, cpu16-23 the E-cores; one P-thread and two E-cores are left for the
# rest of the machine.
SLICES = [
    {'P_CORES': '0-2',   'ALONE_A_CORE': '3',  'ALONE_B_CORE': '4',  'REST_CORES': '16-17'},
    {'P_CORES': '5-7',   'ALONE_A_CORE': '8',  'ALONE_B_CORE': '9',  'REST_CORES': '18-19'},
    {'P_CORES': '10-12', 'ALONE_A_CORE': '13', 'ALONE_B_CORE': '14', 'REST_CORES': '20-21'},
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--n', type=int, default=12, help='runs in total')
    parser.add_argument('--workers', type=int, default=3)
    parser.add_argument('--batch', required=True)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('rest', nargs=argparse.REMAINDER,
                        help='everything after -- goes to run_batch')
    args = parser.parse_args()

    if args.workers > len(SLICES):
        sys.exit(f'{len(SLICES)} slices of this machine are defined; asked for '
                 f'{args.workers}. Each simulation wants about five threads.')
    passthrough = [a for a in args.rest if a != '--']

    # Spread the runs as evenly as they go.
    counts = [args.n // args.workers] * args.workers
    for i in range(args.n % args.workers):
        counts[i] += 1

    workers = []
    for i, count in enumerate(counts):
        if count == 0:
            continue
        name = f'{args.batch}-w{i}'
        shutil.rmtree(os.path.join(RESULTS, name), ignore_errors=True)
        environment = dict(os.environ)
        environment.update(SLICES[i])
        # A domain each. ROS 2 domains are 0-101 and neighbouring ones share
        # ports, so they are spread rather than adjacent.
        environment['PAS_DUAL_ARM_ROS_DOMAIN_ID'] = str(11 + i * 10)
        environment['IGN_PARTITION'] = f'pas_parallel_{i}'
        environment['GZ_PARTITION'] = f'pas_parallel_{i}'
        environment['ROS_HOME'] = os.path.join(REPO, 'log', f'parallel_{i}_ros_home')
        os.makedirs(environment['ROS_HOME'], exist_ok=True)
        command = [sys.executable, os.path.join(HERE, 'run_batch.py'),
                   '--n', str(count), '--batch', name,
                   # A different seed each, or every worker drives the same
                   # world and twelve runs are one run measured twelve times.
                   '--seed', str(args.seed + i * 1000)] + passthrough
        log = open(os.path.join(REPO, 'log', f'parallel-{name}.log'), 'w')
        print(f'radnik {i}: {count} runova, domena '
              f'{environment["PAS_DUAL_ARM_ROS_DOMAIN_ID"]}, particija '
              f'{environment["IGN_PARTITION"]}, jezgre {SLICES[i]["P_CORES"]}'
              f'+{SLICES[i]["REST_CORES"]}')
        workers.append((i, name, subprocess.Popen(
            command, cwd=REPO, env=environment, stdout=log, stderr=subprocess.STDOUT)))
        # Let one finish starting before the next begins; simultaneous Gazebo
        # starts contend for the GPU and time each other out.
        time.sleep(25)

    print(f'\n{len(workers)} radnika vozi. Cekam...')
    for _, name, process in workers:
        process.wait()
        print(f'  {name} gotov')

    # One results.csv, runs renumbered, each row remembering where it came from.
    merged, rows = os.path.join(RESULTS, args.batch), []
    os.makedirs(merged, exist_ok=True)
    fields = None
    for _, name, _ in workers:
        source = os.path.join(RESULTS, name, 'results.csv')
        if not os.path.exists(source):
            continue
        with open(source) as handle:
            reader = csv.DictReader(handle)
            fields = reader.fieldnames
            for row in reader:
                folder = os.path.join(RESULTS, name, row['dir'])
                row['run'] = str(len(rows) + 1)
                new_dir = f'run_{len(rows) + 1:03d}'
                if os.path.isdir(folder):
                    shutil.move(folder, os.path.join(merged, new_dir))
                row['dir'] = new_dir
                rows.append(row)
    if fields:
        with open(os.path.join(merged, 'results.csv'), 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    good = sum(1 for r in rows if r.get('outcome') == 'success')
    print(f'\n{args.batch}: {good}/{len(rows)} uspjesnih, spojeno u {merged}')


if __name__ == '__main__':
    main()
