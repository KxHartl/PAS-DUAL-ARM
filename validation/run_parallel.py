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
    # Two P-cores for the controller, not one. With one each, DWB - 15 x 25 x
    # 25 = 9375 trajectories every 50 ms - missed its deadline 156 times in a
    # run and could not get through a doorway. The laser odometry moves to an
    # E-core of its own instead: it works on scans, not on the control clock.
    {'P_CORES': '0-2',   'ALONE_A_CORE': '3-4',   'ALONE_B_CORE': '16', 'REST_CORES': '17-18'},
    {'P_CORES': '5-7',   'ALONE_A_CORE': '8-9',   'ALONE_B_CORE': '19', 'REST_CORES': '20-21'},
    {'P_CORES': '10-12', 'ALONE_A_CORE': '13-14', 'ALONE_B_CORE': '22', 'REST_CORES': '15,23'},
]


# With two workers each gets roughly half the machine rather than a third, which
# is closer to the single simulation the mission is delivered as. Three workers
# showed a stall in about one run in twelve with bt_navigator starved ("Behavior
# Tree tick rate 100.00 was exceeded") on legs no parameter under test touched.
SLICES_TWO = [
    {'P_CORES': '0-3',  'ALONE_A_CORE': '4-5',   'ALONE_B_CORE': '6',  'REST_CORES': '7,16-19'},
    {'P_CORES': '8-11', 'ALONE_A_CORE': '12-13', 'ALONE_B_CORE': '14', 'REST_CORES': '15,20-23'},
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
        environment.update((SLICES_TWO if args.workers == 2 else SLICES)[i])
        # A domain each. ROS 2 domains are 0-101 and neighbouring ones share
        # ports, so they are spread rather than adjacent.
        #
        # BOTH names, and that is not belt and braces. run_batch calls
        # `ros2 launch` directly rather than through run_native.sh, so
        # ROS_DOMAIN_ID is read from its own environment and
        # PAS_DUAL_ARM_ROS_DOMAIN_ID is never consulted. Setting only the second
        # left two workers sharing domain 5 - the one thing this file exists to
        # prevent - while every log looked right.
        domain = str(11 + i * 10)
        environment['PAS_DUAL_ARM_ROS_DOMAIN_ID'] = domain
        environment['ROS_DOMAIN_ID'] = domain
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
              f'{environment["IGN_PARTITION"]}, jezgre {environment["P_CORES"]}'
              f'+{environment["REST_CORES"]}')
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
