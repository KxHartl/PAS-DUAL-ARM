#!/usr/bin/env python3
"""Run the mission N times and write one row per run.

This exists because of a specific weakness in the results of this project: every
number in them comes from a single run. A single run cannot distinguish "the
system works" from "the system worked once", and the stronger the wording around
such a number, the worse the problem gets. This harness produces the thing that
does distinguish them - repeats, and the spread across them.

Each run is a full, isolated mission: a fresh simulator, headless, on its own
generated world so the repeats are not identical to each other. The harness
never talks to the mission node; it reads the run's log, which is what a person
would read, so the harness cannot accidentally judge a run by something the
robot does not itself report.

    ./scripts/run_native.sh python3 validation/run_batch.py --n 20
    ./scripts/run_native.sh python3 validation/run_batch.py --n 5 --jitter 0.0
    ./scripts/run_native.sh python3 validation/summarize.py

Rows land in validation/results/results.csv; summarize.py turns them into the
table and the chart.
"""
import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(REPO, 'validation', 'results')
WORLD_OUT = os.path.join(REPO, 'validation', 'results', 'worlds')

# What the mission itself prints. Anything not in this list is not evidence.
PATTERNS = {
    'waiting': re.compile(r'WAITING for the user'),
    'complete': re.compile(r'MISSION COMPLETE'),
    'verified': re.compile(r'PLACE VERIFIED.*?(\d+(?:\.\d+)?)\s*mm'),
    'aborted': re.compile(r'Task aborted during:\s*(.+)'),
    'step': re.compile(r'>>> \[(\d+)/(\d+)\]\s*(.+)'),
    'tip': re.compile(r'STEP5d (left|right) tool tip ([\d.]+) mm'),
    'lift': re.compile(r'CARRIAGE LIFT MEASURED left=([\d.]+) right=([\d.]+)'),
    'arrived': re.compile(r'NAV: arrived at "([^"]+)"'),
}

FIELDS = ['run', 'started', 'outcome', 'reason', 'duration_s', 'last_step',
          'last_phase', 'place_error_mm', 'tip_left_mm', 'tip_right_mm',
          'lift_left_m', 'lift_right_m', 'arrivals', 'world', 'log']


def sh(command, **kwargs):
    return subprocess.run(command, shell=isinstance(command, str), **kwargs)


def make_world(index, jitter, seed):
    """A per-run world, so repeats differ in the one thing that matters."""
    os.makedirs(WORLD_OUT, exist_ok=True)
    out = os.path.join(WORLD_OUT, f'run_{index:03d}.sdf')
    command = [sys.executable, os.path.join(REPO, 'scripts', 'gen_world.py'),
               '--out', out, '--cube-jitter', str(jitter), '--seed', str(seed)]
    result = sh(command, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stdout + result.stderr, file=sys.stderr)
        return None
    return out


def parse(log_path):
    """Read the run's own log. No ROS, no assumptions about what happened."""
    found = {'outcome': 'no_log', 'reason': '', 'place_error_mm': '',
             'tip_left_mm': '', 'tip_right_mm': '', 'lift_left_m': '',
             'lift_right_m': '', 'last_step': '', 'last_phase': '', 'arrivals': 0}
    if not os.path.exists(log_path):
        return found
    with open(log_path, errors='replace') as handle:
        for line in handle:
            step = PATTERNS['step'].search(line)
            if step:
                found['last_step'] = f'{step.group(1)}/{step.group(2)}'
                found['last_phase'] = step.group(3).strip()
            if PATTERNS['arrived'].search(line):
                found['arrivals'] += 1
            tip = PATTERNS['tip'].search(line)
            if tip:
                found[f'tip_{tip.group(1)}_mm'] = tip.group(2)
            lift = PATTERNS['lift'].search(line)
            if lift:
                found['lift_left_m'], found['lift_right_m'] = lift.group(1), lift.group(2)
            verified = PATTERNS['verified'].search(line)
            if verified:
                found['place_error_mm'] = verified.group(1)
            abort = PATTERNS['aborted'].search(line)
            if abort:
                found['reason'] = abort.group(1).strip()[:160]
    text_tail_ok = found['place_error_mm'] != ''
    with open(log_path, errors='replace') as handle:
        body = handle.read()
    if PATTERNS['complete'].search(body) and text_tail_ok:
        found['outcome'] = 'success'
    elif found['reason']:
        found['outcome'] = 'aborted'
    elif PATTERNS['waiting'].search(body):
        found['outcome'] = 'stalled'
    else:
        found['outcome'] = 'never_started'
    return found


def wait_for_tf(timeout):
    """Block until map -> odom actually resolves.

    Reaching "WAITING for the user" is not the same as being ready to drive. A
    person takes tens of seconds to read the line and press the button, and by
    then the localiser has warmed up; a harness presses it instantly, and the
    first navigation goal then dies on `Transform data too old when converting
    from map to odom` - the run aborts on its first leg having never moved.
    That failure is an artefact of how fast the harness is, not of the robot,
    and counting it as a failed mission would be wrong.
    """
    try:
        result = subprocess.run(['ros2', 'run', 'tf2_ros', 'tf2_echo', 'map', 'odom'],
                                capture_output=True, text=True, timeout=timeout)
        output = result.stdout
    except subprocess.TimeoutExpired as expired:
        output = (expired.stdout or b'').decode(errors='replace') \
            if isinstance(expired.stdout, bytes) else (expired.stdout or '')
    return 'Translation' in output


def wait_for(log_path, pattern, timeout, proc):
    """Poll the log for a line. The process dying counts as an answer."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if proc.poll() is not None:
            return False
        if os.path.exists(log_path):
            with open(log_path, errors='replace') as handle:
                if pattern.search(handle.read()):
                    return True
        time.sleep(1.0)
    return False


def run_once(index, args):
    started = datetime.now()
    os.makedirs(RESULTS, exist_ok=True)
    log_path = os.path.join(RESULTS, f'run_{index:03d}.log')
    if args.stock_world:
        world = os.path.join(REPO, 'src', 'pas_dual_arm_bringup',
                             'worlds', 'seminar_world.sdf')
    else:
        world = make_world(index, args.jitter, args.seed + index)
    if world is None:
        return {'run': index, 'started': started.isoformat(timespec='seconds'),
                'outcome': 'world_failed', 'reason': 'gen_world.py failed',
                'duration_s': 0, 'world': '', 'log': ''}

    sh(['bash', os.path.join(REPO, 'scripts', 'clean_ros.sh')],
       capture_output=True)

    command = ['ros2', 'launch', 'pas_dual_arm_bringup', 'scenario_mission.launch.py',
               'headless:=true', 'open_rviz:=false', 'gui:=false', 'quiet:=true',
               f'world:={world}']
    print(f'--- run {index}: {" ".join(command[-4:])}')
    began = time.monotonic()
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)

        # The mission waits for a person; here the harness is the person - but
        # a much faster one, so it has to wait for the stack as well as the line.
        if wait_for(log_path, PATTERNS['waiting'], args.startup_timeout, proc):
            if not wait_for_tf(args.settle):
                print(f'    map -> odom never resolved within {args.settle:.0f} s')
            time.sleep(args.settle_extra)
            # `--once` publishes and exits, which can happen before discovery
            # has matched the mission node - the message is then simply lost and
            # the run sits at WAITING until the timeout. Wait for the subscriber
            # and send it more than once.
            sh(['ros2', 'topic', 'pub', '--times', '3', '-w', '1',
                '/mission/start', 'std_msgs/String', '{data: blue}'],
               capture_output=True, timeout=120)
        else:
            print('    never reached "WAITING for the user"')

        end = time.monotonic() + args.run_timeout
        while time.monotonic() < end and proc.poll() is None:
            with open(log_path, errors='replace') as handle:
                body = handle.read()
            if PATTERNS['complete'].search(body) or PATTERNS['aborted'].search(body):
                time.sleep(3.0)          # let the last lines land
                break
            time.sleep(2.0)

        # The launch owns a process group; killing the leader alone leaves
        # Gazebo running and the next run would meet two simulators.
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=10)

    sh(['bash', os.path.join(REPO, 'scripts', 'clean_ros.sh')], capture_output=True)
    row = {'run': index, 'started': started.isoformat(timespec='seconds'),
           'duration_s': round(time.monotonic() - began, 1),
           'world': os.path.basename(world), 'log': os.path.basename(log_path)}
    row.update(parse(log_path))
    print(f'    -> {row["outcome"]}'
          + (f' at {row["last_phase"]}' if row['outcome'] != 'success' else
             f', {row["place_error_mm"]} mm from the marker')
          + f'  ({row["duration_s"]:.0f} s)')
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--n', type=int, default=10, help='how many runs')
    parser.add_argument('--jitter', type=float, default=0.03,
                        help='metres of random box displacement per run (0 = identical worlds)')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--stock-world', action='store_true',
                        help='use the checked-in world instead of a generated one; '
                             'the control case when a failure might be the generator')
    parser.add_argument('--startup-timeout', type=float, default=240.0)
    parser.add_argument('--settle', type=float, default=45.0,
                        help='seconds to wait for map -> odom before starting the mission')
    parser.add_argument('--settle-extra', type=float, default=10.0,
                        help='further settling once the transform exists')
    parser.add_argument('--run-timeout', type=float, default=900.0)
    parser.add_argument('--out', default=os.path.join(RESULTS, 'results.csv'))
    parser.add_argument('--append', action='store_true',
                        help='add to an existing results.csv instead of replacing it')
    args = parser.parse_args()

    os.makedirs(RESULTS, exist_ok=True)
    exists = os.path.exists(args.out) and args.append
    start_at = 1
    if exists:
        with open(args.out) as handle:
            start_at = sum(1 for _ in csv.DictReader(handle)) + 1

    with open(args.out, 'a' if exists else 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS, extrasaction='ignore')
        if not exists:
            writer.writeheader()
        for offset in range(args.n):
            row = run_once(start_at + offset, args)
            writer.writerow(row)
            handle.flush()          # a killed batch still leaves usable results

    print(f'\nWrote {args.out}')
    print('Summarise with:  ./scripts/run_native.sh python3 validation/summarize.py')
    return 0


if __name__ == '__main__':
    sys.exit(main())
