#!/usr/bin/env python3
"""Run the mission N times and write one row per run.

This exists because of a specific weakness in the results of this project: every
number in them comes from a single run. A single run cannot distinguish "the
system works" from "the system worked once", and the stronger the wording around
such a number, the worse the problem gets. This harness produces the thing that
does distinguish them - repeats, and the spread across them.

Each run is a full, isolated mission: a fresh simulator, headless, on its own
generated world so the repeats are not identical to each other.

Two independent records come out of every run, and they answer different
questions:

  * the run's log, parsed here. It says what the robot BELIEVED and reported,
    which is the right basis for "did the mission succeed" - a success has to be
    something the system itself claims, not something the analysis grants it.
  * a rosbag of the run's topics, including Gazebo ground truth. It says what
    actually happened, in numbers, and it is recorded whether or not anyone has
    thought of the question yet. validation/analyze_runs.py turns it into
    measurements: real clearance to the table edge, localisation error against
    ground truth, where the box physically ended up.

The log alone was the earlier design and it was too thin: it can only ever
report quantities someone had already decided to print, in a format meant to be
read on the fly rather than processed.

    ./scripts/run_native.sh python3 validation/run_batch.py --n 20
    ./scripts/run_native.sh python3 validation/run_batch.py --n 5 --jitter 0.0
    ./scripts/run_native.sh python3 validation/run_batch.py --n 20 --no-bag
    ./scripts/run_native.sh python3 validation/run_batch.py --n 5 --batch 2026-09-18_0132
    ./scripts/run_native.sh python3 validation/analyze_runs.py
    ./scripts/run_native.sh python3 validation/summarize.py

Every batch gets its own dated directory, and every run its own folder inside
it, so a number can always be traced back to the exact world, log and recording
it came from:

    validation/results/
        latest -> 2026-09-18_0132/
        2026-09-18_0132/
            results.csv          one row per run (this file's output)
            metrics.csv          measured from the bags (analyze_runs.py)
            run_001/
                run.log          everything the run printed
                world.sdf        the exact world it was driven in
                bag/             the recorded topics
                metrics.json     that run's measurements
"""
import argparse
import csv
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(REPO, 'validation', 'results')
LATEST = os.path.join(RESULTS, 'latest')

# Recorded per run. Cameras and point clouds are left out on purpose: they are
# the bulk of the data and nothing in the analysis reads them, while everything
# below is either a measurement or the state needed to interpret one.
BAG_TOPICS = [
    '/clock',                       # sim time itself: a run can stall or step back
    '/debug/gz_dynamic_pose',       # ground truth: robot, box, anything that moves
    '/debug/loc_error',             # its comparison against TF, as computed live
    '/amcl_pose', '/particle_cloud',
    '/tf', '/tf_static',
    '/base_controller/odom',
    '/laser_odom',                  # laser scan matcher, when laser_odometry:=true
    '/base_imu',                    # 100 Hz base IMU, when the Imu system is in the world
    '/scan', '/scan_filtered',
    '/joint_states',
    '/cmd_vel', '/cmd_vel_safe',
    '/plan', '/local_plan',
    '/map', '/nav_graph',
    '/room_navigator/status', '/room_navigator/goto',
    '/mission/task_status', '/mission/start', '/mission/carried_points',
    '/aruco_box/state',
    '/contact/left_left_tip', '/contact/left_right_tip',
    '/contact/right_left_tip', '/contact/right_right_tip',
]

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
    'controller_up': re.compile(r'Configured and activated'),
}

# Eight controllers have to be active before anything can drive: the base one
# publishes the wheel odometry the whole localisation chain hangs off.
CONTROLLERS = 8

FIELDS = ['run', 'started', 'outcome', 'reason', 'duration_s', 'last_step',
          'last_phase', 'place_error_mm', 'tip_left_mm', 'tip_right_mm',
          'lift_left_m', 'lift_right_m', 'arrivals', 'dir', 'bag']


def sh(command, **kwargs):
    return subprocess.run(command, shell=isinstance(command, str), **kwargs)


def run_dir(batch, index):
    """One folder per run: its world, its log, its recording, its numbers."""
    path = os.path.join(batch, f'run_{index:03d}')
    os.makedirs(path, exist_ok=True)
    return path


def point_latest_at(batch):
    """`latest` is how every other script finds the batch just recorded."""
    try:
        if os.path.islink(LATEST) or os.path.exists(LATEST):
            os.remove(LATEST)
        os.symlink(os.path.basename(batch), LATEST)
    except OSError as error:
        print(f'could not update {LATEST}: {error}', file=sys.stderr)


def make_world(folder, jitter, seed):
    """A per-run world, so repeats differ in the one thing that matters."""
    out = os.path.join(folder, 'world.sdf')
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


def wait_for_tf(timeout, frame='base_footprint'):
    """Block until map -> base_footprint actually resolves.

    Reaching "WAITING for the user" is not the same as being ready to drive. A
    person takes tens of seconds to read the line and press the button, and by
    then the localiser has warmed up; a harness presses it instantly, and the
    first navigation goal then dies - the run aborts on its first leg having
    never moved. That failure is an artefact of how fast the harness is, not of
    the robot, and counting it as a failed mission would be wrong.

    The frame is the one `room_navigator` demands before it accepts a
    destination, not `odom`: map -> odom can exist while the robot itself is
    still missing from the tree, and the earlier check passed in exactly that
    state.
    """
    try:
        result = subprocess.run(['ros2', 'run', 'tf2_ros', 'tf2_echo', 'map', frame],
                                capture_output=True, text=True, timeout=timeout)
        output = result.stdout
    except subprocess.TimeoutExpired as expired:
        output = (expired.stdout or b'').decode(errors='replace') \
            if isinstance(expired.stdout, bytes) else (expired.stdout or '')
    return 'Translation' in output


def wait_for_controllers(log_path, timeout, proc):
    """All eight controllers active, or the stack is not the robot we measure.

    Run 1 of batch 2026-09-18_0143 came up with seven: `base_controller` never
    loaded, so there was no wheel odometry, so AMCL never localised and the
    navigator refused the first destination. Recording that as a failed mission
    would put a startup race into the success rate.
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if proc.poll() is not None:
            return 0
        if os.path.exists(log_path):
            with open(log_path, errors='replace') as handle:
                found = len(PATTERNS['controller_up'].findall(handle.read()))
            if found >= CONTROLLERS:
                return found
        time.sleep(2.0)
    with open(log_path, errors='replace') as handle:
        return len(PATTERNS['controller_up'].findall(handle.read()))


def in_our_partition(pid):
    """Does this process belong to the partition we are running in?

    Without IGN_PARTITION set there is only one simulation on the machine and
    every simulator is ours. With it, another worker's Gazebo is not - and
    treating it as ours is what kept a second worker waiting forever for a
    quiet machine that was never going to be quiet.
    """
    partition = os.environ.get('IGN_PARTITION')
    if not partition:
        return True
    try:
        with open(f'/proc/{pid}/environ', 'rb') as handle:
            return f'IGN_PARTITION={partition}'.encode() in handle.read().split(b'\0')
    except OSError:
        return False


def wait_for_quiet(timeout=60.0):
    """No simulator of OURS left running before the next one starts.

    clean_ros.sh sends the kill and waits a second; Gazebo takes longer than
    that to go, and a run that starts on top of a dying one meets two
    controller managers. Polling until the process is actually gone is the
    difference between independent runs and a batch that poisons itself.

    Ours, not everyone's. This used to count every simulator on the machine,
    which is right when there is one batch and fatal when there are three: the
    second worker sat out its whole timeout waiting for the first to finish.
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        alive = [pid for pid in subprocess.run(
            ['pgrep', '-f', r'ign gazebo|gz sim'],
            capture_output=True, text=True).stdout.split()
            if in_our_partition(pid)]
        if not alive:
            return True
        time.sleep(2.0)
    return False


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


def start_bag(folder):
    """Record the run's topics.

    rosbag2 keeps looking for the listed topics while it runs, so this starts
    with the launch rather than after it: the arms fold and the localiser
    converges before anyone presses the button, and those are measurements too.
    """
    out = os.path.join(folder, 'bag')
    if os.path.exists(out):
        subprocess.run(['rm', '-rf', out])
    proc = subprocess.Popen(['ros2', 'bag', 'record', '-o', out] + BAG_TOPICS,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    return proc, out


def stop_bag(proc):
    """SIGINT, because anything harder loses the metadata and the last chunk."""
    if proc is None or proc.poll() is not None:
        return
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=10)


def run_once(index, args, batch):
    started = datetime.now()
    folder = run_dir(batch, index)
    log_path = os.path.join(folder, 'run.log')
    if args.stock_world:
        # Copied in, not referenced: a run folder that does not contain the
        # world it used cannot be re-measured once the checked-in world changes.
        world = os.path.join(folder, 'world.sdf')
        shutil.copyfile(os.path.join(REPO, 'src', 'pas_dual_arm_bringup',
                                     'worlds', 'seminar_world.sdf'), world)
    else:
        world = make_world(folder, args.jitter, args.seed + index)
    if world is None:
        return {'run': index, 'started': started.isoformat(timespec='seconds'),
                'outcome': 'world_failed', 'reason': 'gen_world.py failed',
                'duration_s': 0, 'dir': os.path.basename(folder), 'bag': ''}

    sh(['bash', os.path.join(REPO, 'scripts', 'clean_ros.sh')],
       capture_output=True)
    if not wait_for_quiet():
        print('    a simulator is still running after clean_ros.sh; '
              'not starting a run on top of it')
        return {'run': index, 'started': started.isoformat(timespec='seconds'),
                'outcome': 'not_ready', 'reason': 'previous simulator still alive',
                'duration_s': 0, 'dir': os.path.basename(folder), 'bag': ''}

    command = ['ros2', 'launch', 'pas_dual_arm_bringup', 'scenario_mission.launch.py',
               'headless:=true', 'open_rviz:=false', 'gui:=false', 'quiet:=true',
               f'debug_truth:={str(args.truth).lower()}', f'world:={world}',
               f'leg_speed:={str(args.leg_speed).lower()}',
               f'laser_odometry:={str(args.laser_odometry).lower()}']
    print(f'--- run {index}: {" ".join(command[-4:])}')
    began = time.monotonic()
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        bag_proc, bag_path = start_bag(folder) if args.bag else (None, '')

        # The mission waits for a person; here the harness is the person - but
        # a much faster one, so it has to wait for the stack as well as the line.
        not_ready = ''
        if wait_for(log_path, PATTERNS['waiting'], args.startup_timeout, proc):
            active = wait_for_controllers(log_path, args.settle, proc)
            if active < CONTROLLERS:
                not_ready = f'only {active}/{CONTROLLERS} controllers active'
                print(f'    {not_ready}')
            if not wait_for_tf(args.settle):
                not_ready = not_ready or 'map -> base_footprint never resolved'
                print(f'    map -> base_footprint never resolved within '
                      f'{args.settle:.0f} s')
            # Every run starts its processes afresh, so the affinity has to be
            # set afresh too. Measured before doing this: controller_server
            # missed its 20 Hz deadline 649 times in a run, and given a core to
            # itself it turned out to want 100 % of one - it was competing, not
            # idling. See scripts/pin_cores.sh for who goes where and why.
            if args.pin:
                sh(['bash', os.path.join(os.path.dirname(os.path.dirname(
                    os.path.abspath(__file__))), 'scripts', 'pin_cores.sh')],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(args.settle_extra)
            # `--once` publishes and exits, which can happen before discovery
            # has matched the mission node - the message is then simply lost and
            # the run sits at WAITING until the timeout. Wait for the subscriber
            # and send it more than once.
            #
            # And check that it LANDED. With three simulations on three DDS
            # domains discovering at once, one worker sat at WAITING for 538 s
            # after its start had been sent - the publisher had matched nothing
            # and nobody noticed, and the run was scored "stalled" as though the
            # robot had done something wrong. So send, look for the mission
            # leaving WAITING, and send again until it has.
            started_pattern = re.compile(r'\[3/8\]')
            for attempt in range(6):
                try:
                    sh(['ros2', 'topic', 'pub', '--times', '3', '-w', '1',
                        '/mission/start', 'std_msgs/String', '{data: blue}'],
                       capture_output=True, timeout=45)
                except subprocess.TimeoutExpired:
                    pass
                deadline = time.monotonic() + 20.0
                landed = False
                while time.monotonic() < deadline and proc.poll() is None:
                    with open(log_path, errors='replace') as handle:
                        if started_pattern.search(handle.read()):
                            landed = True
                            break
                    time.sleep(2.0)
                if landed:
                    break
                print(f'    start not received (attempt {attempt + 1}); sending again')
        else:
            not_ready = 'never reached "WAITING for the user"'
            print(f'    {not_ready}')

        end = time.monotonic() + args.run_timeout
        while time.monotonic() < end and proc.poll() is None:
            with open(log_path, errors='replace') as handle:
                body = handle.read()
            if PATTERNS['complete'].search(body) or PATTERNS['aborted'].search(body):
                time.sleep(3.0)          # let the last lines land
                break
            time.sleep(2.0)

        # The recorder goes first: it is the thing that must see the last
        # message, and it cannot once its publishers are gone.
        stop_bag(bag_proc)

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
           'dir': os.path.basename(folder), 'bag': 'yes' if bag_path else ''}
    row.update(parse(log_path))
    # A stack that never came up is not a mission that failed. Keeping the two
    # apart is the whole point of a success rate.
    if not_ready and row['outcome'] != 'success':
        row['outcome'] = 'not_ready'
        row['reason'] = not_ready
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
    parser.add_argument('--batch', default=None,
                        help='name of the batch directory under validation/results '
                             '(default: the date and time it was started). Naming an '
                             'existing batch adds to it')
    parser.add_argument('--no-bag', dest='bag', action='store_false',
                        help='do not record a rosbag; the run is then judged only '
                             'by what it printed, and nothing can be re-measured '
                             'from it afterwards')
    parser.add_argument('--no-pin', dest='pin', action='store_false',
                        help='do not pin the physics and the controllers to '
                             'cores of their own')
    parser.add_argument('--no-leg-speed', dest='leg_speed', action='store_false',
                        help='leave the speed to the speed mask instead of '
                             'setting it per leg (tried with DWB: fails)')
    parser.add_argument('--laser-odometry', action='store_true',
                        help='take odom -> base_footprint from the laser (D-25)')
    parser.add_argument('--no-truth', dest='truth', action='store_false',
                        help='do not bridge Gazebo ground truth; measurements '
                             'against where the robot actually was are then lost')
    args = parser.parse_args()

    name = args.batch or datetime.now().strftime('%Y-%m-%d_%H%M')
    batch = name if os.path.isabs(name) else os.path.join(RESULTS, name)
    os.makedirs(batch, exist_ok=True)
    point_latest_at(batch)

    # Numbering continues within a batch, so run_007 is the seventh run of this
    # batch and nothing else. Starting a new batch starts the count again.
    out = os.path.join(batch, 'results.csv')
    resuming = os.path.exists(out)
    start_at = 1
    if resuming:
        with open(out) as handle:
            start_at = sum(1 for _ in csv.DictReader(handle)) + 1
        print(f'Adding to {batch} (continuing at run {start_at})')
    else:
        print(f'Batch {batch}')

    with open(out, 'a' if resuming else 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS, extrasaction='ignore')
        if not resuming:
            writer.writeheader()
        for offset in range(args.n):
            row = run_once(start_at + offset, args, batch)
            writer.writerow(row)
            handle.flush()          # a killed batch still leaves usable results

    print(f'\nWrote {out}')
    if args.bag:
        print('Measure with:    ./scripts/run_native.sh python3 validation/analyze_runs.py')
    print('Summarise with:  ./scripts/run_native.sh python3 validation/summarize.py')
    return 0


if __name__ == '__main__':
    sys.exit(main())
