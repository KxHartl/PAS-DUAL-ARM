#!/usr/bin/env python3
"""Measure the recorded runs against the world they were driven in.

run_batch.py judges a run by what the robot printed, because a success has to be
something the system itself claims. This script is the other half: it opens the
rosbag of each run and measures what actually happened, against the geometry of
that run's own world file. Nothing here reads the log, and nothing here is a
quantity someone decided in advance to print.

It produces the numbers the report needs:

  * clearance to the table edge while crossing a room, which is what the
    "0.835 m" figure in the report claims. That figure came from planning on the
    saved map, outside the simulator; this measures the driven path instead, and
    says separately what the clearance was on the deliberate approach to a
    table, where a metre of clearance would mean the robot never arrived.
  * clearance to the walls, and separately the squeeze through each doorway.
  * localisation error: AMCL's pose against the simulator's ground truth.
  * where the box physically ended up, measured against the marker in the world
    file rather than against the robot's own report of its placement.

    ./scripts/run_native.sh python3 validation/analyze_runs.py
    ./scripts/run_native.sh python3 validation/analyze_runs.py --runs 3 4 5

Rows land in validation/results/metrics.csv, one per run, alongside the
results.csv that run_batch.py writes.
"""
import argparse
import csv
import json
import math
import os
import re
import statistics
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(REPO, 'validation', 'results')
BAGS = os.path.join(RESULTS, 'bags')
WORLDS = os.path.join(RESULTS, 'worlds')
STOCK_WORLD = os.path.join(REPO, 'src', 'pas_dual_arm_bringup', 'worlds',
                           'seminar_world.sdf')

# The Gazebo model names, as spawned. Ground truth arrives as a TFMessage whose
# child_frame_id is the model (Fortress names links `<model>::<link>`).
ROBOT_MODEL = 'dual_arm_robot'
CUBE_MODEL = 'aruco_box'

# A leg that ends this close to a table edge was an approach to that table, not
# a crossing of the room. The dock pose puts the robot's front 10 cm from the
# plate, so anything under half a metre cannot be anything else.
APPROACH_END_M = 0.60

# ...but only the tail of such a leg is the approach. The leg starts in another
# room and crosses this one, and that crossing is exactly what the report's
# figure is about, so the split is made at the last moment the robot was still
# this far from the table it is about to dock at. Above the 0.835 m the report
# claims, so the claim is measured on the crossing and not on the docking.
APPROACH_CORRIDOR_M = 1.20

FIELDS = ['run', 'truth_samples', 'drive_time_s', 'path_len_m',
          'table_clear_transit_min_m', 'table_clear_approach_min_m',
          'wall_clear_open_min_m', 'door_passes', 'door_clear_min_m',
          'amcl_err_max_cm', 'amcl_err_rms_cm', 'amcl_yaw_max_deg',
          'cube_lift_m', 'place_err_truth_mm', 'place_dx_mm', 'place_dy_mm']


# --------------------------------------------------------------------- geometry
def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Box:
    """An axis-aligned footprint in the ground plane."""

    def __init__(self, cx, cy, sx, sy, name=''):
        self.name = name
        self.min_x, self.max_x = cx - sx / 2.0, cx + sx / 2.0
        self.min_y, self.max_y = cy - sy / 2.0, cy + sy / 2.0
        self.cx, self.cy, self.sx, self.sy = cx, cy, sx, sy

    def distance(self, x, y):
        """Distance from a point to the box; 0 inside it."""
        dx = max(self.min_x - x, 0.0, x - self.max_x)
        dy = max(self.min_y - y, 0.0, y - self.max_y)
        return math.hypot(dx, dy)


POSE = re.compile(r'<pose>([^<]+)</pose>')
SIZE = re.compile(r'<box>\s*<size>([^<]+)</size>')


def _pose(text, default=(0.0, 0.0)):
    match = POSE.search(text)
    if not match:
        return default
    parts = [float(v) for v in match.group(1).split()]
    return parts[0], parts[1]


def read_world(path):
    """Tables, walls and the destination marker, from the run's own world file.

    The world is regenerated per run, so the geometry a run is measured against
    has to come from that run's file and not from a constant in this script.
    """
    text = open(path).read()
    tables, walls, marker = [], [], None

    for match in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.S):
        name, body = match.group(1), match.group(2)
        head = body.split('<link', 1)[0]
        mx, my = _pose(head)

        if name.endswith('_table'):
            # The top slab is the edge the report's figure is measured from:
            # the legs are what the lidar sees, the slab is what the robot hits.
            top = re.search(r'<collision name="top_c">(.*?)</collision>', body, re.S)
            if top:
                lx, ly = _pose(top.group(1))
                size = SIZE.search(top.group(1))
                sx, sy = (float(v) for v in size.group(1).split()[:2])
                tables.append(Box(mx + lx, my + ly, sx, sy, name))
        elif name == 'place_marker':
            marker = (mx, my)
        elif name == 'rooms':
            for link in re.finditer(r'<link name="([^"]+)">(.*?)</link>', body, re.S):
                lname, lbody = link.group(1), link.group(2)
                lx, ly = _pose(lbody)
                size = SIZE.search(lbody)
                if not size:
                    continue
                sx, sy = (float(v) for v in size.group(1).split()[:2])
                walls.append(Box(mx + lx, my + ly, sx, sy, lname))
    return tables, walls, marker


def find_doors(walls, min_gap=0.30):
    """Doorways, as the gaps between collinear wall segments.

    A door is not written down anywhere in the world file; it is the absence of
    wall. Grouping the segments by the line they lie on and looking at what is
    missing recovers them without trusting a naming convention.
    """
    doors = []
    lines = {}
    for wall in walls:
        if wall.sx >= wall.sy:                      # runs along x
            key = ('x', round(wall.cy, 2))
            lines.setdefault(key, []).append((wall.min_x, wall.max_x))
        else:                                       # runs along y
            key = ('y', round(wall.cx, 2))
            lines.setdefault(key, []).append((wall.min_y, wall.max_y))

    for (axis, fixed), spans in lines.items():
        spans.sort()
        reach = spans[0][1]
        for low, high in spans[1:]:
            gap = low - reach
            if gap >= min_gap:
                doors.append({'axis': axis, 'fixed': fixed,
                              'centre': (reach + low) / 2.0, 'width': gap})
            reach = max(reach, high)
    return doors


def door_state(doors, x, y, reach=0.70):
    """Which doorway the robot is in, if any, and its clearance to the jambs.

    `reach` is how far either side of the wall counts as being in the doorway:
    the robot is 0.821 m long, so it is still threading the gap for a good half
    metre before and after the wall plane itself.
    """
    for index, door in enumerate(doors):
        along, across = (x, y) if door['axis'] == 'x' else (y, x)
        if abs(across - door['fixed']) > reach:
            continue
        if abs(along - door['centre']) > door['width'] / 2.0:
            continue
        return index, door['width'] / 2.0 - abs(along - door['centre'])
    return None, None


# ------------------------------------------------------------------------- bag
def read_bag(path):
    """Everything this script measures, pulled out of one bag in a single pass."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    out = {'truth': [], 'cube': [], 'amcl': [], 'status': []}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in types:
            continue
        seconds = stamp / 1e9
        message = deserialize_message(data, get_message(types[topic]))

        if topic == '/debug/gz_dynamic_pose':
            for transform in message.transforms:
                child = transform.child_frame_id
                if child == ROBOT_MODEL:
                    t = transform.transform
                    out['truth'].append((seconds, t.translation.x, t.translation.y,
                                         quat_to_yaw(t.rotation.x, t.rotation.y,
                                                     t.rotation.z, t.rotation.w)))
                elif child == CUBE_MODEL:
                    t = transform.transform
                    out['cube'].append((seconds, t.translation.x, t.translation.y,
                                        t.translation.z))
        elif topic == '/amcl_pose':
            pose = message.pose.pose
            out['amcl'].append((seconds, pose.position.x, pose.position.y,
                                quat_to_yaw(pose.orientation.x, pose.orientation.y,
                                            pose.orientation.z, pose.orientation.w)))
        elif topic == '/room_navigator/status':
            try:
                status = json.loads(message.data)
            except ValueError:
                continue
            out['status'].append((seconds, status.get('state', ''),
                                  str(status.get('detail', ''))))
    return out


def nearest_in_time(series, when):
    """The sample closest in time to `when`; the series are at different rates."""
    if not series:
        return None
    best, best_gap = None, None
    for sample in series:
        gap = abs(sample[0] - when)
        if best_gap is None or gap < best_gap:
            best, best_gap = sample, gap
        elif sample[0] > when and gap > best_gap:
            break
    return best if best_gap is not None and best_gap <= 1.0 else None


# --------------------------------------------------------------------- measures
def legs_from_status(status, end_time):
    """Split the run into navigation legs, as the navigator reported them."""
    legs = []
    for index, (when, state, detail) in enumerate(status):
        if state not in ('driving', 'arrived'):
            continue
        if state != 'driving':
            continue
        finish = status[index + 1][0] if index + 1 < len(status) else end_time
        legs.append((when, finish, detail))
    return legs


def analyse(run, bag_path, world_path):
    data = read_bag(bag_path)
    truth = data['truth']
    row = {'run': run, 'truth_samples': len(truth)}
    if len(truth) < 2:
        return row

    tables, walls, marker = read_world(world_path)
    doors = find_doors(walls)

    # Per-sample clearances, computed once and sliced afterwards.
    samples = []
    for when, x, y, yaw in truth:
        table_gap = min((box.distance(x, y) for box in tables), default=float('nan'))
        wall_gap = min((box.distance(x, y) for box in walls), default=float('nan'))
        door_index, door_gap = door_state(doors, x, y)
        samples.append({'t': when, 'x': x, 'y': y, 'yaw': yaw,
                        'table': table_gap, 'wall': wall_gap,
                        'door': door_index, 'door_gap': door_gap})

    row['drive_time_s'] = round(samples[-1]['t'] - samples[0]['t'], 1)
    row['path_len_m'] = round(sum(
        math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        for a, b in zip(samples, samples[1:])), 2)

    # Transit versus approach. A leg whose last pose is right up against a table
    # was an approach to it; the report's figure is about crossing a room.
    legs = legs_from_status(data['status'], samples[-1]['t'])
    transit, approach = [], []
    for start, finish, _detail in legs:
        span = [s for s in samples if start <= s['t'] <= finish]
        if not span:
            continue
        if span[-1]['table'] > APPROACH_END_M:
            transit.extend(span)                    # a crossing, start to finish
            continue
        # A docking leg: everything up to the last time the robot was still
        # outside the corridor is the crossing, the tail is the approach.
        cut = len(span)
        for index in range(len(span) - 1, -1, -1):
            if span[index]['table'] > APPROACH_CORRIDOR_M:
                cut = index + 1
                break
        else:
            cut = 0
        transit.extend(span[:cut])
        approach.extend(span[cut:])
    if not legs:                    # no status in the bag: fall back to geometry
        transit = [s for s in samples if s['table'] > APPROACH_CORRIDOR_M]
        approach = [s for s in samples if s['table'] <= APPROACH_CORRIDOR_M]

    if transit:
        row['table_clear_transit_min_m'] = round(min(s['table'] for s in transit), 3)
        open_space = [s for s in transit if s['door'] is None]
        if open_space:
            row['wall_clear_open_min_m'] = round(min(s['wall'] for s in open_space), 3)
    if approach:
        row['table_clear_approach_min_m'] = round(min(s['table'] for s in approach), 3)

    in_door = [s for s in samples if s['door'] is not None]
    row['door_passes'] = len({s['door'] for s in in_door})
    if in_door:
        row['door_clear_min_m'] = round(min(s['door_gap'] for s in in_door), 3)

    # Localisation: what AMCL believed, against where the robot was.
    errors, yaw_errors = [], []
    for when, ax, ay, ayaw in data['amcl']:
        actual = nearest_in_time(truth, when)
        if actual is None:
            continue
        errors.append(math.hypot(ax - actual[1], ay - actual[2]))
        yaw_errors.append(abs(wrap(ayaw - actual[3])))
    if errors:
        row['amcl_err_max_cm'] = round(max(errors) * 100.0, 1)
        row['amcl_err_rms_cm'] = round(
            math.sqrt(sum(e * e for e in errors) / len(errors)) * 100.0, 1)
        row['amcl_yaw_max_deg'] = round(math.degrees(max(yaw_errors)), 1)

    # The box: how far it was lifted, and where it physically came to rest.
    cube = data['cube']
    if cube:
        row['cube_lift_m'] = round(max(c[3] for c in cube) - cube[0][3], 3)
        moved = math.hypot(cube[-1][1] - cube[0][1], cube[-1][2] - cube[0][2]) > 0.05
        # A run that never moved the box has no placement to measure; the
        # distance from where it still sits to the marker is not a placement
        # error, and reporting it as one would be a lie in the safe direction.
        if marker is not None and moved:
            settled = cube[-1]
            row['place_dx_mm'] = round((settled[1] - marker[0]) * 1000.0, 1)
            row['place_dy_mm'] = round((settled[2] - marker[1]) * 1000.0, 1)
            row['place_err_truth_mm'] = round(
                math.hypot(settled[1] - marker[0], settled[2] - marker[1]) * 1000.0, 1)
    return row


# ------------------------------------------------------------------------- main
def summarise(rows):
    """Median and range per column; a single number here would hide the point."""
    print(f'\n{len(rows)} run(s) measured\n')
    width = max(len(f) for f in FIELDS)
    for field in FIELDS:
        if field in ('run', 'truth_samples'):
            continue
        values = [row[field] for row in rows
                  if isinstance(row.get(field), (int, float))]
        if not values:
            continue
        median = statistics.median(values)
        print(f'  {field:<{width}}  median {median:8.3f}   '
              f'min {min(values):8.3f}   max {max(values):8.3f}   n={len(values)}')

    claim = [row['table_clear_transit_min_m'] for row in rows
             if isinstance(row.get('table_clear_transit_min_m'), (int, float))]
    if claim:
        print(f'\n  The report states 0.835 m from the table edge in open room space.')
        print(f'  Driven, crossing a room: min {min(claim):.3f} m, '
              f'median {statistics.median(claim):.3f} m, over {len(claim)} run(s).')
        print('  The approach to a table is excluded and reported separately; '
              'the robot has to reach the table to pick anything up.')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bags', default=BAGS)
    parser.add_argument('--runs', type=int, nargs='*',
                        help='run numbers to measure (default: every recorded bag)')
    parser.add_argument('--out', default=os.path.join(RESULTS, 'metrics.csv'))
    args = parser.parse_args()

    if not os.path.isdir(args.bags):
        print(f'No bags in {args.bags}. Record some first:\n'
              '  ./scripts/run_native.sh python3 validation/run_batch.py --n 3',
              file=sys.stderr)
        return 1

    found = sorted(name for name in os.listdir(args.bags)
                   if re.fullmatch(r'run_\d+', name))
    rows = []
    for name in found:
        run = int(name.split('_')[1])
        if args.runs and run not in args.runs:
            continue
        world = os.path.join(WORLDS, f'run_{run:03d}.sdf')
        if not os.path.exists(world):
            world = STOCK_WORLD
        try:
            row = analyse(run, os.path.join(args.bags, name), world)
        except Exception as error:                      # one bad bag is not the batch
            print(f'run {run}: {type(error).__name__}: {error}', file=sys.stderr)
            continue
        rows.append(row)
        print(f'run {run}: {row.get("truth_samples", 0)} truth samples, '
              f'transit clearance {row.get("table_clear_transit_min_m", "-")} m, '
              f'door {row.get("door_clear_min_m", "-")} m, '
              f'placed {row.get("place_err_truth_mm", "-")} mm from the marker')

    if not rows:
        print('Nothing measured.', file=sys.stderr)
        return 1

    with open(args.out, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    summarise(rows)
    print(f'\nWrote {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
