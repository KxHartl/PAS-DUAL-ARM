#!/usr/bin/env python3
"""In the doorway, who is right about where the robot is - the lidar or AMCL?

room_navigator measures both on every transit and steers on neither, because
P-52 compared them over whole runs and the lidar came out worse: 1.87 cm median
against AMCL's 1.53. That comparison asked the wrong question. Over a run the
lidar matches whatever walls it can see; in a doorway it sees two frames a metre
apart, which is a far better conditioned measurement, and the doorway is where
the failures are - in the tail, not the median.

So this asks the doorway question only, from bags already recorded. The two doorways lie on
different axes - the blue one is crossed heading along -y, so its offset is the
robot's map x; the red one is crossed heading along +x, so its offset is y - and
the leg's own name says which is which.

    ./scripts/run_native.sh python3 validation/doorway_offset.py <batch> [<batch>...]
"""
import argparse
import math
import os
import re
import statistics as st
import sys

ROBOT = 'dual_arm_robot'
LINE = re.compile(r'\[(\d+\.\d+)\].*leg \d+/\d+ - ([^:]+):.*'
                  r'in a ([\d.]+) m opening the lidar put the robot '
                  r'([-+][\d.]+) cm off its axis, the localised pose ([-+][\d.]+) cm')


def truth_xy(bag):
    """(bag time, true x, true y) of the robot, from the ground-truth bridge."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic != '/debug/gz_dynamic_pose':
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        for tf in msg.transforms:
            if tf.child_frame_id == ROBOT:
                q = tf.transform.rotation
                out.append((stamp * 1e-9, tf.transform.translation.x,
                            tf.transform.translation.y,
                            math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('batches', nargs='+')
    args = parser.parse_args()
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

    rows = []
    for batch in args.batches:
        root = os.path.join(here, batch)
        for name in sorted(os.listdir(root)):
            folder = os.path.join(root, name)
            log, bag = os.path.join(folder, 'run.log'), os.path.join(folder, 'bag')
            if not (os.path.isfile(log) and os.path.isdir(bag)):
                continue
            found = LINE.findall(open(log, errors='replace').read())
            if not found:
                continue
            poses = truth_xy(bag)
            if not poses:
                continue
            for when, leg, span, lidar, amcl in found:
                # The log line carries the WALL clock, and so does the bag; rcl
                # stamps its lines from the system clock even under sim time.
                at = min(poses, key=lambda p: abs(p[0] - float(when)))
                # Which map axis is across this doorway: the blue one is entered heading
                # along -y so its offset is x, the red one along +x so it is y.
                across = 2 if 'red' in leg else 1
                # How far the robot is turned out of the doorway's own
                # direction. The blue doorway is crossed at -90 deg, the red at
                # 0; a robot that goes through at an angle presents more than
                # its width, which no offset measurement can show.
                aim = 0.0 if 'red' in leg else math.pi / 2.0
                skew = math.atan2(math.sin(at[3] - aim), math.cos(at[3] - aim))
                # The blue doorway is crossed in both directions and the robot's
                # outline is symmetric, so a half turn presents the same width:
                # fold the skew into +/- 90 deg rather than calling a return
                # leg 179 degrees crooked.
                skew = (skew + math.pi / 2.0) % math.pi - math.pi / 2.0
                rows.append((leg.strip(), float(span), float(lidar),
                             float(amcl), at[across] * 100.0, math.degrees(skew)))

    if not rows:
        sys.exit('no doorway comparisons found - was the batch run with debug_truth?')

    # The outline the robot presents to the opening: 0.83 m across and 0.72 m
    # along, turned by the skew. Measured widths, not nominal ones.
    def presented(skew):
        rad = abs(math.radians(skew))
        return 83.0 * math.cos(rad) + 72.0 * math.sin(rad)

    print(f'\n{"prolaz":<7}{"vrata":<7}{"otvor":>7}{"lidar":>8}{"AMCL":>7}'
          f'{"istina":>8}{"|l-i|":>7}{"|a-i|":>7}{"zakret":>8}{"sirina":>8}{"zazor":>7}')
    print(f'{"":<21}{"[cm]":>7}{"[cm]":>8}{"[cm]":>7}{"[cm]":>8}{"[cm]":>7}{"[cm]":>7}'
          f'{"[deg]":>8}{"[cm]":>8}{"[cm]":>7}')
    for i, (leg, span, lidar, amcl, true, skew) in enumerate(rows, 1):
        door = 'crvena' if 'red' in leg else 'plava'
        wide = presented(skew)
        room = (span * 100.0 - wide) / 2.0 - abs(true)
        print(f'{i:<7}{door:<7}{span:>7.3f}{lidar:>8.1f}{amcl:>7.1f}'
              f'{true:>8.1f}{abs(lidar - true):>7.1f}{abs(amcl - true):>7.1f}'
              f'{skew:>8.1f}{wide:>8.1f}{room:>7.1f}')

    le = [abs(r[2] - r[4]) for r in rows]
    ae = [abs(r[3] - r[4]) for r in rows]
    print(f'\nn={len(rows)}')
    print(f'  lidar : medijan {st.median(le):5.2f} cm   max {max(le):5.2f}')
    print(f'  AMCL  : medijan {st.median(ae):5.2f} cm   max {max(ae):5.2f}')
    better = sum(1 for l, a in zip(le, ae) if l < a)
    print(f'  lidar blizi istini u {better}/{len(rows)} prolaza')
    skews = [abs(r[5]) for r in rows]
    print(f'  |zakret| : medijan {st.median(skews):5.2f} deg   max {max(skews):5.2f}')
    rooms = [(r[1] * 100.0 - presented(r[5])) / 2.0 - abs(r[4]) for r in rows]
    print(f'  zazor    : medijan {st.median(rooms):5.2f} cm    min {min(rooms):5.2f}\n')


if __name__ == '__main__':
    main()
