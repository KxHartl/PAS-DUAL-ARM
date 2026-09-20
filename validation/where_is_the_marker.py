#!/usr/bin/env python3
"""Does the robot see the place marker where it really is?

Three things have now been fixed without the placement error moving at all -
laser odometry, the IMU in the fusion, and AMCL's motion noise. It sits at
21 mm, 19.6 of it sideways, in every series. Whatever produces it is not in the
estimate of where the robot is, so the next place to look is the estimate of
where the marker is.

The robot logs the marker once, in base_link, from the dock. Gazebo knows where
the marker actually is, and where the robot actually was at that moment. The
difference between those two is the answer, and both are already in every bag.

    ./scripts/run_native.sh python3 validation/where_is_the_marker.py <batch>
"""
import argparse
import bisect
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_runs import read_world                                 # noqa: E402

ROBOT = 'dual_arm_robot'
# main_task logs it as: PLACE marker at base_link (0.934, 0.013, 0.672)
READING = re.compile(r'\[(\d+\.\d+)\].*PLACE marker at base_link '
                     r'\(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\)')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(bag):
    """Ground-truth robot poses, on the clock the log is written in.

    Both are WALL time and always were. rcl stamps every log line from the
    system clock even under use_sim_time, and rosbag records arrival on the same
    clock, so the two line up directly. The first version of this converted the
    bag to simulation time and compared it against a wall-clock log stamp, which
    put the marker 250 mm away and had me looking for a fault in the robot.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    poses = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/debug/gz_dynamic_pose':
            msg = deserialize_message(data, get_message(types[topic]))
            for tf in msg.transforms:
                if tf.child_frame_id == ROBOT:
                    poses.append((stamp * 1e-9, tf.transform.translation.x,
                                  tf.transform.translation.y,
                                  yaw_of(tf.transform.rotation)))
    return poses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('batch')
    args = parser.parse_args()
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'results', args.batch)

    print(f'{"run":<9}{"procitano":>22}{"stvarno":>22}{"greska [mm]":>22}')
    rows = []
    for name in sorted(os.listdir(root)):
        folder = os.path.join(root, name)
        log = os.path.join(folder, 'run.log')
        bag = os.path.join(folder, 'bag')
        world = os.path.join(folder, 'world.sdf')
        if not (os.path.isfile(log) and os.path.isdir(bag)):
            continue
        found = READING.search(open(log, errors='replace').read())
        if not found:
            continue
        when, seen_x, seen_y = (float(found.group(1)), float(found.group(2)),
                                float(found.group(3)))
        marker = read_world(world)[2]
        if marker is None:
            continue

        poses = read_bag(bag)
        if not poses:
            continue
        _, rx, ry, ryaw = min(poses, key=lambda p: abs(p[0] - when))
        # The marker in the robot's own frame, from the truth alone.
        dx, dy = marker[0] - rx, marker[1] - ry
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        true_x, true_y = dx * c - dy * s, dx * s + dy * c

        ex, ey = (seen_x - true_x) * 1000.0, (seen_y - true_y) * 1000.0
        rows.append((ex, ey))
        print(f'{name:<9}{seen_x:>11.3f}{seen_y:>11.3f}'
              f'{true_x:>11.3f}{true_y:>11.3f}{ex:>11.1f}{ey:>11.1f}')

    if rows:
        n = len(rows)
        print(f'\nn={n}   naprijed {sum(r[0] for r in rows) / n:+.1f} mm   '
              f'bocno {sum(r[1] for r in rows) / n:+.1f} mm')
        print('Pozitivno bocno znaci da robot marker vidi vise ULIJEVO nego sto jest.')


if __name__ == '__main__':
    main()
