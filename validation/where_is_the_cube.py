#!/usr/bin/env python3
"""Do the wrist cameras see the cube's centre where it actually is?

The placement error is 20 mm, 19.6 of it sideways, and it has survived laser
odometry, the IMU and AMCL's noise model untouched (P-54). It is not in the
estimate of where the robot is. That leaves two places, and this measures one of
them.

At STEP5a the wrist cameras give the cube's centre in base_link, and that number
is what the whole rest of the mission uses: it is written into `_hold` when the
hands close and carried forward through the tool tip's transform, because once
the pads are on the markers no camera can see the cube again. If it is off, the
cube is placed off by the same amount, and nothing downstream can notice.

Gazebo knows where the cube actually was. Both clocks are wall time - rcl stamps
log lines from the system clock even under use_sim_time, and rosbag records
arrival on the same clock - so the two line up directly.

    ./scripts/run_native.sh python3 validation/where_is_the_cube.py <batch>
"""
import argparse
import math
import os
import statistics as st

ROBOT = 'dual_arm_robot'
CUBE = 'aruco_box'
SEEN = __import__('re').compile(
    r'\[(\d+\.\d+)\].*WRIST CAMERAS: markers ([\d.]+) m apart, cube centre '
    r'\(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\)')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    robot, cube = [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic != '/debug/gz_dynamic_pose':
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        for tf in msg.transforms:
            t = tf.transform.translation
            if tf.child_frame_id == ROBOT:
                robot.append((stamp * 1e-9, t.x, t.y, yaw_of(tf.transform.rotation)))
            elif tf.child_frame_id == CUBE:
                cube.append((stamp * 1e-9, t.x, t.y))
    return robot, cube


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('batch')
    args = parser.parse_args()
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results', args.batch)

    print(f'{"run":<9}{"kamere":>17}{"istina":>17}{"greska [mm]":>19}{"span":>8}')
    rows = []
    for name in sorted(os.listdir(root)):
        folder = os.path.join(root, name)
        log, bag = os.path.join(folder, 'run.log'), os.path.join(folder, 'bag')
        if not (os.path.isfile(log) and os.path.isdir(bag)):
            continue
        found = SEEN.search(open(log, errors='replace').read())
        if not found:
            continue
        when, span = float(found.group(1)), float(found.group(2))
        seen_x, seen_y = float(found.group(3)), float(found.group(4))
        robot, cube = read(bag)
        if not robot or not cube:
            continue
        _, rx, ry, ryaw = min(robot, key=lambda p: abs(p[0] - when))
        _, cx, cy = min(cube, key=lambda p: abs(p[0] - when))
        dx, dy = cx - rx, cy - ry
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        true_x, true_y = dx * c - dy * s, dx * s + dy * c
        ex, ey = (seen_x - true_x) * 1000.0, (seen_y - true_y) * 1000.0
        rows.append((ex, ey, span))
        print(f'{name:<9}{seen_x:>8.3f}{seen_y:>9.3f}{true_x:>8.3f}{true_y:>9.3f}'
              f'{ex:>10.1f}{ey:>9.1f}{span:>8.3f}')

    if rows:
        print(f'\nn={len(rows)}')
        print(f'  naprijed: medijan {st.median(r[0] for r in rows):+6.1f} mm   '
              f'raspon {min(r[0] for r in rows):+.1f} .. {max(r[0] for r in rows):+.1f}')
        print(f'  bocno   : medijan {st.median(r[1] for r in rows):+6.1f} mm   '
              f'raspon {min(r[1] for r in rows):+.1f} .. {max(r[1] for r in rows):+.1f}')
        print(f'  span    : medijan {st.median(r[2] for r in rows):.3f} m (stvarno 0.300)')
        print('\nPozitivno bocno = kamere vide kocku vise ULIJEVO nego sto jest.')


if __name__ == '__main__':
    main()
