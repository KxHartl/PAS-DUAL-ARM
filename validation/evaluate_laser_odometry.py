#!/usr/bin/env python3
"""Does laser odometry beat the wheels sideways? Answered off runs already recorded.

D-25 says the laser may not take over `odom -> base_footprint` until it is shown
to do better than 19.5 mm sideways, and D-18 says that showing has to be a
measurement rather than an argument. Both are satisfied without driving anything
new: fifty runs are on disk with /scan_filtered and Gazebo ground truth in the
same bag.

Per run it integrates the scan matcher over the whole bag, then reports how far
each estimate has drifted from ground truth - separately along the robot's
heading and across it, because that split is the entire point.

    ./scripts/run_native.sh python3 validation/evaluate_laser_odometry.py <batch>
"""
import argparse
import math
import os
import statistics as st
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src',
                                'pas_dual_arm_scripts'))
from pas_dual_arm_scripts.scan_matcher import match, scan_to_points  # noqa: E402

ROBOT = 'dual_arm_robot'


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
    scans, truth, odom = [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        seconds = stamp * 1e-9
        if topic == '/scan_filtered':
            msg = deserialize_message(data, get_message(types[topic]))
            scans.append((seconds, msg))
        elif topic == '/debug/gz_dynamic_pose':
            msg = deserialize_message(data, get_message(types[topic]))
            for tf in msg.transforms:
                if tf.child_frame_id == ROBOT:
                    truth.append((seconds, tf.transform.translation.x,
                                  tf.transform.translation.y,
                                  yaw_of(tf.transform.rotation)))
        elif topic == '/tf':
            msg = deserialize_message(data, get_message(types[topic]))
            for tf in msg.transforms:
                if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
                    odom.append((seconds, tf.transform.translation.x,
                                 tf.transform.translation.y,
                                 yaw_of(tf.transform.rotation)))
    return scans, truth, odom


def nearest(series, when):
    return min(series, key=lambda s: abs(s[0] - when)) if series else None


def integrate(scans, odom, stride, keyframe_distance=0.30, keyframe_angle=0.26):
    """Run the matcher over the bag and return its trajectory, seeded by the wheels.

    Matched against a KEYFRAME, not against the previous scan. Scan-to-scan adds
    one matching error per scan and they compound: the first attempt drifted
    286 mm forwards and 265 mm sideways over a run, against the wheels' 12 and
    19, across 2205 matches. Against a keyframe the error is one match's worth
    per keyframe, and a keyframe only changes after 30 cm or 15 degrees.
    """
    pose = [0.0, 0.0, 0.0]
    track, previous, previous_time = [], None, None
    key_pose = None                     # pose where the keyframe was taken
    key_time = None
    for index, (when, msg) in enumerate(scans):
        if index % stride:
            continue
        points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max)
        if previous is not None:
            guess = (0.0, 0.0, 0.0)
            a, b = nearest(odom, previous_time), nearest(odom, when)
            if a and b:
                c, s = math.cos(-a[3]), math.sin(-a[3])
                gx, gy = b[1] - a[1], b[2] - a[2]
                guess = (gx * c - gy * s, gx * s + gy * c,
                         math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
            # The guess is from the keyframe, so it must span the same interval.
            a, b = nearest(odom, key_time), nearest(odom, when)
            if a and b:
                c, s = math.cos(-a[3]), math.sin(-a[3])
                gx, gy = b[1] - a[1], b[2] - a[2]
                guess = (gx * c - gy * s, gx * s + gy * c,
                         math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
            got = match(previous, points, guess=guess)
            if got is not None:
                dx, dy, dtheta, _ = got
                c, s = math.cos(key_pose[2]), math.sin(key_pose[2])
                pose = [key_pose[0] + dx * c - dy * s,
                        key_pose[1] + dx * s + dy * c,
                        math.atan2(math.sin(key_pose[2] + dtheta),
                                   math.cos(key_pose[2] + dtheta))]
                if (math.hypot(dx, dy) > keyframe_distance
                        or abs(dtheta) > keyframe_angle):
                    previous, key_pose, key_time = points, list(pose), when
        else:
            previous, key_pose, key_time = points, list(pose), when
        track.append((when, pose[0], pose[1], pose[2]))
        previous_time = when
    return track


def drift(estimate, truth, heading):
    """How far the estimate has wandered from truth, along the heading and across it."""
    (_, ex0, ey0, _), (_, ex1, ey1, _) = estimate[0], estimate[-1]
    a, b = nearest(truth, estimate[0][0]), nearest(truth, estimate[-1][0])
    dx = (b[1] - a[1]) - (ex1 - ex0)
    dy = (b[2] - a[2]) - (ey1 - ey0)
    c, s = math.cos(-heading), math.sin(-heading)
    return (dx * c - dy * s) * 1000.0, (dx * s + dy * c) * 1000.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch')
    parser.add_argument('--stride', type=int, default=3,
                        help='match every Nth scan; the lidar runs far faster than it needs to')
    parser.add_argument('--runs', type=int, default=0, help='stop after N runs (0 = all)')
    args = parser.parse_args()

    root = os.path.join('validation', 'results', args.batch)
    laser, wheels = [], []
    for name in sorted(os.listdir(root)):
        if not name.startswith('run_'):
            continue
        bag = os.path.join(root, name, 'bag')
        if not os.path.isdir(bag):
            continue
        scans, truth, odom = read(bag)
        if len(scans) < 50 or not truth or not odom:
            print(f'{name}: not enough in the bag')
            continue
        track = integrate(scans, odom, args.stride)
        if len(track) < 10:
            print(f'{name}: the matcher never took hold')
            continue
        heading = nearest(truth, track[0][0])[3]
        lf, ll = drift(track, truth, heading)
        wf, wl = drift([(o[0], o[1], o[2], o[3]) for o in odom
                        if track[0][0] <= o[0] <= track[-1][0]], truth, heading)
        laser.append((lf, ll))
        wheels.append((wf, wl))
        print(f'{name}: laser {lf:+8.1f} / {ll:+8.1f} mm   wheels {wf:+8.1f} / {wl:+8.1f} mm'
              f'   ({len(track)} matches)', flush=True)
        if args.runs and len(laser) >= args.runs:
            break

    if not laser:
        return 1
    print(f'\nDrift from ground truth over a whole run, n={len(laser)} '
          f'(forwards / sideways, |median|):')
    for label, rows in (('laser odometry', laser), ('wheel odometry', wheels)):
        print(f'  {label:<16} {st.median(abs(f) for f, _ in rows):8.1f} mm  '
              f'{st.median(abs(l) for _, l in rows):8.1f} mm')
    print('\nD-25 asks one question: is sideways better than the wheels\' 19.5 mm?')
    return 0


if __name__ == '__main__':
    sys.exit(main())
