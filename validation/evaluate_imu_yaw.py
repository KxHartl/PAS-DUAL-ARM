#!/usr/bin/env python3
"""How well does each source know which way the robot is pointing?

P-52 found the arrival heading sitting at 98 % of its limit, run after run, and
the doorway failures put the robot 57-72 mm off the axis of a passage it enters
2.3 m before. A heading error carried through that length is exactly that
displacement, so heading is worth measuring on its own rather than as part of a
pose.

Every estimate here is relative: each is zeroed at its own first sample and
compared against Gazebo, which is zeroed at the same instant. That is the fair
comparison for odometry, which is never asked for an absolute heading.

The IMU is scored twice. The gyro is integrated by trapezoid, which is what a
filter would actually consume, and the sensor's own orientation is read as well,
because Ignition computes it by integrating the same gyro internally and the two
should agree - if they do not, the integration here is wrong.

    ./scripts/run_native.sh python3 validation/evaluate_imu_yaw.py <bag dir>
"""
import argparse
import math
import os
import sys

ROBOT = 'dual_arm_robot'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def stamped(header):
    """Simulation time from the message itself, for integrating a rate.

    Everything is lined up on bag time, because the ground-truth bridge leaves
    its headers empty and bag time is the one clock every topic shares. But a
    rate measured in simulated seconds may not be integrated over wall-clock
    seconds: the simulation runs at about half real time, so that inflates it by
    1/RTF. The first attempt did exactly that and had the gyro 42 deg out while
    the same sensor's own orientation was 0.02 deg out, which is how the mistake
    surfaced.
    """
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {'truth': [], 'imu': [], 'wheel': [], 'laser': []}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in ('/debug/gz_dynamic_pose', '/base_imu',
                         '/base_controller/odom', '/laser_odom'):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        seconds = stamp * 1e-9
        if topic == '/debug/gz_dynamic_pose':
            for tf in msg.transforms:
                if tf.child_frame_id == ROBOT:
                    out['truth'].append((seconds,
                                         tf.transform.translation.x,
                                         tf.transform.translation.y,
                                         yaw_of(tf.transform.rotation)))
        elif topic == '/base_imu':
            out['imu'].append((seconds, stamped(msg.header),
                               msg.angular_velocity.z, yaw_of(msg.orientation)))
        elif topic == '/base_controller/odom':
            out['wheel'].append((seconds, msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 yaw_of(msg.pose.pose.orientation)))
        elif topic == '/laser_odom':
            out['laser'].append((seconds, msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 yaw_of(msg.pose.pose.orientation)))
    return out


def integrate_gyro(samples):
    """Trapezoidal integration of the yaw rate, zeroed at the first sample.

    Stepped by the sensor's own clock, reported on bag time so it can be
    compared with the rest.
    """
    trace, angle = [], 0.0
    for (_, sim0, w0, _), (bag1, sim1, w1, _) in zip(samples, samples[1:]):
        dt = sim1 - sim0
        if 0.0 < dt < 1.0:                      # a gap means samples were dropped
            angle += 0.5 * (w0 + w1) * dt
        trace.append((bag1, angle))
    return trace


def unwrapped(samples, index=1):
    """Yaw as a continuous angle, zeroed at the first sample."""
    trace, total, previous = [], 0.0, samples[0][index]
    for sample in samples:
        total += wrap(sample[index] - previous)
        previous = sample[index]
        trace.append((sample[0], total))
    return trace


def score(name, trace, truth, report):
    """Compare one zeroed trace against the zeroed truth, sample by sample."""
    if not trace:
        return
    errors, index = [], 0
    for when, angle in trace:
        while index + 1 < len(truth) and truth[index + 1][0] <= when:
            index += 1
        errors.append(abs(angle - truth[index][1]))
    final = abs(trace[-1][1] - truth[-1][1])
    report.append((name, len(trace), math.degrees(max(errors)),
                   math.degrees(sum(errors) / len(errors)), math.degrees(final)))


def in_start_frame(samples):
    """Every pose expressed in the frame of the first one."""
    x0, y0, a0 = samples[0][1], samples[0][2], samples[0][3]
    c, s = math.cos(-a0), math.sin(-a0)
    out = []
    for t, x, y, _ in samples:
        dx, dy = x - x0, y - y0
        out.append((t, dx * c - dy * s, dx * s + dy * c))
    return out


def score_position(name, samples, truth, report):
    """Distance between an estimate and the truth, both in their own start frame."""
    if not samples:
        return
    trace, reference = in_start_frame(samples), in_start_frame(truth)
    errors, index = [], 0
    for when, x, y in trace:
        while index + 1 < len(reference) and reference[index + 1][0] <= when:
            index += 1
        errors.append(math.hypot(x - reference[index][1], y - reference[index][2]))
    report.append((name, len(trace), 1000.0 * max(errors),
                   1000.0 * sum(errors) / len(errors), 1000.0 * errors[-1]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    args = parser.parse_args()
    if not os.path.isdir(args.bag):
        sys.exit(f'no such bag: {args.bag}')

    data = read(args.bag)
    if not data['truth']:
        sys.exit('no ground truth in the bag: relaunch with debug_truth:=true')
    if not data['imu']:
        sys.exit('no /base_imu in the bag: is the Imu system plugin in the world '
                 'and the topic in bridge.yaml?')

    truth = unwrapped(data['truth'], 3)
    report = []
    score('IMU (gyro, integriran)', integrate_gyro(data['imu']), truth, report)
    score('IMU (vlastita orijentacija)', unwrapped(data['imu'], 3), truth, report)
    score('kotaci', unwrapped(data['wheel'], 3), truth, report)
    score('fuzija (/laser_odom)', unwrapped(data['laser'], 3), truth, report)

    turned = math.degrees(sum(abs(b[1] - a[1]) for a, b in zip(truth, truth[1:])))
    duration = truth[-1][0] - truth[0][0]
    print(f'\n{os.path.basename(args.bag)}: {duration:.1f} s, zakrenuto ukupno '
          f'{turned:.0f} deg, {len(data["truth"])} uzoraka istine')
    print(f'{"izvor":<28}{"n":>7}{"max":>9}{"sred":>9}{"kraj":>9}   [deg]')
    for name, count, peak, mean, final in report:
        print(f'{name:<28}{count:>7}{peak:>9.2f}{mean:>9.2f}{final:>9.2f}')

    # Heading is what this drive exists to measure, but a fusion that fixed the
    # heading and lost the position would be no use, so both are reported.
    places = []
    score_position('kotaci', data['wheel'], data['truth'], places)
    score_position('fuzija (/laser_odom)', data['laser'], data['truth'], places)
    if places:
        print(f'\n{"izvor":<28}{"n":>7}{"max":>9}{"sred":>9}{"kraj":>9}   [mm]')
        for name, count, peak, mean, final in places:
            print(f'{name:<28}{count:>7}{peak:>9.1f}{mean:>9.1f}{final:>9.1f}')
    print()


if __name__ == '__main__':
    main()
