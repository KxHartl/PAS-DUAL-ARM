#!/usr/bin/env python3
"""How close does the base have to get before both hands can reach the cube?

Read-only: nothing is commanded, the robot does not move. The cube is measured
once, then the grasp geometry is re-solved at a range of hypothetical distances
to find the nearest one both arms can actually reach. Every row is printed as it
is solved, so a long sweep never looks like a hang.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Point

from pas_dual_arm_scripts.main_task import MainTask, quat_mul, yaw_to_quat

# The dock pose puts the cube about 0.87 m ahead; 0.55 m is as close as the
# base can plausibly get before the table legs at y = -6.125 stop it.
NEAR_LIMIT = 0.55


def solve_pair(node, center, axis, clearance, timeout, roll=0.0):
    """IK for both arms, contact pose and pre-pose. Returns {side: (bool, bool)}."""
    pre_poses = node.squeeze_poses(center, axis, pre=clearance)
    contact_poses = node.squeeze_poses(center, axis)
    if roll:
        rotation = yaw_to_quat(roll)
        for pose in (*pre_poses, *contact_poses):
            pose.orientation = quat_mul(pose.orientation, rotation)
    out = {}
    for side, pre, contact in zip(('left', 'right'), pre_poses, contact_poses):
        group, ee = f'{side}_arm', f'{side}_end_effector_link'
        contact_ik = node._ik(group, ee, contact, timeout=timeout)
        pre_ik = node._ik(group, ee, pre, seed=contact_ik,
                          avoid_collisions=True, timeout=timeout)
        out[side] = (contact_ik is not None, pre_ik is not None)
    return out


def mark(pair):
    return f'{"OK " if pair[0] else "-- "}/{" OK" if pair[1] else " --"}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--clearance', type=float, default=0.10,
                        help='pre-pose stand-off from the contact pose (m)')
    parser.add_argument('--step', type=float, default=0.04,
                        help='distance step for the sweep (m)')
    parser.add_argument('--timeout', type=float, default=1.0,
                        help='per-call IK timeout (s)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MainTask()
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        marker = node.confirm_box(timeout=10.0, samples=5)
        if marker is None:
            raise SystemExit('marker unavailable')
        depth = node.measure_box(marker)
        if depth is None:
            raise SystemExit('depth unavailable')
        measured, _, _, _ = depth
        # X/Y fused, Z from the marker: it sits at the middle of the face,
        # while the cloud only sees the upper part of the cube.
        center = Point(x=measured.x, y=measured.y, z=marker.z)
        axis = node.marker_tangent()
        if axis is None:
            raise SystemExit('marker orientation unavailable')
        node.measure_tip_standoff()
        print(f'CENTER {center.x:.3f} {center.y:.3f} {center.z:.3f} '
              f'AXIS ({axis[0]:+.3f},{axis[1]:+.3f})', flush=True)
        print(f'clearance {args.clearance:.2f} m, IK timeout '
              f'{args.timeout:.1f} s per call', flush=True)
        print('', flush=True)
        print('  udaljenost |  lijeva kontakt/pre | desna kontakt/pre | oboje',
              flush=True)
        print('  -----------+---------------------+-------------------+------',
              flush=True)

        started = time.monotonic()
        reachable = None
        distance = center.x
        while distance >= NEAR_LIMIT - 1e-6:
            trial = Point(x=distance, y=center.y, z=center.z)
            pairs = solve_pair(node, trial, axis, args.clearance, args.timeout)
            both = all(all(pair) for pair in pairs.values())
            print(f'  {distance:9.3f} m | {mark(pairs["left"]):>19} '
                  f'| {mark(pairs["right"]):>17} | '
                  f'{"DA" if both else "ne"}', flush=True)
            if both and reachable is None:
                reachable = distance
            distance -= args.step

        print('', flush=True)
        if reachable is None:
            print('NIJEDNA udaljenost ne prolazi za obje ruke. Sljedeće: '
                  'varijante zakreta zapešća na izmjerenoj udaljenosti.',
                  flush=True)
            for dz in (0.0, 0.04, -0.04):
                for roll in (0.0, math.pi / 2, -math.pi / 2, math.pi):
                    trial = Point(x=center.x, y=center.y, z=center.z + dz)
                    pairs = solve_pair(node, trial, axis, args.clearance,
                                       args.timeout, roll=roll)
                    if all(all(pair) for pair in pairs.values()):
                        print(f'  PAR OK dz={dz:+.3f} roll={roll:+.3f}',
                              flush=True)
        else:
            advance = center.x - reachable
            print(f'NAJDALJA dohvatljiva udaljenost: {reachable:.3f} m '
                  f'-> bazu treba primaknuti {advance:.3f} m', flush=True)
            if advance <= 0.0:
                print('Baza se ne mora micati.', flush=True)
        print(f'(sweep trajao {time.monotonic() - started:.0f} s)', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
