#!/usr/bin/env python3
"""Read-only wrist-roll IK survey at the currently observed cube pose."""

import math

import rclpy
from geometry_msgs.msg import Point

from pas_dual_arm_scripts.main_task import MainTask, quat_mul, yaw_to_quat


def main():
    rclpy.init()
    node = MainTask()
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        marker = node.confirm_box(timeout=10.0, samples=5)
        if marker is None:
            raise RuntimeError('marker unavailable')
        depth = node.measure_box(marker)
        if depth is None:
            raise RuntimeError('depth unavailable')
        measured, _, _, _ = depth
        center = Point(x=measured.x, y=measured.y, z=marker.z)
        axis = node.marker_tangent()
        if axis is None:
            raise RuntimeError('marker orientation unavailable')
        node.measure_tip_standoff()
        print(f'CENTER {center.x:.3f} {center.y:.3f} {center.z:.3f} '
              f'AXIS {axis}', flush=True)
        for height in (0.0, 0.04, -0.04):
            test_center = Point(x=center.x, y=center.y, z=center.z + height)
            for clearance in (0.05, 0.10):
                _, pre = node.squeeze_poses(test_center, axis, pre=clearance)
                _, contact = node.squeeze_poses(test_center, axis)
                for roll in (0.0, math.pi / 2, -math.pi / 2, math.pi):
                    rot = yaw_to_quat(roll)
                    pre.orientation = quat_mul(
                        node.squeeze_poses(test_center, axis, pre=clearance)[1].orientation,
                        rot)
                    contact.orientation = quat_mul(
                        node.squeeze_poses(test_center, axis)[1].orientation,
                        rot)
                    sol = node._ik('right_arm', 'right_end_effector_link', contact)
                    pre_sol = node._ik('right_arm', 'right_end_effector_link',
                                       pre, seed=sol, avoid_collisions=True)
                    if sol is not None and pre_sol is not None:
                        print(f'RIGHT_PAIR_OK dz={height:+.3f} '
                              f'clearance={clearance:.3f} roll={roll:.3f}',
                              flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
