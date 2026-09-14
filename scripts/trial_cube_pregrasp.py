#!/usr/bin/env python3
"""In isolated sim, approach the cube and move arms to collision-checked preposes."""

import math
import time

import rclpy
from geometry_msgs.msg import Point

from pas_dual_arm_scripts.main_task import MainTask


def measure(node):
    marker = node.confirm_box(timeout=10.0, samples=5)
    if marker is None:
        raise RuntimeError('No fresh marker')
    depth = node.measure_box(marker, timeout=10.0)
    if depth is None:
        raise RuntimeError('No accepted depth cluster')
    center, length, height, _ = depth
    if math.hypot(center.x - marker.x, center.y - marker.y) > 0.06:
        raise RuntimeError('Marker/depth XY disagree')
    if abs(center.z - marker.z) > 0.05:
        raise RuntimeError('Marker/depth Z disagree')
    axis = node.marker_tangent()
    if axis is None:
        raise RuntimeError('No marker face orientation')
    return Point(x=center.x, y=center.y, z=marker.z), axis


def main():
    rclpy.init()
    node = MainTask()
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.aim_camera(0.0, 0.65, 'pregrasp camera aim'):
            raise RuntimeError('Camera pan/tilt command failed')
        center, axis = measure(node)
        # Hard stop before the table. At this setup, the cube starts ~0.85 m
        # ahead; advancing at most 0.20 m leaves its centre ~0.65 m ahead.
        advance = min(0.12, max(0.0, center.x - 0.65))
        if advance > 0.02:
            travelled = node.drive_distance(advance, speed=0.04)
            if travelled is None or travelled > 0.14:
                raise RuntimeError('Base advance did not track odometry')
            node.get_logger().info(f'Advanced {travelled:.3f} m; remeasuring')
            if not node.aim_camera(0.0, 0.65, 'close-range camera aim'):
                raise RuntimeError('Camera pan/tilt command failed')
        center, axis = measure(node)
        if not (0.57 <= center.x <= 0.73):
            raise RuntimeError(f'Cube range {center.x:.3f} m outside pregrasp band')
        node.measure_tip_standoff()
        pre_l, pre_r = node.squeeze_poses(center, axis, pre=0.10)
        contact_l, contact_r = node.squeeze_poses(center, axis)
        node.get_logger().info(
            f'Cube center ({center.x:.3f},{center.y:.3f},{center.z:.3f}); '
            f'left pre ({pre_l.position.x:.3f},{pre_l.position.y:.3f},'
            f'{pre_l.position.z:.3f}); right pre '
            f'({pre_r.position.x:.3f},{pre_r.position.y:.3f},'
            f'{pre_r.position.z:.3f})')
        node.publish_collision_scene(center, table_top_z=center.z - 0.15)
        plans = []
        for side, pre, contact in (
                ('left', pre_l, contact_l), ('right', pre_r, contact_r)):
            group, ee = f'{side}_arm', f'{side}_end_effector_link'
            contact_ik = node._ik(group, ee, contact)
            pre_ik = node._ik(group, ee, pre, seed=contact_ik,
                              avoid_collisions=True)
            if contact_ik is None or pre_ik is None:
                raise RuntimeError(f'{side} pre/contact IK unavailable')
            plans.append((side, group, ee, pre, pre_ik))
        for side, group, ee, pre, joints in plans:
            if not node.move_arm_joints(group, joints, f'{side} cube pregrasp'):
                raise RuntimeError(f'{side} pregrasp plan or execution failed')
            node._wait_settle(ee)
            if not node.verify_reached(ee, pre, tol=0.05,
                                       label=f'{side} cube pregrasp'):
                raise RuntimeError(f'{side} did not reach pregrasp')
        print('BOTH_PREGRASPS_REACHED', flush=True)
    finally:
        node._send_vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
