#!/usr/bin/env python3
"""Read marker and depth estimates without commanding any robot motion."""

import math

import rclpy
from geometry_msgs.msg import Point

from pas_dual_arm_scripts.main_task import MainTask


def main():
    rclpy.init()
    node = MainTask()
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.2)
        marker = node.confirm_box(timeout=20.0, samples=5)
        if marker is None:
            raise SystemExit('No fresh ArUco marker')
        depth = node.measure_box(marker, timeout=12.0)
        if depth is None:
            raise SystemExit('Depth box measurement rejected')
        center, length, height, axis = depth
        delta_xy = math.hypot(center.x - marker.x, center.y - marker.y)
        delta_z = abs(center.z - marker.z)
        print(f'MARKER_CENTER {marker.x:.4f} {marker.y:.4f} {marker.z:.4f}', flush=True)
        print(f'DEPTH_CENTER {center.x:.4f} {center.y:.4f} {center.z:.4f}', flush=True)
        print(f'EXTENT {length:.4f} {height:.4f} AXIS {axis}', flush=True)
        print(f'AGREEMENT xy={delta_xy:.4f} z={delta_z:.4f}', flush=True)
        if delta_xy > 0.03 or delta_z > 0.05:
            raise SystemExit('Marker and depth disagree; no grasp poses')
        node.measure_tip_standoff()
        tangent = node.marker_tangent()
        if tangent is None:
            raise SystemExit('No marker orientation; no grasp poses')
        # Marker midpoint is the known cube midpoint vertically. The point
        # cloud only sees part of the box and biases its Z midpoint upward.
        grip_center = Point(x=center.x, y=center.y, z=marker.z)
        pre_left, pre_right = node.squeeze_poses(grip_center, tangent, pre=0.10)
        contact_left, contact_right = node.squeeze_poses(grip_center, tangent)
        print(f'GRASP_CENTER {grip_center.x:.4f} {grip_center.y:.4f} '
              f'{grip_center.z:.4f} TANGENT {tangent}', flush=True)
        for label, pose in (('LEFT_PRE', pre_left), ('RIGHT_PRE', pre_right),
                            ('LEFT_CONTACT', contact_left),
                            ('RIGHT_CONTACT', contact_right)):
            p, q = pose.position, pose.orientation
            print(f'{label} pos=({p.x:.4f},{p.y:.4f},{p.z:.4f}) '
                  f'quat=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})',
                  flush=True)
        for side, pre_pose, contact_pose in (
                ('left', pre_left, contact_left),
                ('right', pre_right, contact_right)):
            group = f'{side}_arm'
            ee = f'{side}_end_effector_link'
            contact_solution = node._ik(group, ee, contact_pose)
            pre_solution = node._ik(group, ee, pre_pose,
                                    seed=contact_solution,
                                    avoid_collisions=True)
            print(f'{side.upper()}_IK contact={contact_solution is not None} '
                  f'pre={pre_solution is not None}', flush=True)
        # Read-only reachability estimates for a later base approach. These
        # targets are hypotheses, not a claim that the robot has moved there.
        for trial_x, trial_y in ((0.65, 0.0), (0.55, 0.0),
                                 (0.55, -0.075)):
            trial = Point(x=trial_x, y=trial_y, z=grip_center.z)
            pre_poses = node.squeeze_poses(trial, tangent, pre=0.10)
            contact_poses = node.squeeze_poses(trial, tangent)
            reachable = []
            for side, pre_pose, contact_pose in zip(
                    ('left', 'right'), pre_poses, contact_poses):
                group, ee = f'{side}_arm', f'{side}_end_effector_link'
                contact_sol = node._ik(group, ee, contact_pose)
                pre_sol = node._ik(group, ee, pre_pose,
                                   seed=contact_sol, avoid_collisions=True)
                reachable.append((contact_sol is not None,
                                  pre_sol is not None))
            print(f'PREDICTED_PRE_IK center=({trial_x:.3f},{trial_y:.3f}) '
                  f'left_contact_pre={reachable[0]} '
                  f'right_contact_pre={reachable[1]}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
