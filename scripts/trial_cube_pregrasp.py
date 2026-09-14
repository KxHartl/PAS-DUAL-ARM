#!/usr/bin/env python3
"""Measure the cube, then bring BOTH hands to their pre-grasp points together.

The two arms are planned separately but released at the same instant, stretched
to a common duration: a hand that arrives first pushes the cube across the table
before the opposite hand is there to balance it. Every pose the grasp is derived
from is published for RViz (`/cube_grasp/poses`, `/cube_grasp/markers`), so the
run can be watched rather than inferred from the log.
"""

import math

import rclpy
from geometry_msgs.msg import Point, PoseArray
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from pas_dual_arm_scripts.main_task import MainTask

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# How far ahead of base_link the cube centre should end up. Measured from the
# URDF: the left shoulder sits at (0.000, 0.153, 0.386) with the carriages at
# their lower limit, so a contact pose at (d, 0.265, 0.821) needs a reach of
# 0.871 m at d = 0.746 and 0.750 m at d = 0.600. The Gen3 reaches 0.902 m to
# the flange, but nothing like that with the wrist turned sideways - which is
# why both arms failed IK at 0.746 m.
TARGET_RANGE = 0.62

# What stops the base. Everything on the robot below 0.23 m passes under the
# tabletop and between the near table legs (they are 0.70 m apart, the base is
# 0.497 m wide). The limit is the forwardmost structure ABOVE tabletop height:
# the torso frame, at x = +0.283 m. The table's near edge is 0.25 m in front of
# the cube centre.
TORSO_FRONT = 0.283
TABLE_EDGE_AHEAD_OF_CUBE = -0.25
EDGE_CLEARANCE = 0.03


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
    node.get_logger().info(
        f'cube: long axis {length:.3f} m, visible height {height:.3f} m')
    # X and Y from the fused estimate, Z from the marker: the marker sits at
    # the middle of the face, while the cloud only sees the upper part of the
    # cube and biases its Z midpoint upward by about 2.7 cm.
    return Point(x=center.x, y=center.y, z=marker.z), axis


def publish_view(node, poses_pub, markers_pub, center, poses):
    """Show the fused centre and all four hand poses in RViz."""
    array = PoseArray()
    array.header.frame_id = 'base_link'
    array.header.stamp = node.get_clock().now().to_msg()
    array.poses = list(poses.values())
    poses_pub.publish(array)

    markers = MarkerArray()
    cube = Marker()
    cube.header = array.header
    cube.ns = 'cube'
    cube.id = 0
    cube.type = Marker.CUBE
    cube.action = Marker.ADD
    cube.pose.position = center
    cube.pose.orientation.w = 1.0
    cube.scale.x = cube.scale.y = cube.scale.z = 0.30
    cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.9, 0.8, 0.1, 0.35
    cube.lifetime = Duration(seconds=0).to_msg()
    markers.markers.append(cube)
    for index, (label, pose) in enumerate(poses.items()):
        text = Marker()
        text.header = array.header
        text.ns = 'labels'
        text.id = index + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pose.position.x
        text.pose.position.y = pose.position.y
        text.pose.position.z = pose.position.z + 0.06
        text.pose.orientation.w = 1.0
        text.scale.z = 0.04
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = label
        markers.markers.append(text)
    markers_pub.publish(markers)


def main():
    rclpy.init()
    node = MainTask()
    poses_pub = node.create_publisher(PoseArray, '/cube_grasp/poses', LATCHED)
    markers_pub = node.create_publisher(
        MarkerArray, '/cube_grasp/markers', LATCHED)
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.aim_camera(0.0, 0.65, 'pregrasp camera aim'):
            raise RuntimeError('Camera pan/tilt command failed')
        center, axis = measure(node)

        # Close in far enough for the arms to reach, and no further than the
        # torso can go without touching the tabletop.
        wanted = center.x - TARGET_RANGE
        table_edge = center.x + TABLE_EDGE_AHEAD_OF_CUBE
        allowed = table_edge - TORSO_FRONT - EDGE_CLEARANCE
        advance = max(0.0, min(wanted, allowed))
        node.get_logger().info(
            f'approach: cube at {center.x:.3f} m, want {TARGET_RANGE:.3f} m '
            f'(advance {wanted:.3f}), table edge at {table_edge:.3f} m allows '
            f'{allowed:.3f} -> advancing {advance:.3f} m')
        if wanted > allowed + 1e-6:
            node.get_logger().warn(
                f'the table stops the base {wanted - allowed:.3f} m short of '
                'the range the arms want; expect IK to be tight')
        if advance > 0.02:
            travelled = node.drive_distance(advance, speed=0.04)
            if travelled is None or travelled > advance + 0.03:
                raise RuntimeError('Base advance did not track odometry')
            node.get_logger().info(f'Advanced {travelled:.3f} m; remeasuring')
            if not node.aim_camera(0.0, 0.65, 'close-range camera aim'):
                raise RuntimeError('Camera pan/tilt command failed')
            center, axis = measure(node)

        node.measure_tip_standoff()
        pre_left, pre_right = node.squeeze_poses(center, axis, pre=0.10)
        contact_left, contact_right = node.squeeze_poses(center, axis)
        publish_view(node, poses_pub, markers_pub, center, {
            'lijeva pre': pre_left, 'desna pre': pre_right,
            'lijeva kontakt': contact_left, 'desna kontakt': contact_right,
        })
        node.get_logger().info(
            f'cube centre ({center.x:.3f},{center.y:.3f},{center.z:.3f}); '
            f'left pre ({pre_left.position.x:.3f},{pre_left.position.y:.3f},'
            f'{pre_left.position.z:.3f}); right pre '
            f'({pre_right.position.x:.3f},{pre_right.position.y:.3f},'
            f'{pre_right.position.z:.3f})')

        node.publish_collision_scene(center, table_top_z=center.z - 0.15)

        # Both arms are planned before either one moves. Solve every pose for
        # BOTH hands before giving up on either, and solve each pre-pose twice:
        # once ignoring collisions, once respecting them. Out of reach and
        # blocked by the table are different problems with different fixes, and
        # a single "IK unavailable" cannot tell them apart.
        targets = {}
        failures = []
        for side, pre, contact in (('left', pre_left, contact_left),
                                   ('right', pre_right, contact_right)):
            group, ee = f'{side}_arm', f'{side}_end_effector_link'
            contact_ik = node._ik(group, ee, contact)
            free_ik = node._ik(group, ee, pre, seed=contact_ik)
            pre_ik = node._ik(group, ee, pre, seed=contact_ik,
                              avoid_collisions=True)
            node.get_logger().info(
                f'{side} IK: kontakt ({contact.position.x:.3f},'
                f'{contact.position.y:+.3f},{contact.position.z:.3f})='
                f'{"OK" if contact_ik else "NE"}  '
                f'pre bez kolizija={"OK" if free_ik else "NE"}  '
                f'pre s kolizijama={"OK" if pre_ik else "NE"}')
            if contact_ik is None:
                failures.append(f'{side}: kontaktna poza je izvan dosega')
            elif free_ik is None:
                failures.append(f'{side}: pred-poza je izvan dosega')
            elif pre_ik is None:
                failures.append(
                    f'{side}: pred-poza je dohvatljiva, ali MoveIt u njoj vidi '
                    'sudar (najvjerojatnije prsti u ploču stola)')
            else:
                targets[side] = (group, ee, pre, pre_ik)
        if failures:
            raise RuntimeError('; '.join(failures))

        trajs = {}
        for side, (group, _ee, _pre, joints) in targets.items():
            traj = node.plan_arm_joints(group, joints, f'{side} cube pregrasp')
            if traj is None:
                raise RuntimeError(f'{side} pregrasp path unavailable')
            trajs[side] = traj

        results = node.move_arms_parallel(trajs, 'cube pregrasp')
        if not all(results.values()):
            raise RuntimeError(f'parallel pregrasp move failed: {results}')

        reached = True
        for side, (_group, ee, pre, _joints) in targets.items():
            node._wait_settle(ee)
            if not node.verify_reached(ee, pre, tol=0.05,
                                       label=f'{side} cube pregrasp'):
                reached = False
        if not reached:
            raise RuntimeError('a hand did not reach its pre-grasp point')
        print('BOTH_PREGRASPS_REACHED', flush=True)
    finally:
        node._send_vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
