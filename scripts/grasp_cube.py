#!/usr/bin/env python3
"""Find the cube, take hold of it with both hands, and lift it. One command.

Stages, in order:
  1. look at the cube and measure it with the head camera
  2. close the base in to a range the arms can actually work at
  3. open both hands and take them to the pre-grasp poses, together
  4. read the marker on each pressed face with that hand's own camera
  5. bring both hands to a standoff off the faces, LEVEL with each other
  6. close both hands together until all four pads report the cube, then stop
  7. lift with the torso carriages

The hands are levelled at the standoff and never asked to change height again.
Correcting height while pressing is what skewed the cube: a wrist shifting in z
against a face it is already touching tilts it, and both hands then hold it
crooked. Four pads stops the arms outright.
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from pas_dual_arm_scripts.main_task import MainTask

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Where the cube centre should end up ahead of base_link. From the URDF the
# left shoulder is at (0.000, 0.153, 0.386) with the carriages down, so a
# contact pose at (d, 0.25, 0.82) needs 0.87 m of reach at d = 0.75 and 0.75 m
# at d = 0.60. The arm manages 0.90 m straight out and much less with the wrist
# turned sideways, which is why 0.75 m failed IK outright.
TARGET_RANGE = 0.62

# What stops the base: everything below 0.23 m passes under the tabletop and
# between the near legs, so the limit is the forwardmost structure ABOVE that
# height - the torso frame at x = +0.283 m - against the table's near edge,
# 0.25 m in front of the cube centre.
TORSO_FRONT = 0.283
TABLE_EDGE_AHEAD_OF_CUBE = -0.25
EDGE_CLEARANCE = 0.03

# Clear distance from the cube FACE to the fingertips at the pre-grasp pose.
PRE_STANDOFF = 0.20

# ... and at the standoff the hands close from, one at a time.
CLOSE_STANDOFF = 0.02

GRIPPER_OPEN = 0.0

# The face markers sit this far above the face centre (seminar_world.sdf).
MARKER_RAISE = 0.056

CUBE_WIDTH = 0.30
WIDTH_TOLERANCE = 0.03

# Closing on a face: step, seconds per step, and how far past the commanded
# contact pose a hand may go. The cube stops the pad, not the pose.
CLOSE_STEP = 0.004
CLOSE_SECONDS = 1.5
CLOSE_OVERSHOOT = 0.025
CLOSE_MAX_STEPS = 24

LIFT_HEIGHT = 0.10


def measure_with_head(node):
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
    # X and Y from the fused estimate, Z from the marker: the marker is at the
    # middle of the face, while the cloud sees only the upper part of the cube.
    return Point(x=center.x, y=center.y, z=marker.z), axis


def face_from_marker(node, side, timeout=5.0):
    """The point on the pressed face that this hand's own camera sees."""
    deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        try:
            tf = node.tf_buffer.lookup_transform(
                'base_link', f'{side}_grasp_marker_frame', rclpy.time.Time())
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
            continue
        t = tf.transform.translation
        return Point(x=t.x, y=t.y, z=t.z - MARKER_RAISE)
    return None


def publish_view(node, poses_pub, markers_pub, center, poses):
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
    cube.scale.x = cube.scale.y = cube.scale.z = CUBE_WIDTH
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


def backed_off(pose, centre, distance):
    """`pose` moved `distance` away from `centre`, horizontally, same height."""
    dx = pose.position.x - centre.x
    dy = pose.position.y - centre.y
    span = math.hypot(dx, dy)
    out = Pose()
    out.orientation = pose.orientation
    out.position = Point(x=pose.position.x + dx / span * distance,
                         y=pose.position.y + dy / span * distance,
                         z=pose.position.z)
    return out


def squeeze_together(node, contacts):
    """Close both hands on the cube at once, and STOP the moment all four pads
    report it.

    Both hands always move, by the same amount, so the cube is squeezed rather
    than pushed. Each step is purely horizontal and keeps the hand at the
    height it is already at: the hands were levelled at the standoff, and
    correcting height WHILE pressing is what skewed the cube - the right wrist
    shifted in z against a face it was already touching, the cube tilted, and
    both hands ended up holding it crooked.

    Four pads is the stop condition, checked before every step. Nothing is
    commanded after it is met.
    """
    headings = {}
    budgets = {}
    for side, goal in contacts.items():
        ee = f'{side}_end_effector_link'
        node._wait_settle(ee)
        here = node._ee_pose(ee)
        if here is None:
            raise RuntimeError(f'no TF for {ee}')
        dx = goal.position.x - here.position.x
        dy = goal.position.y - here.position.y
        span = math.hypot(dx, dy)
        if span < 1e-4:
            raise RuntimeError(f'{side} is already at its contact pose')
        headings[side] = (dx / span, dy / span)
        budgets[side] = span + CLOSE_OVERSHOOT
        node.get_logger().info(
            f'{side}: {span * 1000:.0f} mm to the face, then up to '
            f'{CLOSE_OVERSHOOT * 1000:.0f} mm of squeeze')

    travelled = 0.0
    for step in range(1, CLOSE_MAX_STEPS + 1):
        pads = {side: node.tips_on_box(side) for side in contacts}
        total = sum(pads.values())
        node.get_logger().info(
            f'squeeze step {step}: pads on the box - left {pads["left"]}/2, '
            f'right {pads["right"]}/2')
        if total >= 4:
            node.get_logger().info(
                'all four pads are on the cube - stopping the arms here')
            return True
        size = min([CLOSE_STEP] + [b - travelled for b in budgets.values()])
        if size < 1e-4:
            break

        moving = {}
        for side, goal in contacts.items():
            ee = f'{side}_end_effector_link'
            here = node._ee_pose(ee)
            if here is None:
                raise RuntimeError(f'no TF for {ee}')
            pose = Pose()
            pose.orientation = goal.orientation
            # Height is whatever this hand already holds. Levelling belongs to
            # the standoff, before anything is being pressed.
            pose.position = Point(
                x=here.position.x + headings[side][0] * size,
                y=here.position.y + headings[side][1] * size,
                z=here.position.z)
            moving[side] = (f'{side}_arm', ee, pose)
        results = node.approach_both_linear(
            moving, f'squeeze step {step}', min_duration=CLOSE_SECONDS)
        if results is None or not all(results.values()):
            raise RuntimeError(f'squeeze step {step} did not execute')
        travelled += size

    pads = {side: node.tips_on_box(side) for side in contacts}
    return sum(pads.values()) >= 4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-lift', action='store_true',
                        help='stop once both hands are on the cube')
    parser.add_argument('--pregrasp-only', action='store_true',
                        help='stop at the pre-grasp poses, touch nothing')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MainTask()
    poses_pub = node.create_publisher(PoseArray, '/cube_grasp/poses', LATCHED)
    markers_pub = node.create_publisher(
        MarkerArray, '/cube_grasp/markers', LATCHED)
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.1)

        # 1-2. See the cube, then stand where the arms can work.
        if not node.aim_camera(0.0, 0.65, 'pregrasp camera aim'):
            raise RuntimeError('Camera pan/tilt command failed')
        centre, axis = measure_with_head(node)
        wanted = centre.x - TARGET_RANGE
        table_edge = centre.x + TABLE_EDGE_AHEAD_OF_CUBE
        allowed = table_edge - TORSO_FRONT - EDGE_CLEARANCE
        advance = max(0.0, min(wanted, allowed))
        node.get_logger().info(
            f'approach: cube at {centre.x:.3f} m, want {TARGET_RANGE:.3f} '
            f'(advance {wanted:.3f}), table edge at {table_edge:.3f} allows '
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
            centre, axis = measure_with_head(node)

        # 3. Open, measure the pads, and go to the pre-grasp poses together.
        for label, client in (('left', node.left_grip),
                              ('right', node.right_grip)):
            if not node.set_gripper(client, GRIPPER_OPEN, f'open {label} hand'):
                raise RuntimeError(f'{label} gripper did not open')
        node.measure_tip_standoff()
        pre_left, pre_right = node.squeeze_poses(centre, axis,
                                                 pre=PRE_STANDOFF)
        publish_view(node, poses_pub, markers_pub, centre,
                     {'lijeva pre': pre_left, 'desna pre': pre_right})
        node.publish_collision_scene(centre, table_top_z=centre.z - 0.15)

        targets = {}
        for side, pre in (('left', pre_left), ('right', pre_right)):
            group, ee = f'{side}_arm', f'{side}_end_effector_link'
            contact_ik = node._ik(group, ee, pre)
            pre_ik = node._ik(group, ee, pre, seed=contact_ik,
                              avoid_collisions=True)
            if pre_ik is None:
                raise RuntimeError(f'{side} pre-grasp pose is unreachable')
            targets[side] = (group, ee, pre, pre_ik)

        trajs = {}
        for side, (group, _ee, _pre, joints) in targets.items():
            traj = node.plan_arm_joints(group, joints, f'{side} pre-grasp')
            if traj is None:
                raise RuntimeError(f'{side} pre-grasp path unavailable')
            trajs[side] = traj
        results = node.move_arms_parallel(trajs, 'pre-grasp', min_duration=4.0)
        if not all(results.values()):
            raise RuntimeError(f'pre-grasp move failed: {results}')
        for side, (_group, ee, pre, _joints) in targets.items():
            node._wait_settle(ee)
            node.verify_reached(ee, pre, tol=0.05, label=f'{side} pre-grasp')
        print('PREGRASP_REACHED', flush=True)
        if args.pregrasp_only:
            return

        # 4. Each hand reads the marker on the face it will press. A few
        # millimetres, against a couple of centimetres from a metre away.
        faces = {}
        for side in ('left', 'right'):
            face = face_from_marker(node, side)
            if face is None:
                raise RuntimeError(f'{side} wrist sees no marker')
            faces[side] = face
            node.get_logger().info(
                f'{side} face from its own camera: ({face.x:.3f}, '
                f'{face.y:.3f}, {face.z:.3f})')
        span = math.dist((faces['left'].x, faces['left'].y, faces['left'].z),
                         (faces['right'].x, faces['right'].y, faces['right'].z))
        node.get_logger().info(f'cube width between the faces: {span:.3f} m')
        if abs(span - CUBE_WIDTH) > WIDTH_TOLERANCE:
            raise RuntimeError(
                f'the faces are {span:.3f} m apart, not {CUBE_WIDTH:.2f} - '
                'refusing to press on a reading that cannot be the cube')

        centre = Point(x=0.5 * (faces['left'].x + faces['right'].x),
                       y=0.5 * (faces['left'].y + faces['right'].y),
                       z=0.5 * (faces['left'].z + faces['right'].z))
        span_xy = math.hypot(faces['left'].x - faces['right'].x,
                             faces['left'].y - faces['right'].y)
        axis = ((faces['left'].x - faces['right'].x) / span_xy,
                (faces['left'].y - faces['right'].y) / span_xy)
        node.get_logger().info(
            f'cube from the hands: centre ({centre.x:.3f}, {centre.y:.3f}, '
            f'{centre.z:.3f}), axis ({axis[0]:+.3f}, {axis[1]:+.3f})')

        contacts = dict(zip(('left', 'right'),
                            node.squeeze_poses(centre, axis)))

        # 5. Both hands to a standoff off the faces, at the SAME height. Doing
        # this as one move is what keeps them level: both contact poses share
        # the cube's z, so both hands are commanded to it here, before either
        # touches anything.
        standoffs = {side: backed_off(pose, centre, CLOSE_STANDOFF)
                     for side, pose in contacts.items()}
        publish_view(node, poses_pub, markers_pub, centre, {
            'lijeva kontakt': contacts['left'],
            'desna kontakt': contacts['right'],
        })
        node.get_logger().info(
            f'both hands to {CLOSE_STANDOFF * 100:.0f} cm off the faces, '
            f'level at z = {centre.z:.3f} m')
        results = node.approach_both_linear(
            {side: (f'{side}_arm', f'{side}_end_effector_link', pose)
             for side, pose in standoffs.items()},
            'standoff', min_duration=4.0)
        if results is None or not all(results.values()):
            raise RuntimeError(f'could not reach the standoff: {results}')
        for side, pose in standoffs.items():
            ee = f'{side}_end_effector_link'
            node._wait_settle(ee)
            here = node._ee_pose(ee)
            if here is not None:
                node.get_logger().info(
                    f'{side} standoff height {here.position.z:.3f} m '
                    f'(commanded {pose.position.z:.3f})')

        # 6. Both hands close together and stop on four pads.
        if not squeeze_together(node, contacts):
            pads = {side: node.tips_on_box(side) for side in ('left', 'right')}
            raise RuntimeError(
                f'only {pads["left"] + pads["right"]} of 4 pads are on the box '
                f'(left {pads["left"]}/2, right {pads["right"]}/2)')
        pads = {side: node.tips_on_box(side) for side in ('left', 'right')}
        node.get_logger().info(
            f'pads on the box: left {pads["left"]}/2, right {pads["right"]}/2')
        if not all(count >= 2 for count in pads.values()):
            raise RuntimeError(
                f'only {pads["left"] + pads["right"]} of 4 pads are on the box')
        print('BOTH_HANDS_ON_THE_CUBE', flush=True)
        if args.no_lift:
            return

        # 8. Lift with the carriages, and report what they actually did.
        before = node._joint.get('torso_left_carriage_joint', (0.0, 0.0))[0]
        target = before + LIFT_HEIGHT
        node.move_torso(target, f'lift {LIFT_HEIGHT * 100:.0f} cm', secs=8)
        node._wait_settle('left_end_effector_link')
        after = node._joint.get('torso_left_carriage_joint', (0.0, 0.0))[0]
        node.get_logger().info(
            f'carriages: commanded {target:.4f} m, measured {after:.4f} m '
            f'(moved {after - before:+.4f} m of {LIFT_HEIGHT:.2f})')
        if abs(after - target) > 0.01:
            raise RuntimeError(
                f'the carriages did not lift: {after:.4f} m against '
                f'{target:.4f} commanded (P-13)')
        print('CUBE_LIFTED', flush=True)
    finally:
        node._send_vel(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
