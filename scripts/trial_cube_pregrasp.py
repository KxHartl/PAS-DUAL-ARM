#!/usr/bin/env python3
"""Measure the cube, then bring BOTH hands to their pre-grasp points together.

The two arms are planned separately but released at the same instant, stretched
to a common duration: a hand that arrives first pushes the cube across the table
before the opposite hand is there to balance it. Every pose the grasp is derived
from is published for RViz (`/cube_grasp/poses`, `/cube_grasp/markers`), so the
run can be watched rather than inferred from the log.
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

# How far ahead of base_link the cube centre should end up. Measured from the
# URDF: the left shoulder sits at (0.000, 0.153, 0.386) with the carriages at
# their lower limit, so a contact pose at (d, 0.265, 0.821) needs a reach of
# 0.871 m at d = 0.746 and 0.750 m at d = 0.600. The Gen3 reaches 0.902 m to
# the flange, but nothing like that with the wrist turned sideways - which is
# why both arms failed IK at 0.746 m.
TARGET_RANGE = 0.62

# Clear distance between the cube FACE and the fingertips at the pre-grasp
# pose. squeeze_poses places the end-effector link at
# 0.15 (half the cube) + tip_standoff (0.115) + PRE_STANDOFF from the centre,
# so this is measured tip-to-face, not wrist-to-face. Doubled from 0.10 m: at
# a hand's breadth the pads crowd the cube, and the wrist camera is too close
# to hold the whole face marker in frame.
PRE_STANDOFF = 0.20

# Robotiq 2F-85 knuckle: 0.0 fully open, 0.8 fully closed. The hands arrive
# OPEN. Closed pads sit in front of the wrist camera and crowd its view of the
# face marker, and an open hand puts BOTH fingertip pads on the flat face
# instead of one closed block - four contact points across the two hands
# rather than two, which is what the four fingertip contact sensors are there
# to confirm, and far steadier against yaw.
GRIPPER_OPEN = 0.0

# How far above the face centre the grasp markers sit, from seminar_world.sdf.
# Subtracting it turns a marker reading into the point on the face itself.
MARKER_RAISE = 0.056

# The cube is 0.30 m. Read back from the two face markers it is a check on the
# whole chain: wrong marker, wrong frame or a flipped PnP solution all show up
# as a width that is not 0.30.
CUBE_WIDTH = 0.30
WIDTH_TOLERANCE = 0.03

# The last stretch is walked in short steps rather than taken in one move, and
# the fingertip contact sensors are read between them. A single straight run to
# the contact pose arrives at whatever speed the planner timed it at - 1.4 s
# for 0.20 m in the first trial - and shoves the cube across the table before
# anything has a chance to notice a pad has landed.
CREEP_STEP = 0.02
CREEP_FINE_STEP = 0.005
CREEP_SECONDS = 2.0

# Allowed travel past the commanded contact pose. The pads are meant to be
# stopped by the CUBE, not by arriving at a pose: a couple of centimetres of
# interference makes contact certain even if the face is read a little short.
# Nothing presses harder for it - the step stops the instant a pad reports.
CREEP_OVERSHOOT = 0.025

# Give up rather than creep forever if the pads never report.
CREEP_MAX_STEPS = 40

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--contact', action='store_true',
        help='after the pre-grasp, close onto the faces using what the wrist '
             'cameras see. Off by default: the pre-grasp alone is the safe run.')
    args, _ = parser.parse_known_args()

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

        # Open first, then measure: the stand-off is read from where the pads
        # actually are, and an open hand's pads sit differently from a closed
        # one's.
        for label, client in (('left', node.left_grip),
                              ('right', node.right_grip)):
            if not node.set_gripper(client, GRIPPER_OPEN, f'open {label} hand'):
                raise RuntimeError(f'{label} gripper did not open')
        node.measure_tip_standoff()
        pre_left, pre_right = node.squeeze_poses(
            center, axis, pre=PRE_STANDOFF)
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

        if not args.contact:
            return

        # From here the head camera is out of it. Each hand has its own marker
        # in view at about 0.29 m, and reads it to a few millimetres - an order
        # better than a measurement taken a metre away, which is the whole
        # reason the faces carry markers.
        faces = {}
        for side in ('left', 'right'):
            face = face_from_marker(node, side)
            if face is None:
                raise RuntimeError(
                    f'{side} wrist sees no marker; cannot close on the face')
            faces[side] = face
            node.get_logger().info(
                f'{side} face from its own camera: ({face.x:.3f}, '
                f'{face.y:.3f}, {face.z:.3f})')

        span = math.dist((faces['left'].x, faces['left'].y, faces['left'].z),
                         (faces['right'].x, faces['right'].y, faces['right'].z))
        node.get_logger().info(f'cube width between the two faces: {span:.3f} m')
        if abs(span - CUBE_WIDTH) > WIDTH_TOLERANCE:
            raise RuntimeError(
                f'the two faces are {span:.3f} m apart, not {CUBE_WIDTH:.2f} - '
                'refusing to press on a reading that cannot be the cube')

        # Re-derive the cube from what the hands see, not from the approach
        # measurement, and let the existing geometry place the pads.
        centre = Point(x=0.5 * (faces['left'].x + faces['right'].x),
                       y=0.5 * (faces['left'].y + faces['right'].y),
                       z=0.5 * (faces['left'].z + faces['right'].z))
        span_xy = math.hypot(faces['left'].x - faces['right'].x,
                             faces['left'].y - faces['right'].y)
        axis = ((faces['left'].x - faces['right'].x) / span_xy,
                (faces['left'].y - faces['right'].y) / span_xy)
        node.get_logger().info(
            f'cube re-derived from the hands: centre ({centre.x:.3f}, '
            f'{centre.y:.3f}, {centre.z:.3f}), axis ({axis[0]:+.3f}, '
            f'{axis[1]:+.3f})')

        contact_left, contact_right = node.squeeze_poses(centre, axis)
        publish_view(node, poses_pub, markers_pub, centre, {
            'lijeva kontakt': contact_left, 'desna kontakt': contact_right,
        })

        # Walk in a step at a time, and watch each hand on its own. A hand
        # with nothing touching takes a full step; once ONE of its two pads
        # reports the box it drops to a fine step, so the second pad settles
        # rather than slams; with both pads down it holds still and becomes the
        # backstop the other hand presses the cube against. Done when all four
        # pads are on the box.
        goals = {'left': contact_left, 'right': contact_right}
        limits = {}
        headings = {}
        for side, goal in goals.items():
            ee = f'{side}_end_effector_link'
            node._wait_settle(ee)
            here = node._ee_pose(ee)
            if here is None:
                raise RuntimeError(f'no TF for {ee}')
            dx = goal.position.x - here.position.x
            dy = goal.position.y - here.position.y
            dz = goal.position.z - here.position.z
            span = math.sqrt(dx * dx + dy * dy + dz * dz)
            if span < 1e-4:
                raise RuntimeError(f'{side} is already at its contact pose')
            headings[side] = (dx / span, dy / span, dz / span)
            limits[side] = Point(
                x=goal.position.x + headings[side][0] * CREEP_OVERSHOOT,
                y=goal.position.y + headings[side][1] * CREEP_OVERSHOOT,
                z=goal.position.z + headings[side][2] * CREEP_OVERSHOOT)
            node.get_logger().info(
                f'{side}: {span:.3f} m to contact, then at most '
                f'{CREEP_OVERSHOOT * 100:.1f} cm of interference')

        for step in range(1, CREEP_MAX_STEPS + 1):
            pads = {side: node.tips_on_box(side) for side in goals}
            if all(count >= 2 for count in pads.values()):
                break

            moving = {}
            for side, goal in goals.items():
                if pads[side] >= 2:
                    continue                      # holding, backstopping
                ee = f'{side}_end_effector_link'
                here = node._ee_pose(ee)
                if here is None:
                    raise RuntimeError(f'no TF for {ee}')
                left_to_go = math.dist(
                    (here.position.x, here.position.y, here.position.z),
                    (limits[side].x, limits[side].y, limits[side].z))
                if left_to_go < 1e-3:
                    continue                      # out of allowed travel
                step_size = min(
                    CREEP_FINE_STEP if pads[side] == 1 else CREEP_STEP,
                    left_to_go)
                pose = Pose()
                pose.orientation = goal.orientation
                pose.position = Point(
                    x=here.position.x + headings[side][0] * step_size,
                    y=here.position.y + headings[side][1] * step_size,
                    z=here.position.z + headings[side][2] * step_size)
                moving[side] = (f'{side}_arm', ee, pose)

            if not moving:
                break

            detail = ', '.join(
                f'{side} {pads[side]}/2 pads, '
                f'{"fino" if pads[side] == 1 else "normalno"}'
                for side in sorted(moving))
            node.get_logger().info(f'step {step}: moving {detail}')
            results = node.approach_both_linear(
                moving, f'cube contact step {step}',
                min_duration=CREEP_SECONDS)
            if results is None or not all(results.values()):
                raise RuntimeError(f'step {step} did not execute: {results}')

        pads = {side: node.tips_on_box(side) for side in goals}
        node.get_logger().info(
            f'pads on the box: left {pads["left"]}/2, right {pads["right"]}/2')
        for side, goal in goals.items():
            ee = f'{side}_end_effector_link'
            here = node._ee_pose(ee)
            if here is not None:
                gap = math.dist(
                    (here.position.x, here.position.y, here.position.z),
                    (goal.position.x, goal.position.y, goal.position.z))
                node.get_logger().info(
                    f'{side} stopped {gap * 1000:.0f} mm from the commanded '
                    'contact pose')
        # Honest report. The first trial printed success while the hands were
        # 0.115 and 0.150 m off, because nothing read the check back. Stopping
        # short of the commanded pose is fine and expected - the pads are what
        # decides, and four of them have to say so.
        if not all(count >= 2 for count in pads.values()):
            raise RuntimeError(
                f'only {pads["left"] + pads["right"]} of 4 pads are on the box '
                f'(left {pads["left"]}/2, right {pads["right"]}/2)')
        print('BOTH_HANDS_ON_THE_CUBE', flush=True)
    finally:
        node._send_vel(0.0, 0.0)
        node.destroy_node()
        # On Ctrl-C the signal handler has already shut the context down, and
        # calling it again raises over the real exit reason.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
