#!/usr/bin/env python3
"""Find the cube, take hold of it with both hands, and lift it. One command.

Stages, in order:
  1. look at the cube and measure it with the head camera
  2. close the base in to a range the arms can actually work at
  3. open both hands and take them to the pre-grasp poses, together
  4. read the marker on each pressed face with that hand's own camera
  5. bring both hands to a standoff off the faces, LEVEL with each other
  6. approach together; stop each hand on its first pad contact
  7. advance only incomplete hands in 2 mm steps until all four pads report
     the cube
  8. hold for one second, then lift 20 cm with the torso carriages

The hands are levelled at the standoff and never asked to change height again.
Correcting height while pressing is what skewed the cube: a wrist shifting in z
against a face it is already touching tilts it. Contact loss stops the lift and
returns to the same bounded fine correction; no rigid attach is used.
"""

import argparse
import math
import time

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

# ... and at the standoff from which contact approach starts.
CLOSE_STANDOFF = 0.02
STANDOFF_SPEED = 0.025       # 25 mm/s from pre-grasp to standoff
STANDOFF_PAUSE = 1.0         # visibly stop before beginning contact approach
BASE_APPROACH_SPEED = 0.08   # m/s before arm positioning
PREGRASP_VELOCITY_SCALE = 0.40

GRIPPER_OPEN = 0.0
GRIPPER_GRASP = 0.7

# The face markers sit this far above the face centre (seminar_world.sdf).
MARKER_RAISE = 0.056

CUBE_WIDTH = 0.30
WIDTH_TOLERANCE = 0.03

# From the 2 cm standoff both hands approach together at this Cartesian rate.
# Each arm is stopped independently as soon as either of its pads touches.
APPROACH_SPEED = 0.002       # 2 mm/s

# Once both hands have found the box, move only one hand at a time until both
# of its pads touch.  Commands below 0.5 mm were swallowed by the simulated
# position tracking: hundreds of 0.1 mm goals produced almost no TF motion.
FINE_STEP = 0.0020           # 2.0 mm correction of an incomplete hand
FINE_SPEED = 0.005           # 5 mm/s
FINE_MAX_PAST_FACE = 0.010   # 10 mm past the marker-derived face
FINE_MIN_SECONDS = 0.10
FINE_STALL_THRESHOLD = 0.0002  # commanded 2 mm but moved less than 0.2 mm
FINE_STALL_COUNT = 3
REALIGN_BACKOFF = 0.003
CONTACT_MAX_AGE = 0.40       # tolerate the loaded Gazebo->ROS bridge

# Seconds to hold the grip before lifting.
HOLD_SECONDS = 1.0

# Lift height and duration — slow enough that both carriages rise in unison.
LIFT_HEIGHT = 0.20
LIFT_SPEED = 0.010           # 10 mm/s


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


def contact_counts(node):
    state = node.box_contact_snapshot(max_age=CONTACT_MAX_AGE)
    return {
        side: sum(state[f'{side}_{finger}']
                  for finger in ('left', 'right'))
        for side in ('left', 'right')
    }


def all_four_contacts(node):
    return sum(contact_counts(node).values()) == 4


def contact_status(node):
    pads = contact_counts(node)
    return f'left {pads["left"]}/2\nright {pads["right"]}/2'


def make_contact_limits(node, contacts):
    """Fixed horizontal approach headings and bounded post-face limits."""
    headings = {}
    limits = {}
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
        if abs(span - CLOSE_STANDOFF) > 0.005:
            raise RuntimeError(
                f'{side} is {span * 1000:.1f} mm from the measured face, '
                f'not {CLOSE_STANDOFF * 1000:.0f} +/- 5 mm')
        headings[side] = (dx / span, dy / span)
        limits[side] = Point(
            x=goal.position.x + headings[side][0] * FINE_MAX_PAST_FACE,
            y=goal.position.y + headings[side][1] * FINE_MAX_PAST_FACE,
            z=goal.position.z)
        node.get_logger().info(
            f'{side}: {span * 1000:.1f} mm slow approach to the face, then '
            f'at most {FINE_MAX_PAST_FACE * 1000:.1f} mm fine correction')
    return headings, limits


def fine_until_four(node, contacts, headings, limits):
    """Correct only incomplete hands while preserving level height."""
    step = 0
    stalled = {'left': 0, 'right': 0}
    while rclpy.ok():
        pads = contact_counts(node)
        if sum(pads.values()) == 4:
            return True

        moved = False
        for side in ('left', 'right'):
            pads = contact_counts(node)
            if sum(pads.values()) == 4:
                return True
            if pads[side] >= 2:
                continue

            ee = f'{side}_end_effector_link'
            here = node._ee_pose(ee)
            if here is None:
                raise RuntimeError(f'no TF for {ee}')
            hx, hy = headings[side]
            remaining = sum((
                (limits[side].x - here.position.x) * hx,
                (limits[side].y - here.position.y) * hy,
            ))
            if remaining < 0.0001:
                node.get_logger().error(
                    f'{side}: fine correction exhausted at '
                    f'{FINE_MAX_PAST_FACE * 1000:.1f} mm past the face')
                return False

            size = min(FINE_STEP, remaining)
            pose = Pose()
            # Keep the marker-derived square orientation, but never correct Z
            # while a pad is loaded against the cube.
            pose.orientation = contacts[side].orientation
            pose.position = Point(
                x=here.position.x + hx * size,
                y=here.position.y + hy * size,
                z=here.position.z)
            step += 1
            node.get_logger().info(
                f'fine step {step}: {side} {size * 1000:.1f} mm')
            result = node.approach_both_linear(
                {side: (f'{side}_arm', ee, pose)},
                f'fine contact {side} step {step}', min_frac=0.5,
                min_duration=max(FINE_MIN_SECONDS, size / FINE_SPEED))
            if result is None or not result.get(side, False):
                raise RuntimeError(f'fine contact step failed for {side}')
            after = node._ee_pose(ee)
            if after is None:
                raise RuntimeError(f'no TF for {ee} after fine step')
            actual = sum((
                (after.position.x - here.position.x) * hx,
                (after.position.y - here.position.y) * hy,
            ))
            nominal_remaining = sum((
                (contacts[side].position.x - after.position.x) * hx,
                (contacts[side].position.y - after.position.y) * hy,
            ))
            node.get_logger().info(
                f'fine step {step}: {side} actual advance '
                f'{actual * 1000:+.3f} mm, wrist remaining to nominal '
                f'contact {nominal_remaining * 1000:+.2f} mm')
            node.get_logger().info(contact_status(node))
            if actual < FINE_STALL_THRESHOLD:
                stalled[side] += 1
            else:
                stalled[side] = 0
            if stalled[side] >= FINE_STALL_COUNT:
                node.get_logger().warn(
                    f'{side}: arm is loaded and stalled; backing off '
                    f'{REALIGN_BACKOFF * 1000:.1f} mm to realign')
                back = Pose()
                back.orientation = contacts[side].orientation
                back.position = Point(
                    x=after.position.x - hx * REALIGN_BACKOFF,
                    y=after.position.y - hy * REALIGN_BACKOFF,
                    z=after.position.z)
                recovered = node.approach_both_linear(
                    {side: (f'{side}_arm', ee, back)},
                    f'realign backoff {side}', min_frac=0.5,
                    min_duration=REALIGN_BACKOFF / FINE_SPEED)
                if recovered is None or not recovered.get(side, False):
                    raise RuntimeError(f'realign backoff failed for {side}')
                stalled[side] = 0
            moved = True

        if not moved:
            return False


def hold_four_contacts(node, seconds):
    """Require all four fresh box contacts continuously for ``seconds``."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not all_four_contacts(node):
            return False
        rclpy.spin_once(node, timeout_sec=0.01)
    return True


def wait_for_motion_ownership(node, timeout=30.0):
    """Do not compete with startup staging or the legacy orchestrator."""
    deadline = time.monotonic() + timeout
    announced = False
    while rclpy.ok():
        names = node.get_node_names()
        if names.count('cube_grasp_node') > 1:
            raise RuntimeError('another cube_grasp_node is already running')
        if 'main_task_node' in names:
            raise RuntimeError(
                'legacy main_task_node is running; launch task.launch.py with '
                'auto_start:=false')
        if 'cube_table_ready' not in names:
            break
        if not announced:
            node.get_logger().info(
                'Waiting for cube_table_ready to release arm/torso control')
            announced = True
        if time.monotonic() >= deadline:
            raise RuntimeError('cube_table_ready did not finish within 30 s')
        rclpy.spin_once(node, timeout_sec=0.05)

    sensor_deadline = time.monotonic() + 10.0
    while rclpy.ok():
        missing = [topic for topic in node.tip_contact_topics.values()
                   if node.count_publishers(topic) == 0]
        if not missing:
            node.get_logger().info(
                'All four fingertip contact topics have publishers')
            return
        if time.monotonic() >= sensor_deadline:
            raise RuntimeError(
                'contact sensor bridge missing publishers: '
                + ', '.join(missing))
        rclpy.spin_once(node, timeout_sec=0.05)


def squeeze_together(node, contacts):
    """Slow simultaneous approach, independent stop, then alternating trim."""
    headings, limits = make_contact_limits(node, contacts)
    targets = {}
    distances = []
    for side, pose in contacts.items():
        here = node._ee_pose(f'{side}_end_effector_link')
        if here is None:
            raise RuntimeError(f'no TF for {side}_end_effector_link')
        target = Pose()
        target.position = pose.position
        target.orientation = pose.orientation
        targets[side] = (f'{side}_arm', f'{side}_end_effector_link', target)
        distances.append(math.hypot(pose.position.x - here.position.x,
                                    pose.position.y - here.position.y))
    duration = max(distances) / APPROACH_SPEED
    node.get_logger().info(
        f'slow contact approach at {APPROACH_SPEED * 1000:.1f} mm/s '
        f'for at most {duration:.1f} s')
    node.approach_both_until_contact(targets, 'first contact', duration)

    # Closing changes the two fingertip linkages symmetrically and supplies the
    # clamping force that an arm-only press cannot create. Keep both sides
    # synchronized so one gripper cannot rotate the cube before the other.
    node.get_logger().info('closing both grippers for fine grasp')
    if not node.set_both_grippers(
            GRIPPER_GRASP, 'fine grasp close', max_effort=20.0):
        raise RuntimeError('both grippers did not close for fine grasp')

    if not fine_until_four(node, contacts, headings, limits):
        return None
    return headings, limits


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-lift', action='store_true',
                        help='stop once both hands are on the cube')
    parser.add_argument('--pregrasp-only', action='store_true',
                        help='stop at the pre-grasp poses, touch nothing')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MainTask(node_name='cube_grasp_node')
    poses_pub = node.create_publisher(PoseArray, '/cube_grasp/poses', LATCHED)
    markers_pub = node.create_publisher(
        MarkerArray, '/cube_grasp/markers', LATCHED)
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        wait_for_motion_ownership(node)

        # This run intentionally tests a friction-only grasp.  Fortress starts
        # the detachable joint attached, so enforce the invariant locally too.
        node._attach_box(False)

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
            travelled = node.drive_distance(
                advance, speed=BASE_APPROACH_SPEED)
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
            traj = node.plan_arm_joints(
                group, joints, f'{side} pre-grasp',
                velocity_scale=PREGRASP_VELOCITY_SCALE,
                acceleration_scale=PREGRASP_VELOCITY_SCALE)
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
        standoff_distances = []
        for side, pose in standoffs.items():
            here = node._ee_pose(f'{side}_end_effector_link')
            if here is None:
                raise RuntimeError(f'no TF for {side}_end_effector_link')
            standoff_distances.append(math.dist(
                (here.position.x, here.position.y, here.position.z),
                (pose.position.x, pose.position.y, pose.position.z)))
        standoff_seconds = max(standoff_distances) / STANDOFF_SPEED
        node.get_logger().info(
            f'both hands to {CLOSE_STANDOFF * 100:.0f} cm off the faces, '
            f'level at z = {centre.z:.3f} m, at no more than '
            f'{STANDOFF_SPEED * 1000:.0f} mm/s')
        results = node.approach_both_linear(
            {side: (f'{side}_arm', f'{side}_end_effector_link', pose)
             for side, pose in standoffs.items()},
            'standoff', min_duration=standoff_seconds)
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
        node.hold_current_mechanisms(reason='standoff hold')
        node.get_logger().info(
            f'STANDOFF_REACHED: holding at {CLOSE_STANDOFF * 100:.0f} cm '
            f'for {STANDOFF_PAUSE:.0f} s before slow contact approach')
        # Let any brief contact made while entering the standoff age out. A
        # contact that is physically still present keeps refreshing and will
        # therefore still fail the safety check below.
        clear_deadline = time.monotonic() + CONTACT_MAX_AGE
        while time.monotonic() < clear_deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        pause_deadline = time.monotonic() + STANDOFF_PAUSE
        while time.monotonic() < pause_deadline:
            unexpected = contact_counts(node)
            if sum(unexpected.values()):
                raise RuntimeError(
                    f'box contact present at the '
                    f'{CLOSE_STANDOFF * 100:.0f} cm standoff '
                    f'(left {unexpected["left"]}/2, right '
                    f'{unexpected["right"]}/2); stopping before approach')
            rclpy.spin_once(node, timeout_sec=0.01)

        # 6. Both hands approach together. Each hand stops at its own first
        # contact; then only incomplete hands advance in bounded 2 mm steps.
        contact_motion = squeeze_together(node, contacts)
        if contact_motion is None:
            pads = contact_counts(node)
            raise RuntimeError(
                f'only {pads["left"] + pads["right"]} of 4 pads are on the box '
                f'(left {pads["left"]}/2, right {pads["right"]}/2)')
        headings, limits = contact_motion
        pads = contact_counts(node)
        node.get_logger().info(contact_status(node))
        if not all(count >= 2 for count in pads.values()):
            raise RuntimeError(
                f'only {pads["left"] + pads["right"]} of 4 pads are on the box')
        print('BOTH_HANDS_ON_THE_CUBE', flush=True)

        # Four contacts must remain continuous for a full second. A transient
        # loss returns to the same bounded fine-contact procedure.
        while not hold_four_contacts(node, HOLD_SECONDS):
            node.get_logger().warn(
                'contact lost during 1 s hold; resuming fine correction')
            if not fine_until_four(node, contacts, headings, limits):
                raise RuntimeError('could not restore all four contacts')
        if args.no_lift:
            return

        # 7. Friction-only lift. Contact loss cancels the carriage trajectory;
        # after re-contact and another stable second, continue to the original
        # absolute targets rather than adding another 20 cm.
        names = ('torso_left_carriage_joint', 'torso_right_carriage_joint')
        before = [node._joint.get(name, (0.0, 0.0))[0] for name in names]
        targets = [height + LIFT_HEIGHT for height in before]
        while True:
            outcome = node.move_torso_guarded(
                targets,
                f'friction lift to +{LIFT_HEIGHT * 100:.0f} cm',
                LIFT_SPEED, lambda: all_four_contacts(node))
            if outcome == 'reached':
                break
            if outcome != 'guard_failed':
                raise RuntimeError('carriage lift trajectory failed')
            if not fine_until_four(node, contacts, headings, limits):
                raise RuntimeError(
                    'contact lost during lift and could not be restored')
            while not hold_four_contacts(node, HOLD_SECONDS):
                if not fine_until_four(node, contacts, headings, limits):
                    raise RuntimeError(
                        'contact could not remain stable before lift resume')

        node._wait_settle('left_end_effector_link')
        after = [node._joint.get(name, (0.0, 0.0))[0] for name in names]
        node.get_logger().info(
            f'carriages: commanded {targets[0]:.4f}/{targets[1]:.4f} m, '
            f'measured {after[0]:.4f}/{after[1]:.4f} m '
            f'(moved {after[0] - before[0]:+.4f}/'
            f'{after[1] - before[1]:+.4f} m)')
        if max(abs(actual - target)
               for actual, target in zip(after, targets)) > 0.01:
            raise RuntimeError(
                'the carriages did not reach their independent +20 cm targets')
        if not all_four_contacts(node):
            raise RuntimeError('carriages reached the target but contact was lost')
        print('CARRIAGES_LIFTED_20CM_WITH_4_CONTACTS', flush=True)
    finally:
        node._send_vel(0.0, 0.0)
        # A completed/cancelled JTC goal keeps its final command. With the soft
        # simulated position interface the mechanism can therefore keep
        # creeping after this script raises. Replace every such command by the
        # currently measured joint state before destroying the action clients.
        node.hold_current_mechanisms(reason='shutdown hold')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
