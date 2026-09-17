#!/usr/bin/env python3
"""Open-hand cube grasp using Gen3 torque feedback and camera observations.

Gazebo contacts are evaluation-only. Run --pregrasp-only to inspect geometry,
--characterize to collect a bounded no-lift force trial, or provide a passed
--qualification JSON to enable the 2 cm / 20 cm lift, hold and lower cycle.
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
from pas_dual_arm_scripts.force_grasp import ForceGrasp

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

# The face markers sit this far above the face centre (seminar_world.sdf).
MARKER_RAISE = 0.056

CUBE_WIDTH = 0.30
WIDTH_TOLERANCE = 0.03

# From the 2 cm standoff both hands approach together at this Cartesian rate.
# Each arm is stopped independently as soon as either of its pads touches.
APPROACH_SPEED = 0.002       # 2 mm/s

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
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if not 0 <= node.get_clock().now().nanoseconds * 1e-9 - stamp <= 0.4:
            rclpy.spin_once(node, timeout_sec=0.02)
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

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-lift', action='store_true',
                        help='stop once both hands are on the cube')
    parser.add_argument('--pregrasp-only', action='store_true',
                        help='stop at the pre-grasp poses, touch nothing')
    parser.add_argument('--characterize', action='store_true',
                        help='bounded no-lift trial to validate force sensing')
    parser.add_argument('--qualification', default='',
                        help='passed force/friction qualification JSON')
    args, _ = parser.parse_known_args()
    if not (args.pregrasp_only or args.characterize or args.qualification):
        parser.error('lift/no-lift squeeze requires --qualification; use --characterize for validation')

    rclpy.init()
    node = MainTask(node_name='cube_grasp_node', simulated_contacts=False)
    force = ForceGrasp(node, args.qualification, args.characterize)
    poses_pub = node.create_publisher(PoseArray, '/cube_grasp/poses', LATCHED)
    markers_pub = node.create_publisher(
        MarkerArray, '/cube_grasp/markers', LATCHED)
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        wait_for_motion_ownership(node)

        force.check_environment(require_qualification=not args.pregrasp_only)

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
            if not node.verify_reached(ee, pre, tol=0.02, label=f'{side} pre-grasp'):
                raise RuntimeError(f'{side} pregrasp tracking failed')
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
        force.run(contacts, no_lift=args.no_lift)
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
