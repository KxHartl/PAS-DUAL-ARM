"""Stage the cube grasp so grasp poses can be designed by hand.

Starts from the drive posture (ARM_CARRY_V2) at the table's dock pose and
leaves the robot in front of the table with the cube free on it and both hands
in the side-scanning pre-grasp: open, `pre_standoff` off the two faces, each
wrist camera looking at the marker on its own face. Nothing touches the cube.

Order matters. The hands change posture at the dock, where ARM_CARRY_V2 is
15 cm clear of the table, and only then does the base drive in: carried up to
the cube, the carry posture's hands end up under the tabletop and among the
cube (measured, P-43). The scan posture clears both by 6 and 20 cm on the way.
From there grasp poses are dialled in with `scripts/joint_gui.py --sim` and
tried on the simulated arms (notes/00_run/00_testing/definiranje_poza_hvata.md).

This is the pre-grasp validated in run 72 and by `grasp_cube.py
--pregrasp-only`, without the shelved force layer that script pulls in. The
cube is found by the head camera, not taken from the simulator: the poses a
person designs here are meant to carry over to the mission, which has only the
cameras.

    ros2 launch pas_dual_arm_bringup grasp_stage.launch.py
"""

import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseArray
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from pas_dual_arm_scripts.main_task import MainTask

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CARRIAGES = ('torso_left_carriage_joint', 'torso_right_carriage_joint')

# What stops the base (grasp_cube.py): everything below 0.23 m passes under the
# tabletop, so the limit is the torso frame at x = +0.283 m against the table's
# near edge, 0.25 m in front of the cube centre.
TORSO_FRONT = 0.283
TABLE_EDGE_AHEAD_OF_CUBE = -0.25
EDGE_CLEARANCE = 0.03
BASE_APPROACH_SPEED = 0.08
PREGRASP_VELOCITY_SCALE = 0.40
GRIPPER_OPEN = 0.0

# The face markers sit this far above the face centre (seminar_world.sdf).
MARKER_RAISE = 0.056
CUBE_WIDTH = 0.30


def measure_with_head(node):
    """Cube centre and face axis from the head camera: marker plus depth."""
    marker = node.confirm_box(timeout=10.0, samples=5)
    if marker is None:
        raise RuntimeError('no fresh marker from the head camera')
    depth = node.measure_box(marker, timeout=10.0)
    if depth is None:
        raise RuntimeError('no accepted depth cluster')
    center, length, height, _ = depth
    if math.hypot(center.x - marker.x, center.y - marker.y) > 0.06:
        raise RuntimeError('marker and depth disagree in XY')
    if abs(center.z - marker.z) > 0.05:
        raise RuntimeError('marker and depth disagree in Z')
    axis = node.marker_tangent()
    if axis is None:
        raise RuntimeError('no marker face orientation')
    node.get_logger().info(
        f'cube: long axis {length:.3f} m, visible height {height:.3f} m, '
        f'centre ({center.x:.3f}, {center.y:.3f}, {marker.z:.3f})')
    # X and Y from the fused estimate, Z from the marker: the marker is at the
    # middle of the face, while the cloud sees only the upper part of the cube.
    return Point(x=center.x, y=center.y, z=marker.z), axis


def face_from_marker(node, side, timeout=5.0):
    """The point on this hand's face, as its own wrist camera sees it."""
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
    """Show the cube estimate and the target poses in RViz (latched)."""
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


STARTUP_MOVERS = ('cube_table_ready', 'set_posture')


def wait_for_table_ready(node, no_show=30.0, timeout=300.0):
    """Let the spawn staging finish before this node moves anything.

    sim.launch.py starts one startup mover after the controllers come up:
    `set_posture` folding the arms into ARM_CARRY_V2, or `table_ready` with
    table_arms. Two nodes commanding the same controllers is how arms get
    yanked between two targets, so wait until it has come and gone - or, if
    none appears once the controllers are up, until that is clearly not going
    to happen.
    """
    if not node.torso.wait_for_server(timeout_sec=timeout):
        raise RuntimeError('torso controller never came up')
    deadline = time.monotonic() + timeout
    seen, quiet_since = False, None
    while rclpy.ok() and time.monotonic() < deadline:
        names = node.get_node_names()
        if 'main_task_node' in names:
            raise RuntimeError(
                'main_task_node is running; start task.launch.py with auto_start:=false')
        if any(mover in names for mover in STARTUP_MOVERS):
            if not seen:
                node.get_logger().info('waiting for the startup posture node to finish')
            seen, quiet_since = True, None
        else:
            quiet_since = quiet_since or time.monotonic()
            if seen or time.monotonic() - quiet_since > no_show:
                return
        rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError('the startup posture node did not finish')


def carriage_heights(node, settle=2.0):
    end = time.monotonic() + settle
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return [node._joint.get(n, (float('nan'), 0.0))[0] for n in CARRIAGES]


def ensure_carriages(node, height):
    """Carriages at `height`, measured - the action status is not trusted."""
    at = carriage_heights(node)
    if max(abs(a - height) for a in at) > 0.01:
        if not node.move_torso(height, 'stage carriages', secs=8):
            raise RuntimeError('carriages did not accept the command')
        at = carriage_heights(node, settle=3.0)
    node.get_logger().info(
        f'CARRIAGE MEASURED left={at[0]:.4f} right={at[1]:.4f} m (wanted {height:.4f})')
    if max(abs(a - height) for a in at) > 0.01:
        raise RuntimeError(f'carriages at {at}, wanted {height:.3f}')


def current_arm(node, side):
    return {f'{side}_joint_{j}': node._joint[f'{side}_joint_{j}'][0]
            for j in range(1, 8) if f'{side}_joint_{j}' in node._joint}


def scan_solutions(node, pose_left, pose_right):
    """Left arm from IK near where it is now; right arm as its mirror image.

    Two independent IK calls return two unrelated branches: run S2 reached
    mirror-image HAND poses with the left arm at j1 -180 deg and the right at
    +30 deg, so the arms looked nothing alike. Solving one arm and mirroring
    it makes the whole arm symmetric, not only the hands.
    """
    left = node._ik('left_arm', 'left_end_effector_link', pose_left,
                    seed=current_arm(node, 'left'), avoid_collisions=True)
    if left is None:
        left = node._ik('left_arm', 'left_end_effector_link', pose_left,
                        avoid_collisions=True)
    if left is None:
        raise RuntimeError('left scan pose has no collision-free IK')
    right = None
    try:
        import PyKDL as kdl
        from pas_dual_arm_scripts.force_model import ArmModel
        from pas_dual_arm_scripts.kinematics import Kinematics, mirror_right
        deadline = time.monotonic() + 5.0
        while node._description is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        model = ArmModel(node._description, 'right')
        q = {name: value[0] for name, value in node._joint.items()}
        q.update(left)
        joints, pos_err, rot_err = mirror_right(
            Kinematics(node._description), model,
            kdl.ChainIkSolverPos_LMA(model.chain, 1e-7, 500), q)
        if joints is not None and pos_err < 0.002 and rot_err < 0.01:
            right = {f'right_joint_{j + 1}': v for j, v in enumerate(joints)}
            node.get_logger().info(
                f'SYMMETRIC: right arm is the mirror of the left '
                f'({pos_err * 1000:.2f} mm, rotation {rot_err:.1e})')
    except Exception as exc:
        node.get_logger().warn(f'mirroring failed: {exc}')
    if right is None:
        node.get_logger().warn('right arm solved on its own - arms NOT symmetric')
        right = node._ik('right_arm', 'right_end_effector_link', pose_right,
                         seed=current_arm(node, 'right'), avoid_collisions=True)
        if right is None:
            raise RuntimeError('right scan pose has no collision-free IK')
    return {'left': left, 'right': right}


def base_pose(node):
    """(x, y, yaw) of the base in the odometry frame, from the latest sample."""
    msg = node._last_odom
    if msg is None:
        return None
    p, o = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
    return p.x, p.y, yaw


def follow_scene(node, centre):
    """Keep MoveIt's table and cube where they really are while the base moves.

    MoveIt plans in the robot's own frame (no virtual joint to the world), so
    the scene published at staging rides along with the base: drive away with
    teleop_twist_keyboard and "Posalji (MoveIt)" would still dodge a table that
    is no longer there. The cube and the table heading are pinned in the
    odometry frame here and re-expressed in base_link whenever the base has
    moved 5 mm or 0.5 deg. Odometry drifts with wheel slip, and a cube pushed
    by a hand is not tracked - the head camera would have to measure it again.
    """
    start = None
    while rclpy.ok() and start is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        start = base_pose(node)
    ox, oy, oyaw = start
    cube_x = ox + math.cos(oyaw) * centre.x - math.sin(oyaw) * centre.y
    cube_y = oy + math.sin(oyaw) * centre.x + math.cos(oyaw) * centre.y
    last = logged = start
    node.get_logger().info(
        'SCENE FOLLOWS THE BASE: drive with teleop_twist_keyboard, MoveIt keeps '
        'the table and cube where they are (odometry)')
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        now = base_pose(node)
        if now is None:
            continue
        turned = math.atan2(math.sin(now[2] - last[2]), math.cos(now[2] - last[2]))
        if math.hypot(now[0] - last[0], now[1] - last[1]) < 0.005 and abs(turned) < math.radians(0.5):
            continue
        x, y, yaw = now
        dx, dy = cube_x - x, cube_y - y
        local = Point(x=math.cos(yaw) * dx + math.sin(yaw) * dy,
                      y=-math.sin(yaw) * dx + math.cos(yaw) * dy, z=centre.z)
        heading = math.atan2(math.sin(oyaw - yaw), math.cos(oyaw - yaw))
        node.publish_collision_scene(local, table_top_z=centre.z - 0.15,
                                     yaw=heading, settle=False)
        last = now
        if (math.hypot(now[0] - logged[0], now[1] - logged[1]) > 0.05
                or abs(math.atan2(math.sin(now[2] - logged[2]),
                                  math.cos(now[2] - logged[2]))) > math.radians(5.0)):
            node.get_logger().info(
                f'scene moved with the base: cube now ({local.x:.3f}, {local.y:.3f}) '
                f'in base_link, table turned {math.degrees(heading):+.1f} deg')
            logged = now


def log_arm_joints(node):
    """Print both arms in the posture-registry format, radians and degrees."""
    for side in ('left', 'right'):
        values = {j: node._joint.get(f'{side}_joint_{j}', (float('nan'), 0.0))[0]
                  for j in range(1, 8)}
        node.get_logger().info(
            f'SCAN_{side.upper()} = {{' +
            ', '.join(f'{j}: {v:.3f}' for j, v in values.items()) + '}  (deg: ' +
            ', '.join(f'j{j} {math.degrees(math.atan2(math.sin(v), math.cos(v))):.0f}'
                      for j, v in values.items()) + ')')


def stage(node, poses_pub, markers_pub):
    height = float(node.get_parameter('carriage_height').value)
    target_range = float(node.get_parameter('target_range').value)
    standoff = float(node.get_parameter('pre_standoff').value)
    tilt = float(node.get_parameter('scan_tilt').value)

    wait_for_table_ready(node)
    ensure_carriages(node, height)
    # The DetachableJoint starts attached to the left wrist: released here, or
    # the cube follows the first arm motion.
    node._attach_box(False)

    if not node.aim_camera(0.0, 0.65, 'stage camera aim'):
        raise RuntimeError('pan/tilt command failed')
    centre, axis = measure_with_head(node)
    wanted = centre.x - target_range
    allowed = centre.x + TABLE_EDGE_AHEAD_OF_CUBE - TORSO_FRONT - EDGE_CLEARANCE
    advance = max(0.0, min(wanted, allowed))
    if advance <= 0.02:
        advance = 0.0
    node.get_logger().info(
        f'approach: cube at {centre.x:.3f} m, want {target_range:.3f}, '
        f'table allows {allowed:.3f} -> advancing {advance:.3f} m')
    # Where the cube will be once the base has driven in. The arm is fixed to
    # base_link, so the scan pose is solved for that position now and simply
    # carried there by the drive.
    arrived = Point(x=centre.x - advance, y=centre.y, z=centre.z)

    if not node.set_both_grippers(GRIPPER_OPEN, 'stage open hands'):
        raise RuntimeError('grippers did not open')
    node.measure_tip_standoff()
    pre_left, pre_right = node.squeeze_poses(arrived, axis, pre=standoff, tilt=tilt)
    publish_view(node, poses_pub, markers_pub, arrived,
                 {'lijevo skeniranje': pre_left, 'desno skeniranje': pre_right})
    # Planning happens HERE, at the dock, so the scene holds the cube where it
    # is now.
    node.publish_collision_scene(centre, table_top_z=centre.z - 0.15)

    solutions = scan_solutions(node, pre_left, pre_right)
    targets, trajs = {}, {}
    for side, pose in (('left', pre_left), ('right', pre_right)):
        group, ee = f'{side}_arm', f'{side}_end_effector_link'
        # The path out of the carry posture threads past the near table leg.
        # RRT is random: the same request succeeded headless (S6) and came back
        # "found but invalid after postprocessing" - a fingertip clipping the
        # leg - in the user's GUI run. A fresh attempt is a fresh path.
        traj = None
        for attempt in range(1, 4):
            traj = node.plan_arm_joints(
                group, solutions[side], f'{side} scan pose (attempt {attempt})',
                velocity_scale=PREGRASP_VELOCITY_SCALE,
                acceleration_scale=PREGRASP_VELOCITY_SCALE)
            if traj is not None:
                break
        if traj is None:
            raise RuntimeError(f'no plan to the {side} scan pose after 3 attempts')
        targets[side], trajs[side] = (ee, pose), traj
    # Both or neither, released together (P-26).
    results = node.move_arms_parallel(trajs, 'stage scan pose', min_duration=4.0)
    if not all(results.values()):
        raise RuntimeError(f'scan pose move failed: {results}')
    for side, (ee, pose) in targets.items():
        node._wait_settle(ee)
        if not node.verify_reached(ee, pose, tol=0.02, label=f'{side} scan pose'):
            raise RuntimeError(f'{side} hand did not reach the scan pose')

    if advance > 0.0:
        travelled = node.drive_distance(advance, speed=BASE_APPROACH_SPEED)
        if travelled is None or travelled > advance + 0.03:
            raise RuntimeError('base advance did not track odometry')
        node.get_logger().info(f'drove in {travelled:.3f} m with the hands in the scan pose')
        # The head camera is NOT asked again: its depth window reaches 0.25 m
        # around the cube and the open fingertips are now 0.20 m off the faces,
        # inside it. The wrist cameras below are the measurement from here.
        centre = Point(x=centre.x - travelled, y=centre.y, z=centre.z)
        node.publish_collision_scene(centre, table_top_z=centre.z - 0.15)

    faces = {side: face_from_marker(node, side) for side in ('left', 'right')}
    for side, face in faces.items():
        if face is None:
            node.get_logger().warn(f'WRIST {side}: marker NOT seen')
        else:
            node.get_logger().info(
                f'WRIST {side}: face centre at ({face.x:.3f}, {face.y:.3f}, {face.z:.3f})')
    if all(faces.values()):
        width = math.dist((faces['left'].x, faces['left'].y, faces['left'].z),
                          (faces['right'].x, faces['right'].y, faces['right'].z))
        node.get_logger().info(
            f'WRIST cameras: faces {width:.3f} m apart (cube {CUBE_WIDTH:.2f}), '
            f'cube centre ({(faces["left"].x + faces["right"].x) / 2:.3f}, '
            f'{(faces["left"].y + faces["right"].y) / 2:.3f}, '
            f'{(faces["left"].z + faces["right"].z) / 2:.3f}) '
            f'vs odometry estimate ({centre.x:.3f}, {centre.y:.3f}, {centre.z:.3f})')
    node.measure_width('scan pose')
    log_arm_joints(node)
    node.get_logger().info(
        'STAGE_READY - hands in the scan pose, cube free on the table. '
        'Design grasp poses with: scripts/joint_gui.py --sim')
    return centre


def main():
    rclpy.init()
    node = MainTask(node_name='grasp_stage_node')
    node.declare_parameter('carriage_height', 0.20)
    node.declare_parameter('target_range', 0.62)
    node.declare_parameter('pre_standoff', 0.20)
    node.declare_parameter('scan_tilt', 0.0)
    poses_pub = node.create_publisher(PoseArray, '/cube_grasp/poses', LATCHED)
    markers_pub = node.create_publisher(MarkerArray, '/cube_grasp/markers', LATCHED)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        centre = stage(node, poses_pub, markers_pub)
        # Stay up: RViz keeps the latched cube and poses, and the MoveIt scene
        # follows the base if it is driven by hand. Nothing here moves the robot.
        follow_scene(node, centre)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        node.get_logger().error(f'STAGE FAILED: {exc}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
