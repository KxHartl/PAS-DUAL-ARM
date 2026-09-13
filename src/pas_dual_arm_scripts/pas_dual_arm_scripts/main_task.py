#!/usr/bin/env python3
"""End-to-end seminar task orchestration (M6).

State machine that drives the full assignment:

  1. NAV_TO_BOX   - Nav2 NavigateToPose to a pre-grasp pose in front of the
                    Aruco box (box sits at map (1.0, 0, 0.15), before the door).
  2. LOOK         - aim the pan-tilt down and wait for the aruco_marker_frame TF
                    so the box pose is confirmed from perception, not hard-coded.
  3. GRASP        - plan both Kinova arms (MoveIt2 'both_arms' group) to the two
                    side faces of the box and close both grippers.
  4. LIFT         - raise the torso carriages to pick the box off the ground.
  5. NAV_TO_TABLE - Nav2 through the 0.8 m doorway (wall at X=2.0) to the table
                    at map (4.0, 0), stopping at the pre-place pose.
  6. PLACE        - lower the box onto the table top (Z=0.775) and open grippers.
  7. RETRACT      - back the arms off and report done.

Every step is a real action call (Nav2 NavigateToPose, MoveIt2 MoveGroup,
control_msgs GripperCommand / FollowJointTrajectory); there are no time.sleep
stubs. Poses are parameters so they can be tuned against the running sim.

Run order (separate terminals, all with Fast DDS):
  ros2 launch pas_dual_arm_bringup sim.launch.py
  ros2 launch pas_dual_arm_bringup nav2.launch.py
  ros2 launch pas_dual_arm_bringup task.launch.py   # move_group + aruco + this
"""
import math
import subprocess
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from nav2_msgs.action import NavigateToPose
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    BoundingVolume,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pas_dual_arm_scripts.base_drive import BaseDriver
from pas_dual_arm_scripts.postures import (ARM_DRIVE, POSTURES, joint_constraints,
                                           move_to_posture, switch_arm_hold)

# Optional Ignition contact-sensor feedback. Bridged via ros_gz_interfaces; if the
# package or the sensor is missing, the grasp verification falls back to depth +
# gripper-stall + arm-effort (see fingertips_on_box / close_until_contact).
try:
    from ros_gz_interfaces.msg import Contacts as _Contacts
except Exception:  # pragma: no cover - bridge/package may be absent
    _Contacts = None


def quat_to_matrix(q):
    """Quaternion (geometry_msgs) -> 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_matrix(R):
    """3x3 rotation matrix -> geometry_msgs Quaternion (Shepperd's method)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


def pointcloud2_xyz(msg):
    """Extract finite XYZ points from a PointCloud2 as an (N,3) float array,
    without PCL/sensor_msgs_py (numpy only, matching aruco_detector's style)."""
    offs = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)

    def field(name):
        o = offs[name]
        return raw[:, o:o + 4].copy().view(np.float32).ravel()

    pts = np.stack([field('x'), field('y'), field('z')], axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def quat_mul(a, b):
    """Hamilton product a*b of two geometry_msgs Quaternions (apply b, then a)."""
    return Quaternion(
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z)


# Named postures live in postures.py, which mirrors the measured registry in
# notes/08_poze.md. The old local ARM_CARRY was removed: nothing used it, and it
# was measured at 1.29 m wide, so it never was the "tucked" posture its comment
# claimed. ARM_CARRY_V2 is the narrow one (85.4 cm).

# Top-down grasp orientation for the end_effector_link. The Kinova/Robotiq tool
# frame has its approach axis along local +Z (palm -> fingertips) and the finger
# opening along local +X (measured from TF). A 180 deg rotation about X points the
# approach straight down (world -Z) with the fingers straddling the bar across X
# (the bar runs along Y, so the fingers close on its 0.06 m width).
GRASP_DOWN = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


class MainTask(BaseDriver, Node):
    def __init__(self):
        super().__init__('main_task_node')

        # --- tunable geometry (map frame unless noted) -----------------------
        # Pre-grasp: stop short of the pick table (table front edge at X~0.85, the
        # bar's near face at X~0.865). Kept back so the base does not jam the
        # table and Nav2 can reach the goal; the bar then sits at base_link X~0.55.
        # Transport stability probe (Phase 1): after a verified attach+lift,
        # skip the place and instead drive the base (straight + in-place turn)
        # to test whether the DART DetachableJoint survives base motion.
        self.declare_parameter('probe_transport', False)
        self.declare_parameter('pregrasp_xy', [0.35, 0.0])
        self.declare_parameter('pregrasp_yaw', 0.0)
        # Pre-place: just in front of the table (table front face at X~3.5).
        # Stop short of the table (front face at X=3.5) so the base footprint does
        # not jam against the table obstacle; the arms still reach onto it.
        self.declare_parameter('preplace_xy', [3.0, 0.0])
        self.declare_parameter('preplace_yaw', 0.0)
        # Centered, straight-on staging pose right before the 0.8 m doorway so the
        # long robot threads it square instead of approaching off-center/angled.
        self.declare_parameter('door_xy', [1.5, 0.0])
        self.declare_parameter('door_yaw', 0.0)
        # Bar grasp height (base_link). The bar rests on the pick table, centre
        # at ~0.25 m; calibrated against the grasp geometry telemetry.
        self.declare_parameter('box_grasp_z', 0.25)
        self.declare_parameter('table_place_z', 0.85)
        # Half-extent in Y at which each gripper grips the bar (bar is 0.30 m
        # long -> ends at ±0.15; grip just inside the ends at ±0.13).
        self.declare_parameter('grasp_half_width', 0.13)
        # Calibration gate: stop after the pick+lift to verify the friction grasp
        # in isolation before re-enabling transport/place.
        self.declare_parameter('pick_only', True)
        self.declare_parameter('navigate_region', False)
        self.declare_parameter('region_x', 0.0)
        self.declare_parameter('region_y', -4.5)
        self.declare_parameter('region_yaw', -1.57079632679)

        self.pregrasp_xy = self.get_parameter('pregrasp_xy').value
        self.pregrasp_yaw = self.get_parameter('pregrasp_yaw').value
        self.preplace_xy = self.get_parameter('preplace_xy').value
        self.preplace_yaw = self.get_parameter('preplace_yaw').value
        self.door_xy = self.get_parameter('door_xy').value
        self.door_yaw = self.get_parameter('door_yaw').value
        self.box_grasp_z = self.get_parameter('box_grasp_z').value
        self.table_place_z = self.get_parameter('table_place_z').value
        self.grasp_half_width = self.get_parameter('grasp_half_width').value

        # --- action clients --------------------------------------------------
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.navigate = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # Straight-line Cartesian moves (press/lift): a 10 cm press must be a
        # 10 cm motion - RRTConnect happily returns metre-long detours that
        # sweep the unmodelled box mid-path.
        self.cart_srv = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')
        self.ik_srv = self.create_client(GetPositionIK, '/compute_ik')
        self.exec_traj = ActionClient(self, ExecuteTrajectory,
                                      '/execute_trajectory')
        # Direct arm-controller clients: move_group executes ONE trajectory at
        # a time, but the squeeze needs BOTH arms pressing simultaneously.
        self.left_jtc = ActionClient(
            self, FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory')
        self.right_jtc = ActionClient(
            self, FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory')
        # Planning-scene diffs: MoveIt knows NOTHING about the world by
        # default, so RRT paths sweep the arms straight through the table (the
        # reaction shoved the whole base metres away, seen live in the GUI).
        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        self.left_grip = ActionClient(
            self, GripperCommand, '/left_gripper_controller/gripper_cmd')
        self.right_grip = ActionClient(
            self, GripperCommand, '/right_gripper_controller/gripper_cmd')
        self.torso = ActionClient(
            self, FollowJointTrajectory,
            '/torso_controller/follow_joint_trajectory')
        self.pan_tilt = ActionClient(
            self, FollowJointTrajectory,
            '/pan_tilt_controller/follow_joint_trajectory')

        # --- base velocity ---------------------------------------------------
        # Drive the base directly (no Nav2): the skid-steer base rotates by wheel
        # slip, which breaks wheel odometry and SLAM during an in-place turn, so
        # Nav2/SLAM navigation is unreliable. Instead we visual-servo to the
        # marker with direct cmd_vel. BaseDriver also subscribes to the wheel
        # odometry used to measure the ACTUAL close-in drive distance (the
        # open-loop profile has cm-level error, enough for the grippers to close
        # beside the box).
        self.init_base_drive()

        # --- perception ------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # The 640x480 point cloud is heavy; subscribing all the time starves the
        # /clock callback in the busy servo loops (sim time then never advances
        # and the loops hang). Subscribe only while measuring the box.
        self._last_cloud = None

        # Wrist->fingertip stand-off along the gripper approach axis, measured once
        # from TF after the arms reach a known posture (so end_effector goals place
        # the FINGER TIPS - not the wrist - on the bar). Nominal until measured.
        self.tip_standoff = 0.145

        # --- contact / force feedback ---------------------------------------
        # The arm joints expose an 'effort' state interface (Gen3 joint-torque
        # sensors); the gripper exposes position+velocity. We read both from
        # /joint_states: gripper position drives the grasp-stall signal (finger
        # stops short of the full stroke), and wrist effort is logged for context
        # (not a hard gate - a top-down grasp loads the wrist mostly with gravity).
        self._joint = {}            # name -> (position, effort)
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_cb, 10)
        # Optional Ignition fingertip contact sensors (bridged Contacts msgs). Each
        # entry holds the wall-clock time a non-empty contact was last seen;
        # _tip_box_contact_t only counts contacts whose other collider is the box.
        self._tip_contact_t = {}
        self._tip_box_contact_t = {}
        self.tip_contact_topics = {
            'left_left': '/contact/left_left_tip',
            'left_right': '/contact/left_right_tip',
            'right_left': '/contact/right_left_tip',
            'right_right': '/contact/right_right_tip',
        }
        if _Contacts is not None:
            for key, topic in self.tip_contact_topics.items():
                self.create_subscription(
                    _Contacts, topic,
                    lambda msg, k=key: self._contact_cb(k, msg), 10)
            self.get_logger().info('Fingertip contact sensors subscribed.')
        else:
            self.get_logger().warn(
                'ros_gz_interfaces/Contacts unavailable; contact-sensor signal '
                'disabled (depth + stall + effort still used).')

        self.get_logger().info('Main task node ready.')

    def _cloud_cb(self, msg):
        self._last_cloud = msg

    def _joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            eff = msg.effort[i] if i < len(msg.effort) else 0.0
            self._joint[name] = (pos, eff)

    def _contact_cb(self, key, msg):
        if msg.contacts:
            self._tip_contact_t[key] = time.monotonic()
            # Track WHAT was touched: only a contact whose other collider is
            # the target box may count as grasp evidence - a pad brushing the
            # table or the robot itself must not pass the gate.
            for c in msg.contacts:
                names = (getattr(c.collision1, 'name', '') + '|'
                         + getattr(c.collision2, 'name', ''))
                if 'aruco_box' in names:
                    self._tip_box_contact_t[key] = time.monotonic()
                    break

    def _tip_in_contact(self, key, max_age=0.4, box_only=False):
        """True if the named fingertip contact sensor fired within max_age s.
        With box_only=True only contacts AGAINST THE BOX count (a pad brushing
        the table or the robot itself is not grasp evidence)."""
        t = (self._tip_box_contact_t if box_only
             else self._tip_contact_t).get(key)
        return t is not None and (time.monotonic() - t) < max_age

    # ------------------------------------------------------------------ utils
    def _wait_server(self, client, name, timeout=30.0):
        self.get_logger().info(f'Waiting for action server: {name}')
        if not client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f'Action server {name} not available.')
            return False
        return True

    def _spin_until_done(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _send_and_wait(self, client, goal, name):
        """Send a goal, block for the result, return True on SUCCEEDED."""
        send_future = client.send_goal_async(goal)
        handle = self._spin_until_done(send_future)
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{name}: goal rejected.')
            return False
        result_future = handle.get_result_async()
        result = self._spin_until_done(result_future)
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f'{name}: {"OK" if ok else "FAILED"}')
        return ok

    def _attach_box(self, attach):
        """Rigidly attach / detach the box to the left wrist via the
        DetachableJoint topics (Ignition transport). Only called to ATTACH after
        the contact check confirms the grippers are genuinely on the box, so the
        rigid hold stands in for the friction DART cannot provide - not a fake."""
        topic = '/aruco_box/attach' if attach else '/aruco_box/detach'
        subprocess.run(
            ['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Empty',
             '-p', 'unused: true'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.get_logger().info(
            f'Box {"ATTACHED to" if attach else "DETACHED from"} left wrist.')

    # ------------------------------------------------------------ TF / reach
    def _tf_point(self, target, source):
        """Translation (np.array xyz) of source frame in target frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time())
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def measure_tip_standoff(self):
        """Measure the wrist->fingertip distance along the gripper approach axis
        from TF (end_effector_link -> finger_tip_link) so grasp goals can place the
        finger tips, not the wrist, on the bar. Updates self.tip_standoff."""
        vals = []
        for side in ('left', 'right'):
            for f in ('left', 'right'):
                try:
                    tf = self.tf_buffer.lookup_transform(
                        f'{side}_end_effector_link',
                        f'{side}_robotiq_85_{f}_finger_tip_link',
                        rclpy.time.Time())
                    t = tf.transform.translation
                    vals.append(math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z))
                except Exception:
                    continue
        if vals:
            self.tip_standoff = sum(vals) / len(vals)
            self.get_logger().info(
                f'Measured wrist->fingertip stand-off = {self.tip_standoff:.3f} m '
                f'({len(vals)} tips).')
        else:
            self.get_logger().warn(
                f'Could not measure fingertip stand-off; using nominal '
                f'{self.tip_standoff:.3f} m.')
        return self.tip_standoff

    def _linear_traj(self, group, ee, pose, label, min_frac=0.7):
        """Timed straight-line RobotTrajectory of `ee` to `pose` (base_link)
        via MoveIt's compute_cartesian_path, or None. avoid_collisions=False:
        press goals deliberately touch the box."""
        if not self.cart_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(f'{label}: cartesian service unavailable.')
            return None
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.start_state.is_diff = True
        req.group_name = group
        req.link_name = ee
        req.waypoints = [pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        fut = self.cart_srv.call_async(req)
        self._spin_until_done(fut)
        res = fut.result()
        if res is None or res.error_code.val != 1 or res.fraction < 0.3:
            frac = -1.0 if res is None else res.fraction
            self.get_logger().warn(
                f'{label}: cartesian path failed (fraction {frac:.2f}).')
            return None
        if res.fraction < min_frac:
            # A partial straight segment is still useful: the caller's
            # verify/contact/retry loop iterates from wherever it ends.
            self.get_logger().warn(
                f'{label}: partial linear path (fraction {res.fraction:.2f}).')
        else:
            self.get_logger().info(
                f'{label}: linear path fraction {res.fraction:.2f}.')
        traj = res.solution.joint_trajectory
        # Unwrap continuous joints onto the branch NEAREST the current state:
        # IK returns angles in [-pi,pi] while the physical Kinova joints
        # accumulate turns (seen live: state 3.28 vs IK -3.00 - the same
        # angle). Executed raw, the JTC unwinds the joint a full 2*pi
        # mid-grasp (the 'weird slow rotations' in the GUI) or flings the arm.
        for j, name in enumerate(traj.joint_names):
            cur = self._joint.get(name)
            if cur is None:
                continue
            ref = cur[0]
            for pt in traj.points:
                val = float(pt.positions[j])
                while val - ref > math.pi:
                    val -= 2.0 * math.pi
                while ref - val > math.pi:
                    val += 2.0 * math.pi
                pt.positions[j] = val
                ref = val
        # compute_cartesian_path returns an UNTIMED path (time_from_start all
        # zero) - executing it as-is makes the controller lurch anywhere (seen
        # live: a '0.89-fraction straight line' ended 0.4 m from goal). Apply a
        # simple constant-velocity time parameterization and let the JTC
        # interpolate positions.
        t = 0.0
        prev = None
        for pt in traj.points:
            if prev is None:
                t = 0.2
            else:
                delta = max(abs(a - b) for a, b in zip(pt.positions, prev))
                t += max(0.1, delta / 0.4)      # <= ~0.4 rad/s per joint
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            pt.velocities = []
            pt.accelerations = []
            prev = list(pt.positions)
        return res.solution, res.fraction

    def move_linear(self, group, ee, pose, label, min_frac=0.7):
        """Straight-line Cartesian move (single arm), executed via move_group.
        Returns True on execution."""
        got = self._linear_traj(group, ee, pose, label, min_frac=min_frac)
        if got is None:
            return False
        sol, _ = got
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = sol
        if not self._wait_server(self.exec_traj, 'execute_trajectory'):
            return False
        return self._send_and_wait(self.exec_traj, goal, label)

    def _ik(self, group, ee, pose, seed=None, avoid_collisions=False):
        """Joint solution {name: position} for `ee` at `pose`, or None. With
        `seed` (a prior solution dict) the solver starts on that BRANCH, so
        pre-pose and press-pose solutions stay a small joint motion apart."""
        if not self.ik_srv.wait_for_service(timeout_sec=10.0):
            return None
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = group
        r.ik_link_name = ee
        r.pose_stamped.header.frame_id = 'base_link'
        r.pose_stamped.pose = pose
        r.avoid_collisions = avoid_collisions
        r.timeout.sec = 3
        if seed is None:
            r.robot_state.is_diff = True
        else:
            r.robot_state.is_diff = True
            r.robot_state.joint_state.name = list(seed.keys())
            r.robot_state.joint_state.position = [float(v)
                                                  for v in seed.values()]
        fut = self.ik_srv.call_async(req)
        self._spin_until_done(fut)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        arm_prefix = group.split('_')[0]
        return {n: p for n, p in zip(res.solution.joint_state.name,
                                     res.solution.joint_state.position)
                if n.startswith(f'{arm_prefix}_joint_')}

    def move_arm_joints(self, group, joints, label):
        """Joint-space plan+execute of ONE arm to explicit joint targets."""
        if not self._wait_server(self.move, 'move_action'):
            return False
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        c = Constraints(name=label)
        for name, val in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        self.get_logger().info(f'{label}: planning {group} (joint-space)')
        return self._send_and_wait(self.move, goal, label)

    @staticmethod
    def _roll180(pose):
        """The same pose with the tool rolled 180 deg about its approach axis.
        The fingertip pads are symmetric, so the press is identical - but the
        IK lands in a different wrist branch (the left arm's straight press
        line is chronically infeasible in the primary branch)."""
        p = Pose()
        p.position = pose.position
        p.orientation = quat_mul(pose.orientation,
                                 Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
        return p

    def _traj_starts_here(self, traj, label, tol=0.15):
        """True if the trajectory's FIRST point matches the arm's CURRENT
        joint positions (within tol rad per joint). A mismatch means the path
        was computed from a stale state or a flipped IK branch - executing it
        makes the controller lurch the arm across the workspace."""
        if not traj.points:
            return False
        for name, target in zip(traj.joint_names, traj.points[0].positions):
            cur = self._joint.get(name)
            if cur is None:
                continue
            if abs(cur[0] - float(target)) > tol:
                self.get_logger().warn(
                    f'{label}: trajectory start differs from the actual state '
                    f'({name}: {cur[0]:.2f} vs {float(target):.2f}) - '
                    'discarding.')
                return False
        return True

    def press_both_linear(self, lp, rp, pre_l, pre_r):
        """Compute straight press lines for BOTH arms, then execute them
        SIMULTANEOUSLY by sending the trajectories directly to the two arm
        controllers. A lone press bulldozes the 1 kg cube across the table
        (~0.3 m, seen live) - only opposed simultaneous presses balance out.
        Per arm: if the line is IK-infeasible from the current configuration,
        re-roll the pre-squeeze pose (new RRT configuration) and retry."""
        trajs = {}
        for side, group, ee, pre_pose, press_pose in (
                ('left', 'left_arm', 'left_end_effector_link', pre_l, lp),
                ('right', 'right_arm', 'right_end_effector_link', pre_r, rp)):
            # Try the primary roll and the 180deg-rolled variant (identical
            # press, different IK branch) from each pre-squeeze configuration.
            # Whether a CLEAN straight line exists is a lottery over the arm
            # configuration the pose-goal RRT lands in (fractions swing
            # 0.0-1.0 run to run), so keep RE-ROLLING the pre-squeeze until a
            # configuration with fraction >= 0.9 shows up; only the final
            # attempt settles for a partial line.
            variants = [press_pose, self._roll180(press_pose)]
            attempts = 4
            for attempt in range(attempts):
                # The line must start from where the arm ACTUALLY is: computing
                # it while the arm still creeps after the previous motion gave
                # a trajectory from a stale state (possibly a different IK
                # branch) - the JTC then lurches the arm a metre away.
                self._wait_settle(ee)
                got = self._linear_traj(group, ee, variants[attempt % 2],
                                        f'STEP5 press {side}')
                if got is not None:
                    sol, frac = got
                    if self._traj_starts_here(sol.joint_trajectory,
                                              f'STEP5 press {side}') and (
                            frac >= 0.9 or attempt == attempts - 1):
                        trajs[side] = sol.joint_trajectory
                        break
                if attempt % 2 == 1 and attempt < attempts - 1:
                    self.plan_arm(group, ee, pre_pose, 'base_link',
                                  f'STEP5 re-pre-squeeze {side}', ori_tol=0.4)
        outs = {'left': False, 'right': False}
        # A side that never got a usable line is brought NEAR the face by an
        # ordinary (collision-checked) RRT pose goal instead - a target ~2 cm
        # off the face is always plannable, and the per-arm linear re-press
        # afterwards converges reliably from there (measured dl=0.001-0.005 m
        # in consecutive runs). The opposite pad bounds any cube shove.
        for side, group, ee, pre_pose, press_pose in (
                ('left', 'left_arm', 'left_end_effector_link', pre_l, lp),
                ('right', 'right_arm', 'right_end_effector_link', pre_r, rp)):
            if side in trajs:
                continue
            near = Pose()
            near.position = Point(
                x=press_pose.position.x
                + 0.4 * (pre_pose.position.x - press_pose.position.x),
                y=press_pose.position.y
                + 0.4 * (pre_pose.position.y - press_pose.position.y),
                z=press_pose.position.z)
            near.orientation = press_pose.orientation
            outs[side] = self.plan_arm(
                group, ee, near, 'base_link',
                f'STEP5 press {side} (RRT near-face)', ori_tol=0.3)
        if trajs:
            clients = {'left': self.left_jtc, 'right': self.right_jtc}
            goals = {}
            for side, traj in trajs.items():
                if not self._wait_server(clients[side], f'{side} JTC'):
                    return False, False
                g = FollowJointTrajectory.Goal()
                g.trajectory = traj
                goals[side] = clients[side].send_goal_async(g)
            handles = {}
            for side, fut in goals.items():
                self._spin_until_done(fut)
                h = fut.result()
                handles[side] = h if (h is not None and h.accepted) else None
            for side, h in handles.items():
                if h is None:
                    continue
                rf = h.get_result_async()
                self._spin_until_done(rf)
                r = rf.result()
                outs[side] = (r is not None
                              and r.status == GoalStatus.STATUS_SUCCEEDED)
                self.get_logger().info(
                    f'STEP5 press {side} (simultaneous): '
                    f'{"OK" if outs[side] else "FAILED"}')
        return outs['left'], outs['right']

    def publish_collision_scene(self, center, table_top_z):
        """Add the floor, the pick table and the target cube to the MoveIt
        planning scene (base_link frame). Without them RRT freely plans arm
        sweeps THROUGH the table - the physical snag then shoves the whole
        base. The deliberate press still touches the cube because it runs via
        compute_cartesian_path with avoid_collisions=False."""
        def box_object(name, cx, cy, cz, sx, sy, sz):
            co = CollisionObject()
            co.header.frame_id = 'base_link'
            co.id = name
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [float(sx), float(sy), float(sz)]
            pose = Pose()
            pose.position = Point(x=float(cx), y=float(cy), z=float(cz))
            pose.orientation.w = 1.0
            co.primitives = [prim]
            co.primitive_poses = [pose]
            co.operation = CollisionObject.ADD
            return co

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            box_object('floor', 0.7, 0.0, -0.10, 4.0, 4.0, 0.02),
            box_object('pick_table', center.x, center.y,
                       table_top_z - 0.05, 0.55, 0.65, 0.10),
            box_object('target_cube', center.x, center.y,
                       table_top_z + 0.15, 0.30, 0.30, 0.30),
        ]
        self.scene_pub.publish(scene)
        # Give the move_group monitor a moment to apply the diff.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(
            'Planning scene: floor + pick_table + cube published.')

    def _wait_settle(self, link, timeout=8.0):
        """Block until `link` stops moving (<4 mm over 0.4 s) or timeout. The
        JTC reports SUCCEEDED when the trajectory CLOCK ends, but the simulated
        arm lags and keeps creeping toward the last point for seconds (seen
        live: EE error improved 0.20->0.05 m AFTER the action result) - any
        readback verification must wait for the mechanism to settle first."""
        deadline = time.monotonic() + timeout
        prev = None
        while time.monotonic() < deadline:
            cur = self._tf_point('base_link', link)
            if cur is not None and prev is not None:
                d = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
                if d < 0.004 and abs(cur[2] - prev[2]) < 0.004:
                    return
            prev = cur
            end = time.monotonic() + 0.4
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)

    def _ee_pose(self, link):
        """Current full pose of `link` in base_link from TF, or None."""
        try:
            tf = self.tf_buffer.lookup_transform('base_link', link,
                                                 rclpy.time.Time())
        except Exception:
            return None
        p = Pose()
        p.position = Point(x=tf.transform.translation.x,
                           y=tf.transform.translation.y,
                           z=tf.transform.translation.z)
        p.orientation = tf.transform.rotation
        return p

    def retreat_linear(self, group, ee, v, dist, label):
        """Retreat straight by dist along the XY direction v from the CURRENT
        pose, KEEPING the current orientation. After a press the wrist has
        deflected from the commanded orientation, so a waypoint built from the
        ideal pre-pose is IK-infeasible from here (seen live: fraction 0.00);
        a pure translation of the actual pose always has a nearby solution."""
        cur = self._ee_pose(ee)
        if cur is None:
            return False
        tgt = Pose()
        tgt.position = Point(x=cur.position.x + dist * float(v[0]),
                             y=cur.position.y + dist * float(v[1]),
                             z=cur.position.z)
        tgt.orientation = cur.orientation
        return self.move_linear(group, ee, tgt, label, min_frac=0.5)

    def verify_reached(self, link, pose, tol=0.04, label=''):
        """Read the ACTUAL end-effector pose from TF after a motion and compare it
        to the commanded pose. Independent of perception, so it catches the failure
        where the arm could not reach and stayed short (the '50 cm in front' bug).
        Returns True only if the wrist is within tol of the goal position."""
        actual = self._tf_point('base_link', link)
        if actual is None:
            self.get_logger().warn(f'{label}: no TF for {link}; cannot verify.')
            return False
        tgt = np.array([pose.position.x, pose.position.y, pose.position.z])
        d = float(np.linalg.norm(actual - tgt))
        ok = d <= tol
        self.get_logger().info(
            f'{label}: EE at ({actual[0]:.3f}, {actual[1]:.3f}, {actual[2]:.3f}) '
            f'vs goal Δ={d:.3f} m -> {"REACHED" if ok else "OFF TARGET"}.')
        return ok

    # ----------------------------------------------------------------- moveit
    def _pose_goal_constraint(self, link, frame, pose, pos_tol=0.03, ang_tol=0.2):
        c = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = link
        pc.target_point_offset.x = 0.0
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])
        bv = BoundingVolume()
        bv.primitives.append(region)
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = link
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = ang_tol
        oc.absolute_y_axis_tolerance = ang_tol
        oc.absolute_z_axis_tolerance = ang_tol
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def plan_arm(self, group, link, pose, frame, label, ori_tol=3.14, retries=4):
        """Plan a single arm to an end-effector pose. Orientation tolerance
        defaults to wide-open (3.14 rad) so this is effectively a position goal:
        the box is symmetric, so any wrist orientation that reaches the side
        face is acceptable, and a loose goal region is what lets the IK sampler
        actually find states (a tight dual-arm pose goal does not).

        RRTConnect is randomized, so an individual attempt sometimes returns a
        path that fails the post-plan collision check ('invalid states'); retry a
        few times since a later sample usually yields a collision-free path."""
        if not self._wait_server(self.move, 'move_action'):
            return False
        for attempt in range(1, retries + 1):
            goal = MoveGroup.Goal()
            req = goal.request
            req.group_name = group
            req.num_planning_attempts = 20
            req.allowed_planning_time = 8.0
            req.max_velocity_scaling_factor = 0.2
            req.max_acceleration_scaling_factor = 0.2
            req.goal_constraints.append(
                self._pose_goal_constraint(link, frame, pose,
                                           pos_tol=0.05, ang_tol=ori_tol))
            goal.planning_options.plan_only = False
            self.get_logger().info(
                f'{label}: planning {group} -> '
                f'({pose.position.x:.2f}, {pose.position.y:.2f}, '
                f'{pose.position.z:.2f}) [attempt {attempt}/{retries}]')
            if self._send_and_wait(self.move, goal, label):
                return True
        return False

    def plan_both_arms(self, left_pose, right_pose, frame, label):
        """Plan both arms simultaneously to the given end-effector poses."""
        if not self._wait_server(self.move, 'move_action'):
            return False
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'both_arms'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.goal_constraints.append(Constraints(
            name=label,
            position_constraints=(
                self._pose_goal_constraint('left_end_effector_link', frame,
                                           left_pose).position_constraints
                + self._pose_goal_constraint('right_end_effector_link', frame,
                                             right_pose).position_constraints),
            orientation_constraints=(
                self._pose_goal_constraint('left_end_effector_link', frame,
                                           left_pose).orientation_constraints
                + self._pose_goal_constraint('right_end_effector_link', frame,
                                             right_pose).orientation_constraints),
        ))
        goal.planning_options.plan_only = False
        self.get_logger().info(f'{label}: planning both_arms')
        return self._send_and_wait(self.move, goal, label)

    def move_arms_joint(self, posture, label):
        """Joint-space plan of both arms to a named posture (see postures.py).
        Joint goals are always IK-solvable, so this reliably exercises MoveIt2
        planning + trajectory execution on the real controllers."""
        if not self._wait_server(self.move, 'move_action'):
            return False
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'both_arms'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.goal_constraints.append(joint_constraints(posture, label))
        goal.planning_options.plan_only = False
        self.get_logger().info(f'{label}: planning both_arms (joint-space)')
        return self._send_and_wait(self.move, goal, label)

    # --------------------------------------------------------------- pan-tilt
    def look_down(self, pitch, label):
        """Tilt the camera down so the Aruco box enters the field of view."""
        if not self._wait_server(self.pan_tilt, 'pan_tilt_controller'):
            return False
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['pan_tilt_yaw_joint', 'pan_tilt_pitch_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, float(pitch)]
        pt.time_from_start.sec = 2
        traj.points.append(pt)
        goal.trajectory = traj
        self.get_logger().info(f'{label}: tilt camera to pitch={pitch:.2f}')
        return self._send_and_wait(self.pan_tilt, goal, label)

    # ---------------------------------------------------------------- gripper
    def set_gripper(self, client, position, label, max_effort=50.0):
        if not self._wait_server(client, label):
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        return self._send_and_wait(client, goal, label)

    # ------------------------------------------------------------------ torso
    def move_torso(self, height, label, secs=4):
        if not self._wait_server(self.torso, 'torso_controller'):
            return False
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['torso_left_carriage_joint',
                            'torso_right_carriage_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [float(height), float(height)]
        pt.time_from_start.sec = int(secs)
        traj.points.append(pt)
        goal.trajectory = traj
        self.get_logger().info(f'{label}: torso -> {height} m over {secs}s')
        return self._send_and_wait(self.torso, goal, label)

    # ------------------------------------------------ grasp-geometry telemetry
    def _log_grasp_geometry(self, box, when):
        """Log the four finger-tip positions in base_link and their Y gap to the
        box side faces, so the squeeze depth can be calibrated empirically (the
        end_effector_link is at the wrist, ~0.15 m short of the tips, so the
        true contact point must be read from TF, not assumed)."""
        tips = [
            'left_robotiq_85_left_finger_tip_link',
            'left_robotiq_85_right_finger_tip_link',
            'right_robotiq_85_left_finger_tip_link',
            'right_robotiq_85_right_finger_tip_link',
        ]
        self.get_logger().info(f'--- grasp geometry [{when}] '
                               f'box centre y={box.y:.3f}, faces at '
                               f'y=±{0.15 + box.y:.3f}/{box.y - 0.15:.3f} ---')
        for tip in tips:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', tip, rclpy.time.Time())
                t = tf.transform.translation
                gap = abs(t.y - box.y) - 0.15  # +ve: gap to face, -ve: pressing
                self.get_logger().info(
                    f'  {tip}: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) '
                    f'y-gap to face = {gap:+.3f} m')
            except Exception:
                self.get_logger().warn(f'  {tip}: TF unavailable')

    # ---------------------------------------------------------------- look-up
    def confirm_box(self, timeout=10.0, samples=5):
        """Return the box-centre Point in base_link from the Aruco marker TF,
        or None if the marker is not seen within timeout.

        The marker is painted on the box face nearest the robot, so the centre
        of the 0.3 m cube is half a box depth (0.15 m) further along +X (into
        the box, away from the robot). A handful of TF samples are averaged to
        smooth out per-frame solvePnP jitter."""
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        xs, ys, zs = [], [], []
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', 'aruco_marker_frame',
                    rclpy.time.Time())
                # Reject stale detections: a leftover TF from the scan would give
                # a garbage pose now that the robot has moved.
                age = (self.get_clock().now().nanoseconds * 1e-9
                       - (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9))
                if age > 0.5:
                    raise RuntimeError('stale marker')
                t = tf.transform.translation
                xs.append(t.x)
                ys.append(t.y)
                zs.append(t.z)
                if len(xs) >= samples:
                    break
            except Exception:
                pass
            rclpy.spin_once(self, timeout_sec=0.2)
        if not xs:
            self.get_logger().warn(
                'Aruco marker not seen; using nominal box pose.')
            return None
        n = len(xs)
        # Marker face -> cube centre: half the assignment cube (0.30 m) inward.
        center = Point(x=sum(xs) / n + 0.15, y=sum(ys) / n, z=sum(zs) / n)
        self.get_logger().info(
            f'Aruco confirmed ({n} samples); box centre base_link '
            f'({center.x:.2f}, {center.y:.2f}, {center.z:.2f}).')
        return center

    def measure_box(self, seed, timeout=8.0):
        """Measure the box from the depth point cloud: returns (centre Point,
        length, height, u) in base_link, or None - where u is the bar's long-axis
        unit XY direction from PCA. The camera sees the front face + top, so it
        measures length and height directly and the long-axis orientation; the
        gripped thickness (occluded back face) is taken as the known 0.06 m. 'seed'
        is the approximate box centre (from the marker) used to isolate the box
        cluster from the table/background."""
        self._last_cloud = None
        sub = self.create_subscription(PointCloud2, '/camera/points',
                                       self._cloud_cb, 1)
        try:
            # Wait for a FRESH cloud. spin_once() returns as soon as it services
            # ANY callback, and this node has high-rate subscriptions (/clock,
            # TF, contacts), so a fixed iteration count burns through in well
            # under a second - the 6 Hz cloud never gets its turn. Bound the
            # wait by a real wall-clock deadline instead.
            deadline = time.monotonic() + timeout
            while self._last_cloud is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(sub)
        if self._last_cloud is None:
            self.get_logger().warn('No point cloud; cannot measure box.')
            return None
        msg = self._last_cloud
        pts = pointcloud2_xyz(msg)
        # The Fortress rgbd_camera stamps every output with ignition_frame_id
        # (the optical frame, which Aruco needs for the image), but the point
        # cloud DATA is in the sensor BODY frame (x forward, z up) - verified
        # empirically: transformed via the optical frame the whole cloud lands
        # ~90 deg off to the side. Transform via the mounting link instead.
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'camera_link', rclpy.time.Time())
        except Exception:
            self.get_logger().warn('No TF for cloud; cannot measure box.')
            return None
        tr = tf.transform.translation
        P = pts @ quat_to_matrix(tf.transform.rotation).T \
            + np.array([tr.x, tr.y, tr.z])
        # Keep points above the table, in front, near the marker seed. The lower
        # x bound is 0.35, NOT 0.2: after the close-in drive the tilted camera
        # also sees the robot's own front (torso/base structures at x~0.25-0.35),
        # and once those points merge with the box the PCA axis/length are junk
        # (seen live: length 0.63 for the 0.30 bar, grippers closed on air).
        # The marker sits on the box face, so nothing of the box can be higher
        # than the face centre + ~box height; points above that are arms/other.
        m = ((P[:, 2] > 0.12) & (P[:, 2] < min(0.50, seed.z + 0.20))
             & (P[:, 0] > 0.35) & (P[:, 0] < 1.0)
             & (np.abs(P[:, 0] - seed.x) < 0.25)
             & (np.abs(P[:, 1] - seed.y) < 0.30))
        B = P[m]
        if len(B) < 40:
            self.get_logger().warn(
                f'Too few box points ({len(B)}); cannot measure box.')
            return None
        zmin, zmax = float(B[:, 2].min()), float(B[:, 2].max())
        height = zmax - zmin
        # PCA on the XY footprint: the dominant eigenvector is the bar's long axis
        # (its 0.30 m length), so the grasp aligns to the real bar instead of
        # assuming it lies along base_link Y (the bar is yawed in the world).
        xy = B[:, :2]
        mean = xy.mean(axis=0)
        cov = np.cov((xy - mean).T)
        w, V = np.linalg.eigh(cov)
        u = V[:, int(np.argmax(w))]
        u = u / (np.linalg.norm(u) + 1e-9)
        if u[1] < 0:                       # consistent sign (point +Y-ish)
            u = -u
        p = np.array([-u[1], u[0]])        # perpendicular = thickness axis
        if p[0] < 0:                       # point away from the robot (+X)
            p = -p
        proj_u = (xy - mean) @ u
        proj_p = (xy - mean) @ p
        length = float(proj_u.max() - proj_u.min())
        u_mid = float(proj_u.max() + proj_u.min()) / 2.0
        # Camera sees only the front face along the thickness axis; the centre is
        # half the assignment cube (0.30 m) inward from the nearest face.
        p_center = float(proj_p.min()) + 0.15
        cxy = mean + u * u_mid + p * p_center
        center = Point(x=float(cxy[0]), y=float(cxy[1]), z=(zmin + zmax) / 2.0)
        yaw = math.atan2(float(u[1]), float(u[0]))
        self.get_logger().info(
            f'Box measured from depth ({len(B)} pts): length={length:.3f} '
            f'height={height:.3f} long-axis yaw={yaw:.2f} rad '
            f'u=({u[0]:.2f},{u[1]:.2f}) centre=({center.x:.2f}, {center.y:.2f}, '
            f'{center.z:.2f}).')
        # The target's dimensions are given by the assignment, so a cluster that
        # does not measure like the box IS NOT the box (stray geometry merged
        # in) - reject it rather than send the grippers to a junk pose.
        if not (0.15 < length < 0.45) or not (0.06 < height < 0.32):
            self.get_logger().warn(
                f'Measured extent {length:.2f}x{height:.2f} m does not match '
                'the target box; rejecting this measurement.')
            return None
        return center, length, height, (float(u[0]), float(u[1]))

    # --------------------------------------------------------------- sequence
    def grasp_poses(self, center, u=None, half_width=None, z_offset=None):
        """Left/right end-effector poses (base_link) for a top-down grasp of the
        two bar ends, aligned to the bar's long axis u (unit XY direction).

        Each wrist sits above one end along u; the gripper is yawed so its
        finger-opening axis crosses the bar's thickness (perpendicular to u), and
        points straight down. z_offset defaults to the measured wrist->fingertip
        stand-off so the FINGER TIPS land at the bar; pass a larger z_offset for a
        higher pre-grasp stand-off. For u=+Y this reduces to the plain top-down
        GRASP_DOWN pose."""
        half = self.grasp_half_width if half_width is None else half_width
        if z_offset is None:
            z_offset = self.tip_standoff
        if u is None:
            u = (0.0, 1.0)
        ux, uy = float(u[0]), float(u[1])
        gripper_yaw = math.atan2(uy, ux) - math.pi / 2.0
        ori = quat_mul(yaw_to_quat(gripper_yaw), GRASP_DOWN)
        left = Pose(
            position=Point(x=center.x + half * ux, y=center.y + half * uy,
                           z=center.z + z_offset),
            orientation=ori)
        right = Pose(
            position=Point(x=center.x - half * ux, y=center.y - half * uy,
                           z=center.z + z_offset),
            orientation=ori)
        return left, right

    def marker_tangent(self):
        """Horizontal unit tangent of the marker face in base_link XY, or None.
        For the assignment cube this IS the squeeze axis: the marker face's
        square footprint makes depth-PCA yaw degenerate, but the marker's
        solvePnP orientation is exact, and the two faces ADJACENT to the marker
        are the ones the arms press."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'aruco_marker_frame', rclpy.time.Time())
        except Exception:
            return None
        R = quat_to_matrix(tf.transform.rotation)
        n = R[:, 2]                              # marker normal (out of the face)
        v = np.array([-n[1], n[0]])              # horizontal, perpendicular to n
        norm = float(np.linalg.norm(v))
        if norm < 1e-6:
            return None
        v = v / norm
        if v[1] < 0:                             # consistent sign (+Y-ish)
            v = -v
        return (float(v[0]), float(v[1]))

    def squeeze_poses(self, center, v, pre=0.0):
        """Left/right EE poses for the dual-arm SQUEEZE of the assignment cube:
        each CLOSED gripper (fingertip pads) approaches horizontally along -/+v
        and presses one of the two opposite side faces. pre>0 hovers off the
        face; pre<0 commands a slight interference so the pads genuinely press.
        Approach axis is tool +Z (same convention as the top-down grasp, where
        the tips extend along +Z from the wrist); tool +X is kept horizontal so
        the two pads contact side by side (stable against yaw torque)."""
        off = 0.15 + self.tip_standoff + pre
        vv = np.array([float(v[0]), float(v[1]), 0.0])
        poses = []
        for sgn in (+1.0, -1.0):        # left presses from +v, right from -v
            z_ax = -sgn * vv                             # approach: into the box
            x_ax = np.array([-z_ax[1], z_ax[0], 0.0])    # horizontal, ⟂ approach
            x_ax = x_ax / np.linalg.norm(x_ax)
            y_ax = np.cross(z_ax, x_ax)
            R = np.column_stack([x_ax, y_ax, z_ax])
            poses.append(Pose(
                position=Point(x=center.x + sgn * off * vv[0],
                               y=center.y + sgn * off * vv[1],
                               z=center.z),
                orientation=quat_from_matrix(R)))
        return poses[0], poses[1]

    # ------------------------------------------------ marker-relative servoing
    def aim_camera(self, pan, pitch, label):
        """Point the pan-tilt camera (pan + pitch)."""
        if not self._wait_server(self.pan_tilt, 'pan_tilt_controller'):
            return False
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['pan_tilt_yaw_joint', 'pan_tilt_pitch_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [float(pan), float(pitch)]
        pt.time_from_start.sec = 2
        traj.points.append(pt)
        goal.trajectory = traj
        self.get_logger().info(f'{label}: camera pan={pan:.2f} pitch={pitch:.2f}')
        return self._send_and_wait(self.pan_tilt, goal, label)

    def marker_in_base(self, max_age=0.5):
        """(x, y) of the marker in base_link from a FRESH detection, or None.
        Independent of camera pan/tilt and of SLAM (pure robot TF + detector), so
        it is reliable feedback for visual servoing."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'aruco_marker_frame', rclpy.time.Time())
        except Exception:
            return None
        age = (self.get_clock().now().nanoseconds * 1e-9
               - (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9))
        if age > max_age:
            return None
        t = tf.transform.translation
        return t.x, t.y

    def scan_for_marker(self):
        """Find the marker by PANNING the camera across its yaw range while the
        base stays still (so the skid-steer turn does not disturb odom/SLAM).
        Returns (x, y, pan) - marker in base_link and the camera pan it was found
        at (so the approach can keep the camera on it) - or None."""
        for pan in [0.0, -0.5, -1.0, 0.5, 1.0]:
            self.aim_camera(pan, 0.7, 'SCAN pan')
            hits = []
            for _ in range(20):  # ~2 s wall, iteration-bounded (clock-safe)
                if not rclpy.ok():
                    break
                m = self.marker_in_base(max_age=0.4)
                if m is not None:
                    hits.append(m)
                    if len(hits) >= 3:
                        mx = sum(h[0] for h in hits) / len(hits)
                        my = sum(h[1] for h in hits) / len(hits)
                        self.get_logger().info(
                            f'SCAN: marker found at base_link ({mx:.2f}, '
                            f'{my:.2f}) [pan={pan:.1f}].')
                        return mx, my, pan
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn('SCAN: marker not found by panning.')
        return None

    def visual_approach(self, scan_pan=0.0, target_x=0.55, timeout=60.0):
        """Drive to the marker by visual servoing on its base_link position (no
        Nav2/SLAM). Two phases, both closed-loop on the marker (robust to
        skid-steer slip): (1) turn the base to face the marker while the camera
        stays at the pan it was found at (so it stays in view); (2) recentre the
        camera and creep forward to target_x. Pitch 0.7 keeps the low box in the
        field of view across the whole distance. Returns True on success."""
        # Phase 1: turn to face the marker (camera held at scan_pan).
        self.aim_camera(scan_pan, 0.7, 'APPROACH hold camera')
        if not self._servo_turn(timeout):
            return False
        # Phase 2: camera forward, creep in.
        self.aim_camera(0.0, 0.7, 'APPROACH camera forward')
        return self._servo_creep(target_x, timeout)

    def _servo_turn(self, timeout):
        """Turn the base in place until the marker is dead ahead."""
        end = time.monotonic() + timeout
        last_seen = time.monotonic()
        while rclpy.ok() and time.monotonic() < end:
            m = self.marker_in_base(max_age=0.6)
            if m is None:
                self._send_vel(0.0, 0.0)
                if time.monotonic() - last_seen > 6.0:
                    self.get_logger().warn('Approach: lost marker (turn).')
                    return False
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            last_seen = time.monotonic()
            bearing = math.atan2(m[1], m[0])
            if abs(bearing) < 0.08:
                self._send_vel(0.0, 0.0)
                return True
            self._send_vel(0.0, max(-0.4, min(0.4, 1.0 * bearing)))
            rclpy.spin_once(self, timeout_sec=0.05)
        self._send_vel(0.0, 0.0)
        return False

    def _servo_creep(self, target_x, timeout):
        """Creep to the marker: turn in place when off-bearing, else drive
        straight (never both - with mu2=0 the skid-steer base crabs if it does)."""
        end = time.monotonic() + timeout
        last_seen = time.monotonic()
        while rclpy.ok() and time.monotonic() < end:
            m = self.marker_in_base(max_age=0.6)
            if m is None:
                self._send_vel(0.0, 0.0)
                if time.monotonic() - last_seen > 6.0:
                    self.get_logger().warn('Approach: lost marker (creep).')
                    return False
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            last_seen = time.monotonic()
            x, y = m
            bearing = math.atan2(y, x)
            dist = math.hypot(x, y)
            if dist <= target_x + 0.06 and abs(bearing) < 0.18:
                self._send_vel(0.0, 0.0)
                self.get_logger().info(
                    f'Approach done: marker at base_link ({x:.2f}, {y:.2f}).')
                return True
            # Hysteresis: only turn if badly off (>0.22); otherwise drive straight
            # even with a little bearing error, so it does not oscillate forever
            # turning <-> driving near the target.
            if abs(bearing) > 0.22:
                self._send_vel(0.0, 0.3 if bearing > 0 else -0.3)
            elif dist > target_x:
                self._send_vel(max(0.06, min(0.15, 0.6 * (dist - target_x))), 0.0)
            else:
                self._send_vel(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._send_vel(0.0, 0.0)
        self.get_logger().warn('Approach: timed out.')
        return False

    _TIP_LINKS = {
        'left': ['left_robotiq_85_left_finger_tip_link',
                 'left_robotiq_85_right_finger_tip_link'],
        'right': ['right_robotiq_85_left_finger_tip_link',
                  'right_robotiq_85_right_finger_tip_link'],
    }

    def fingertips_on_box(self, center, u, half_len,
                          half_thick=0.05, half_h=0.11):
        """Geometric check, INDEPENDENT of the commanded grasp: are both grippers'
        finger tips inside the box volume? center/u come from a FRESH depth (or
        marker) reading, not from the pose the arms were told to reach, so a
        perception or reach error cannot pass it (this is what makes it honest -
        the old check compared tips against the very point they were commanded to).
        Returns (ok, {side: bool})."""
        uv = np.array([u[0], u[1]])
        pv = np.array([-u[1], u[0]])       # thickness axis
        got = {'left': False, 'right': False}
        for side, links in self._TIP_LINKS.items():
            for link in links:
                t = self._tf_point('base_link', link)
                if t is None:
                    continue
                d = np.array([t[0] - center.x, t[1] - center.y])
                along = abs(float(d @ uv))
                across = abs(float(d @ pv))
                dz = abs(float(t[2] - center.z))
                if (along < half_len + 0.04 and across < half_thick + 0.04
                        and dz < half_h):
                    got[side] = True
        ok = got['left'] and got['right']
        self.get_logger().info(
            f'Fingertips-in-box (independent): left={got["left"]} '
            f'right={got["right"]} -> {"ON BOX" if ok else "NOT on box"}.')
        return ok, got

    def _knuckle_pos(self, side):
        """Gripper knuckle joint position (0=open .. 0.8=closed), or None."""
        v = self._joint.get(f'{side}_robotiq_85_left_knuckle_joint')
        return None if v is None else v[0]

    def close_until_contact(self, side, client, target=0.7, max_effort=20.0,
                            settle=2.5):
        """Command the gripper closed and watch for a genuine stop signal while it
        moves: (1) fingertip contact sensors firing, (2) the knuckle stalling short
        of the commanded close (an object is between the pads). Returns a dict of
        the signals observed, for fusion in the caller."""
        keys = (f'{side}_left', f'{side}_right')
        before = self._knuckle_pos(side)
        # Issue the close (non-blocking send; we poll while it executes).
        goal = GripperCommand.Goal()
        goal.command.position = float(target)
        goal.command.max_effort = float(max_effort)
        self._wait_server(client, f'{side} gripper')
        send_future = client.send_goal_async(goal)
        self._spin_until_done(send_future)
        contact = False
        end = time.monotonic() + settle
        while rclpy.ok() and time.monotonic() < end:
            if self._tip_in_contact(keys[0]) or self._tip_in_contact(keys[1]):
                contact = True
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        # Let motion settle, then read the final knuckle position for stall.
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        after = self._knuckle_pos(side)
        stalled = (after is not None and after < target - 0.06)
        # Arm wrist joint effort (Gen3 joint-torque sensors) is monitored for
        # context, but NOT used as a hard gate: a top-down straddle grasp loads the
        # wrist mostly with gravity, so an uncalibrated threshold would false-fire.
        eff = self._joint.get(f'{side}_joint_7')
        wrist_eff = abs(eff[1]) if eff else float('nan')
        self.get_logger().info(
            f'{side} close: contact_sensor={contact} '
            f'knuckle {before}->{after} stalled={stalled} '
            f'wrist|effort|={wrist_eff:.2f}.')
        return {'contact': contact, 'stalled': bool(stalled)}

    def _navigate_to_region(self):
        """Reach the operator's approximate map-frame region before scanning."""
        xy = (self.get_parameter('region_x').value,
              self.get_parameter('region_y').value)
        yaw = float(self.get_parameter('region_yaw').value)
        if not all(math.isfinite(float(v)) for v in (*xy, yaw)):
            self.get_logger().error('region coordinates must be finite')
            return False
        if not self.navigate.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('/navigate_to_pose is unavailable')
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(f'Nav2 region goal: ({xy[0]:.2f}, {xy[1]:.2f}), yaw={yaw:.2f}')
        send = self.navigate.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 600.0
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            for side, joints in POSTURES[ARM_DRIVE].items():
                for index, target in joints.items():
                    joint = f'{side}_joint_{index}'
                    actual = self._joint.get(joint)
                    if actual is None:
                        continue
                    error = math.atan2(math.sin(actual[0] - target),
                                       math.cos(actual[0] - target))
                    if abs(error) > 0.15:
                        self.get_logger().error(
                            f'{joint} deviated {error:.3f} rad while navigating; '
                            'canceling before the doorway')
                        handle.cancel_goal_async()
                        self._send_vel(0.0, 0.0)
                        return False
        if not result_future.done():
            handle.cancel_goal_async()
            self._send_vel(0.0, 0.0)
            self.get_logger().error('Nav2 region goal timed out')
            return False
        result = result_future.result()
        return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED

    # --------------------------------------------------------------------- run
    def run(self):
        # The DetachableJoint starts attached; release it so the box sits free on
        # the table until we deliberately grasp it.
        self._attach_box(False)

        if self.get_parameter('navigate_region').value:
            if not move_to_posture(self, ARM_DRIVE, label='travel to region', freeze=True):
                return self._fail('could not reach narrow travel posture')
            try:
                arrived = self._navigate_to_region()
            finally:
                switch_arm_hold(self, False)
            if not arrived:
                return self._fail('Nav2 did not reach the requested region')

        # 1. SCAN: pan the camera (base still) until the marker is found.
        found = self.scan_for_marker()
        if found is None:
            return self._fail('scan: marker not found')
        _, _, scan_pan = found

        # 2. APPROACH: visual-servo only to ~0.9 m. Closer, the low marker forces
        #    such a steep camera tilt that the vertical marker foreshortens and
        #    ArUco drops it, so we stop where it is still readable, record the box
        #    pose, then close the last bit open-loop + measure with depth.
        if not self.visual_approach(scan_pan=scan_pan, target_x=0.90):
            return self._fail('visual approach to box failed')
        self.look_down(0.65, 'STEP3 look at box')
        face = self.confirm_box()
        if face is None:
            return self._fail('box not seen at the table')

        # 3. MEASURE from here (~0.95 m), BEFORE closing in: at this range the
        #    seed window (within 0.25 m of the box) cannot contain any part of
        #    the robot itself, so the cluster is clean by construction. Depth
        #    does not foreshorten, so range is no problem for the cloud - the
        #    close-up was only ever needed for the MARKER. The camera is already
        #    aimed at the box from confirm_box (pitch 0.65). Measuring AFTER the
        #    drive was tried twice and both arm postures (spawn and ARM_CARRY)
        #    put the wrists inside the close-range window -> junk clusters.
        seed = Point(x=face.x, y=face.y, z=face.z)
        meas = self.measure_box(seed)
        if meas is None:
            # No silent fall-back to a guessed x=0.55: a phantom centre is exactly
            # what made the arms close ~0.5 m in front of the box. Abort honestly.
            return self._fail('depth measurement of the box failed (no fake grasp)')
        center, length, height, u = meas

        # Cross-check the depth centre against the marker - same vantage point,
        # nothing moved in between. A bad cluster must not silently send the
        # arms to the wrong place.
        if abs(center.x - face.x) > 0.15 or abs(center.y - face.y) > 0.15:
            self.get_logger().warn(
                f'Depth centre ({center.x:.2f},{center.y:.2f}) disagrees with '
                f'marker ({face.x:.2f},{face.y:.2f}); re-measuring once.')
            meas2 = self.measure_box(seed)
            if meas2 is None:
                return self._fail('box measurement disagreement, not confirmed')
            center, length, height, u = meas2
            if abs(center.x - face.x) > 0.15 or abs(center.y - face.y) > 0.15:
                # Accepting an unverified re-measure is how a phantom cluster
                # (e.g. the robot's own arms) walks straight into the grasp.
                return self._fail(
                    'depth centre disagrees with the marker twice (no fake grasp)')

        # 4. CLOSE IN: drive straight so the box sits at ~0.50 m (well within
        #    reach). The drive is straight ahead, so the measured centre simply
        #    shifts by the driven distance along base x. The +0.8 s compensates
        #    the trapezoidal ramps (each ramp loses half its duration of travel,
        #    2 x 0.4 s at 0.12 m/s ~ 0.10 m otherwise undershot). Any residual
        #    open-loop error is caught by the later INDEPENDENT gates (EE reach
        #    readback, fingertip geometry, physical contact) - a miss ends in an
        #    abort, never a fake grasp.
        close = max(0.0, face.x - 0.50)
        if close > 0.03:
            o0 = self._odom_xy()
            self.drive(0.12, 0.0, close / 0.12 + 0.8)
            o1 = self._odom_xy()
            # Shift the percept by the ACTUALLY driven distance (wheel odometry
            # over a straight segment, wheels tuned for negligible slip), not by
            # the commanded one: cm-level open-loop error was enough for the
            # grippers to close cleanly beside the bar (box untouched, live).
            if o0 is not None and o1 is not None:
                driven = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
            else:
                driven = close
                self.get_logger().warn(
                    'No odometry sample; assuming the commanded close-in.')
            self.get_logger().info(
                f'Close-in: commanded {close:.3f} m, odometry {driven:.3f} m.')
            center = Point(x=center.x - driven, y=center.y, z=center.z)

        # Squeeze axis from the MARKER's orientation (depth PCA is degenerate
        # on the cube's square top). marker_tangent() reads the last marker TF
        # (pre-drive) - still valid: the close-in drive did not rotate the base.
        vt = self.marker_tangent()
        if vt is not None:
            u = vt
        else:
            self.get_logger().warn(
                'No marker orientation; falling back to the depth PCA axis.')

        # 4b. CENTER the cube: turn in place until it sits dead ahead. The
        #     approach leaves it at y~-0.13, so the LEFT arm would press across
        #     the torso centre where its straight line is chronically
        #     IK-infeasible (partial fractions run after run), while the right
        #     arm pressing comfortably outside was perfect every time. The
        #     actual turn is measured by odometry and the percept (centre AND
        #     squeeze axis) is rotated by it.
        # Aim the cube at y ~ -0.075, NOT dead centre: the two arms' feasible
        # straight-press regions don't overlap at one bearing (right was
        # perfect with the cube at y=-0.13 but chronically line-infeasible at
        # y=-0.02; left exactly the opposite) - the midpoint serves both.
        bearing = math.atan2(center.y + 0.075, center.x)
        if abs(bearing) > 0.05:
            o0 = self._odom_yaw()
            self.drive(0.0, 0.3 * (1.0 if bearing > 0 else -1.0),
                       abs(bearing) / 0.3 + 0.8)
            o1 = self._odom_yaw()
            if o0 is not None and o1 is not None:
                dyaw = math.atan2(math.sin(o1 - o0), math.cos(o1 - o0))
            else:
                dyaw = bearing
                self.get_logger().warn('No odometry yaw; assuming commanded.')
            c, s = math.cos(-dyaw), math.sin(-dyaw)
            center = Point(x=c * center.x - s * center.y,
                           y=s * center.x + c * center.y, z=center.z)
            u = (c * u[0] - s * u[1], s * u[0] + c * u[1])
            self.get_logger().info(
                f'Centering turn {math.degrees(dyaw):.1f} deg; cube now at '
                f'({center.x:.2f},{center.y:.2f}), v=({u[0]:.2f},{u[1]:.2f}).')

        # 4c. SQUEEZE PLAN: the assignment cube (0.30 m, 1 kg) cannot be
        #     enclosed by the 85 mm grippers, so BOTH arms press opposite side
        #     faces with their CLOSED grippers (fingertip pads) and the
        #     contact-verified attach joint carries it.
        self.measure_tip_standoff()
        grip = Point(x=center.x, y=center.y, z=face.z)  # marker sits mid-face
        self.get_logger().info(
            f'Squeeze plan: centre=({grip.x:.2f},{grip.y:.2f},{grip.z:.2f}) '
            f'faces +/-0.15 along v=({u[0]:.2f},{u[1]:.2f}).')
        # Let MoveIt SEE the world before any arm planning: floor, table and
        # cube as collision objects (cube bottom = marker height - half cube).
        self.publish_collision_scene(grip, table_top_z=grip.z - 0.15)

        # 5. SQUEEZE: ready posture, CLOSE both grippers so the fingertips act
        #    as pressing pads (contact sensors stay live on the tips), pre-pose
        #    beside each face, then press horizontally with 5 mm interference.
        self.move_arms_joint('ARM_HOME', 'STEP5 ready posture')
        self.set_gripper(self.left_grip, 0.7, 'STEP5 close left pad')
        self.set_gripper(self.right_grip, 0.7, 'STEP5 close right pad')
        # Pre-squeeze goes through IK EXPLICITLY: solve IK for the PRESS pose
        # first, then for the pre-pose SEEDED by that solution (same branch),
        # and command the pre-pose as a JOINT goal. A plain pose-goal RRT
        # parks the arm in an arbitrary IK branch, and from most of them the
        # straight press line is infeasible (left arm: fractions 0.09-0.30
        # across every roll, run after run).
        pre_l, pre_r = self.squeeze_poses(grip, u, pre=0.10)
        lp, rp = self.squeeze_poses(grip, u, pre=-0.030)
        for side, group, ee, pre_pose, press_pose in (
                ('left', 'left_arm', 'left_end_effector_link', pre_l, lp),
                ('right', 'right_arm', 'right_end_effector_link', pre_r, rp)):
            sol_press = self._ik(group, ee, press_pose)
            sol_pre = (self._ik(group, ee, pre_pose, seed=sol_press,
                                avoid_collisions=True)
                       if sol_press is not None else None)
            if sol_pre is not None:
                self.move_arm_joints(group, sol_pre,
                                     f'STEP5 pre-squeeze {side}')
            else:
                self.get_logger().warn(
                    f'STEP5 pre-squeeze {side}: IK chain failed, falling back '
                    'to a pose goal.')
                self.plan_arm(group, ee, pre_pose, 'base_link',
                              f'STEP5 pre-squeeze {side}', ori_tol=0.5)
        # The press is a STRAIGHT Cartesian segment (~13 cm) for EACH arm, and
        # both arms press SIMULTANEOUSLY (opposed forces cancel; a lone press
        # bulldozed the cube 0.3 m across the table). The 3 cm interference is
        # deliberate: the cube (not the goal pose) stops the pads, so contact
        # is guaranteed for any tracking shortfall up to ~3.5 cm - with a 5 mm
        # target the pads regularly hovered 1-3 cm off the face. The squeeze
        # force stays bounded by the joint effort limits.
        ok_l, ok_r = self.press_both_linear(lp, rp, pre_l, pre_r)
        self._log_grasp_geometry(grip, 'at press pose')

        # 5b. REACH CHECK (independent of perception): both wrists must ACTUALLY
        #     be at the press poses (TF readback vs commanded). One corrective
        #     retry per arm: the tracking regularly leaves 4-10 cm of residual
        #     (worst on the right arm), so re-command the goal shifted by the
        #     measured error, then re-verify against the ORIGINAL goal.
        def pad_contact(side, settle=1.5):
            end = time.monotonic() + settle
            while time.monotonic() < end:
                if (self._tip_in_contact(f'{side}_left', max_age=1.0,
                                         box_only=True)
                        or self._tip_in_contact(f'{side}_right', max_age=1.0,
                                                box_only=True)):
                    return True
                rclpy.spin_once(self, timeout_sec=0.05)
            return False

        def press_reached(side, group, ee, goal_pose, ok_flag):
            if not ok_flag:
                return False
            # The JTC reports success at trajectory-end TIME while the sim arm
            # still creeps toward the goal for seconds - settle before reading.
            self._wait_settle(ee)
            if self.verify_reached(ee, goal_pose, tol=0.05,
                                   label=f'STEP5b reach {side}'):
                return True
            # A press is INTENDED contact: once the pad is on the face the
            # wrist CANNOT converge to the interference pose - the residual is
            # the press force, not a miss (seen live: tips exactly on the face
            # plane, wrist 8 cm 'short'). Accept a live pad contact as reached;
            # the two-sided physical gate below still has the final word.
            if pad_contact(side):
                self.get_logger().info(
                    f'STEP5b {side}: wrist short of goal but the pad IS in '
                    'contact - press OK.')
                return True
            # Re-target the ORIGINAL goal: move_linear replans from the current
            # pose, so no error-mirroring is needed - and mirroring OVERSHOOTS
            # (a 10 cm shortfall became a target 10 cm INSIDE the cube; the
            # stiff position-controlled arm then tunnels straight through it,
            # seen live in the GUI).
            self.move_linear(group, ee, goal_pose, f'STEP5b re-press {side}')
            self._wait_settle(ee)
            if self.verify_reached(ee, goal_pose, tol=0.05,
                                   label=f'STEP5b re-check {side}'):
                return True
            if pad_contact(side):
                self.get_logger().info(
                    f'STEP5b {side}: re-press ended in pad contact - press OK.')
                return True
            return False

        reached_l = press_reached('left', 'left_arm',
                                  'left_end_effector_link', lp, ok_l)
        reached_r = press_reached('right', 'right_arm',
                                  'right_end_effector_link', rp, ok_r)
        if not (reached_l and reached_r):
            return self._fail('arms did not reach the press poses (no fake grasp)')

        # 5c. GEOMETRY check with the cube's dimensions: both grippers' tips at
        #     the perceived cube volume (tips from TF, cube from the independent
        #     depth+marker percept), plus one corrective nudge per failing arm
        #     (MoveIt+controller tracking leaves up to ~4 cm residual error).
        on_box_geom, tips = self.fingertips_on_box(
            grip, u, 0.15, half_thick=0.15, half_h=0.16)
        if not on_box_geom:
            for side, goal_pose, group, ee in (
                    ('left', lp, 'left_arm', 'left_end_effector_link'),
                    ('right', rp, 'right_arm', 'right_end_effector_link')):
                if tips.get(side):
                    continue
                # Straight-line nudge to the ORIGINAL press goal, WITHOUT
                # collision checking (the goal deliberately touches the cube,
                # which is now a planning-scene obstacle) and WITHOUT error-
                # mirroring (that overshoots the target into the cube and the
                # stiff arm tunnels through it).
                self.move_linear(group, ee, goal_pose, f'STEP5c nudge {side}')
                self._wait_settle(ee)
            on_box_geom, tips = self.fingertips_on_box(
                grip, u, 0.15, half_thick=0.15, half_h=0.16)

        # 5d. PHYSICAL evidence: the press itself must register on the fingertip
        #     contact sensors of BOTH sides. A flat-face press produces no
        #     gripper stall (the fingers are already closed), so the sensors are
        #     the physical signal; sample them over a short settle window.
        deadline = time.monotonic() + 3.0
        contact_l = contact_r = False
        while time.monotonic() < deadline and not (contact_l and contact_r):
            contact_l = (contact_l
                         or self._tip_in_contact('left_left', box_only=True)
                         or self._tip_in_contact('left_right', box_only=True))
            contact_r = (contact_r
                         or self._tip_in_contact('right_left', box_only=True)
                         or self._tip_in_contact('right_right', box_only=True))
            rclpy.spin_once(self, timeout_sec=0.05)
        self._log_grasp_geometry(grip, 'after press')

        # FUSE the evidence honestly. The PHYSICAL signal is primary: BOTH pads
        # report a fresh contact whose other collider is literally `aruco_box`
        # (collision names from the message) - the historical failure mode
        # (grippers closing on air 0.5 m away + fake weld) cannot produce that.
        # Placement is the sanity layer: the strict fingertips-in-volume check
        # OR both wrists within 12 cm of their press goals (guards against a
        # pad merely grazing the cube while the arm is wildly displaced).
        def ee_near(ee, goal_pose, tol=0.12):
            a = self._tf_point('base_link', ee)
            return a is not None and math.sqrt(
                (a[0] - goal_pose.position.x) ** 2
                + (a[1] - goal_pose.position.y) ** 2
                + (a[2] - goal_pose.position.z) ** 2) < tol

        near_ok = (ee_near('left_end_effector_link', lp)
                   and ee_near('right_end_effector_link', rp))
        placement_ok = on_box_geom or near_ok
        physical_ok = contact_l and contact_r
        self.get_logger().info(
            f'Squeeze evidence: placement(geom={on_box_geom}, '
            f'near={near_ok})={placement_ok} '
            f'physical(contact L={contact_l}, R={contact_r})={physical_ok}.')
        if not (placement_ok and physical_ok):
            # Retreat both arms off the faces before aborting.
            self.plan_arm('left_arm', 'left_end_effector_link', pre_l,
                          'base_link', 'release left', ori_tol=0.5)
            self.plan_arm('right_arm', 'right_end_effector_link', pre_r,
                          'base_link', 'release right', ori_tol=0.5)
            return self._fail(
                'squeeze not confirmed (need placement AND contact on BOTH '
                'pads; no fake lift)')

        # 6. ATTACH (contact-verified), then LIFT. DART cannot hold the cube by
        #    pad friction alone, so the rigid attach - engaged only after the
        #    verified two-sided press - carries it (physically justified, not a
        #    teleport). NEVER two rigid holds: the RIGHT arm releases its
        #    pressure and retreats FIRST; a second rigid path shoving the fixed
        #    cube explodes the constraint solver (flings it metres away).
        self._attach_box(True)
        # The cube now moves WITH the hand: drop its (stale) collision object
        # from the planning scene - it otherwise blocks every later RRT
        # fallback (release/lower/retreat plans kept failing against the copy
        # of the cube frozen at the grasp spot).
        drop = PlanningScene()
        drop.is_diff = True
        co = CollisionObject()
        co.header.frame_id = 'base_link'
        co.id = 'target_cube'
        co.operation = CollisionObject.REMOVE
        drop.world.collision_objects = [co]
        self.scene_pub.publish(drop)
        # Right pad retreats along -v FROM ITS ACTUAL POSE (post-press wrist
        # orientation differs from the ideal one - a retreat to the ideal
        # pre-pose is IK-infeasible from here).
        if not self.retreat_linear('right_arm', 'right_end_effector_link',
                                   u, -0.10, 'STEP6 release right'):
            away_r = self.squeeze_poses(grip, u, pre=0.10)[1]
            self.plan_arm('right_arm', 'right_end_effector_link', away_r,
                          'base_link', 'STEP6 release right (RRT)', ori_tol=0.6)
        lift_l = Pose()
        lift_l.position = Point(x=lp.position.x, y=lp.position.y,
                                z=lp.position.z + 0.15)
        lift_l.orientation = lp.orientation
        self.move_linear('left_arm', 'left_end_effector_link', lift_l,
                         'STEP6 lift left')
        self._log_grasp_geometry(grip, 'after lift')
        self.get_logger().info('PICK+LIFT done (cube held by verified attach).')

        # 7. TRANSPORT PROBE (Phase 1, gated by the probe_transport param):
        #    open the pads so ONLY the rigid joint holds the cube (no gripper
        #    contact left to fight the constraint - the leading explosion
        #    hypothesis), then drive straight and turn in place while the cube
        #    hangs on the left wrist. The cube's world pose is watched from
        #    outside (gz) - if the solver explodes, it flies off visibly.
        if self.get_parameter('probe_transport').value:
            self.set_gripper(self.left_grip, 0.0, 'PROBE open left pad')
            self.set_gripper(self.right_grip, 0.0, 'PROBE open right pad')
            self.get_logger().info('TRANSPORT PROBE: straight 0.4 m.')
            self.drive(0.10, 0.0, 4.0 + 0.8)
            self.get_logger().info('TRANSPORT PROBE: settling 3 s.')
            end = time.monotonic() + 3.0
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().info('TRANSPORT PROBE: turn ~60 deg in place.')
            self.drive(0.0, 0.3, 3.5 + 0.8)
            self.get_logger().info(
                'TRANSPORT PROBE done - check the cube pose externally.')
            return

        # 8. PLACE with CONTACT: lower with the LEFT arm to a target 2 cm
        #    BELOW the pick height - the TABLE (not the goal pose) stops the
        #    cube, so at detach the drop height is ~zero (the same interference
        #    trick as the press; releasing a few cm high tipped the cube over).
        place_l = Pose()
        place_l.position = Point(x=lp.position.x, y=lp.position.y,
                                 z=lp.position.z - 0.02)
        place_l.orientation = lp.orientation
        for attempt in ('STEP8 lower left', 'STEP8 re-lower left',
                        'STEP8 re-lower left (2)'):
            self.move_linear('left_arm', 'left_end_effector_link', place_l,
                             attempt)
            self._wait_settle('left_end_effector_link')
            if self.verify_reached('left_end_effector_link', lp, tol=0.04,
                                   label=f'{attempt} check'):
                break
        self._attach_box(False)
        self.retreat_linear('left_arm', 'left_end_effector_link',
                            u, 0.10, 'STEP8 retreat left')
        self.get_logger().info('TASK COMPLETE: cube placed on the table.')

    def _fail(self, where):
        self.get_logger().error(f'Task aborted during: {where}')


def main(args=None):
    rclpy.init(args=args)
    node = MainTask()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
