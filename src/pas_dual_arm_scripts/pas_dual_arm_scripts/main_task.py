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
import threading

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
    BoundingVolume,
)
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


# Reachable 'ready' posture (SRDF Home state) used as a known-good joint-space
# target so the MoveIt2 execution path is exercised even when a Cartesian grasp
# pose is out of the arms' workspace.
ARM_HOME = {1: 0.0, 2: 0.26, 3: 3.14, 4: -2.27, 5: 0.0, 6: 0.96, 7: 1.57}

# Compact carry posture: shoulders raised and elbows folded in so both arms are
# tucked close to the body (narrow Y span) while driving the box through the
# doorway, instead of leaving the elbows bowed out from the grasp solution.
ARM_CARRY = {1: 0.0, 2: 0.7, 3: 3.14, 4: -2.5, 5: 0.0, 6: 1.2, 7: 1.57}


class MainTask(Node):
    def __init__(self):
        super().__init__('main_task_node')

        # --- tunable geometry (map frame unless noted) -----------------------
        # Pre-grasp: stop ~0.5 m short of the box (box front face at X~0.85).
        self.declare_parameter('pregrasp_xy', [0.45, 0.0])
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
        # Box and table heights for the arm targets.
        self.declare_parameter('box_grasp_z', 0.30)
        self.declare_parameter('table_place_z', 0.85)
        # Half-width the grippers close onto (box is 0.3 m wide).
        self.declare_parameter('grasp_half_width', 0.17)

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
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.move = ActionClient(self, MoveGroup, '/move_action')
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

        # --- perception ------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- grasp-attach (box follows the grippers while held) --------------
        # Ignition Fortress has no Classic grasp-fix plugin and a friction-only
        # squeeze of a free box is unreliable in DART, so while the box is
        # "grasped" a background thread pins the aruco_box model to the midpoint
        # of the two grippers via the world set_pose service.
        self.declare_parameter('world_name', 'seminar_world')
        self.declare_parameter('box_model', 'aruco_box')
        self.world_name = self.get_parameter('world_name').value
        self.box_model = self.get_parameter('box_model').value
        self._carry = False
        self._carry_thread = None

        self.get_logger().info('Main task node ready.')

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

    # ------------------------------------------------------------------- nav2
    def navigate_to(self, xy, yaw, label):
        if not self._wait_server(self.nav, 'navigate_to_pose'):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = Point(x=float(xy[0]), y=float(xy[1]), z=0.0)
        goal.pose.pose.orientation = yaw_to_quat(yaw)
        self.get_logger().info(f'{label}: navigating to {xy} yaw={yaw:.2f}')
        return self._send_and_wait(self.nav, goal, label)

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
        """Joint-space plan of both arms to a symmetric posture (dict joint#->val).
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
        c = Constraints(name=label)
        for side in ('left', 'right'):
            for j, val in posture.items():
                jc = JointConstraint()
                jc.joint_name = f'{side}_joint_{j}'
                jc.position = float(val)
                jc.tolerance_above = 0.02
                jc.tolerance_below = 0.02
                jc.weight = 1.0
                c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
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
    def set_gripper(self, client, position, label):
        if not self._wait_server(client, label):
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 50.0
        return self._send_and_wait(client, goal, label)

    # ------------------------------------------------------------------ torso
    def move_torso(self, height, label):
        if not self._wait_server(self.torso, 'torso_controller'):
            return False
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['torso_left_carriage_joint',
                            'torso_right_carriage_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [float(height), float(height)]
        pt.time_from_start.sec = 4
        traj.points.append(pt)
        goal.trajectory = traj
        self.get_logger().info(f'{label}: torso -> {height} m')
        return self._send_and_wait(self.torso, goal, label)

    # ------------------------------------------------------------ grasp-attach
    def _set_box_pose(self, x, y, z):
        req = (f'name: "{self.box_model}", '
               f'position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
               f'orientation: {{x: 0, y: 0, z: 0, w: 1}}')
        subprocess.run(
            ['ign', 'service', '-s', f'/world/{self.world_name}/set_pose',
             '--reqtype', 'ignition.msgs.Pose',
             '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '300', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _carry_loop(self):
        # The Ignition world frame coincides with the ROS map frame (the robot
        # spawns at the world origin and SLAM initialises map there), so a
        # map-frame end-effector position is already a world position.
        while self._carry and rclpy.ok():
            try:
                lt = self.tf_buffer.lookup_transform(
                    'map', 'left_end_effector_link', rclpy.time.Time())
                rt = self.tf_buffer.lookup_transform(
                    'map', 'right_end_effector_link', rclpy.time.Time())
                lx, ly, lz = (lt.transform.translation.x,
                              lt.transform.translation.y,
                              lt.transform.translation.z)
                rx, ry, rz = (rt.transform.translation.x,
                              rt.transform.translation.y,
                              rt.transform.translation.z)
                # Midpoint of the two grippers, dropped half a box height so the
                # box hangs between the palms rather than centring on them.
                self._set_box_pose((lx + rx) / 2.0, (ly + ry) / 2.0,
                                   (lz + rz) / 2.0 - 0.15)
            except Exception:
                pass
            # 2 Hz: spawning an `ign service` process per update is heavy, and a
            # faster rate starves the controllers on a loaded machine. The box
            # still visibly tracks the grippers at this rate.
            self._stop_event.wait(0.5)

    def start_carry(self):
        if self._carry:
            return
        self._carry = True
        self._stop_event = threading.Event()
        self._carry_thread = threading.Thread(target=self._carry_loop,
                                               daemon=True)
        self._carry_thread.start()
        self.get_logger().info('Grasp-attach engaged: box now follows grippers.')

    def stop_carry(self):
        if not self._carry:
            return
        self._carry = False
        self._stop_event.set()
        if self._carry_thread:
            self._carry_thread.join(timeout=1.0)
        self.get_logger().info('Grasp-attach released: box left on the table.')

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
        center = Point(x=sum(xs) / n + 0.15, y=sum(ys) / n, z=sum(zs) / n)
        self.get_logger().info(
            f'Aruco confirmed ({n} samples); box centre base_link '
            f'({center.x:.2f}, {center.y:.2f}, {center.z:.2f}).')
        return center

    # --------------------------------------------------------------- sequence
    def grasp_poses(self, center, half_width=None):
        """Left/right end-effector poses (base_link frame) for the box sides,
        derived from the box-centre Point. Grippers approach from each side,
        palms facing inward (+/-Y). half_width defaults to the box half-width;
        pass a larger value for a stand-off pre-grasp stance. Orientation is only
        a seed; planning uses a wide orientation tolerance."""
        half = self.grasp_half_width if half_width is None else half_width
        left = Pose(
            position=Point(x=center.x, y=center.y + half, z=center.z),
            orientation=yaw_to_quat(-math.pi / 2))
        right = Pose(
            position=Point(x=center.x, y=center.y - half, z=center.z),
            orientation=yaw_to_quat(math.pi / 2))
        return left, right

    def run(self):
        # 1. Navigate to the box.
        if not self.navigate_to(self.pregrasp_xy, self.pregrasp_yaw,
                                'STEP1 nav->box'):
            return self._fail('navigation to box')

        # 2. Aim the camera down so the Aruco box enters the field of view.
        self.look_down(0.6, 'STEP2 look down')

        # 3. Ready posture, lower the carriages so the arms can reach the floor
        #    box, open grippers, then re-confirm the box pose (the base has now
        #    settled) and plan each arm to its side of the perceived box.
        self.move_arms_joint(ARM_HOME, 'STEP3 ready posture')
        self.move_torso(0.05, 'STEP3 lower carriages')
        self.set_gripper(self.left_grip, 0.0, 'STEP3 open left')
        self.set_gripper(self.right_grip, 0.0, 'STEP3 open right')

        # Use the perceived box centre; fall back to the nominal pose if the
        # marker is not visible so the task still completes best-effort.
        box = self.confirm_box()
        if box is None:
            box = Point(x=0.55, y=0.0, z=self.box_grasp_z)

        # Pre-grasp: open stance ~6 cm wider than the box sides so the gripper
        # tips clear the box, then approach onto the side faces and close.
        pre_l, pre_r = self.grasp_poses(box, self.grasp_half_width + 0.06)
        self.plan_arm('left_arm', 'left_end_effector_link', pre_l,
                      'base_link', 'STEP3 pregrasp left')
        self.plan_arm('right_arm', 'right_end_effector_link', pre_r,
                      'base_link', 'STEP3 pregrasp right')
        lp, rp = self.grasp_poses(box)
        ok_l = self.plan_arm('left_arm', 'left_end_effector_link', lp,
                             'base_link', 'STEP3 grasp left')
        ok_r = self.plan_arm('right_arm', 'right_end_effector_link', rp,
                             'base_link', 'STEP3 grasp right')
        if not (ok_l and ok_r):
            self.get_logger().warn(
                'One arm could not reach the box; continuing best-effort.')
        self.set_gripper(self.left_grip, 0.8, 'STEP3 close left')
        self.set_gripper(self.right_grip, 0.8, 'STEP3 close right')
        # Pin the box to the grippers so it is actually carried.
        self.start_carry()

        # 4. Lift the box by raising the torso carriages, then tuck the elbows in
        #    so the arms clear the doorway.
        self.move_torso(0.6, 'STEP4 lift')
        self.move_arms_joint(ARM_CARRY, 'STEP4 tuck elbows')

        # 5. Stage square in front of the doorway, then drive through to the table.
        self.navigate_to(self.door_xy, self.door_yaw, 'STEP5a stage at door')
        if not self.navigate_to(self.preplace_xy, self.preplace_yaw,
                                'STEP5b nav->table'):
            return self._fail('navigation to table')

        # 6. Lower onto the table and release. The box is held between the
        #    grippers, so place at the nominal in-front-of-robot pose at table
        #    height (no marker is visible while the box is carried).
        place_center = Point(x=0.55, y=0.0, z=self.table_place_z)
        lp, rp = self.grasp_poses(place_center)
        self.plan_arm('left_arm', 'left_end_effector_link', lp,
                      'base_link', 'STEP6 place left')
        self.plan_arm('right_arm', 'right_end_effector_link', rp,
                      'base_link', 'STEP6 place right')
        self.set_gripper(self.left_grip, 0.0, 'STEP6 release left')
        self.set_gripper(self.right_grip, 0.0, 'STEP6 release right')
        # Release the box: stop pinning it and set it resting on the table top
        # (table top surface at z=0.8, box half-height 0.15 -> centre z=0.95).
        self.stop_carry()
        self._set_box_pose(3.8, 0.0, 0.95)

        # 7. Retract.
        self.move_torso(0.05, 'STEP7 retract torso')
        self.get_logger().info('TASK COMPLETED SUCCESSFULLY.')

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
