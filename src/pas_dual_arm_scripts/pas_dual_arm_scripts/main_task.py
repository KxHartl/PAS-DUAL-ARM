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
import json
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, Point, Point32, Polygon, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from nav2_msgs.action import NavigateToPose
from moveit_msgs.srv import GetStateValidity, GetCartesianPath, GetPositionIK
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
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from sensor_msgs.msg import JointState, PointCloud2
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from pas_dual_arm_scripts.robot_extent import ExtentMeasurer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pas_dual_arm_scripts.base_drive import BaseDriver
from pas_dual_arm_scripts.kinematics import ArmJog, Kinematics, pad_contact_pose
from pas_dual_arm_scripts.robot_extent import link_points
from pas_dual_arm_scripts.postures import (ARM_CARRY, ARM_DRIVE, DETECTION_CARRIAGE,
                                           DRIVE_CARRIAGE,
                                           GRIPPER_CLOSED,
                                           POSTURES, joint_constraints,
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

# The fingertip link frame is near the back of its collision mesh. Both Robotiq
# tip collision STLs extend 0.051019 m farther along local +Z, which is also the
# horizontal approach axis in the squeeze pose. Standoff distances must be
# measured from this leading collision surface, not from the link origin.
TIP_COLLISION_FORWARD = 0.051019



# Front face of the torso's collision box, in base_link (dual_arm_torso xacro:
# box 0.566 m deep centred on the column). A cube pulled in must stay ahead of it.
TORSO_FRONT_X = 0.283

# The pick table (seminar_world.sdf), placed relative to the measured cube: the
# near edge 0.25 m in front of the cube centre, as grasp_cube.py assumes too.
TABLE_SIZE = 0.80
TABLE_EDGE_AHEAD_OF_CUBE = -0.25
TABLE_LEG = 0.05
TABLE_LEG_OFFSET = 0.35

class MainTask(BaseDriver, Node):
    def __init__(self, node_name='main_task_node', simulated_contacts=True):
        super().__init__(node_name)

        # --- tunable geometry (map frame unless noted) -----------------------
        # Pre-grasp: stop short of the pick table (table front edge at X~0.85, the
        # bar's near face at X~0.865). Kept back so the base does not jam the
        # table and Nav2 can reach the goal; the bar then sits at base_link X~0.55.
        # Transport stability probe (Phase 1): after a verified attach+lift,
        # skip the place and instead drive the base (straight + in-place turn)
        # to test whether the DART DetachableJoint survives base motion.
        self.declare_parameter('probe_transport', False)
        # How far PAST the measured face the press goal is placed. The pads need
        # some interference to register on the fingertip contact sensors, but
        # the old 30 mm shoved the cube across the table - and the comment next
        # to it claimed 5 mm, so the value had drifted unnoticed. A parameter so
        # a run can find the smallest value that still registers, without an
        # edit-rebuild cycle.
        self.declare_parameter('squeeze_interference', 0.010)
        # Carriage height that puts the arms at a good working height for this
        # table, and the cube centre measured from there. The grasp height is
        # carried 1:1 from the measurement, so only these two move if the table
        # or the cube changes.
        self.declare_parameter('carriage_reference_height', 0.20)
        self.declare_parameter('carriage_reference_cube_z', 0.82)
        # How far the right pad eases off the face once the cube is attached.
        # Not a retreat - both hands are meant to stay on the cube.
        self.declare_parameter('release_backoff', 0.005)
        # The narrowest doorway on the mission route (notes/06_parametri).
        self.declare_parameter('door_width', 1.0)
        # How far the press approach tilts DOWN from horizontal, in radians.
        # This is what makes the grasp fit through a doorway. Measured offline
        # over the full IK null space (scripts/grasp_width.py), cube at 0.62 m:
        #   tilt   0 deg -> 1.003 m    45 deg -> 0.846 m
        #         30 deg -> 0.937 m    50 deg -> 0.823 m
        #         40 deg -> 0.881 m    60 deg -> 0.823 m
        # At 0 deg the spherical wrist lies on the approach axis and sets the
        # width on its own, identically in all 1620 IK solutions - no choice of
        # elbow helps. Past ~50 deg the hands are tucked inside the SHOULDER
        # mounts (0.412 m per side), which is this robot's hard floor, so there
        # is nothing to gain by tilting further. 50 deg is that floor, and it
        # is narrower than ARM_CARRY_V2 (0.840 m), the posture that was
        # validated through the 1.0 m doorways. See notes/03_problemi/P-43.
        self.declare_parameter('press_tilt', 0.873)
        # The V4 sequence (user, 16. 9.; notes/03_problemi/P-44): drive in
        # DRIVE_V4, stand at the dock, DETECTION_V4 reads the side markers,
        # GRASP_V4 then the tool tips go to the marker centres.
        # Cube distance to stand at while locating it: the dock. DRIVE_V4 reaches
        # only 0.27 m ahead, 36 cm clear of the table there.
        self.declare_parameter('detect_stand_range', 0.87)
        self.declare_parameter('detection_carriage_height', DETECTION_CARRIAGE)
        # Where the cube has to be for DETECTION_V4 and GRASP_V4 (their pads
        # and wrist cameras are set for a cube this far ahead of base_link).
        self.declare_parameter('detection_cube_range', 0.63)
        # How far the pads go INTO the face: just enough for the contact sensors
        # to register. Squeezing was tried by hand and the cube slips out (16. 9.).
        self.declare_parameter('touch_depth', 0.002)
        self.declare_parameter('grasp_approach_speed', 0.01)
        # After the lift: back away from the table this far, then lower the
        # carriages to the drive height (user, 16. 9.). At 0.40 m the cube's
        # front face is exactly at the table edge and lowering it scrapes the
        # table (0.0 cm, offline); 0.50 m leaves 10 cm.
        self.declare_parameter('back_off_after_lift', 0.50)
        # Where the carriages end, with the cube in the hands (user, 16. 9.: 100 mm).
        # Checked by MoveIt: the held grasp is free of self-collision there.
        self.declare_parameter('final_carriage_height', 0.10)
        # After the lift, pull the cube towards the robot (user, 16. 9.): both
        # hands straight back together, orientation held. Width does not grow
        # (0.82 m). Lowered to 0.10 the elbows then sit beside the torso column,
        # so they swivel away from it with the hands held: 15 cm with 15 deg is
        # free of self-collision in MoveIt both lifted and lowered. The cube's
        # back face stays >= `pull_torso_clearance` in front of the torso box.
        self.declare_parameter('pull_cube_in', 0.15)
        self.declare_parameter('pull_elbow_out_deg', 20.0)
        # A second helping of the same, used only when the carriages cannot come
        # down because the elbows are against the torso.
        self.declare_parameter('extra_elbow_out_deg', 15.0)
        self.declare_parameter('pull_torso_clearance', 0.03)
        self.declare_parameter('pull_speed', 0.04)
        # DETECTION_V4 with the hands this much further out (user: a little
        # wider): wrist cameras 0.35 m from the markers (S6 read them at 0.34),
        # 20 cm off the cube while driving in.
        self.declare_parameter('detection_widen', 0.05)
        # Carriages and arms together (user). The arms go through a point this
        # far ABOVE the detection pose and start after this share of the
        # carriage stroke, then drop straight down: 10.3 cm off the table all the
        # way (offline); straight joint interpolation alone passed 1.8 cm over it.
        self.declare_parameter('detection_via_rise', 0.12)
        self.declare_parameter('arm_start_delay', 0.3)
        # That path was checked with the cube at the dock (0.87 m); nearer than
        # this the carriages go first and MoveIt plans the arms.
        self.declare_parameter('together_min_range', 0.85)
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
        # --- mission mode (user, 16. 9.) -------------------------------------
        # The whole [MAIL] run in one node: wait for the user, drive to the room
        # with the cube, pick it, carry it through the doorway, place it. Off by
        # default so the isolated pick runs (V2-V5, spawned at the dock) are
        # unchanged. Driving goes through room_navigator, NOT through the raw
        # /navigate_to_pose goal `navigate_region` sends: only the navigator has
        # the portal poses and the dock geometry that made the doorways work
        # (P-39, runs 62/70).
        self.declare_parameter('mission', False)
        self.declare_parameter('pick_room', 'blue')
        self.declare_parameter('place_room', 'red')
        self.declare_parameter('goto_timeout', 600.0)
        # Putting the cube down on the marker. How high above the table top the
        # cube's bottom face rides while the base moves it into place, how far
        # into the table it is pressed at the end (nothing: it is set down, not
        # pushed), and how fast the carriages lower it.
        # How high the cube's BOTTOM rides above the marker while the base moves
        # it in: the user's rule is hands at marker + this + half a cube (16. 9.).
        self.declare_parameter('place_clearance', 0.20)
        self.declare_parameter('place_touch', 0.002)
        self.declare_parameter('place_lower_speed', 0.01)
        # The lowering is two moves, not one. place_lower_speed is right for the
        # last couple of centimetres, where the cube meets the table; it was
        # being used for the whole descent, 20 cm of air at 1 cm/s, which is
        # 20.6 s of every mission (measured, phase by phase, over ten runs).
        self.declare_parameter('place_fast_lower_speed', 0.05)
        self.declare_parameter('place_slow_band', 0.02)
        # Backing the base away from a table, with or without the cube. It was a
        # literal 0.08 m/s in two places, 6 s each; a straight reverse in open
        # floor does not need to crawl.
        self.declare_parameter('back_off_speed', 0.15)
        # The descent onto the detection pose is in free air, above the cube,
        # and was a literal 3 cm/s. The approach that actually touches the cube
        # is grasp_approach_speed and stays slow.
        self.declare_parameter('descent_speed', 0.06)
        # The base closing the last 20-odd cm to the place table with the cube
        # held above it. It was a literal 0.05 m/s - about 3 cm/s once the
        # wheels have slipped - and took 8 s of every mission. It does not set
        # the placement accuracy: the arms carry the cube onto the marker centre
        # immediately afterwards, and the landing is checked by measurement.
        self.declare_parameter('place_approach_speed', 0.10)
        # How far the base may close in on the table from the dock. Worked out
        # from the run of 16. 9.: the marker sat 0.915 m ahead and the table edge
        # 0.22 m nearer, so the edge was 0.695 m ahead of base_link while the
        # base itself ends at 0.41 m - 0.285 m of slack. This keeps 4.5 cm of it.
        self.declare_parameter('place_max_advance', 0.24)

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

        # DetachableJoint topics (bridged via ros_gz_bridge)
        self.attach_box_pub = self.create_publisher(
            Empty, '/aruco_box/attach', 10)
        # AMCL's way of being asked for a fix without having moved; see
        # _wake_the_localiser. None if the service never appears, and the code
        # carries on without it.
        self._nomotion = self.create_client(EmptySrv, '/request_nomotion_update')
        self.detach_box_pub = self.create_publisher(
            Empty, '/aruco_box/detach', 10)
        self._box_state = None
        self._box_state_sub = self.create_subscription(
            String, '/aruco_box/state', self._on_box_state, 10)

        # --- base velocity ---------------------------------------------------
        # Drive the base directly (no Nav2): the skid-steer base rotates by wheel
        # slip, which breaks wheel odometry and SLAM during an in-place turn, so
        # Nav2/SLAM navigation is unreliable. Instead we visual-servo to the
        # marker with direct cmd_vel. BaseDriver also subscribes to the wheel
        # odometry used to measure the ACTUAL close-in drive distance (the
        # open-loop profile has cm-level error, enough for the grippers to close
        # beside the box).
        self.init_base_drive()

        # Robot description, for measuring the robot's own width at the grasp
        # pose. The press reaches around the cube from both sides, which puts
        # the elbows outboard: measured 15. 9., the press pose is 1.531 m wide
        # against 1.0 m doorways and a 0.854 m carry posture. A grasp that
        # cannot fit through a door is not a grasp we can deliver with.
        self._description = None
        self._extent = None
        self.create_subscription(
            String, '/robot_description',
            lambda msg: setattr(self, '_description', msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # --- perception ------------------------------------------------------
        self.tf_buffer = Buffer()
        # /tf on its own thread. robot_state_publisher republishes at the joint
        # rate - 1000 Hz under the force_grasp profile - while `spin_once` runs
        # ONE callback per call, so on the shared executor the buffer falls
        # further behind the longer a loop runs. Measured 15. 9. during the
        # squeeze: the marker transform read back median 0.32 s old against a
        # 0.4 s limit, and when that limit was raised to 0.8 s the median rose
        # to 0.73 s - the lag tracked the limit, which is a backlog, not a
        # stale sensor. The detector itself never missed a frame.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        # The 640x480 point cloud is heavy; subscribing all the time starves the
        # /clock callback in the busy servo loops (sim time then never advances
        # and the loops hang). Subscribe only while measuring the box.
        self._last_cloud = None

        # Wrist->fingertip stand-off along the gripper approach axis, measured once
        # from TF after the arms reach a known posture (so end_effector goals place
        # the FINGER TIPS - not the wrist - on the bar). Nominal until measured.
        self.tip_standoff = 0.150

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
        if simulated_contacts and _Contacts is not None:
            for key, topic in self.tip_contact_topics.items():
                self.create_subscription(
                    _Contacts, topic,
                    lambda msg, k=key: self._contact_cb(k, msg), 10)
            self.get_logger().info('Fingertip contact sensors subscribed.')
        elif simulated_contacts:
            self.get_logger().warn(
                'ros_gz_interfaces/Contacts unavailable; contact-sensor signal '
                'disabled (depth + stall + effort still used).')

        # --- mission plumbing -------------------------------------------------
        # room_navigator drives; this node only asks and waits. Its status topic
        # is LATCHED, so an 'arrived' left over from an earlier leg arrives the
        # moment we subscribe - see _goto for why that cannot be trusted on its
        # own.
        self.goto_pub = self.create_publisher(String, '/room_navigator/goto', 10)
        self._nav_status = None
        self.create_subscription(
            String, '/room_navigator/status', self._nav_status_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # The user's go-ahead: the panel button or a topic publication. Nothing
        # moves the base before this arrives.
        self._mission_start = None
        self.create_subscription(String, '/mission/start',
                                 self._mission_start_cb, 10)
        # Which posture the arms are holding, for the navigator's doorway gate.
        # Empty: DRIVE_V4. With the cube: CARRY_V4. Latched, because the gate
        # reads it whenever it happens to run, not when we happen to publish.
        self.arm_posture_pub = self.create_publisher(
            String, '/room_navigator/arm_posture',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # The cube's own ground corners while it is carried, so the footprint
        # Nav2 plans with includes what the robot holds and not just its arms.
        self.carried_pub = self.create_publisher(
            Polygon, '/mission/carried_points',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.task_status_pub = self.create_publisher(
            String, '/mission/task_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._current_step = 0
        self._total_steps = 8
        # How the cube sits in the hand, recorded at the moment of the attach:
        # {'offset', 'normal', 'half'} in the left_tool_tip frame. The wrist
        # cameras CANNOT be used once the cube is held - the pads end up 2 cm
        # from the markers and nothing decodes at that range - so the pose of a
        # held cube comes from this rigid relation instead (P-45).
        self._hold = None

        self.get_logger().info('Main task node ready.')

    def _nav_status_cb(self, msg):
        try:
            self._nav_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._nav_status = {'state': 'unknown', 'detail': msg.data}

    def _mission_start_cb(self, msg):
        self._mission_start = msg.data.strip() or ''
        self.get_logger().info(
            f'mission start requested by the user: "{self._mission_start}"')

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

    def _on_box_state(self, msg: String):
        self._box_state = msg.data.strip().lower()

    def _attach_box(self, attach: bool, timeout: float = 5.0) -> bool:
        """Rigidly attach / detach the box to the left wrist via the
        DetachableJoint topics. Only called to ATTACH after
        the contact check confirms the grippers are genuinely on the box, so the
        rigid hold stands in for the friction DART cannot provide - not a fake."""
        target_state = 'attached' if attach else 'detached'
        topic_name = '/aruco_box/attach' if attach else '/aruco_box/detach'
        pub = self.attach_box_pub if attach else self.detach_box_pub

        if self._box_state == target_state:
            self.get_logger().info(f'Box already {target_state}.')
            return True

        self.get_logger().info(f'Requesting box {target_state.upper()}...')
        msg = Empty()
        deadline = time.monotonic() + timeout
        last_pub = 0.0

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_pub >= 0.2:
                # 1. Publish via persistent ROS 2 bridge
                pub.publish(msg)
                # 2. Also trigger via ign topic CLI as redundant fallback
                subprocess.run(
                    ['ign', 'topic', '-t', topic_name, '-m', 'ignition.msgs.Empty',
                     '-p', 'unused: true'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                last_pub = now

            rclpy.spin_once(self, timeout_sec=0.05)

            if self._box_state == target_state:
                self.get_logger().info(
                    f'Box {target_state.upper()} confirmed via /aruco_box/state.')
                return True

        if self._box_state is None and not attach:
            # At startup, joint may have already been detached on spawn and therefore
            # no state change was emitted.
            self.get_logger().warn(
                'No state received on /aruco_box/state; detach command was sent.')
            return True

        self.get_logger().error(
            f'Box {target_state.upper()} NOT confirmed! Current state: {self._box_state}')
        return False

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
        spread = []
        for side in ('left', 'right'):
            for f in ('left', 'right'):
                try:
                    tf = self.tf_buffer.lookup_transform(
                        f'{side}_end_effector_link',
                        f'{side}_robotiq_85_{f}_finger_tip_link',
                        rclpy.time.Time())
                    t = tf.transform.translation
                    # Project onto the approach axis (+Z) rather than taking
                    # the distance. With the fingers CLOSED the two are the
                    # same to within a millimetre, but open pads sit well off
                    # the axis, and their distance would then read as reach the
                    # hand does not have - stopping it short of the face.
                    vals.append(t.z + TIP_COLLISION_FORWARD)
                    spread.append(math.hypot(t.x, t.y))
                except Exception:
                    continue
        if vals:
            self.tip_standoff = sum(vals) / len(vals)
            self.get_logger().info(
                f'Measured wrist->leading fingertip collision = '
                f'{self.tip_standoff:.3f} m along the approach axis '
                f'(includes {TIP_COLLISION_FORWARD * 1000:.1f} mm mesh extent), '
                f'pads {sum(spread) / len(spread):.3f} m off it '
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

    def _ik(self, group, ee, pose, seed=None, avoid_collisions=False,
            timeout=3.0):
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
        r.timeout.sec = int(timeout)
        r.timeout.nanosec = int((timeout - int(timeout)) * 1e9)
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
        req.max_velocity_scaling_factor = 0.7
        req.max_acceleration_scaling_factor = 0.7
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

    def plan_arm_joints(self, group, joints, label, velocity_scale=0.6,
                        acceleration_scale=0.6):
        """Plan ONE arm to explicit joint targets WITHOUT executing it, and
        return the trajectory. Needed because move_group executes one
        trajectory at a time: a two-arm move driven through it is necessarily
        sequential, and the arm that arrives first shoves the cube across the
        table before the opposite one is there to balance it (P-26). Planning
        both first and releasing them together is the only way the two hands
        actually travel at the same time."""
        if not self._wait_server(self.move, 'move_action'):
            return None
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = float(velocity_scale)
        req.max_acceleration_scaling_factor = float(acceleration_scale)
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
        goal.planning_options.plan_only = True
        self.get_logger().info(f'{label}: planning {group} (joint-space)')
        send_future = self.move.send_goal_async(goal)
        handle = self._spin_until_done(send_future)
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{label}: planning goal rejected.')
            return None
        result_future = handle.get_result_async()
        result = self._spin_until_done(result_future)
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'{label}: planning FAILED.')
            return None
        traj = result.result.planned_trajectory.joint_trajectory
        if not traj.points:
            self.get_logger().error(f'{label}: planner returned an empty path.')
            return None
        return traj

    @staticmethod
    def _stretch_to(traj, duration):
        """Scale a trajectory's timing so it ends at `duration`. Two separately
        planned paths have different lengths; without this the short one
        finishes early and that hand stands alone against the cube for the
        remainder."""
        own = (traj.points[-1].time_from_start.sec
               + traj.points[-1].time_from_start.nanosec * 1e-9)
        if own <= 0.0 or duration <= own:
            return
        scale = duration / own
        for point in traj.points:
            t = (point.time_from_start.sec
                 + point.time_from_start.nanosec * 1e-9) * scale
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            # The velocities belonged to the original timing.
            point.velocities = []
            point.accelerations = []

    def tips_on_box(self, side, max_age=0.6):
        """How many of this hand's two pads are freshly touching THE BOX."""
        return sum(self._tip_in_contact(f'{side}_{finger}', max_age=max_age,
                                        box_only=True)
                   for finger in ('left', 'right'))

    def box_contact_snapshot(self, max_age=0.12):
        """
        NOT USED BY THE MISSION. Kept because the force-grasp experiment
        needs this primitive; the delivered mission never calls it, so the
        contact sensors do not close a loop here. See docs/SENSORS.md.

        Return the fresh box-only state of every fingertip sensor.

        The contact sensors publish at 50 Hz.  A short freshness window filters
        old contacts without treating one delayed bridge sample as a new grasp.
        """
        return {
            f'{side}_{finger}': self._tip_in_contact(
                f'{side}_{finger}', max_age=max_age, box_only=True)
            for side in ('left', 'right')
            for finger in ('left', 'right')
        }

    def approach_both_linear(self, targets, label, min_frac=0.85,
                             min_duration=0.0):
        """Take BOTH hands along a straight line to their targets at once.

        `targets` is {side: (group, ee_link, pose)}. Straight matters here: the
        pads have to arrive square against the face, and a joint-space plan for
        the last stretch curves the hand in sideways. Simultaneous matters for
        the same reason it does everywhere else on this cube - whichever hand
        lands first pushes it out from under the other (P-26).

        Returns {side: bool}, or None if a line could not be computed.
        """
        trajs = {}
        for side, (group, ee, pose) in targets.items():
            # Compute from where the arm ACTUALLY is; a line built while it is
            # still creeping starts from a stale state and the JTC lurches.
            self._wait_settle(ee)
            got = self._linear_traj(group, ee, pose, f'{label} {side}')
            if got is None:
                self.get_logger().error(f'{label} {side}: no straight line')
                return None
            solution, fraction = got
            if fraction < min_frac:
                self.get_logger().error(
                    f'{label} {side}: line only {fraction:.2f} complete')
                return None
            if not self._traj_starts_here(solution.joint_trajectory,
                                          f'{label} {side}'):
                return None
            trajs[side] = solution.joint_trajectory
        return self.move_arms_parallel(trajs, label,
                                       min_duration=min_duration)

    def move_arms_parallel(self, trajs, label, min_duration=0.0):
        """Run both arm trajectories at the same time, sent straight to the two
        JTCs. Both are stretched to the same duration first, so the hands
        arrive together rather than one waiting on the other. `min_duration`
        stretches them further: the planners time a short path in a fraction of
        a second, which near the cube is a lunge. Returns {side: bool}."""
        clients = {'left': self.left_jtc, 'right': self.right_jtc}
        for side, traj in trajs.items():
            if not self._traj_starts_here(traj, f'{label} {side}'):
                return {side: False for side in trajs}
            if not self._wait_server(clients[side], f'{side} JTC'):
                return {side: False for side in trajs}
        longest = max(
            [t.points[-1].time_from_start.sec
             + t.points[-1].time_from_start.nanosec * 1e-9
             for t in trajs.values()] + [float(min_duration)])
        for traj in trajs.values():
            self._stretch_to(traj, longest)
        self.get_logger().info(
            f'{label}: releasing {len(trajs)} arms together over '
            f'{longest:.1f} s')
        goals = {}
        for side, traj in trajs.items():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            goals[side] = clients[side].send_goal_async(goal)
        handles = {}
        for side, future in goals.items():
            self._spin_until_done(future)
            handle = future.result()
            handles[side] = handle if (handle is not None
                                       and handle.accepted) else None
        outs = {}
        for side, handle in handles.items():
            if handle is None:
                outs[side] = False
                self.get_logger().error(f'{label} {side}: JTC rejected the path')
                continue
            result_future = handle.get_result_async()
            self._spin_until_done(result_future)
            result = result_future.result()
            outs[side] = (result is not None
                          and result.status == GoalStatus.STATUS_SUCCEEDED)
            self.get_logger().info(
                f'{label} {side}: {"OK" if outs[side] else "FAILED"}')
        return outs

    def approach_both_until_contact(self, targets, label, duration,
                                    contact_max_age=0.4):
        """
        NOT USED BY THE MISSION. Kept because the force-grasp experiment
        needs this primitive; the delivered mission never calls it, so the
        contact sensors do not close a loop here. See docs/SENSORS.md.

        Approach with both arms and independently stop each on first contact.

        MoveIt computes the straight Cartesian paths, but the two trajectories
        are sent directly to their JTCs.  Cancelling a JTC goal installs its
        hold-position trajectory, so one hand can stop while the other keeps
        approaching.  Returns the final fresh pad count for each hand.
        """
        trajs = {}
        for side, (group, ee, pose) in targets.items():
            self._wait_settle(ee)
            got = self._linear_traj(group, ee, pose, f'{label} {side}')
            if got is None:
                raise RuntimeError(f'{label} {side}: no straight path')
            solution, fraction = got
            if fraction < 0.85:
                raise RuntimeError(
                    f'{label} {side}: path only {fraction:.2f} complete')
            traj = solution.joint_trajectory
            if not self._traj_starts_here(traj, f'{label} {side}'):
                raise RuntimeError(f'{label} {side}: stale trajectory start')
            self._stretch_to(traj, duration)
            trajs[side] = traj

        clients = {'left': self.left_jtc, 'right': self.right_jtc}
        for side in ('left', 'right'):
            if not self._wait_server(clients[side], f'{side} JTC'):
                raise RuntimeError(f'{side} JTC unavailable')
        send_futures = {}
        for side in ('left', 'right'):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajs[side]
            send_futures[side] = clients[side].send_goal_async(goal)

        handles = {}
        for side, future in send_futures.items():
            handle = self._spin_until_done(future)
            if handle is None or not handle.accepted:
                for accepted in handles.values():
                    accepted.cancel_goal_async()
                raise RuntimeError(f'{label} {side}: goal rejected')
            handles[side] = handle

        results = {side: handle.get_result_async()
                   for side, handle in handles.items()}
        cancel_futures = {}
        active = set(handles)
        while active:
            rclpy.spin_once(self, timeout_sec=0.01)
            for side in tuple(active):
                if results[side].done():
                    active.remove(side)
                elif self.tips_on_box(side, max_age=contact_max_age) >= 1:
                    if side not in cancel_futures:
                        self.get_logger().info(
                            f'{label}: {side} first pad contact; stopping arm')
                        cancel_futures[side] = handles[side].cancel_goal_async()
            for side, future in tuple(cancel_futures.items()):
                if future.done() and side in active:
                    response = future.result()
                    if response is None or not response.goals_canceling:
                        raise RuntimeError(
                            f'{label} {side}: controller did not accept stop')
                    active.remove(side)

        # Let accepted cancels reach the controller and verify both mechanisms
        # have stopped before any fine correction is issued.
        for side, future in cancel_futures.items():
            self._spin_until_done(future)
            self.get_logger().info(f'{label}: {side} arm is holding')
        for side in ('left', 'right'):
            self._wait_settle(f'{side}_end_effector_link')
        return {side: self.tips_on_box(side, max_age=contact_max_age)
                for side in ('left', 'right')}

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
        controllers. A lone press bulldozes the 0.3 kg cube across the table
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

    def publish_collision_scene(self, center, table_top_z, yaw=0.0, settle=True):
        """Add the floor, the pick table and the target cube to the MoveIt
        planning scene (base_link frame). Without them RRT freely plans arm
        sweeps THROUGH the table - the physical snag then shoves the whole
        base. The deliberate press still touches the cube because it runs via
        compute_cartesian_path with avoid_collisions=False.

        MoveIt has no virtual joint to the world here, so it plans in the
        robot's own frame and these objects ride along with the base. `yaw` is
        the table's heading relative to base_link, for re-publishing them where
        they really are after the base has moved (grasp_stage follows teleop
        this way); `settle=False` skips the one-second wait for move_group."""
        c, s_ = math.cos(yaw), math.sin(yaw)

        def box_object(name, dx, dy, cz, sx, sy, sz):
            """A box at (dx, dy) from the cube centre in the TABLE's axes."""
            co = CollisionObject()
            co.header.frame_id = 'base_link'
            co.id = name
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [float(sx), float(sy), float(sz)]
            pose = Pose()
            pose.position = Point(x=float(center.x + c * dx - s_ * dy),
                                  y=float(center.y + s_ * dx + c * dy), z=float(cz))
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            co.primitives = [prim]
            co.primitive_poses = [pose]
            co.operation = CollisionObject.ADD
            return co

        scene = PlanningScene()
        scene.is_diff = True
        # The table as it is (seminar_world.sdf): a 0.80 m square top, 4 cm
        # thick, on four 5 cm legs 0.35 m from its centre, its near edge 0.25 m
        # in front of the cube - the same offset grasp_cube.py and grasp_stage
        # stop the base by. It used to be a 0.55 x 0.65 x 0.10 m slab centred
        # on the cube: too thick (MoveIt saw the carry posture's hands, which sit
        # under the top, as a collision - F12), too narrow (the real top reaches
        # y = +/-0.40) and without legs, so the planner swung a hand straight
        # into the near leg and the controller still reported success (S5).
        top_dx = TABLE_EDGE_AHEAD_OF_CUBE + TABLE_SIZE / 2.0
        legs = [box_object(f'pick_table_leg_{i}',
                           top_dx + sx * TABLE_LEG_OFFSET, sy * TABLE_LEG_OFFSET,
                           (table_top_z - 0.04) / 2.0,
                           TABLE_LEG, TABLE_LEG, table_top_z - 0.04)
                for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)))]
        floor = box_object('floor', 0.0, 0.0, -0.10, 4.0, 4.0, 0.02)
        floor.primitive_poses[0].position = Point(x=0.7, y=0.0, z=-0.10)
        scene.world.collision_objects = [
            floor,
            box_object('pick_table', top_dx, 0.0,
                       table_top_z - 0.02, TABLE_SIZE, TABLE_SIZE, 0.04),
            *legs,
            box_object('target_cube', 0.0, 0.0,
                       table_top_z + 0.15, 0.30, 0.30, 0.30),
        ]
        self.scene_pub.publish(scene)
        if not settle:
            return
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
            req.max_velocity_scaling_factor = 0.7
            req.max_acceleration_scaling_factor = 0.7
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
        req.max_velocity_scaling_factor = 0.7
        req.max_acceleration_scaling_factor = 0.7
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
        req.max_velocity_scaling_factor = 0.7
        req.max_acceleration_scaling_factor = 0.7
        req.goal_constraints.append(joint_constraints(posture, label))
        goal.planning_options.plan_only = False
        self.get_logger().info(f'{label}: planning both_arms (joint-space)')
        return self._send_and_wait(self.move, goal, label)

    # --------------------------------------------------------------- pan-tilt
    def look_down(self, pitch, label, yaw=0.0):
        """Point the camera: `pitch` down, `yaw` across. Default straight ahead."""
        if not self._wait_server(self.pan_tilt, 'pan_tilt_controller'):
            return False
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['pan_tilt_yaw_joint', 'pan_tilt_pitch_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [float(yaw), float(pitch)]
        pt.time_from_start.sec = 1
        traj.points.append(pt)
        goal.trajectory = traj
        self.get_logger().info(
            f'{label}: camera to pitch={pitch:.2f}, yaw={yaw:.2f}')
        return self._send_and_wait(self.pan_tilt, goal, label)

    # ---------------------------------------------------------------- gripper
    def set_gripper(self, client, position, label, max_effort=50.0):
        if not self._wait_server(client, label):
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        return self._send_and_wait(client, goal, label)

    def set_both_grippers(self, position, label, max_effort=50.0):
        """Command both grippers at the same instant and wait for both."""
        clients = {'left': self.left_grip, 'right': self.right_grip}
        for side, client in clients.items():
            if not self._wait_server(client, f'{label} {side}'):
                return False
        futures = {}
        for side, client in clients.items():
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = float(max_effort)
            futures[side] = client.send_goal_async(goal)
        handles = {}
        for side, future in futures.items():
            handle = self._spin_until_done(future)
            if handle is None or not handle.accepted:
                return False
            handles[side] = handle
        ok = True
        for side, handle in handles.items():
            result = self._spin_until_done(handle.get_result_async())
            succeeded = (result is not None
                         and result.status == GoalStatus.STATUS_SUCCEEDED)
            self.get_logger().info(
                f'{label} {side}: {"OK" if succeeded else "FAILED"}')
            ok = ok and succeeded
        return ok

    def measure_width(self, label):
        """Robot's lateral extent right now, in metres, or None.

        Same geometry the navigator gates on (`robot_extent`), so the number
        here and the number that decides a doorway cannot drift apart.
        """
        if self._description is None:
            deadline = time.monotonic() + 5.0
            while self._description is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
        if self._description is None:
            self.get_logger().warn(f'{label}: no /robot_description, width unknown')
            return None
        if self._extent is None:
            try:
                self._extent = ExtentMeasurer(self._description)
            except Exception as exc:
                self.get_logger().warn(f'{label}: cannot build extent model: {exc}')
                return None
        # ExtentMeasurer.points() SKIPS any link TF cannot place, so a width
        # built from a subset of the robot reads narrower than the robot is.
        # Count what was actually placed and refuse to answer on a partial
        # sample - a width nobody can trust is worse than no width (D-12).
        placed = []
        for name in self._extent.links:
            try:
                self.tf_buffer.lookup_transform(
                    self._extent.frame, name, rclpy.time.Time())
                placed.append(name)
            except Exception:
                pass
        missing = [n for n in self._extent.links if n not in placed]
        points = self._extent.points(self.tf_buffer, rclpy.time.Time())
        if points is None:
            self.get_logger().warn(f'{label}: TF placed no link, width unknown')
            return None
        if missing:
            self.get_logger().warn(
                f'{label}: width UNRELIABLE - TF placed {len(placed)}/'
                f'{len(self._extent.links)} links, missing {sorted(missing)[:6]}')
            return None
        width = float(points[:, 1].max() - points[:, 1].min())
        door = float(self.get_parameter('door_width').value)
        verdict = 'fits' if width < door else 'TOO WIDE'
        self.get_logger().info(
            f'WIDTH MEASURED [{label}] {width:.3f} m vs doorway {door:.3f} m '
            f'-> {verdict} ({len(placed)} links)')
        return width

    # ------------------------------------------------------------------ torso
    def move_torso(self, height, label, secs=2):
        if not self._wait_server(self.torso, 'torso_controller'):
            return False
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['torso_left_carriage_joint',
                            'torso_right_carriage_joint']

        # Single endpoint with explicit zero velocities. The JTC assumes the
        # robot starts from rest (current state) and generates a smooth cubic
        # profile to reach this point. An explicit t=0 anchor was tried but
        # caused a jerk: the JTC activates the trajectory with a small delay
        # after send_goal_async, so by the time it processes t=0 the clock has
        # already passed it — the controller sees a nonzero position error at
        # t=0 and issues a large corrective velocity spike.
        left_now = self._joint.get('torso_left_carriage_joint', (0.0, 0.0))[0]
        right_now = self._joint.get('torso_right_carriage_joint', (0.0, 0.0))[0]
        pt = JointTrajectoryPoint()
        pt.positions = [float(height), float(height)]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start.sec = int(secs)
        traj.points.append(pt)

        goal.trajectory = traj
        self.get_logger().info(f'{label}: torso {left_now:.4f}/{right_now:.4f}'
                               f' -> {height:.4f} m over {secs}s')
        return self._send_and_wait(self.torso, goal, label)

    def move_torso_guarded(self, targets, label, speed, guard):
        """
        NOT USED BY THE MISSION. Kept because the force-grasp experiment
        needs this primitive; the delivered mission never calls it, so the
        contact sensors do not close a loop here. See docs/SENSORS.md.

        Move both carriages while polling a safety predicate.

        ``targets`` contains the independent left/right absolute heights.  If
        ``guard`` becomes false, cancel the trajectory immediately and leave
        the controller holding the measured position.  The return value is
        ``'reached'``, ``'guard_failed'`` or ``'failed'``.
        """
        if not self._wait_server(self.torso, 'torso_controller'):
            return 'failed'
        names = ['torso_left_carriage_joint',
                 'torso_right_carriage_joint']
        current = [self._joint.get(name, (0.0, 0.0))[0] for name in names]
        duration = max(abs(float(target) - now)
                       for target, now in zip(targets, current)) / float(speed)
        duration = max(duration, 0.1)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in targets]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int(
            (duration - int(duration)) * 1e9)
        goal.trajectory.points = [point]
        self.get_logger().info(
            f'{label}: torso {current[0]:.4f}/{current[1]:.4f} -> '
            f'{targets[0]:.4f}/{targets[1]:.4f} m over {duration:.1f}s')

        send_future = self.torso.send_goal_async(goal)
        handle = self._spin_until_done(send_future)
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{label}: torso goal rejected')
            return 'failed'
        result_future = handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
            if not guard():
                self.get_logger().warn(
                    f'{label}: contact guard lost; stopping carriages')
                cancel_future = handle.cancel_goal_async()
                self._spin_until_done(cancel_future)
                response = cancel_future.result()
                if response is None or not response.goals_canceling:
                    self.get_logger().error(
                        f'{label}: torso controller rejected stop')
                    return 'failed'
                self._wait_settle('left_end_effector_link')
                return 'guard_failed'

        result = result_future.result()
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f'{label}: {"OK" if ok else "FAILED"}')
        return 'reached' if ok else 'failed'

    def hold_current_mechanisms(self, reason='safety hold'):
        """Replace residual arm/torso commands with measured joint positions."""
        if not rclpy.ok():
            return False
        specs = (
            ('left', self.left_jtc,
             [f'left_joint_{index}' for index in range(1, 8)]),
            ('right', self.right_jtc,
             [f'right_joint_{index}' for index in range(1, 8)]),
            ('torso', self.torso,
             ['torso_left_carriage_joint',
              'torso_right_carriage_joint']),
        )
        sends = {}
        for label, client, names in specs:
            if not all(name in self._joint for name in names):
                self.get_logger().warn(
                    f'{reason} {label}: joint state unavailable')
                continue
            if not client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn(
                    f'{reason} {label}: controller unavailable')
                continue
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = names
            point = JointTrajectoryPoint()
            point.positions = [float(self._joint[name][0]) for name in names]
            point.velocities = [0.0] * len(names)
            point.time_from_start.nanosec = 300000000
            goal.trajectory.points = [point]
            sends[label] = client.send_goal_async(goal)

        accepted = {}
        for label, future in sends.items():
            self._spin_until_done(future)
            handle = future.result()
            if handle is not None and handle.accepted:
                accepted[label] = handle
            else:
                self.get_logger().error(
                    f'{reason} {label}: goal rejected')
        for label, handle in accepted.items():
            result = handle.get_result_async()
            self._spin_until_done(result)
            self.get_logger().info(
                f'{reason} {label}: current position latched')
        return len(accepted) == len(specs)

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
        seen_stamps = set()
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
                stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
                if stamp in seen_stamps:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue
                seen_stamps.add(stamp)
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
        # The pickup table puts the box around z=0.9 m in the current world.
        # Anchor the crop to the observed marker instead of the old floor-box
        # height, while excluding tabletop points below the marker.
        m = ((P[:, 2] > seed.z - 0.10) & (P[:, 2] < seed.z + 0.25)
             & (P[:, 0] > max(0.35, seed.x - 0.19))
             & (P[:, 0] < seed.x + 0.18)
             & (np.abs(P[:, 1] - seed.y) < 0.20))
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

    def squeeze_poses(self, center, v, pre=0.0, half_width=None, tilt=None):
        """Left/right EE poses for the dual-arm SQUEEZE of the assignment cube:
        each CLOSED gripper (fingertip pads) approaches horizontally along -/+v
        and presses one of the two opposite side faces. pre>0 hovers off the
        face; pre<0 commands a slight interference so the pads genuinely press.
        Approach axis is tool +Z (same convention as the top-down grasp, where
        the tips extend along +Z from the wrist); tool +X is kept horizontal so
        the two pads contact side by side (stable against yaw torque)."""
        # half_width: half the cube's depth along the approach axis. It used to
        # be hard-coded 0.15 (half the nominal 0.30 m cube) while the depth
        # camera's MEASURED length was logged and thrown away, so a cube of any
        # other size would have been pressed at the wrong distance. Callers pass
        # the measurement; the nominal value stays as the fallback.
        half = 0.15 if half_width is None else float(half_width)
        # `tilt` overrides the press tilt, e.g. 0 for the horizontal side-scan
        # pre-grasp in which the wrist cameras were validated (run 72).
        tilt = float(self.get_parameter('press_tilt').value if tilt is None else tilt)
        vv = np.array([float(v[0]), float(v[1]), 0.0])
        vv = vv / np.linalg.norm(vv)
        up = np.array([0.0, 0.0, 1.0])
        poses = []
        for sgn in (+1.0, -1.0):        # left presses from +v, right from -v
            # Approach axis: into the box, tilted DOWN by `tilt`. Horizontal
            # (tilt = 0) is the obvious choice and the one that cannot pass a
            # doorway: the spherical wrist sits ON this axis, 0.2025 m outboard
            # of the fingertip, so a horizontal press is 2*(0.15 + 0.149 +
            # 0.2025) = 1.003 m wide whatever the elbows do - measured across
            # 1620 IK solutions, every one of them exactly 0.502 m per side.
            # Tilting the axis down swings the wrist UP instead of OUT, which
            # is the only free direction left (P-43).
            z_ax = -sgn * np.cos(tilt) * vv - np.sin(tilt) * up
            z_ax = z_ax / np.linalg.norm(z_ax)
            # Horizontal, across the face - the SAME roll as the flat version
            # (-z_ax[1], z_ax[0]). Flipping it (tried 15. 9.) turns the hand 180 deg
            # about the approach axis: the wrist camera then sits 5.6 cm BELOW the
            # axis instead of above it, and the face marker, raised 5.6 cm for a
            # camera above, falls out of frame (stage run S1: marker NOT seen).
            x_ax = np.array([vv[1], -vv[0], 0.0]) * sgn
            y_ax = np.cross(z_ax, x_ax)
            y_ax = y_ax / np.linalg.norm(y_ax)
            x_ax = np.cross(y_ax, z_ax)
            R = np.column_stack([x_ax, y_ax, z_ax])
            # Aim the fingertip at the CENTRE of the side face, then back the
            # wrist off ALONG the approach axis. Offsetting from the cube
            # centre along a tilted axis instead (what the flat version did,
            # correctly, because its axis was horizontal) lands the tip
            # half*tan(tilt) too high - 0.15 m at 45 deg, the top edge of a
            # 0.30 m cube rather than its face.
            contact = np.array([center.x, center.y, center.z]) + sgn * half * vv
            p = contact - (self.tip_standoff + pre) * z_ax
            poses.append(Pose(position=Point(x=float(p[0]), y=float(p[1]),
                                             z=float(p[2])),
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
        pt.time_from_start.sec = 1
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
        """
        NOT USED BY THE MISSION. Kept because the force-grasp experiment
        needs this primitive; the delivered mission never calls it, so the
        contact sensors do not close a loop here. See docs/SENSORS.md.

        Command the gripper closed and watch for a genuine stop signal while it
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

    def _wait_for_start(self, default_room):
        """Block until the user sends the robot after the cube. Returns the room."""
        self.get_logger().info(
            'WAITING for the user: press "MISIJA: po kutiju" in the navigation '
            'panel, or run: ros2 topic pub --once /mission/start '
            f'std_msgs/String "{{data: {default_room}}}"')
        while rclpy.ok() and self._mission_start is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        room = (self._mission_start or '').strip() or default_room
        self._mission_start = None
        self.get_logger().info(f'MISSION START: going for the cube in "{room}"')
        self._wait_for_localisation()
        return room

    def _wait_for_localisation(self, timeout=90.0, freshness=1.0, settle=2.0):
        """Do not drive on a transform that is not there yet, or is stale.

        AMCL needs a few seconds and a scan or two after the stack is up. Press
        the button before that and the controller works with an out-of-date
        `map -> odom` - "Transform data too old", visible in the log - and the
        goal, transformed into the local frame with it, can land on top of the
        robot. Nav2 then reports the leg as reached without the robot having
        moved. Waiting here costs seconds; not waiting produces a run that
        claims to have arrived somewhere it never left.
        """
        started = time.time()
        stable_since = None
        while rclpy.ok() and time.time() - started < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time())
            except Exception:
                stable_since = None
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - stamp > freshness:
                stable_since = None
                continue
            stable_since = stable_since or time.time()
            if time.time() - stable_since >= settle:
                self.get_logger().info(
                    f'localised: map -> base_footprint fresh and stable for '
                    f'{settle:.0f} s; driving')
                return True
        self.get_logger().error(
            'map -> base_footprint never became fresh; AMCL is not localised. '
            'Driving now would be driving on a stale transform.')
        return False

    def _goto(self, destination, timeout=None):
        """Drive to `destination` through room_navigator and wait for its verdict.

        Not /navigate_to_pose: the navigator is what knows the portal poses, the
        doorway gate and the dock geometry (P-39). The verdict is read off the
        latched status topic, where a stale 'arrived' from an earlier leg is
        indistinguishable from this one's - so an 'arrived' only counts after a
        'driving' has been seen for the destination we just asked for.
        """
        timeout = (float(self.get_parameter('goto_timeout').value)
                   if timeout is None else float(timeout))
        self._nav_status = None
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and self.goto_pub.get_subscription_count() < 1:
            if time.monotonic() > deadline:
                self.get_logger().error(
                    'room_navigator is not listening on /room_navigator/goto; '
                    'is nav2.launch.py up (zones:=true)?')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        self.goto_pub.publish(String(data=destination))
        self.get_logger().info(f'NAV: asked room_navigator for "{destination}"')
        seen_driving = False
        # On the SIMULATION clock, like every other timeout the robot's
        # behaviour depends on. On the wall clock the same 600 s shrinks with
        # the real-time factor: three simulations in parallel run at 0.29, so it
        # meant 174 s of the robot's time, and a transport that took one stall
        # retry was abandoned mid-doorway while it was driving normally.
        # Wall time still bounds it (x4), so a frozen /clock cannot hang the
        # mission forever.
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        wall_end = time.monotonic() + 4.0 * timeout
        while (rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < end
               and time.monotonic() < wall_end):
            rclpy.spin_once(self, timeout_sec=0.1)
            status = self._nav_status
            if not status or status.get('destination') != destination:
                continue
            state = status.get('state')
            if state == 'driving':
                if not seen_driving:
                    self.get_logger().info(f'NAV: {status.get("detail")}')
                seen_driving = True
            elif state in ('failed', 'aborted', 'cancelled'):
                self.get_logger().error(
                    f'NAV {state} on the way to "{destination}": '
                    f'{status.get("detail")}')
                return False
            elif state == 'arrived' and seen_driving:
                self.get_logger().info(f'NAV: arrived at "{destination}"')
                return True
        self.get_logger().error(f'NAV: timed out on the way to "{destination}"')
        return False

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
    # ------------------------------------------------------- grasp helpers
    def _torso_to(self, height, label, secs=3):
        """Carriages to `height`, then MEASURED - the action status is not trusted."""
        if not self.move_torso(height, label, secs=secs):
            return False
        settle = time.monotonic() + 1.2
        while time.monotonic() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        at = [self._joint.get(n, (float('nan'), 0.0))[0]
              for n in ('torso_left_carriage_joint', 'torso_right_carriage_joint')]
        self.get_logger().info(
            f'{label}: CARRIAGE MEASURED left={at[0]:.4f} right={at[1]:.4f} m '
            f'(commanded {height:.4f})')
        return max(abs(a - height) for a in at) <= 0.01

    def _grasp_tools(self):
        """Whole-robot kinematics, collision hulls and per-arm jog, from the URDF."""
        if getattr(self, '_kin_tools', None) is None:
            deadline = time.monotonic() + 5.0
            while self._description is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if self._description is None:
                raise RuntimeError('no /robot_description for the grasp kinematics')
            kin = Kinematics(self._description)
            self._kin_tools = (kin, link_points(self._description, 'collision'),
                               {side: ArmJog(self._description, side, kin)
                                for side in ('left', 'right')})
        return self._kin_tools

    def _live_joints(self):
        return {name: value[0] for name, value in self._joint.items()}

    def wrist_face(self, side, timeout=5.0, below_marker=0.056):
        """A point on this hand's cube face as its own wrist camera sees it (base_link).

        `below_marker` 0.056 gives the face centre (the markers sit that far above
        it, seminar_world.sdf); 0 gives the marker centre itself.
        """
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', f'{side}_grasp_marker_frame', rclpy.time.Time())
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if not 0 <= self.get_clock().now().nanoseconds * 1e-9 - stamp <= 0.4:
                rclpy.spin_once(self, timeout_sec=0.02)
                continue
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z - below_marker])
        return None

    def _plan_both(self, goals, label, retries=3):
        """Collision-checked joint plans for both arms, released together."""
        trajs = {}
        for side, joints in goals.items():
            for attempt in range(1, retries + 1):
                traj = self.plan_arm_joints(f'{side}_arm', joints,
                                            f'{label} {side} (attempt {attempt})')
                if traj is not None:
                    trajs[side] = traj
                    break
            else:
                return False
        results = self.move_arms_parallel(trajs, label, min_duration=4.0)
        return all(results.values())

    def _torso_and_arms_together(self, height, via, detection, jog, rise, seconds=4.0):
        """Carriages to `height` while both arms swing to `via`, then drop to
        the detection pose. The arms start after `arm_start_delay` of the
        carriage stroke and take as long as it does; the path was checked
        offline against the table and the cube at the dock (P-44)."""
        delay = float(self.get_parameter('arm_start_delay').value)
        if not self._wait_server(self.torso, 'torso_controller'):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['torso_left_carriage_joint', 'torso_right_carriage_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(height), float(height)]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = int(seconds)
        goal.trajectory.points = [point]
        torso_future = self.torso.send_goal_async(goal)
        # Arms: hold for delay*T, then a straight joint-space swing to `via`.
        steps = 30
        paths = {}
        for side in ('left', 'right'):
            names = jog[side].names
            start = {n: self._joint[n][0] for n in names}
            hold = max(1, int(round(steps * delay)))
            path = [dict(start) for _ in range(hold + 1)]
            path += [{n: start[n] + k / steps * (via[n] - start[n]) for n in names}
                     for k in range(1, steps + 1)]
            paths[side] = path
        self.get_logger().info(
            f'STEP3b carriages -> {height:.2f} m over {seconds:.0f} s, arms join after '
            f'{delay * 100:.0f}% through a point {rise * 100:.0f} cm above DETECTION_V4')
        arms_ok = self._straight_both(paths, 'STEP3b carriages and arms together', seconds / steps)
        handle = self._spin_until_done(torso_future)
        if handle is None or not handle.accepted:
            self.get_logger().error('STEP3b torso goal rejected')
            return False
        self._spin_until_done(handle.get_result_async())
        if not arms_ok:
            return False
        settle = time.monotonic() + 2.0
        while time.monotonic() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        at = [self._joint.get(n, (float('nan'), 0.0))[0]
              for n in ('torso_left_carriage_joint', 'torso_right_carriage_joint')]
        self.get_logger().info(
            f'STEP3b: CARRIAGE MEASURED left={at[0]:.4f} right={at[1]:.4f} m (commanded {height:.4f})')
        if max(abs(a - height) for a in at) > 0.01:
            return False
        # Last part: straight down onto the detection pose, orientation held.
        live = self._live_joints()
        down, q = {}, dict(live)
        for side in ('left', 'right'):
            q.update(detection[side])
        target = {side: jog[side].hand(q) for side in ('left', 'right')}
        for side in ('left', 'right'):
            R, t = target[side]
            walk, path = dict(live), [{n: live[n] for n in jog[side].names}]
            start = jog[side].hand(live)[1]
            n_steps = max(2, int(math.ceil(np.linalg.norm(t - start) / 0.005)))
            for k in range(1, n_steps + 1):
                walk.update(zip(jog[side].names, jog[side].solve(
                    walk, R, start + k / n_steps * (t - start), max_jump=0.3)))
                path.append({n: walk[n] for n in jog[side].names})
            down[side] = path
        return self._straight_both(
            down, 'STEP3c straight down onto DETECTION_V4',
            0.005 / float(self.get_parameter('descent_speed').value))

    def _state_valid(self, joints, label):
        """MoveIt's verdict on a whole-robot joint state: True, False, or None
        when the service is not there. The attached cube is not in its scene."""
        if getattr(self, '_validity', None) is None:
            self._validity = self.create_client(GetStateValidity, '/check_state_validity')
        if not self._validity.wait_for_service(timeout_sec=5.0):
            return None
        req = GetStateValidity.Request()
        req.group_name = 'both_arms'
        names = [n for n, v in joints.items() if isinstance(v, float)]
        req.robot_state.joint_state.name = names
        req.robot_state.joint_state.position = [float(joints[n]) for n in names]
        result = self._spin_until_done(self._validity.call_async(req))
        if result is None:
            return None
        if not result.valid:
            pairs = sorted({f'{c.contact_body_1} x {c.contact_body_2}' for c in result.contacts})
            self.get_logger().warn(f'{label}: MoveIt says self-collision - {pairs[:3]}')
        return bool(result.valid)

    def _pull_cube_in(self, grip, half):
        """Both hands straight towards the robot with the cube, then the elbows
        away from the torso column (hands held). Skipped - with the cube still
        held - whenever MoveIt or the torso clearance says no."""
        pull = float(self.get_parameter('pull_cube_in').value)
        limit = grip.x - half - (TORSO_FRONT_X + float(self.get_parameter('pull_torso_clearance').value))
        if pull > limit:
            self.get_logger().warn(
                f'STEP6b pull limited to {max(0.0, limit) * 100:.1f} cm: the cube would come '
                'too close to the torso')
            pull = max(0.0, limit)
        if pull < 0.005:
            return
        kin, hulls, jog = self._grasp_tools()
        live = self._live_joints()
        out = math.radians(float(self.get_parameter('pull_elbow_out_deg').value))
        paths, turn = {}, {}
        try:
            pulled = dict(live)
            for side in ('left', 'right'):
                R, t = jog[side].hand(live)
                steps = max(1, int(math.ceil(pull / 0.005)))
                walk, path = dict(live), [{n: live[n] for n in jog[side].names}]
                for k in range(1, steps + 1):
                    walk.update(zip(jog[side].names, jog[side].solve(
                        walk, R, t - np.array([k / steps * pull, 0.0, 0.0]), max_jump=0.3)))
                    path.append({n: walk[n] for n in jog[side].names})
                paths[side] = path
                pulled.update(path[-1])
            swung = dict(pulled)
            for side in ('left', 'right'):
                joints, _ = jog[side].elbow_swivel(swung, -out)
                swung.update(zip(jog[side].names, joints))
                start = {n: pulled[n] for n in jog[side].names}
                turn[side] = [{n: start[n] + k / 10 * (swung[n] - start[n]) for n in start}
                              for k in range(11)]
        except ValueError as exc:
            self.get_logger().warn(f'STEP6b cube not pulled in: {exc}')
            return
        # Checked where it happens: lifted, at the table, the table in the scene.
        # (The lowered state is checked in step 7, after backing away - checked
        # here it is 'in the table', which it will never be: run V4.)
        if self._state_valid(swung, 'STEP6b check, lifted at the table') is not True:
            self.get_logger().warn('STEP6b cube not pulled in: the end state is not '
                                   'collision-free for MoveIt')
            return
        speed = float(self.get_parameter('pull_speed').value)
        if not self._straight_both(paths, f'STEP6b pull the cube {pull * 100:.0f} cm in', 0.005 / speed):
            self.get_logger().warn('STEP6b pull did not complete')
            return
        self._straight_both(turn, 'STEP6b elbows away from the torso', 0.3)
        self.get_logger().info(
            f'STEP6b cube pulled {pull * 100:.0f} cm in: back face '
            f'{(grip.x - pull - half - TORSO_FRONT_X) * 100:.1f} cm in front of the torso')
        self.measure_width('cube pulled in')

    def _swing_elbows_out(self, angle):
        """Rotate both elbows away from the torso, holding the hands still.

        The hands do not move, so the cube - rigidly attached to the left wrist -
        is not disturbed. This is the same motion STEP6b makes after the pull,
        which is why the machinery is already here.
        """
        try:
            _, _, jog = self._grasp_tools()
            live = self._live_joints()
            swung = dict(live)
            paths = {}
            for side in ('left', 'right'):
                joints, _ = jog[side].elbow_swivel(swung, -angle)
                swung.update(zip(jog[side].names, joints))
                start = {n: live[n] for n in jog[side].names}
                paths[side] = [{n: start[n] + k / 10 * (swung[n] - start[n]) for n in start}
                               for k in range(11)]
        except Exception as exc:
            self.get_logger().warn(f'STEP7 elbows could not be swung out: {exc}')
            return False
        if self._state_valid(swung, 'STEP7 elbows further out') is False:
            self.get_logger().warn('STEP7 swinging the elbows further out collides; '
                                   'leaving them where they are')
            return False
        return bool(self._straight_both(paths, 'STEP7 elbows further from the torso', 0.3))

    def _publish_carried_cube(self):
        """Tell the footprint publisher the ground corners of the held cube.

        Measured, like everything else about the cube while it is held: the two
        wrist cameras give its centre and the direction across it, and the cube
        is square. Without this the outline Nav2 plans with ends at the hands
        and the cube hangs outside it (user, 16. 9.).
        """
        cube, half, normal = self._cube_in_hands()
        if cube is None:
            self.get_logger().error(
                'cannot work out where the held cube is; the footprint would cover '
                'the arms only and the cube would hang outside it')
            return False
        along = np.array([-normal[1], normal[0], 0.0])
        message = Polygon()
        for sx in (1.0, -1.0):
            for sy in (1.0, -1.0):
                corner = cube + sx * half * normal + sx * sy * half * along
                message.points.append(Point32(x=float(corner[0]),
                                              y=float(corner[1]), z=0.0))
        self.carried_pub.publish(message)
        self.get_logger().info(
            f'carried cube published to the footprint: {2 * half:.3f} m square at '
            f'({cube[0]:.2f}, {cube[1]:.2f})')
        return True

    def _publish_carried_nothing(self):
        self.carried_pub.publish(Polygon())
        self.get_logger().info('carrying nothing: footprint is the robot again')

    def _wake_the_localiser(self, why, timeout=2.0):
        """Make AMCL process the current scan even though the robot has not moved.

        AMCL gates its updates on distance travelled - update_min_d, 3 cm here -
        and that gate is fed by WHEEL odometry. So the estimator that cannot see
        a sideways slide is also the one deciding when the laser may correct it.
        Through a placement the robot moves in millimetres, and /amcl_pose was
        measured at 0.08 Hz: twelve messages in 152 seconds. map -> odom is then
        frozen, and anything read in `map` is just odometry with a stale constant
        in front of it.

        The gate is there for a good reason - a particle filter must not fold the
        same scan in twice, or it grows confident about a pose it has not
        re-measured - so this is not lowered globally. It is asked for by name,
        once, at the two moments where a stale correction actually costs
        something: reading the marker, and checking where the box landed.
        """
        if self._nomotion is None:
            return False
        if not self._nomotion.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(
                f'{why}: AMCL has no /request_nomotion_update; the pose stays as it is')
            return False
        future = self._nomotion.call_async(EmptySrv.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.done():
            self.get_logger().info(f'{why}: asked AMCL for a fresh fix without moving')
            return True
        self.get_logger().warn(f'{why}: AMCL did not answer the no-motion request')
        return False

    def _place_marker_in_base(self, timeout=8.0, max_age=0.6):
        """Centre of the place marker in base_link, from a FRESH detection."""
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', 'place_marker_frame', rclpy.time.Time())
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if not 0 <= age <= max_age:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z])
        return None

    def _cube_in_hands(self, timeout=6.0):
        """(centre, half width, face normal) of the cube, in base_link.

        While the cube is HELD this comes from the relation recorded when the
        hands closed on it (`_hold`), carried forward through the live transform
        of the tool tip. The wrist cameras are useless at that point: the pads
        sit on the markers, 2 cm away, and nothing decodes there - which is what
        aborted the place on 16. 9. (P-45). Before the grasp, when the hands are
        still 30 cm off, the cameras are exactly the right source and are used.
        """
        if self._hold is not None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', 'left_tool_tip', rclpy.time.Time())
            except Exception as exc:
                self.get_logger().error(f'no tool tip transform for the held cube: {exc}')
                return None, None, None
            rotation = quat_to_matrix(tf.transform.rotation)
            origin = np.array([tf.transform.translation.x,
                               tf.transform.translation.y,
                               tf.transform.translation.z])
            centre = rotation @ self._hold['offset'] + origin
            normal = rotation @ self._hold['normal']
            return centre, self._hold['half'], normal
        faces = {side: self.wrist_face(side, timeout=timeout, below_marker=0.0)
                 for side in ('left', 'right')}
        if any(f is None for f in faces.values()):
            missing = [s for s, f in faces.items() if f is None]
            self.get_logger().error(
                f'wrist cameras cannot see the cube they are holding: {missing}')
            return None, None, None
        across = faces['left'] - faces['right']
        span = float(np.linalg.norm(across[:2]))
        if not 0.24 <= span <= 0.36:
            self.get_logger().error(
                f'faces {span:.3f} m apart - that is not the 0.30 m cube')
            return None, None, None
        centre = (faces['left'] + faces['right']) / 2.0
        # The markers sit 0.056 m above the face centres (seminar_world.sdf).
        centre[2] -= 0.056
        normal = np.array([across[0], across[1], 0.0]) / span
        return centre, span / 2.0, normal

    def _move_cube_by(self, delta, label):
        """Carry the held cube `delta` (base_link metres) with both hands at once.

        A straight line to a computed point, orientation held - this is how the
        arms put the cube exactly where it has to go once the base has stopped,
        forward AND across (user, 16. 9.). MoveIt checks the end state, so this
        either makes the move or says it cannot.
        """
        delta = np.asarray(delta, dtype=float)
        distance = float(np.linalg.norm(delta))
        if distance < 0.005:
            return True
        kin, hulls, jog = self._grasp_tools()
        live = self._live_joints()
        paths, moved = {}, dict(live)
        try:
            for side in ('left', 'right'):
                R, t = jog[side].hand(live)
                steps = max(1, int(math.ceil(distance / 0.005)))
                walk, path = dict(live), [{n: live[n] for n in jog[side].names}]
                for k in range(1, steps + 1):
                    walk.update(zip(jog[side].names, jog[side].solve(
                        walk, R, t + k / steps * delta, max_jump=0.3)))
                    path.append({n: walk[n] for n in jog[side].names})
                paths[side] = path
                moved.update(path[-1])
        except ValueError as exc:
            self.get_logger().error(f'{label}: the arms cannot reach there: {exc}')
            return False
        if self._state_valid(moved, f'{label}: MoveIt check') is False:
            self.get_logger().error(f'{label}: that move ends in a collision')
            return False
        speed = float(self.get_parameter('pull_speed').value)
        return self._straight_both(paths, label, 0.005 / speed)

    def _to_odom(self, point_xy):
        """A base_link (x, y) as an odom (x, y), or None."""
        here = self._odom_xy()
        yaw = self._odom_yaw()
        if here is None or yaw is None:
            return None
        return (here[0] + point_xy[0] * math.cos(yaw) - point_xy[1] * math.sin(yaw),
                here[1] + point_xy[0] * math.sin(yaw) + point_xy[1] * math.cos(yaw))

    def _marker_in_base(self, mark_odom, height):
        """Where the remembered marker is now, in base_link, from odometry.

        The marker is read ONCE, from the dock, and carried forward from there.
        Close in it cannot be read at all - after the base has taken its 24 cm
        the plate is under the robot's own nose - and a stale reading looks
        exactly like a fresh one. On 16. 9. that cost the run: the marker was
        still remembered at 0.92 m while the cube had moved 0.24 m closer, so the
        arms were asked to reach 43 cm and refused, correctly, as unreachable.
        """
        here = self._odom_xy()
        yaw = self._odom_yaw()
        if here is None or yaw is None:
            return None
        dx, dy = mark_odom[0] - here[0], mark_odom[1] - here[1]
        return np.array([dx * math.cos(yaw) + dy * math.sin(yaw),
                         -dx * math.sin(yaw) + dy * math.cos(yaw),
                         float(height)])

    def _map_pose(self, timeout=1.0):
        """(x, y, yaw) of base_link in `map`, or None.

        This is the localiser's answer - wheel odometry with the laser's
        correction folded in - as opposed to `_odom_xy`, which is the wheels
        alone.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception:
            return None
        tr, q = tf.transform.translation, tf.transform.rotation
        return (tr.x, tr.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _to_map(self, point_xy):
        """A base_link (x, y) as a `map` (x, y), or None."""
        pose = self._map_pose()
        if pose is None:
            return None
        x, y, yaw = pose
        return (x + point_xy[0] * math.cos(yaw) - point_xy[1] * math.sin(yaw),
                y + point_xy[0] * math.sin(yaw) + point_xy[1] * math.cos(yaw))

    def _marker_in_base_from_map(self, mark_map, height):
        """The remembered marker in base_link again, but carried in `map`."""
        pose = self._map_pose()
        if pose is None or mark_map is None:
            return None
        x, y, yaw = pose
        dx, dy = mark_map[0] - x, mark_map[1] - y
        return np.array([dx * math.cos(yaw) + dy * math.sin(yaw),
                         -dx * math.sin(yaw) + dy * math.cos(yaw),
                         float(height)])

    def _place_on_marker(self):
        """Put the cube down with the centre of its BOTTOM face on the marker.

        The mirror of the pick: the marker is measured the way the cube was, the
        carriages set the height, the base closes the last stretch and the
        carriages lower the cube onto the table. Nothing is taken from the world
        file - the marker is where the camera says it is, and if the camera
        cannot find it the cube is not put down anywhere (D-12).
        """
        clearance = float(self.get_parameter('place_clearance').value)
        touch = float(self.get_parameter('place_touch').value)
        lower_speed = float(self.get_parameter('place_lower_speed').value)

        # P1. Read the marker from the dock: far enough not to foreshorten it
        #     (P-19) and with nothing of the robot over it.
        self._report_mission_step(7, 8, 'Detekcija markera za odlaganje',
                                  'Glavna kamera traži ciljni marker na stolu (ID 3), robot i ruke se pozicioniraju...')
        # One look is not enough to call it: run 6 of the 18 Sep series docked
        # 4.7 cm and +1.4 deg from the pose, which is what a GOOD arrival looks
        # like here - the successful run beside it managed 4.5 cm - and still
        # came up empty at pitch 0.65, so the mission threw away a cube it was
        # holding correctly. The difference between seeing the plate and not is
        # a degree of head tilt, not a fault, so the head sweeps a little before
        # the run is allowed to fail. Nothing is relaxed: the marker still has
        # to be SEEN, freshly, or the cube stays in the hands (D-12).
        self._wake_the_localiser('PLACE before reading the marker')
        mark = None
        # A grid, not three guesses. 0.65 is the pitch the mission has always
        # used and it is first because it is the one that works; the rest widen
        # the search the way a person would - further down, further up, then
        # left and right - instead of stopping at the first thing that failed.
        #
        # Run 10 of the centring series is why the yaw column exists: it found
        # nothing at any pitch because the cube sat 45 cm too high in front of
        # the camera (fixed at its source in STEP7), and a head that can only
        # look straight ahead has no way around an obstacle in the middle of its
        # view. Each look costs about 16 s and the alternative is throwing away
        # a cube the robot is holding correctly.
        for yaw, pitch in ((0.0, 0.65), (0.0, 0.55), (0.0, 0.75), (0.0, 0.85),
                           (-0.35, 0.65), (0.35, 0.65),
                           (-0.35, 0.75), (0.35, 0.75)):
            self.look_down(pitch, f'PLACE look at the table '
                                  f'(pitch {pitch:.2f}, yaw {yaw:.2f})', yaw=yaw)
            mark = self._place_marker_in_base()
            if mark is not None:
                break
            self.get_logger().warn(
                f'PLACE: no marker at pitch {pitch:.2f}, yaw {yaw:.2f}; '
                'moving the head and looking again')
        if mark is not None:
            self.look_down(0.65, 'PLACE head back to centre')
        if mark is None:
            self._fail('the place marker was never seen at any head tilt; '
                       'the cube stays in the hands')
            return False
        self.get_logger().info(
            f'PLACE marker at base_link ({mark[0]:.3f}, {mark[1]:.3f}, {mark[2]:.3f})')
        # Remember it in odom too: once the cube is on it the marker is covered,
        # and the check at the end still has to know where it was.
        mark_odom = self._to_odom(mark)
        if mark_odom is None:
            self._fail('no odometry to remember where the marker is')
            return False
        # The same reading kept a second time, in `map`. Only the final check
        # uses it, and only for the sideways half - see the verification below.
        mark_map = self._to_map(mark)
        if mark_map is None:
            self.get_logger().warn(
                'PLACE: no map pose to remember the marker in; the check at the '
                'end will be odometry-only and will read low sideways')
        # The table top does not move, so its height is kept from this one
        # reading; everything after this works from the odometry copy.
        mark_height = float(mark[2])
        # The table under the marker, so MoveIt knows what the arms are over.
        self.publish_collision_scene(
            Point(x=float(mark[0]), y=float(mark[1]), z=float(mark[2]) + 0.15),
            table_top_z=float(mark[2]))

        # P2. Stopped at the table, the robot is no longer driving: first it
        #     lifts the cube to where the user wants the hands - the marker,
        #     plus `place_clearance`, plus half a cube - so the cube rides
        #     clear above the table top before the base moves at all.
        cube, half, _ = self._cube_in_hands()
        if cube is None:
            self._fail('cannot work out where the held cube is')
            return False
        carriage = self._joint.get('torso_left_carriage_joint', (0.0, 0.0))[0]
        hands_want = mark[2] + clearance + half
        want = max(0.06, min(0.64, carriage + hands_want - cube[2]))
        self.get_logger().info(
            f'PLACE marker at {mark[2]:.3f} m -> hands to {hands_want:.3f} m '
            f'(+{clearance * 100:.0f} cm +{half * 100:.0f} cm half a cube); '
            f'carriages {carriage:.3f} -> {want:.3f} m')
        if not self._torso_to(want, 'PLACE carriages lift the cube over the table'):
            self._fail('carriages did not reach the height that clears the table')
            return False

        # P3. The BASE goes first, with the cube still tucked in against the
        #     robot (user, 16. 9.): the arms are up in the air and block nothing,
        #     so there is no reason to reach out before the base has taken every
        #     centimetre it can. The arms then cover the whole remaining gap in
        #     one move - the 15 cm the cube was pulled in with, plus whatever the
        #     base could not close.
        #
        # P4. The base closes in on the table by what the ROBOT can do, not
        #     by what the marker asks for (user, 16. 9.): forward as far as it
        #     still fits, and no further. At the dock the base's real front
        #     (0.41 m ahead of base_link) sits about 0.26 m from the table edge,
        #     because the dock pose is computed with the conservative 0.52 m
        #     half-length - `place_max_advance` is that slack, less a margin.
        #     Sideways is a different matter: nothing blocks it, so the cube is
        #     still centred across the marker.
        limit = float(self.get_parameter('place_max_advance').value)
        mark = self._marker_in_base(mark_odom, mark_height)
        cube, half, _ = self._cube_in_hands()
        if mark is None or cube is None:
            self._fail('cannot work out where the held cube is before lowering')
            return False
        need = float(mark[0] - cube[0])
        advance = max(0.0, min(need, limit))
        self.get_logger().info(
            f'PLACE step 4: the cube is {need * 100:+.1f} cm short of the marker; the '
            f'base closes in {advance * 100:.1f} cm of the {limit * 100:.0f} cm it may')
        if advance >= 0.01 and self.drive_distance(
                advance, speed=float(self.get_parameter('place_approach_speed').value)) is None:
            self._fail('the approach to the table did not track odometry')
            return False

        # 5. THE ROBOT lines up with the marker: a strafe puts the marker on the
        #    robot's own centreline, so the arms are left as little to do as
        #    possible (user, 16. 9.).
        mark = self._marker_in_base(mark_odom, mark_height)
        if mark is None:
            self._fail('lost the odometry copy of the marker')
            return False
        if abs(float(mark[1])) >= 0.01:
            self.get_logger().info(
                f'PLACE step 5: the marker is {float(mark[1]) * 100:+.1f} cm to the side; '
                'strafing the robot onto it')
            if self.strafe_distance(float(mark[1])) is None:
                self._fail('the strafe onto the marker did not track odometry')
                return False

        # 6. THE ARMS put the cube on the marker centre - across as well as
        #    forward, in one straight move to a computed point.
        mark = self._marker_in_base(mark_odom, mark_height)
        cube, half, _ = self._cube_in_hands()
        if mark is None or cube is None:
            self._fail('cannot work out where the held cube is before lowering')
            return False
        delta = np.array([float(mark[0] - cube[0]), float(mark[1] - cube[1]), 0.0])
        self.get_logger().info(
            f'PLACE step 6: the arms carry the cube {delta[0] * 100:+.1f} cm forward and '
            f'{delta[1] * 100:+.1f} cm across, onto the marker centre')
        if float(np.linalg.norm(delta)) >= 0.01 and not self._move_cube_by(
                delta, 'PLACE step 6: cube onto the marker centre'):
            self.get_logger().warn(
                'PLACE the arms could not take the cube all the way onto the centre')
        self._publish_carried_cube()
        mark = self._marker_in_base(mark_odom, mark_height)
        cube, half, _ = self._cube_in_hands()
        if mark is not None and cube is not None:
            self.get_logger().info(
                f'PLACE the cube sits {(mark[0] - cube[0]) * 100:+.1f} cm / '
                f'{(mark[1] - cube[1]) * 100:+.1f} cm from the marker centre')

        # P5. Down until the cube rests on the table: slowly, and stopping if the
        #     cube leaves the pads on the way.
        self._report_mission_step(8, 8, 'Odlaganje kocke na marker',
                                  'Spuštanje kocke na stol, otpuštanje hvata (detach) i povlačenje robota...')
        mark = self._marker_in_base(mark_odom, mark_height)
        cube, half, normal = self._cube_in_hands()
        if mark is None or cube is None:
            self._fail('cannot measure the cube to lower it')
            return False
        carriage = self._joint.get('torso_left_carriage_joint', (0.0, 0.0))[0]
        target = max(0.06, min(0.64, carriage + (mark[2] + touch) - (cube[2] - half)))
        self.get_logger().info(
            f'PLACE step 7: hands down to {mark[2] + half:.3f} m (marker + half a cube), '
            f'carriages {carriage:.3f} -> {target:.3f} m at {lower_speed * 100:.0f} cm/s')
        # No contact guard on the way down. It used to watch the fingertip
        # sensors, but the cube is carried by the rigid attach with the pads only
        # touching (`touch_depth` 2 mm), so there is nothing for them to report:
        # on 16. 9. the guard fired in the same millisecond the descent started
        # and the cube never moved. What decides this is the MEASURED height
        # afterwards, which is the check right below - and it did its job, it
        # refused to let go 20 cm in the air.
        # Fast through the air, slow for the last band above the table.
        band = float(self.get_parameter('place_slow_band').value)
        fast = float(self.get_parameter('place_fast_lower_speed').value)
        above = target + band
        if carriage - above > 0.01:
            seconds = max(1.0, (carriage - above) / max(fast, 0.001))
            if not self.move_torso(above, 'PLACE lower the cube to just above the marker',
                                   secs=max(1, int(round(seconds)))):
                self._fail('the carriages did not lower the cube')
                return False
            carriage = above
        seconds = max(2.0, abs(target - carriage) / max(lower_speed, 0.001))
        if not self.move_torso(target, 'PLACE lower the cube onto the marker',
                               secs=int(round(seconds))):
            self._fail('the carriages did not lower the cube')
            return False
        settle = time.monotonic() + 1.2
        while time.monotonic() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)

        # P5b. DOWN FIRST, LET GO SECOND (user, 16. 9.). The carriages reporting
        #      that they arrived is not the cube being down - measure it, and if
        #      it is still in the air, keep holding it rather than drop it.
        cube, half, normal = self._cube_in_hands()
        if cube is None:
            self._fail('cannot check whether the cube is down; not letting go of it')
            return False
        resting = float(cube[2] - half - mark[2])
        self.get_logger().info(
            f'PLACE the cube bottom is {resting * 1000:+.0f} mm from the table top')
        if resting > 0.02:
            self._fail(f'the cube is still {resting * 100:.1f} cm above the table; '
                       'not letting go of it in mid-air')
            return False

        # 8. LET GO of the hard link, then ease the pads apart before any arm
        #    moves - a hand that leaves while still pressing drags the cube off
        #    the marker, and a hand pulled off a still-attached cube explodes the
        #    solver (P-17).
        if not self._attach_box(False):
            self.get_logger().error(
                'PLACE step 8: box could not be detached from left wrist! '
                'Aborting before arm motion to avoid dragging box.')
            return False
        self._hold = None
        self._publish_carried_nothing()
        settle = time.monotonic() + 1.0
        while time.monotonic() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        for side, sign in (('left', 1.0), ('right', -1.0)):
            self.retreat_linear(f'{side}_arm', f'{side}_end_effector_link',
                                (sign * normal[0], sign * normal[1]), 0.03,
                                f'PLACE step 8: take the {side} pad off the cube')

        # 9. DETECTION posture: hands wide and well clear of what they just put
        #    down. MoveIt plans it against the table, so if it will not go there
        #    from this close the run says so and carries on backing away.
        if not move_to_posture(self, 'DETECTION_V4', label='PLACE step 9: detection posture'):
            self.get_logger().warn(
                'PLACE step 9: could not reach the detection posture at the table')

        # 10. BACK AWAY from the table, empty.
        if self.drive_distance(-0.50, speed=float(
                self.get_parameter('back_off_speed').value)) is None:
            self.get_logger().warn('PLACE step 10: backing away did not track odometry')
        drop = PlanningScene()
        drop.is_diff = True
        for object_id in ['pick_table'] + [f'pick_table_leg_{i}' for i in range(4)]:
            co = CollisionObject()
            co.header.frame_id = 'base_link'
            co.id = object_id
            co.operation = CollisionObject.REMOVE
            drop.world.collision_objects.append(co)
        self.scene_pub.publish(drop)

        # 11. DRIVE posture, and the carriages back to travel height.
        if move_to_posture(self, ARM_DRIVE, label='PLACE step 11: drive posture'):
            self.arm_posture_pub.publish(String(data=ARM_DRIVE))
        self._torso_to(DRIVE_CARRIAGE, 'PLACE carriages back to the drive height')

        # P8. PROOF (D-12): where did it actually land? The head camera measures
        #     the cube again - its front marker is visible once we have backed
        #     off - and that is compared with where the marker was, in odom, the
        #     one frame that does not move when the robot does.
        self._wake_the_localiser('PLACE before the landing check')
        self.look_down(0.65, 'PLACE look at what we left behind')
        face = self.confirm_box(timeout=10.0)
        if face is None:
            self.get_logger().warn(
                'PLACE: the cube is down but the camera cannot see it again; '
                'landing position UNVERIFIED')
            self._report_mission_step(8, 8, 'MISIJA ZAVRŠENA',
                                      'Kocka je odložena, ali pozicija nije ponovno verificirana kamerom.',
                                      state='success')
            return True
        meas = self.measure_box(Point(x=face.x, y=face.y, z=face.z))
        if meas is None:
            self.get_logger().warn(
                'PLACE: the cube is down but depth could not measure it; '
                'landing position UNVERIFIED')
            self._report_mission_step(8, 8, 'MISIJA ZAVRŠENA',
                                      'Kocka je odložena, ali dubinski senzor nije izmjerio centar.',
                                      state='success')
            return True
        # Each frame is asked only for the axis it is good at.
        #
        # This check used to be odometry alone, and it read 15.4 mm low, every
        # single time: over the 13 successful runs of the 18 Sep series the
        # robot reported 2-7 mm while the box was recorded 16.9-21.6 mm from the
        # marker, and the gap had a standard deviation of 0.2 mm. A constant
        # that tight is not noise, it is a frame being wrong in one direction.
        #
        # Measured between remembering the marker and this check, against
        # ground truth, over those 13 runs:
        #
        #                       forwards            sideways
        #     odometry          0.8 mm (max 2.0)    19.5 mm (max 22.0)
        #     map (AMCL)       65.5 mm (max 72.2)    1.3 mm (max  7.8)
        #
        # Wheels measure what they ROLL. Forwards they roll, so odometry is good
        # to under a millimetre. Sideways this base slides - that is how mecanum
        # works at all, mu2 = 0.20 by design (P-09) - and a slide leaves the
        # wheels no rotation to count, so it is invisible to them. The laser has
        # the opposite shape: it sees the side walls squarely, which pins the
        # sideways estimate, while along the room there is little to bite on and
        # AMCL wanders 7 cm.
        #
        # So: forwards from odometry, sideways from the map. This changes no
        # motion whatsoever - only what the robot then says about the result.
        # The placement itself is still ~18 mm out, for the same sliding reason,
        # and neither frame can see that at the moment it happens (both are
        # ~17 mm out in the placement window). What this fixes is the honesty of
        # the report, not the accuracy of the placement (D-12, D-24).
        cube_base = (float(meas[0].x), float(meas[0].y))
        m_odom = self._marker_in_base(mark_odom, mark_height)
        m_map = self._marker_in_base_from_map(mark_map, mark_height)
        if m_odom is None:
            self.get_logger().warn('PLACE: no odometry for the landing check')
            self._report_mission_step(8, 8, 'MISIJA ZAVRŠENA',
                                      'Kocka je odložena na stol (nedostaje odometrija za provjeru).',
                                      state='success')
            return True
        dx = cube_base[0] - float(m_odom[0])
        if m_map is not None:
            dy = cube_base[1] - float(m_map[1])
        else:
            dy = cube_base[1] - float(m_odom[1])
            self.get_logger().warn(
                'PLACE: no map pose for the landing check; the sideways figure '
                'is odometry-only and reads about 15 mm low')
        # Both readings, so the difference stays visible in the log.
        self.get_logger().info(
            f'PLACE landing check: sideways from map {dy * 1000:+.1f} mm, '
            f'from odometry {(cube_base[1] - float(m_odom[1])) * 1000:+.1f} mm')
        err_mm = math.hypot(dx, dy) * 1000
        self.get_logger().info(
            f'PLACE VERIFIED: the centre of the cube is {err_mm:.0f} mm '
            f'from the marker centre (dx {dx * 1000:+.0f}, dy {dy * 1000:+.0f} mm)')
        self._report_mission_step(8, 8, 'MISIJA USPJEŠNO ZAVRŠENA',
                                  f'Kocka je točno na markeru! Odstupanje od centra: {err_mm:.0f} mm (dx: {dx * 1000:+.0f} mm, dy: {dy * 1000:+.0f} mm).',
                                  state='success')
        return True

    def _posture_goals(self, name):
        """{side: {joint: value}} of a named posture, each continuous-joint value
        taken as the equivalent nearest the arm's live angle (no extra turns)."""
        goals = {}
        for side, joints in POSTURES[name].items():
            goals[side] = {}
            for j, target in joints.items():
                joint = f'{side}_joint_{j}'
                live = self._joint.get(joint, (target, 0.0))[0]
                if j in (1, 3, 5, 7):
                    target = live + math.atan2(math.sin(target - live), math.cos(target - live))
                goals[side][joint] = target
        return goals

    def _interpolate_both(self, goals, label, seconds=6.0, steps=40):
        """Both arms straight through joint space to `goals`, released together.

        For a move between two postures already known to be collision-free
        when MoveIt will not plan it (the goal is a hand's breadth from the cube).
        """
        paths = {}
        for side, goal in goals.items():
            start = {n: self._joint[n][0] for n in goal}
            paths[side] = [{n: start[n] + k / steps * (goal[n] - start[n]) for n in goal}
                           for k in range(steps + 1)]
        return self._straight_both(paths, label, seconds / steps)

    def _straight_both(self, paths, label, step_time):
        """Send precomputed joint paths (lists of joint dicts) to both arms at once."""
        trajs = {}
        for side, path in paths.items():
            traj = JointTrajectory()
            traj.joint_names = [f'{side}_joint_{j}' for j in range(1, 8)]
            for k, joints in enumerate(path):
                point = JointTrajectoryPoint()
                point.positions = [float(joints[n]) for n in traj.joint_names]
                t = k * step_time
                point.time_from_start.sec = int(t)
                point.time_from_start.nanosec = int((t - int(t)) * 1e9)
                traj.points.append(point)
            trajs[side] = traj
        results = self.move_arms_parallel(trajs, label)
        return all(results.values())

    def run(self):
        # The DetachableJoint starts attached; release it so the box sits free on
        # the table until we deliberately grasp it.
        self._attach_box(False)

        # Get the wrists above tabletop height BEFORE driving anywhere. The
        # robot spawns with the carriages near their lower stop, which puts the
        # hands below the 0.75 m table, and the approach in step 2 drives right
        # up to it. Step 4d refines this to the measured cube height once the
        # cube has actually been seen; this is only about clearing the table.
        travel = float(self.get_parameter('carriage_reference_height').value)
        if not self.move_torso(travel, 'STEP0 carriages to travel height', secs=3):
            return self._fail('carriages did not accept the travel height')
        settle = time.monotonic() + 1.2
        while time.monotonic() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        at = [self._joint.get(n, (float('nan'), 0.0))[0]
              for n in ('torso_left_carriage_joint', 'torso_right_carriage_joint')]
        self.get_logger().info(
            f'CARRIAGE TRAVEL MEASURED left={at[0]:.4f} right={at[1]:.4f} m '
            f'(commanded {travel:.4f})')
        if max(abs(a - travel) for a in at) > 0.01:
            return self._fail(
                f'carriages did not reach the travel height ({at} vs {travel:.3f})')

        if bool(self.get_parameter('mission').value):
            # A. The arms leave the spawn posture (ARM_ZERO, 2.28 m wide) before
            #    the base moves at all. In the mission the robot spawns with them
            #    spread, exactly as the user asked to see it.
            self._report_mission_step(1, 8, 'Priprema ruku (DRIVE_V4)',
                                      'Sklapanje ruku iz početnog položaja u transportnu pozu za vožnju.')
            if not move_to_posture(self, ARM_DRIVE, label='mission: drive posture'):
                return self._fail('arms did not reach the drive posture')
            self.arm_posture_pub.publish(String(data=ARM_DRIVE))
            # B. Nothing drives until the user says so.
            self._report_mission_step(2, 8, 'Čekanje naredbe korisnika',
                                      'Ruke su sklopljene. Pritisnite "MISIJA: po kutiju" u GUI panelu.',
                                      state='waiting')
            room = self._wait_for_start(str(self.get_parameter('pick_room').value))
            # C. To the table in that room: approach pose, then the dock pose.
            self._report_mission_step(3, 8, 'Navigacija do sobe s kockom',
                                      f'Autonomna vožnja kroz vrata do stola ({room}:dock)...')
            if not self._goto(f'{room}:dock'):
                return self._fail(f'room_navigator did not reach {room}:dock')

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
        self._report_mission_step(4, 8, 'Percepcija i prilaz kocki',
                                  'Traženje ArUco markera kamerom na glavi i vizualni prilaz stolu...')
        found = self.scan_for_marker()
        if found is None:
            return self._fail('scan: marker not found')
        _, _, scan_pan = found

        # 2. APPROACH in the drive posture to the dock distance. DRIVE_V4 is
        #    tucked (0.27 m reach), so standing at the table is safe.
        #    visual_approach servos on the MARKER, on the near face - half a cube
        #    (0.15 m) in front of the centre the range refers to.
        stand = float(self.get_parameter('detect_stand_range').value)
        if not self.visual_approach(scan_pan=scan_pan, target_x=stand - 0.15):
            return self._fail('visual approach to box failed')
        self.look_down(0.65, 'STEP3 look at box')
        face = self.confirm_box()
        if face is None:
            return self._fail('box not seen at the table')

        # 3. LOCATE the cube with the head camera: marker plus depth.
        seed = Point(x=face.x, y=face.y, z=face.z)
        meas = self.measure_box(seed)
        if meas is None:
            return self._fail('depth measurement of the box failed; grasp not confirmed')
        center, length, height, u = meas
        if abs(center.x - face.x) > 0.15 or abs(center.y - face.y) > 0.15:
            return self._fail('depth centre disagrees with the marker; grasp not confirmed')
        vt = self.marker_tangent()
        if vt is not None:
            u = vt

        # 3b/3c. CARRIAGES UP and ARMS INTO DETECTION_V4 (a little wider).
        self._report_mission_step(5, 8, 'Dvoručni hvat i podizanje kocke',
                                  f'Kocka izmjerena na ({center.x:.2f}, {center.y:.2f}) m. Priprema šaka za zahvat...')
        try:
            kin, hulls, jog = self._grasp_tools()
        except RuntimeError as exc:
            return self._fail(str(exc))
        self.publish_collision_scene(Point(x=center.x, y=center.y, z=face.z),
                                     table_top_z=face.z - 0.15)
        lift_to = float(self.get_parameter('detection_carriage_height').value)
        widen = float(self.get_parameter('detection_widen').value)
        rise = float(self.get_parameter('detection_via_rise').value)
        goals = self._posture_goals('DETECTION_V4')
        final = dict(self._live_joints())
        for side_goal in goals.values():
            final.update(side_goal)
        final['torso_left_carriage_joint'] = final['torso_right_carriage_joint'] = lift_to
        via = dict(final)
        try:
            for side, sign in (('left', 1.0), ('right', -1.0)):
                R, t = jog[side].hand(final)
                final.update(zip(jog[side].names, jog[side].solve(
                    final, R, t + np.array([0.0, sign * widen, 0.0]), max_jump=0.5)))
                R, t = jog[side].hand(final)
                via.update(zip(jog[side].names, jog[side].solve(
                    final, R, t + np.array([0.0, 0.0, rise]), max_jump=0.8)))
        except ValueError as exc:
            return self._fail(f'no widened detection pose: {exc}')
        detection = {side: {n: final[n] for n in jog[side].names} for side in ('left', 'right')}
        if center.x >= float(self.get_parameter('together_min_range').value):
            if not self._torso_and_arms_together(lift_to, via, detection, jog, rise):
                return self._fail('carriages and arms did not reach DETECTION_V4 together')
        else:
            self.get_logger().info(
                f'STEP3b cube only {center.x:.2f} m ahead: carriages first, then MoveIt')
            if not self._torso_to(lift_to, 'STEP3b carriages to the detection height'):
                return self._fail('carriages did not reach the detection height')
            if not self._plan_both(detection, 'STEP3c DETECTION_V4 (widened)'):
                return self._fail('arms did not reach DETECTION_V4')

        # 4. DRIVE IN until the cube is where the V4 postures expect it, then
        #    centre by strafing (>= 11.8 cm off the table, 15 cm off the cube).
        cube_range = float(self.get_parameter('detection_cube_range').value)
        advance = center.x - cube_range
        if advance > 0.02:
            driven = self.drive_distance(advance, speed=0.08)
            if driven is None:
                return self._fail('drive-in did not track odometry')
            center = Point(x=center.x - driven, y=center.y, z=center.z)
            self.get_logger().info(f'STEP4 drove in {driven:.3f} m in DETECTION_V4')
        # Centre on a FRESH reading of the marker, not on where the depth camera
        # put the cube before the drive-in. On 16. 9. the robot strafed away from
        # a cube it was already centred on - the stale offset was the only reason
        # it moved - and the hands then had to make uneven moves to reach it.
        fresh = self.marker_in_base(max_age=0.6)
        if fresh is None:
            self.get_logger().info(
                'STEP4 no fresh marker to centre on; leaving the base where it is')
        else:
            offset = float(fresh[1])
            if abs(offset - center.y) > 0.05:
                self.get_logger().warn(
                    f'STEP4 the marker says the cube is {offset * 100:+.1f} cm to the side '
                    f'but the depth measurement said {center.y * 100:+.1f} cm; '
                    'centring on the fresh marker')
            if abs(offset) <= 0.02:
                self.get_logger().info(
                    f'STEP4 cube already centred ({offset * 100:+.1f} cm); no strafe')
                center = Point(x=center.x, y=offset, z=center.z)
            elif self.strafe_distance(offset) is None:
                self.get_logger().warn('STEP4 strafe did not track odometry; '
                                       'the wrist cameras still set the targets')
            else:
                after = self.marker_in_base(max_age=0.6)
                residual = center.y if after is None else float(after[1])
                self.get_logger().info(
                    f'STEP4 strafed {offset * 100:+.1f} cm to centre; '
                    f'cube now {residual * 100:+.1f} cm off centre')
                center = Point(x=center.x, y=residual, z=center.z)

        # 5a. The WRIST CAMERAS read the side markers. Their centres ARE the
        #     targets for the tool tips (user, 16. 9.).
        targets = {side: self.wrist_face(side, below_marker=0.0) for side in ('left', 'right')}
        if any(t is None for t in targets.values()):
            missing = [side for side, t in targets.items() if t is None]
            return self._fail(f'wrist camera did not read the side marker: {missing}')
        across = targets['left'] - targets['right']
        span = float(np.linalg.norm(across[:2]))
        normal = np.array([across[0], across[1], 0.0]) / span
        yaw = math.atan2(-normal[0], normal[1])
        middle = (targets['left'] + targets['right']) / 2.0
        grip = Point(x=float(middle[0]), y=float(middle[1]), z=float(middle[2]) - 0.056)
        u = (float(normal[0]), float(normal[1]))
        self.get_logger().info(
            f'STEP5a WRIST CAMERAS: markers {span:.3f} m apart, cube centre '
            f'({grip.x:.3f}, {grip.y:.3f}, {grip.z:.3f}), turned {math.degrees(yaw):+.1f} deg')
        if not 0.24 <= span <= 0.36:
            return self._fail(f'markers {span:.3f} m apart - not the 0.30 m cube')
        self.publish_collision_scene(grip, table_top_z=grip.z - 0.15, yaw=yaw)

        # 5b. ARMS TO GRASP_V4. MoveIt first; its fingers end 1.4 cm off the
        #     faces, so if the planner will not go that close, both arms move
        #     straight through joint space - both postures are collision-free.
        grasp_goals = self._posture_goals('GRASP_V4')
        if not self._plan_both(grasp_goals, 'STEP5b GRASP_V4', retries=2):
            self.get_logger().warn('STEP5b: MoveIt would not plan GRASP_V4; interpolating joints')
            if not self._interpolate_both(grasp_goals, 'STEP5b GRASP_V4 (joint interpolation)'):
                return self._fail('arms did not reach GRASP_V4')
        for side in ('left', 'right'):
            self._wait_settle(f'{side}_end_effector_link')

        # 5c. TOOL TIPS TO THE MARKER CENTRES: a straight line from GRASP_V4 with
        #     its tool orientation kept (turned to the cube), both hands at once,
        #     only `touch_depth` into the face - no squeeze.
        live = self._live_joints()
        reference = dict(live)
        for side, joints in POSTURES['GRASP_V4'].items():
            reference.update({f'{side}_joint_{j}': live.get(f'{side}_joint_{j}', v)
                              for j, v in joints.items()})
            reference[f'{side}_robotiq_85_left_knuckle_joint'] = GRIPPER_CLOSED
        base = kin.place(live)['base_link'][1]
        touch = float(self.get_parameter('touch_depth').value)
        speed = float(self.get_parameter('grasp_approach_speed').value)
        paths, tip_goals = {}, {}
        try:
            for side, sign in (('left', 1.0), ('right', -1.0)):
                R, hand = pad_contact_pose(kin, hulls, reference, side,
                                           base + targets[side], sign * normal, touch, yaw=yaw)
                start = jog[side].hand(reference)[1]
                steps = max(2, int(math.ceil(np.linalg.norm(hand - start) / 0.005)))
                q, path = dict(reference), [{n: reference[n] for n in jog[side].names}]
                for k in range(1, steps + 1):
                    q.update(zip(jog[side].names,
                                 jog[side].solve(q, R, start + k / steps * (hand - start), max_jump=0.3)))
                    path.append({n: q[n] for n in jog[side].names})
                paths[side] = path
                tip_goals[side] = hand + 0.127 * R[:, 2] - base
                moved = max(abs(math.degrees(q[n] - reference[n])) for n in jog[side].names)
                self.get_logger().info(
                    f'STEP5c {side}: tool tip goes {np.linalg.norm(hand - start) * 100:.1f} cm to the '
                    f'marker centre; largest joint change from GRASP_V4 {moved:.1f} deg')
        except ValueError as exc:
            return self._fail(f'tool tips cannot reach the marker centres from GRASP_V4: {exc}')
        if not self._straight_both(paths, 'STEP5c tool tips to the marker centres', 0.005 / speed):
            return self._fail('straight approach to the marker centres failed')
        self.measure_width('grasp pose')

        # 5d. PROOF (R-17, D-05): BOTH hands in contact with the box itself, and
        #     BOTH tool tips where they were sent. Anything less is an abort.
        deadline = time.monotonic() + 3.0
        contact_l = contact_r = False
        while time.monotonic() < deadline and not (contact_l and contact_r):
            contact_l = contact_l or self.tips_on_box('left', max_age=1.0) >= 1
            contact_r = contact_r or self.tips_on_box('right', max_age=1.0) >= 1
            rclpy.spin_once(self, timeout_sec=0.05)
        placed = {}
        for side in ('left', 'right'):
            at = self._tf_point('base_link', f'{side}_tool_tip')
            error = None if at is None else float(np.linalg.norm(at - tip_goals[side]))
            placed[side] = error is not None and error <= 0.01
            self.get_logger().info(
                f'STEP5d {side} tool tip ' + ('not in TF' if error is None
                                              else f'{error * 1000:.1f} mm from its target'))
        self.get_logger().info(
            f'Grasp evidence: contact L={contact_l} R={contact_r}, '
            f'tool tips on target L={placed["left"]} R={placed["right"]}')
        if not (contact_l and contact_r and placed['left'] and placed['right']):
            self._interpolate_both(grasp_goals, 'release back to GRASP_V4', seconds=3.0)
            return self._fail('grasp not confirmed (need contact AND tool tips on target on '
                              'BOTH hands)')

        # 6. ATTACH (contact-verified), then LIFT. DART cannot hold the cube by
        #    pad friction alone, so the rigid attach - engaged only after the
        #    verified two-sided press - carries it (physically justified, not a
        #    teleport). NEVER two rigid holds: the RIGHT arm releases its
        #    pressure and retreats FIRST; a second rigid path shoving the fixed
        #    cube explodes the constraint solver (flings it metres away).
        if not self._attach_box(True):
            return self._fail('box could not be attached to wrist')
        # Record how the cube sits in the hand WHILE THE MEASUREMENT IS STILL
        # GOOD. From here the cube is rigidly attached to the left wrist, so this
        # relation holds until it is let go - and it is the only way to know
        # where the cube is once the pads cover its markers (P-45).
        # Where the cube actually ended up between the pads, measured rather
        # than assumed. `grip` is the centre the wrist cameras saw BEFORE the
        # press, and the press moves the cube: the two tool tips do not reach
        # their targets equally (measured 4.4-5.4 mm on the left against
        # 0.3-0.8 on the right, the same way round in every run), and whatever
        # of that asymmetry the cube absorbed is a shift of its centre.
        #
        # The pads now rest on opposite faces, so the point halfway between the
        # tool tips IS the centre of whatever is between them. It assumes
        # nothing about the cube - not its size, not that it is a cube - which
        # is the same reason the grasp is driven by the side markers rather
        # than by a dimension in a file. This is the last moment it can be
        # taken: the right pad eases off just below, and from then on the pads
        # cover the markers and no camera can see the thing being carried.
        #
        # Nothing moves here. The correction reaches the world through the
        # place, which brings this centre onto the marker with BOTH arms
        # travelling the same vector - the grip is never disturbed, so the
        # cube cannot be dropped by fixing where we think it is.
        tips = {side: self._tf_point('base_link', f'{side}_tool_tip')
                for side in ('left', 'right')}
        seen = np.array([grip.x, grip.y, grip.z])
        centre = seen
        if all(tip is not None for tip in tips.values()):
            between = (tips['left'] + tips['right']) / 2.0
            shift = between - seen
            # A pad that lost its face, or a transform read mid-motion, would
            # put the midpoint somewhere absurd; the cameras' centre is then
            # still the better of the two.
            if float(np.linalg.norm(shift)) <= 0.05:
                centre = between
                self.get_logger().info(
                    'cube centred on the pads: %.1f mm from where the cameras '
                    'put it (%.1f right, %.1f forward, %.1f up)' % (
                        np.linalg.norm(shift) * 1000.0, shift[1] * 1000.0,
                        shift[0] * 1000.0, shift[2] * 1000.0))
            else:
                self.get_logger().warn(
                    'the point between the pads is %.0f mm from the cameras\' '
                    'centre - too far to believe; keeping the cameras\' figure' % (
                        np.linalg.norm(shift) * 1000.0))
        else:
            missing = [s for s, t in tips.items() if t is None]
            self.get_logger().warn(
                f'no tool tip transform for {missing}; the cube centre stays '
                'the one the cameras measured before the press')

        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'left_tool_tip', rclpy.time.Time())
            rotation = quat_to_matrix(tf.transform.rotation)
            origin = np.array([tf.transform.translation.x,
                               tf.transform.translation.y,
                               tf.transform.translation.z])
            self._hold = {
                'offset': rotation.T @ (centre - origin),
                'normal': rotation.T @ normal,
                'half': span / 2.0,
            }
            self.get_logger().info(
                f'cube held: centre {span / 2.0 * 2:.3f} m square, '
                f'{np.linalg.norm(self._hold["offset"]) * 100:.1f} cm from the tool tip')
        except Exception as exc:
            self._hold = None
            self.get_logger().error(
                f'could not record how the cube sits in the hand: {exc}; '
                'the carry footprint and the place will have nothing to go on')
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
        # The right pad EASES OFF, it does not leave. It used to retreat 100 mm,
        # so the cube visibly ended up held by one hand - the assignment asks for
        # both. Backing off only the interference relieves the fight against the
        # now-rigid cube (two rigid paths explode the solver, P-17) while the pad
        # stays on the face. A parameter, because the smallest back-off that is
        # still stable is a measurement, not a guess.
        # With a touch instead of a squeeze there is only `touch_depth` to give
        # back: the right pad then rests on the face instead of pressing it.
        backoff = float(self.get_parameter('touch_depth').value)
        if not self.retreat_linear('right_arm', 'right_end_effector_link',
                                   u, -backoff, 'STEP6 ease right pad off'):
            self.get_logger().warn(
                'STEP6: right pad could not ease off; leaving it where it is')
        self.get_logger().info(
            f'STEP6: right pad eased off {backoff * 1000:.0f} mm, stays on the face')
        # LIFT WITH THE CARRIAGES, not with the arm. D-09 moved the lift onto
        # the arm because the carriages would not rise under load - that turned
        # out to be wrong: they were spawned exactly on their lower hard stop,
        # where Ignition freezes the joint in both directions. From 0.06 m they
        # carry the arms to 0.35 m with 0.0000 mm of error (P-13 #10-11), so the
        # lift goes back where R-03 wants it. The arm holds its pressed pose and
        # the whole assembly rises with the rail.
        # Not onto the rail's upper stop (0.65 m): a carriage resting exactly on
        # a stop cannot be moved off it again (P-13), and from 0.50 m a full
        # 0.15 m lift would end there.
        pick_height = self._joint.get('torso_left_carriage_joint', (0.0, 0.0))[0]
        lift_height = min(pick_height + 0.15, 0.64)
        if not self.move_torso(lift_height, 'STEP6 lift on the carriages', secs=3):
            return self._fail('carriages did not accept the lift command')
        settle = time.monotonic() + 1.2
        while time.monotonic() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        reached = [self._joint.get(n, (float('nan'), 0.0))[0]
                   for n in ('torso_left_carriage_joint', 'torso_right_carriage_joint')]
        self.get_logger().info(
            f'CARRIAGE LIFT MEASURED left={reached[0]:.4f} right={reached[1]:.4f} m '
            f'(commanded {lift_height:.4f})')
        self._log_grasp_geometry(grip, 'after lift')
        carry_width = self.measure_width('carrying the cube')
        door = float(self.get_parameter('door_width').value)
        if carry_width is not None and carry_width >= door:
            self.get_logger().warn(
                f'CARRY WIDTH {carry_width:.3f} m does NOT clear the {door:.2f} m '
                'doorway - this grasp can pick the cube up but cannot deliver it')
        self.get_logger().info('PICK+LIFT done (cube held by verified attach).')

        # 6b. PULL THE CUBE IN towards the robot before leaving the table (user).
        self._pull_cube_in(grip, span / 2.0)
        # From here the arms hold the cube, so the doorway gate has to check them
        # against the CARRY posture instead of the drive one - the mission
        # aborted at the doorway on exactly this (P-45).
        self.arm_posture_pub.publish(String(data=ARM_CARRY))
        self.get_logger().info(f'arms now hold the cube: gate reference is {ARM_CARRY}')
        self._publish_carried_cube()

        # 7. BACK AWAY FROM THE TABLE with the cube held, and stop (user, 16. 9.).
        #    The arms keep the grasp; 0.83 m wide with the cube between the hands.
        back = float(self.get_parameter('back_off_after_lift').value)
        moved = self.drive_distance(
            -back, speed=float(self.get_parameter('back_off_speed').value))
        if moved is None:
            return self._fail('could not back away from the table with the cube')
        self.get_logger().info(f'STEP7 backed away {moved:.3f} m with the cube held')
        # The table is behind us now, but MoveIt's copy rides along with the base
        # (no world frame): drop it before asking about the lowered arms.
        drop = PlanningScene()
        drop.is_diff = True
        for object_id in ['pick_table'] + [f'pick_table_leg_{i}' for i in range(4)]:
            co = CollisionObject()
            co.header.frame_id = 'base_link'
            co.id = object_id
            co.operation = CollisionObject.REMOVE
            drop.world.collision_objects.append(co)
        self.scene_pub.publish(drop)
        settle = time.monotonic() + 0.5
        while time.monotonic() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        # Then down with the cube in the hands (user: to 100 mm), as far as the
        # arms stay clear of the torso column.
        #
        # Two candidate heights used to be tried, the wanted one and the drive
        # one, and if MoveIt refused both the carriages simply stayed up. Run 10
        # of the centring series is what that costs: the left arm would have
        # touched the torso at 0.10 m, so the cube rode the whole way at the lift
        # height, 45 cm higher than every other run - and four minutes later the
        # head camera could not see the place marker past it at any of three
        # tilts, so the mission threw away a cube it was carrying correctly.
        #
        # So come down as far as it can rather than not at all. Any lowering
        # moves the cube out of the camera's line to the table; the height that
        # self-collides is not evidence that the one above it does.
        wanted = float(self.get_parameter('final_carriage_height').value)
        live = self._live_joints()
        here = live.get('torso_left_carriage_joint', DRIVE_CARRIAGE)
        ladder = [wanted, DRIVE_CARRIAGE]
        step = 0.05
        rung = wanted + step
        while rung < here - 1e-6:                   # every rung between, lowest first
            ladder.append(round(rung, 3))
            rung += step
        # Lowest first, so the one chosen is the lowest that MoveIt accepts.
        candidates = sorted({h for h in ladder if h < here - 1e-6})

        final_height = None
        for height in candidates:
            state = dict(live)
            state['torso_left_carriage_joint'] = state['torso_right_carriage_joint'] = height
            if self._state_valid(state, f'STEP7 check at carriages {height:.2f}') is not False:
                final_height = height
                break
        # What stops the descent is always the same pair - the left elbow against
        # the torso column - and the arms have a way out of it that does not
        # disturb the grip: swivel the elbows further from the torso, which
        # holds the hands where they are. STEP6b already does this once, 20 deg.
        # When the ladder cannot reach the wanted height, do it again and
        # re-check, because the cost of riding high is the whole run: the cube
        # then blocks the head camera and the place marker is never seen at any
        # head pose (P-54), which is one failure in every twelve.
        if final_height is None or final_height > wanted + 0.02:
            extra = math.radians(float(self.get_parameter('extra_elbow_out_deg').value))
            self.get_logger().warn(
                f'STEP7 the arms block the descent below '
                f'{final_height if final_height is not None else here:.2f} m; '
                f'swinging the elbows {math.degrees(extra):.0f} deg further out')
            if self._swing_elbows_out(extra):
                live = self._live_joints()
                for height in candidates:
                    if final_height is not None and height >= final_height:
                        break
                    state = dict(live)
                    state['torso_left_carriage_joint'] = height
                    state['torso_right_carriage_joint'] = height
                    if self._state_valid(
                            state, f'STEP7 re-check at carriages {height:.2f}') is not False:
                        final_height = height
                        break

        if final_height is None:
            self.get_logger().warn(
                'STEP7 the held arms would touch the torso at every height down to '
                f'{min(candidates) if candidates else here:.2f} m; the carriages stay '
                'where they are and the cube rides high - the head camera may not see '
                'past it at the far table')
        elif final_height > wanted + 1e-6:
            self.get_logger().warn(
                f'STEP7 carriages can only come down to {final_height:.2f} m, not the '
                f'{wanted:.2f} m wanted: lower than that the held arms touch the torso')
        elif not self._torso_to(final_height, 'STEP7 carriages down with the cube'):
            return self._fail('carriages did not come down with the cube')
        self.measure_width('carrying the cube, away from the table')
        self.get_logger().info('TASK COMPLETE: cube lifted and carried away from the table.')

        # 8. MISSION: carry it to the other room and put it down on the marker.
        if bool(self.get_parameter('mission').value):
            room = str(self.get_parameter('place_room').value)
            self._report_mission_step(6, 8, 'Prijenos u sobu za odlaganje',
                                      f'Nošenje kocke kroz prolaze prema sobi "{room}" ({room}:dock)...')
            if not self._goto(f'{room}:dock'):
                return self._fail(f'room_navigator did not reach {room}:dock')
            if not self._place_on_marker():
                return None
            self.get_logger().info('MISSION COMPLETE: the cube is on the marker.')

    def _report_mission_step(self, step, total_steps, phase, detail, state='running'):
        """Report mission progress to /mission/task_status topic and print a clean banner."""
        self._current_step = step
        self._total_steps = total_steps
        payload = {
            'step': step,
            'total_steps': total_steps,
            'phase': phase,
            'detail': detail,
            'state': state,
            'time': time.time(),
        }
        if hasattr(self, 'task_status_pub'):
            self.task_status_pub.publish(String(data=json.dumps(payload)))

        prefix = f"[{step}/{total_steps}]" if (step and total_steps) else "[ZADATAK]"
        color = "\033[1;32m" if state == 'success' else (
            "\033[1;31m" if state == 'aborted' else (
                "\033[1;33m" if state == 'waiting' else "\033[1;36m"
            )
        )
        reset = "\033[0m"
        banner = (
            f"\n{color}"
            f"======================================================================\n"
            f">>> {prefix} {phase}\n"
            f"    {detail}\n"
            f"======================================================================{reset}\n"
        )
        sys.stdout.write(banner)
        sys.stdout.flush()
        self.get_logger().info(f"{prefix} {phase}: {detail}")

    def _fail(self, where):
        self.get_logger().error(f'Task aborted during: {where}')
        self._report_mission_step(
            getattr(self, '_current_step', 0),
            getattr(self, '_total_steps', 8),
            'PREKID ZADATKA',
            f'Misija prekinuta: {where}',
            state='aborted'
        )


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
