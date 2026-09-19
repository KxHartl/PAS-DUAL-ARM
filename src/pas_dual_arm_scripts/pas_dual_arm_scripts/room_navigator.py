"""Drive room to room along the zone graph, one square leg at a time.

Replaces the hand-surveyed waypoints in `go_to_room.py`.  Every pose here comes
out of `/nav_graph`, which `nav_zones` derives from the doorways and tables the
SLAM map contains, so nothing has to be re-measured when the world changes.

A room-to-room trip is never one Nav2 goal.  A single goal across two rooms
gives the planner every reason to enter the doorway on a diagonal, because the
diagonal is shorter.  Instead each doorway becomes two goals on its centreline -
a portal pose 1.45 m out on this side, then the matching pose on the far side -
so the leg through the opening is a straight line the robot is already squared up
to before it starts.

Before committing to that leg the preconditions are actually checked, and a
failed check aborts loudly rather than driving on and hoping (D-12):

  * the arms are in ARM_DRIVE.  They are what makes the robot 0.854 m wide, and
    they have been measured sagging out to 1.109 m while driving (P-37), which
    does not fit through a 1.0 m door.
  * the robot is really on the lane centreline and really square to it, within
    a few centimetres and a few degrees - the swept width is
    1.04*sin(yaw) + 0.854*cos(yaw), so a couple of degrees is the whole budget.

and while the leg runs, the scan is watched: if the gap on either side drops
below the abort threshold the goal is cancelled instead of scraping through.

    ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: blue}"
    ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: stop}"

`scripts/nav_gui.py` is the button panel over the same two topics.
"""

import json
import math
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Polygon, PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from pas_dual_arm_scripts.postures import ARM_DRIVE, POSTURES, move_to_posture
from pas_dual_arm_scripts.robot_extent import ExtentMeasurer

# Nav2 footprint, so the geometry here and the costmap's agree.
HALF_LENGTH, HALF_WIDTH = 0.52, 0.427
# Left plus right reading below this means the robot is between the jambs rather
# than in a room: the doorway is 1.0 m, the narrowest room dimension is 6 m.
DOORWAY_SPAN = 1.5
# How far from a leg's goal the robot may be and still count as having arrived.
# Nav2's own goal checker works to 5 cm, so this is not a precision gate: it is
# the distance beyond which "succeeded" cannot be true, whatever Nav2 reports.
ARRIVAL_TOLERANCE = 0.30
ARRIVAL_YAW_TOLERANCE = math.radians(20.0)


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def swept_width(yaw_error, length=2 * HALF_LENGTH, width=2 * HALF_WIDTH):
    """How wide an opening this robot needs at a given heading error."""
    return length * abs(math.sin(yaw_error)) + width * abs(math.cos(yaw_error))


class Leg:
    def __init__(self, pose, label, transit=False, dock=False):
        self.x, self.y, self.yaw = pose
        self.label = label
        self.transit = transit
        self.dock = dock


class RoomNavigator(Node):
    def __init__(self):
        super().__init__('room_navigator')
        self.declare_parameter('arm_tolerance', 0.15)
        # How wide the robot may be to be let at a doorway. The openings measure
        # 0.997-0.999 m live, and the carry posture measures well under this, so
        # the margin is real rather than nominal. It is the quantity the doorway
        # actually constrains, which the joint tolerance above only stood in for.
        self.declare_parameter('max_width', 0.95)
        # Whether being too wide is a reason to try before it is a reason to
        # abort. Only ever acts with empty hands; see _try_narrowing.
        self.declare_parameter('narrow_before_abort', True)
        # How far the heading may drift DURING a transit before the robot stops
        # and squares up again. The gate before the opening allows 0.087 rad,
        # and then nothing looks at the heading for the 2.3 m that follows: a
        # run of 19 Sep entered at 1.3 deg and crossed at 6.05, which costs
        # 6 cm of a 99.5 cm opening (P-55). 0.05 rad is 2.9 deg, comfortably
        # above the 1.32 deg median and well below what closes the gap.
        self.declare_parameter('transit_heading_limit', 0.05)
        self.declare_parameter('transit_squares_up', True)
        # A leg that stops moving while Nav2 still holds the goal. It happens a
        # few centimetres short: the position critics behave as though they have
        # arrived and RotateToGoal returns zero outside xy_goal_tolerance, so
        # the controller publishes exact zeros and the leg sits there until the
        # 240 s timeout. Measured directly - 6042 commands of 0.000 on all three
        # axes over 160 s, the robot 6 cm from the goal and 46 deg off its
        # heading. Re-sending the goal makes Nav2 plan again from where it
        # actually is, which is usually enough to leave the dead zone.
        self.declare_parameter('stall_seconds', 25.0)
        self.declare_parameter('stall_radius', 0.02)
        self.declare_parameter('centreline_tolerance', 0.08)
        # 5.0 degrees. The crossing that was actually driven on 14 Sep went
        # through door 0 at 4.2 degrees and 1.8 cm, so a 4.0 degree limit would
        # have refused a passage that demonstrably worked.
        self.declare_parameter('heading_tolerance', 0.087)
        # 5 mm (0.005 m). Physical door is 1.0 m, robot width is 0.854 m (clearance
        # nominal ~4.8 to 7.3 cm). 0.03 m aborted safe passages with 0.7 cm clearance.
        self.declare_parameter('min_side_clearance', 0.005)
        # Below this a leg counts as axis aligned already; it also absorbs the
        # few centimetres between a detected door centre and a table centre.
        self.declare_parameter('corner_threshold', 0.30)
        # The L-shaped detour through the middle of a room. Off: the verified
        # run crossed the empty room on the direct diagonal and squared up at
        # the next portal, which is both shorter and what was measured.
        self.declare_parameter('square_corners', False)
        self.declare_parameter('leg_timeout', 240.0)
        self.declare_parameter('require_arms', True)
        # Speed per leg (user, 16. 9.). The open-room legs run at `fast_*`; a
        # doorway transit and a dock approach drop to `slow_*`. The collision
        # monitor still scales anything down near an obstacle on top of this.
        # Forward speed in the open, and it is NOT limited by this number.
        # Measured on a run at 0.60: the controller commands 0.566 m/s, all four
        # wheels turn at 6.11 rad/s - 0.466 m/s at the rim, already short of the
        # command - and the robot moves at 0.230. Half the commanded speed is
        # lost to slip, because the mecanum wheels are emulated with anisotropic
        # friction at 45 degrees (P-09) and forward motion is partly along the
        # low-friction axis. Raising this only spins the wheels faster against
        # the floor: 0.45 and 0.60 gave runs of 576 s and 585 s.
        self.declare_parameter('fast_vel_x', 0.45)
        # Open-room speed. Note that `fast_vel_x` below is written over
        # FollowPath.max_vel_x at the start of every leg, so raising the limit in
        # nav2_params.yaml alone changes nothing - the first attempt at PAL's
        # speeds did exactly that and drove a mismatched robot.
        self.declare_parameter('fast_speed_xy', 0.50)
        self.declare_parameter('slow_vel_x', 0.18)
        self.declare_parameter('slow_speed_xy', 0.22)
        # A doorway and a dock were one tier, and they are not the same kind of
        # slow. The dock is where the arrival has to be accurate to a few
        # centimetres for the grasp; the doorway only has to be narrow-safe, and
        # it is watched the whole way through (heading guard, gap gate). The
        # three transits were 43 s of every mission at 0.16 m/s on 2.3 m -
        # the largest single item in the driving - so they get their own number.
        self.declare_parameter('door_vel_x', 0.26)
        self.declare_parameter('door_speed_xy', 0.30)
        # Who decides the speed. TRUE: per leg, which is what the 12/12 series
        # manip-par measured. The mask route below was tried with DWB and failed
        # - it let DWB take doorways at 80 % where per-leg speed gives 0.22 m/s,
        # and DWB could not find a trajectory through at that speed.
        #
        # The original reasons for turning it off, kept for whoever tries again:
        #
        # The SpeedFilter reads a limit off a mask over the map, so the speed
        # changes where the building is tight rather than where one leg ends and
        # the next begins - that is the point of it, and two mechanisms setting
        # the same ceiling would multiply into a number meaning nothing.
        #
        # And the per-leg write below sets `FollowPath.max_vel_x`, which DOES
        # NOT EXIST under MPPI - it is `vx_max` there. Left on, it would fail
        # silently every leg and the mission would drive at whatever the YAML
        # happened to say.
        self.declare_parameter('set_leg_speed', True)

        self._graph = None
        self._scan = None
        self._joints = None
        self._request = None
        self._pending_request = None
        self._cancel = False
        self._busy = False
        # Index of the table the robot is currently docked at, if any. A docked
        # robot has its front 10 cm from a table top and cannot turn: its
        # circumscribed radius is 0.673 m and the table starts 0.50 m away. So it
        # leaves the way it came in, and that is not optional - see _undock_leg.
        self._docked = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, '/room_navigator/status', latched)
        # Used only to square the robot up mid-transit, and only after the Nav2
        # goal has been cancelled - two publishers on one base fight.
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/nav_graph', self._on_graph, latched)
        self.create_subscription(String, '/room_navigator/goto', self._on_goto, 10)
        # Which posture the arms are meant to be holding right now. The mission
        # switches this to the carry posture once the cube is in the hands.
        # Latched, so it survives whoever subscribes late.
        self._arm_posture = ARM_DRIVE
        self.create_subscription(String, '/room_navigator/arm_posture',
                                 self._on_arm_posture, latched)
        # The robot's live outline, for the width gate below. Same source the
        # costmaps' footprint comes from, so the two cannot disagree.
        self._measurer = None
        self.create_subscription(String, '/robot_description',
                                 self._on_robot_description, latched)
        # What the robot is carrying, as ground corners in base_link. The URDF
        # knows nothing about the cube, so without this the width would be the
        # arms' alone - the same reason footprint_publisher subscribes to it.
        self._carried = None
        self.create_subscription(Polygon, '/mission/carried_points',
                                 self._on_carried, latched)
        # nav2.launch.py remaps Nav2's own /goal_pose to /bt_goal_pose, so RViz
        # "2D Goal Pose" lands here instead of going straight to bt_navigator.
        # A goal in another room is then routed through the doorway portals
        # rather than driven at on the diagonal.
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose, 10)
        self.create_subscription(LaserScan, '/scan_filtered', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # Several poses as one goal, for the stretches between doorways: the
        # robot passes through them instead of parking on each (user, 16. 9.).
        self._nav_through = ActionClient(self, NavigateThroughPoses,
                                         '/navigate_through_poses')
        self._speed = self.create_client(SetParameters,
                                         '/controller_server/set_parameters')
        self._status('idle', 'waiting for /nav_graph')

    # -------------------------------------------------------------- plumbing
    def _on_graph(self, msg):
        self._graph = json.loads(msg.data)
        names = [room['name'] for room in self._graph['rooms']]
        self.get_logger().info(f'zone graph: rooms {names}, '
                               f'{len(self._graph["doors"])} door(s), '
                               f'{len(self._graph["tables"])} table(s)')
        if not self._busy:
            self._status('idle', f'ready, rooms: {", ".join(names)}')

    def _on_scan(self, msg):
        self._scan = msg

    def _on_joints(self, msg):
        self._joints = dict(zip(msg.name, msg.position))

    def _on_goto(self, msg):
        request = msg.data.strip().lower()
        if request == 'undock':
            request = self._docked_room() or 'undock'
        if request in ('stop', 'cancel', 'abort'):
            self._cancel = True
            self._pending_request = None
            self.get_logger().warn('cancel requested')
            return
        if self._busy:
            self.get_logger().info(f'cancelling active navigation to take "{request}"')
            self._cancel = True
            self._pending_request = request
            return
        self._request = request

    def _on_robot_description(self, msg):
        if self._measurer is not None:
            return
        try:
            self._measurer = ExtentMeasurer(msg.data)
        except Exception as exc:
            self.get_logger().warn(
                f'could not read the robot description, so the width gate falls '
                f'back to comparing joints: {exc}')

    def _on_carried(self, msg):
        points = [(p.x, p.y) for p in msg.points]
        self._carried = points if len(points) >= 3 else None

    def odom_pose(self):
        """(x, y, yaw) in `odom`, for asking whether the robot has MOVED.

        Not `map`: the localised pose jumps a centimetre or two on every AMCL
        update, and a stall test with a 2 cm threshold reads that jitter as
        motion and never fires. It did exactly that. The odometry is smooth -
        1.8 mm of drift over a calibration drive - and for "has anything
        changed in the last 25 seconds" its lack of absolute truth does not
        matter.
        """
        try:
            tf = self._tf.lookup_transform('odom', 'base_footprint',
                                           rclpy.time.Time()).transform
        except Exception:
            return None
        return (tf.translation.x, tf.translation.y,
                math.atan2(2.0 * (tf.rotation.w * tf.rotation.z),
                           1.0 - 2.0 * tf.rotation.z * tf.rotation.z))

    def width_now(self):
        """How wide the robot is right now, across its own heading, or None.

        Every link's collision geometry through live TF, projected into
        base_link, and the span across y. That is the number a doorway cares
        about, measured on the robot that is actually standing there.
        """
        if self._measurer is None:
            return None
        points = self._measurer.points(self._tf, rclpy.time.Time())
        if points is None or not len(points):
            return None
        spread = [float(points[:, 1].min()), float(points[:, 1].max())]
        if self._carried:
            spread += [y for _, y in self._carried]
        return max(spread) - min(spread)

    def _on_arm_posture(self, msg):
        name = msg.data.strip() or ARM_DRIVE
        if name not in POSTURES:
            self.get_logger().error(
                f'unknown arm posture "{name}"; keeping {self._arm_posture}')
            return
        if name != self._arm_posture:
            self.get_logger().info(f'arm reference posture is now {name}')
        self._arm_posture = name

    def _set_leg_speed(self, tier, label):
        """Nav2's speed limit for this leg.

        Full speed in the open, slow through a doorway and onto a dock, where the
        margins are centimetres (user, 16. 9.: fast when it is safe, slower
        through doors and up to tables). A warning, not an abort: driving at the
        speed it already has is safe, just slower.
        """
        if not self.get_parameter('set_leg_speed').value:
            return
        # Only DWB has these names. Under MPPI the equivalents are vx_max and
        # there is no separate norm cap, so this path is DWB-only by design.
        # `tier` is 'fast', 'door' or 'slow'; a bare bool from an older caller
        # still means fast/slow.
        if tier is True:
            tier = 'fast'
        elif tier is False:
            tier = 'slow'
        vel_x = self.get_parameter(f'{tier}_vel_x').value
        speed_xy = self.get_parameter(f'{tier}_speed_xy').value
        if not self._speed.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f'{label}: controller_server parameters unavailable; speed unchanged')
            return
        request = SetParameters.Request()
        for name, value in (('FollowPath.max_vel_x', vel_x),
                            ('FollowPath.max_speed_xy', speed_xy)):
            request.parameters.append(Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=float(value))))
        future = self._speed.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result() if future.done() else None
        if response is None or not all(r.successful for r in response.results):
            self.get_logger().warn(f'{label}: could not set the speed limit')
            return
        self.get_logger().info(
            f'{label}: {"full" if tier == "fast" else "reduced"} speed '
            f'({vel_x:.2f} m/s, {tier})')

    def _on_goal_pose(self, msg):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        yaw = quat_to_yaw(msg.pose.orientation)
        target = (x, y, yaw)
        self.get_logger().info(
            f'goal received on /goal_pose: ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.1f} deg)')
        if self._busy:
            self.get_logger().info('cancelling active navigation to take new RViz goal')
            self._cancel = True
            self._pending_request = target
            return
        self._request = target

    def _status(self, state, detail, **extra):
        msg = String()
        msg.data = json.dumps({'state': state, 'detail': detail, **extra}, allow_nan=False)
        self._status_pub.publish(msg)

    def _spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ------------------------------------------------------------------- pose
    def pose(self):
        """Localised pose as (x, y, yaw) in map, or None."""
        try:
            tf = self._tf.lookup_transform('map', 'base_footprint',
                                           rclpy.time.Time()).transform
        except Exception:
            return None
        q = tf.rotation
        return (tf.translation.x, tf.translation.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def side_clearance(self):
        """Nearest return to left and right of the robot, or None.

        Only returns beside the robot count (|x| < half its length), so in open
        space this reads the room and in a doorway it reads the jambs.  The
        lidar sits on the base centreline with no lateral offset, so scan
        coordinates are base coordinates.
        """
        scan = self._scan
        if scan is None:
            return None
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        good = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        if not good.any():
            return None
        xs, ys = ranges[good] * np.cos(angles[good]), ranges[good] * np.sin(angles[good])
        beside = np.abs(xs) < HALF_LENGTH
        left, right = ys[beside & (ys > 0)], ys[beside & (ys < 0)]
        return (float(left.min()) if left.size else float('inf'),
                float(-right.max()) if right.size else float('inf'))

    # ------------------------------------------------------------------ gates
    def arms_ok(self):
        """Every joint of the REFERENCE posture within tolerance, or why not.

        Width first, posture only as a fallback. The reference posture is
        DRIVE_V4 when the robot travels empty and CARRY_V4 once
        it has the cube: with the cube in the hands the arms are nowhere near the
        drive posture, and a gate that only knew DRIVE_V4 aborted the mission at
        the doorway (P-45). Both postures are measured and both fit the opening -
        which posture is held is the question, not whether the arms may move.
        """
        if not self.get_parameter('require_arms').value:
            return True, 'arm check disabled'

        # Ask the robot how wide it is, rather than how far its joints are from
        # a remembered posture. A joint 0.010 rad past the tolerance aborted a
        # run of the 18 Sep series without anyone knowing whether the robot had
        # actually grown: the posture is a proxy for the width, and this is the
        # width. It also stops being a lie the moment the arms find a different
        # way to hold the same cube.
        width = self.width_now()
        if width is not None:
            limit = float(self.get_parameter('max_width').value)
            if width <= limit:
                return True, f'{width * 100:.1f} cm wide, limit {limit * 100:.0f}'
            return False, (f'{width * 100:.1f} cm wide against a {limit * 100:.0f} cm '
                           f'limit; the arms set the width that has to fit the doorway')

        # No description or no transforms yet: fall back to the posture, which
        # is what this gate was before the outline was available.
        if self._joints is None:
            return False, 'no /joint_states yet'
        posture = self._arm_posture
        if posture not in POSTURES:
            return False, f'unknown arm reference posture "{posture}"'
        tolerance = self.get_parameter('arm_tolerance').value
        worst_name, worst = None, 0.0
        for side, targets in POSTURES[posture].items():
            for index, target in targets.items():
                name = f'{side}_joint_{index}'
                if name not in self._joints:
                    return False, f'{name} missing from /joint_states'
                # Joints 1/3/5/7 are continuous: 2 pi apart is the SAME pose, so
                # the raw difference can read as a huge error on an arm that is
                # exactly where it should be.
                delta = self._joints[name] - target
                error = abs(math.atan2(math.sin(delta), math.cos(delta)))
                if error > worst:
                    worst_name, worst = name, error
        if worst > tolerance:
            return False, (f'{worst_name} is {worst:.3f} rad off {posture} '
                           f'(limit {tolerance:.2f}); the arms set the width that '
                           f'has to fit the doorway')
        return True, f'{posture} held, worst joint {worst:.3f} rad'

    @staticmethod
    def lane_offset_of(pose, leg):
        """Signed distance of a pose from the leg's centreline, positive to its left.

        The centreline runs along the leg's own heading.  Split out so the gate and
        the telemetry in `_drive` compare exactly the same number.
        """
        return (-(pose[0] - leg.x) * math.sin(leg.yaw)
                + (pose[1] - leg.y) * math.cos(leg.yaw))

    def lane_offset(self, leg):
        """`lane_offset_of` for the currently localised pose, or None.

        Only as true as AMCL is - which is the point of comparing it with the lidar.
        """
        pose = self.pose()
        return None if pose is None else self.lane_offset_of(pose, leg)

    def aligned_with(self, leg):
        """On the lane centreline and square to it, within the doorway budget."""
        pose = self.pose()
        if pose is None:
            return False, 'no map -> base_footprint transform'
        heading_error = wrap(pose[2] - leg.yaw)
        offset = abs(self.lane_offset_of(pose, leg))
        lateral = self.get_parameter('centreline_tolerance').value
        heading = self.get_parameter('heading_tolerance').value
        detail = (f'offset {offset:.3f} m, heading {math.degrees(heading_error):+.1f} deg, '
                  f'needs {swept_width(heading_error):.3f} m of doorway')
        if offset > lateral:
            return False, f'{offset:.3f} m off the lane centreline (limit {lateral:.2f}); {detail}'
        if abs(heading_error) > heading:
            return False, (f'{math.degrees(heading_error):+.1f} deg off square '
                           f'(limit {math.degrees(heading):.1f}); {detail}')
        return True, detail

    # ------------------------------------------------------------------ route
    def room_at(self, x, y):
        if self._graph is None:
            return None
        for room in self._graph['rooms']:
            x0, y0, x1, y1 = room['bbox']
            if x0 <= x <= x1 and y0 <= y <= y1:
                return room['name']
        return None

    def _door_between(self, a, b):
        for door in self._graph['doors']:
            if a in (door['rooms'] or []) and b in (door['rooms'] or []):
                return door
        return None

    def _room_path(self, start, target):
        """Shortest chain of rooms, breadth first over the doorways."""
        if start == target:
            return [start]
        queue, seen = [[start]], {start}
        while queue:
            path = queue.pop(0)
            for door in self._graph['doors']:
                rooms = door['rooms'] or []
                if path[-1] not in rooms:
                    continue
                nxt = rooms[1] if rooms[0] == path[-1] else rooms[0]
                if nxt is None or nxt in seen:
                    continue
                if nxt == target:
                    return path + [nxt]
                seen.add(nxt)
                queue.append(path + [nxt])
        return None

    def nearest_room(self, x, y):
        """Closest detected room to a point, for a goal just outside one."""
        if self._graph is None or not self._graph.get('rooms'):
            return None
        best_room, best_dist = None, float('inf')
        for room in self._graph['rooms']:
            x0, y0, x1, y1 = room['bbox']
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            dist = math.hypot(dx, dy)
            if dist < best_dist:
                best_dist, best_room = dist, room['name']
        return best_room

    def _table_by_pose(self, pose, tolerance=0.30):
        """Find a table whose dock pose matches the given (x, y)."""
        if self._graph is None:
            return None
        for table in self._graph.get('tables', []):
            dock = table.get('dock')
            if dock and math.hypot(pose[0] - dock[0], pose[1] - dock[1]) <= tolerance:
                return table
        return None

    def _docked_table(self):
        """Table the robot is currently docked at, or None."""
        if self._graph is None:
            return None
        tables = self._graph.get('tables', [])
        if self._docked is not None:
            for table in tables:
                if table.get('id') == self._docked or table.get('room') == self._docked:
                    return table
        pose = self.pose()
        if pose is not None:
            for table in tables:
                dock = table.get('dock')
                if dock and math.hypot(pose[0] - dock[0], pose[1] - dock[1]) < 0.25:
                    return table
        return None

    def _docked_room(self):
        """Room name of the table the robot is currently docked at, or None."""
        table = self._docked_table()
        return table['room'] if table is not None else None

    def _undock_leg(self):
        """If docked at a table, back off straight along the dock axis to the approach pose."""
        table = self._docked_table()
        if table is None:
            return []
        return [Leg(table['approach'],
                    f'{table["room"]}: back off the table to approach pose')]

    def plan(self, destination):
        """Legs to `destination`, a room name or an (x, y, yaw), or (None, reason)."""
        if self._graph is None:
            return None, 'no zone graph yet - is nav_zones running and /map published?'
        pose = self.pose()
        if pose is None:
            return None, 'no map -> base_footprint transform; is AMCL localised?'
        start = self.room_at(pose[0], pose[1]) or self.nearest_room(pose[0], pose[1])
        if start is None:
            return None, (f'robot at ({pose[0]:.2f}, {pose[1]:.2f}) is not inside '
                          f'any detected room')

        target_pose = None
        want_dock = False
        if isinstance(destination, str) and destination.endswith(':dock'):
            destination, want_dock = destination[:-len(':dock')], True
        if isinstance(destination, str):
            known = [room['name'] for room in self._graph['rooms']]
            if destination not in known:
                return None, f'unknown room "{destination}"; known rooms: {", ".join(known)}'
            target_room = destination
        elif isinstance(destination, (tuple, list)) and len(destination) >= 3:
            target_pose = (float(destination[0]), float(destination[1]),
                           float(destination[2]))
            target_room = self.room_at(target_pose[0], target_pose[1]) or \
                self.nearest_room(target_pose[0], target_pose[1])
            if target_room is None:
                return None, (f'target ({target_pose[0]:.2f}, {target_pose[1]:.2f}) '
                              f'is outside every known room')
        else:
            return None, f'unsupported destination format: {destination}'

        # Backing off a dock comes before anything else, including working out a
        # route: every other leg assumes the robot may turn where it stands.
        prefix = self._undock_leg()

        # A goal in the room we are already in needs no doorway at all.
        if target_pose is not None and start == target_room:
            return prefix + [Leg(target_pose, f'{start}: direct to goal')], \
                f'same room ({start}), {len(prefix) + 1} leg(s)'

        path = self._room_path(start, target_room)
        if path is None:
            return None, f'no doorway chain from {start} to {target_room}'
        destination = target_room

        legs = list(prefix)
        for here, there in zip(path, path[1:]):
            door = self._door_between(here, there)
            portal = (door.get('portals') or {}).get(here)
            if portal is None:
                return None, f'door between {here} and {there} has no portal pose for {here}'
            legs.append(Leg(portal['approach'], f'{here} -> {there}: line up at the doorway'))
            legs.append(Leg(portal['exit'], f'{here} -> {there}: straight through the doorway',
                            transit=True))

        if target_pose is not None:
            legs.append(Leg(target_pose, f'{destination}: direct to goal'))
        else:
            table = next((t for t in self._graph['tables']
                          if t['room'] == destination), None)
            if table is not None:
                if not prefix or (prefix and start != destination):
                    legs.append(Leg(table['approach'],
                                    f'{destination}: square up in front of the table'))
                if want_dock:
                    # Straight in along the same axis it just squared up on, so
                    # the move is a pure translation and nothing swings sideways.
                    legs.append(Leg(table['dock'],
                                    f'{destination}: dock at the table', dock=True))
            elif not legs:
                room = next(r for r in self._graph['rooms'] if r['name'] == destination)
                legs.append(Leg((room['centre'][0], room['centre'][1], pose[2]),
                                f'{destination}: room centre'))
            else:
                room = next(r for r in self._graph['rooms'] if r['name'] == destination)
                legs.append(Leg((room['centre'][0], room['centre'][1], legs[-1].yaw),
                                f'{destination}: room centre'))
        if self.get_parameter('square_corners').value:
            legs = self._squared_off(legs, pose)
        return legs, f'{" -> ".join(path)}, {len(legs)} legs'

    def _squared_off(self, legs, pose):
        """Break a leg that moves on both axes into two, so the route turns corners.

        Doorways and tables are handled by the zones; this is about the open
        middle of a room, where nothing stops the planner cutting a diagonal
        between two portals.  Driving out along the current heading and then
        turning is not shorter, but it is the rectilinear motion this is for,
        and it means the robot is already square well before the next doorway.
        """
        threshold = self.get_parameter('corner_threshold').value
        squared, previous = [], Leg((pose[0], pose[1], pose[2]), 'start')
        for leg in legs:
            corner = self._corner(previous, leg, threshold)
            if corner is not None:
                squared.append(corner)
                previous = corner
            squared.append(leg)
            previous = leg
        return squared

    def _corner(self, previous, leg, threshold):
        """The turning point of an L between two poses, if one is needed and fits."""
        dx, dy = leg.x - previous.x, leg.y - previous.y
        if abs(dx) < threshold or abs(dy) < threshold:
            return None
        # Carry on along the axis the robot is already facing, then turn.
        along_x = abs(math.cos(previous.yaw)) >= abs(math.sin(previous.yaw))
        x, y = (leg.x, previous.y) if along_x else (previous.x, leg.y)
        room = self.room_at(x, y)
        if room is None or room != self.room_at(previous.x, previous.y):
            return None       # the corner would leave the room; keep the direct leg
        bbox = next(r['bbox'] for r in self._graph['rooms'] if r['name'] == room)
        margin = math.hypot(HALF_LENGTH, HALF_WIDTH)
        if not (bbox[0] + margin <= x <= bbox[2] - margin
                and bbox[1] + margin <= y <= bbox[3] - margin):
            return None       # too close to a wall to turn there
        return Leg((x, y, leg.yaw), f'corner onto the {"x" if along_x else "y"} axis')

    # -------------------------------------------------------------- execution
    def tick(self):
        request = self._request
        self._request = None
        if request is None and self._pending_request is not None:
            request = self._pending_request
            self._pending_request = None
        if request is None:
            return
        self._cancel = False
        self._busy = True
        try:
            self.run(request)
        finally:
            self._busy = False

    def run(self, destination):
        # A room name stays a name; an RViz pose becomes a short label, so the
        # status topic and the GUI both stay readable.
        dest = destination if isinstance(destination, str) else \
            f'({destination[0]:+.2f}, {destination[1]:+.2f})'
        legs, detail = self.plan(destination)
        if legs is None:
            self.get_logger().error(f'cannot go to "{dest}": {detail}')
            self._status('failed', detail, destination=dest)
            return
        self.get_logger().info(f'going to {dest}: {detail}')
        if not self._nav.wait_for_server(timeout_sec=20.0):
            self._status('failed', 'Nav2 /navigate_to_pose is not up', destination=dest)
            self.get_logger().error('Nav2 /navigate_to_pose is not up')
            return

        # Consecutive legs that need no gate are driven as ONE goal, so the robot
        # flows through the corners instead of stopping on each. Two kinds of leg
        # always stand alone, and neither is negotiable:
        #   transit  gated before it starts and watched while it runs (P-39)
        #   dock     entered straight along the table's own axis with no rotation
        #            at the end of it, 15 cm from the table top (D-20). A pose
        #            that is only passed through does not settle its heading, and
        #            this is the one place where a few degrees is the whole
        #            budget.
        groups, run_of = [], []
        for leg in legs:
            if leg.transit or getattr(leg, 'dock', False):
                if run_of:
                    groups.append(run_of)
                    run_of = []
                groups.append([leg])
            else:
                run_of.append(leg)
        if run_of:
            groups.append(run_of)

        for index, group in enumerate(groups, start=1):
            first, last = group[0], group[-1]
            label = f'leg {index}/{len(groups)} - {first.label}' if len(group) == 1 else \
                f'leg {index}/{len(groups)} - {first.label} -> {last.label} ' \
                f'({len(group)} poses, without stopping)'
            if self._cancel:
                self._status('cancelled', f'cancelled before {label}', destination=dest)
                return
            if first.transit and not self._gate(first, label, dest):
                return
            self._status('driving', label, destination=dest,
                         goal=[last.x, last.y, last.yaw])
            self.get_logger().info(
                f'{label} -> ({last.x:+.2f}, {last.y:+.2f}, '
                f'{math.degrees(last.yaw):+.1f} deg)')
            docking = any(getattr(leg, 'dock', False) for leg in group)
            transit = any(leg.transit for leg in group)
            tier = 'slow' if docking else ('door' if transit else 'fast')
            self._set_leg_speed(tier, label)
            driven = (self._drive(first, label, dest) if len(group) == 1
                      else self._drive_through(group, label, dest))
            if not driven:
                return
            if not self._confirm_arrival(last, label, dest):
                return
            if getattr(last, 'dock', False):
                table = self._table_by_pose((last.x, last.y))
                self._docked = table['id'] if table is not None else dest.split(':')[0]
            else:
                self._docked = None

        self._status('arrived', f'reached {dest}', destination=dest)
        self.get_logger().info(f'reached {dest}')

    def _try_narrowing(self, why):
        """Too wide for the doorway: fold back to the reference posture and re-measure.

        Refusing was the whole behaviour before. But an arm that has drifted
        wide is a thing the robot can undo, and the posture it should be holding
        is known - so ask for it once, measure again, and only then give up.

        NOT while the cube is held. The cube is rigidly attached to the left
        wrist, and driving both arms through MoveIt against a fixed object is
        what throws it metres across the room (P-17): two rigid paths fighting
        one constraint. Carrying, the arms are left exactly where they are and
        the reason is said out loud, because a mission that silently risked the
        cube to save a leg would be the wrong trade.
        """
        if not self.get_parameter('narrow_before_abort').value:
            return False, why
        if self._arm_posture != ARM_DRIVE:
            return False, (f'{why}; the cube is in the hands, so the arms are NOT '
                           'moved to make room - a planned motion against a rigidly '
                           'held cube throws it (P-17)')
        self.get_logger().warn(f'{why}; folding back to {self._arm_posture} and '
                               'measuring again before giving up')
        try:
            moved = move_to_posture(self, self._arm_posture, timeout=25.0,
                                    label='narrow for the doorway')
        except Exception as exc:
            return False, f'{why}; folding back to {self._arm_posture} failed: {exc}'
        if not moved:
            return False, f'{why}; the arms would not go back to {self._arm_posture}'
        ok, detail = self.arms_ok()
        if ok:
            return True, f'{detail} (after folding back to {self._arm_posture})'
        return False, f'{detail} even after folding back to {self._arm_posture}'

    def _gate(self, leg, label, destination):
        """Preconditions for a doorway transit. Refusing here beats scraping."""
        ok, detail = self.arms_ok()
        if not ok:
            ok, detail = self._try_narrowing(detail)
        if not ok:
            self.get_logger().error(f'ABORT before {label}: {detail}')
            self._status('aborted', f'arms: {detail}', destination=destination)
            return False
        self.get_logger().info(f'arms: {detail}')

        ok, detail = self.aligned_with(leg)
        if not ok:
            self.get_logger().error(f'ABORT before {label}: {detail}')
            self._status('aborted', f'alignment: {detail}', destination=destination)
            return False
        self.get_logger().info(f'alignment: {detail}')
        return True

    def _drive_through(self, group, label, destination):
        """Drive several poses as ONE goal, retrying once if it stops moving."""
        for attempt in (1, 2):
            outcome = self._drive_through_once(group, label, destination, attempt)
            if outcome != 'retry':
                return outcome
        return False

    def _drive_through_once(self, group, label, destination, attempt=1):
        """Drive several poses as ONE goal, passing through the intermediate ones.

        A route used to be one Nav2 goal per leg, so the robot parked on every
        corner to 5 cm and 1.4 deg and set off again - the stop-start motion the
        user objected to on 16. 9. Nav2 can already do better: with
        `navigate_through_poses` all but the last pose are waypoints it only has
        to pass, and the field of D-20 still shapes every metre of the path.

        Doorways are deliberately NOT merged into a group: a transit is gated
        before it starts and watched while it runs, and that stays exactly as it
        was (P-39). This only removes the stops that were never load-bearing.
        """
        goal = NavigateThroughPoses.Goal()
        for leg in group:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(leg.x)
            pose.pose.position.y = float(leg.y)
            pose.pose.orientation = yaw_to_quaternion(leg.yaw)
            goal.poses.append(pose)

        send = self._nav_through.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{label}: Nav2 rejected the route')
            self._status('failed', f'{label}: Nav2 rejected the route',
                         destination=destination)
            return False

        result = handle.get_result_async()
        deadline = time.monotonic() + self.get_parameter('leg_timeout').value * len(group)
        tightest = float('inf')
        # The same stall guard the single-pose path has. It belongs here too -
        # in fact it belongs here MORE. Three series in a row failed on this one
        # leg, the robot standing still for 170 s at 5 cm and 30 deg from its
        # goal, and neither guard fired because both lived in the other method
        # and a multi-pose leg never goes through it.
        still_since, still_at = None, None
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if attempt < 2:
                now, here = time.monotonic(), self.odom_pose()
                if here is not None:
                    if still_at is None or math.hypot(
                            here[0] - still_at[0],
                            here[1] - still_at[1]) > self.get_parameter('stall_radius').value \
                            or abs(wrap(here[2] - still_at[2])) > 0.05:
                        still_since, still_at = now, here
                    elif now - still_since > self.get_parameter('stall_seconds').value:
                        last = group[-1]
                        located = self.pose()
                        where = ''
                        if located is not None:
                            where = (f', {math.hypot(located[0] - last.x, located[1] - last.y) * 100:.1f} cm '
                                     f'and {math.degrees(wrap(located[2] - last.yaw)):+.1f} deg from the last pose')
                        self.get_logger().warn(
                            f'{label}: nothing has moved for {now - still_since:.0f} s'
                            f'{where}; asking Nav2 for the route again')
                        handle.cancel_goal_async()
                        self._spin(1.0)
                        return 'retry'
            if time.monotonic() > deadline:
                self._abort(handle, f'{label}: timed out', destination)
                return False
            if self._cancel:
                self._abort(handle, f'{label}: cancelled', destination, level='cancelled')
                return False
            clearance = self.side_clearance()
            if clearance is not None:
                tightest = min(tightest, min(clearance) - HALF_WIDTH)

        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'{label}: Nav2 finished with status {status}')
            self._status('failed', f'{label}: Nav2 status {status}',
                         destination=destination)
            return False
        if math.isfinite(tightest):
            self.get_logger().info(
                f'{label}: tightest gap beside the robot {tightest * 100:.1f} cm')
        return True

    # Nav2 picks the controller through the behaviour tree, and the goal carries
    # the tree's name, so the choice can be made per leg. A doorway transit is
    # left on the mission tree and its untouched FollowPath; everything else -
    # lining up, squaring up, docking - gets the approach tree, whose only
    # difference is a GoalAlign critic.
    #
    # The split exists because the two jobs are not the same. A transit is a
    # straight run down a measured centreline with centimetres to spare, and it
    # works; an approach has to ARRIVE, heading and all, and that is where a leg
    # sat 227 s of its 241 s timeout 5.5 cm and 12.3 deg from its pose on 18 Sep
    # with nothing in the critic list rewarding it to turn. Keeping the transit
    # out of the experiment is the point (P-39, D-18).
    ALIGN_TREE = 'navigate_to_pose_align.xml'

    def _tree_for(self, leg):
        """Which behaviour tree drives this leg; '' means Nav2's configured default."""
        if getattr(leg, 'transit', False):
            return ''
        share = get_package_share_directory('pas_dual_arm_bringup')
        return os.path.join(share, 'behavior_trees', self.ALIGN_TREE)

    def _square_up(self, leg, label):
        """Turn on the spot until the robot is square with the leg again.

        Nav2 must already have been cancelled: this publishes on the same topic
        it does, and two controllers on one base fight. Rotation only - the base
        is not moved off the line it is on, only turned back onto its heading.
        """
        deadline = time.monotonic() + 8.0
        gain = 1.2
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self.pose()
            if pose is None:
                continue
            error = wrap(pose[2] - leg.yaw)
            if abs(error) <= 0.5 * self.get_parameter('transit_heading_limit').value:
                self._cmd_vel.publish(Twist())
                self.get_logger().info(
                    f'{label}: squared up, now {math.degrees(error):+.1f} deg off')
                return True
            command = Twist()
            # Slow: the point is to stop drifting, not to swing the arms about.
            command.angular.z = max(-0.25, min(0.25, -gain * error))
            self._cmd_vel.publish(command)
        self._cmd_vel.publish(Twist())
        self.get_logger().warn(f'{label}: could not square up within 8 s')
        return False

    def _drive(self, leg, label, destination):
        # A transit may be driven twice: once, and once more after the robot has
        # been stopped and squared up because its heading drifted on the way
        # through. See the heading guard below and P-55.
        for attempt in (1, 2, 3):
            outcome = self._drive_once(leg, label, destination, attempt)
            if outcome != 'retry':
                return outcome
        return False

    def _drive_once(self, leg, label, destination, attempt=1):
        goal = NavigateToPose.Goal()
        goal.behavior_tree = self._tree_for(leg)
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(leg.x)
        goal.pose.pose.position.y = float(leg.y)
        goal.pose.pose.orientation = yaw_to_quaternion(leg.yaw)

        send = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{label}: Nav2 rejected the goal')
            self._status('failed', f'{label}: Nav2 rejected the goal', destination=destination)
            return False

        result = handle.get_result_async()
        deadline = time.monotonic() + self.get_parameter('leg_timeout').value
        tightest = float('inf')
        limit = self.get_parameter('min_side_clearance').value
        # Worst disagreement between where the lidar says the robot sits in the
        # opening and where the localised pose says it sits, as (gap, lidar, amcl).
        disagreement = None
        still_since, still_at = None, None
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.05)

            # Has it stopped moving while Nav2 still thinks it is driving?
            if attempt < 3:
                now, here = time.monotonic(), self.odom_pose()
                if here is not None:
                    if still_at is None or math.hypot(
                            here[0] - still_at[0],
                            here[1] - still_at[1]) > self.get_parameter('stall_radius').value \
                            or abs(wrap(here[2] - still_at[2])) > 0.05:
                        still_since, still_at = now, here
                    elif now - still_since > self.get_parameter('stall_seconds').value:
                        located = self.pose()
                        where = ''
                        if located is not None:
                            where = (f', {math.hypot(located[0] - leg.x, located[1] - leg.y) * 100:.1f} cm '
                                     f'and {math.degrees(wrap(located[2] - leg.yaw)):+.1f} deg from the goal')
                        self.get_logger().warn(
                            f'{label}: nothing has moved for '
                            f'{now - still_since:.0f} s{where}; asking Nav2 for it again')
                        handle.cancel_goal_async()
                        self._spin(1.0)
                        return 'retry'

            if time.monotonic() > deadline:
                self._abort(handle, f'{label}: timed out', destination)
                return False
            if self._cancel:
                self._abort(handle, f'{label}: cancelled', destination, level='cancelled')
                return False
            clearance = self.side_clearance()
            if clearance is None:
                continue
            left, right = clearance
            # Measured from the base centreline, so the gap beside the robot is
            # the reading minus its half width.
            gap = min(left, right) - HALF_WIDTH
            tightest = min(tightest, gap)
            if leg.transit:
                # Only inside the opening do the nearest returns beside the robot
                # mean the jambs; in the room they are the far walls and the
                # comparison is meaningless.
                span = left + right
                if math.isfinite(span) and span < DOORWAY_SPAN:
                    # Positive means displaced to the robot's left, same sign
                    # convention as lane_offset, so the two are comparable.
                    measured = (right - left) / 2.0
                    believed = self.lane_offset(leg)
                    if believed is not None:
                        delta = measured - believed
                        if disagreement is None or abs(delta) > abs(disagreement[0]):
                            disagreement = (delta, measured, believed, span)
            # The heading, watched all the way through rather than once before
            # the opening. Nav2 turns the robot during a transit - three bursts
            # of wz in the run that failed - and nothing in its critic list
            # scores the heading, so the drift is nobody's job to notice.
            if (leg.transit and attempt == 1
                    and self.get_parameter('transit_squares_up').value):
                pose = self.pose()
                if pose is not None:
                    drift = wrap(pose[2] - leg.yaw)
                    if abs(drift) > self.get_parameter('transit_heading_limit').value:
                        self.get_logger().warn(
                            f'{label}: {math.degrees(drift):+.1f} deg off square part '
                            f'way through, needing {swept_width(drift):.3f} m of the '
                            f'opening; stopping to square up and going again')
                        handle.cancel_goal_async()
                        self._spin(1.0)
                        self._square_up(leg, label)
                        return 'retry'

            if leg.transit and gap < limit:
                self._report_disagreement(label, disagreement)
                self._abort(handle,
                            f'{label}: only {gap * 100:.1f} cm beside the robot '
                            f'(left {left:.3f} m, right {right:.3f} m), limit '
                            f'{limit * 100:.1f} cm', destination)
                return False

        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'{label}: Nav2 finished with status {status}')
            self._status('failed', f'{label}: Nav2 status {status}', destination=destination)
            return False
        if math.isfinite(tightest):
            self.get_logger().info(
                f'{label}: tightest gap beside the robot {tightest * 100:.1f} cm')
        self._report_disagreement(label, disagreement)
        return True

    def _report_disagreement(self, label, disagreement):
        """Say how far the localised pose was from what the lidar measured.

        Run 59 aborted with the lidar reading the 1.00 m opening as 1.002 m and the
        robot 7.8 cm off its axis, while the localised pose put it on the axis.  The
        controller was doing what it was told; the pose it was told was wrong.  That
        was worked out afterwards, by hand, off a screenshot - so measure it in every
        transit instead, and from the lidar alone, which a physical robot also has.

        Reports only.  Nothing gates on it and nothing steers on it: turning this
        into a correction needs a run that shows the lidar number is the better one
        (D-18), and that run is the next increment, not this one.
        """
        if disagreement is None:
            return
        delta, measured, believed, span = disagreement
        self.get_logger().info(
            f'{label}: in a {span:.3f} m opening the lidar put the robot '
            f'{measured * 100:+.1f} cm off its axis, the localised pose '
            f'{believed * 100:+.1f} cm - they disagree by {delta * 100:+.1f} cm')

    def _abort(self, handle, reason, destination, level='aborted'):
        self.get_logger().error(reason)
        handle.cancel_goal_async()
        self._spin(1.0)
        self._status(level, reason, destination=destination)

    def _confirm_arrival(self, leg, label, destination):
        """Nav2 saying "succeeded" is a claim; this is the measurement.

        On 18 Sep a leg came back SUCCEEDED 0.6 s after it was sent, with the
        robot still 1.84 m and 90 deg from the goal, because the button had been
        pressed before the localiser was warm and the goal had been transformed
        with a stale `map -> odom`. The distance was already being computed and
        printed here; it was simply not being believed. It is now.
        """
        pose = self.pose()
        if pose is None:
            self.get_logger().error(
                f'{label}: Nav2 reported success but there is no pose to check it against')
            self._status('failed', f'{label}: no pose to confirm arrival',
                         destination=destination)
            return False
        offset = math.hypot(pose[0] - leg.x, pose[1] - leg.y)
        heading = wrap(pose[2] - leg.yaw)
        if offset > ARRIVAL_TOLERANCE or abs(heading) > ARRIVAL_YAW_TOLERANCE:
            detail = (f'{label}: Nav2 reported success {offset * 100:.1f} cm and '
                      f'{math.degrees(heading):+.1f} deg from the goal - it did not '
                      f'arrive (limits {ARRIVAL_TOLERANCE * 100:.0f} cm, '
                      f'{math.degrees(ARRIVAL_YAW_TOLERANCE):.0f} deg)')
            self.get_logger().error(detail)
            self._status('failed', detail, destination=destination)
            return False
        self.get_logger().info(
            f'{label}: arrived {offset * 100:.1f} cm and '
            f'{math.degrees(heading):+.1f} deg from the goal')
        return True


def main():
    rclpy.init()
    node = RoomNavigator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
