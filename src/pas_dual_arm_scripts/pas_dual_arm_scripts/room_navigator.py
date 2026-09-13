"""Drive room to room along the zone graph, one square leg at a time.

Replaces the hand-surveyed waypoints in `go_to_room.py`.  Every pose here comes
out of `/nav_graph`, which `nav_zones` derives from the doorways and tables the
SLAM map contains, so nothing has to be re-measured when the world changes.

A room-to-room trip is never one Nav2 goal.  A single goal across two rooms
gives the planner every reason to enter the doorway on a diagonal, because the
diagonal is shorter.  Instead each doorway becomes two goals on its centreline -
a portal pose about 2 m out on this side, then the matching pose on the far side -
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
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from pas_dual_arm_scripts.postures import ARM_DRIVE, POSTURES

# Nav2 footprint, so the geometry here and the costmap's agree.
HALF_LENGTH, HALF_WIDTH = 0.52, 0.427


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def swept_width(yaw_error, length=2 * HALF_LENGTH, width=2 * HALF_WIDTH):
    """How wide an opening this robot needs at a given heading error."""
    return length * abs(math.sin(yaw_error)) + width * abs(math.cos(yaw_error))


class Leg:
    def __init__(self, pose, label, transit=False):
        self.x, self.y, self.yaw = pose
        self.label = label
        self.transit = transit


class RoomNavigator(Node):
    def __init__(self):
        super().__init__('room_navigator')
        self.declare_parameter('arm_tolerance', 0.15)
        self.declare_parameter('centreline_tolerance', 0.08)
        self.declare_parameter('heading_tolerance', 0.07)
        self.declare_parameter('min_side_clearance', 0.03)
        # Below this a leg counts as axis aligned already; it also absorbs the
        # few centimetres between a detected door centre and a table centre.
        self.declare_parameter('corner_threshold', 0.30)
        self.declare_parameter('leg_timeout', 240.0)
        self.declare_parameter('require_arms', True)

        self._graph = None
        self._scan = None
        self._joints = None
        self._request = None
        self._cancel = False
        self._busy = False

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, '/room_navigator/status', latched)
        self.create_subscription(String, '/nav_graph', self._on_graph, latched)
        self.create_subscription(String, '/room_navigator/goto', self._on_goto, 10)
        self.create_subscription(LaserScan, '/scan_filtered', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
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
        if request in ('stop', 'cancel', 'abort'):
            self._cancel = True
            self.get_logger().warn('cancel requested')
            return
        if self._busy:
            self.get_logger().warn(f'busy; ignoring "{request}"')
            return
        self._request = request

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
        """Every ARM_DRIVE joint within tolerance, or a reason why not."""
        if not self.get_parameter('require_arms').value:
            return True, 'arm check disabled'
        if self._joints is None:
            return False, 'no /joint_states yet'
        tolerance = self.get_parameter('arm_tolerance').value
        worst_name, worst = None, 0.0
        for side, targets in POSTURES[ARM_DRIVE].items():
            for index, target in targets.items():
                name = f'{side}_joint_{index}'
                if name not in self._joints:
                    return False, f'{name} missing from /joint_states'
                error = abs(self._joints[name] - target)
                if error > worst:
                    worst_name, worst = name, error
        if worst > tolerance:
            return False, (f'{worst_name} is {worst:.3f} rad off {ARM_DRIVE} '
                           f'(limit {tolerance:.2f}); the arms set the width that '
                           f'has to fit the doorway')
        return True, f'{ARM_DRIVE} held, worst joint {worst:.3f} rad'

    def aligned_with(self, leg):
        """On the lane centreline and square to it, within the doorway budget."""
        pose = self.pose()
        if pose is None:
            return False, 'no map -> base_footprint transform'
        heading_error = wrap(pose[2] - leg.yaw)
        # Distance from the leg's centreline, which runs along its own heading.
        offset = abs(-(pose[0] - leg.x) * math.sin(leg.yaw)
                     + (pose[1] - leg.y) * math.cos(leg.yaw))
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

    def plan(self, destination):
        """Legs from where the robot is to `destination`, or (None, reason)."""
        if self._graph is None:
            return None, 'no zone graph yet - is nav_zones running and /map published?'
        known = [room['name'] for room in self._graph['rooms']]
        if destination not in known:
            return None, f'unknown room "{destination}"; known rooms: {", ".join(known)}'
        pose = self.pose()
        if pose is None:
            return None, 'no map -> base_footprint transform; is AMCL localised?'
        start = self.room_at(pose[0], pose[1])
        if start is None:
            return None, (f'robot at ({pose[0]:.2f}, {pose[1]:.2f}) is not inside '
                          f'any detected room')
        path = self._room_path(start, destination)
        if path is None:
            return None, f'no doorway chain from {start} to {destination}'

        legs = []
        for here, there in zip(path, path[1:]):
            door = self._door_between(here, there)
            portal = (door.get('portals') or {}).get(here)
            if portal is None:
                return None, f'door between {here} and {there} has no portal pose for {here}'
            legs.append(Leg(portal['approach'], f'{here} -> {there}: line up at the doorway'))
            legs.append(Leg(portal['exit'], f'{here} -> {there}: straight through the doorway',
                            transit=True))

        table = next((t for t in self._graph['tables'] if t['room'] == destination), None)
        if table is not None:
            legs.append(Leg(table['approach'], f'{destination}: square up in front of the table'))
        elif not legs:
            room = next(r for r in self._graph['rooms'] if r['name'] == destination)
            legs.append(Leg((room['centre'][0], room['centre'][1], pose[2]),
                            f'{destination}: room centre'))
        else:
            room = next(r for r in self._graph['rooms'] if r['name'] == destination)
            legs.append(Leg((room['centre'][0], room['centre'][1], legs[-1].yaw),
                            f'{destination}: room centre'))
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
        if request is None:
            return
        self._cancel = False
        self._busy = True
        try:
            self.run(request)
        finally:
            self._busy = False

    def run(self, destination):
        legs, detail = self.plan(destination)
        if legs is None:
            self.get_logger().error(f'cannot go to "{destination}": {detail}')
            self._status('failed', detail, destination=destination)
            return
        self.get_logger().info(f'going to {destination}: {detail}')
        if not self._nav.wait_for_server(timeout_sec=20.0):
            self._status('failed', 'Nav2 /navigate_to_pose is not up', destination=destination)
            self.get_logger().error('Nav2 /navigate_to_pose is not up')
            return

        for index, leg in enumerate(legs, start=1):
            label = f'leg {index}/{len(legs)} - {leg.label}'
            if self._cancel:
                self._status('cancelled', f'cancelled before {label}', destination=destination)
                return
            if leg.transit and not self._gate(leg, label, destination):
                return
            self._status('driving', label, destination=destination,
                         goal=[leg.x, leg.y, leg.yaw])
            self.get_logger().info(
                f'{label} -> ({leg.x:+.2f}, {leg.y:+.2f}, {math.degrees(leg.yaw):+.1f} deg)')
            if not self._drive(leg, label, destination):
                return
            self._report_arrival(leg, label)

        self._status('arrived', f'reached {destination}', destination=destination)
        self.get_logger().info(f'reached {destination}')

    def _gate(self, leg, label, destination):
        """Preconditions for a doorway transit. Refusing here beats scraping."""
        ok, detail = self.arms_ok()
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

    def _drive(self, leg, label, destination):
        goal = NavigateToPose.Goal()
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
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.05)
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
            if gap < limit:
                self._abort(handle,
                            f'{label}: only {gap * 100:.1f} cm beside the robot '
                            f'(left {left:.3f} m, right {right:.3f} m), limit '
                            f'{limit * 100:.0f} cm', destination)
                return False

        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'{label}: Nav2 finished with status {status}')
            self._status('failed', f'{label}: Nav2 status {status}', destination=destination)
            return False
        if math.isfinite(tightest):
            self.get_logger().info(f'{label}: tightest gap beside the robot {tightest * 100:.1f} cm')
        return True

    def _abort(self, handle, reason, destination, level='aborted'):
        self.get_logger().error(reason)
        handle.cancel_goal_async()
        self._spin(1.0)
        self._status(level, reason, destination=destination)

    def _report_arrival(self, leg, label):
        pose = self.pose()
        if pose is None:
            return
        offset = math.hypot(pose[0] - leg.x, pose[1] - leg.y)
        heading = wrap(pose[2] - leg.yaw)
        self.get_logger().info(
            f'{label}: arrived {offset * 100:.1f} cm and '
            f'{math.degrees(heading):+.1f} deg from the goal')


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
