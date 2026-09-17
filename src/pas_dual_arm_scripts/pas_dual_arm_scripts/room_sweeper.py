#!/usr/bin/env python3
"""Map a building by covering each room, then stepping through its doorways.

This replaces frontier exploration for this world, and the reason is worth
stating plainly, because frontier search is the textbook answer (Yamauchi,
1997) and abandoning it needs a better argument than "it looked bad".

**A frontier is a poor target when the sensor outranges the room.** The lidar
reaches 25 m and a room here is 6 m, so the robot sees the whole room the
instant it arrives. What is left unmapped is not "far away", it is *occluded* -
the shadow behind a table leg, the strip behind the robot's own body. Those
shadows form one connected ring of frontier around the robot, and a search that
scores a cluster by size and steers at its centroid computes the centroid of a
ring, which is the middle of the ring: where the robot already stands. Measured
in the run of 17 Sep 23:33 - one cluster of 4127 cells, goal chosen 0.83 m away
at 136 degrees, 90 s spent on it, and then the same thing again.

**Coverage does not have that failure mode.** Drive the room wall to wall on a
fixed grid of lanes, and the shadows get looked at from a second angle because
the robot passes them from somewhere else. The motion is decided before the
robot moves, so it can be drawn, checked, and watched - and that matters as
much as the map, because a mapping run that nobody can follow is a run nobody
can debug.

**The lanes exploit the base.** This is a mecanum base, so a lane and the step
to the next lane are both pure translation: drive +x to the wall, strafe -y one
lane, drive -x to the wall. The heading never changes during a sweep. That is
not only tidy - turning in place is the one motion where this base slips
(mu2 = 0, P-10), and every slip feeds yaw error into the scan matcher (P-11).
A sweep with zero in-place rotation is the best-conditioned input SLAM can get,
and the resulting map is the one the mission later drives a 0.98 m doorway on.

Doorways are the exception, and they stay on Nav2. The corridor for the robot's
*centre* through a 1.0 m opening is about 12 cm; P-45 measured that a custom
controller cannot hold it and that Nav2 drives the same opening 5/5 because it
checks the footprint polygon rather than following a gradient. So: sweeping in
open space is driven here, and every doorway transit is a Nav2 goal.

The loop is
    detect passages -> seal them -> sweep the room they enclose ->
    patch what stayed unknown -> step through an unvisited passage -> repeat
and it ends when no passage is unvisited and no reachable unknown is left.
Nothing in it knows how many rooms there are, what they are called, or where.
"""
import json
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from pas_dual_arm_scripts.feature_registry import door_candidates

FREE, UNKNOWN = 0, -1
OCCUPIED_FROM = 65


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ------------------------------------------------------------- pure array
# Module level and taking plain arrays, so the offline checker runs exactly the
# code the robot runs. A harness with its own copy of the rules is a harness
# that passes while the robot stalls.

def fits_mask(grid, resolution, clearance):
    """Cells whose distance to the nearest wall exceeds `clearance`.

    Unknown counts as passable on purpose: the unseen pockets inside a room are
    the reason for sweeping it. Walls are what the robot must not touch, and a
    wall is something the map has actually observed as occupied.
    """
    distance = ndimage.distance_transform_edt(grid < OCCUPIED_FROM) * resolution
    return distance > clearance


def seal_mask(grid, info, passages, thickness=0.30, overhang=0.40):
    """Cells to treat as wall so a flood fill stops at a doorway.

    Without this the free space seen *through* an opening joins the next room
    onto this one, and "sweep the room I am in" quietly becomes "sweep the
    building in one pass" - lanes that cross a doorway diagonally, which is the
    one thing this robot cannot do.
    """
    mask = np.zeros(grid.shape, dtype=bool)
    res = info.resolution
    xs = info.origin.position.x + (np.arange(info.width) + 0.5) * res
    ys = info.origin.position.y + (np.arange(info.height) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    for passage in passages:
        nx, ny = passage['normal']
        along = abs(gx - passage['x']) * abs(ny) + abs(gy - passage['y']) * abs(nx)
        across = abs(gx - passage['x']) * abs(nx) + abs(gy - passage['y']) * abs(ny)
        mask |= (along <= passage['width'] / 2.0 + overhang) & (across <= thickness / 2.0)
    return mask


def seed_cell(info, x, y):
    return (int((y - info.origin.position.y) / info.resolution),
            int((x - info.origin.position.x) / info.resolution))


def room_region(grid, info, seed_rc, sealed):
    """The free space the robot can reach without crossing a sealed doorway."""
    free = (grid == FREE) & ~sealed
    labels, count = ndimage.label(free, structure=np.ones((3, 3)))
    if count == 0:
        return np.zeros(grid.shape, dtype=bool)
    row, col = seed_rc
    label = labels[row, col] if (0 <= row < grid.shape[0]
                                 and 0 <= col < grid.shape[1]) else 0
    if label == 0:
        # The cells under the robot can still be unknown - scan_filter blanks
        # returns inside the footprint, so nothing raytraces them clear. Take
        # the nearest labelled cell instead of giving up.
        window = max(2, int(round(1.0 / info.resolution)))
        r0, r1 = max(0, row - window), min(grid.shape[0], row + window + 1)
        c0, c1 = max(0, col - window), min(grid.shape[1], col + window + 1)
        near = labels[r0:r1, c0:c1]
        nonzero = near[near > 0]
        if nonzero.size == 0:
            return np.zeros(grid.shape, dtype=bool)
        label = int(np.bincount(nonzero).argmax())
    return labels == label


def unknown_reachable(grid, info, seed_rc, clearance, sealed=None):
    """Unknown cells touching the free space the robot can actually stand in.

    This is the stopping condition, and it is stronger than "I have no frontier
    I like": a cluster that is too small, too tight or blacklisted disappears
    from a frontier search without ever being looked at, and the run then calls
    itself finished. Counting the unknown that remains *reachable* cannot be
    satisfied by losing interest.

    `sealed` answers a different question with the same code. Passed the doorway
    seals, the count stops at the walls of the room the robot is in, which is
    "are there holes in *this* room" - the question that decides whether to walk
    it again. Left out, it spans the building, which is "is the map finished".
    Using the building-wide number to decide a second pass is what made the
    first run sweep the starting room twice over: the 4485 unknown cells it was
    chasing were the two rooms nobody had entered yet.
    """
    standable = (grid == FREE) & fits_mask(grid, info.resolution, clearance)
    if sealed is not None:
        standable &= ~sealed
    labels, count = ndimage.label(standable, structure=np.ones((3, 3)))
    if count == 0:
        return 0
    row, col = seed_rc
    label = labels[row, col] if (0 <= row < grid.shape[0]
                                 and 0 <= col < grid.shape[1]) else 0
    if label == 0:
        nonzero = labels[labels > 0]
        if nonzero.size == 0:
            return 0
        label = int(np.bincount(nonzero).argmax())
    reach = ndimage.binary_dilation(labels == label, structure=np.ones((3, 3)))
    return int(((grid == UNKNOWN) & reach).sum())


def interest_mask(grid, region, sealed=None, grow=3):
    """Unknown cells that belong to this room - the only reason to drive.

    Unknown beyond a wall is someone else's room and must not pull a lane at
    it; unknown beyond a doorway belongs to the room we have not entered, so
    the seals exclude it too. What is left is what the sweep is for: the
    shadows behind furniture, and the pockets a single viewpoint could not see
    into.
    """
    near = ndimage.binary_dilation(region, structure=np.ones((3, 3)), iterations=grow)
    unknown = (grid == UNKNOWN) & near
    if sealed is not None:
        unknown &= ~sealed
    return unknown


def sweep_waypoints(grid, info, region, robot_xy, lane_spacing, clearance,
                    min_run, offset=0.0, interest=None, interest_radius=None):
    """Boustrophedon waypoints covering `region`, on the map's own axes.

    Lanes run along the region's longer side, so a long room is crossed the few
    long ways rather than the many short ways. `offset` shifts the whole lane
    set, which is how a second pass looks into the shadows the first one left:
    same room, different viewing angles, no special case for tables.

    With `interest`, a lane is trimmed to the stretches that can still reveal
    something, and a lane that can reveal nothing is not driven at all. This is
    not an optimisation, it is the difference between covering a room and
    performing covering it: a 25 m lidar sees an empty 6 m room from anywhere in
    it, so driving all four lanes of one buys nothing and costs a hundred
    seconds - measured on 18 Sep, where the room was fully known before the
    second lane and the robot drove the other three anyway.
    """
    rows, cols = np.nonzero(region)
    if rows.size == 0:
        return []
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    fits = fits_mask(grid, res, clearance)
    if interest is not None:
        radius = interest_radius if interest_radius is not None else lane_spacing
        if not interest.any():
            return []
        reach = ndimage.distance_transform_edt(~interest) * res
        fits = fits & (reach <= radius)

    r0, r1 = int(rows.min()), int(rows.max())
    c0, c1 = int(cols.min()), int(cols.max())
    along_x = (c1 - c0) >= (r1 - r0)
    step = max(1, int(round(lane_spacing / res)))
    shift = int(round(offset / res)) % step

    lo, hi = (r0, r1) if along_x else (c0, c1)
    lanes = list(range(lo + shift + step // 2, hi + 1, step)) or [(lo + hi) // 2]

    # Start at the end the robot is already nearest, so the first move is not a
    # drive across the whole room to reach lane one.
    robot_rc = seed_cell(info, *robot_xy)
    here = robot_rc[0] if along_x else robot_rc[1]
    if abs(here - lanes[-1]) < abs(here - lanes[0]):
        lanes.reverse()

    min_cells = max(1, int(round(min_run / res)))
    waypoints, flip = [], False
    for lane in lanes:
        line = fits[lane, c0:c1 + 1] if along_x else fits[r0:r1 + 1, lane]
        base = c0 if along_x else r0
        index = np.flatnonzero(line)
        if index.size == 0:
            continue
        breaks = np.flatnonzero(np.diff(index) > 1)
        starts = np.concatenate(([0], breaks + 1))
        ends = np.concatenate((breaks, [index.size - 1]))
        points = []
        for first, last in zip(starts, ends):
            a, b = base + int(index[first]), base + int(index[last])
            if b - a < min_cells:
                continue
            if along_x:
                points += [(ox + (a + 0.5) * res, oy + (lane + 0.5) * res),
                           (ox + (b + 0.5) * res, oy + (lane + 0.5) * res)]
            else:
                points += [(ox + (lane + 0.5) * res, oy + (a + 0.5) * res),
                           (ox + (lane + 0.5) * res, oy + (b + 0.5) * res)]
        if flip:
            points.reverse()
        waypoints += points
        flip = not flip
    return waypoints


def opening_width(grid, info, passage, reach=1.2):
    """Measure the opening on the map as it is now, across its own axis.

    The width recorded when a passage was first detected came from a map that
    has been growing ever since. Re-measuring before committing costs one array
    slice and is the difference between "the detector once thought this was a
    door" and "the robot fits through this today".

    Returns None when the centre is not on the map any more.
    """
    nx, ny = passage['normal']
    res = info.resolution
    row, col = seed_cell(info, passage['x'], passage['y'])
    if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
        return None
    steps = int(reach / res)
    # Travel is along the normal, so the opening runs across it.
    dr, dc = (0, 1) if abs(nx) < abs(ny) else (1, 0)
    span = 0
    for direction in (1, -1):
        for step in range(1, steps + 1):
            r, c = row + dr * step * direction, col + dc * step * direction
            if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
                break
            if grid[r, c] >= OCCUPIED_FROM:
                break
            span += 1
    return (span + 1) * res


def line_is_clear(grid, info, start, end, clearance, blocked=None):
    """Can the robot translate straight from `start` to `end`?

    Sampled rather than rasterised: at 2 cm cells a 6 m leg is 300 samples, and
    the test is only ever asked before a move, not in the control loop.

    `blocked` is where a straight translation is forbidden regardless of how
    much room there is - in practice the doorway seals, because a doorway is
    driven by Nav2 with a footprint check and never glided through. Relying on
    the clearance number alone to keep a glide out of a doorway would work by
    1 cm: the widest gap in this world's 0.98 m opening measures 0.490 m and
    the threshold is 0.50. A 1.05 m door would quietly become glidable.
    """
    fits = fits_mask(grid, info.resolution, clearance)
    if blocked is not None:
        fits &= ~blocked
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(2, int(length / info.resolution) + 1)
    for i in range(steps + 1):
        t = i / steps
        row, col = seed_cell(info, start[0] + t * (end[0] - start[0]),
                             start[1] + t * (end[1] - start[1]))
        if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
            return False
        if not fits[row, col]:
            return False
    return True


class RoomSweeper(Node):
    def __init__(self):
        super().__init__('room_sweeper')
        # --- sweep geometry -------------------------------------------------
        # Lane spacing is not a sensor range: the lidar sees the whole room from
        # anywhere in it. It is a *parallax* step - how far the robot moves
        # before it looks behind a table leg from a usefully different angle.
        self.declare_parameter('lane_spacing', 1.5)
        # Above the robot's half length (0.52 m), so an axis-aligned body clears
        # the wall at any of the four headings a sweep can hold. It also keeps
        # lanes out of a 1.0 m doorway, whose centreline is only 0.5 m from each
        # jamb - which is what stops a lane crossing into the next room.
        self.declare_parameter('clearance', 0.60)
        self.declare_parameter('min_run', 0.60)
        # Reachability for the progress number is a different question from
        # where a lane may go, and needs a different margin. `clearance` is
        # deliberately wider than a 1.0 m doorway's half width so a lane never
        # wanders into one; asking "could the robot ever get there" with that
        # same number would declare a whole unvisited room unreachable and
        # report the map closed while a door stands open. This one is just over
        # the inscribed radius (0.427 m), which is what actually fits.
        self.declare_parameter('closure_clearance', 0.44)
        self.declare_parameter('patch_passes', 1)
        # --- driving --------------------------------------------------------
        self.declare_parameter('sweep_speed', 0.22)
        self.declare_parameter('approach_speed', 0.08)
        self.declare_parameter('arrive_tolerance', 0.12)
        self.declare_parameter('yaw_hold_gain', 1.0)
        self.declare_parameter('max_yaw_rate', 0.20)
        self.declare_parameter('stop_distance', 0.65)
        # Straighter test than lane placement. `clearance` keeps lanes a lane's
        # worth away from walls; asking the same of the line between two lane
        # ends sent half the legs to the planner in the first run ("straight
        # line blocked"), which is slow and is exactly the driving we replaced.
        # Still above a 1.0 m doorway's half width, so a glide can never take a
        # short cut through a door.
        self.declare_parameter('line_clearance', 0.50)
        self.declare_parameter('guard_sector', 0.35)
        self.declare_parameter('leg_timeout', 60.0)
        # --- doorways (these go to Nav2, never driven here) ------------------
        self.declare_parameter('door_standoff', 1.10)
        # How far past the centre to stop, on the far side. Shorter than the
        # near standoff on purpose: that pose is in a room nobody has seen, so
        # every extra centimetre is a guess. Wall 0.10 m + half the robot
        # 0.52 m + margin is enough to be through, and the sweep of the new room
        # takes over from there.
        self.declare_parameter('door_exit', 0.90)
        # The drive posture measures 0.821 m across (DRIVE_V4). An opening has
        # to beat that by a real margin before the robot is sent at it.
        self.declare_parameter('min_passage_width', 0.90)
        self.declare_parameter('door_match_radius', 1.20)
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('door_attempts', 2)
        # --- pacing ---------------------------------------------------------
        self.declare_parameter('start_delay', 8.0)
        self.declare_parameter('settle_time', 1.5)
        self.declare_parameter('scan_stale_timeout', 5.0)
        # Unknown cells still touching reachable free space that we accept as
        # "closed". At 0.02 m a cell is 4 cm^2, so 200 cells is 0.08 m^2 - the
        # width of the sensor's own blind spot, not a room.
        self.declare_parameter('unknown_done', 200)
        # A room is worth a bounded amount of time. Without this the run can
        # spend its whole life perfecting the room it started in and never open
        # a door - observed 18 Sep. Reaching it is not a failure; it just means
        # the next opening is now worth more than another lane here.
        self.declare_parameter('room_budget', 240.0)
        # The map is built by SLAM as the robot stands there, so planning a
        # route in the first second plans it over whatever has been integrated
        # so far - 5.0 m^2 of a 34 m^2 room, in the first run. Wait until the
        # known area stops growing before deciding where to drive.
        self.declare_parameter('map_settle_timeout', 25.0)
        self.declare_parameter('map_settle_quiet', 3.0)
        self.declare_parameter('map_settle_ratio', 0.02)
        # Waiting for the map is not enough on its own. On 18 Sep the wait ended
        # honestly - the map really had stopped changing - and the room was
        # still only 5.0 m2 of 34, because SLAM adds a scan to the graph only
        # once the robot has moved (minimum_travel_distance), so a stationary
        # robot's map settles *early* rather than *complete*. The route has to
        # be allowed to notice the room getting bigger underneath it.
        self.declare_parameter('replan_growth', 1.4)
        self.declare_parameter('max_replans', 3)

        self.p = {name: self.get_parameter(name).value for name in (
            'lane_spacing', 'clearance', 'min_run', 'closure_clearance',
            'patch_passes',
            'sweep_speed', 'approach_speed', 'arrive_tolerance',
            'yaw_hold_gain', 'max_yaw_rate', 'stop_distance', 'guard_sector',
            'leg_timeout', 'door_standoff', 'door_match_radius',
            'goal_timeout', 'door_attempts', 'start_delay', 'settle_time',
            'door_exit', 'min_passage_width',
            'scan_stale_timeout', 'unknown_done', 'line_clearance',
            'room_budget', 'map_settle_timeout', 'map_settle_quiet',
            'map_settle_ratio', 'replan_growth', 'max_replans')}

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = None
        self.scan = None
        self.scan_stamp = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(LaserScan, '/scan_filtered', self._on_scan,
                                 qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/exploration/status', 10)
        # Into the collision monitor, not past it: the monitor reads /cmd_vel
        # and publishes /cmd_vel_safe, and cmd_vel_relay prefers the safe topic.
        # Publishing anywhere else would be a way of driving around the guard.
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.passages = []          # every opening ever seen, with visited flag
        self.stalled_reason = None

    # --------------------------------------------------------------- inputs
    def _on_map(self, msg):
        self.map = msg

    def _on_scan(self, msg):
        self.scan = msg
        self.scan_stamp = self.get_clock().now()

    def _say(self, state, detail, **extra):
        payload = {'state': state, 'detail': detail}
        payload.update(extra)
        self.status.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'SWEEP {state}: {detail}')

    def pose(self):
        try:
            tf = self._tf.lookup_transform('map', 'base_footprint',
                                           rclpy.time.Time()).transform
        except Exception:
            return None
        q = tf.rotation
        return (tf.translation.x, tf.translation.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _grids(self):
        info = self.map.info
        grid = np.asarray(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        return info, grid

    def seals(self):
        """Doorway seals, rebuilt against the map as it is right now.

        Never cached. The SLAM map grows, so its grid changes shape mid-run, and
        a mask kept from an earlier sweep stops matching it - which is exactly
        how the 18 Sep run ended: `operands could not be broadcast together with
        shapes (596,597) (595,595)`, thrown in the closure check, killing the
        node before it ever got to choose a doorway.
        """
        info, grid = self._grids()
        return seal_mask(grid, info, self.passages)

    def scan_age(self):
        """Seconds since the last scan. The one signal everything hangs on.

        Watching `map -> odom` instead does not work: slam_toolbox republishes
        that transform at 50 Hz with a fresh stamp whether or not a new scan
        ever arrives, so it stays young while the robot is blind (P-47).
        """
        if self.scan_stamp is None:
            return None
        return (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9

    def _scan_stale(self):
        age = self.scan_age()
        if age is None or age <= self.p['scan_stale_timeout']:
            return False
        self.stalled_reason = (
            f'/scan_filtered nije stigao {age:.1f} s — skenovi su stali. '
            f'Pokreni scripts/scan_watch.py i vidi puca li senzor, most ili DDS (P-47).')
        return True

    # ------------------------------------------------------------- passages
    def update_passages(self):
        """Merge newly visible openings into the registry, keeping visited flags.

        `both_sides=False` is the whole point: while mapping, the far side of a
        door has not been seen, and the strict test finds nothing at all - which
        is exactly what nav_zones logged for a full run on 17 Sep.
        """
        found = door_candidates(self.map, both_sides=False)
        for door in found:
            match = next((p for p in self.passages
                          if math.hypot(p['x'] - door['x'], p['y'] - door['y'])
                          < self.p['door_match_radius']), None)
            if match is None:
                self.passages.append(dict(door, visited=False, failures=0))
            else:
                # Keep the better centre once both sides are known, never the
                # visited flag - re-detecting an opening is not un-driving it.
                match['x'], match['y'] = door['x'], door['y']
                match['width'] = door['width']
        return self.passages

    def unvisited(self, robot):
        options = [p for p in self.passages
                   if not p['visited'] and p['failures'] < self.p['door_attempts']]
        return sorted(options, key=lambda p: math.hypot(p['x'] - robot[0],
                                                        p['y'] - robot[1]))

    # -------------------------------------------------------------- driving
    def _clear_ahead(self, bx, by):
        """Metres to the nearest return in the direction of travel, base frame.

        The lidar shares base_link's axes and sits at its centre, so a reading
        of r means r - 0.52 m of air in front of the bumper on the long axis.
        This is a backstop with a wide margin; the collision monitor is the
        guarantee.
        """
        if self.scan is None:
            return math.inf
        heading = math.atan2(by, bx)
        ranges = np.asarray(self.scan.ranges, dtype=np.float32)
        angles = self.scan.angle_min + np.arange(ranges.size) * self.scan.angle_increment
        sector = np.abs(np.arctan2(np.sin(angles - heading),
                                   np.cos(angles - heading))) <= self.p['guard_sector']
        valid = sector & np.isfinite(ranges) & (ranges > self.scan.range_min)
        return float(ranges[valid].min()) if valid.any() else math.inf

    def stop(self):
        """Zero velocity, and never raise on the way out.

        Called from the shutdown path too, where the context may already be
        gone; a traceback there makes every clean run end looking like a crash.
        """
        try:
            self.cmd.publish(Twist())
        except Exception:
            pass

    def glide(self, x, y, hold_yaw, label='lane'):
        """Translate to (x, y) holding a heading. No turn, no path, no planner.

        Deliberately the simplest thing that can work, and only ever used in
        open space: a straight line the map already says is clear, on a base
        that can move along it without rotating. The only feedback term is the
        heading hold, which exists because wheel slip would otherwise let the
        body drift round over a 6 m leg - it is not a position regulator, and
        the translation is open-loop constant speed with a taper at the end.
        """
        deadline = time.monotonic() + self.p['leg_timeout']
        rate_period = 0.05
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=rate_period)
            if self._scan_stale():
                self.stop()
                return False
            robot = self.pose()
            if robot is None:
                continue
            ex, ey = x - robot[0], y - robot[1]
            distance = math.hypot(ex, ey)
            if distance <= self.p['arrive_tolerance']:
                self.stop()
                return True
            if time.monotonic() > deadline:
                self.stop()
                self.get_logger().warn(
                    f'{label}: {distance:.2f} m short after '
                    f'{self.p["leg_timeout"]:.0f} s, giving the leg up')
                return False
            # Error into the body frame; on a mecanum base that is the command.
            cos_yaw, sin_yaw = math.cos(robot[2]), math.sin(robot[2])
            bx = cos_yaw * ex + sin_yaw * ey
            by = -sin_yaw * ex + cos_yaw * ey
            if self._clear_ahead(bx, by) < self.p['stop_distance']:
                self.stop()
                self.get_logger().warn(f'{label}: something in the way, stopping the leg')
                return False
            speed = min(self.p['sweep_speed'],
                        max(self.p['approach_speed'], distance))
            twist = Twist()
            twist.linear.x = speed * bx / distance
            twist.linear.y = speed * by / distance
            twist.angular.z = max(-self.p['max_yaw_rate'],
                                  min(self.p['max_yaw_rate'],
                                      self.p['yaw_hold_gain'] * wrap(hold_yaw - robot[2])))
            self.cmd.publish(twist)
        self.stop()
        return False

    def nav_goal(self, x, y, yaw, label):
        """Hand the move to Nav2. Used where the footprint has to be checked."""
        if not self.nav.wait_for_server(timeout_sec=20.0):
            self.stalled_reason = 'Nav2 navigate_to_pose nije dostupan.'
            return False
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = yaw_to_quaternion(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'{label}: goal rejected')
            return False
        result = handle.get_result_async()
        deadline = time.monotonic() + self.p['goal_timeout']
        while rclpy.ok() and not result.done():
            if self._scan_stale() or time.monotonic() > deadline:
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel)
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            names = {GoalStatus.STATUS_ABORTED: 'Nav2 ABORTED (planer ili upravljač odustao)',
                     GoalStatus.STATUS_CANCELED: 'Nav2 CANCELED',
                     GoalStatus.STATUS_UNKNOWN: 'Nav2 UNKNOWN'}
            self.get_logger().warn(
                f'{label}: {names.get(status, f"Nav2 status {status}")} '
                f'— cilj ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        return status == GoalStatus.STATUS_SUCCEEDED

    def move_to(self, x, y, hold_yaw, label='move'):
        """Straight line if the map allows one, Nav2 if it does not."""
        robot = self.pose()
        if robot is None:
            return False
        info, grid = self._grids()
        if line_is_clear(grid, info, (robot[0], robot[1]), (x, y),
                         self.p['line_clearance'], self.seals()):
            return self.glide(x, y, hold_yaw, label)
        self.get_logger().info(f'{label}: straight line blocked, planning instead')
        return self.nav_goal(x, y, hold_yaw, label)

    # --------------------------------------------------------------- sweeps
    def wait_for_map(self):
        """Let SLAM finish drawing the room before planning a route across it.

        A 360 degree lidar sees the whole room from the doorway, but the map is
        built scan by scan, and the first route of the first run was planned
        over 5.0 m2 of a 34 m2 room - six waypoints in a corner, then a second
        pass once the rest existed. Waiting a few seconds for the known area to
        level off costs less than sweeping a room twice.
        """
        deadline = time.monotonic() + self.p['map_settle_timeout']
        quiet_for = 0.0
        last = -1
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.map is None:
                continue
            _info, grid = self._grids()
            known = int((grid != UNKNOWN).sum())
            if last > 0 and abs(known - last) <= self.p['map_settle_ratio'] * last:
                quiet_for += 0.2
                if quiet_for >= self.p['map_settle_quiet']:
                    return known
            else:
                quiet_for = 0.0
            last = known
        return last

    def plan_route(self, offset):
        """The room the robot is in right now, and the lanes that cover it."""
        info, grid = self._grids()
        robot = self.pose()
        if robot is None:
            return None, [], 0.0
        sealed = seal_mask(grid, info, self.update_passages())
        region = room_region(grid, info, seed_cell(info, robot[0], robot[1]), sealed)
        if not region.any():
            return region, [], 0.0
        interest = interest_mask(grid, region, sealed)
        waypoints = sweep_waypoints(grid, info, region, (robot[0], robot[1]),
                                    self.p['lane_spacing'], self.p['clearance'],
                                    self.p['min_run'], offset, interest,
                                    self.p['lane_spacing'])
        return region, waypoints, float(region.sum()) * info.resolution ** 2

    def sweep(self, hold_yaw, offset=0.0, tag='sweep', deadline=None):
        """Cover the room the robot is standing in, lane by lane.

        The route is re-planned if the room turns out to be bigger than it
        looked: SLAM fills the map in as the robot drives, so the first plan of
        a run can be drawn over a fraction of the room. Re-planning is bounded,
        because a route that keeps restarting is its own kind of stuck.

        Returns the number of lane ends actually reached, so the caller can tell
        a room that was swept from one the robot could not move in at all.
        """
        reached, replans = 0, 0
        while rclpy.ok():
            region, waypoints, area = self.plan_route(offset)
            if region is None:
                return reached
            if not waypoints:
                self._say('covered',
                          'U ovoj sobi nema više nepoznatog na koji bi se traka isplatila.')
                return reached
            self._say('sweeping',
                      f'{tag}: soba {area:.1f} m², {len(waypoints)} točaka '
                      f'u trakama po {self.p["lane_spacing"]:.1f} m',
                      waypoints=len(waypoints), area=round(area, 1))
            grew = False
            for index, (x, y) in enumerate(waypoints, start=1):
                if self._scan_stale():
                    return reached
                if deadline is not None and time.monotonic() > deadline:
                    self._say('budget',
                              f'{tag}: vrijeme za ovu sobu je isteklo na '
                              f'{index}/{len(waypoints)} — idem na prolaz.')
                    self.stop()
                    return reached
                if self.move_to(x, y, hold_yaw, f'{tag} {index}/{len(waypoints)}'):
                    reached += 1
                # A lane end that cannot be reached is not a reason to stop: the
                # next lane is usually reachable and covers most of what it saw.
                if replans < self.p['max_replans']:
                    _region, _points, now = self.plan_route(offset)
                    if now > area * self.p['replan_growth']:
                        self._say('replanning',
                                  f'{tag}: soba je narasla {area:.1f} → {now:.1f} m² '
                                  f'dok sam vozio — planiram trake iznova.',
                                  area=round(now, 1))
                        grew = True
                        break
            self.stop()
            if not grew:
                return reached
            replans += 1
        return reached

    def closure(self, this_room_only=False):
        """Unknown cells still reachable - in this room, or in the building.

        Two different decisions read this number and they need different
        answers: "walk this room again?" must not be swayed by rooms nobody has
        entered, and "is the map finished?" must be.
        """
        robot = self.pose()
        if robot is None or self.map is None:
            return None
        info, grid = self._grids()
        return unknown_reachable(grid, info, seed_cell(info, robot[0], robot[1]),
                                 self.p['closure_clearance'],
                                 self.seals() if this_room_only else None)

    def prepare_passage(self, passage):
        """Everything that can be decided about an opening before driving at it.

        Without this the robot commits two Nav2 goals - up to 90 s each - to
        find out what one array slice could have told it, and a failure says
        only "navigation did not succeed". The checks are made against the map
        as it is *now*, not the map the passage was first detected on, because
        that map has been growing ever since.

        Returns a plan dict, or a string saying why there is none.
        """
        robot = self.pose()
        if robot is None:
            return 'nemam pozu robota'
        info, grid = self._grids()

        width = opening_width(grid, info, passage)
        if width is None:
            return 'središte prolaza više nije na karti'
        passage['measured'] = width
        if width < self.p['min_passage_width']:
            return (f'otvor je {width:.2f} m, a treba barem '
                    f'{self.p["min_passage_width"]:.2f} m')

        nx, ny = passage['normal']
        near_d, far_d = self.p['door_standoff'], self.p['door_exit']
        side_a = (passage['x'] + nx * near_d, passage['y'] + ny * near_d)
        side_b = (passage['x'] - nx * near_d, passage['y'] - ny * near_d)
        towards_a = math.hypot(side_a[0] - robot[0], side_a[1] - robot[1])
        towards_b = math.hypot(side_b[0] - robot[0], side_b[1] - robot[1])
        sign = 1.0 if towards_a <= towards_b else -1.0
        near = (passage['x'] + nx * near_d * sign, passage['y'] + ny * near_d * sign)
        far = (passage['x'] - nx * far_d * sign, passage['y'] - ny * far_d * sign)
        heading = math.atan2(-ny * sign, -nx * sign)

        # The near pose is in the room we are standing in, so the map can be
        # held to the full standard: the robot has to fit there.
        fits = fits_mask(grid, info.resolution, self.p['closure_clearance'])
        row, col = seed_cell(info, *near)
        if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1] and fits[row, col]):
            return 'poza za poravnanje pred prolazom nije prohodna'

        # The far pose is in a room nobody has seen, so unknown is allowed and
        # only occupied is not. Pull it in until it stops sitting in a wall
        # rather than declaring the passage unusable.
        while far_d > 0.30:
            row, col = seed_cell(info, *far)
            inside = 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]
            if inside and grid[row, col] < OCCUPIED_FROM:
                break
            far_d -= 0.10
            far = (passage['x'] - nx * far_d * sign, passage['y'] - ny * far_d * sign)
        else:
            return 'iza prolaza je zid na svakoj udaljenosti'

        # And the line between them, which is the part that actually threads the
        # opening. Occupied only: unknown is the whole point of going.
        steps = int(math.hypot(far[0] - near[0], far[1] - near[1]) / info.resolution)
        for step in range(steps + 1):
            t = step / max(1, steps)
            row, col = seed_cell(info, near[0] + t * (far[0] - near[0]),
                                 near[1] + t * (far[1] - near[1]))
            if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1] \
                    and grid[row, col] >= OCCUPIED_FROM:
                return 'os prolaza je zapriječena'
        return {'near': near, 'far': far, 'heading': heading, 'width': width}

    def through_passage(self, passage):
        """Square up in front of the opening, then go straight through.

        Both poses go to Nav2 on its stock tree, and that is the one place this
        node gives up control on purpose: 1.0 m of opening leaves this robot
        about 7 cm a side, an angle eats that in a couple of degrees, and P-45
        measured that only a footprint check holds it.
        """
        plan = self.prepare_passage(passage)
        if isinstance(plan, str):
            passage['failures'] += 1
            self._say('doorway-rejected',
                      f'Prolaz ({passage["x"]:.2f}, {passage["y"]:.2f}) preskočen: {plan}',
                      x=passage['x'], y=passage['y'])
            return False
        near, far, heading = plan['near'], plan['far'], plan['heading']

        self._say('doorway',
                  f'Prolaz ({passage["x"]:.2f}, {passage["y"]:.2f}), izmjereno '
                  f'{plan["width"]:.2f} m: poravnanje pa ravno kroz.',
                  x=passage['x'], y=passage['y'], width=round(plan['width'], 3))
        if not self.nav_goal(near[0], near[1], heading, 'ispred prolaza'):
            passage['failures'] += 1
            return False
        self.spin(self.p['settle_time'])
        if not self.nav_goal(far[0], far[1], heading, 'kroz prolaz'):
            passage['failures'] += 1
            return False
        passage['visited'] = True
        self.spin(self.p['settle_time'])
        return True

    # ----------------------------------------------------------------- loop
    def run(self):
        self._say('waiting', 'Čekam kartu i Nav2…')
        self.spin(self.p['start_delay'])
        while rclpy.ok() and (self.map is None or self.pose() is None):
            rclpy.spin_once(self, timeout_sec=0.2)

        rooms = 0
        while rclpy.ok():
            robot = self.pose()
            if robot is None:
                self.spin(0.5)
                continue
            if self._scan_stale():
                self._say('stalled', self.stalled_reason)
                return False
            rooms += 1
            hold_yaw = robot[2]
            deadline = time.monotonic() + self.p['room_budget']

            # 0. let SLAM finish drawing the room before planning a route on it
            known = self.wait_for_map()
            self._say('surveying', f'soba {rooms}: karta se smirila ({known} poznatih ćelija).')

            # 1. cover the room
            self.sweep(hold_yaw, 0.0, f'soba {rooms}', deadline)

            # 2. patch what stayed unknown IN THIS ROOM, by walking it again on
            #    lanes shifted half a spacing - the shadows move, so they get
            #    seen. Measured with the doorways sealed, or the unexplored
            #    rooms next door would order a second pass every single time.
            for extra in range(int(self.p['patch_passes'])):
                if time.monotonic() > deadline:
                    break
                here = self.closure(this_room_only=True)
                if here is None or here <= self.p['unknown_done']:
                    break
                self._say('patching',
                          f'soba {rooms}: još {here} nepoznatih ćelija u ovoj sobi — '
                          f'drugi prolaz s pomakom pola trake.', unknown_reachable=here)
                self.sweep(hold_yaw, self.p['lane_spacing'] / 2.0,
                           f'soba {rooms} dopuna {extra + 1}', deadline)

            here = self.closure(this_room_only=True)
            left = self.closure()
            self._say('swept',
                      f'soba {rooms} pokrivena; nepoznato u sobi: {here}, ukupno: {left}',
                      unknown_reachable=left, room_unknown=here,
                      passages=len(self.passages))

            # 3. step through an opening nobody has driven yet. A passage that
            #    fails sends us to the NEXT passage, never back to the lanes:
            #    this room is already covered, and sweeping it again is how the
            #    18 Sep run spent its second hundred seconds in the room it
            #    started in.
            moved = False
            while rclpy.ok() and not moved:
                robot = self.pose() or robot
                options = self.unvisited(robot)
                if not options:
                    break
                if self.through_passage(options[0]):
                    moved = True
                else:
                    self.get_logger().warn(
                        'prolaz nije prošao; biram sljedeći bez ponovnog pometanja')
            if moved:
                continue

            if left is not None and left > self.p['unknown_done']:
                self._say('incomplete',
                          f'Nema prohodnog neposjećenog prolaza, a {left} nepoznatih '
                          f'ćelija ostaje uz dohvat. Dovrši ručno pa „MAPIRANJE GOTOVO".',
                          unknown_reachable=left, passages=len(self.passages))
                return False
            self._say('done',
                      f'Sve sobe pokrivene i svi prolazi prođeni '
                      f'({len(self.passages)}). Karta je zatvorena.',
                      unknown_reachable=left, passages=len(self.passages))
            return True
        return False


def main():
    rclpy.init()
    node = RoomSweeper()
    try:
        node.run()
        node.stop()
        # Stay alive: the panel keeps reading the last status, and the map is
        # saved by the button.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
