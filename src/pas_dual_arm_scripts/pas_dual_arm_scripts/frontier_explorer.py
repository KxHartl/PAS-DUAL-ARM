#!/usr/bin/env python3
"""Explore an unknown world on its own, and treat doorways as doorways.

Frontier exploration is the standard answer to "map this place without being
driven": find the boundary between mapped-free and unknown, go stand on it, and
repeat until there is no boundary left (Yamauchi, 1997). What makes the plain
version fail *here* is the geometry of this world, in two specific ways.

**A doorway frontier is small, so it never wins.** Scoring a frontier by how much
unknown space it touches is what makes exploration efficient, but the sliver of
frontier inside a 1.0 m opening is tiny next to the wall of unknown behind a
half-mapped room. The next room therefore never gets visited. So doorways are
detected explicitly - reusing the same ``door_candidates`` that the navigation
zones are built from - and a frontier seen through one is scored as the room
behind it, not as the sliver.

**A doorway cannot be entered at an angle.** The robot is 0.85 m wide and the
opening is 1.0 m. A planner that treats the doorway as ordinary free space will
approach it diagonally and wedge the robot on the frame. So a goal on the far
side of a door is not sent as one goal: the robot is first squared up in front
of the opening, then driven straight through, and only then does exploration
continue in the new room. That is the same perpendicular-transit rule the mission
navigator uses; here it is applied to goals nobody chose by hand.

**A frontier goal has no heading worth holding.** The lidar sees 360 degrees, so
the orientation the robot ends a leg in carries no information - but Nav2 was
being asked for it to 1.4 degrees, which on a base that slides when it turns in
place (mu2 = 0, P-10) meant a pirouette per leg, each one feeding yaw error into
the scan matcher. Frontier goals therefore run on the mapping profile
(``behavior_trees/explore_to_pose.xml``: heading free, no Spin recovery, reverse
only as a recovery), while the doorway poses keep Nav2's stock tree, because
squaring up in front of an opening is the one turn in place that earns its cost.

Progress is published on ``/exploration/status`` as JSON so the panel can show it.
Exploration ends by itself when no reachable frontier is left; the map is still
saved by the user pressing the button, which also lets them stop early or drive
somewhere by hand first.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from pas_dual_arm_scripts.feature_registry import door_candidates

FREE, UNKNOWN = 0, -1
OCCUPIED_FROM = 65


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


# ---------------------------------------------------------------- pure array
# These are module-level and take plain arrays so the same code can be checked
# offline against a saved map (scripts/check_frontier.py). Keeping the reasoning
# in one place is not tidiness: a copy in the harness that drifts from the node
# is a harness that passes while the robot stalls.

def frontier_mask(grid):
    """Free cells that touch unknown space."""
    free, unknown = grid == FREE, grid == UNKNOWN
    touching = np.zeros_like(unknown)
    touching[1:, :] |= unknown[:-1, :]
    touching[:-1, :] |= unknown[1:, :]
    touching[:, 1:] |= unknown[:, :-1]
    touching[:, :-1] |= unknown[:, 1:]
    return free & touching


def gateway_from_cluster(grid, cells, resolution,
                         min_width=0.60, max_width=1.45, max_depth=0.45,
                         flank=0.35):
    """Is this frontier cluster an opening in a wall, rather than open space?

    A doorway into an unexplored room cannot be found by looking for free space
    on both sides of a gap - that is what `door_candidates` does, and by
    definition the far side has not been seen yet. What it looks like *from this
    side* is unmistakable though: a short, thin run of frontier, the width of a
    door, with wall on both ends.

    Returns the opening's centre and the direction through it, or None.
    """
    rows, cols = cells[:, 0], cells[:, 1]
    height = (rows.max() - rows.min() + 1) * resolution
    width = (cols.max() - cols.min() + 1) * resolution
    span, depth = max(height, width), min(height, width)
    if depth > max_depth or not (min_width <= span <= max_width):
        return None

    # The long axis runs along the wall; travel is across it.
    along_rows = height >= width
    flank_cells = max(1, int(round(flank / resolution)))
    occupied = grid >= OCCUPIED_FROM
    if along_rows:
        col = int(round(cols.mean()))
        lo, hi = int(rows.min()), int(rows.max())
        before = occupied[max(0, lo - flank_cells):lo, col]
        after = occupied[hi + 1:hi + 1 + flank_cells, col]
        normal = [1.0, 0.0]
    else:
        row = int(round(rows.mean()))
        lo, hi = int(cols.min()), int(cols.max())
        before = occupied[row, max(0, lo - flank_cells):lo]
        after = occupied[row, hi + 1:hi + 1 + flank_cells]
        normal = [0.0, 1.0]
    if not (before.any() and after.any()):
        return None
    return {'row': float(rows.mean()), 'col': float(cols.mean()), 'normal': normal,
            'width': span}


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        # --- what counts as a frontier worth driving to -------------------
        self.declare_parameter('min_frontier_cells', 12)
        self.declare_parameter('min_door_frontier_cells', 3)
        # Robot half-diagonal plus a little; a goal must have this much room.
        self.declare_parameter('goal_clearance', 0.55)
        # Scoring: information gain against how far it is.
        self.declare_parameter('gain_weight', 1.0)
        self.declare_parameter('cost_weight', 1.6)
        self.declare_parameter('door_bonus', 25.0)
        # --- doorway transit ----------------------------------------------
        self.declare_parameter('door_standoff', 1.10)
        self.declare_parameter('door_match_radius', 1.20)
        # --- pacing ---------------------------------------------------------
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('settle_time', 1.5)
        self.declare_parameter('blacklist_radius', 0.6)
        self.declare_parameter('max_failures', 3)
        self.declare_parameter('start_delay', 8.0)
        # How stale map -> odom may get before the run is called off. SLAM
        # republishes it on every scan, so seconds of staleness means the scans
        # have stopped, not that the robot is somewhere unexpected (P-47).
        self.declare_parameter('tf_stale_timeout', 5.0)
        # --- which behaviour tree drives which goal -------------------------
        # Frontier goals run on the mapping profile: heading free, no Spin
        # recovery, no reverse as ordinary travel. Doorway poses do not - lining
        # up in front of an opening is a turn in place, and it is the one turn
        # worth its cost - so they stay on Nav2's stock tree with the tight
        # general_goal_checker. An empty string means "the stock tree".
        self.declare_parameter('explore_bt', self._default_explore_bt())

        self.p = {name: self.get_parameter(name).value for name in (
            'min_frontier_cells', 'min_door_frontier_cells', 'goal_clearance',
            'gain_weight', 'cost_weight', 'door_bonus', 'door_standoff',
            'door_match_radius', 'goal_timeout', 'settle_time',
            'blacklist_radius', 'max_failures', 'start_delay',
            'tf_stale_timeout')}

        self.explore_bt = self.get_parameter('explore_bt').value
        if self.explore_bt and not os.path.exists(self.explore_bt):
            # Falling back silently would leave the robot pirouetting again with
            # nothing in the log to say why.
            self.get_logger().warn(
                f'mapping behaviour tree not found at {self.explore_bt}; '
                'falling back to the stock tree, so frontier goals will hold a '
                'heading and recoveries will spin in place')
            self.explore_bt = ''

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.status = self.create_publisher(String, '/exploration/status', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.blacklist = []
        self.visited_doors = []
        self.failures = 0
        # Set when the run is stopped by something that is not the frontier's
        # fault, so the frontier does not get blamed and blacklisted for it.
        self.stalled_reason = None

    @staticmethod
    def _default_explore_bt():
        try:
            return os.path.join(get_package_share_directory('pas_dual_arm_bringup'),
                                'behavior_trees', 'explore_to_pose.xml')
        except Exception:
            return ''

    # --------------------------------------------------------------- inputs
    def _on_map(self, msg):
        self.map = msg

    def _say(self, state, detail, **extra):
        payload = {'state': state, 'detail': detail}
        payload.update(extra)
        self.status.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'EXPLORE {state}: {detail}')

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

    def map_odom_age(self):
        """Seconds since `map -> odom` was last refreshed, or None if absent.

        slam_toolbox stamps this transform with the last scan it processed, so
        its age is a direct measure of whether scans are still arriving. When
        they stop, `controller_server` starts rejecting the transform as too
        old and simply issues no velocity - with no error anywhere that names
        the real cause, while the explorer happily resends the same goal
        forever (P-47). Measuring it here turns that silence into one line.
        """
        try:
            tf = self._tf.lookup_transform('map', 'odom', rclpy.time.Time())
        except Exception:
            return None
        stamp = rclpy.time.Time.from_msg(tf.header.stamp)
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _tf_stale(self):
        """True (and records why) when map -> odom has stopped being updated."""
        age = self.map_odom_age()
        if age is None or age <= self.p['tf_stale_timeout']:
            return False
        self.stalled_reason = (
            f'map → odom nije osvježen {age:.1f} s — skenovi su stali. '
            f'Provjeri /scan i /scan_filtered (scripts/scan_watch.py), P-47.')
        return True

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------------------------------------------------ frontiers
    def _grids(self):
        info = self.map.info
        grid = np.asarray(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        return info, grid

    def _to_world(self, info, row, col):
        return (info.origin.position.x + (col + 0.5) * info.resolution,
                info.origin.position.y + (row + 0.5) * info.resolution)

    def _has_room(self, info, grid, row, col):
        """Is there space for the robot centred on this cell?"""
        radius = max(1, int(round(self.p['goal_clearance'] / info.resolution)))
        r0, r1 = max(0, row - radius), min(grid.shape[0], row + radius + 1)
        c0, c1 = max(0, col - radius), min(grid.shape[1], col + radius + 1)
        return not (grid[r0:r1, c0:c1] >= OCCUPIED_FROM).any()

    def frontiers(self):
        """Clusters of free cells that touch unknown space, as world points.

        A cluster that is an opening in a wall is kept even when it is tiny and
        even when the robot would not fit standing on it, because it is not
        driven to directly: the transit poses stand off from it on either side.
        Applying the open-space rules to a doorway is what makes plain frontier
        exploration stop at the first closed door.
        """
        info, grid = self._grids()
        frontier = frontier_mask(grid)
        if not frontier.any():
            return []

        labels, count = ndimage.label(frontier, structure=np.ones((3, 3)))
        doors = door_candidates(self.map) if self.map is not None else []
        out = []
        for label in range(1, count + 1):
            cells = np.argwhere(labels == label)
            size = len(cells)
            gateway = gateway_from_cluster(grid, cells, info.resolution)

            if gateway is not None:
                if size < self.p['min_door_frontier_cells']:
                    continue
                x, y = self._to_world(info, gateway['row'], gateway['col'])
                if self._blacklisted(x, y) or self._door_done(x, y):
                    continue
                door = {'x': x, 'y': y, 'normal': gateway['normal'],
                        'width': gateway['width']}
                # If both sides happen to be mapped already, the established
                # detector has the better centre; prefer it.
                known = self._nearest_door(doors, x, y)
                if known is not None:
                    door = dict(known, normal=known.get('normal', gateway['normal']))
                out.append({'x': door['x'], 'y': door['y'], 'size': size, 'door': door})
                continue

            if size < self.p['min_frontier_cells']:
                continue
            # The centroid of a curved frontier can land in a wall, so drive to
            # the cluster cell nearest to it that the robot actually fits in.
            centre = cells.mean(axis=0)
            order = np.argsort(np.linalg.norm(cells - centre, axis=1))
            pick = next((cells[i] for i in order
                         if self._has_room(info, grid, int(cells[i][0]), int(cells[i][1]))),
                        None)
            if pick is None:
                continue
            x, y = self._to_world(info, int(pick[0]), int(pick[1]))
            if self._blacklisted(x, y):
                continue
            out.append({'x': x, 'y': y, 'size': size, 'door': None})
        return out

    def _door_done(self, x, y):
        """Already driven through: otherwise the same opening is chosen forever."""
        return any(math.hypot(x - dx, y - dy) < self.p['door_match_radius']
                   for dx, dy in self.visited_doors)

    def _nearest_door(self, doors, x, y):
        best, best_d = None, self.p['door_match_radius']
        for door in doors:
            d = math.hypot(door['x'] - x, door['y'] - y)
            if d < best_d:
                best, best_d = door, d
        return best

    def _blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.p['blacklist_radius']
                   for bx, by in self.blacklist)

    def choose(self, candidates, robot):
        """Highest information gain per unit of travel, doors counted as rooms."""
        rx, ry, _ = robot
        best, best_score = None, -math.inf
        for candidate in candidates:
            distance = math.hypot(candidate['x'] - rx, candidate['y'] - ry)
            score = (self.p['gain_weight'] * math.sqrt(candidate['size'])
                     - self.p['cost_weight'] * distance)
            if candidate['door'] is not None:
                score += self.p['door_bonus']
            if score > best_score:
                best, best_score = candidate, score
        return best

    # ----------------------------------------------------------- navigation
    def _goal(self, x, y, yaw, tree=''):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = yaw_to_quaternion(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = tree
        return goal

    def drive_to(self, x, y, yaw, label, tree=''):
        if not self.nav.wait_for_server(timeout_sec=20.0):
            self._say('failed', 'Nav2 navigate_to_pose nije dostupan.')
            return False
        self._say('driving', f'{label} → ({x:.2f}, {y:.2f})', x=float(x), y=float(y))
        send = self.nav.send_goal_async(self._goal(x, y, yaw, tree))
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'{label}: goal rejected')
            return False
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.p['goal_timeout']
        while rclpy.ok() and not result_future.done():
            if self._tf_stale():
                self.get_logger().error(f'{label}: {self.stalled_reason}')
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel)
                return False
            if time.monotonic() > deadline:
                self.get_logger().warn(f'{label}: timed out, cancelling')
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel)
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        ok = result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        if not ok:
            self.get_logger().warn(f'{label}: navigation did not succeed')
        return ok

    def through_door(self, door, target):
        """Square up, go straight through, then carry on in the new room.

        Sending the far-side goal directly is what wedges the robot on the frame:
        the planner has no reason to arrive perpendicular, and 7 cm per side does
        not forgive an angle.
        """
        robot = self.pose()
        if robot is None:
            return False
        nx, ny = door['normal']
        standoff = self.p['door_standoff']
        # Two poses on the door axis, one either side; near one is whichever is
        # closer to the robot right now.
        side_a = (door['x'] + nx * standoff, door['y'] + ny * standoff)
        side_b = (door['x'] - nx * standoff, door['y'] - ny * standoff)
        d_a = math.hypot(side_a[0] - robot[0], side_a[1] - robot[1])
        d_b = math.hypot(side_b[0] - robot[0], side_b[1] - robot[1])
        near, far = (side_a, side_b) if d_a <= d_b else (side_b, side_a)
        heading = math.atan2(far[1] - near[1], far[0] - near[0])

        self._say('doorway',
                  f'Prolaz na ({door["x"]:.2f}, {door["y"]:.2f}): '
                  f'poravnanje pa ravno kroz.')
        if not self.drive_to(near[0], near[1], heading, 'ispred vrata'):
            return False
        self.spin(self.p['settle_time'])
        if not self.drive_to(far[0], far[1], heading, 'kroz vrata'):
            return False
        self.visited_doors.append((door['x'], door['y']))
        self.spin(self.p['settle_time'])
        # Let the new room open up before choosing the next frontier; going
        # straight for the old target would aim at a point the map now knows.
        return True

    # ----------------------------------------------------------------- loop
    def explore(self):
        self._say('waiting', 'Čekam kartu i Nav2…')
        self.spin(self.p['start_delay'])
        while rclpy.ok() and self.map is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            robot = self.pose()
            if robot is None:
                self.spin(0.5)
                continue
            if self._tf_stale():
                self._say('stalled', self.stalled_reason)
                return False
            candidates = self.frontiers()
            if not candidates:
                self._say('done', 'Nema više dostupnih granica — prostor je istražen.')
                return True
            target = self.choose(candidates, robot)
            self._say('planning',
                      f'{len(candidates)} granica; biram '
                      f'({target["x"]:.2f}, {target["y"]:.2f}), {target["size"]} ćelija'
                      + (' [kroz vrata]' if target['door'] else ''),
                      remaining=len(candidates))

            if target['door'] is not None:
                ok = self.through_door(target['door'], target)
            else:
                # The goal keeps the heading the robot already has. Any other
                # value is one the robot would have to turn in place to reach,
                # for a sensor that sees in every direction anyway; the mapping
                # goal checker ignores it, and this makes that explicit.
                ok = self.drive_to(target['x'], target['y'], robot[2], 'granica',
                                   tree=self.explore_bt)

            if ok:
                self.failures = 0
                self.spin(self.p['settle_time'])
            elif self.stalled_reason is not None:
                # Not this frontier's fault: the pose feedback stopped, so every
                # other frontier would fail the same way. Blacklisting them one
                # by one would spend three goals to reach the wrong conclusion.
                self._say('stalled', self.stalled_reason)
                return False
            else:
                # One unreachable frontier must not stop the run; remember it and
                # try the next one. Enough of them in a row does mean stop.
                self.blacklist.append((target['x'], target['y']))
                self.failures += 1
                if self.failures >= self.p['max_failures']:
                    self._say('stalled',
                              f'{self.failures} neuspjeha zaredom — zaustavljam istraživanje. '
                              f'Dovršite ručno i pritisnite „MAPIRANJE GOTOVO“.')
                    return False
        return False


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        node.explore()
        # Stay alive: the map is saved by the user, and the panel keeps reading
        # the last status.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
