"""Register observed doors, low table tops and the box marker in map frame.

Only measurements are published.  The low tables are invisible to the 0.21 m
planar lidar, so they come from the RGB-D cloud; the ArUco point is the visible
front-face marker, not an invented cube centre.  Output is JSON on
/semantic_features, with per-feature sources and timestamps.
"""

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def door_candidates(msg, both_sides=True):
    """Find ~1 m breaks between long, collinear occupied wall segments.

    `both_sides` is what makes this usable *while* mapping. With it True (the
    default, and what the mission uses) an opening only counts once free space
    has been observed on either side of it - which is right for a finished map
    and useless on a growing one, because the far side of a door the robot has
    not driven through yet is unknown by definition. That is why nav_zones
    logged "no doorways detected" for a whole exploration run.

    With it False the far side may be unknown: the gap has to be a door-width
    break in a wall with free space on *one* side, which is exactly what a
    doorway looks like from the room you are standing in. The near-side test is
    kept, so a break in a wall with nothing observed on either side - the far
    edge of the map - is still not a door.
    """
    info = msg.info
    grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    occupied = grid >= 65
    free = grid == 0
    # What may sit inside the opening. Seen-through doorways are free; a doorway
    # seen from one side only is unknown in the middle, and unknown is not a
    # wall, so it may not be excluded when we are still mapping.
    passable = free if both_sides else (grid < 65)
    resolution = info.resolution
    min_wall = max(8, round(0.8 / resolution))
    min_gap = max(2, round(0.7 / resolution))
    max_gap = round(1.3 / resolution)
    found = []
    for axis in (0, 1):
        lines = occupied if axis == 0 else occupied.T
        frees = free if axis == 0 else free.T
        passables = passable if axis == 0 else passable.T
        for line_index, line in enumerate(lines):
            changes = np.diff(np.pad(line.astype(np.int8), 1))
            starts = np.flatnonzero(changes == 1)
            ends = np.flatnonzero(changes == -1)
            for first_end, second_start, first_start, second_end in zip(
                    ends[:-1], starts[1:], starts[:-1], ends[1:]):
                gap = second_start - first_end
                if (first_end - first_start < min_wall or
                        second_end - second_start < min_wall or
                        not min_gap <= gap <= max_gap or
                        np.mean(passables[line_index, first_end:second_start]) < 0.6):
                    continue
                # Observed free space beside the opening: on both sides for a
                # finished map, on at least one while still mapping.
                mid = (first_end + second_start) // 2
                side_a = max(0, line_index - round(0.3 / resolution))
                side_b = min(lines.shape[0] - 1, line_index + round(0.3 / resolution))
                near, far = frees[side_a, mid], frees[side_b, mid]
                if not (near and far) if both_sides else not (near or far):
                    continue
                ix, iy = (mid, line_index) if axis == 0 else (line_index, mid)
                x = info.origin.position.x + (ix + 0.5) * resolution
                y = info.origin.position.y + (iy + 0.5) * resolution
                found.append({'x': x, 'y': y, 'width': gap * resolution,
                              'wall_axis': 'x' if axis == 0 else 'y'})
    # `wall_axis` is the axis the WALL runs along, so the direction a robot has
    # to travel to get through is the other one. Carrying it as an explicit
    # vector keeps every consumer from re-deriving (and mis-deriving) it.
    # Wall thickness yields several adjacent candidates. Merge within 0.25 m.
    merged = []
    for candidate in sorted(found, key=lambda d: (d['wall_axis'], d['x'], d['y'])):
        match = next((d for d in merged if d['wall_axis'] == candidate['wall_axis']
                      and math.hypot(d['x'] - candidate['x'], d['y'] - candidate['y']) < 0.25), None)
        if match is None:
            candidate['observations'] = 1
            merged.append(candidate)
        else:
            n = match['observations']
            for key in ('x', 'y', 'width'):
                match[key] = (match[key] * n + candidate[key]) / (n + 1)
            match['observations'] += 1
    doors = [d for d in merged if d['observations'] >= (2 if both_sides else 1)]
    for door in doors:
        door['normal'] = [0.0, 1.0] if door['wall_axis'] == 'x' else [1.0, 0.0]
    return doors


def table_candidates(msg, max_post=0.25, max_span=1.6, min_posts=3):
    """Find tables in the map as clusters of free-standing posts (the legs).

    The 0.21 m scan plane cuts the legs, not the 0.75 m top, so a table appears
    as three or four small blobs that touch nothing. That is a far more
    dependable signal than the RGB-D top: it is in the map from the moment the
    room is mapped, needs no camera pointing and no robot pose near the table.
    Walls are excluded by size - they are one huge connected component.
    """
    info = msg.info
    grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    resolution = info.resolution
    labels, count = ndimage.label(grid >= 65, structure=np.ones((3, 3)))
    posts = []
    for label in range(1, count + 1):
        cells = np.argwhere(labels == label)
        extent = (cells.max(axis=0) - cells.min(axis=0) + 1) * resolution
        if max(extent) > max_post:
            continue          # a wall, or anything else too big to be a leg
        iy, ix = cells.mean(axis=0)
        posts.append((info.origin.position.x + (ix + 0.5) * resolution,
                      info.origin.position.y + (iy + 0.5) * resolution))

    # Single-link clustering: legs of one table are within a table diagonal of
    # each other, and tables are metres apart.
    unassigned = list(posts)
    tables = []
    while unassigned:
        group = [unassigned.pop()]
        grew = True
        while grew:
            grew = False
            for post in list(unassigned):
                if any(math.hypot(post[0] - g[0], post[1] - g[1]) <= max_span
                       for g in group):
                    group.append(post)
                    unassigned.remove(post)
                    grew = True
        if len(group) < min_posts:
            continue
        xs = [p[0] for p in group]
        ys = [p[1] for p in group]
        tables.append({
            'x': (min(xs) + max(xs)) / 2.0, 'y': (min(ys) + max(ys)) / 2.0,
            'size_x': max(xs) - min(xs), 'size_y': max(ys) - min(ys),
            'posts': len(group), 'source': 'occupancy_grid_legs',
        })
    return tables


class FeatureRegistry(Node):
    def __init__(self):
        super().__init__('feature_registry')
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        # `tables` are the map-derived leg clusters used to build navigation
        # zones; `table_tops` are RGB-D sightings of the 0.75 m surface, kept
        # separate so a camera glimpse can never move a planning zone.
        self._features = {'doors': [], 'tables': [], 'table_tops': [],
                          'box_marker': None}
        self._last_cloud = 0.0
        self._last_map = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(String, '/semantic_features', qos)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_subscription(PointCloud2, '/camera/points', self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/aruco_single/pose', self._on_marker, 10)

    def _publish(self):
        msg = String()
        msg.data = json.dumps({'frame_id': 'map', **self._features}, allow_nan=False)
        self._pub.publish(msg)

    def _transform(self, frame):
        try:
            tf = self._tf.lookup_transform('map', frame, rclpy.time.Time()).transform
            return rotation_matrix(tf.rotation), np.array([
                tf.translation.x, tf.translation.y, tf.translation.z])
        except Exception:
            return None

    def _on_map(self, msg):
        if time.monotonic() - self._last_map < 3.0:
            return
        self._last_map = time.monotonic()
        self._features['doors'] = [dict(d, source='occupancy_grid')
                                   for d in door_candidates(msg)]
        self._features['tables'] = table_candidates(msg)
        self._publish()

    def _on_marker(self, msg):
        transform = self._transform(msg.header.frame_id)
        if transform is None:
            return
        rot, origin = transform
        p = msg.pose.position
        xyz = rot @ np.array([p.x, p.y, p.z]) + origin
        self._features['box_marker'] = {
            'x': float(xyz[0]), 'y': float(xyz[1]), 'z': float(xyz[2]),
            'source': 'aruco_id_0', 'stamp': msg.header.stamp.sec +
            msg.header.stamp.nanosec * 1e-9}
        self._publish()

    def _on_cloud(self, msg):
        if time.monotonic() - self._last_cloud < 2.0:
            return
        self._last_cloud = time.monotonic()
        transform = self._transform(msg.header.frame_id)
        if transform is None:
            return
        rot, origin = transform
        try:
            points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                                    skip_nans=True)[::8]
        except (ValueError, KeyError):
            return
        if len(points) == 0:
            return
        points = points[np.all(np.isfinite(points), axis=1)]
        if len(points) == 0:
            return
        world = points @ rot.T + origin
        # Both tables in the world have their top surface at 0.75 m (the 0.10 m
        # band this used to carry belonged to the old low slab and matched
        # nothing).  This is an observation band, not a claim that every
        # cluster is a table.
        world = world[(world[:, 2] > 0.70) & (world[:, 2] < 0.80)]
        if len(world) < 50:
            return
        cell = 0.05
        minimum = world[:, :2].min(axis=0)
        ij = np.floor((world[:, :2] - minimum) / cell).astype(int)
        shape = ij.max(axis=0) + 1
        if np.prod(shape) > 2_000_000:
            return
        image = np.zeros(tuple(shape), dtype=np.int16)
        np.add.at(image, (ij[:, 0], ij[:, 1]), 1)
        labels, count = ndimage.label(image > 0)
        tops = []
        for label in range(1, count + 1):
            cells = np.argwhere(labels == label)
            if len(cells) < 20:
                continue
            extent = (cells.max(axis=0) - cells.min(axis=0) + 1) * cell
            if not (0.25 <= extent[0] <= 1.2 and 0.25 <= extent[1] <= 1.2):
                continue
            xy = minimum + cells.mean(axis=0) * cell
            tops.append({'x': float(xy[0]), 'y': float(xy[1]),
                         'size_x': float(extent[0]), 'size_y': float(extent[1]),
                         'top_z': 0.75, 'source': 'rgbd_horizontal_cluster'})
        if tops:
            self._features['table_tops'] = tops
            self._publish()


def main():
    rclpy.init()
    node = FeatureRegistry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
