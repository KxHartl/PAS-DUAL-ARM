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
from visualization_msgs.msg import Marker, MarkerArray


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def door_candidates(msg):
    """Find ~1 m breaks between long, collinear occupied wall segments."""
    info = msg.info
    grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    occupied = grid >= 65
    free = grid == 0
    resolution = info.resolution
    min_wall = max(8, round(0.8 / resolution))
    min_gap = max(2, round(0.7 / resolution))
    max_gap = round(1.3 / resolution)
    found = []
    for axis in (0, 1):
        lines = occupied if axis == 0 else occupied.T
        frees = free if axis == 0 else free.T
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
                        np.mean(frees[line_index, first_end:second_start]) < 0.6):
                    continue
                # Both sides of the opening must be observed free space.
                mid = (first_end + second_start) // 2
                side_a = max(0, line_index - round(0.3 / resolution))
                side_b = min(lines.shape[0] - 1, line_index + round(0.3 / resolution))
                if not (frees[side_a, mid] and frees[side_b, mid]):
                    continue
                ix, iy = (mid, line_index) if axis == 0 else (line_index, mid)
                x = info.origin.position.x + (ix + 0.5) * resolution
                y = info.origin.position.y + (iy + 0.5) * resolution
                found.append({'x': x, 'y': y, 'width': gap * resolution,
                              'wall_axis': 'x' if axis == 0 else 'y'})
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
    return [d for d in merged if d['observations'] >= 2]


class FeatureRegistry(Node):
    def __init__(self):
        super().__init__('feature_registry')
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._features = {'doors': [], 'tables': [], 'box_marker': None}
        self._last_cloud = 0.0
        self._last_map = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(String, '/semantic_features', qos)
        self._marker_pub = self.create_publisher(MarkerArray, '/door_markers', qos)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_subscription(PointCloud2, '/camera/points', self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/aruco_single/pose', self._on_marker, 10)

    def _publish(self):
        msg = String()
        msg.data = json.dumps({'frame_id': 'map', **self._features}, allow_nan=False)
        self._pub.publish(msg)

    def _publish_door_markers(self, doors):
        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 0
        for d in doors:
            x, y, w, axis = d['x'], d['y'], d['width'], d['wall_axis']
            # Door threshold line
            m_thresh = Marker()
            m_thresh.header.frame_id = 'map'
            m_thresh.header.stamp = now
            m_thresh.ns = 'doors'
            m_thresh.id = mid; mid += 1
            m_thresh.type = Marker.CUBE
            m_thresh.action = Marker.ADD
            m_thresh.pose.position.x = float(x)
            m_thresh.pose.position.y = float(y)
            m_thresh.pose.position.z = 0.05
            m_thresh.scale.x = float(w) if axis == 'x' else 0.15
            m_thresh.scale.y = 0.15 if axis == 'x' else float(w)
            m_thresh.scale.z = 0.10
            m_thresh.color.r = 0.1; m_thresh.color.g = 0.9; m_thresh.color.b = 0.2; m_thresh.color.a = 0.8
            array.markers.append(m_thresh)

            # Staging pose in HOME and Transit pose in ROOM (1.20 m from threshold)
            if axis == 'x': # wall along X (e.g. BLUE door at y = -3.0)
                stage = (float(x), float(y + 1.2), -math.pi / 2, 'Staging (HOME)')
                transit = (float(x), float(y - 1.2), -math.pi / 2, 'Transit (ROOM)')
            else: # wall along Y (e.g. RED door at x = 3.0)
                stage = (float(x - 1.2), float(y), 0.0, 'Staging (HOME)')
                transit = (float(x + 1.2), float(y), 0.0, 'Transit (ROOM)')

            for px, py, pyaw, label in (stage, transit):
                m_pt = Marker()
                m_pt.header.frame_id = 'map'
                m_pt.header.stamp = now
                m_pt.ns = 'doors'
                m_pt.id = mid; mid += 1
                m_pt.type = Marker.ARROW
                m_pt.action = Marker.ADD
                m_pt.pose.position.x = px
                m_pt.pose.position.y = py
                m_pt.pose.position.z = 0.1
                m_pt.pose.orientation.z = math.sin(pyaw * 0.5)
                m_pt.pose.orientation.w = math.cos(pyaw * 0.5)
                m_pt.scale.x = 0.6; m_pt.scale.y = 0.12; m_pt.scale.z = 0.12
                m_pt.color.r = 0.2; m_pt.color.g = 0.8; m_pt.color.b = 1.0; m_pt.color.a = 0.9
                array.markers.append(m_pt)

                m_txt = Marker()
                m_txt.header.frame_id = 'map'
                m_txt.header.stamp = now
                m_txt.ns = 'doors'
                m_txt.id = mid; mid += 1
                m_txt.type = Marker.TEXT_VIEW_FACING
                m_txt.action = Marker.ADD
                m_txt.pose.position.x = px
                m_txt.pose.position.y = py
                m_txt.pose.position.z = 0.4
                m_txt.scale.z = 0.25
                m_txt.color.r = 1.0; m_txt.color.g = 1.0; m_txt.color.b = 1.0; m_txt.color.a = 1.0
                m_txt.text = f"{label} ({px:.1f}, {py:.1f})"
                array.markers.append(m_txt)

        self._marker_pub.publish(array)

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
        self._publish()
        self._publish_door_markers(self._features['doors'])

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
        # Table tops are at 0.10 m; the floor is near zero.  This is an
        # observation band, not a claim that every cluster is a table.
        world = world[(world[:, 2] > 0.075) & (world[:, 2] < 0.125)]
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
        tables = []
        for label in range(1, count + 1):
            cells = np.argwhere(labels == label)
            if len(cells) < 20:
                continue
            extent = (cells.max(axis=0) - cells.min(axis=0) + 1) * cell
            if not (0.25 <= extent[0] <= 0.9 and 0.25 <= extent[1] <= 0.9):
                continue
            xy = minimum + cells.mean(axis=0) * cell
            tables.append({'x': float(xy[0]), 'y': float(xy[1]),
                           'size_x': float(extent[0]), 'size_y': float(extent[1]),
                           'top_z': 0.10, 'source': 'rgbd_horizontal_cluster'})
        if tables:
            self._features['tables'] = tables
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
