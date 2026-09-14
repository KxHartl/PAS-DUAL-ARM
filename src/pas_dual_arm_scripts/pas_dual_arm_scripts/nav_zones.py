"""Turn detected doorways and tables into navigation zones and a room graph.

Why this exists
---------------
Nav2 was steering the robot into a 1.0 m doorway at an angle and along the side
of a table.  That is not a planner defect: a grid planner has no reason to
prefer a square approach, and the swept width of this robot grows fast with
heading error - 1.04 m long by 0.854 m wide means a 14 degree error needs
1.09 m of doorway (P-12, P-35).  The fix is to take the choice away from the
planner near the features where orientation matters, by reshaping the space it
plans in.

Around every detected doorway this publishes two "guide walls" that leave a
lane exactly as wide as the measured opening, a short way into each room.  A
path can then only enter along the door normal, and the controller can only
track it square.  Around every detected table it publishes a halo, so routes
keep their distance instead of grazing a corner.  Straight legs and square
corners are the point; the shortest path is not.

Nothing here is hand-surveyed.  Doors, tables and rooms all come out of the
SLAM map through `feature_registry`, so the zones move if the world does.  The
one exception is the *names* of the rooms - an occupancy grid has no colour, so
`room_labels` maps a name onto whichever detected room contains a given point.

Outputs
-------
/keepout_filter_mask  nav_msgs/OccupancyGrid, latched - consumed by Nav2's
                      KeepoutFilter on both costmaps.  Replaces the static
                      `filter_mask_server` PGM.
/nav_graph            std_msgs/String (JSON), latched - rooms, doors with their
                      portal poses, tables with their approach poses.  This is
                      what `room_navigator` drives from.
/nav_zones_markers    visualization_msgs/MarkerArray, latched - the same thing
                      drawn for RViz, so a human can see the zones before
                      trusting them.
"""

import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from pas_dual_arm_scripts.feature_registry import door_candidates, table_candidates

KEEPOUT = 100
FREE = 0


class Zones:
    """Geometry of one map: rooms, doors, tables, keepout rectangles, poses.

    Kept free of ROS types so it can be built and checked offline against a
    saved map (`scripts/check_zones.py`) without a simulator.
    """

    def __init__(self, grid, info, params):
        self.info = info
        self.params = params
        self.doors = door_candidates(_Msg(grid, info))
        self.tables = table_candidates(_Msg(grid, info))
        self.rects = []
        self.rooms = []
        self.warnings = []
        self._segment_rooms(grid)
        self._attach_doors_to_rooms()
        self._attach_tables_to_rooms()
        self._build_poses()
        # Painted in this order: a table halo, then the gate cut through it, and
        # the door guide walls last so a table can never open a doorway lane.
        self._build_table_zones()
        self._build_door_zones()

    # ---------------------------------------------------------------- zones
    def _build_door_zones(self):
        """Two guide walls per door, leaving a lane the width of the opening."""
        length = self.params['chute_length']
        thickness = self.params['chute_thickness']
        for index, door in enumerate(self.doors):
            cx, cy = door['x'], door['y']
            half = door['width'] / 2.0 - self.params['lane_margin']
            if half <= 0.0:
                self.warnings.append(
                    f'door {index} at ({cx:.2f}, {cy:.2f}) is {door["width"]:.2f} m '
                    f'wide; lane_margin {self.params["lane_margin"]:.2f} closes it')
                continue
            for side in (-1.0, 1.0):
                near, far = half, half + thickness
                if door['wall_axis'] == 'x':      # wall along X, travel along Y
                    self.rects.append(('door', index,
                                       cx + side * near, cx + side * far,
                                       cy - length, cy + length, KEEPOUT))
                else:                             # wall along Y, travel along X
                    self.rects.append(('door', index,
                                       cx - length, cx + length,
                                       cy + side * near, cy + side * far, KEEPOUT))

    def _build_table_zones(self):
        """A halo around each table, open on the side it is approached from.

        A closed halo would also wall off the approach, and in a 6 m room it
        crowds the nearby doorway portal.  Leaving the approach face open is
        what the halo is for anyway: block the three sides and the corners the
        robot has no business driving past, keep the head-on lane.
        """
        pad = self.params['table_clearance']
        gate = self.params['table_gate_margin']
        for index, table in enumerate(self.tables):
            half_x, half_y = table['size_x'] / 2.0, table['size_y'] / 2.0
            self.rects.append(('table', index,
                               table['x'] - half_x - pad, table['x'] + half_x + pad,
                               table['y'] - half_y - pad, table['y'] + half_y + pad,
                               KEEPOUT))
            fx, fy = table['face']
            if fx:   # approached along X, so the gate spans Y
                near = table['x'] + fx * half_x
                self.rects.append(('table_gate', index,
                                   near, near + fx * pad,
                                   table['y'] - half_y - gate,
                                   table['y'] + half_y + gate, FREE))
            else:    # approached along Y, gate spans X
                near = table['y'] + fy * half_y
                self.rects.append(('table_gate', index,
                                   table['x'] - half_x - gate,
                                   table['x'] + half_x + gate,
                                   near, near + fy * pad, FREE))

    # ---------------------------------------------------------------- rooms
    def _segment_rooms(self, grid):
        """Rooms are what free space falls apart into once the doors are shut.

        Seal each detected opening, label the connected free space, and drop
        anything touching the map border - that is the outside of the building,
        which reads as free because the map's free_thresh puts unknown grey on
        the free side.
        """
        info = self.info
        free = (grid == FREE)
        sealed = free.copy()
        for door in self.doors:
            half = door['width'] / 2.0 + 0.10
            depth = self.params['seal_depth']
            if door['wall_axis'] == 'x':
                box = (door['x'] - half, door['x'] + half,
                       door['y'] - depth, door['y'] + depth)
            else:
                box = (door['x'] - depth, door['x'] + depth,
                       door['y'] - half, door['y'] + half)
            r0, r1, c0, c1 = _cell_box(info, *box)
            sealed[r0:r1, c0:c1] = False

        labels, count = ndimage.label(sealed)
        cell_area = info.resolution ** 2
        border = set(labels[0, :]) | set(labels[-1, :]) | \
            set(labels[:, 0]) | set(labels[:, -1])
        for label in range(1, count + 1):
            if label in border:
                continue
            cells = np.argwhere(labels == label)
            area = len(cells) * cell_area
            if area < self.params['min_room_area']:
                continue
            rows, cols = cells[:, 0], cells[:, 1]
            self.rooms.append({
                'name': f'room_{len(self.rooms)}',
                'label': int(label),
                'area': float(area),
                'centre': [float(info.origin.position.x + (cols.mean() + 0.5) * info.resolution),
                           float(info.origin.position.y + (rows.mean() + 0.5) * info.resolution)],
                'bbox': [float(info.origin.position.x + (cols.min() + 0.5) * info.resolution),
                         float(info.origin.position.y + (rows.min() + 0.5) * info.resolution),
                         float(info.origin.position.x + (cols.max() + 0.5) * info.resolution),
                         float(info.origin.position.y + (rows.max() + 0.5) * info.resolution)],
                'doors': [], 'tables': [],
            })
        self._labels = labels
        self._name_rooms()

    def _name_rooms(self):
        """Attach human names from `room_labels`; an occupancy grid has no colour."""
        for name, x, y in self.params['room_labels']:
            room = self.room_at(x, y)
            if room is None:
                self.warnings.append(
                    f'room label "{name}" points at ({x:.2f}, {y:.2f}), '
                    f'which is not inside any detected room')
                continue
            if not room['name'].startswith('room_'):
                self.warnings.append(
                    f'room labels "{room["name"]}" and "{name}" resolve to the same room')
                continue
            room['name'] = name

    def room_at(self, x, y):
        row, col = _cell(self.info, x, y)
        if not (0 <= row < self.info.height and 0 <= col < self.info.width):
            return None
        label = int(self._labels[row, col])
        return next((r for r in self.rooms if r['label'] == label), None)

    def _attach_doors_to_rooms(self):
        """A door belongs to the rooms found just outside each of its jambs."""
        reach = self.params['seal_depth'] + self.params['chute_length'] / 2.0
        for index, door in enumerate(self.doors):
            nx, ny = door['normal']
            door['rooms'] = []
            for side in (1.0, -1.0):
                room = self.room_at(door['x'] + side * reach * nx,
                                    door['y'] + side * reach * ny)
                door['rooms'].append(None if room is None else room['name'])
                if room is not None and index not in room['doors']:
                    room['doors'].append(index)
            if None in door['rooms']:
                self.warnings.append(
                    f'door {index} at ({door["x"]:.2f}, {door["y"]:.2f}) does not '
                    f'connect two detected rooms: {door["rooms"]}')

    def _attach_tables_to_rooms(self):
        for index, table in enumerate(self.tables):
            room = self.room_at(table['x'], table['y'])
            table['room'] = None if room is None else room['name']
            if room is not None:
                room['tables'].append(index)

    # ---------------------------------------------------------------- poses
    def _build_poses(self):
        """Portal poses outside the guide walls, and one approach pose per table.

        The portal standoff is not decoration: the robot has to be able to turn
        on the spot there without a corner entering the lane, so it sits at
        least a circumscribed radius clear of the guide walls.
        """
        standoff = self.params['chute_length'] + self.params['portal_standoff']
        for door in self.doors:
            nx, ny = door['normal']
            door['portals'] = {}
            for side, room_name in zip((1.0, -1.0), door['rooms']):
                if room_name is None:
                    continue
                # Travelling out of `room_name` means heading against `side`.
                yaw = math.atan2(-side * ny, -side * nx)
                door['portals'][room_name] = {
                    'approach': [door['x'] + side * standoff * nx,
                                 door['y'] + side * standoff * ny, yaw],
                    'exit': [door['x'] - side * standoff * nx,
                             door['y'] - side * standoff * ny, yaw],
                }

        for table in self.tables:
            face = self._table_face(table)
            table['face'] = list(face)
            half = abs(face[0]) * table['size_x'] / 2.0 + abs(face[1]) * table['size_y'] / 2.0
            distance = (half + self.params['table_clearance']
                        + self.params['table_standoff'])
            table['approach'] = [table['x'] + face[0] * distance,
                                 table['y'] + face[1] * distance,
                                 math.atan2(-face[1], -face[0])]

    def _table_face(self, table):
        """The side to approach from: the one facing this room's nearest door.

        Snapped to an axis, because a rectilinear route is the whole point and
        the tables here are axis-aligned.
        """
        doors = [d for d in self.doors if table['room'] in (d['rooms'] or [])]
        if not doors:
            return (0.0, 1.0)
        door = min(doors, key=lambda d: math.hypot(d['x'] - table['x'],
                                                   d['y'] - table['y']))
        dx, dy = door['x'] - table['x'], door['y'] - table['y']
        return (math.copysign(1.0, dx), 0.0) if abs(dx) > abs(dy) \
            else (0.0, math.copysign(1.0, dy))

    # --------------------------------------------------------------- output
    def mask(self):
        """Rasterise the keepout rectangles onto a grid shaped like the map."""
        data = np.zeros((self.info.height, self.info.width), dtype=np.int8)
        for _kind, _index, x0, x1, y0, y1, value in self.rects:
            r0, r1, c0, c1 = _cell_box(self.info, x0, x1, y0, y1)
            data[r0:r1, c0:c1] = value
        return data

    def graph(self):
        return {
            'frame_id': 'map',
            'rooms': [{k: v for k, v in r.items() if k != 'label'} for r in self.rooms],
            'doors': [{'id': i, 'centre': [d['x'], d['y']], 'normal': d['normal'],
                       'width': d['width'], 'rooms': d['rooms'],
                       'portals': d.get('portals', {})}
                      for i, d in enumerate(self.doors)],
            'tables': [{'id': i, 'centre': [t['x'], t['y']],
                        'size': [t['size_x'], t['size_y']], 'room': t['room'],
                        'face': t['face'], 'approach': t['approach']}
                       for i, t in enumerate(self.tables)],
            'warnings': self.warnings,
        }


class _Msg:
    """Adapter so the detectors can be handed a plain array plus map info."""

    def __init__(self, grid, info):
        self.data = grid.reshape(-1)
        self.info = info


def _cell(info, x, y):
    return (int((y - info.origin.position.y) / info.resolution),
            int((x - info.origin.position.x) / info.resolution))


def _cell_box(info, x0, x1, y0, y1):
    """Half-open row/column slice covering a world-frame rectangle."""
    r0, c0 = _cell(info, min(x0, x1), min(y0, y1))
    r1, c1 = _cell(info, max(x0, x1), max(y0, y1))
    return (max(0, r0), min(info.height, r1 + 1),
            max(0, c0), min(info.width, c1 + 1))


def default_params(node=None):
    """Zone geometry. Defaults are derived, not guessed - see the comments."""
    spec = {
        # Guide walls run this far into each room from the wall plane. They only
        # have to commit the robot to the lane before its front reaches the
        # opening, so 0.85 m is comfortably more than its 0.52 m half length.
        # Longer is not safer: at 1.20 m the guide wall came within 5 cm of the
        # robot's swept circle when it turned on the spot in front of the table
        # (0.673 m circumscribed radius, plus 0.10 m of goal tolerance), and it
        # snagged. Anything that changes this must re-run scripts/check_zones.py,
        # which checks the turn clearance at every pose the navigator stops at.
        'chute_length': 0.50,
        # Thick enough that the planner cannot squeeze a path around the
        # outside of a guide wall at map resolution.
        'chute_thickness': 0.65,
        # Negative widens the lane. Widening by 15 cm a side puts the guide walls
        # comfortably clear of the 0.95 m opening, preventing lethal keepout
        # collisions while committing the planner to a square approach.
        'lane_margin': -0.15,
        # Portal poses must clear the guide walls by more than the robot's
        # circumscribed radius (hypot(0.52, 0.427) = 0.673 m) so the robot can
        # turn to the door heading without a corner entering the lane - plus the
        # 0.10 m it may stop short by and room for the drift of a DWB turn, which
        # rotates and translates at once. 0.673 + 0.10 + 0.15 = 0.923.
        'portal_standoff': 0.95,
        'table_clearance': 0.55,
        'table_standoff': 0.60,
        # The head-on gate spans the full width of the halo, not just the table:
        # at 0.20 m the robot could drive in but not turn round again without a
        # corner sweeping into the halo beside the gate. The three other sides
        # still close the halo, so no route through the room can graze the table.
        'table_gate_margin': 0.55,
        # Sealing depth when splitting free space into rooms: more than the
        # 0.10 m wall thickness, less than a room.
        'seal_depth': 0.30,
        'min_room_area': 8.0,
    }
    if node is not None:
        for key, value in spec.items():
            spec[key] = node.declare_parameter(key, value).value
    labels = ['home:0.0,0.0', 'blue:0.0,-6.0', 'red:6.0,0.0']
    if node is not None:
        labels = node.declare_parameter('room_labels', labels).value
    spec['room_labels'] = [(item.split(':')[0],
                            float(item.split(':')[1].split(',')[0]),
                            float(item.split(':')[1].split(',')[1]))
                           for item in labels]
    return spec


class NavZones(Node):
    def __init__(self):
        super().__init__('nav_zones')
        self._params = default_params(self)
        self._zones = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._mask_pub = self.create_publisher(OccupancyGrid, '/keepout_filter_mask', latched)
        self._graph_pub = self.create_publisher(String, '/nav_graph', latched)
        self._marker_pub = self.create_publisher(MarkerArray, '/nav_zones_markers', latched)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.get_logger().info('waiting for /map to build navigation zones')

    def _on_map(self, msg):
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        zones = Zones(grid, msg.info, self._params)
        if not zones.doors:
            self.get_logger().warn('no doorways detected in /map - publishing no zones')
            return
        self._zones = zones
        for warning in zones.warnings:
            self.get_logger().warn(warning)

        mask = OccupancyGrid()
        mask.header.frame_id = 'map'
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info = msg.info
        mask.data = zones.mask().reshape(-1).tolist()
        self._mask_pub.publish(mask)

        graph = String()
        graph.data = json.dumps(zones.graph(), allow_nan=False)
        self._graph_pub.publish(graph)
        self._marker_pub.publish(self._markers(zones))

        self.get_logger().info(
            f'{len(zones.rooms)} room(s) {[r["name"] for r in zones.rooms]}, '
            f'{len(zones.doors)} door(s), {len(zones.tables)} table(s), '
            f'{len(zones.rects)} keepout rectangles')

    def _markers(self, zones):
        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        counter = [0]

        def new(kind, ns):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = ns
            marker.id = counter[0]
            counter[0] += 1
            marker.type = kind
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            return marker

        for kind, _index, x0, x1, y0, y1, value in zones.rects:
            if value == FREE:
                continue      # the gate is a hole in a halo, not a zone to draw
            box = new(Marker.CUBE, f'keepout_{kind}')
            box.pose.position.x = (x0 + x1) / 2.0
            box.pose.position.y = (y0 + y1) / 2.0
            box.pose.position.z = 0.02
            box.scale.x = abs(x1 - x0)
            box.scale.y = abs(y1 - y0)
            box.scale.z = 0.04
            box.color.r, box.color.g, box.color.b, box.color.a = 0.9, 0.2, 0.2, 0.45
            array.markers.append(box)

        for door in zones.doors:
            lane = new(Marker.CUBE, 'lane')
            lane.pose.position.x = door['x']
            lane.pose.position.y = door['y']
            lane.pose.position.z = 0.01
            span = 2.0 * self._params['chute_length']
            along = door['width']
            lane.scale.x = along if door['wall_axis'] == 'x' else span
            lane.scale.y = span if door['wall_axis'] == 'x' else along
            lane.scale.z = 0.02
            lane.color.r, lane.color.g, lane.color.b, lane.color.a = 0.1, 0.9, 0.3, 0.30
            array.markers.append(lane)
            for room_name, portal in door.get('portals', {}).items():
                for role in ('approach', 'exit'):
                    x, y, yaw = portal[role]
                    arrow = new(Marker.ARROW, 'portals')
                    arrow.pose.position.x = x
                    arrow.pose.position.y = y
                    arrow.pose.position.z = 0.10
                    arrow.pose.orientation.z = math.sin(yaw / 2.0)
                    arrow.pose.orientation.w = math.cos(yaw / 2.0)
                    arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.55, 0.10, 0.10
                    arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (
                        (0.2, 0.7, 1.0, 0.9) if role == 'approach' else (0.2, 0.4, 1.0, 0.9))
                    array.markers.append(arrow)
                    text = new(Marker.TEXT_VIEW_FACING, 'portals')
                    text.pose.position.x = x
                    text.pose.position.y = y
                    text.pose.position.z = 0.45
                    text.scale.z = 0.22
                    text.color.r = text.color.g = text.color.b = text.color.a = 1.0
                    text.text = f'{room_name} {role}'
                    array.markers.append(text)

        for table in zones.tables:
            x, y, yaw = table['approach']
            arrow = new(Marker.ARROW, 'tables')
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.10
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.55, 0.10, 0.10
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.8, 0.1, 0.9
            array.markers.append(arrow)
            text = new(Marker.TEXT_VIEW_FACING, 'tables')
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.45
            text.scale.z = 0.22
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'{table["room"]} table'
            array.markers.append(text)
        return array


def main():
    rclpy.init()
    node = NavZones()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
