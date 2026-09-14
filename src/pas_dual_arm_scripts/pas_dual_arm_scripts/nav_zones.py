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
/keepout_filter_mask  nav_msgs/OccupancyGrid, latched - the zones as they are,
                      for the LOCAL costmap, whose controller scores the robot's
                      real footprint.  Replaces the static `filter_mask_server` PGM.
/keepout_filter_mask_planner
                      the same zones grown by the robot's inscribed radius, for
                      the GLOBAL costmap.  NavFn plans a point, so it has to be
                      shown zones that already account for the body - see
                      Zones.mask().
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
# Measured in ARM_CARRY_V2 (P-35), the same pair room_navigator works from.
# The robot is longer than it is wide, which is why an isotropic radius cannot be
# right for it: head-on its front reaches HALF_LENGTH, beside it only HALF_WIDTH.
HALF_LENGTH, HALF_WIDTH = 0.52, 0.427


class Zones:
    """Geometry of one map: rooms, doors, tables, keepout rectangles, poses.

    Kept free of ROS types so it can be built and checked offline against a
    saved map (`scripts/check_zones.py`) without a simulator.
    """

    def __init__(self, grid, info, params):
        self.info = info
        self.params = params
        # Kept: the field is computed from the occupied cells, not from the
        # features derived off them.
        self.grid = np.asarray(grid, dtype=np.int16).reshape(info.height, info.width)
        self.doors = door_candidates(_Msg(grid, info))
        self.tables = table_candidates(_Msg(grid, info))
        self.rects = []
        self.rooms = []
        self.warnings = []
        self._segment_rooms(grid)
        self._attach_doors_to_rooms()
        self._attach_tables_to_rooms()
        self._build_poses()
        # Guide walls are the only hand-drawn rectangles left. They are not a
        # field: they are geometry that forces a square entry into a doorway, and
        # the doorway works with them. Everything else the robot is meant to keep
        # away from is in the field (see `field`), which is computed from the map
        # rather than drawn around chosen features.
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

    def table_half_extent(self, table):
        """Half the table's real footprint, not the one the laser can see.

        `table_candidates` clusters leg posts, so `size_*` is the span between
        leg centres - 0.70 m for these tables.  The top is 0.80 m and overhangs
        them, and at 0.2086 m the scan plane passes under it: what the robot
        would actually meet appears in no sensor layer at all (P-39, finding 3).
        The approach pose is measured from here, so it is measured from the table
        rather than from the part of it the laser happens to see.
        """
        over = self.params['table_overhang']
        return table['size_x'] / 2.0 + over, table['size_y'] / 2.0 + over

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
            half_x, half_y = self.table_half_extent(table)
            half = abs(face[0]) * half_x + abs(face[1]) * half_y
            yaw = math.atan2(-face[1], -face[0])
            distance = half + self.params['table_standoff']
            table['approach'] = [table['x'] + face[0] * distance,
                                 table['y'] + face[1] * distance, yaw]
            # The dock pose is pure geometry: the robot's front touches the table
            # at half + HALF_LENGTH, so this stands it off by table_dock_safety.
            # It is NOT a pose to turn on - the circumscribed radius is 0.673 m
            # and from here the swept circle reaches inside the table - so the
            # navigator only ever leaves it by reversing back down this same axis
            # (room_navigator._table_legs).
            dock = half + HALF_LENGTH + self.params['table_dock_safety']
            table['dock'] = [table['x'] + face[0] * dock,
                             table['y'] + face[1] * dock, yaw]
            if dock >= distance:
                self.warnings.append(
                    f'table at ({table["x"]:.2f}, {table["y"]:.2f}): dock pose '
                    f'{dock:.2f} m out is not closer than the approach pose '
                    f'{distance:.2f} m; table_standoff is too small to back out to')

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
    def _furrows(self):
        """Where the field is deliberately switched off, as (kind, corridor) tests.

        A field that repels everywhere also repels in the one place the robot has
        no room. These are the places it must not: the corridor through each
        doorway, and the head-on lane to each table. In both the robot is meant to
        go straight at something, so a cost gradient there is not caution, it is
        an obstacle.

        Returned as (label, centre_x, centre_y, ux, uy, half_along, half_across)
        with (ux, uy) the corridor's own axis.
        """
        out = []
        reach = self.params['chute_length'] + self.params['portal_standoff']
        for index, door in enumerate(self.doors):
            nx, ny = door['normal']
            half_across = door['width'] / 2.0 - self.params['lane_margin']
            out.append((f'door {index}', door['x'], door['y'], nx, ny,
                        reach, half_across))
        across = HALF_WIDTH + self.params['furrow_margin']
        for index, table in enumerate(self.tables):
            fx, fy = table['face']
            half_x, half_y = self.table_half_extent(table)
            half = abs(fx) * half_x + abs(fy) * half_y
            # From the table's own edge out past the pose the robot squares up at.
            reach_out = self.params['table_standoff'] + half
            centre_x = table['x'] + fx * (half + reach_out) / 2.0
            centre_y = table['y'] + fy * (half + reach_out) / 2.0
            out.append((f'table {index}', centre_x, centre_y, fx, fy,
                        (reach_out - half) / 2.0 + self.info.resolution, across))
        return out

    def field(self, grow=0.0):
        """One cost surface for the whole map: obstacles repel, furrows do not.

        This replaces three separate things that used to do the repelling and
        disagreed about what repelling means - nav2's inflation layer (a hard
        prohibition at a fixed radius), the doorway guide walls (rectangles), and
        a hand-drawn ramp around tables. Stacked, they produced two different
        shapes around one table and a robot that could not get near it.

        The surface has two parts, and they are not the same kind of thing:

          core   `grow` metres of lethal cell around every occupied cell. NavFn
                 plans a POINT: without this it routes the centre one cell off a
                 wall. This is a prohibition and is sized by the robot.
          ramp   a linear falloff over `field_width` beyond the core, peaking at
                 `field_peak`. This is a preference - what makes the planner
                 choose the middle of a room over its edge.

        Then the furrows are cut: inside them the ramp is zero and only the core
        remains. That is how "attraction" is expressed on a costmap, which has no
        negative numbers - the doorway is not pulled at, everything beside it is
        pushed away, and what is left is a channel of free cost through the middle.

        It is also a hard requirement rather than a nicety. NavFn's cost is
        50 + 0.8 * v saturating at 253, and it extracts the path by walking the
        potential downhill; the centre of the robot may be 0.427 to 0.49 m from a
        jamb in a 0.98 m opening, so a ramp reaching into that corridor puts all
        of it at the cap and the walk fails outright (run 65, "Failed to create a
        plan from potential when a legal potential was found"). The furrow is what
        keeps that corridor at zero.

        `grow` is the robot's inscribed radius for the planner's copy and 0 for the
        controller's, which scores its real footprint against the voxel layer and
        would be counting the same metre twice.
        """
        info = self.info
        occupied = self.grid == 100
        if not occupied.any():
            return np.zeros(occupied.shape, dtype=np.int16)

        # Include table tops as occupied obstacles so the distance transform measures from
        # the real tabletop perimeter rather than from the 5 cm recessed legs.
        occupied_obstacles = occupied.copy()
        for t in self.tables:
            hx, hy = self.table_half_extent(t)
            r0, r1, c0, c1 = _cell_box(info, t['x'] - hx, t['x'] + hx, t['y'] - hy, t['y'] + hy)
            occupied_obstacles[r0:r1, c0:c1] = True

        # Distance from every cell to the nearest occupied one. scipy is already
        # a dependency here (_segment_rooms labels with it); cv2 is not, and this
        # runs inside a ROS node.
        distance = ndimage.distance_transform_edt(~occupied_obstacles) * info.resolution

        width = self.params['field_width']
        peak = self.params['field_peak']
        field = np.zeros(occupied.shape, dtype=np.int16)
        if width > 0.0 and peak > 0:
            ramp = peak * (1.0 - (distance - grow) / width)
            field = np.clip(ramp, 0, peak).astype(np.int16)

        # Cut the furrows out of the ramp. The core is re-applied afterwards, so
        # a furrow can never open a hole in a wall.
        ys = info.origin.position.y + np.arange(info.height) * info.resolution
        xs = info.origin.position.x + np.arange(info.width) * info.resolution
        gx, gy = np.meshgrid(xs, ys)
        for _label, cx, cy, ux, uy, half_along, half_across in self._furrows():
            dx, dy = gx - cx, gy - cy
            along = np.abs(dx * ux + dy * uy)
            across = np.abs(-dx * uy + dy * ux)
            # A cell is 2 cm wide and is addressed by its lower-left corner, so
            # zeroing exactly up to the declared edge leaves the boundary cell
            # short by up to a whole cell - and the corridor is declared for a
            # reason. A cell of slack makes the whole of it zero. The core is
            # re-applied afterwards, so the slack can never open a wall.
            slack = info.resolution
            field[(along <= half_along + slack) & (across <= half_across + slack)] = 0

        field[distance <= grow] = KEEPOUT
        field[occupied_obstacles] = KEEPOUT
        self._table_core(field, grow)
        return field

    def _table_core(self, field, grow):
        """Where the planner may not put the robot's centre near a table.

        The generic core is `grow` metres around every occupied cell, and around a
        table those cells are its LEGS - the 0.2086 m scan plane passes under the
        top. That is wrong twice over. The legs are 5 cm narrower than the top, so
        the core sits 5 cm too close; and `grow` is the inscribed radius, which is
        half the robot's WIDTH, while a robot driving at a table leads with half
        its LENGTH, 9.3 cm further. Together the generic core would let the planner
        route the centre 0.41 m inside the point where the robot's front touches
        the table, and only the footprint check would notice.

        So a table gets its core stated in its own terms: the real top grown by the
        robot's half width sideways, and out to the dock pose head-on, which is
        itself top + half length + safety. Nothing here is a preference - it is the
        geometry of a 1.04 x 0.854 m robot meeting a 0.80 m table.
        """
        across = HALF_WIDTH + self.params['furrow_margin']
        for table in self.tables:
            half_x, half_y = self.table_half_extent(table)
            r0, r1, c0, c1 = _cell_box(self.info,
                                       table['x'] - half_x - grow,
                                       table['x'] + half_x + grow,
                                       table['y'] - half_y - grow,
                                       table['y'] + half_y + grow)
            field[r0:r1, c0:c1] = KEEPOUT

            if grow:
                # Head-on, out to one cell short of the dock pose, which has to stay
                # free or the planner has nowhere to deliver the robot to.
                fx, fy = table['face']
                half = abs(fx) * half_x + abs(fy) * half_y
                reach = (half + HALF_LENGTH + self.params['table_dock_safety']
                         - 2 * self.info.resolution)
                x0, x1 = sorted((table['x'] + fx * half, table['x'] + fx * reach))
                y0, y1 = sorted((table['y'] + fy * half, table['y'] + fy * reach))
                r0, r1, c0, c1 = _cell_box(self.info,
                                           x0 - abs(fy) * across, x1 + abs(fy) * across,
                                           y0 - abs(fx) * across, y1 + abs(fx) * across)
                field[r0:r1, c0:c1] = KEEPOUT

    def field_profile(self, x0, y0, x1, y1, samples=25, grow=0.0):
        """Sample the field along a line - for printing a cross-section."""
        field = self.field(grow=grow)
        out = []
        for t in np.linspace(0.0, 1.0, samples):
            x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
            row, col = _cell(self.info, x, y)
            if 0 <= row < self.info.height and 0 <= col < self.info.width:
                out.append((x, y, int(field[row, col])))
        return out

    def mask(self, grow=0.0):
        """Rasterise the keepout rectangles onto a grid shaped like the map.

        `grow` expands every zone by that many metres, which is how the same
        zones get told to a planner and to a controller without either one being
        wrong.  NavFn plans a point: it only asks whether the robot's centre
        lands in a lethal cell, so against an un-grown zone it will happily route
        the centre one cell outside the boundary - a path whose every pose puts
        the robot's body inside the zone.  DWB then scores that body, rejects
        every trajectory, and the robot stops on a path that was never possible.

        Nav2's own inflation layer exists to close that gap, but it cannot here:
        it is a plugin and the keepout is a filter, and filters run after every
        plugin, so the zone is written into the costmap once inflation has
        already finished.  Growing the rectangles here does the same job, and
        for axis-aligned rectangles it is exact except at the corners, where a
        square grows where a disc would round - the conservative direction.
        """
        # The field goes down first: the guide walls are absolute, and must not
        # be softened by a ramp that happens to reach them.
        data = np.clip(self.field(grow=grow), 0, KEEPOUT).astype(np.int8)
        for _kind, _index, x0, x1, y0, y1, value in self.rects:
            if grow and value != FREE:
                # A guide wall is built outwards from the lane, so its corners
                # arrive in whichever order that put them in; growing them as
                # given would shrink half of them.
                x0, x1 = min(x0, x1) - grow, max(x0, x1) + grow
                y0, y1 = min(y0, y1) - grow, max(y0, y1) + grow
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
                        'face': t['face'], 'approach': t['approach'],
                        'dock': t['dock']}
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
        # The top overhangs the legs the detector finds: 0.80 m of table over a
        # 0.70 m leg span, and the 0.2086 m scan plane sees only the legs. 0.05 m
        # a side is the difference, added to every zone and pose built from a
        # table. Measured from seminar_world.sdf, not estimated; if the RGB-D top
        # detector (feature_registry, 'table_tops') ever reports a measured
        # extent, that measurement should supersede this.
        'table_overhang': 0.05,
        # Where the robot squares up in front of a table, measured from the
        # table's own edge to the robot's centre. 1.15 m puts it ~1.55 m from the
        # centre of these tables, which is the 1.50 m that worked in runs 46-58.
        # It only has to be close enough for the camera; the last metre is the
        # visual servo's, on raw cmd_vel, which reads no costmap at all.
        'table_standoff': 1.15,
        # The field (Zones.field). `field_width` is how far the ramp falls off
        # beyond the lethal core, `field_peak` how hard it pushes right at the
        # core - a mask value 0-100, which Costmap2D turns into 0-254 of cost and
        # NavFn charges as 50 + 0.8 * cost against 50 for open floor.
        #
        # The peak must stay well clear of NavFn's 253 cap or the potential
        # saturates and the path cannot be extracted from it (run 65). It is
        # chosen by measurement, not by feel: scripts/check_costmap_path.py
        # reports how far routes then keep from real obstacles, and the target is
        # 0.6-0.9 m - above the 0.52 m at which collision_monitor deadlocks
        # (run 64), below the point where routes hug the far wall instead
        # (run 68).
        # Measured with scripts/check_costmap_path.py --width/--peak: routes settle
        # just outside the ramp, so the WIDTH is what sets their clearance and the
        # peak only decides how firmly. With tabletop perimeter included in the
        # obstacle map, 0.40 m width and 50 peak gives 0.835 m clearance, providing
        # a clear, visible and reliable safety margin without hugging opposite walls.
        'field_width': 0.40,
        'field_peak': 50,
        # Half-width of a furrow beside the robot, on top of its own half width.
        'furrow_margin': 0.10,
        # Clearance from the table's edge to the robot's front at the dock pose.
        # Everything else about that pose is geometry: plate + HALF_LENGTH is
        # where the front touches.
        'table_dock_safety': 0.10,
        # How far the doorway zones are grown before the global planner is shown
        # them - see Zones.mask(). The robot's inscribed radius: the largest
        # circle that fits inside its 1.04 x 0.854 m footprint, which is the same
        # measure nav2's own inflation layer uses to decide a cell is untraversable.
        # Bigger would be safer per pose and would also close the 1.25 m doorway
        # lane, which is the one place the robot has to fit exactly.
        'planner_inflation': 0.427,
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
        # Two masks of the same zones. The controller checks the robot's real
        # footprint against the first; the planner, which reasons about a point,
        # is given the second - already grown by the robot's inscribed radius, so
        # a path it calls clear is one the body can actually be driven along.
        self._mask_pub = self.create_publisher(OccupancyGrid, '/keepout_filter_mask', latched)
        self._planner_mask_pub = self.create_publisher(
            OccupancyGrid, '/keepout_filter_mask_planner', latched)
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

        stamp = self.get_clock().now().to_msg()
        grow = self._params['planner_inflation']
        for publisher, cells in ((self._mask_pub, zones.mask()),
                                 (self._planner_mask_pub, zones.mask(grow=grow))):
            mask = OccupancyGrid()
            mask.header.frame_id = 'map'
            mask.header.stamp = stamp
            mask.info = msg.info
            mask.data = cells.reshape(-1).tolist()
            publisher.publish(mask)

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
                continue      # a rectangle that clears keepout is a hole, not a zone
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
