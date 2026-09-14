#!/usr/bin/env python3
"""Build navigation zones from a saved map and check them, without a simulator.

The zones decide where the robot is allowed to be and which way it must face
near a doorway, so they have to be right before anything drives.  This runs the
same `Zones` the node runs, prints the room graph it derives and every pose it
would publish, and can draw the result.  A wrong lane or a portal pose sitting
inside a guide wall shows up here in a second instead of as a jammed robot.

    ./scripts/run_native.sh python3 scripts/check_zones.py
    ./scripts/run_native.sh python3 scripts/check_zones.py --png /tmp/zones.png
    ./scripts/run_native.sh python3 scripts/check_zones.py --ascii
"""

import argparse
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'pas_dual_arm_scripts'))
sys.path.insert(0, os.path.join(REPO, 'scripts'))

from check_doors import DEFAULT_MAP, load_map  # noqa: E402
from pas_dual_arm_scripts.nav_zones import KEEPOUT, Zones, default_params  # noqa: E402

# Half length and half width of the Nav2 footprint; the circumscribed radius is
# what the robot sweeps when it turns on the spot.
HALF_LENGTH, HALF_WIDTH = 0.52, 0.427
CIRCUMSCRIBED = math.hypot(HALF_LENGTH, HALF_WIDTH)
# How far off the commanded pose the robot may be standing when it turns.
# nav2_params.yaml now delivers to 0.05 m (both general_goal_checker and
# FollowPath); this keeps the old 0.10 deliberately, as headroom - a check that
# only just passes at the delivered tolerance is not a check.
XY_TOLERANCE = 0.10
# Room demanded beyond the swept circle. Not padding for its own sake: DWB scores
# trajectories that rotate and translate together, so a "turn on the spot" drifts.
# A run with 5 cm of bare clearance left at the table snagged the guide wall, so
# bare non-overlap is not the test - this margin is.
TURN_MARGIN = 0.15


def check(zones, grid_obj):
    """Assert the properties the navigator depends on. Returns a failure list."""
    failures = []
    mask = zones.mask()
    info = zones.info

    def keepout_at(x, y):
        row = int((y - info.origin.position.y) / info.resolution)
        col = int((x - info.origin.position.x) / info.resolution)
        if not (0 <= row < info.height and 0 <= col < info.width):
            return True
        # Only a full-value cell forbids anything. Below that the mask carries
        # the graded table field, which is a cost to be preferred against, not a
        # place the robot may not be (nav_zones.Zones.table_field).
        return mask[row, col] >= KEEPOUT

    def can_turn(x, y):
        """Room to turn on the spot, allowing for stopping a tolerance off.

        The robot does not pivot cleanly - DWB scores trajectories that rotate
        and translate at once - so the swept circle is checked from the worst
        arrival position, not just from the pose that was commanded.
        """
        reach = CIRCUMSCRIBED + TURN_MARGIN
        for offset in np.linspace(0, 2 * math.pi, 8, endpoint=False):
            ox = x + XY_TOLERANCE * math.cos(offset)
            oy = y + XY_TOLERANCE * math.sin(offset)
            for angle in np.linspace(0, 2 * math.pi, 36, endpoint=False):
                if keepout_at(ox + reach * math.cos(angle),
                              oy + reach * math.sin(angle)):
                    return (ox, oy)
        return None

    # Every pose the navigator stops at, doorway portals and table approaches
    # alike. Missing the table approach here is what let a too-long guide wall
    # through: the robot reached the table fine and then jammed turning round.
    stops = [(f'door {i} {room} {role}', portal[role][0], portal[role][1])
             for i, door in enumerate(zones.doors)
             for room, portal in (door.get('portals') or {}).items()
             for role in ('approach', 'exit')]
    stops += [(f'table {i} ({t["room"]}) approach', t['approach'][0], t['approach'][1])
              for i, t in enumerate(zones.tables)]
    for label, x, y in stops:
        if keepout_at(x, y):
            failures.append(f'{label}: pose is inside a keepout zone')
            continue
        jam = can_turn(x, y)
        if jam:
            failures.append(
                f'{label}: cannot turn on the spot - the {CIRCUMSCRIBED:.3f} m swept '
                f'circle reaches a keepout zone from ({jam[0]:+.2f}, {jam[1]:+.2f}), '
                f'{XY_TOLERANCE:.2f} m off the commanded pose')

    for index, door in enumerate(zones.doors):
        nx, ny = door['normal']
        # The lane itself must stay open along the whole guide-wall run.
        for step in np.linspace(-zones.params['chute_length'],
                                zones.params['chute_length'], 25):
            if keepout_at(door['x'] + step * nx, door['y'] + step * ny):
                failures.append(f'door {index}: lane is blocked {step:+.2f} m along its normal')
                break
        for room_name, portal in door.get('portals', {}).items():
            for role in ('approach', 'exit'):
                x, y, yaw = portal[role]
                # Heading must be along the door normal, not merely near it.
                if min(abs(_wrap(yaw - math.atan2(ny, nx))),
                       abs(_wrap(yaw - math.atan2(-ny, -nx)))) > 1e-6:
                    failures.append(f'door {index} {room_name} {role}: yaw is not along the door normal')
        if len(door.get('rooms') or []) != 2 or None in (door.get('rooms') or []):
            failures.append(f'door {index}: does not join two rooms ({door.get("rooms")})')

    for index, table in enumerate(zones.tables):
        x, y, _yaw = table['approach']
        if keepout_at(x, y):
            failures.append(f'table {index}: approach pose is inside its own halo')
        if table['room'] is None:
            failures.append(f'table {index}: is not inside any detected room')
        # No halo any more: a table the robot has to pick a box off is not a
        # place to fence off (the user, after run 62). What still has to hold is
        # that the head-on lane to it is clear of the doorway zones, which are
        # the only zones left and do sit near the tables in a 6 m room.
        fx, fy = table['face']
        # Clear from the approach pose down to the DOCK pose, and no further: past
        # the dock the field is lethal on purpose, because that is where the robot
        # would be standing inside the table. The dock is itself derived from the
        # geometry (top + half length + safety), so this checks the robot can
        # actually be delivered to the closest pose it is allowed to hold.
        dock = table.get('dock')
        if dock is None:
            failures.append(f'table {index}: no dock pose')
            continue
        run_in = math.hypot(x - dock[0], y - dock[1])
        for step in np.linspace(0.0, run_in, 20):
            if keepout_at(x - fx * step, y - fy * step):
                failures.append(f'table {index}: approach lane blocked {step:.2f} m in, '
                                f'before the dock pose {run_in:.2f} m in')
                break
        if keepout_at(dock[0], dock[1]):
            failures.append(f'table {index}: the dock pose itself is lethal - '
                            f'the planner has nowhere to deliver the robot to')

    failures += _check_furrows(zones)
    failures += _check_planner_lane(zones)
    failures += _check_room_still_passable(zones, grid_obj)
    return failures


def _check_planner_lane(zones):
    """The grown mask must still leave the planner a lane through each doorway.

    The zones are handed to the global planner already grown by the robot's
    inscribed radius, because NavFn plans a point (see Zones.mask).  That is the
    right thing to do and it is also the thing that can close the one gap the
    robot has to fit through: the lane is 1.25 m and growing both guide walls by
    0.427 m leaves 0.396 m for the centre.  It has to leave enough for the centre
    to pass, and the centre has to be able to be wrong by the goal tolerance.
    """
    info = zones.info
    grown = zones.mask(grow=zones.params['planner_inflation'])
    failures = []

    def blocked(x, y):
        row = int((y - info.origin.position.y) / info.resolution)
        col = int((x - info.origin.position.x) / info.resolution)
        if not (0 <= row < info.height and 0 <= col < info.width):
            return True
        return grown[row, col] >= KEEPOUT

    for index, door in enumerate(zones.doors):
        nx, ny = door['normal']
        px, py = -ny, nx                      # along the wall, across the lane
        reach = zones.params['chute_length'] + zones.params['portal_standoff']
        width = None
        for along in np.linspace(-reach, reach, 41):
            x = door['x'] + along * nx
            y = door['y'] + along * ny
            # The contiguous run through the axis, not every clear sample:
            # outside the guide walls the room is open again, and counting that
            # would report a lane that is wide because it is not there.
            offsets = np.linspace(-0.9, 0.9, 181)
            clear = [not blocked(x + a * px, y + a * py) for a in offsets]
            middle = len(offsets) // 2
            if not clear[middle]:
                failures.append(f'door {index}: the grown zones close the lane '
                                f'on its own axis, {along:+.2f} m along the normal')
                break
            lo = hi = middle
            while lo > 0 and clear[lo - 1]:
                lo -= 1
            while hi < len(offsets) - 1 and clear[hi + 1]:
                hi += 1
            span = offsets[hi] - offsets[lo]
            width = span if width is None else min(width, span)
        else:
            # The yardstick is the doorway itself, not a goal tolerance. The
            # opening is what it is: the centre of the robot fits through
            # width - 2 * inscribed radius and no more, and that is the number
            # the doorway has worked at all along. What must not happen is the
            # ZONES making it narrower than the doorway already does.
            core = zones.params['planner_inflation']
            physical = zones.doors[index]['width'] - 2 * core
            if width <= 0.0:
                failures.append(f'door {index}: the zones close the lane completely')
            elif width < physical - 2 * info.resolution:
                failures.append(
                    f'door {index}: the zones leave the planner {width:.3f} m for the '
                    f'robot centre where the opening itself allows {physical:.3f} m - '
                    f'the zones, not the doorway, are the constraint')
            else:
                print(f'planner lane, door {index}: {width:.3f} m for the robot centre '
                      f'(the {zones.doors[index]["width"]:.2f} m opening allows '
                      f'{physical:.3f} m; the zones are not the limit)')
    return failures


def _check_furrows(zones):
    """The furrows must really be free of field, and the core must really be there.

    Two failures this catches, both already paid for once:

    A furrow that is not at zero cost is what broke run 65. NavFn extracts its path
    by walking the potential downhill and its cost saturates at 253; the robot's
    centre may only be 0.427 to 0.49 m from a jamb in a 0.98 m opening, so any ramp
    reaching into that corridor flattens the potential and the walk fails outright.
    "The furrow is zero" is therefore a condition, not a preference.

    And a furrow that eats the core is the opposite mistake: the field is switched
    off in the corridor, so nothing but the core stops the planner routing the
    robot's centre into the wall the corridor passes through.
    """
    info = zones.info
    field = zones.field(grow=zones.params['planner_inflation'])
    failures = []

    def value(x, y):
        row, col = int((y - info.origin.position.y) / info.resolution), \
            int((x - info.origin.position.x) / info.resolution)
        if not (0 <= row < info.height and 0 <= col < info.width):
            return None
        return int(field[row, col])

    for label, cx, cy, ux, uy, half_along, half_across in zones._furrows():
        px, py = -uy, ux
        graded = []
        for along in np.linspace(-half_along, half_along, 41):
            for across in np.linspace(-half_across, half_across, 21):
                v = value(cx + along * ux + across * px, cy + along * uy + across * py)
                if v is not None and 0 < v < 100:
                    graded.append((along, across, v))
        if graded:
            along, across, v = graded[0]
            failures.append(
                f'{label} furrow: field is {v} at {along:+.2f} m along / '
                f'{across:+.2f} m across, not 0 - NavFn cannot walk a saturated '
                f'potential (run 65)')

    # The core still has to exist: sample right beside every mapped wall.
    occupied = zones.grid == 100
    rows, cols = np.nonzero(occupied)
    if rows.size:
        step = max(1, rows.size // 400)
        core = zones.params['planner_inflation']
        holes = 0
        for row, col in zip(rows[::step], cols[::step]):
            x = info.origin.position.x + col * info.resolution
            y = info.origin.position.y + row * info.resolution
            for dx, dy in ((core * 0.5, 0), (-core * 0.5, 0), (0, core * 0.5), (0, -core * 0.5)):
                v = value(x + dx, y + dy)
                if v is not None and v < 100:
                    holes += 1
                    break
        if holes:
            failures.append(f'the core is missing beside {holes} sampled wall cell(s) - '
                            f'a furrow has opened a hole in something solid')
    if not failures:
        print(f'furrows: {len(zones._furrows())} corridor(s) at zero cost, '
              f'core intact beside every sampled wall')
    return failures


def _check_room_still_passable(zones, grid_obj):
    """Can a robot-sized body still get from every stop pose to every other?

    A keepout zone is added to stop the robot going somewhere, and the failure it
    is easy not to notice is that it also stopped the robot going somewhere it
    needed to.  That is the rule the project already keeps (D-18: nothing that can
    refuse a drive goes in without evidence it has to), and closing the table halo
    is exactly the kind of change that can break it - the red table's halo leaves
    1.48 m between itself and the east wall for a 0.854 m robot.

    So: take the free cells, subtract the keepout, shrink what is left by the
    robot's half width, and check that every pose the navigator stops at survives
    and that they all land in one connected piece.  Erosion by the half width is
    the honest test for a corridor, which the robot drives down lengthwise.
    """
    import cv2

    info = zones.info
    grid = np.asarray(grid_obj.data, dtype=np.int16).reshape(info.height, info.width)
    passable = ((grid == 0) & (zones.mask() < KEEPOUT)).astype(np.uint8)

    # cv2's distance transform measures to the nearest zero, which is exactly the
    # distance to the nearest wall or zone; a robot centre needs half a width.
    reach = HALF_WIDTH / info.resolution
    room = (cv2.distanceTransform(passable, cv2.DIST_L2, 5) >= reach).astype(np.uint8)
    count, labels = cv2.connectedComponents(room)

    stops = [(f'door {i} {r} {role}', p[role][0], p[role][1])
             for i, d in enumerate(zones.doors)
             for r, p in (d.get('portals') or {}).items()
             for role in ('approach', 'exit')]
    stops += [(f'table {i} ({t["room"]}) approach', t['approach'][0], t['approach'][1])
              for i, t in enumerate(zones.tables)]
    # A room centre is only ever a goal for a room with no table in it:
    # room_navigator sends the robot to the table's approach pose otherwise
    # (room_navigator.plan, the `square up in front of the table` leg). Checking
    # the centre of a room whose centre IS the table would be checking a pose the
    # navigator will never ask for - and the answer would always be no.
    stops += [(f'{r["name"]} centre', r['centre'][0], r['centre'][1])
              for r in zones.rooms if not r['tables']]

    failures, seen = [], {}
    for label, x, y in stops:
        row = int((y - info.origin.position.y) / info.resolution)
        col = int((x - info.origin.position.x) / info.resolution)
        if not (0 <= row < info.height and 0 <= col < info.width) or labels[row, col] == 0:
            failures.append(f'{label}: no room for a {2 * HALF_WIDTH:.3f} m robot here - '
                            f'the zones closed around it')
            continue
        seen[label] = int(labels[row, col])

    if len(set(seen.values())) > 1:
        groups = {}
        for label, component in seen.items():
            groups.setdefault(component, []).append(label)
        parts = ' | '.join(', '.join(sorted(names)) for names in groups.values())
        failures.append(f'the zones cut the world into {len(groups)} pieces a robot '
                        f'cannot drive between: {parts}')
    elif seen:
        print(f'passable: {count - 1} region(s) wide enough for the robot; all '
              f'{len(seen)} stop poses reachable from one another')
    return failures


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def render(zones, grid, path):
    """Write a PNG: map in grey, keepout red, lanes green, poses blue."""
    from PIL import Image
    height, width = grid.shape
    rgb = np.full((height, width, 3), 245, dtype=np.uint8)
    rgb[grid == -1] = (205, 205, 205)
    rgb[grid == 100] = (30, 30, 30)
    mask = zones.mask()
    rgb[mask != 0] = (230, 90, 90)
    for kind, _i, x0, x1, y0, y1, _value in zones.rects:
        if kind != 'table':
            continue
        r0, r1, c0, c1 = _box(zones.info, x0, x1, y0, y1)
        block = rgb[r0:r1, c0:c1]
        # Only where the halo actually survived, in case a zone overpaints it.
        block[mask[r0:r1, c0:c1] != 0] = (235, 160, 60)
    for door in zones.doors:
        nx, ny = door['normal']
        for step in np.linspace(-zones.params['chute_length'], zones.params['chute_length'], 60):
            _dot(rgb, zones.info, door['x'] + step * nx, door['y'] + step * ny, (60, 180, 80))
    for door in zones.doors:
        for portal in door.get('portals', {}).values():
            for role in ('approach', 'exit'):
                _dot(rgb, zones.info, portal[role][0], portal[role][1], (40, 90, 220), 2)
    for table in zones.tables:
        _dot(rgb, zones.info, table['approach'][0], table['approach'][1], (220, 170, 20), 2)
    Image.fromarray(np.flipud(rgb)).resize((width * 3, height * 3), Image.NEAREST).save(path)
    print(f'wrote {path}')


def _box(info, x0, x1, y0, y1):
    r0 = int((min(y0, y1) - info.origin.position.y) / info.resolution)
    r1 = int((max(y0, y1) - info.origin.position.y) / info.resolution) + 1
    c0 = int((min(x0, x1) - info.origin.position.x) / info.resolution)
    c1 = int((max(x0, x1) - info.origin.position.x) / info.resolution) + 1
    return max(0, r0), r1, max(0, c0), c1


def _dot(rgb, info, x, y, colour, radius=0):
    row = int((y - info.origin.position.y) / info.resolution)
    col = int((x - info.origin.position.x) / info.resolution)
    height, width = rgb.shape[:2]
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if 0 <= row + dr < height and 0 <= col + dc < width:
                rgb[row + dr, col + dc] = colour


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml', nargs='?', default=DEFAULT_MAP)
    parser.add_argument('--png', help='write an overlay image here')
    parser.add_argument('--ascii', action='store_true', help='print a coarse overlay')
    args = parser.parse_args()

    grid_obj, _meta = load_map(args.map_yaml)
    info = grid_obj.info
    grid = np.asarray(grid_obj.data, dtype=np.int16).reshape(info.height, info.width)
    params = default_params()
    zones = Zones(grid, info, params)

    print(f'map    : {args.map_yaml}')
    print('params : ' + ', '.join(f'{k}={v}' for k, v in params.items() if k != 'room_labels'))

    print(f'\nrooms  ({len(zones.rooms)}):')
    for room in zones.rooms:
        bbox = room['bbox']
        print(f'  {room["name"]:<8} area {room["area"]:6.1f} m^2  '
              f'centre ({room["centre"][0]:+.2f}, {room["centre"][1]:+.2f})  '
              f'bbox x[{bbox[0]:+.2f},{bbox[2]:+.2f}] y[{bbox[1]:+.2f},{bbox[3]:+.2f}]  '
              f'doors {room["doors"]}  tables {room["tables"]}')

    print(f'\ndoors  ({len(zones.doors)}):')
    for index, door in enumerate(zones.doors):
        print(f'  [{index}] centre ({door["x"]:+.3f}, {door["y"]:+.3f})  '
              f'width {door["width"]:.3f} m  normal {door["normal"]}  '
              f'joins {door["rooms"]}')
        for room_name, portal in door.get('portals', {}).items():
            a, e = portal['approach'], portal['exit']
            print(f'        from {room_name:<6} approach ({a[0]:+.3f}, {a[1]:+.3f}, '
                  f'{math.degrees(a[2]):+7.2f} deg) -> exit ({e[0]:+.3f}, {e[1]:+.3f})')

    print(f'\ntables ({len(zones.tables)}):')
    for index, table in enumerate(zones.tables):
        a = table['approach']
        print(f'  [{index}] centre ({table["x"]:+.3f}, {table["y"]:+.3f})  '
              f'legs {table["size_x"]:.2f} x {table["size_y"]:.2f} m  '
              f'room {table["room"]}  face {table["face"]}')
        print(f'        approach ({a[0]:+.3f}, {a[1]:+.3f}, {math.degrees(a[2]):+7.2f} deg), '
              f'{math.hypot(a[0] - table["x"], a[1] - table["y"]):.2f} m out')

    keepout_cells = int((zones.mask() != 0).sum())
    print(f'\nmask   : {len(zones.rects)} rectangles, {keepout_cells} cells '
          f'({keepout_cells * info.resolution ** 2:.1f} m^2)')

    for warning in zones.warnings:
        print(f'  WARN {warning}')

    failures = check(zones, grid_obj)
    print()
    for failure in failures:
        print(f'  FAIL {failure}')

    if args.png:
        render(zones, grid, args.png)
    if args.ascii:
        _ascii(zones, grid)

    print('\nPASS' if not failures else '\nFAIL')
    return 0 if not failures else 1


def _ascii(zones, grid, step=4):
    info = zones.info
    mask = zones.mask()
    print('\noverlay ("#" wall, "K" keepout, "." free, "?" unknown):')
    for row in range(info.height - 1, -1, -step):
        line = ''
        for col in range(0, info.width, step):
            block_g = grid[max(0, row - step + 1):row + 1, col:col + step]
            block_m = mask[max(0, row - step + 1):row + 1, col:col + step]
            if (block_g == 100).any():
                line += '#'
            elif (block_m != 0).any():
                line += 'K'
            elif (block_g == -1).all():
                line += '?'
            else:
                line += '.'
        print(f'  y={info.origin.position.y + row * info.resolution:6.2f} {line}')


if __name__ == '__main__':
    sys.exit(main())
