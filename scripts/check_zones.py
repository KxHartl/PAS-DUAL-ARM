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
from pas_dual_arm_scripts.nav_zones import Zones, default_params  # noqa: E402

# Half length and half width of the Nav2 footprint; the circumscribed radius is
# what the robot sweeps when it turns on the spot.
HALF_LENGTH, HALF_WIDTH = 0.52, 0.427
CIRCUMSCRIBED = math.hypot(HALF_LENGTH, HALF_WIDTH)
# nav2_params.yaml general_goal_checker.xy_goal_tolerance: the robot can be this
# far from where it was sent, so the turn has to fit from there too.
XY_TOLERANCE = 0.10
# Room demanded beyond the swept circle. Not padding for its own sake: DWB scores
# trajectories that rotate and translate together, so a "turn on the spot" drifts.
# A run with 5 cm of bare clearance left at the table snagged the guide wall, so
# bare non-overlap is not the test - this margin is.
TURN_MARGIN = 0.15


def check(zones):
    """Assert the properties the navigator depends on. Returns a failure list."""
    failures = []
    mask = zones.mask()
    info = zones.info

    def keepout_at(x, y):
        row = int((y - info.origin.position.y) / info.resolution)
        col = int((x - info.origin.position.x) / info.resolution)
        if not (0 <= row < info.height and 0 <= col < info.width):
            return True
        return mask[row, col] != 0

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
        # The head-on lane from the approach pose to the table face must be open,
        # or the gate was cut on the wrong side.
        fx, fy = table['face']
        reach = math.hypot(x - table['x'], y - table['y'])
        for step in np.linspace(0.0, reach - 0.40, 20):
            if keepout_at(x - fx * step, y - fy * step):
                failures.append(f'table {index}: approach lane blocked {step:.2f} m in')
                break
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
        # Only where the halo actually survived; the gate is a hole in it.
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

    failures = check(zones)
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
