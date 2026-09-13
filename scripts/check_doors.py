#!/usr/bin/env python3
"""Run the live door detector against a saved map, without a simulator.

Zones for orthogonal doorway transit are generated from detected door features,
so the detector is load-bearing: if it misses a doorway or places it 20 cm off,
the robot is aimed at a wall.  This harness feeds `maps/*.pgm` through the exact
same `door_candidates()` the node uses, so the detector can be checked (and
regressions caught) in a second, with no Gazebo and no SLAM run.

    ./scripts/run_native.sh python3 scripts/check_doors.py
    ./scripts/run_native.sh python3 scripts/check_doors.py --ascii

Expectations come from the world (seminar_world.sdf): a 1.0 m opening at
(0, -3) through the wall running along X, and one at (3, 0) through the wall
running along Y.  SLAM thickens walls, so the detected opening reads ~0.95 m;
that narrower number is the honest one to build zones from.
"""

import argparse
import os
import sys

import numpy as np
import yaml
from PIL import Image

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'pas_dual_arm_scripts'))

from pas_dual_arm_scripts.feature_registry import door_candidates  # noqa: E402

DEFAULT_MAP = os.path.join(REPO, 'src', 'pas_dual_arm_bringup', 'maps', 'seminar_map.yaml')

# (label, x, y, wall_axis, width) from seminar_world.sdf.
EXPECTED = [
    ('HOME<->BLUE', 0.0, -3.0, 'x', 1.0),
    ('HOME<->RED', 3.0, 0.0, 'y', 1.0),
]
CENTRE_TOL = 0.10
WIDTH_RANGE = (0.85, 1.05)


class FakeGrid:
    """The two attributes `door_candidates()` reads off a nav_msgs/OccupancyGrid.

    Building the real message would drag in rclpy and a running ROS graph for
    what is a pure array computation.
    """

    class _Origin:
        class _Position:
            def __init__(self, x, y):
                self.x, self.y = x, y

        def __init__(self, x, y):
            self.position = self._Position(x, y)

    class _Info:
        def __init__(self, width, height, resolution, ox, oy):
            self.width, self.height = width, height
            self.resolution = resolution
            self.origin = FakeGrid._Origin(ox, oy)

    def __init__(self, data, info):
        self.data, self.info = data, info


def load_map(yaml_path):
    """Convert a map_server .yaml/.pgm pair into occupancy values (-1/0/100)."""
    with open(yaml_path, 'r', encoding='utf-8') as handle:
        meta = yaml.safe_load(handle)
    pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])
    pixels = np.array(Image.open(pgm))
    if meta.get('negate', 0):
        pixels = 255 - pixels

    # map_server's trinary rule, applied here so the harness sees exactly what
    # the node sees at runtime.
    occ = (255.0 - pixels) / 255.0
    grid = np.full(pixels.shape, -1, dtype=np.int16)
    grid[occ > float(meta['occupied_thresh'])] = 100
    grid[occ < float(meta['free_thresh'])] = 0

    # The PGM's first row is the top of the map; occupancy grids index from the
    # bottom-left, so the rows flip.
    grid = np.flipud(grid)
    height, width = grid.shape
    info = FakeGrid._Info(width, height, float(meta['resolution']),
                          float(meta['origin'][0]), float(meta['origin'][1]))
    return FakeGrid(grid.reshape(-1).tolist(), info), meta


def ascii_view(grid_obj, cx, cy, half=1.2):
    """Print the occupancy around a point: '#' occupied, '.' free, '?' unknown."""
    info = grid_obj.info
    grid = np.asarray(grid_obj.data, dtype=np.int16).reshape(info.height, info.width)
    res = info.resolution

    def col(x):
        return int(round((x - info.origin.position.x) / res))

    def row(y):
        return int(round((y - info.origin.position.y) / res))

    rows = range(max(0, row(cy - half)), min(info.height, row(cy + half) + 1))
    cols = range(max(0, col(cx - half)), min(info.width, col(cx + half) + 1))
    symbol = {100: '#', 0: '.', -1: '?'}
    for r in reversed(rows):
        line = ''.join(symbol[int(grid[r, c])] for c in cols)
        print(f'  y={info.origin.position.y + r * res:6.2f}  {line}')
    print(f'  x from {info.origin.position.x + cols[0] * res:.2f} '
          f'to {info.origin.position.x + cols[-1] * res:.2f} step {res}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml', nargs='?', default=DEFAULT_MAP)
    parser.add_argument('--ascii', action='store_true',
                        help='print the occupancy around each expected doorway')
    args = parser.parse_args()

    grid_obj, meta = load_map(args.map_yaml)
    info = grid_obj.info
    print(f'map   : {args.map_yaml}')
    print(f'grid  : {info.width} x {info.height} px, {info.resolution} m/px, '
          f'origin ({info.origin.position.x}, {info.origin.position.y})')
    counts = np.bincount(np.asarray(grid_obj.data, dtype=np.int16) + 1,
                         minlength=102)
    print(f'cells : {counts[1]} free, {counts[101]} occupied, {counts[0]} unknown '
          f'(free_thresh {meta["free_thresh"]}, occupied_thresh {meta["occupied_thresh"]})')

    doors = door_candidates(grid_obj)
    print(f'\ndetected {len(doors)} door(s):')
    for door in sorted(doors, key=lambda d: (d['wall_axis'], d['x'], d['y'])):
        print(f'  ({door["x"]:+.3f}, {door["y"]:+.3f})  width {door["width"]:.3f} m  '
              f'wall along {door["wall_axis"].upper()}  '
              f'observations {door["observations"]}')

    print('\nagainst the world:')
    ok = True
    for label, ex, ey, axis, width in EXPECTED:
        match = next((d for d in doors
                      if d['wall_axis'] == axis
                      and abs(d['x'] - ex) <= CENTRE_TOL
                      and abs(d['y'] - ey) <= CENTRE_TOL), None)
        if match is None:
            near = [d for d in doors if d['wall_axis'] == axis]
            print(f'  FAIL {label}: expected ({ex:+.2f}, {ey:+.2f}), '
                  f'no candidate within {CENTRE_TOL} m'
                  + (f'; nearest on this wall: {[(round(d["x"], 2), round(d["y"], 2)) for d in near]}'
                     if near else '; nothing found on this wall'))
            ok = False
        elif not WIDTH_RANGE[0] <= match['width'] <= WIDTH_RANGE[1]:
            print(f'  FAIL {label}: width {match["width"]:.3f} m outside '
                  f'{WIDTH_RANGE[0]}-{WIDTH_RANGE[1]} m (true opening {width:.2f} m)')
            ok = False
        else:
            print(f'  OK   {label}: ({match["x"]:+.3f}, {match["y"]:+.3f}) '
                  f'off by ({match["x"] - ex:+.3f}, {match["y"] - ey:+.3f}) m, '
                  f'width {match["width"]:.3f} m vs {width:.2f} m true')

    extra = len(doors) - sum(1 for _ in EXPECTED)
    if extra > 0:
        print(f'  WARN {extra} extra candidate(s) - zones would be built on them too')

    if args.ascii:
        for label, ex, ey, _axis, _w in EXPECTED:
            print(f'\n{label} around ({ex:+.2f}, {ey:+.2f}):')
            ascii_view(grid_obj, ex, ey)

    print('\nPASS' if ok else '\nFAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
