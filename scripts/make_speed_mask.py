#!/usr/bin/env python3
"""Draw a speed limit onto the map, so the robot slows where the map is tight.

Speed is set per LEG today: room_navigator writes a limit into the controller
when a leg starts and another when the next one does. That ties the speed to the
route's structure rather than to the building, so the robot brakes at every leg
boundary whether or not anything there is narrow - measured, it peaks at
0.217 m/s and spends its driving time at a median of 0.081.

Nav2 already has the right mechanism. A SpeedFilter reads a mask laid over the
map and limits the speed by WHERE THE ROBOT IS, continuously, so a doorway slows
it because it is a doorway and an open room does not. The route can then be one
uninterrupted goal without losing the caution.

The mask is generated, not drawn: the doorways come from the same detector the
navigation graph uses (feature_registry.door_candidates), and the tables from
the world file, so a new map produces a new mask with nothing to keep in step by
hand.

    ./scripts/run_native.sh python3 scripts/make_speed_mask.py \
        --map src/pas_dual_arm_bringup/maps/seminar_map.yaml \
        --world src/pas_dual_arm_bringup/worlds/seminar_world.sdf
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'pas_dual_arm_scripts'))
from pas_dual_arm_scripts.feature_registry import door_candidates  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'validation'))
from analyze_runs import read_world  # noqa: E402


class Point:
    def __init__(self, x, y):
        self.x, self.y = x, y


class Pose:
    def __init__(self, x, y):
        self.position = Point(x, y)


class Info:
    """The bits of nav_msgs/MapMetaData the detector actually reads."""
    def __init__(self, resolution, width, height, origin):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = Pose(origin[0], origin[1])


class Grid:
    def __init__(self, data, info):
        self.data = data
        self.info = info


def read_pgm(path):
    """A binary PGM, as (array, maxval). Plain enough not to need an image library."""
    with open(path, 'rb') as handle:
        magic = handle.readline().strip()
        if magic != b'P5':
            raise ValueError(f'{path}: expected a binary PGM, found {magic!r}')
        fields = []
        while len(fields) < 3:
            line = handle.readline()
            if line.startswith(b'#'):
                continue
            fields += line.split()
        width, height, maxval = (int(f) for f in fields[:3])
        pixels = np.frombuffer(handle.read(width * height), dtype=np.uint8)
    return pixels.reshape(height, width), maxval


def write_pgm(path, array):
    with open(path, 'wb') as handle:
        handle.write(b'P5\n')
        handle.write(f'{array.shape[1]} {array.shape[0]}\n255\n'.encode())
        handle.write(array.astype(np.uint8).tobytes())


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default=os.path.join(
        here, '..', 'src', 'pas_dual_arm_bringup', 'maps', 'seminar_map.yaml'))
    parser.add_argument('--world', default=os.path.join(
        here, '..', 'src', 'pas_dual_arm_bringup', 'worlds', 'seminar_world.sdf'))
    parser.add_argument('--out', default=None,
                        help='where to write mask.pgm/.yaml (default: beside the map)')
    # Percentages of the controller's maximum. The doorway number is the one
    # the mission already drives openings at (slow_speed_xy 0.22 against
    # fast 0.50), so this changes WHERE it applies, not how fast it goes.
    parser.add_argument('--door-percent', type=float, default=45.0)
    parser.add_argument('--table-percent', type=float, default=55.0)
    # How far the limit reaches around each feature. A doorway needs the robot
    # already slow when it arrives, not slowing as it enters.
    parser.add_argument('--door-radius', type=float, default=1.30)
    parser.add_argument('--table-radius', type=float, default=1.00)
    args = parser.parse_args()

    meta = yaml.safe_load(open(args.map))
    image = os.path.join(os.path.dirname(os.path.abspath(args.map)), meta['image'])
    grid, maxval = read_pgm(image)
    resolution = float(meta['resolution'])
    origin = [float(v) for v in meta['origin']]
    height, width = grid.shape

    # The detector wants an OccupancyGrid: 0 free, 100 occupied, -1 unknown.
    # A map PGM stores free as white and occupied as black, and row 0 is the TOP
    # of the image while row 0 of an OccupancyGrid is the BOTTOM.
    scale = grid.astype(np.float32) / maxval
    occupancy = np.full(grid.shape, -1, dtype=np.int16)
    occupancy[scale > (1.0 - float(meta['free_thresh']))] = 0
    occupancy[scale < (1.0 - float(meta['occupied_thresh']))] = 100
    occupancy = np.flipud(occupancy)

    doors = door_candidates(Grid(occupancy.reshape(-1).tolist(),
                                 Info(resolution, width, height, origin)))
    tables = read_world(args.world)[0]
    print(f'{len(doors)} vrata, {len(tables)} stolova')

    # Mask values are percentages of the maximum speed; 0 means no limit.
    mask = np.zeros(grid.shape, dtype=np.uint8)
    ys, xs = np.mgrid[0:height, 0:width]
    world_x = origin[0] + (xs + 0.5) * resolution
    world_y = origin[1] + (height - 1 - ys + 0.5) * resolution

    def paint(cx, cy, radius, percent):
        inside = (world_x - cx) ** 2 + (world_y - cy) ** 2 <= radius ** 2
        # The tighter limit wins where two features overlap.
        current = mask[inside]
        mask[inside] = np.where(current == 0, percent, np.minimum(current, percent))

    for door in doors:
        paint(door['x'], door['y'], args.door_radius, int(args.door_percent))
        print(f"  vrata na ({door['x']:+.2f}, {door['y']:+.2f}) sirine "
              f"{door['width']:.2f} m -> {args.door_percent:.0f} %")
    for table in tables:
        paint(table.cx, table.cy, args.table_radius + max(table.sx, table.sy) / 2.0,
              int(args.table_percent))
        print(f'  stol "{table.name}" na ({table.cx:+.2f}, {table.cy:+.2f}), '
              f'{table.sx:.2f} x {table.sy:.2f} m -> {args.table_percent:.0f} %')

    out = args.out or os.path.dirname(os.path.abspath(args.map))
    os.makedirs(out, exist_ok=True)
    write_pgm(os.path.join(out, 'speed_mask.pgm'), mask)
    with open(os.path.join(out, 'speed_mask.yaml'), 'w') as handle:
        yaml.safe_dump({
            'image': 'speed_mask.pgm',
            'mode': 'scale',          # the pixel value IS the percentage
            'resolution': resolution,
            'origin': origin,
            'negate': 0,
            'occupied_thresh': 1.0,
            'free_thresh': 0.0,
        }, handle, default_flow_style=False)
    limited = int((mask > 0).sum())
    print(f'\nmaska: {limited} od {mask.size} celija ogranicenih '
          f'({100.0 * limited / mask.size:.1f} %), zapisano u {out}')


if __name__ == '__main__':
    main()
