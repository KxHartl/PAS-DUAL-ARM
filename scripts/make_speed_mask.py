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
    # Percentages of the controller's maximum, and free space is not limited
    # at all. The first attempt used 45 and 55 over generous radii, and measured
    # what that costs: the robot drove at 0.142 m/s in the open against 0.081
    # before, so removing the per-leg braking worked - but it spent three times
    # as many samples inside the restricted zones as outside them, and the run
    # came out 626 s against 579. The mask is not the only thing slowing the
    # robot down; DWB's own critics already back off near obstacles, and these
    # numbers were set as though it were.
    parser.add_argument('--door-percent', type=float, default=80.0)
    parser.add_argument('--table-percent', type=float, default=80.0)
    parser.add_argument('--table-close-percent', type=float, default=50.0)
    # How far each limit reaches, and it should be little. Measured with the
    # radii at 0.90/1.20/0.55: the robot spent 35.7 % of its driving time inside
    # the table rings, at a median of 0.038 m/s, and the mask was not what held
    # it there - the 95th percentile inside those rings was well under what the
    # mask allows. A ring that reaches past the feature only lets the docking
    # manoeuvre look like a speed limit.
    parser.add_argument('--door-radius', type=float, default=0.70)
    parser.add_argument('--table-radius', type=float, default=0.80)
    parser.add_argument('--table-close-radius', type=float, default=0.40)
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
        half = max(table.sx, table.sy) / 2.0
        # Two rings: near the table, and right up against it where the robot is
        # docking and a bump costs the run.
        paint(table.cx, table.cy, args.table_radius + half, int(args.table_percent))
        paint(table.cx, table.cy, args.table_close_radius + half,
              int(args.table_close_percent))
        print(f'  stol "{table.name}" na ({table.cx:+.2f}, {table.cy:+.2f}), '
              f'{table.sx:.2f} x {table.sy:.2f} m -> {args.table_percent:.0f} % '
              f'do {args.table_radius + half:.2f} m, {args.table_close_percent:.0f} % '
              f'do {args.table_close_radius + half:.2f} m')

    out = args.out or os.path.dirname(os.path.abspath(args.map))
    os.makedirs(out, exist_ok=True)
    write_pgm(os.path.join(out, 'speed_mask.pgm'), mask)
    with open(os.path.join(out, 'speed_mask.yaml'), 'w') as handle:
        yaml.safe_dump({
            'image': 'speed_mask.pgm',
            # RAW, not scale: in scale mode the map server rescales the pixel,
            # so a 45 would arrive as 82. In raw mode the value passes through
            # untouched and the pixel IS the percentage. Nothing in the mask is
            # 255, which raw reads as unknown.
            'mode': 'raw',
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
