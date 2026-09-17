#!/usr/bin/env python3
"""Check the room sweeper's route before a simulator is ever started.

A mapping run is expensive to watch and hard to read: a wrong route looks
exactly like a robot that "just drove oddly". This harness feeds the sweeper
the same occupancy grid it sees at runtime - once with the other rooms erased
back to unknown, as they are when the robot has only ever stood in the first
one - and asks the four questions that decide whether the run will look right:

  1. is every doorway found while it is still seen from one side only,
  2. does the route stay inside the room, instead of wandering through a door,
  3. is every move a pure translation along one map axis, and
  4. does the route reach everything still worth looking at - and drive nowhere
     when there is nothing left to look at.

Question 3 is the one that makes the run watchable. On a mecanum base a lane
and the step to the next lane are both single-axis translations, so a correct
route never needs the robot to turn - and turning in place is exactly where
this base slips and feeds yaw error into the scan matcher (P-10, P-11).

    ./scripts/run_native.sh python3 scripts/check_sweep.py
    ./scripts/run_native.sh python3 scripts/check_sweep.py --ascii
    ./scripts/run_native.sh python3 scripts/check_sweep.py ~/.ros/pas_dual_arm/live_map.yaml

Exit code 0 means every question is answered yes for every stage.
"""
import argparse
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'pas_dual_arm_scripts'))
sys.path.insert(0, os.path.join(REPO, 'scripts'))

from check_doors import load_map                                   # noqa: E402
from check_frontier import DEFAULT_MAP, ROOMS, erase               # noqa: E402
from pas_dual_arm_scripts.feature_registry import door_candidates  # noqa: E402
from pas_dual_arm_scripts.room_sweeper import (                    # noqa: E402
    interest_mask, room_region, seal_mask, seed_cell, sweep_waypoints,
    unknown_reachable)

# Must match the node's defaults; they are the numbers being checked.
LANE_SPACING = 1.5
CLEARANCE = 0.60
CLOSURE_CLEARANCE = 0.44
MIN_RUN = 0.60
ROBOT = (0.0, 0.0)
AXIS_TOLERANCE = 0.05       # m; a "pure" move may not drift more than this


def arrays(grid_obj):
    info = grid_obj.info
    grid = np.asarray(grid_obj.data, dtype=np.int16).reshape(info.height, info.width)
    return info, grid


def route(grid_obj, robot=ROBOT):
    info, grid = arrays(grid_obj)
    doors = door_candidates(grid_obj, both_sides=False)
    sealed = seal_mask(grid, info, doors)
    region = room_region(grid, info, seed_cell(info, *robot), sealed)
    interest = interest_mask(grid, region, sealed)
    points = sweep_waypoints(grid, info, region, robot, LANE_SPACING,
                             CLEARANCE, MIN_RUN, 0.0, interest, LANE_SPACING)
    return doors, region, interest, points


def axis_moves(points):
    """Every consecutive pair, as ('x'|'y'|'diagonal', length)."""
    out = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        if dy <= AXIS_TOLERANCE and dx > AXIS_TOLERANCE:
            out.append(('x', dx))
        elif dx <= AXIS_TOLERANCE and dy > AXIS_TOLERANCE:
            out.append(('y', dy))
        elif dx <= AXIS_TOLERANCE and dy <= AXIS_TOLERANCE:
            out.append(('none', 0.0))
        else:
            out.append(('diagonal', math.hypot(dx, dy)))
    return out


def coverage(info, target, points):
    """Furthest cell worth seeing that the route never comes near.

    Measured against what is still unknown, not against the whole room: a room
    the lidar has already seen through needs no lane driven in it at all, and
    demanding one is how a run spends a hundred seconds performing coverage.
    """
    if not target.any():
        return 0.0
    if not points:
        return math.inf
    rows, cols = np.nonzero(target)
    xs = info.origin.position.x + (cols + 0.5) * info.resolution
    ys = info.origin.position.y + (rows + 0.5) * info.resolution
    best = np.full(xs.shape, np.inf)
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        vx, vy = x1 - x0, y1 - y0
        length2 = vx * vx + vy * vy
        if length2 < 1e-9:
            continue
        t = np.clip(((xs - x0) * vx + (ys - y0) * vy) / length2, 0.0, 1.0)
        best = np.minimum(best, np.hypot(xs - (x0 + t * vx), ys - (y0 + t * vy)))
    return float(best.max())


def draw(info, region, points, doors):
    """Coarse picture of the room and the route, one character per 0.25 m."""
    step = max(1, int(round(0.25 / info.resolution)))
    rows, cols = np.nonzero(region)
    r0, r1, c0, c1 = rows.min(), rows.max(), cols.min(), cols.max()
    canvas = [[' ' if region[r, c] else '.' for c in range(c0, c1 + 1, step)]
              for r in range(r0, r1 + 1, step)]

    def put(x, y, ch):
        row, col = seed_cell(info, x, y)
        r, c = (row - r0) // step, (col - c0) // step
        if 0 <= r < len(canvas) and 0 <= c < len(canvas[0]):
            canvas[r][c] = ch
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        for t in np.linspace(0.0, 1.0, 120):
            put(x0 + t * (x1 - x0), y0 + t * (y1 - y0), '-')
    for index, (x, y) in enumerate(points):
        put(x, y, str(index % 10))
    for door in doors:
        put(door['x'], door['y'], 'D')
    put(*ROBOT, 'R')
    print('   +' + '-' * len(canvas[0]) + '+')
    for line in reversed(canvas):          # +y up, as in the map frame
        print('   |' + ''.join(line) + '|')
    print('   +' + '-' * len(canvas[0]) + '+   R robot  D prolaz  0-9 točke')


def with_shadow(grid_obj, centre=(-1.6, 1.6), half=0.55):
    """Put an unseen pocket back into a finished room.

    The shipped map has nothing left unknown inside a room, so a route planned
    on it is correctly empty - which means it exercises no lane generation at
    all. A patch of unknown where a table's shadow would fall gives the checker
    something to plan around, and asks the question that matters: does a lane
    appear, and does it get near the pocket?
    """
    info = grid_obj.info
    grid = np.asarray(grid_obj.data, dtype=np.int16).reshape(
        info.height, info.width).copy()
    row, col = seed_cell(info, *centre)
    span = int(round(half / info.resolution))
    grid[row - span:row + span, col - span:col + span] = -1
    return type(grid_obj)(grid.reshape(-1).tolist(), info)


def stage(name, grid_obj, expect_doors, ascii_view):
    info, grid = arrays(grid_obj)
    doors, region, interest, points = route(grid_obj)
    area = float(region.sum()) * info.resolution ** 2
    unknown = unknown_reachable(grid, info, seed_cell(info, *ROBOT), CLOSURE_CLEARANCE)
    moves = axis_moves(points)
    span = coverage(info, interest, points)
    bad = [m for m in moves if m[0] == 'diagonal']

    print(f'=== {name}')
    print(f'    prolazi        : {len(doors)} '
          + ', '.join(f'({d["x"]:.2f}, {d["y"]:.2f}) {d["width"]:.2f} m' for d in doors))
    print(f'    soba           : {area:.1f} m², {len(points)} točaka')
    print(f'    nepoznato      : {unknown} ćelija uz dohvat')
    print(f'    za vidjeti     : {int(interest.sum())} nepoznatih ćelija u ovoj sobi')
    print(f'    pokrivenost    : najdalja takva ćelija {span:.2f} m od rute '
          f'(granica {LANE_SPACING + CLEARANCE:.2f} m)')
    print(f'    gibanja        : {sum(1 for m in moves if m[0] == "x")}× po x, '
          f'{sum(1 for m in moves if m[0] == "y")}× po y, {len(bad)}× dijagonalno')
    if ascii_view:
        draw(info, region, points, doors)

    ok = True
    if len(doors) < expect_doors:
        print(f'    FAIL: očekivano {expect_doors} prolaza, nađeno {len(doors)}')
        ok = False
    if bad:
        print(f'    FAIL: {len(bad)} gibanja nisu po osi — robot bi se morao okretati')
        ok = False
    if not points and interest.any():
        print('    FAIL: ima nepoznatog u sobi, a ruta je prazna')
        ok = False
    if points and not interest.any():
        print('    FAIL: nema se što vidjeti, a ruta ipak vozi')
        ok = False
    if span > LANE_SPACING + CLEARANCE + 1e-6:
        print(f'    FAIL: nepoznato je {span:.2f} m od rute — ostaje nepregledano')
        ok = False
    return ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('map', nargs='?', default=DEFAULT_MAP)
    parser.add_argument('--ascii', action='store_true', help='draw the route')
    args = parser.parse_args()

    full, _meta = load_map(args.map)
    stages = [('gotova karta (sve viđeno) — ruta mora biti prazna', full, 2)]
    if args.map == DEFAULT_MAP:
        # Only the shipped map has known room geometry to erase.
        partial = erase(erase(full, ROOMS['blue']['beyond']), ROOMS['red']['beyond'])
        stages.insert(0, ('polazna soba, ostalo nepoznato', partial, 2))
        stages.insert(0, ('soba sa sjenom iza prepreke — mora se voziti',
                          with_shadow(partial), 2))

    ok = all(stage(name, grid, expect, args.ascii) for name, grid, expect in stages)
    print()
    print('PASS: ruta pokriva sobu, ostaje u njoj i vozi se bez okretanja.' if ok
          else 'FAIL: vidi poruke iznad.')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
