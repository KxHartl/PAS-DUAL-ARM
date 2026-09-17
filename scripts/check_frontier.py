#!/usr/bin/env python3
"""Check the frontier explorer's map reasoning, without a simulator.

Exploration is hard to debug live: a wrong frontier looks exactly like a robot
that "just did not go there". This harness feeds the explorer the same kind of
occupancy grid it sees at runtime, but with a room deliberately erased back to
unknown, and asks the two questions that decide whether the run will work:

  1. does a frontier appear on the boundary of the erased room, and
  2. is the frontier that leads into it recognised as being **at a doorway**?

The second one is the whole point. A doorway frontier is small - it is the width
of the opening - so a plain size-based score never picks it, and the robot maps
one room forever. If this check says the door bonus is not being applied, the
run will stall at exactly that doorway.

    ./scripts/run_native.sh python3 scripts/check_frontier.py
    ./scripts/run_native.sh python3 scripts/check_frontier.py --room red --ascii

Exit code 0 means both questions are answered yes for every erased room.
"""
import argparse
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'pas_dual_arm_scripts'))
sys.path.insert(0, os.path.join(REPO, 'scripts'))

from check_doors import FakeGrid, load_map  # noqa: E402
from pas_dual_arm_scripts.feature_registry import door_candidates  # noqa: E402
from pas_dual_arm_scripts.frontier_explorer import (  # noqa: E402
    frontier_mask, gateway_from_cluster)

DEFAULT_MAP = os.path.join(REPO, 'src', 'pas_dual_arm_bringup', 'maps', 'seminar_map.yaml')

# What the robot has actually seen when it is still in the home room: the near
# face of the dividing wall, and nothing beyond it. So the erased region is the
# half-plane past the wall, with the wall itself left known - erasing the wall
# too would invent a frontier along its whole length and hide the real problem,
# which is that the only way through is the 1 m opening.
ROOMS = {
    'blue': {'beyond': ('y<', -3.06), 'door': (0.0, -3.0)},
    'red': {'beyond': ('x>', 3.06), 'door': (3.0, 0.0)},
}
ROBOT = (0.0, 0.0)          # home room, where the robot spawns

MIN_FRONTIER_CELLS = 12
MIN_DOOR_FRONTIER_CELLS = 3
GOAL_CLEARANCE = 0.55
DOOR_MATCH_RADIUS = 1.20


def erase(grid_obj, beyond):
    """Set everything past a line back to unknown, as if never observed."""
    axis, value = beyond
    info = grid_obj.info
    grid = np.asarray(grid_obj.data, dtype=np.int16).reshape(info.height, info.width).copy()
    res = info.resolution
    xs = info.origin.position.x + (np.arange(info.width) + 0.5) * res
    ys = info.origin.position.y + (np.arange(info.height) + 0.5) * res
    if axis == 'y<':
        grid[ys < value, :] = -1
    elif axis == 'y>':
        grid[ys > value, :] = -1
    elif axis == 'x<':
        grid[:, xs < value] = -1
    else:
        grid[:, xs > value] = -1
    return FakeGrid(grid.reshape(-1).tolist(), info)


def frontier_clusters(grid_obj):
    """Run the node's own frontier extraction over a saved map.

    The helpers are imported from the node module, not reimplemented here: a
    harness with its own copy of the rules is a harness that can pass while the
    robot stalls.
    """
    from scipy import ndimage
    info = grid_obj.info
    grid = np.asarray(grid_obj.data, dtype=np.int16).reshape(info.height, info.width)
    labels, count = ndimage.label(frontier_mask(grid), structure=np.ones((3, 3)))

    radius = max(1, int(round(GOAL_CLEARANCE / info.resolution)))
    out = []
    for label in range(1, count + 1):
        cells = np.argwhere(labels == label)
        gateway = gateway_from_cluster(grid, cells, info.resolution)
        if gateway is not None:
            out.append({
                'x': info.origin.position.x + (gateway['col'] + 0.5) * info.resolution,
                'y': info.origin.position.y + (gateway['row'] + 0.5) * info.resolution,
                'size': len(cells), 'gateway': gateway,
            })
            continue
        centre = cells.mean(axis=0)
        order = np.argsort(np.linalg.norm(cells - centre, axis=1))
        pick = None
        for index in order:
            row, col = int(cells[index][0]), int(cells[index][1])
            r0, r1 = max(0, row - radius), min(grid.shape[0], row + radius + 1)
            c0, c1 = max(0, col - radius), min(grid.shape[1], col + radius + 1)
            if not (grid[r0:r1, c0:c1] >= 65).any():
                pick = (row, col)
                break
        if pick is None:
            continue
        out.append({
            'x': info.origin.position.x + (pick[1] + 0.5) * info.resolution,
            'y': info.origin.position.y + (pick[0] + 0.5) * info.resolution,
            'size': len(cells), 'gateway': None,
        })
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml', nargs='?', default=DEFAULT_MAP)
    parser.add_argument('--room', choices=sorted(ROOMS) + ['all'], default='all')
    parser.add_argument('--ascii', action='store_true',
                        help='list every frontier cluster found')
    args = parser.parse_args()

    full, _ = load_map(args.map_yaml)
    rooms = sorted(ROOMS) if args.room == 'all' else [args.room]
    failures = 0

    for name in rooms:
        spec = ROOMS[name]
        print(f'\n=== everything past the {name.upper()} doorway erased ===')
        partial = erase(full, spec['beyond'])
        doors = door_candidates(partial)
        clusters = frontier_clusters(partial)
        print(f'  doorways still detectable: {len(doors)}')
        print(f'  frontier clusters:         {len(clusters)}')

        if args.ascii:
            for cluster in sorted(clusters, key=lambda c: -c['size'])[:10]:
                kind = 'opening' if cluster['gateway'] else 'open space'
                print(f'    ({cluster["x"]:6.2f}, {cluster["y"]:6.2f})  '
                      f'{cluster["size"]:5d} cells  {kind}')

        # 1. is there a frontier at all, near the erased room's doorway?
        dx, dy = spec['door']
        near_door = [c for c in clusters
                     if math.hypot(c['x'] - dx, c['y'] - dy) < DOOR_MATCH_RADIUS]
        if not near_door:
            print(f'  FAIL: no frontier within {DOOR_MATCH_RADIUS} m of the doorway '
                  f'at ({dx}, {dy}) - the room behind it would never be entered')
            failures += 1
            continue
        best = max(near_door, key=lambda c: c['size'])
        print(f'  frontier at the doorway:   ({best["x"]:.2f}, {best["y"]:.2f}), '
              f'{best["size"]} cells')

        # 2. would the plain size rule have picked it?
        biggest = max(clusters, key=lambda c: c['size'])
        matched = best.get('gateway') is not None
        if best['size'] < MIN_FRONTIER_CELLS:
            print(f'  NOTE: {best["size"]} cells is below the plain floor of '
                  f'{MIN_FRONTIER_CELLS}; it only survives because of the doorway rule '
                  f'(floor {MIN_DOOR_FRONTIER_CELLS})')
        if not matched:
            print('  FAIL: that frontier is NOT recognised as an opening, so it gets '
                  'no bonus, is held to the open-space clearance rule, and loses to '
                  'bigger frontiers')
            failures += 1
            continue
        gate = best['gateway']
        print(f'  recognised as an opening: width {gate["width"]:.2f} m, '
              f'travel direction {gate["normal"]} -> door bonus and square transit apply')
        if biggest is not best:
            print(f'  (the biggest frontier is elsewhere, {biggest["size"]} cells at '
                  f'({biggest["x"]:.2f}, {biggest["y"]:.2f}) - exactly the case the '
                  f'bonus exists for)')

    print()
    if failures:
        print(f'FAILED: {failures} room(s) would not be explored through their doorway.')
        return 1
    print('PASS: every erased room is reachable through a doorway frontier.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
