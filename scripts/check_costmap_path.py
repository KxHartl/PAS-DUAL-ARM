#!/usr/bin/env python3
"""Predict what the global planner will do, before anything drives.

Every costmap change so far has been judged by driving it, and twice the answer
was that the robot could no longer get somewhere (P-39).  The global planner is
cheap to imitate offline, though: its input is the saved map plus the keepout
mask plus an inflation layer, all of which exist without a simulator, and NavFn
minimises the same cost any Dijkstra does.

So this builds that cost field and plans between the poses the navigator stops
at, and reports the two numbers a costmap change is really judged on:

  reachable    does a route still exist - the failure mode of tightening
  clearance    how close the route comes to an obstacle along the way, which is
               what decides whether the route can be driven. The collision
               monitor projects the real 1.04 x 0.854 m footprint 1.5 s ahead in
               a straight line, so a route that hugs an obstacle through a bend
               gets damped to a stop even though every pose on it is legal
               (run 64: FootprintApproach engaged 20 times, DWB reported no
               invalid trajectories at all, and the leg timed out).

    ./scripts/run_native.sh python3 scripts/check_costmap_path.py
    ./scripts/run_native.sh python3 scripts/check_costmap_path.py --inflation 0.85 --scaling 2.0
"""

import argparse
import heapq
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'pas_dual_arm_scripts'))
sys.path.insert(0, os.path.join(REPO, 'scripts'))

from check_doors import DEFAULT_MAP, load_map  # noqa: E402
from pas_dual_arm_scripts.nav_zones import Zones, default_params  # noqa: E402

# nav2_costmap_2d constants, and the footprint they are derived from.
LETHAL = 254
INSCRIBED = 253
HALF_LENGTH, HALF_WIDTH = 0.52, 0.427
INSCRIBED_RADIUS = HALF_WIDTH
# collision_monitor FootprintApproach.time_before_collision x DWB max_vel_x:
# how far ahead the real footprint is swept before the velocity is damped.
LOOK_AHEAD = 1.5 * 0.30
# Above this the field is overdoing it: routes stop preferring clearance and start
# hugging the opposite wall instead (run 68, "otisao je u drugi ekstrem").
ROOM_CLEARANCE_MAX = 0.90
# NavfnPlanner's own translation of costmap cost into search cost.
COST_NEUTRAL, COST_FACTOR = 50.0, 0.8


def inflate(lethal, resolution, inflation_radius, scaling):
    """nav2's InflationLayer, as a distance transform.

    cost = (INSCRIBED - 1) * exp(-scaling * (distance - inscribed_radius))
    inside the inflation radius, INSCRIBED within the inscribed radius, and
    nothing beyond - which is the cliff that makes a path sit exactly there.
    """
    import cv2
    distance = cv2.distanceTransform((~lethal).astype(np.uint8), cv2.DIST_L2, 5) * resolution
    cost = np.zeros(lethal.shape, dtype=np.float64)
    inflated = (distance > INSCRIBED_RADIUS) & (distance <= inflation_radius)
    cost[inflated] = (INSCRIBED - 1) * np.exp(
        -scaling * (distance[inflated] - INSCRIBED_RADIUS))
    cost[distance <= INSCRIBED_RADIUS] = INSCRIBED
    cost[lethal] = LETHAL
    return cost, distance


def plan(cost, start, goal):
    """Dijkstra over NavFn's own cost translation. Returns the path, or None."""
    height, width = cost.shape
    blocked = cost >= INSCRIBED
    if blocked[start] or blocked[goal]:
        return None
    step = np.where(blocked, np.inf, COST_NEUTRAL + COST_FACTOR * cost)

    dist = np.full(cost.shape, np.inf)
    dist[start] = 0.0
    came = {}
    queue = [(0.0, start)]
    neighbours = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                  (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                  (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    while queue:
        d, node = heapq.heappop(queue)
        if node == goal:
            break
        if d > dist[node]:
            continue
        row, col = node
        for dr, dc, weight in neighbours:
            nr, nc = row + dr, col + dc
            if not (0 <= nr < height and 0 <= nc < width) or blocked[nr, nc]:
                continue
            nd = d + weight * step[nr, nc]
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                came[(nr, nc)] = node
                heapq.heappush(queue, (nd, (nr, nc)))
    if not np.isfinite(dist[goal]):
        return None
    path, node = [goal], goal
    while node != start:
        node = came[node]
        path.append(node)
    return path[::-1]


def _configured_inflation():
    """The values nav2 will actually run with, so a bare run checks reality."""
    import yaml
    path = os.path.join(REPO, 'src', 'pas_dual_arm_bringup', 'config', 'nav2_params.yaml')
    layer = (yaml.safe_load(open(path, encoding='utf-8'))
             ['global_costmap']['global_costmap']['ros__parameters']['inflation_layer'])
    if not layer.get('enabled', True):
        # Disabled: the repelling is the mask's job now (nav_zones.Zones.field).
        return 0.0, 0.0
    return float(layer['inflation_radius']), float(layer['cost_scaling_factor'])


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml', nargs='?', default=DEFAULT_MAP)
    configured = _configured_inflation()
    parser.add_argument('--inflation', type=float, default=configured[0],
                        help='global_costmap inflation_layer.inflation_radius '
                             f'(configured: {configured[0]})')
    parser.add_argument('--scaling', type=float, default=configured[1],
                        help='global_costmap inflation_layer.cost_scaling_factor '
                             f'(configured: {configured[1]})')
    parser.add_argument('--peak', type=float,
                        help='override nav_zones field_peak, to choose it by measurement')
    parser.add_argument('--width', type=float, help='override nav_zones field_width')
    args = parser.parse_args()

    grid_obj, _meta = load_map(args.map_yaml)
    info = grid_obj.info
    grid = np.asarray(grid_obj.data, dtype=np.int16).reshape(info.height, info.width)
    params = default_params()
    if args.peak is not None:
        params['field_peak'] = args.peak
    if args.width is not None:
        params['field_width'] = args.width
    zones = Zones(grid, info, params)

    # Layer them in the order the real costmap does, which is the whole point:
    # static_layer, then inflation_layer, and only then the keepout filter -
    # filters run after every plugin, so the zones are written in once inflation
    # has finished and are never themselves inflated. Inflating them here as well
    # would be modelling a costmap nav2 does not build: the zones already carry
    # the robot's inscribed radius (nav_zones.Zones.mask), so a second helping
    # closes the 0.41 m doorway lane outright, which is exactly what this script
    # reported the first time it was run.
    walls = grid == 100
    mask = zones.mask(grow=params['planner_inflation'])
    # Costmap2D converts an OccupancyGrid's 0..100 linearly onto 0..254, and
    # KeepoutFilter takes the maximum against what the layers already put there.
    # So the mask is not simply a wall any more: 100 is lethal, and the graded
    # table field below it is a cost, which is the entire point of it.
    mask_cost = np.round(np.asarray(mask, dtype=np.float64) * (LETHAL / 100.0))
    if args.inflation > 0.0:
        cost, _wall_distance = inflate(walls, info.resolution, args.inflation, args.scaling)
    else:
        cost = np.zeros(walls.shape, dtype=np.float64)
        cost[walls] = LETHAL
    cost = np.maximum(cost, mask_cost)
    zone = mask >= 100

    # Two clearances, because they mean different things and only one of them can
    # stop the robot. A route may lie against a guide wall all day: the zone is
    # virtual, the collision monitor has never heard of it, and the zone already
    # carries the robot's half width. What damps the robot to a standstill is a
    # route that bends close to something the laser can see, because the monitor
    # projects the real footprint straight ahead through the bend.
    import cv2
    distance = cv2.distanceTransform((~walls).astype(np.uint8),
                                     cv2.DIST_L2, 5) * info.resolution
    zone_distance = cv2.distanceTransform((~zone).astype(np.uint8),
                                          cv2.DIST_L2, 5) * info.resolution

    print(f'map       : {args.map_yaml}  ({info.resolution} m/px)')
    if args.inflation > 0.0:
        print(f'inflation : radius {args.inflation} m, scaling {args.scaling}, '
              f'inscribed {INSCRIBED_RADIUS} m')
    else:
        print('inflation : OFF - all repelling comes from the field in the mask')
    print(f'field     : peak {params["field_peak"]}, width {params["field_width"]} m, '
          f'core {params["planner_inflation"]} m')

    def cell(x, y):
        return (int(round((y - info.origin.position.y) / info.resolution)),
                int(round((x - info.origin.position.x) / info.resolution)))

    stops = []
    for index, door in enumerate(zones.doors):
        for room, portal in (door.get('portals') or {}).items():
            stops.append((f'door {index} {room} approach', *portal['approach'][:2]))
    for index, table in enumerate(zones.tables):
        stops.append((f'table {index} ({table["room"]}) approach', *table['approach'][:2]))
    for room in zones.rooms:
        if not room['tables']:
            stops.append((f'{room["name"]} centre', *room['centre'][:2]))
    # The far side of each table: the goal that jammed in run 64.
    for index, table in enumerate(zones.tables):
        fx, fy = table['face']
        stops.append((f'table {index} far side', table['x'] - fx * 1.4, table['y'] - fy * 1.4))

    print(f'\nplanning between {len(stops)} poses the navigator is sent to:')
    failures = 0
    worst = None
    for i in range(len(stops)):
        for j in range(i + 1, len(stops)):
            (a_name, ax, ay), (b_name, bx, by) = stops[i], stops[j]
            route = plan(cost, cell(ax, ay), cell(bx, by))
            if route is None:
                print(f'  UNREACHABLE  {a_name}  ->  {b_name}')
                failures += 1
                continue
            tightest = min(distance[r, c] for r, c in route)
            length = len(route) * info.resolution
            if worst is None or tightest < worst[0]:
                worst = (tightest, a_name, b_name, length,
                         min(zone_distance[r, c] for r, c in route))
    if worst:
        tightest, a_name, b_name, length, zone_gap = worst
        print(f'  tightest route: {a_name} -> {b_name}, {length:.1f} m, closest to a '
              f'real obstacle {tightest:.3f} m (to a zone edge {zone_gap:.3f} m)')

    # Away from the doorway, where the robot is not forced to squeeze, how much
    # room does a route actually keep? That is what the collision monitor's
    # straight-line projection needs through a bend.
    open_pairs = [(n, x, y) for n, x, y in stops if 'door' not in n]
    clearances = []
    for i in range(len(open_pairs)):
        for j in range(i + 1, len(open_pairs)):
            (ax, ay), (bx, by) = open_pairs[i][1:], open_pairs[j][1:]
            route = plan(cost, cell(ax, ay), cell(bx, by))
            if route is None:
                continue
            # Skip the doorway stretch: anything within a metre of a door centre.
            gaps = [distance[r, c] for r, c in route
                    if all(math.hypot(info.origin.position.x + c * info.resolution - d['x'],
                                      info.origin.position.y + r * info.resolution - d['y']) > 1.0
                           for d in zones.doors)]
            if gaps:
                clearances.append(min(gaps))
    if clearances:
        room = min(clearances)
        print(f'  away from the doorways, routes keep at least {room:.3f} m '
              f'from a REAL obstacle - the monitor projects {HALF_LENGTH:.2f} m of robot '
              f'plus {LOOK_AHEAD:.2f} m of look-ahead straight through every bend')
        # FootprintApproach scales the velocity, and the swept length is
        # HALF_LENGTH + v * time_before_collision. As v goes to zero that still
        # leaves HALF_LENGTH, so a route closer than that to something the laser
        # can see cannot be cleared at ANY speed: the robot is damped to a halt
        # and stays there. That is not the monitor being cautious, it is a
        # deadlock, and it is what run 64 spent 240 s doing.
        if room > ROOM_CLEARANCE_MAX:
            print(f'  WARN routes keep {room:.3f} m, more than {ROOM_CLEARANCE_MAX:.2f} m: '
                  f'the field is strong enough to push routes against whatever is on '
                  f'the far side, which is how run 68 went wrong')
        if room <= HALF_LENGTH:
            print(f'  FAIL routes come within {room:.3f} m, inside the robot\'s own '
                  f'{HALF_LENGTH:.2f} m half length: the collision monitor cannot clear '
                  f'that at any speed')
            failures += 1

    print('\nPASS' if not failures else f'\nFAIL: {failures} problem(s) above')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
