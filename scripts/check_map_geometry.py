#!/usr/bin/env python3
"""Measure a saved map's geometry where the robot is actually tight: at the doors.

`check_map.py` answers "did we map all three rooms" and says so - it checks span
and free area and explicitly leaves geometry to the eye.  But the run that fails
does not fail on coverage.  With a 0.854 m robot in a 1.00 m doorway the whole
budget is 7.3 cm per side, so a map that reads the opening 5 cm narrow, or puts
its centre 1.5 cm off, or has the wall on one side of the door stepped a
centimetre against the other, has already spent it - and none of that shows up in
a coverage number or, at this scale, in a picture.

Three measurements, each one a thing that moves the robot sideways:

  width/centre  every zone, portal pose and lane axis is built from the detected
                doorway (nav_zones -> feature_registry.door_candidates), so an
                error here is an error in where the robot aims.
  door spacing  the distance between the two detected doorways against the world's
                own 3.0 m / 3.0 m - map-scale distortion, visible nowhere else.
  wall fit      the two faces of the wall either side of each doorway, fitted
                separately: RMS says how crisp a face is, the tilt says whether it
                drifted, the step between the faces on either side of the opening is
                local distortion right at the jambs the robot must thread, and the
                thickness against the world's 0.10 m is the fattening that eats the
                opening in the first place.

    ./scripts/run_native.sh python3 scripts/check_map_geometry.py
    ./scripts/run_native.sh python3 scripts/check_map_geometry.py path/to/map.yaml

Exit code 0 only if every gate passes, so save_map.sh can refuse a map the way it
already refuses an incomplete one.
"""

import argparse
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'scripts'))
sys.path.insert(0, os.path.join(REPO, 'src', 'pas_dual_arm_scripts'))

from check_doors import DEFAULT_MAP, load_map  # noqa: E402
from pas_dual_arm_scripts.feature_registry import door_candidates  # noqa: E402

# From seminar_world.sdf: (label, x, y, wall_axis, opening).
EXPECTED = [
    ('HOME<->BLUE', 0.0, -3.0, 'x', 1.0),
    ('HOME<->RED', 3.0, 0.0, 'y', 1.0),
]

# Gates. The robot is 0.854 m wide in ARM_CARRY_V2 and the opening is 1.00 m, so
# every centimetre here is a centimetre taken off 7.3 cm of side clearance.
MIN_WIDTH = 0.97          # m, detected opening
MAX_CENTRE_ERROR = 0.01   # m, detected centre vs the world
MAX_SPACING_ERROR = 0.02  # m, distance between the two doorways vs the world
MAX_WALL_RMS = 0.01       # m, scatter of one wall face about its own line
MAX_WALL_TILT = 1.0       # deg, fitted face against the axis it is built on
MAX_JAMB_STEP = 0.02      # m, offset between the faces either side of one doorway
MAX_THICKNESS = 0.14      # m, mapped wall thickness; the world builds them 0.10 m

WALL_BAND = 0.20          # m, how far off the wall line to still count as wall
WALL_REACH = 2.5          # m, how far along the wall to fit
JAMB_SKIP = 0.6           # m, ignore this much either side of the centre (the opening)
TRUE_THICKNESS = 0.10     # m, from seminar_world.sdf


def occupied_points(grid_obj):
    """Every occupied cell as (x, y) in map coordinates."""
    info = grid_obj.info
    grid = np.asarray(grid_obj.data, dtype=np.int16).reshape(info.height, info.width)
    rows, cols = np.nonzero(grid == 100)
    xs = info.origin.position.x + cols * info.resolution
    ys = info.origin.position.y + rows * info.resolution
    return xs, ys


def wall_faces(along, cross, lo, hi, resolution):
    """Split a stretch of wall into its two faces and fit a line to each.

    A wall is 0.10 m thick, so at any usable resolution its occupied cells are two
    surfaces, not one line.  Fitting through both at once just measures the
    thickness (~50 mm RMS) and says nothing about whether either face is crisp -
    and it is a face, not a centreline, that the robot's corner touches.

    Bins the cells by position along the wall, takes the nearest and furthest cell
    in each bin as the two faces, then fits each.  Returns (near, far, thickness)
    where each face is (count, position at 0, tilt in deg, RMS), or None.
    """
    keep = (along >= lo) & (along <= hi)
    if keep.sum() < 20:
        return None
    a, c = along[keep], cross[keep]

    bins = np.round(a / resolution).astype(np.int64)
    order = np.argsort(bins)
    a, c, bins = a[order], c[order], bins[order]
    edges = np.flatnonzero(np.diff(bins)) + 1

    near_a, near_c, far_a, far_c, thickness = [], [], [], [], []
    for chunk_a, chunk_c in zip(np.split(a, edges), np.split(c, edges)):
        near_a.append(chunk_a[0])
        near_c.append(chunk_c.min())
        far_a.append(chunk_a[0])
        far_c.append(chunk_c.max())
        # One cell is occupied across its whole width, so the span between cell
        # centres is a resolution short of the material.
        thickness.append(chunk_c.max() - chunk_c.min() + resolution)

    def fit(fa, fc):
        if len(fa) < 10:
            return None
        slope, intercept = np.polyfit(np.asarray(fa), np.asarray(fc), 1)
        residual = np.asarray(fc) - (slope * np.asarray(fa) + intercept)
        return (len(fa), float(intercept), math.degrees(math.atan(slope)),
                float(np.sqrt(np.mean(residual ** 2))))

    return fit(near_a, near_c), fit(far_a, far_c), float(np.mean(thickness))


def check_wall(label, xs, ys, cx, cy, axis, resolution):
    """Fit both faces of the wall either side of one doorway.

    The doorway's own axis is the one the wall runs along; a face's `position` is
    then where that face sits at the doorway, which is what the jamb is made of.
    """
    if axis == 'x':                      # wall runs along X, sits at y = cy
        along, cross, centre, wall_at = xs, ys, cx, cy
    else:                                # wall runs along Y, sits at x = cx
        along, cross, centre, wall_at = ys, xs, cy, cx

    band = np.abs(cross - wall_at) <= WALL_BAND
    along, cross = along[band], cross[band]

    sides = {
        'lower': wall_faces(along, cross, centre - WALL_REACH, centre - JAMB_SKIP,
                            resolution),
        'upper': wall_faces(along, cross, centre + JAMB_SKIP, centre + WALL_REACH,
                            resolution),
    }

    ok = True
    positions = {'near': [], 'far': []}
    for side, result in sides.items():
        if result is None:
            print(f'    {side:5s}: too few occupied cells to fit a wall')
            ok = False
            continue
        near, far, thickness = result
        for name, face in (('near', near), ('far', far)):
            if face is None:
                print(f'    {side:5s} {name}: too few cells')
                ok = False
                continue
            count, position, tilt, rms = face
            positions[name].append(position)
            flags = []
            if rms > MAX_WALL_RMS:
                flags.append(f'RMS > {MAX_WALL_RMS * 1000:.0f} mm')
            if abs(tilt) > MAX_WALL_TILT:
                flags.append(f'tilt > {MAX_WALL_TILT} deg')
            ok = ok and not flags
            print(f'    {side:5s} {name:4s} face: {count:4d} bins, at {position:+.3f} m, '
                  f'tilt {tilt:+.2f} deg, RMS {rms * 1000:5.1f} mm'
                  + ('   <-- ' + ', '.join(flags) if flags else ''))
        fat = thickness > MAX_THICKNESS
        ok = ok and not fat
        print(f'    {side:5s} thickness: {thickness * 1000:5.1f} mm vs '
              f'{TRUE_THICKNESS * 1000:.0f} mm true '
              f'({(thickness - TRUE_THICKNESS) / 2 * 1000:+.1f} mm per face)'
              + (f'   <-- > {MAX_THICKNESS * 1000:.0f} mm' if fat else ''))

    for name, values in positions.items():
        if len(values) == 2:
            step = abs(values[0] - values[1])
            bad = step > MAX_JAMB_STEP
            ok = ok and not bad
            print(f'    step across the opening, {name} face: {step * 1000:5.1f} mm'
                  + (f'   <-- > {MAX_JAMB_STEP * 1000:.0f} mm' if bad else ''))
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml', nargs='?', default=DEFAULT_MAP)
    args = parser.parse_args()

    grid_obj, _meta = load_map(args.map_yaml)
    info = grid_obj.info
    print(f'map   : {args.map_yaml}')
    print(f'grid  : {info.width} x {info.height} px, {info.resolution} m/px, '
          f'origin ({info.origin.position.x}, {info.origin.position.y})')

    doors = door_candidates(grid_obj)
    xs, ys = occupied_points(grid_obj)
    ok = True
    matched = {}

    print('\ndoorways (the detector nav_zones builds every portal pose from):')
    for label, ex, ey, axis, opening in EXPECTED:
        door = next((d for d in doors
                     if d['wall_axis'] == axis
                     and math.hypot(d['x'] - ex, d['y'] - ey) <= 0.20), None)
        if door is None:
            print(f'  {label}: NOT FOUND near ({ex:+.2f}, {ey:+.2f})')
            ok = False
            continue
        matched[label] = door
        # Only the component along the wall moves the robot off the lane axis; the
        # one across it just says where the wall plane sits.
        if axis == 'x':
            lateral, normal = door['x'] - ex, door['y'] - ey
        else:
            lateral, normal = door['y'] - ey, door['x'] - ex
        flags = []
        if door['width'] < MIN_WIDTH:
            flags.append(f'width < {MIN_WIDTH} m')
        if abs(lateral) > MAX_CENTRE_ERROR:
            flags.append(f'axis off > {MAX_CENTRE_ERROR * 100:.0f} cm')
        ok = ok and not flags
        print(f'  {label}: ({door["x"]:+.3f}, {door["y"]:+.3f}), width '
              f'{door["width"]:.3f} m vs {opening:.2f} true, lane axis off '
              f'{lateral * 100:+.1f} cm (wall plane off {normal * 100:+.1f} cm)'
              + ('   <-- ' + ', '.join(flags) if flags else ''))
        # Half the opening, minus half the robot, is all the room it has.
        print(f'    leaves {(door["width"] - 0.854) / 2 * 100:+.1f} cm per side '
              f'for a 0.854 m robot (the world leaves +7.3)')
        print('    wall either side of this doorway:')
        ok = check_wall(label, xs, ys, door['x'], door['y'], axis,
                        info.resolution) and ok

    if len(matched) == 2:
        (la, a), (lb, b) = matched.items()
        measured = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
        truth = math.hypot(EXPECTED[0][1] - EXPECTED[1][1],
                           EXPECTED[0][2] - EXPECTED[1][2])
        error = abs(measured - truth)
        bad = error > MAX_SPACING_ERROR
        ok = ok and not bad
        print(f'\nmap scale: {la} to {lb} is {measured:.3f} m, world says '
              f'{truth:.3f} m, off by {error * 1000:.1f} mm'
              + (f'   <-- > {MAX_SPACING_ERROR * 1000:.0f} mm' if bad else ''))

    extra = len(doors) - len(matched)
    if extra > 0:
        print(f'\nWARN {extra} extra door candidate(s) - nav_zones would build zones '
              f'on them too')

    print('\nPASS' if ok else '\nFAIL: this map spends clearance the doorway does not have')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
