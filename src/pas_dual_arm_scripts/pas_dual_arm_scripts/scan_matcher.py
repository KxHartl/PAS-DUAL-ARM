"""Two consecutive laser scans in, the movement between them out.

Why this exists at all: the wheels cannot measure how this base moves sideways.
Measured against ground truth over 13 runs, wheel odometry is out by 0.8 mm
forwards and 19.5 mm sideways, because a mecanum base with mu2 = 0.20 SLIDES
sideways by design and a slide leaves the wheels no rotation to count. The
manufacturer reached the same conclusion: PAL ships this base with
`enable_odom_tf: false` and the comment "odom tf will be published by direct
laser odometry" (D-25).

The laser has no such blind spot. It sees the walls, and walls do not move.

Kept as a plain module, not a node, so the offline evaluation against the fifty
runs already recorded and the eventual ROS node run the SAME code. A matcher
that is validated offline and then reimplemented online has not been validated.
"""
import math

import numpy as np
from scipy.spatial import cKDTree


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Valid returns as an (N, 2) array in the laser's own frame."""
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + np.arange(ranges.size) * angle_increment
    good = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r, a = ranges[good], angles[good]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def surface_normals(points, tree, neighbours=6):
    """A unit normal per point, from the local line its neighbours lie on.

    Point-to-point ICP has a known weakness in exactly the place this project
    cares about: along a flat wall, every point matches a neighbour just as well
    as the right one, so the fit is free to slide the cloud sideways. A corridor
    or a doorway is mostly flat wall, and sliding sideways is the error being
    hunted. Scoring the distance along the surface NORMAL instead removes that
    freedom - across the wall the fit is pinned, along it nothing is claimed.
    """
    _, index = tree.query(points, k=min(neighbours, len(points)), workers=-1)
    normals = np.empty_like(points)
    for i, neighbourhood in enumerate(np.atleast_2d(index)):
        local = points[neighbourhood]
        local = local - local.mean(axis=0)
        _, _, vt = np.linalg.svd(local, full_matrices=False)
        normals[i] = (-vt[-1][1], vt[-1][0]) if False else vt[-1]
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.where(lengths > 1e-9, lengths, 1.0)


def _transform(points, dx, dy, dtheta):
    c, s = math.cos(dtheta), math.sin(dtheta)
    return np.column_stack((
        points[:, 0] * c - points[:, 1] * s + dx,
        points[:, 0] * s + points[:, 1] * c + dy))


def match(previous, current, guess=(0.0, 0.0, 0.0),
          iterations=30, tolerance=1e-5, trim=0.85, max_pair=0.5,
          point_to_line=True):
    """Point-to-point ICP: the (dx, dy, dtheta) that carries `current` onto `previous`.

    `guess` seeds the search - the wheels are a poor measure of sideways motion
    but a good one forwards, so they are worth starting from rather than
    ignoring.

    `trim` keeps only the closest fraction of pairs each iteration. Without it a
    door frame entering or leaving the scan drags the fit: those points have no
    counterpart in the other scan, and a least-squares fit will happily move the
    whole cloud to accommodate them.

    Returns (dx, dy, dtheta, fitness) where fitness is the RMS of the pairs that
    were kept, in metres; None if there is not enough to match on.
    """
    if previous is None or current is None:
        return None
    if len(previous) < 30 or len(current) < 30:
        return None

    tree = cKDTree(previous)
    normals = surface_normals(previous, tree) if point_to_line else None
    dx, dy, dtheta = guess
    fitness = float('inf')

    for _ in range(iterations):
        moved = _transform(current, dx, dy, dtheta)
        distance, index = tree.query(moved, workers=-1)

        keep = distance < max_pair
        if keep.sum() < 30:
            return None
        if 0.0 < trim < 1.0:
            cutoff = np.quantile(distance[keep], trim)
            keep &= distance <= cutoff
            if keep.sum() < 30:
                return None

        src, dst = moved[keep], previous[index[keep]]
        if normals is None:
            src_c, dst_c = src.mean(axis=0), dst.mean(axis=0)
            cov = (src - src_c).T @ (dst - dst_c)
            u, _, vt = np.linalg.svd(cov)
            rot = vt.T @ u.T
            if np.linalg.det(rot) < 0:        # never accept a reflection
                vt[-1] *= -1
                rot = vt.T @ u.T
            step_theta = math.atan2(rot[1, 0], rot[0, 0])
            step = dst_c - rot @ src_c
        else:
            # Point-to-line, linearised in the small rotation: for each pair,
            #     n . (R(w) s + t - d) = 0,  R(w) s ~ s + w * perp(s)
            # which is one linear equation per pair in (tx, ty, w).
            n = normals[index[keep]]
            perp = np.column_stack((-src[:, 1], src[:, 0]))
            a = np.column_stack((n[:, 0], n[:, 1], (n * perp).sum(axis=1)))
            b = (n * (dst - src)).sum(axis=1)
            solution, *_ = np.linalg.lstsq(a, b, rcond=None)
            step = solution[:2]
            step_theta = float(np.clip(solution[2], -0.2, 0.2))
            rot = np.array([[math.cos(step_theta), -math.sin(step_theta)],
                            [math.sin(step_theta), math.cos(step_theta)]])
            step = step + src.mean(axis=0) - rot @ src.mean(axis=0)

        c, s = math.cos(step_theta), math.sin(step_theta)
        dx, dy = (c * dx - s * dy + step[0], s * dx + c * dy + step[1])
        dtheta = math.atan2(math.sin(dtheta + step_theta),
                            math.cos(dtheta + step_theta))

        new_fitness = float(np.sqrt((distance[keep] ** 2).mean()))
        if abs(fitness - new_fitness) < tolerance:
            fitness = new_fitness
            break
        fitness = new_fitness

    return dx, dy, dtheta, fitness
