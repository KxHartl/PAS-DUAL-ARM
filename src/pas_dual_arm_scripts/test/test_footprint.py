"""The footprint may be approximate, but only in one direction: outwards."""

import math

import numpy as np

from pas_dual_arm_scripts.footprint_publisher import polygon_changed
from pas_dual_arm_scripts.robot_extent import ground_hull


def _inside(polygon, point):
    """Is a point inside a convex polygon? (consistent winding assumed)"""
    signs = []
    for index in range(len(polygon)):
        a, b = polygon[index], polygon[(index + 1) % len(polygon)]
        edge = (b[0] - a[0], b[1] - a[1])
        to_point = (point[0] - a[0], point[1] - a[1])
        signs.append(edge[0] * to_point[1] - edge[1] * to_point[0])
    return all(s >= -1e-9 for s in signs) or all(s <= 1e-9 for s in signs)


def test_hull_contains_every_point_it_was_built_from():
    rng = np.random.default_rng(0)
    points = rng.normal(size=(500, 2)) * [0.5, 0.4]
    hull = ground_hull(points, max_vertices=24)
    assert len(hull) >= 3
    for point in points:
        assert _inside(hull, point)


def test_vertex_cap_circumscribes_rather_than_simplifies():
    # A circle's exact hull has far more vertices than the cap allows, so the
    # reduction path runs. Simplifying inward here would quietly shrink the
    # robot and let it plan through a gap it does not fit.
    angles = np.linspace(0, 2 * math.pi, 400, endpoint=False)
    points = np.stack([0.6 * np.cos(angles), 0.45 * np.sin(angles)], axis=1)

    exact = ground_hull(points, max_vertices=1000)
    capped = ground_hull(points, max_vertices=12)
    assert len(exact) > 12
    assert len(capped) <= 12
    for point in points:
        assert _inside(capped, point)


def test_a_degenerate_or_tiny_input_is_returned_unchanged():
    points = np.array([[0.0, 0.0], [1.0, 0.0]])
    assert len(ground_hull(points)) == 2


def test_republish_only_when_the_outline_really_moved():
    square = np.array([[0.5, 0.4], [-0.5, 0.4], [-0.5, -0.4], [0.5, -0.4]])
    assert polygon_changed(None, square, 0.01)
    # Same shape, vertices listed from a different starting corner.
    rotated = np.roll(square, 1, axis=0)
    assert not polygon_changed(square, rotated, 0.01)
    # An arm swinging out by 10 cm is a change worth sending.
    wider = square + np.array([0.0, 0.10])
    assert polygon_changed(square, wider, 0.01)


def test_eight_vertices_keep_width_and_length_exact():
    # The vertex cap exists because ObstacleFootprint rasterises the outline once
    # per sampled trajectory, and DWB samples thousands per cycle. Eight is not
    # an arbitrary compromise: the directions include +-x and +-y, so the extent
    # that has to fit through a doorway is preserved to the millimetre and only
    # the corners are chamfered outward. Six drops +-90 degrees and the width
    # starts to grow, which would hand the doorway back the margin we just won.
    rng = np.random.default_rng(1)
    points = np.vstack([
        rng.uniform([-0.35, -0.42], [0.67, 0.42], size=(4000, 2)),
        rng.normal([0.55, 0.0], [0.05, 0.30], size=(1000, 2)),
    ])

    def extent(polygon):
        return (polygon[:, 0].max() - polygon[:, 0].min(),
                polygon[:, 1].max() - polygon[:, 1].min())

    exact_l, exact_w = extent(ground_hull(points, max_vertices=10_000))
    capped = ground_hull(points, max_vertices=8)
    length, width = extent(capped)
    assert len(capped) == 8
    assert abs(width - exact_w) < 1e-9
    assert abs(length - exact_l) < 1e-9
    for point in points:
        assert _inside(capped, point)
