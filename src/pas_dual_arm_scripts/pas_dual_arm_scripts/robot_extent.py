"""The robot's real outline, from its own geometry and live TF.

Why this exists
---------------
Nav2 is given a static `footprint` polygon in YAML, measured once for one arm
posture (P-35: 1.04 x 0.854 m in ARM_CARRY_V2, agreed to 2 mm by a
collision-corridor search and by mesh vertices).  But the arms move, and
`ign_ros2_control` holds position only softly, so they sag: measured 1.109 m
wide while driving (P-37).  Every layer that decides about space - the global
planner, the controller's footprint critics, inflation - was reasoning about a
robot that had not existed since the arms last moved.

`Costmap2DROS` accepts a `geometry_msgs/Polygon` on its `footprint` topic and
swaps the footprint at runtime.  So the fix is not a new gate or a new
controller: it is telling Nav2 the truth about the robot's shape, continuously.
This module turns the URDF plus live TF into that polygon.

Each link's geometry is reduced to its convex hull once, at startup: the hull
has the same extent as the full vertex set in every direction, so nothing is
approximated away, but a mesh of thousands of vertices becomes tens of points
and the per-update work is one small matrix multiply per link.

`scripts/mesh_extent.py` imports the same loaders, so the number it prints
offline and the polygon Nav2 plans against cannot drift apart.
"""

import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_dae(path):
    """Vertex positions of a COLLADA file, in metres, Z-up."""
    root = ET.parse(path).getroot()
    ns = {'c': root.tag.split('}')[0].strip('{')} if '}' in root.tag else {}

    def find(tag):
        return root.iter(f'{{{ns["c"]}}}{tag}' if ns else tag)

    unit, up = 1.0, 'Y_UP'
    for a in find('unit'):
        unit = float(a.get('meter', 1.0))
    for a in find('up_axis'):
        up = (a.text or 'Y_UP').strip()

    # Follow <vertices><input semantic="POSITION" source="#ID"/> instead of
    # guessing by name: exporters emit opaque ids (ID4, ID6...), so a name
    # filter silently picks up the NORMALS array, whose +/-1 range reads as a
    # 2 m wide part (this is what made the D435 look 2 m wide).
    wanted = set()
    for verts in find('vertices'):
        for inp in verts:
            if inp.tag.endswith('input') and inp.get('semantic') == 'POSITION':
                wanted.add((inp.get('source') or '').lstrip('#'))
    points = []
    for src in find('source'):
        if src.get('id') not in wanted:
            continue
        array = None
        for child in src:
            if child.tag.endswith('float_array'):
                array = child
        if array is None:
            continue
        values = np.fromstring(array.text.replace('\n', ' '), sep=' ')
        if values.size % 3:
            continue
        points.append(values.reshape(-1, 3))
    if not points:
        return None
    vertices = np.vstack(points) * unit
    if up == 'Y_UP':
        vertices = vertices[:, [0, 2, 1]] * np.array([1, -1, 1])
    return vertices


def load_stl(path):
    data = open(path, 'rb').read()
    if data[:5].lower() == b'solid' and b'facet' in data[:2048]:
        vertices = [list(map(float, m)) for m in
                    re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data)]
        return np.array(vertices, dtype=float) if vertices else None
    count = int.from_bytes(data[80:84], 'little')
    out = np.zeros((count * 3, 3))
    for i in range(count):
        offset = 84 + i * 50 + 12
        triangle = np.frombuffer(data, dtype='<f4', count=9, offset=offset)
        out[i * 3:i * 3 + 3] = triangle.reshape(3, 3)
    return out


def resolve(uri):
    """file:// or package:// URI -> filesystem path."""
    if uri.startswith('file://'):
        return uri[len('file://'):]
    if uri.startswith('package://'):
        package, _, rest = uri[len('package://'):].partition('/')
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory(package), rest)
        except Exception:
            return uri
    return uri


_MESH_CACHE = {}


def mesh_points(uri, scale):
    path = resolve(uri)
    if path not in _MESH_CACHE:
        try:
            _MESH_CACHE[path] = (load_dae(path) if path.lower().endswith('.dae')
                                 else load_stl(path))
        except Exception:
            _MESH_CACHE[path] = None
    vertices = _MESH_CACHE[path]
    return None if vertices is None else vertices * np.array(scale)


def geom_points(geom):
    """Local-frame points approximating one <geometry> element."""
    kid = list(geom)[0]
    tag = kid.tag.split('}')[-1]
    if tag == 'mesh':
        scale = [float(s) for s in (kid.get('scale') or '1 1 1').split()]
        return mesh_points(kid.get('filename'), scale)
    if tag == 'box':
        sx, sy, sz = [float(s) for s in kid.get('size').split()]
        return np.array([[x, y, z] for x in (-sx / 2, sx / 2)
                         for y in (-sy / 2, sy / 2) for z in (-sz / 2, sz / 2)])
    if tag == 'cylinder':
        radius, length = float(kid.get('radius')), float(kid.get('length'))
        angles = np.linspace(0, 2 * np.pi, 24)
        return np.array([[radius * np.cos(t), radius * np.sin(t), z]
                         for t in angles for z in (-length / 2, length / 2)])
    if tag == 'sphere':
        radius = float(kid.get('radius'))
        return np.array([[x, y, z] for x in (-radius, radius)
                         for y in (-radius, radius) for z in (-radius, radius)])
    return None


def _hull(points):
    """Convex hull vertices, or the points themselves if a hull is degenerate."""
    if len(points) <= 8:
        return points
    try:
        from scipy.spatial import ConvexHull
        return points[ConvexHull(points).vertices]
    except Exception:
        return points     # coplanar or collinear geometry: keep it as it is


def link_points(urdf_xml, kind='collision'):
    """{link name: hull points in the link frame} for one geometry kind."""
    root = ET.fromstring(urdf_xml)
    per_link = {}
    for link in root.findall('link'):
        collected = []
        for element in link.findall(kind):
            geom = element.find('geometry')
            if geom is None:
                continue
            points = geom_points(geom)
            if points is None or len(points) == 0:
                continue
            origin = element.find('origin')
            if origin is not None:
                xyz = np.array([float(v) for v in
                                (origin.get('xyz') or '0 0 0').split()])
                rpy = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
                points = points @ rpy_matrix(*rpy).T + xyz
            collected.append(np.asarray(points, dtype=float))
        if collected:
            per_link[link.get('name')] = _hull(np.vstack(collected))
    return per_link


def ground_hull(points_xy, max_vertices=24):
    """Convex polygon in the ground plane containing every given point.

    The footprint is rasterised by every costmap that uses it, so an outline
    with hundreds of vertices is a real cost.  When the exact hull is larger
    than `max_vertices` it is replaced by a circumscribing polygon rather than a
    simplified one: bin the points by angle, take the furthest in each bin, and
    push that radius out by 1/cos(pi/N).  A regular N-gon's edge sits at
    R*cos(pi/N) from the centre, so scaling by its inverse guarantees the edge
    passes outside every point in its sector.  Approximating a footprint is only
    safe in one direction.
    """
    points_xy = np.asarray(points_xy, dtype=float)
    if len(points_xy) < 3:
        return points_xy
    try:
        from scipy.spatial import ConvexHull
        hull = points_xy[ConvexHull(points_xy).vertices]
    except Exception:
        return points_xy
    if len(hull) <= max_vertices:
        return hull

    # Build the reduced outline as the intersection of supporting half-planes.
    # For each of N directions take the furthest the robot reaches that way;
    # every point satisfies every one of those inequalities, so their
    # intersection contains the whole robot no matter how the radii vary. An
    # angular "take the furthest point per sector" reduction looks equivalent
    # and is not: where two neighbouring sectors reach very differently, the
    # edge between them cuts inside the shape.
    angles = np.linspace(0.0, 2.0 * math.pi, max_vertices, endpoint=False)
    normals = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    support = (points_xy @ normals.T).max(axis=0)
    polygon = []
    for index in range(max_vertices):
        following = (index + 1) % max_vertices
        (ax, ay), (bx, by) = normals[index], normals[following]
        determinant = ax * by - ay * bx
        ha, hb = support[index], support[following]
        polygon.append([(ha * by - hb * ay) / determinant,
                        (hb * ax - ha * bx) / determinant])
    return np.asarray(polygon)


class ExtentMeasurer:
    """Live geometry of the robot in a chosen frame, from live TF.

    `kind` defaults to the collision geometry, because that is what decides
    whether the robot fits: P-35 confirmed the visual and collision meshes of
    the arms are identical here, but that is a property of this model, not a
    rule.  `frame` defaults to `base_link` because that is the costmaps'
    `robot_base_frame`, and a footprint is expressed in it.
    """

    def __init__(self, urdf_xml, kind='collision', frame='base_link'):
        self.frame = frame
        self.kind = kind
        self.links = link_points(urdf_xml, kind)

    def points(self, tf_buffer, time_point):
        """Every link's geometry in `frame`, or None if TF can place none."""
        stacked = []
        for name, points in self.links.items():
            try:
                tf = tf_buffer.lookup_transform(self.frame, name, time_point).transform
            except Exception:
                continue
            rotation = quat_matrix([tf.rotation.x, tf.rotation.y,
                                    tf.rotation.z, tf.rotation.w])
            translation = np.array([tf.translation.x, tf.translation.y,
                                    tf.translation.z])
            stacked.append(points @ rotation.T + translation)
        if not stacked:
            return None
        return np.vstack(stacked)

    def footprint(self, tf_buffer, time_point, max_vertices=24):
        """The robot's outline right now, as a convex polygon, or None.

        Everything is projected straight down.  That is deliberate: a costmap
        footprint is a 2D shadow, and an arm held out at table height collides
        with a table whatever its own height happens to be.
        """
        points = self.points(tf_buffer, time_point)
        if points is None:
            return None
        return ground_hull(points[:, :2], max_vertices)
