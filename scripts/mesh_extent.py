#!/usr/bin/env python3
"""True lateral extent of the robot from mesh vertices, visual vs collision.

RViz draws the VISUAL meshes while MoveIt collides the COLLISION meshes. If the
two differ, what you see is not what the planner checks. This transforms every
mesh vertex by its link's live TF and reports the real width of both.

    ./scripts/run_native.sh python3 scripts/mesh_extent.py
"""
import argparse
import os
import re
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, 'src', 'pas_dual_arm_bringup', 'urdf', 'robot.urdf.xacro')
_cache = {}


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
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
    pts = []
    for src in find('source'):
        if src.get('id') not in wanted:
            continue
        fa = None
        for child in src:
            if child.tag.endswith('float_array'):
                fa = child
        if fa is None:
            continue
        vals = np.fromstring(fa.text.replace('\n', ' '), sep=' ')
        if vals.size % 3:
            continue
        pts.append(vals.reshape(-1, 3))
    if not pts:
        return None
    v = np.vstack(pts) * unit
    if up == 'Y_UP':
        v = v[:, [0, 2, 1]] * np.array([1, -1, 1])
    return v


def load_stl(path):
    data = open(path, 'rb').read()
    if data[:5].lower() == b'solid' and b'facet' in data[:2048]:
        vals = np.array([float(x) for x in re.findall(
            rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data).__iter__().__next__()]) \
            if False else None
        verts = [list(map(float, m)) for m in
                 re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data)]
        return np.array(verts, dtype=float) if verts else None
    n = int.from_bytes(data[80:84], 'little')
    out = np.zeros((n * 3, 3))
    for i in range(n):
        off = 84 + i * 50 + 12
        tri = np.frombuffer(data, dtype='<f4', count=9, offset=off)
        out[i * 3:i * 3 + 3] = tri.reshape(3, 3)
    return out


def resolve(uri):
    """file:// or package:// URI -> filesystem path."""
    if uri.startswith('file://'):
        return uri[len('file://'):]
    if uri.startswith('package://'):
        pkg, _, rest = uri[len('package://'):].partition('/')
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory(pkg), rest)
        except Exception:
            return uri
    return uri


def mesh_points(uri, scale):
    path = resolve(uri)
    if path not in _cache:
        try:
            _cache[path] = (load_dae(path) if path.lower().endswith('.dae')
                            else load_stl(path))
        except Exception as e:
            print(f'  ! ne mogu procitati {os.path.basename(path)}: {e}')
            _cache[path] = None
    v = _cache[path]
    return None if v is None else v * np.array(scale)


def geom_points(geom):
    """Local-frame points approximating one <geometry> element."""
    kid = list(geom)[0]
    tag = kid.tag.split('}')[-1]
    if tag == 'mesh':
        scale = [float(s) for s in (kid.get('scale') or '1 1 1').split()]
        return mesh_points(kid.get('filename'), scale)
    if tag == 'box':
        sx, sy, sz = [float(s) for s in kid.get('size').split()]
        c = np.array([[x, y, z] for x in (-sx / 2, sx / 2)
                      for y in (-sy / 2, sy / 2) for z in (-sz / 2, sz / 2)])
        return c
    if tag == 'cylinder':
        r, l = float(kid.get('radius')), float(kid.get('length'))
        a = np.linspace(0, 2 * np.pi, 24)
        return np.array([[r * np.cos(t), r * np.sin(t), z]
                         for t in a for z in (-l / 2, l / 2)])
    if tag == 'sphere':
        r = float(kid.get('radius'))
        return np.array([[x, y, z] for x in (-r, r) for y in (-r, r)
                         for z in (-r, r)])
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default='base_footprint')
    args = ap.parse_args()

    urdf = subprocess.run(['xacro', URDF], capture_output=True, text=True,
                          check=True).stdout
    root = ET.fromstring(urdf)

    rclpy.init()
    node = Node('mesh_extent')
    buf = Buffer()
    TransformListener(buf, node)
    end = time.time() + 5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    results = {}
    for kind in ('visual', 'collision'):
        widest, wy = None, 0.0
        allpts = []
        per_link = {}
        for link in root.findall('link'):
            name = link.get('name')
            try:
                tr = buf.lookup_transform(args.frame, name, rclpy.time.Time())
            except Exception:
                continue
            t = tr.transform
            R = quat_matrix([t.rotation.x, t.rotation.y, t.rotation.z,
                             t.rotation.w])
            T = np.array([t.translation.x, t.translation.y, t.translation.z])
            for el in link.findall(kind):
                g = el.find('geometry')
                if g is None:
                    continue
                pts = geom_points(g)
                if pts is None or len(pts) == 0:
                    continue
                o = el.find('origin')
                if o is not None:
                    xyz = np.array([float(v) for v in
                                    (o.get('xyz') or '0 0 0').split()])
                    rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
                    pts = pts @ rpy_matrix(*rpy).T + xyz
                world = pts @ R.T + T
                allpts.append(world)
                m = np.abs(world[:, 1]).max()
                per_link[name] = max(per_link.get(name, 0.0), m)
                if m > wy:
                    wy, widest = m, name
        if not allpts:
            continue
        P = np.vstack(allpts)
        results[kind] = (2 * np.abs(P[:, 1]).max(), widest,
                         P[:, 0].max() - P[:, 0].min(), P[:, 2].max())
        print(f'{kind.upper():10s} sirina {results[kind][0]:.3f} m   '
              f'duljina {results[kind][2]:.2f} m   visina {results[kind][3]:.2f} m'
              f'   najsiri: {widest}')
        for name, m in sorted(per_link.items(), key=lambda kv: -kv[1])[:5]:
            print(f'    {2 * m:.3f} m  {name}')

    if len(results) == 2:
        d = results['visual'][0] - results['collision'][0]
        print(f'\nRazlika vizualno - kolizijski: {d * 100:+.1f} cm')
        if abs(d) > 0.01:
            print('  RViz crta VIZUALNE meshove, MoveIt/Gazebo racunaju s KOLIZIJSKIMA.')
            print('  Za prolaz kroz vrata mjerodavna je kolizijska geometrija,')
            print('  ali oko vidi vizualnu - zato izgleda sire nego sto test kaze.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
