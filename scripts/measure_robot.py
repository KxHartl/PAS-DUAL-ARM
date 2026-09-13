#!/usr/bin/env python3
"""Measure the whole robot's envelope in the posture currently shown in RViz.

Walks every frame in the TF tree, expresses it in base_footprint and reports the
overall bounding box plus a per-group breakdown (base, torso, arms, head), so a
posture can be judged against a doorway or a table.

    ./scripts/run_native.sh python3 scripts/measure_robot.py

Link origins are padded by LINK_RADIUS to approximate the real envelope; the
base is additionally floored at its known footprint (0.90 x 0.60 m).
"""
import time

import rclpy
import yaml
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

LINK_RADIUS = 0.06
BASE_L, BASE_W = 0.90, 0.60     # nav2_params footprint (+/-0.45, +/-0.30)

GROUPS = [
    ('baza', lambda f: f.startswith(('base_', 'wheel_', 'virtual_'))),
    ('torzo', lambda f: 'torso' in f or 'carriage' in f or 'wedge' in f),
    ('lijeva ruka', lambda f: f.startswith('left_')),
    ('desna ruka', lambda f: f.startswith('right_')),
    ('glava/kamera', lambda f: 'pan_tilt' in f or 'camera' in f),
]


def main():
    rclpy.init()
    node = Node('measure_robot')
    buf = Buffer()
    TransformListener(buf, node)
    end = time.time() + 5.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    frames = yaml.safe_load(buf.all_frames_as_yaml()) or {}
    pts = {}
    for f in frames:
        try:
            t = buf.lookup_transform(
                'base_footprint', f, rclpy.time.Time()).transform.translation
            pts[f] = (t.x, t.y, t.z)
        except Exception:
            continue
    if not pts:
        print('Nema TF-a. Je li display.launch.py pokrenut?')
        return 1

    def box(sel):
        s = {f: p for f, p in pts.items() if sel(f)}
        if not s:
            return None
        xs = [p[0] for p in s.values()]
        ys = [p[1] for p in s.values()]
        zs = [p[2] for p in s.values()]
        return (min(xs) - LINK_RADIUS, max(xs) + LINK_RADIUS,
                min(ys) - LINK_RADIUS, max(ys) + LINK_RADIUS,
                max(0.0, min(zs) - LINK_RADIUS), max(zs) + LINK_RADIUS, s)

    print(f'\nTF okvira: {len(pts)} (referenca: base_footprint, pod = z 0)')
    print(f'\n{"skupina":14s} {"duljina X":>22s} {"sirina Y":>22s} {"visina Z":>20s}')
    print('-' * 82)
    for name, sel in GROUPS:
        b = box(sel)
        if not b:
            continue
        x0, x1, y0, y1, z0, z1, _ = b
        print(f'{name:14s} {x1 - x0:6.2f} m [{x0:+.2f}..{x1:+.2f}] '
              f'{y1 - y0:6.2f} m [{y0:+.2f}..{y1:+.2f}] '
              f'{z1 - z0:6.2f} m [{z0:.2f}..{z1:.2f}]')

    x0, x1, y0, y1, z0, z1, _ = box(lambda f: True)
    length = max(BASE_L, x1 - x0)
    width = max(BASE_W, y1 - y0)
    print('-' * 82)
    print(f'{"UKUPNO":14s} {length:6.2f} m [{x0:+.2f}..{x1:+.2f}] '
          f'{width:6.2f} m [{y0:+.2f}..{y1:+.2f}] '
          f'{z1:6.2f} m [{z0:.2f}..{z1:.2f}]')

    # what is sticking out the most in each direction
    def extreme(key, fn, label):
        f, p = fn(pts.items(), key=lambda kv: kv[1][key])
        print(f'  {label:18s} {f}  ({p[0]:+.2f}, {p[1]:+.2f}, {p[2]:.2f})')

    print('\nNajistaknutiji linkovi:')
    extreme(0, max, 'najdalje naprijed')
    extreme(0, min, 'najdalje nazad')
    extreme(1, max, 'najdalje lijevo')
    extreme(1, min, 'najdalje desno')
    extreme(2, max, 'najvise')

    print(f'\nZa prolaz treba otvor >= {width + 0.10:.2f} m '
          f'(sirina {width:.2f} m + 10 cm rezerve).')
    print(f'Visina najvise tocke: {z1:.2f} m (zidovi su 3.0 m, vrata su bez nadvoja).')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
