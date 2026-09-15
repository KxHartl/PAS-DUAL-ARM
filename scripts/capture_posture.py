#!/usr/bin/env python3
"""Capture the arm posture currently shown in RViz and record its dimensions.

Reads /joint_states (published by joint_state_publisher_gui while you drag the
sliders) and the TF tree (published by robot_state_publisher), measures how wide
/ long / tall the robot is in that posture, and appends the result to the
posture registry in notes/.

    ./scripts/run_native.sh python3 scripts/capture_posture.py <ime> ["opis"]

Width is measured across every arm/gripper link as 2 * (max |y| + LINK_RADIUS)
and floored at the base width, so it is directly comparable between postures.
"""
import argparse
import datetime
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

# Rough tube radius added around each link origin so the numbers describe the
# real envelope rather than the joint centre line.
LINK_RADIUS = 0.06
# The omni base itself is 0.6 m wide; the robot is never narrower than that.
BASE_WIDTH = 0.60

ARM_LINKS = [
    'shoulder_link', 'half_arm_1_link', 'half_arm_2_link', 'forearm_link',
    'spherical_wrist_1_link', 'spherical_wrist_2_link', 'bracelet_link',
    'end_effector_link', 'robotiq_85_base_link',
    'robotiq_85_left_finger_tip_link', 'robotiq_85_right_finger_tip_link',
]

REGISTRY = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'notes', '08_poze.md')


def measure(node, buf):
    """Return (dims, per-link positions) of the arm links in base_link."""
    points = {}
    for side in ('left', 'right'):
        for link in ARM_LINKS:
            name = f'{side}_{link}'
            try:
                t = buf.lookup_transform(
                    'base_link', name, rclpy.time.Time()).transform.translation
            except Exception:
                continue
            points[name] = (t.x, t.y, t.z)
    if not points:
        return None, {}
    ys = [p[1] for p in points.values()]
    xs = [p[0] for p in points.values()]
    zs = [p[2] for p in points.values()]
    widest = max(points.items(), key=lambda kv: abs(kv[1][1]))
    dims = {
        'width': max(BASE_WIDTH, 2 * (max(abs(y) for y in ys) + LINK_RADIUS)),
        'max_y': max(abs(y) for y in ys),
        'front': max(xs) + LINK_RADIUS,
        'back': min(xs) - LINK_RADIUS,
        'top': max(zs) + LINK_RADIUS,
        'bottom': min(zs) - LINK_RADIUS,
        'widest_link': widest[0],
    }
    return dims, points


def joints(node):
    """Latest arm joint positions, as {side: {index: value}}."""
    got = {}

    def cb(msg):
        for name, pos in zip(msg.name, msg.position):
            got[name] = pos

    node.create_subscription(JointState, '/joint_states', cb, 10)
    end = time.time() + 8.0
    while time.time() < end and not got:
        rclpy.spin_once(node, timeout_sec=0.2)
    out = {'left': {}, 'right': {}}
    for side in out:
        for j in range(1, 8):
            key = f'{side}_joint_{j}'
            if key in got:
                out[side][j] = got[key]
    extra = {k: v for k, v in got.items()
             if 'carriage' in k or 'pan_tilt' in k or k.endswith('robotiq_85_left_knuckle_joint')}
    return out, extra


def symmetric(q):
    """True when both arms hold the same joint values (within 0.01 rad)."""
    return all(abs(q['left'].get(j, 0.0) - q['right'].get(j, 0.0)) < 0.01
               for j in range(1, 8))


def fmt_dict(d):
    return '{' + ', '.join(f'{j}: {v:.3f}' for j, v in sorted(d.items())) + '}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name', help='posture name, e.g. ARM_DOOR')
    ap.add_argument('description', nargs='?', default='',
                    help='what the posture is for')
    args = ap.parse_args()

    rclpy.init()
    node = Node('capture_posture')
    buf = Buffer()
    TransformListener(buf, node)
    end = time.time() + 5.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    q, extra = joints(node)
    dims, points = measure(node, buf)
    if dims is None:
        print('Nema TF-a za ruke - je li display.launch.py pokrenut?')
        return 1

    door = dims['width'] + 0.10
    print(f"\n=== POZA: {args.name} ===")
    if args.description:
        print(args.description)
    print(f"  sirina robota   {dims['width']:.2f} m   (najsiri link: "
          f"{dims['widest_link'].replace('_link', '')} na y={dims['max_y']:+.2f})")
    print(f"  potrebna vrata  {door:.2f} m   (sirina + 10 cm)")
    print(f"  naprijed/nazad  {dims['front']:+.2f} / {dims['back']:+.2f} m")
    print(f"  visina          {dims['bottom']:.2f} .. {dims['top']:.2f} m")
    sym = symmetric(q)
    print(f"  simetricna      {'DA' if sym else 'NE (ruke razlicito)'}")
    print(f"  lijeva  {fmt_dict(q['left'])}")
    print(f"  desna   {fmt_dict(q['right'])}")
    if extra:
        print('  ostalo  ' + ', '.join(f'{k}={v:.3f}' for k, v in sorted(extra.items())))

    stamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
    row = (f"| `{args.name}` | {dims['width']:.2f} m | {door:.2f} m | "
           f"{dims['front']:.2f} m | {dims['bottom']:.2f}–{dims['top']:.2f} m | "
           f"{dims['widest_link'].replace('_link', '')} | "
           f"{'da' if sym else 'ne'} | {args.description or '—'} | {stamp} |\n")
    block = (f"\n### {args.name}\n"
             f"{args.description}\n\n"
             f"```python\n"
             f"# lijeva ruka\n{args.name}_LEFT = {fmt_dict(q['left'])}\n"
             f"# desna ruka\n{args.name}_RIGHT = {fmt_dict(q['right'])}\n"
             # The rest of the posture: carriages and gripper opening. They
             # used to be printed and dropped, so a saved grasp did not say
             # how high or how closed it was.
             + ''.join(f"{args.name}_{key} = {value!r}\n" for key, value in (
                 ('TORSO', {k: round(v, 4) for k, v in sorted(extra.items()) if 'carriage' in k}),
                 ('GRIPPER', {k: round(v, 4) for k, v in sorted(extra.items()) if 'knuckle' in k}),
             ) if value) +
             f"```\n"
             f"Širina {dims['width']:.2f} m → vrata {door:.2f} m; "
             f"doseg naprijed {dims['front']:.2f} m; "
             f"visina {dims['bottom']:.2f}–{dims['top']:.2f} m; "
             f"najširi link `{dims['widest_link']}`. Snimljeno {stamp}.\n")

    if not os.path.exists(REGISTRY):
        with open(REGISTRY, 'w') as f:
            f.write(HEADER)
    with open(REGISTRY) as f:
        text = f.read()
    marker = '<!-- POSTURE ROWS -->'
    if marker in text:
        text = text.replace(marker, marker + '\n' + row.rstrip('\n'))
    else:
        text += row
    text += block
    with open(REGISTRY, 'w') as f:
        f.write(text)
    print(f"\nSpremljeno u {os.path.relpath(REGISTRY)}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


HEADER = """---
id: POZE
type: registar
updated: 2026-09-13
---
# Registar poza ruku (izmjereno, ne procijenjeno)

> Poze snima korisnik u RViz-u (`display.launch.py`, slideri), a sprema ih
> `scripts/capture_posture.py`. Svaka poza nosi **izmjerene** dimenzije iz TF-a, pa se zna što
> stvarno predstavlja. Širina = 2 × (max |y| linkova ruku + 0.06 m), najmanje 0.60 m (širina baze).
> „Vrata“ = širina + 10 cm (pravilo korisnika, 13. 9. 2026.).
> Vidi [[P-35_arm_span_too_wide_for_door]] i [[06_parametri]].

| Poza | Širina | Vrata | Naprijed | Visina | Najširi link | Simetrična | Svrha | Snimljeno |
|---|---|---|---|---|---|---|---|---|
<!-- POSTURE ROWS -->
"""


if __name__ == '__main__':
    sys.exit(main())
