#!/usr/bin/env python3
"""Find the narrowest doorway the robot fits through, in its current posture.

Instead of padding link origins by a guess, this puts a real wall with a gap
into the MoveIt planning scene and asks MoveIt whether the robot collides with
it, using the very collision meshes MoveIt and Gazebo use. A binary search over
the gap width then gives the true minimum.

    ./scripts/run_native.sh python3 scripts/fit_test.py            # current posture
    ./scripts/run_native.sh python3 scripts/fit_test.py --poza ARM_CARRY_V2

Requires move_group to be running.
"""
import argparse
import ast
import os
import re
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REGISTRY = os.path.join(ROOT, 'notes', '08_poze.md')
# The gauge must be a CORRIDOR, not a thin slab: a 0.1 m thick wall at x=0 only
# samples the cross-section there, and arms reaching to x=+0.7 simply pass
# beside it. CORRIDOR_L spans the whole robot length so the widest point counts.
CORRIDOR_L, WALL_W, WALL_H = 3.00, 1.50, 2.10


def load_posture(name):
    text = open(REGISTRY).read()
    out = {}
    for side in ('LEFT', 'RIGHT'):
        m = re.search(rf'^{re.escape(name)}_{side}\s*=\s*(\{{[^}}]*\}})', text, re.M)
        if m:
            for j, v in ast.literal_eval(m.group(1)).items():
                out[f'{side.lower()}_joint_{j}'] = float(v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--poza', help='poza iz notes/08_poze.md; inace trenutna')
    ap.add_argument('--od', type=float, default=0.60, help='donja granica [m]')
    ap.add_argument('--do', dest='gore', type=float, default=1.60,
                    help='gornja granica [m]')
    ap.add_argument('--tocnost', type=float, default=0.005, help='[m]')
    args = ap.parse_args()

    rclpy.init()
    node = Node('fit_test')
    apply_cli = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    valid_cli = node.create_client(GetStateValidity, '/check_state_validity')
    for c, n in ((apply_cli, '/apply_planning_scene'), (valid_cli, '/check_state_validity')):
        if not c.wait_for_service(timeout_sec=25.0):
            print(f'{n} nedostupan - je li move_group pokrenut?')
            return 1

    # posture: from the registry, or whatever is on /joint_states right now
    state = {}
    sub = node.create_subscription(
        JointState, '/joint_states',
        lambda m: state.update(zip(m.name, m.position)), 10)
    end = time.time() + 6
    while time.time() < end and not state:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if args.poza:
        state.update(load_posture(args.poza))
    if not state:
        print('nema /joint_states')
        return 1

    js = JointState()
    for name, pos in sorted(state.items()):
        js.name.append(name)
        js.position.append(float(pos))
    rs = RobotState()
    rs.joint_state = js
    rs.is_diff = True

    def call(cli, req, timeout=10.0):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(node, f, timeout_sec=timeout)
        return f.result()

    def set_walls(gap):
        """Two slabs in base_footprint leaving a centred gap of `gap` metres."""
        scene = PlanningScene()
        scene.is_diff = True
        for i, sign in enumerate((+1, -1)):
            co = CollisionObject()
            co.header.frame_id = 'base_footprint'
            co.id = f'gauge_{i}'
            co.operation = CollisionObject.ADD
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [CORRIDOR_L, WALL_W, WALL_H]
            p = Pose()
            p.position.x = 0.0
            p.position.y = sign * (gap / 2.0 + WALL_W / 2.0)
            p.position.z = WALL_H / 2.0
            p.orientation.w = 1.0
            co.primitives = [box]
            co.primitive_poses = [p]
            scene.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        return call(apply_cli, req) is not None

    def fits(gap):
        set_walls(gap)
        time.sleep(0.25)
        req = GetStateValidity.Request()
        req.group_name = ''          # empty = whole robot, not just one group
        req.robot_state = rs
        r = call(valid_cli, req)
        if r is None:
            return None, []
        hits = sorted({c.contact_body_1 if 'gauge' not in c.contact_body_1
                       else c.contact_body_2 for c in r.contacts})
        return r.valid, hits

    label = args.poza or 'trenutna poza'
    print(f'\nMjerim stvarnom kolizijskom geometrijom (MoveIt), poza: {label}')
    ok, hits = fits(args.gore)
    if not ok:
        print(f'  Ni na {args.gore:.2f} m ne prolazi - sudara: {", ".join(hits[:4])}')
        print('  (mozda je poza sama po sebi u samokoliziji)')
        return 1
    bad, _ = fits(args.od)
    if bad:
        print(f'  Prolazi vec na {args.od:.2f} m; suzi granicu s --od')
        return 0

    lo, hi = args.od, args.gore     # lo: ne prolazi, hi: prolazi
    last_hits = []
    while hi - lo > args.tocnost:
        mid = (lo + hi) / 2.0
        ok, hits = fits(mid)
        print(f'  otvor {mid * 100:6.1f} cm -> {"PROLAZI" if ok else "sudar: " + ", ".join(h.replace("_link", "") for h in hits[:3])}')
        if ok:
            hi = mid
        else:
            lo, last_hits = mid, hits

    print(f'\n  MINIMALNI OTVOR: {hi * 100:.1f} cm')
    if last_hits:
        print(f'  prvi dodirne: {", ".join(h for h in last_hits[:4])}')
    print(f'  uz 10 cm rezerve -> vrata {(hi + 0.10) * 100:.0f} cm')

    # clean the gauge out of the scene again
    scene = PlanningScene()
    scene.is_diff = True
    for i in range(2):
        co = CollisionObject()
        co.header.frame_id = 'base_footprint'
        co.id = f'gauge_{i}'
        co.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(co)
    req = ApplyPlanningScene.Request()
    req.scene = scene
    call(apply_cli, req)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
