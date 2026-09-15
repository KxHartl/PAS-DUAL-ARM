#!/usr/bin/env python3
"""How wide the robot is while pressing the cube, without running a simulation.

Why this exists
---------------
The two-pad press has to fit through a 1.0 m doorway while carrying the cube,
and measuring that from a GUI run costs minutes per candidate. Everything the
question depends on - the URDF, the cube's size, where the pads must touch - is
static, so it can be answered offline in seconds and for the whole IK null space
at once instead of for the one branch a planner happened to pick.

What it found (P-43): with a HORIZONTAL approach the spherical wrist lies ON the
approach axis, 0.2025 m outboard of the fingertip, so the press is
2*(0.15 + 0.149 + 0.2025) = 1.003 m wide in EVERY one of 1620 IK solutions. The
elbow can be tucked; the wrist cannot. Tilting the approach down swings the
wrist up instead of out, and past ~50 deg the hands are inside the shoulder
mounts, which are this robot's hard floor at 0.412 m per side.

    ./scripts/run_native.sh python3 scripts/grasp_width.py
    ./scripts/run_native.sh python3 scripts/grasp_width.py --tilt 50 --show-joints

No ROS graph is needed - it reads the xacro directly.
"""
import argparse
import os
import subprocess
import sys

import numpy as np
import PyKDL as kdl

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'pas_dual_arm_scripts'))

from pas_dual_arm_scripts.force_model import ArmModel          # noqa: E402
from pas_dual_arm_scripts.kinematics import Kinematics  # noqa: E402
from pas_dual_arm_scripts.robot_extent import link_points  # noqa: E402


def urdf_xml():
    xacro = os.path.join(ROOT, 'src/pas_dual_arm_bringup/urdf/robot.urdf.xacro')
    return subprocess.run(['xacro', xacro], capture_output=True, text=True,
                          check=True).stdout


def narrowest(kin, hulls, model, side, tilt, carriage, cube, seeds):
    """Narrowest IK solution for one arm's press pose, or None.

    Returns (half width, joint values, number of solutions examined). The half
    width is the arm's own lateral reach, so left + right is the robot's width
    whenever the arms are the widest thing on it - which the caller checks.
    """
    carriages = {'torso_left_carriage_joint': carriage,
                 'torso_right_carriage_joint': carriage}
    frames = kin.place(carriages)
    base_offset = frames['base_link'][1]
    arm_R, arm_t = frames[f'{side}_base_link']
    sign = +1.0 if side == 'left' else -1.0
    cube_x, cube_z, cube_half, standoff = cube

    # Approach axis: into the near face, tilted down by `tilt`.
    approach = np.array([0.0, -sign * np.cos(tilt), -np.sin(tilt)])
    approach /= np.linalg.norm(approach)
    across = np.array([sign, 0.0, 0.0])
    normal = np.cross(approach, across)
    normal /= np.linalg.norm(normal)
    across = np.cross(normal, approach)
    goal_R = np.column_stack([across, normal, approach])
    # The fingertip touches the CENTRE of the side face; the wrist backs off
    # along the approach axis from there.
    contact = np.array([cube_x, sign * cube_half, cube_z]) + base_offset
    goal_p = contact - standoff * approach

    target = kdl.Frame(kdl.Rotation(*(arm_R.T @ goal_R).flatten()),
                       kdl.Vector(*(arm_R.T @ (goal_p - arm_t))))
    fk_solver = kdl.ChainFkSolverPos_recursive(model.chain)
    ik_solver = kdl.ChainIkSolverPos_LMA(model.chain, 1e-6, 400)
    lower = [kin.limits[n][0] for n in model.names]
    upper = [kin.limits[n][1] for n in model.names]
    arm_links = [n for n in hulls if n.startswith(side)]
    table_top = cube_z - cube_half

    best, count = None, 0
    for seed in seeds:
        start, solution = kdl.JntArray(7), kdl.JntArray(7)
        for i, value in enumerate(seed):
            start[i] = float(value)
        if ik_solver.CartToJnt(start, target, solution) < 0:
            continue
        angles = np.array([solution[i] for i in range(7)])
        if any(angles[i] < lower[i] - 1e-6 or angles[i] > upper[i] + 1e-6
               for i in range(7)):
            continue
        reached = kdl.Frame()
        fk_solver.JntToCart(solution, reached)
        if np.linalg.norm(np.array([reached.p[0], reached.p[1], reached.p[2]])
                          - (arm_R.T @ (goal_p - arm_t))) > 1e-3:
            continue
        positions = dict(carriages)
        positions.update({n: angles[i] for i, n in enumerate(model.names)})
        placed = kin.place(positions)
        points = np.vstack([hulls[n] @ placed[n][0].T + placed[n][1]
                            for n in arm_links])
        # Only geometry that is actually over the table can hit the table.
        over = points[points[:, 0] > 0.40]
        if len(over) and over[:, 2].min() < table_top - 0.01:
            continue
        count += 1
        half = points[:, 1].max() if side == 'left' else -points[:, 1].min()
        if best is None or half < best[0]:
            best = (float(half), angles)
    return None if best is None else (best[0], best[1], count)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tilt', type=float, nargs='*',
                        default=[0, 30, 40, 45, 50, 60],
                        help='approach tilt below horizontal, degrees')
    parser.add_argument('--carriage', type=float, default=0.20)
    parser.add_argument('--cube-x', type=float, default=0.62)
    parser.add_argument('--cube-z', type=float, default=0.82)
    parser.add_argument('--cube-half', type=float, default=0.15)
    parser.add_argument('--standoff', type=float, default=0.149,
                        help='wrist origin to leading fingertip, metres')
    parser.add_argument('--door', type=float, default=1.0)
    parser.add_argument('--seeds', type=int, default=600)
    parser.add_argument('--show-joints', action='store_true')
    args = parser.parse_args()

    xml = urdf_xml()
    kin = Kinematics(xml)
    hulls = link_points(xml, 'collision')
    models = {side: ArmModel(xml, side) for side in ('left', 'right')}
    rng = np.random.default_rng(2)
    seeds = rng.uniform(-3.0, 3.0, size=(args.seeds, 7))
    cube = (args.cube_x, args.cube_z, args.cube_half, args.standoff)

    print(f'cube at x={args.cube_x:.2f} z={args.cube_z:.2f}, half-depth '
          f'{args.cube_half:.3f} m, carriages {args.carriage:.2f} m')
    print(f'{"tilt":>6} {"left":>7} {"right":>7} {"width":>7}  verdict')
    for degrees in args.tilt:
        tilt = np.radians(degrees)
        sides = {s: narrowest(kin, hulls, models[s], s, tilt, args.carriage,
                              cube, seeds) for s in ('left', 'right')}
        if any(v is None for v in sides.values()):
            print(f'{degrees:6.0f}   no IK solution within the joint limits')
            continue
        width = sides['left'][0] + sides['right'][0]
        verdict = 'fits' if width < args.door else 'TOO WIDE'
        print(f'{degrees:6.0f} {sides["left"][0]:7.3f} {sides["right"][0]:7.3f} '
              f'{width:7.3f}  {verdict} (doorway {args.door:.2f} m, '
              f'{sides["left"][2]}/{sides["right"][2]} solutions)')
        if args.show_joints:
            for side, (_, angles, _) in sides.items():
                print(f'       {side}: [' +
                      ', '.join(f'{v:+.4f}' for v in angles) + ']')


if __name__ == '__main__':
    main()
