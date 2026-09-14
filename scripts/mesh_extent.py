#!/usr/bin/env python3
"""True lateral extent of the robot from mesh vertices, visual vs collision.

RViz draws the VISUAL meshes while MoveIt collides the COLLISION meshes. If the
two differ, what you see is not what the planner checks. This transforms every
mesh vertex by its link's live TF and reports the real width of both.

The geometry itself lives in `pas_dual_arm_scripts.robot_extent`, which the
`envelope_monitor` node uses to measure the same thing continuously while the
robot drives. One implementation, so the number printed here and the number the
navigator gates on cannot drift apart.

    ./scripts/run_native.sh python3 scripts/mesh_extent.py
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, 'src', 'pas_dual_arm_bringup', 'urdf', 'robot.urdf.xacro')
sys.path.insert(0, os.path.join(ROOT, 'src', 'pas_dual_arm_scripts'))

from pas_dual_arm_scripts.robot_extent import link_points, quat_matrix  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frame', default='base_footprint')
    args = parser.parse_args()

    urdf = subprocess.run(['xacro', URDF], capture_output=True, text=True,
                          check=True).stdout

    rclpy.init()
    node = Node('mesh_extent')
    buffer = Buffer()
    TransformListener(buffer, node)
    deadline = time.time() + 5
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    results = {}
    for kind in ('visual', 'collision'):
        widest, half_width = None, 0.0
        collected, per_link = [], {}
        for name, points in link_points(urdf, kind).items():
            try:
                tf = buffer.lookup_transform(args.frame, name, rclpy.time.Time()).transform
            except Exception:
                continue
            rotation = quat_matrix([tf.rotation.x, tf.rotation.y,
                                    tf.rotation.z, tf.rotation.w])
            translation = np.array([tf.translation.x, tf.translation.y,
                                    tf.translation.z])
            world = points @ rotation.T + translation
            collected.append(world)
            reach = float(np.abs(world[:, 1]).max())
            per_link[name] = max(per_link.get(name, 0.0), reach)
            if reach > half_width:
                half_width, widest = reach, name
        if not collected:
            continue
        allpoints = np.vstack(collected)
        results[kind] = (2 * np.abs(allpoints[:, 1]).max(), widest,
                         allpoints[:, 0].max() - allpoints[:, 0].min(),
                         allpoints[:, 2].max())
        print(f'{kind.upper():10s} sirina {results[kind][0]:.3f} m   '
              f'duljina {results[kind][2]:.2f} m   visina {results[kind][3]:.2f} m'
              f'   najsiri: {widest}')
        for name, reach in sorted(per_link.items(), key=lambda kv: -kv[1])[:5]:
            print(f'    {2 * reach:.3f} m  {name}')

    if len(results) == 2:
        difference = results['visual'][0] - results['collision'][0]
        print(f'\nRazlika vizualno - kolizijski: {difference * 100:+.1f} cm')
        if abs(difference) > 0.01:
            print('  RViz crta VIZUALNE meshove, MoveIt/Gazebo racunaju s KOLIZIJSKIMA.')
            print('  Za prolaz kroz vrata mjerodavna je kolizijska geometrija,')
            print('  ali oko vidi vizualnu - zato izgleda sire nego sto test kaze.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
