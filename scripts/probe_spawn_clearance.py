#!/usr/bin/env python3
"""Report actual table-clearance heights at rest, without commanding motion."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('cube_spawn_clearance_probe')
    joints = {}
    node.create_subscription(
        JointState, '/joint_states',
        lambda msg: joints.update(zip(msg.name, msg.position)), 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        deadline = time.monotonic() + 12.0
        frames = ('left_end_effector_link', 'right_end_effector_link',
                  'left_robotiq_85_left_finger_tip_link',
                  'right_robotiq_85_left_finger_tip_link')
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(name in joints for name in ('torso_left_carriage_joint',
                                               'torso_right_carriage_joint')):
                try:
                    points = {}
                    for frame in frames:
                        tf = buffer.lookup_transform(
                            'base_link', frame, rclpy.time.Time())
                        p = tf.transform.translation
                        points[frame] = (p.x, p.y, p.z)
                    break
                except Exception:
                    pass
        else:
            raise RuntimeError('Joint states or TF unavailable')
        for name in ('torso_left_carriage_joint',
                     'torso_right_carriage_joint'):
            print(f'{name} {joints[name]:.4f} m', flush=True)
        for frame, p in points.items():
            print(f'{frame} ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) m', flush=True)
    finally:
        node.destroy_node()
        # On Ctrl-C the signal handler has already shut the context down, and
        # calling it again raises over the real exit reason.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
