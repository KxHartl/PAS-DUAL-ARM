#!/usr/bin/env python3
"""Read-only MoveIt FK comparison of known arm postures."""

import rclpy
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node

from pas_dual_arm_scripts.postures import POSTURES


def main():
    rclpy.init()
    node = Node('cube_posture_fk_probe')
    try:
        client = node.create_client(GetPositionFK, '/compute_fk')
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('MoveIt FK service unavailable')
        for name, sides in POSTURES.items():
            req = GetPositionFK.Request()
            req.header.frame_id = 'base_link'
            req.fk_link_names = ['left_end_effector_link',
                                 'right_end_effector_link']
            req.robot_state.is_diff = True
            for side, values in sides.items():
                for j, value in values.items():
                    req.robot_state.joint_state.name.append(f'{side}_joint_{j}')
                    req.robot_state.joint_state.position.append(float(value))
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            res = fut.result()
            if res is None or res.error_code.val != 1:
                print(f'{name}: FK failed', flush=True)
                continue
            for link, pose in zip(res.fk_link_names, res.pose_stamped):
                p = pose.pose.position
                print(f'{name} {link} ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})',
                      flush=True)
    finally:
        node.destroy_node()
        # On Ctrl-C the signal handler has already shut the context down, and
        # calling it again raises over the real exit reason.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
