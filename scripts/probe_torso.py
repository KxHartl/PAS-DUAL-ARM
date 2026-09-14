#!/usr/bin/env python3
"""Measure loaded carriage tracking in the isolated cube simulation."""

import argparse
import json
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = ('torso_left_carriage_joint', 'torso_right_carriage_joint')


class TorsoProbe(Node):
    def __init__(self):
        super().__init__('cube_torso_probe')
        self.positions = {}
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/torso_controller/follow_joint_trajectory')

    def on_joints(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def wait_for_positions(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(joint in self.positions for joint in JOINTS):
                return [float(self.positions[joint]) for joint in JOINTS]
        raise TimeoutError('no torso joint positions received')

    def send_height(self, height, duration):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise TimeoutError('torso trajectory action unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [height, height]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        goal.trajectory.points = [point]
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('torso trajectory goal rejected')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + 15.0)
        result = result_future.result()
        if result is None:
            raise TimeoutError('torso trajectory result unavailable')
        return int(result.status), int(result.result.error_code)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--height', type=float, default=0.20)
    parser.add_argument('--duration', type=float, default=4.0)
    parser.add_argument('--settle', type=float, default=2.0)
    args = parser.parse_args()
    rclpy.init()
    node = TorsoProbe()
    try:
        before = node.wait_for_positions(10.0)
        status, error_code = node.send_height(args.height, args.duration)
        settle_until = time.monotonic() + args.settle
        while time.monotonic() < settle_until:
            rclpy.spin_once(node, timeout_sec=0.1)
        after = node.wait_for_positions(2.0)
        print(json.dumps({
            'before_m': before,
            'command_m': args.height,
            'after_m': after,
            'error_m': [round(args.height - value, 4) for value in after],
            'action_status': status,
            'action_error_code': error_code,
        }, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
