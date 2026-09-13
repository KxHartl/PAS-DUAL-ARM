"""Command the pan-tilt camera head to a specific pitch and yaw angle.

Usage:
  ros2 run pas_dual_arm_scripts set_camera [pitch] [yaw]
  ros2 run pas_dual_arm_scripts set_camera --pitch 0.45 --yaw 0.0

Default: pitch=0.45 rad (~25.8 deg downwards, looking at 75cm tables), yaw=0.0 rad.
"""

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SetCamera(Node):
    def __init__(self):
        super().__init__('set_camera')
        self.client = ActionClient(
            self, FollowJointTrajectory, '/pan_tilt_controller/follow_joint_trajectory')

    def set_angles(self, pitch: float, yaw: float = 0.0, duration_sec: float = 1.5) -> bool:
        self.get_logger().info('Waiting for /pan_tilt_controller/follow_joint_trajectory...')
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('pan_tilt_controller action server not available')
            return False

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['pan_tilt_yaw_joint', 'pan_tilt_pitch_joint']

        pt = JointTrajectoryPoint()
        pt.positions = [float(yaw), float(pitch)]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start.sec = sec
        pt.time_from_start.nanosec = nanosec

        traj.points.append(pt)
        goal.trajectory = traj

        self.get_logger().info(f'Moving camera to pitch={pitch:.2f} rad ({pitch*180/3.14159:.1f}°), yaw={yaw:.2f} rad ({yaw*180/3.14159:.1f}°)...')
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by pan_tilt_controller')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.get_logger().info('Camera positioned successfully.')
        return True


def main():
    parser = argparse.ArgumentParser(description='Set pan-tilt camera pitch and yaw')
    parser.add_argument('pos_pitch', nargs='?', type=float, default=None,
                        help='Pitch angle in radians (positive = downwards)')
    parser.add_argument('pos_yaw', nargs='?', type=float, default=None,
                        help='Yaw angle in radians (positive = left)')
    parser.add_argument('--pitch', type=float, default=0.45,
                        help='Pitch angle in radians (default: 0.45 rad = ~25.8° down)')
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='Yaw angle in radians (default: 0.0 rad)')
    parser.add_argument('--duration', type=float, default=1.5,
                        help='Movement duration in seconds (default: 1.5)')

    args = parser.parse_args()
    pitch = args.pos_pitch if args.pos_pitch is not None else args.pitch
    yaw = args.pos_yaw if args.pos_yaw is not None else args.yaw

    rclpy.init()
    node = SetCamera()
    ok = node.set_angles(pitch=pitch, yaw=yaw, duration_sec=args.duration)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
