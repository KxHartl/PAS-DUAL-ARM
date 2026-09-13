#!/usr/bin/env python3
"""Room navigation with orthogonal door staging and straight-line passage.

Ensures the robot never approaches narrow 1.0 m doorways at an angle:
  1. Navigates to a staging pose 1.3 m before the door on its centerline.
  2. Turns in open space to align perpendicular to the wall.
  3. Drives straight through the doorway to the transit pose on the other side.
  4. Continues to the desired room destination.
"""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw in radians to geometry_msgs Quaternion."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw * 0.5),
        w=math.cos(yaw * 0.5),
    )


# Key poses in map frame (meters, radians)
# Door HOME <-> BLUE is at Y = -3.0, X in [-0.5, 0.5]
# Staging poses positioned 1.20 m from door threshold (front edge is 0.68 m from door)
BLUE_STAGING_HOME = (0.0, -1.80, -math.pi / 2)    # in HOME, facing BLUE (1.2 m from door)
BLUE_TRANSIT_ROOM = (0.0, -4.20, -math.pi / 2)    # in BLUE, facing BLUE (1.2 m past door)
BLUE_GOAL_REGION  = (0.0, -5.20, -math.pi / 2)    # in BLUE, facing table / ArUco

# Door HOME <-> RED is at X = 3.0, Y in [-0.5, 0.5]
RED_STAGING_HOME  = (1.80, 0.0, 0.0)             # in HOME, facing RED (1.2 m from door)
RED_TRANSIT_ROOM  = (4.20, 0.0, 0.0)             # in RED, facing RED (1.2 m past door)
RED_GOAL_REGION   = (5.20, 0.0, 0.0)             # in RED, facing place table

# Reverse poses for returning to HOME
BLUE_STAGING_ROOM = (0.0, -4.20, math.pi / 2)     # in BLUE, facing HOME
BLUE_TRANSIT_HOME = (0.0, -1.80, math.pi / 2)     # in HOME, facing HOME

RED_STAGING_ROOM  = (4.20, 0.0, math.pi)          # in RED, facing HOME
RED_TRANSIT_HOME  = (1.80, 0.0, math.pi)          # in HOME, facing HOME

HOME_CENTER       = (0.0, 0.0, 0.0)


class RoomNavigator(Node):
    def __init__(self):
        super().__init__('room_navigator')
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def wait_for_server(self):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available!')
            return False
        return True

    def send_goal(self, x: float, y: float, yaw: float, description: str) -> bool:
        self.get_logger().info(f'--> Step: {description} (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f})')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quaternion(yaw)

        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal for {description} was rejected by Nav2!')
            return False

        self.get_logger().info(f'Navigating to {description}...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[OK] Reached {description}')
            return True
        else:
            self.get_logger().error(f'[FAIL] Failed to reach {description} (status: {status})')
            return False

    def navigate_sequence(self, steps):
        for x, y, yaw, desc in steps:
            success = self.send_goal(x, y, yaw, desc)
            if not success:
                self.get_logger().error(f'Navigation sequence aborted at: {desc}')
                return False
            time.sleep(0.5)
        return True


def main():
    parser = argparse.ArgumentParser(description='Orthogonal room navigation')
    parser.add_argument('room', choices=['blue', 'red', 'home'],
                        help='Destination room: blue, red, or home')
    args = parser.parse_args()

    rclpy.init()
    nav = RoomNavigator()
    if not nav.wait_for_server():
        rclpy.shutdown()
        sys.exit(1)

    if args.room == 'blue':
        steps = [
            (*BLUE_STAGING_HOME, '1. Door Staging (HOME side of Blue door)'),
            (*BLUE_TRANSIT_ROOM, '2. Straight passage through door'),
            (*BLUE_GOAL_REGION,  '3. Blue Room target region'),
        ]
    elif args.room == 'red':
        steps = [
            (*RED_STAGING_HOME, '1. Door Staging (HOME side of Red door)'),
            (*RED_TRANSIT_ROOM, '2. Straight passage through door'),
            (*RED_GOAL_REGION,  '3. Red Room target region'),
        ]
    elif args.room == 'home':
        steps = [
            (*HOME_CENTER, 'Return to HOME center'),
        ]

    nav.get_logger().info(f'Starting orthogonal navigation to: {args.room.upper()}')
    success = nav.navigate_sequence(steps)
    if success:
        nav.get_logger().info(f'SUCCESS: Reached {args.room.upper()}!')
    else:
        nav.get_logger().error(f'FAILED to complete navigation to {args.room.upper()}')

    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
