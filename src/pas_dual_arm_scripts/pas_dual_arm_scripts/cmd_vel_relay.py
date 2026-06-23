#!/usr/bin/env python3
"""Relay /cmd_vel -> /base_controller/cmd_vel_unstamped.

Nav2 (and manual teleop) publish geometry_msgs/Twist on /cmd_vel, while the
diff_drive base_controller subscribes on its own namespaced topic. The
controller runs inside the gz controller_manager, so a plain topic remap is not
available - this tiny relay bridges the two.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(
            Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.create_subscription(Twist, '/cmd_vel', self.pub.publish, 10)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
