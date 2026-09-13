#!/usr/bin/env python3
"""Relay node for base controller commands, odometry, and TF.

Bridges:
- Command inputs: /cmd_vel and /base_controller/cmd_vel_unstamped ->
  /base_controller/reference_unstamped (for mecanum_drive_controller)
- Odometry: /base_controller/odometry -> /base_controller/odom
  (for backward compatibility with Nav2, SLAM, and task scripts)
- TF: /base_controller/tf_odometry -> /tf
  (for odom -> base_footprint transform)
"""
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        # Command velocity forwarding to mecanum controller
        self.cmd_pub = self.create_publisher(
            Twist, '/base_controller/reference_unstamped', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(
            Twist, '/base_controller/cmd_vel_unstamped', self._on_cmd_vel, 10)

        # Odometry forwarding for backward compatibility
        self.odom_pub = self.create_publisher(
            Odometry, '/base_controller/odom', 10)
        self.create_subscription(
            Odometry, '/base_controller/odometry', self._on_odom, 10)

        # TF odometry forwarding to standard /tf
        self.tf_pub = self.create_publisher(
            TFMessage, '/tf', 10)
        self.create_subscription(
            TFMessage, '/base_controller/tf_odometry', self._on_tf, 10)

    def _on_cmd_vel(self, msg: Twist):
        self.cmd_pub.publish(msg)

    def _on_odom(self, msg: Odometry):
        self.odom_pub.publish(msg)

    def _on_tf(self, msg: TFMessage):
        self.tf_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
