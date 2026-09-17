#!/usr/bin/env python3
"""Relay node for base controller commands, odometry, and TF.

Bridges:
- Command inputs: /cmd_vel and /base_controller/cmd_vel_unstamped ->
  /base_controller/reference_unstamped (for mecanum_drive_controller)
- Safety: while /cmd_vel_safe is being published, raw /cmd_vel is ignored, so
  the collision monitor filtering Nav2's output cannot be routed around. With
  no monitor running nothing arrives on that topic and /cmd_vel drives the base
  exactly as before, which keeps teleop and the mapping tour working.
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
        # The collision monitor's output. Longer than the monitor's own period
        # so a late message does not briefly hand control back to raw /cmd_vel.
        self.declare_parameter('safe_command_timeout', 1.0)
        self._last_safe = None
        self.create_subscription(Twist, '/cmd_vel_safe', self._on_safe_cmd, 10)

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

    def _now(self):
        """Seconds on the same clock everything else in this run uses.

        This was wall-clock (monotonic) while every other timeout in the system
        is simulation time. At RTF 0.57 a 1.0 s wall window is 0.57 s of
        simulated time, so a collision monitor publishing at its nominal rate
        can still look dead here and raw /cmd_vel gets control back while the
        monitor is in fact filtering. It held only because the margin was
        generous (P-47; AGENT_GUIDE invariant 4).
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _safety_live(self):
        timeout = self.get_parameter('safe_command_timeout').value
        return self._last_safe is not None and \
            self._now() - self._last_safe <= timeout

    def _on_cmd_vel(self, msg: Twist):
        if self._safety_live():
            return          # the monitor is filtering; its output is the truth
        self.cmd_pub.publish(msg)

    def _on_safe_cmd(self, msg: Twist):
        if self._last_safe is None:
            self.get_logger().info(
                'collision monitor is live; raw /cmd_vel is now ignored')
        self._last_safe = self._now()
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
