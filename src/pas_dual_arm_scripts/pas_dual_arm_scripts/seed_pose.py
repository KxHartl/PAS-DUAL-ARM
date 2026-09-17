#!/usr/bin/env python3
"""Tell AMCL where the robot is after a mapping run, then get out of the way.

The checked-in map assumes the robot starts at the origin, which is why the
mission launch spawns it there. A map built during this run ends wherever the
robot happened to stop, so AMCL has to be told. map_handoff wrote that pose next
to the map; this node publishes it on /initialpose.

It publishes a few times rather than once: AMCL subscribes late, and a single
latched-looking message sent before it is listening is simply lost.
"""
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

# AMCL's own default spread. Large enough to absorb the small disagreement
# between the SLAM pose and the first scan match, small enough that the filter
# does not wander into the next room.
COVARIANCE = {0: 0.25, 7: 0.25, 35: 0.06853891909122467}


class SeedPose(Node):
    def __init__(self):
        super().__init__('seed_pose')
        self.declare_parameter('pose_file', '')
        self.declare_parameter('repeat', 8)
        self.declare_parameter('period', 1.0)

        self.pose_file = self.get_parameter('pose_file').value
        self.left = int(self.get_parameter('repeat').value)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.pose = self._read()
        if self.pose is None:
            # Not fatal: the map may be the checked-in one, whose origin the
            # spawn already matches. Saying so beats failing silently.
            self.get_logger().warn(
                f'No pose file at "{self.pose_file}"; leaving AMCL on its configured '
                f'initial pose.')
            self.left = 0
        self.create_timer(float(self.get_parameter('period').value), self._tick)

    def _read(self):
        if not self.pose_file or not os.path.exists(self.pose_file):
            return None
        try:
            with open(self.pose_file) as handle:
                data = yaml.safe_load(handle) or {}
            return (float(data['x']), float(data['y']),
                    float(data.get('qz', 0.0)), float(data.get('qw', 1.0)))
        except (KeyError, ValueError, TypeError, yaml.YAMLError) as exc:
            self.get_logger().error(f'Cannot read {self.pose_file}: {exc}')
            return None

    def _tick(self):
        if self.left <= 0:
            return
        x, y, qz, qw = self.pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        for index, value in COVARIANCE.items():
            msg.pose.covariance[index] = value
        self.pub.publish(msg)
        self.left -= 1
        if self.left == 0:
            yaw = math.degrees(2.0 * math.atan2(qz, qw))
            self.get_logger().info(
                f'AMCL seeded at x={x:.2f} y={y:.2f} yaw={yaw:.1f} deg.')


def main():
    rclpy.init()
    node = SeedPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
