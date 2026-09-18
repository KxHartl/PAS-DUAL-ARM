#!/usr/bin/env python3
"""Where the robot is, measured from the walls instead of the wheels.

The wheels of this base cannot report how it moves sideways. Measured against
ground truth over ten runs, wheel odometry drifts 30.1 mm forwards and 22.3 mm
sideways over a run; the same scans, matched, drift 0.9 and 1.5. It wins every
run on both axes. That is not a tuning difference - a mecanum base with
mu2 = 0.20 SLIDES sideways by design, and a slide leaves the wheels no rotation
to count. PAL reached the same conclusion for this base and ships it with
`enable_odom_tf: false` and the comment "odom tf will be published by direct
laser odometry" (D-25, P-52).

This node is the same matching that was measured, wrapped for ROS. The algorithm
lives in scan_matcher.py and is imported, not copied: a matcher validated offline
and then reimplemented online has not been validated.

    publish_tf:=false  (default)  publishes /laser_odom only, and changes nothing
    publish_tf:=true              takes over odom -> base_footprint

It stays false until a series says otherwise, because the wheels currently own
that transform through cmd_vel_relay and two publishers on one transform is a
worse failure than either alone (D-18).
"""
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from pas_dual_arm_scripts.scan_matcher import match, scan_to_points


def quaternion_z(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """Pose `b` expressed in the frame of pose `a`, both (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + b[0] * c - b[1] * s,
            a[1] + b[0] * s + b[1] * c,
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def invert(a):
    c, s = math.cos(-a[2]), math.sin(-a[2])
    return (-(a[0] * c - a[1] * s), -(a[0] * s + a[1] * c), -a[2])


class LaserOdometry(Node):

    def __init__(self):
        super().__init__('laser_odometry')
        self.declare_parameter('scan_topic', '/scan_filtered')
        self.declare_parameter('wheel_odom_topic', '/base_controller/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', False)
        # Every Nth scan. The lidar runs at about 13 Hz and the matching needs
        # nothing like that: 19 ms a match measured, against ~4 Hz of work.
        self.declare_parameter('stride', 3)
        # A keyframe only moves after this much, which is what stops one
        # matching error per scan from compounding - scan-to-scan drifted
        # 286 mm over a run, keyframes brought it to 22, the metric did the rest.
        self.declare_parameter('keyframe_distance', 0.30)
        self.declare_parameter('keyframe_angle', 0.26)
        # A match this poor is not trusted; the wheels carry that interval.
        self.declare_parameter('max_fitness', 0.08)

        self._pose = (0.0, 0.0, 0.0)        # laser in `odom`
        self._key_points = None
        self._key_pose = (0.0, 0.0, 0.0)
        self._key_wheel = None
        self._wheel = None
        self._base_from_laser = None
        self._count = 0
        self._skipped = 0

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.caster = tf2_ros.TransformBroadcaster(self)

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan,
                                 self.get_parameter('scan_topic').value,
                                 self._on_scan, sensor_qos)
        self.create_subscription(Odometry,
                                 self.get_parameter('wheel_odom_topic').value,
                                 self._on_wheel, 20)
        self.publisher = self.create_publisher(Odometry, '/laser_odom', 10)
        self.get_logger().info(
            'laser odometry: matching %s; publish_tf=%s' % (
                self.get_parameter('scan_topic').value,
                self.get_parameter('publish_tf').value))

    def _on_wheel(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._wheel = (p.x, p.y, yaw_of(q))

    def _laser_offset(self, frame):
        """base_footprint -> laser, looked up once; the mounting does not move."""
        if self._base_from_laser is not None:
            return self._base_from_laser
        try:
            tf = self.buffer.lookup_transform(
                self.get_parameter('base_frame').value, frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        self._base_from_laser = (t.x, t.y, yaw_of(tf.transform.rotation))
        self.get_logger().info(
            'laser sits at (%.3f, %.3f, %.1f deg) in %s' % (
                t.x, t.y, math.degrees(self._base_from_laser[2]),
                self.get_parameter('base_frame').value))
        return self._base_from_laser

    def _on_scan(self, msg):
        self._count += 1
        if self._count % max(1, self.get_parameter('stride').value):
            return
        offset = self._laser_offset(msg.header.frame_id)
        if offset is None:
            return

        points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max)
        if self._key_points is None:
            self._key_points, self._key_pose = points, self._pose
            self._key_wheel = self._wheel
            return

        # Seed from the wheels. They are poor sideways and good forwards, which
        # makes them a bad measurement and a perfectly good starting point.
        guess = (0.0, 0.0, 0.0)
        if self._wheel is not None and self._key_wheel is not None:
            a, b = self._key_wheel, self._wheel
            c, s = math.cos(-a[2]), math.sin(-a[2])
            dx, dy = b[0] - a[0], b[1] - a[1]
            guess = (dx * c - dy * s, dx * s + dy * c,
                     math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2])))

        got = match(self._key_points, points, guess=guess)
        if got is None:
            self._skipped += 1
            return
        dx, dy, dtheta, fitness = got
        if fitness > self.get_parameter('max_fitness').value:
            self._skipped += 1
            if self._skipped % 20 == 1:
                self.get_logger().warn(
                    'scan match rejected: %.3f m residual, keeping the previous pose' % fitness)
            return

        self._pose = compose(self._key_pose, (dx, dy, dtheta))
        if (math.hypot(dx, dy) > self.get_parameter('keyframe_distance').value
                or abs(dtheta) > self.get_parameter('keyframe_angle').value):
            self._key_points, self._key_pose = points, self._pose
            self._key_wheel = self._wheel

        base = compose(self._pose, invert(offset))
        self._publish(base, msg.header.stamp)

    def _publish(self, base, stamp):
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        qz, qw = quaternion_z(base[2])

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = base[0], base[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        self.publisher.publish(msg)

        if not self.get_parameter('publish_tf').value:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = odom_frame
        tf.child_frame_id = base_frame
        tf.transform.translation.x, tf.transform.translation.y = base[0], base[1]
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.caster.sendTransform(tf)


def main():
    rclpy.init()
    node = LaserOdometry()
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
