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
from sensor_msgs.msg import Imu, LaserScan

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
        self.declare_parameter('imu_topic', '/base_imu')
        # Where heading comes from. Measured over the calibration drive
        # (scripts/yaw_drive.py, validation/evaluate_imu_yaw.py), against
        # ground truth, mean absolute error over the whole drive:
        #
        #     IMU (gyro, integrated)   0.03 deg
        #     laser (scan matching)    0.12 deg
        #     wheels                   3.23 deg, ending 14.94 deg out
        #
        # The wheels are not merely worse, they are worse by two orders of
        # magnitude, and a heading error is the one error that grows with
        # distance travelled rather than staying put. `wheels` is kept so the
        # comparison can be re-run, not because it is a reasonable choice.
        self.declare_parameter('yaw_source', 'imu')
        # Where the TRANSLATION comes from, which is the part the wheels can
        # still do. `wheels` takes their step between two messages and rotates
        # it by the gyro's heading, so the transform goes out at their rate.
        # `laser` drops them entirely: translation then changes only when a
        # scan matches, about 13 times a second against a controller running at
        # 20, and the transform goes out on the IMU instead.
        #
        # Which is better is a measurement, not an argument. The argument for
        # keeping them - that 13 Hz is too thin to steer on - was made before
        # anything was measured, and PAL's `enable_odom_tf: false` forbids the
        # wheels the TRANSFORM, not the use of what they measure.
        self.declare_parameter('translation_source', 'wheels')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', False)
        # Every Nth scan. 1 means every one the lidar gives, about 13 Hz, and a
        # match costs 19 ms so there is no reason to ask for less. The offline
        # evaluation used 3 only to get through fifty bags faster.
        self.declare_parameter('stride', 1)
        # A keyframe only moves after this much, which is what stops one
        # matching error per scan from compounding - scan-to-scan drifted
        # 286 mm over a run, keyframes brought it to 22, the metric did the rest.
        self.declare_parameter('keyframe_distance', 0.30)
        self.declare_parameter('keyframe_angle', 0.26)
        # A match this poor is not trusted; the wheels carry that interval.
        self.declare_parameter('max_fitness', 0.08)

        self._pose = (0.0, 0.0, 0.0)        # laser in `odom`
        # The laser corrects; the wheels carry between corrections.
        #
        # 13 Hz is the lidar's rate and it is the ceiling on matching, but a
        # transform at 13 Hz is thin for a controller running at 20. So the
        # laser is not asked to BE the odometry - it is asked to correct it, and
        # the transform goes out whenever the wheels report, at 50 Hz.
        #
        # This is the same shape AMCL uses for map -> odom: a correction that
        # updates when a measurement arrives, composed with a fast dead
        # reckoning in between. It plays to what each one is good at. Wheels are
        # accurate over a tenth of a second and hopeless over a run, because
        # their error accumulates; the laser is accurate over a run and arrives
        # too rarely to steer on. Between two matches the wheels move the robot
        # a few millimetres and their sideways blindness has no time to matter.
        self._correction = (0.0, 0.0, 0.0)  # laser pose = correction o wheel pose
        self._key_points = None
        self._key_pose = (0.0, 0.0, 0.0)
        self._key_wheel = None
        self._wheel = None          # the wheels' own pose, as published
        self._dead = None           # what the transform is actually built on
        self._imu_yaw = None        # gyro, integrated
        self._imu_time = None
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
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                 self._on_imu, sensor_qos)
        self.publisher = self.create_publisher(Odometry, '/laser_odom', 10)
        self.get_logger().info(
            'laser odometry: matching %s; yaw from %s, translation from %s; '
            'publish_tf=%s' % (
                self.get_parameter('scan_topic').value,
                self.get_parameter('yaw_source').value,
                self.get_parameter('translation_source').value,
                self.get_parameter('publish_tf').value))

    def _on_imu(self, msg):
        """Integrate the yaw rate. Nothing else on the IMU is used.

        The accelerometers are left alone deliberately: turning acceleration
        into position needs a double integration and an attitude good enough to
        separate gravity from motion, and the laser already supplies position.
        The gyro is the one channel that measures something nothing else here
        measures at all.

        Bias is not estimated. Ignition gives this sensor 7.5e-6 rad/s of it,
        which is 0.02 deg per minute, and the laser re-seats the heading through
        the correction below on every accepted match anyway. On a real base the
        bias would be larger and that correction is what would absorb it.
        """
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._imu_time is None:
            self._imu_yaw, self._imu_time = 0.0, stamp
            return
        dt = stamp - self._imu_time
        self._imu_time = stamp
        if 0.0 < dt < 1.0:                      # a gap means messages were dropped
            self._imu_yaw += msg.angular_velocity.z * dt

        # Without the wheels there is nothing else ticking at this rate, so the
        # transform goes out from here: the translation stands still between
        # matches, the heading does not.
        if self.get_parameter('translation_source').value != 'wheels':
            heading = self._imu_yaw
            if self.get_parameter('yaw_source').value != 'imu':
                # The comparison has to stay possible in this mode too, and it
                # is the mode where heading matters most: nothing but the
                # heading moves the pose between two matches.
                if self._wheel is None:
                    return
                heading = self._wheel[2]
            if self._dead is None:
                self._dead = (0.0, 0.0, heading)
            else:
                self._dead = (self._dead[0], self._dead[1], heading)
            if self._base_from_laser is not None:
                self._publish(compose(self._correction, self._dead),
                              msg.header.stamp)

    def _on_wheel(self, msg):
        """Advance the dead reckoning and send the transform out.

        The wheels are asked for one thing only: how far the robot moved in its
        own frame since the last message. That they can do - over a fiftieth of
        a second the base has barely moved and there is nothing for the sideways
        blindness to accumulate into. What they must not be asked for is which
        way the robot is now pointing, because the heading is where their error
        lives, and every later displacement is rotated by it.

        So the step is taken from the wheels and rotated by the IMU's heading.
        PAL says the wheels may not own `odom -> base_footprint` on this base
        (`enable_odom_tf: false`); this is that rule applied to the part of the
        pose the measurement actually condemns.
        """
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        previous, self._wheel = self._wheel, (p.x, p.y, yaw_of(q))
        if self.get_parameter('translation_source').value != 'wheels':
            return                              # the IMU carries it instead

        by_imu = (self.get_parameter('yaw_source').value == 'imu'
                  and self._imu_yaw is not None)
        if not by_imu:
            self._dead = self._wheel
        elif previous is None or self._dead is None:
            self._dead = (self._wheel[0], self._wheel[1], self._imu_yaw)
        else:
            step = compose(invert(previous), self._wheel)       # in the body frame
            c, s = math.cos(self._dead[2]), math.sin(self._dead[2])
            self._dead = (self._dead[0] + step[0] * c - step[1] * s,
                          self._dead[1] + step[0] * s + step[1] * c,
                          self._imu_yaw)

        if self._base_from_laser is None:
            return
        # Published at the wheels' rate, carrying the laser's correction.
        self._publish(compose(self._correction, self._dead), msg.header.stamp)

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
            self._key_wheel = self._dead
            return

        # Seed from the dead reckoning: the wheels' step under the IMU's
        # heading. A bad measurement and a perfectly good starting point.
        guess = (0.0, 0.0, 0.0)
        if self._dead is not None and self._key_wheel is not None:
            a, b = self._key_wheel, self._dead
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
            self._key_wheel = self._dead

        # Re-seat the correction on this measurement. Nothing is published from
        # here: the next wheel message carries it out, at 50 Hz instead of 13.
        base = compose(self._pose, invert(offset))
        if self.get_parameter('translation_source').value != 'wheels':
            # The laser owns the translation, so the dead reckoning is re-seated
            # on it here; between matches only the heading moves it on.
            if self.get_parameter('yaw_source').value == 'imu':
                heading = self._imu_yaw if self._imu_yaw is not None else base[2]
            else:
                heading = self._wheel[2] if self._wheel is not None else base[2]
            self._dead = (base[0], base[1], heading)
        if self._dead is not None:
            self._correction = compose(base, invert(self._dead))

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
