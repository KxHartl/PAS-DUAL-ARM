"""Open-loop base motion with sim-time pacing and odometry read-back.

Lifted verbatim out of main_task.py so the mapping tour, doorway transits and the
task all drive the base through one implementation. The behaviour is unchanged;
what it encodes is three findings that cost real debugging time:

  * `spin_once()` is not a pause (P-21). It returns as soon as it services any
    callback and /clock ticks at ~1 kHz, so every "wait N seconds" loop written
    that way finished in milliseconds. Motion is paced by SIM time against a
    deadline, with a wall-clock net so a paused simulator cannot hang the caller.
  * Commanded seconds only integrate into distance at the simulator's rate. With
    the GUI running at RTF ~0.5, a 0.48 m wall-paced drive covered 0.24 m.
  * A constant-velocity step jerks the base. Ramps in and out.

Invariant that belongs with this code (P-10): the skid-steer base turns by
sliding its wheels, so **turning and driving never happen at the same time**.
Call drive() with either `lin` or `ang`, never both.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

CMD_VEL_TOPIC = '/base_controller/cmd_vel_unstamped'
# Read from the TRANSFORM, not from the wheel controller's topic.
#
# Until 18. 9. this read /base_controller/odom directly, and that quietly
# defeated its own experiment: `laser_odometry:=true` hands odom ->
# base_footprint to the scan matcher and stops the wheels publishing it, but
# everything here - the drive primitives, _to_odom, _marker_in_base - went on
# reading the wheels regardless. A ten-run series was driven on the laser and
# placed the cube 21.0 mm from the marker, exactly as before, because the part
# being measured never saw the laser at all.
#
# Through TF it inherits whatever owns the transform, which is the point: the
# wheels today, the laser and the gyro when they are switched on.
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'


class BaseDriver:
    """Mixin for a rclpy Node. Call init_base_drive() from the node's __init__."""

    def init_base_drive(self):
        # The base controller lives inside the gz controller_manager, so a plain
        # topic remap cannot reach it; publish to its own topic directly. Nav2's
        # /cmd_vel reaches the same place through cmd_vel_relay.
        self.cmd_vel = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self._odom_stamp = None

    # ------------------------------------------------------------------ odometry
    def _drive_buffer(self):
        """The transform buffer to read poses from.

        A node that already keeps one - main_task does - lends it, so there is
        one listener per node rather than two filling the same history.
        """
        buffer = getattr(self, 'tf_buffer', None) or getattr(self, '_drive_tf', None)
        if buffer is None:
            buffer = Buffer()
            self._drive_tf = buffer
            self._drive_tf_listener = TransformListener(buffer, self, spin_thread=True)
        return buffer

    def _odom_pose(self, timeout=2.0):
        """A FRESH odom -> base_footprint pose as (x, y, yaw), or None.

        Fresh means newer than the last one this returned. The old version
        cleared its cached message and waited for the next; a transform is
        always available, so what is waited for here is a newer stamp.
        """
        buffer = self._drive_buffer()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                tf = buffer.lookup_transform(ODOM_FRAME, BASE_FRAME,
                                             rclpy.time.Time())
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if self._odom_stamp is None or stamp > self._odom_stamp:
                self._odom_stamp = stamp
                t, q = tf.transform.translation, tf.transform.rotation
                return (t.x, t.y,
                        math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def _odom_latest(self):
        """The current odom -> base_footprint pose, without waiting, or None.

        For callers that are tracking a pose continuously and would rather have
        the last one than block for a new one.
        """
        try:
            tf = self._drive_buffer().lookup_transform(
                ODOM_FRAME, BASE_FRAME, rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _odom_xy(self, timeout=2.0):
        """A FRESH odometry XY sample (odom frame), or None."""
        pose = self._odom_pose(timeout)
        return None if pose is None else (pose[0], pose[1])

    def _odom_yaw(self, timeout=2.0):
        """A FRESH odometry yaw sample (odom frame), or None."""
        pose = self._odom_pose(timeout)
        return None if pose is None else pose[2]

    # ---------------------------------------------------------------- commanding
    def _send_vel(self, lin, ang, lat=0.0):
        tw = Twist()
        tw.linear.x = float(lin)
        tw.linear.y = float(lat)      # mecanum: sideways without turning
        tw.angular.z = float(ang)
        self.cmd_vel.publish(tw)

    def drive(self, lin, ang, secs, rate=20.0, lat=0.0):
        """Drive the base for a duration with a trapezoidal velocity profile
        (ramp up, cruise, ramp down). A constant-velocity step jerks the base and,
        when the box is rigidly attached to the arm, that impulse makes the attach
        joint's constraint solver explode and flings the box; smooth ramps avoid
        it (the way Nav2's smoothed velocities did when carry worked before)."""
        n = max(1, int(secs * rate))
        # ~0.8 s ease in / ease out, but never more than a third of the move:
        # on a short correction an 0.8 s ramp is the whole profile, so the base
        # either crawls or - with the ramp padding the duration - sails past the
        # target. A 2 cm strafe has to be a 2 cm strafe (user, 16. 9.).
        ramp = max(1, min(int(0.8 * rate), n // 3))
        period = 1.0 / rate
        # Pace the ticks by SIM time, not wall time: `secs` of commanded motion
        # only integrates `secs` of sim time worth of distance, and the GUI sim
        # runs below real-time (measured RTF ~0.5: commanded 0.48 m, driven
        # 0.24 m with wall pacing). spin_once() alone is no pacing at all - it
        # returns per serviced callback and /clock runs at ~1 kHz. The wall
        # deadline is a safety net so a paused sim cannot hang us forever.
        t0_ns = self.get_clock().now().nanoseconds
        wall_deadline = time.monotonic() + 4.0 * secs + 5.0
        for i in range(n):
            if i < ramp:
                s = (i + 1) / ramp
            elif i >= n - ramp:
                s = max(0.0, (n - i) / ramp)
            else:
                s = 1.0
            self._send_vel(lin * s, ang * s, lat * s)
            target_ns = t0_ns + int((i + 1) * period * 1e9)
            while (self.get_clock().now().nanoseconds < target_ns
                   and time.monotonic() < wall_deadline):
                rclpy.spin_once(self, timeout_sec=0.02)
        self._send_vel(0.0, 0.0)

    # --------------------------------------------------------- measured motion
    def drive_distance(self, metres, speed=0.15):
        """Drive straight until odometry reaches the requested displacement.

        Abort on yaw drift or lack of progress.  This is a low-level mapping
        primitive, not a substitute for Nav2 collision avoidance.
        """
        start = self._odom_xy()
        start_yaw = self._odom_yaw()
        if start is None or start_yaw is None:
            return None
        previous = 0.0
        for _ in range(24):
            end = self._odom_xy()
            yaw = self._odom_yaw()
            if end is None or yaw is None:
                return None
            moved = math.hypot(end[0] - start[0], end[1] - start[1])
            yaw_error = math.atan2(math.sin(yaw - start_yaw),
                                   math.cos(yaw - start_yaw))
            if abs(yaw_error) > 0.08:
                self.get_logger().error(f'straight drive yaw drifted {yaw_error:.3f} rad')
                return None
            if moved >= abs(metres) - 0.01:
                return moved
            if _ > 0 and moved < previous + 0.004:
                self.get_logger().error('straight drive made no odometry progress')
                return None
            previous = moved
            remaining = abs(metres) - moved
            # Ease off as the target approaches: at full speed the shortest
            # possible burst still covers several centimetres, which turns a
            # small correction into an overshoot.
            step = min(speed, max(0.02, remaining / 1.5))
            self.drive(math.copysign(step, metres), 0.0,
                       max(0.4, min(2.0, remaining / step + 0.4)))
        self.get_logger().error('straight drive failed to reach requested distance')
        return None

    def strafe_distance(self, metres, speed=0.08):
        """Shift the base SIDEWAYS by an odometry-measured distance.

        The base is mecanum, so lining the cube up between the two arms does not
        need a turn: turning changes the approach heading the whole grasp
        geometry was measured in, and it is what made the robot visibly swing
        around before a pick. Strafing keeps the heading and just slides across.
        Positive is to the robot's left (+Y in base_footprint).
        """
        start = self._odom_xy()
        start_yaw = self._odom_yaw()
        if start is None or start_yaw is None:
            return None
        previous = 0.0
        for attempt in range(24):
            here = self._odom_xy()
            yaw = self._odom_yaw()
            if here is None or yaw is None:
                return None
            moved = math.hypot(here[0] - start[0], here[1] - start[1])
            yaw_error = math.atan2(math.sin(yaw - start_yaw),
                                   math.cos(yaw - start_yaw))
            if abs(yaw_error) > 0.08:
                self.get_logger().error(f'strafe yaw drifted {yaw_error:.3f} rad')
                return None
            if moved >= abs(metres) - 0.008:
                return moved
            if attempt > 0 and moved < previous + 0.003:
                self.get_logger().error('strafe made no odometry progress')
                return None
            previous = moved
            remaining = abs(metres) - moved
            # Same as the straight drive: the last centimetres are taken slowly,
            # or a 2 cm correction lands 4 cm across (user, 16. 9.).
            step = min(speed, max(0.02, remaining / 1.5))
            self.drive(0.0, 0.0, max(0.4, min(2.0, remaining / step + 0.4)),
                       lat=math.copysign(step, metres))
        self.get_logger().error('strafe failed to reach requested distance')
        return None

    def turn_angle(self, radians, rate=0.25):
        """Turn to an odometry-measured yaw, in bounded pulses.

        Skid-steer wheel slip means a timed 90 degree command can achieve only
        ~29 degrees.  Never report that as a completed doorway alignment.
        """
        start = self._odom_yaw()
        if start is None:
            return None
        target = start + radians
        last_error = abs(radians)
        for _ in range(24):
            current = self._odom_yaw()
            if current is None:
                return None
            error = math.atan2(math.sin(target - current),
                               math.cos(target - current))
            if abs(error) < 0.025:
                return math.atan2(math.sin(current - start),
                                  math.cos(current - start))
            if abs(error) > last_error + 0.08:
                self.get_logger().error('turn diverged from requested yaw')
                return None
            last_error = abs(error)
            pulse = max(0.5, min(2.0, abs(error) / rate + 0.8))
            self.drive(0.0, math.copysign(rate, error), pulse)
        self.get_logger().error('turn could not reach requested yaw within 24 pulses')
        return None
