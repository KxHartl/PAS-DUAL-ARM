#!/usr/bin/env python3
"""Measure how far AMCL's pose is from where the robot actually is.

Every doorway abort so far has been argued about without a number: the navigator
reports the gap it measures with the lidar, AMCL reports a pose, and nobody knows
which one is wrong.  Run 59 finally made the disagreement computable - the lidar
read the 1.00 m doorway as 1.002 m and the robot 7.8 cm off its axis, while AMCL
believed it was centred - so this node measures that disagreement continuously
instead of once, by hand, off a screenshot.

Ground truth is a simulator privilege.  A physical robot has no `/debug/gz_dynamic_pose`,
so this node exists purely to tell us which knob helped:

  * it only ever prints and publishes `/debug/loc_error`, a JSON string;
  * nothing in the control chain subscribes to either topic;
  * it is started only by `sim.launch.py debug_truth:=true`, off by default.

The pose it compares against is TF `map -> base_footprint`, not `/amcl_pose`,
because that transform is what Nav2, the costmaps and `room_navigator` actually
steer on.

The first sample is logged separately as a calibration check: the robot spawns at
the world origin and AMCL is seeded there (`set_initial_pose`), so at t=0 the two
should agree to within a few millimetres.  A large constant offset at startup means
the Gazebo model frame is not `base_footprint`, or the map origin is not the world
origin - a frame bug to fix, not a localisation error to chase.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocError(Node):

    def __init__(self):
        super().__init__('loc_error')
        # The Gazebo model name, as spawned by sim.launch.py (-name dual_arm_robot).
        self.declare_parameter('model_name', 'dual_arm_robot')
        # Ground truth arrives at the simulator's rate; 2 Hz is plenty for reading.
        self.declare_parameter('report_period', 0.5)

        self._model = self.get_parameter('model_name').value
        self._truth = None          # (x, y, yaw) in the Gazebo world
        self._first = None          # first matched (truth, localised) pair
        self._peak = [0.0, 0.0, 0.0]
        self._leg_peak = [0.0, 0.0, 0.0]
        self._leg = None
        self._warned_missing = False

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self.create_subscription(TFMessage, '/debug/gz_dynamic_pose',
                                 self._on_truth, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/room_navigator/status',
                                 self._on_status, latched)
        self._pub = self.create_publisher(String, '/debug/loc_error', 10)
        self.create_timer(float(self.get_parameter('report_period').value), self._report)
        self.get_logger().info(
            f'measuring |TF map->base_footprint  -  Gazebo {self._model}|; '
            'diagnostics only, nothing steers on this')

    # ------------------------------------------------------------------ inputs
    def _on_truth(self, msg):
        """Pick the robot's own transform out of every pose Gazebo moved."""
        for tf in msg.transforms:
            child = tf.child_frame_id
            # Fortress names the model itself `<model>`, and its links
            # `<model>::<link>`; only the model pose is the base pose.
            if child == self._model:
                t = tf.transform
                self._truth = (t.translation.x, t.translation.y,
                               quat_to_yaw(t.rotation))
                return

    def _on_status(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        leg = status.get('detail')
        if leg == self._leg:
            return
        if self._leg is not None and any(self._leg_peak):
            self.get_logger().info(
                f'leg done ["{self._leg}"]: peak dx {self._leg_peak[0] * 100:+.1f} cm  '
                f'dy {self._leg_peak[1] * 100:+.1f} cm  '
                f'dyaw {math.degrees(self._leg_peak[2]):+.1f} deg')
        self._leg = leg
        self._leg_peak = [0.0, 0.0, 0.0]

    def localised(self):
        try:
            tf = self._tf.lookup_transform('map', 'base_footprint',
                                           rclpy.time.Time()).transform
        except Exception:
            return None
        return (tf.translation.x, tf.translation.y, quat_to_yaw(tf.rotation))

    # ----------------------------------------------------------------- reading
    def _report(self):
        truth = self._truth
        if truth is None:
            if not self._warned_missing:
                self.get_logger().warn(
                    'no ground truth yet on /debug/gz_dynamic_pose - is the sim up '
                    'with debug_truth:=true?')
                self._warned_missing = True
            return
        est = self.localised()
        if est is None:
            return
        self._warned_missing = False

        err = (est[0] - truth[0], est[1] - truth[1], wrap(est[2] - truth[2]))
        if self._first is None:
            self._first = err
            self.get_logger().info(
                f'first sample: localised ({est[0]:+.3f}, {est[1]:+.3f}, '
                f'{math.degrees(est[2]):+.1f} deg), actual ({truth[0]:+.3f}, '
                f'{truth[1]:+.3f}, {math.degrees(truth[2]):+.1f} deg) -> '
                f'offset {err[0] * 100:+.1f} / {err[1] * 100:+.1f} cm, '
                f'{math.degrees(err[2]):+.1f} deg')
            if max(abs(err[0]), abs(err[1])) > 0.03:
                self.get_logger().warn(
                    'that offset at startup is a frame or map-origin mismatch, not '
                    'drift - subtract it before reading anything below')

        for i, value in enumerate(err):
            if abs(value) > abs(self._peak[i]):
                self._peak[i] = value
            if abs(value) > abs(self._leg_peak[i]):
                self._leg_peak[i] = value

        self.get_logger().info(
            f'dx {err[0] * 100:+.1f} cm  dy {err[1] * 100:+.1f} cm  '
            f'dyaw {math.degrees(err[2]):+.1f} deg   '
            f'| peak {self._peak[0] * 100:+.1f} / {self._peak[1] * 100:+.1f} cm, '
            f'{math.degrees(self._peak[2]):+.1f} deg')
        msg = String()
        msg.data = json.dumps({
            'dx': err[0], 'dy': err[1], 'dyaw': err[2],
            'peak_dx': self._peak[0], 'peak_dy': self._peak[1], 'peak_dyaw': self._peak[2],
            'leg': self._leg,
            'localised': {'x': est[0], 'y': est[1], 'yaw': est[2]},
            'actual': {'x': truth[0], 'y': truth[1], 'yaw': truth[2]},
        })
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = LocError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Printed, not logged: on SIGTERM rclpy has already torn the context down,
        # so the logger drops exactly the line the whole run was for.
        print(f'loc_error final peak: dx {node._peak[0] * 100:+.1f} cm  '
              f'dy {node._peak[1] * 100:+.1f} cm  '
              f'dyaw {math.degrees(node._peak[2]):+.1f} deg', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
