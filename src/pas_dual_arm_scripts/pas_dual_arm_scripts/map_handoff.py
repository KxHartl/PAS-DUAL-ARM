#!/usr/bin/env python3
"""Hand a freshly built SLAM map over to localisation, without a rebuild.

The mapping scenarios build a map and then run the mission on it in the same
launch. That handover has three parts, and this node does the first two:

  1. save what slam_toolbox currently holds, to a **runtime** directory rather
     than into the source tree. Nav2 reads its default map out of
     ``install/.../share``, which is why saving into ``src/`` normally needs a
     rebuild before the map can be driven on. An absolute path handed to
     map_server skips that entirely.
  2. record where the robot is *now*, in the map frame, so AMCL can be seeded
     with it instead of the (0, 0, 0) that the checked-in map assumes. This is
     what makes the mission start from wherever mapping happened to end.
  3. exit. The launch file watches for that exit and only then shuts slam_toolbox
     down and brings up map_server + AMCL: two things must never publish
     ``map -> odom`` at once.

So a non-zero exit means "do not continue", and the scenario stops rather than
driving on a map that was never written.

Triggered by ``/mapping/finish`` (the panel button, or ``ros2 topic pub``).
"""
import os
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

DEFAULT_DIR = os.path.expanduser('~/.ros/pas_dual_arm')


class MapHandoff(Node):
    def __init__(self):
        super().__init__('map_handoff')
        self.declare_parameter('map_dir', DEFAULT_DIR)
        self.declare_parameter('map_name', 'live_map')
        # Long enough that a slow save is not mistaken for a failed one.
        self.declare_parameter('save_timeout', 60.0)
        # Mapping can legitimately take a while; this only bounds the wait for
        # the *user*, and 0 disables it.
        self.declare_parameter('finish_timeout', 0.0)

        self.map_dir = self.get_parameter('map_dir').value
        self.map_name = self.get_parameter('map_name').value
        self.save_timeout = float(self.get_parameter('save_timeout').value)
        self.finish_timeout = float(self.get_parameter('finish_timeout').value)
        os.makedirs(self.map_dir, exist_ok=True)

        self.stem = os.path.join(self.map_dir, self.map_name)
        self._asked = False

        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.create_subscription(Empty, '/mapping/finish', self._on_finish, 10)
        # A String on the same job, so the button and a plain `topic pub` of
        # either type both work.
        self.create_subscription(String, '/mapping/finish_str', self._on_finish, 10)
        self.state = self.create_publisher(String, '/mapping/handoff_status', 10)

        self.save_cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.ser_cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

        self.get_logger().info(
            f'MAPPING: drive the robot around. When the map looks complete, press '
            f'"MAPIRANJE GOTOVO" (or: ros2 topic pub --once /mapping/finish '
            f'std_msgs/Empty "{{}}"). The map will be written to {self.stem}.*')
        self._say('mapping',
                  'Mapiranje u tijeku. Pritisnite „MAPIRANJE GOTOVO“ kad je karta potpuna.')
        self._t0 = time.monotonic()

    def _say(self, state, detail):
        self.state.publish(String(data=f'{state}|{detail}'))

    def _on_finish(self, _msg):
        if self._asked:
            return
        self._asked = True

    def expired(self):
        return (self.finish_timeout > 0.0
                and (time.monotonic() - self._t0) > self.finish_timeout)

    # ------------------------------------------------------------------ steps
    def _call(self, client, request, what):
        if not client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(f'{what}: service never appeared; is slam_toolbox running?')
            return False
        future = client.call_async(request)
        deadline = time.monotonic() + self.save_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error(f'{what}: timed out after {self.save_timeout:.0f} s')
            return False
        return True

    def save_map(self):
        """Write <stem>.yaml/.pgm and <stem>.posegraph/.data."""
        self._say('saving', 'Spremanje karte…')
        req = SaveMap.Request()
        req.name = String(data=self.stem)
        if not self._call(self.save_cli, req, 'save_map'):
            return False
        # The pose graph is not needed to drive, but it is what lets this map be
        # extended later instead of re-mapped from scratch.
        self._call(self.ser_cli, SerializePoseGraph.Request(filename=self.stem),
                   'serialize_map')

        for _ in range(50):                      # the service returns before the write lands
            if os.path.exists(f'{self.stem}.yaml') and os.path.exists(f'{self.stem}.pgm'):
                return True
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.0)
        self.get_logger().error(f'save_map reported success but {self.stem}.yaml is not there')
        return False

    def save_pose(self):
        """Record map -> base_footprint so AMCL can be seeded where SLAM ended."""
        for _ in range(50):
            try:
                tf = self._tf.lookup_transform('map', 'base_footprint',
                                               rclpy.time.Time()).transform
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            pose = {'x': float(tf.translation.x), 'y': float(tf.translation.y),
                    'qz': float(tf.rotation.z), 'qw': float(tf.rotation.w)}
            with open(f'{self.stem}.pose.yaml', 'w') as handle:
                yaml.safe_dump(pose, handle)
            self.get_logger().info(
                f'Handing over at x={pose["x"]:.2f} y={pose["y"]:.2f} (map frame).')
            return True
        # Without a pose AMCL would start at the map default, which is almost
        # certainly not where the robot is. Better to stop than to drive blind.
        self.get_logger().error('map -> base_footprint never resolved; cannot seed AMCL')
        return False


def main():
    rclpy.init()
    node = MapHandoff()
    code = 0
    try:
        while rclpy.ok() and not node._asked:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.expired():
                node.get_logger().error('Timed out waiting for the mapping-finished signal.')
                node._say('failed', 'Isteklo vrijeme čekanja na kraj mapiranja.')
                code = 2
                break
        if code == 0 and rclpy.ok():
            node.get_logger().info('Mapping finished; saving the map.')
            if node.save_map() and node.save_pose():
                node._say('saved', f'Karta spremljena: {node.stem}.yaml')
                node.get_logger().info(
                    'Map saved. Switching from SLAM to localisation; the mission '
                    'starts once AMCL is up.')
            else:
                node._say('failed', 'Spremanje karte nije uspjelo — misija se ne pokreće.')
                code = 1
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
