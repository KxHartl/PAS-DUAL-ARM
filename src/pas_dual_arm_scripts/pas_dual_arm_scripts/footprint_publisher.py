"""Tell Nav2 the shape the robot actually has, continuously.

Both costmaps are configured with a static `footprint` polygon measured once for
one arm posture.  The arms move, and they sag while driving (P-37: 1.109 m
against the 0.854 m in the config), so every layer that reasons about space -
the global planner, the controller's footprint critics, inflation, and the
collision monitor - has been working from a robot that stopped existing the
moment the arms last moved.  That is the literal sense in which the stack "does
not know the space the robot occupies".

`Costmap2DROS` subscribes to a `geometry_msgs/Polygon` on its own `footprint`
topic and swaps the footprint at runtime, so this node publishes the convex
outline of the whole robot, from the URDF and live TF, to both costmaps.  One
change, and every one of those layers starts being right.

Deliberately: this node has **no authority**.  It publishes a description and
nothing else - it cannot stop, refuse, or gate anything.  If TF is not ready it
simply stays quiet and the costmaps keep the YAML footprint, which is the
correct posture's polygon and a sane fallback.

    ros2 topic echo /local_costmap/published_footprint
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point32, Polygon
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from pas_dual_arm_scripts.robot_extent import ExtentMeasurer


def polygon_changed(previous, current, tolerance):
    """Has the outline moved enough to be worth republishing?

    Compared by how far the outline reaches in each direction rather than vertex
    by vertex, because the hull can renumber its vertices without the shape
    moving at all. Republishing every tick would make the costmap re-rasterise
    the footprint continuously for no gain.
    """
    if previous is None or len(previous) != len(current):
        return True
    return float(np.abs(np.sort(np.hypot(previous[:, 0], previous[:, 1])) -
                        np.sort(np.hypot(current[:, 0], current[:, 1]))).max()) > tolerance


class FootprintPublisher(Node):
    def __init__(self):
        super().__init__('footprint_publisher')
        self.declare_parameter('rate', 5.0)
        # The costmaps' robot_base_frame: a footprint is expressed in it.
        self.declare_parameter('frame', 'base_link')
        self.declare_parameter('geometry', 'collision')
        # ObstacleFootprint rasterises this outline once per sampled trajectory,
        # and DWB samples 3711 of them per cycle: at 24 vertices the control loop
        # missed its 10 Hz rate 1184 times in one run, which Nav2 read as a stuck
        # robot and answered with a Spin recovery. Eight directions keep the
        # extent exact where it matters - they include +-x and +-y, so length and
        # width are unchanged - and only chamfer the corners outward.
        self.declare_parameter('max_vertices', 8)
        # Even when the shape has not moved, say so now and then, so a costmap
        # that subscribes late is not left with the configured polygon.
        self.declare_parameter('resend_period', 2.0)
        # Every costmap rasterises the polygon on receipt, so republish only
        # when the shape has really moved.
        self.declare_parameter('change_tolerance', 0.01)
        self.declare_parameter('costmaps', ['local_costmap', 'global_costmap'])
        # Safety net against a bad TF or a mesh this code misreads: a polygon
        # reaching further than this is not published, and the costmap keeps
        # whatever it had. A wrong footprint is worse than a stale one.
        self.declare_parameter('max_reach', 1.50)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Latched, and resent periodically. The costmaps subscribe when they
        # configure, which is after this node starts and again on every restart
        # of the Nav2 stack; a volatile publisher that only speaks on change
        # leaves them on the YAML footprint until the arms happen to move. That
        # is exactly what was seen: the outline only appeared after a posture
        # change.
        self._costmap_pubs = {
            name: self.create_publisher(Polygon, f'/{name}/footprint', latched)
            for name in self.get_parameter('costmaps').value
        }
        # Also published latched under our own name, so RViz and any later node
        # can see the outline without subscribing to a costmap's input.
        self._own = self.create_publisher(Polygon, '/robot_footprint', latched)
        self.create_subscription(String, '/robot_description',
                                 self._on_description, latched)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)

        self._measurer = None
        self._last = None
        self._sent_at = None
        self._reported = None
        self.create_timer(1.0 / max(0.5, self.get_parameter('rate').value), self._tick)
        self.get_logger().info('waiting for /robot_description to build the footprint')

    def _on_description(self, msg):
        if self._measurer is not None:
            return
        try:
            self._measurer = ExtentMeasurer(
                msg.data,
                kind=self.get_parameter('geometry').value,
                frame=self.get_parameter('frame').value)
        except Exception as exc:
            self.get_logger().error(f'cannot read robot geometry: {exc}')
            return
        self.get_logger().info(
            f'{len(self._measurer.links)} links of {self._measurer.kind} geometry '
            f'in {self._measurer.frame}; publishing to '
            f'{", ".join(self._costmap_pubs)}')

    def _tick(self):
        if self._measurer is None:
            return
        polygon = self._measurer.footprint(
            self._tf, rclpy.time.Time(),
            max_vertices=self.get_parameter('max_vertices').value)
        if polygon is None or len(polygon) < 3:
            self.get_logger().warn('no link transforms yet; costmaps keep their '
                                   'configured footprint',
                                   throttle_duration_sec=5.0)
            return

        reach = float(np.hypot(polygon[:, 0], polygon[:, 1]).max())
        limit = self.get_parameter('max_reach').value
        if reach > limit:
            self.get_logger().error(
                f'computed a footprint reaching {reach:.2f} m, over the '
                f'{limit:.2f} m sanity limit; not publishing it',
                throttle_duration_sec=5.0)
            return

        moved = polygon_changed(self._last, polygon,
                                self.get_parameter('change_tolerance').value)
        due = (self._sent_at is None or
               time.monotonic() - self._sent_at >
               self.get_parameter('resend_period').value)
        if not moved and not due:
            return
        self._last = polygon
        self._sent_at = time.monotonic()

        message = Polygon()
        message.points = [Point32(x=float(x), y=float(y), z=0.0)
                          for x, y in polygon]
        for publisher in self._costmap_pubs.values():
            publisher.publish(message)
        self._own.publish(message)

        width = float(polygon[:, 1].max() - polygon[:, 1].min())
        length = float(polygon[:, 0].max() - polygon[:, 0].min())
        if self._reported is None or abs(width - self._reported) > 0.02:
            self._reported = width
            self.get_logger().info(
                f'footprint now {length:.3f} m long, {width:.3f} m wide, '
                f'{len(polygon)} vertices')


def main():
    rclpy.init()
    node = FootprintPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
