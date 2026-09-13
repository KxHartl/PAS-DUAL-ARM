"""Remove laser returns from the robot's own body before SLAM and Nav2.

The carry posture can sag into the low laser plane while driving.  A return
inside the robot envelope cannot be a traversable external obstacle; leaving it
in a scan creates moving walls in slam_toolbox.  The raw /scan remains available
for diagnosis.  This is deliberately a small geometry filter because the Humble
laser_filters package is not installed in the project's ROS environment.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.declare_parameter('half_length', 0.75)
        self.declare_parameter('half_width', 0.47)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        # Reliable output also serves Nav2 subscriptions using the default QoS;
        # the raw bridge input is best-effort sensor data.
        self._pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, scan):
        try:
            transform = self._tf.lookup_transform('base_footprint', scan.header.frame_id,
                                                  rclpy.time.Time()).transform
        except Exception as exc:
            self.get_logger().warn(f'no laser transform; dropping scan: {exc}',
                                   throttle_duration_sec=5.0)
            return
        q = transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)
        ox, oy = transform.translation.x, transform.translation.y
        length = self.get_parameter('half_length').value
        width = self.get_parameter('half_width').value
        result = LaserScan()
        result.header = scan.header
        result.angle_min = scan.angle_min
        result.angle_max = scan.angle_max
        result.angle_increment = scan.angle_increment
        result.time_increment = scan.time_increment
        result.scan_time = scan.scan_time
        result.range_min = scan.range_min
        result.range_max = scan.range_max
        result.intensities = scan.intensities
        result.ranges = list(scan.ranges)
        for i, distance in enumerate(result.ranges):
            if not math.isfinite(distance) or distance < scan.range_min:
                continue
            angle = scan.angle_min + i * scan.angle_increment + yaw
            x = ox + distance * math.cos(angle)
            y = oy + distance * math.sin(angle)
            if abs(x) <= length and abs(y) <= width:
                result.ranges[i] = float('inf')
        self._pub.publish(result)


def main():
    rclpy.init()
    node = ScanFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
