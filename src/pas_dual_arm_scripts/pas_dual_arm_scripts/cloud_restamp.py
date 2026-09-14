"""Republish the depth cloud under the frame its data is actually in.

The Ignition rgbd sensor stamps `camera_color_optical_frame` (z forward, x
right), which is what ArUco needs for the image, but the point cloud DATA comes
out in the sensor BODY convention (x forward, z up). `measure_box` already
works around this by transforming through `camera_link` and ignoring the stamp,
but RViz has no such override: it trusts the stamp, rotates the cloud by the
optical frame, and the whole thing lands about 90 degrees off to the side.

So this relay copies the cloud through with the frame corrected. Nothing
subscribes to it except RViz - the control path still measures from the raw
topic, so this node cannot change what the grasp is computed from.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


class CloudRestamp(Node):
    def __init__(self):
        super().__init__('cloud_restamp')
        self.declare_parameter('input', '/camera/points')
        self.declare_parameter('output', '/camera/points_body')
        self.declare_parameter('frame_id', 'camera_link')
        self.frame = self.get_parameter('frame_id').value
        source = self.get_parameter('input').value
        target = self.get_parameter('output').value
        self.pub = self.create_publisher(PointCloud2, target, SENSOR_QOS)
        self.create_subscription(PointCloud2, source, self.relay, SENSOR_QOS)
        self.get_logger().info(
            f'relaying {source} -> {target} as frame "{self.frame}" '
            '(display only)')

    def relay(self, msg):
        msg.header.frame_id = self.frame
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CloudRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
