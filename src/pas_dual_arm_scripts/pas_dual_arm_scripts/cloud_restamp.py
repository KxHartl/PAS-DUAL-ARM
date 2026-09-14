"""Republish depth clouds under the frame their data is actually in.

Every Ignition rgbd sensor on this robot stamps an OPTICAL frame (z along the
view), which is what the ArUco detectors need for the image, while the point
cloud DATA comes out in the sensor BODY convention (x forward, z up).
`measure_box` already works around this for the head camera by transforming
through `camera_link` and ignoring the stamp, but RViz has no such override: it
trusts the stamp, rotates the cloud, and the whole thing lands about 90 degrees
off to the side.

So this relay copies each cloud through with the frame corrected. Nothing
subscribes to the corrected topics except RViz - the control path still measures
from the raw ones, so this node cannot change what a grasp is computed from.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

# input topic, output topic, the body-convention frame the data is really in.
DEFAULT_INPUTS = ['/camera/points',
                  '/wrist_left/points',
                  '/wrist_right/points']
DEFAULT_OUTPUTS = ['/camera/points_body',
                   '/wrist_left/points_body',
                   '/wrist_right/points_body']
DEFAULT_FRAMES = ['camera_link',
                  'left_camera_color_frame',
                  'right_camera_color_frame']


class CloudRestamp(Node):
    def __init__(self):
        super().__init__('cloud_restamp')
        self.declare_parameter('inputs', DEFAULT_INPUTS)
        self.declare_parameter('outputs', DEFAULT_OUTPUTS)
        self.declare_parameter('frames', DEFAULT_FRAMES)
        inputs = list(self.get_parameter('inputs').value)
        outputs = list(self.get_parameter('outputs').value)
        frames = list(self.get_parameter('frames').value)
        if not len(inputs) == len(outputs) == len(frames):
            raise ValueError(
                'inputs, outputs and frames must have the same length')
        self._subs = []
        for source, target, frame in zip(inputs, outputs, frames):
            publisher = self.create_publisher(PointCloud2, target, SENSOR_QOS)
            self._subs.append(self.create_subscription(
                PointCloud2, source,
                lambda msg, pub=publisher, f=frame: self.relay(msg, pub, f),
                SENSOR_QOS))
            self.get_logger().info(
                f'relaying {source} -> {target} as frame "{frame}" '
                '(display only)')

    @staticmethod
    def relay(msg, publisher, frame):
        msg.header.frame_id = frame
        publisher.publish(msg)


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
