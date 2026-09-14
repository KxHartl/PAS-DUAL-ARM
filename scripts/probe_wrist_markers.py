#!/usr/bin/env python3
"""Do the wrist cameras actually see the marker on the face they will press?

Read-only: nothing is commanded. Run it with the hands at their pre-grasp poses.
For each wrist it reports whether images arrive, whether the marker decodes, and
where the marker centre sits - both in the camera frame and in `base_link`, so
the number can be compared against the cube centre the head camera measured.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
SIDES = ('left', 'right')


class WristProbe(Node):
    def __init__(self):
        super().__init__('wrist_marker_probe')
        self.frames = {side: 0 for side in SIDES}
        for side in SIDES:
            self.create_subscription(
                Image, f'/wrist_{side}/image',
                lambda _msg, s=side: self.frames.__setitem__(
                    s, self.frames[s] + 1),
                SENSOR_QOS)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=6.0,
                        help='how long to watch before reporting')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = WristProbe()
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        for side in SIDES:
            rate = node.frames[side] / args.seconds
            marker = f'{side}_grasp_marker_frame'
            camera = f'{side}_camera_color_frame'
            print(f'\n--- {side.upper()} zapesce', flush=True)
            print(f'  slika: {node.frames[side]} okvira '
                  f'({rate:.1f} Hz)', flush=True)
            if node.frames[side] == 0:
                print('  NEMA slike - senzor ili most nisu podignuti',
                      flush=True)
                continue
            seen = False
            for parent, label in ((camera, 'u kameri'), ('base_link', 'u base_link')):
                try:
                    tf = node.buffer.lookup_transform(
                        parent, marker, rclpy.time.Time())
                except Exception as exc:
                    print(f'  marker {label}: NE ({type(exc).__name__})',
                          flush=True)
                    continue
                seen = True
                t = tf.transform.translation
                print(f'  marker {label}: ({t.x:+.4f}, {t.y:+.4f}, '
                      f'{t.z:+.4f}) m', flush=True)
            if not seen:
                print('  marker se ne dekodira - provjeri je li ploha u '
                      'vidnom polju i koliko je udaljena', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
