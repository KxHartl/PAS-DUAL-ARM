#!/usr/bin/env python3
"""Do the wrist cameras actually see the marker on the face they will press?

Read-only: nothing is commanded. Run it with the hands at their pre-grasp poses.
For each wrist it reports whether images arrive, whether the marker decodes, and
where the marker centre sits - both in the camera frame and in `base_link`, so
the number can be compared against the cube centre the head camera measured.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
SIDES = ('left', 'right')


def _rotate(q, v):
    """Rotate v by quaternion q (geometry_msgs order)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    # t = 2 * (q_vec x v); v' = v + w*t + q_vec x t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


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
            # Where this camera is and where it points, so a failure to
            # decode can be told apart from a failure to aim. The mount frame
            # is the one Gazebo aims along: +X is the view, +Z is image up.
            try:
                cam_tf = node.buffer.lookup_transform(
                    'base_link', camera, rclpy.time.Time())
                p = cam_tf.transform.translation
                q = cam_tf.transform.rotation
                view = _rotate(q, (1.0, 0.0, 0.0))
                print(f'  kamera: ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) m, '
                      f'gleda ({view[0]:+.2f}, {view[1]:+.2f}, '
                      f'{view[2]:+.2f})', flush=True)
            except Exception as exc:
                print(f'  kamera: nema TF ({type(exc).__name__})', flush=True)

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
                extra = ''
                if parent == camera:
                    # In the mount frame x is the view axis, so the angle off
                    # the optical centre is the angle between x and the ray.
                    forward = t.x
                    sideways = math.hypot(t.y, t.z)
                    if forward > 1e-6:
                        angle = math.degrees(math.atan2(sideways, forward))
                        extra = (f'  [udaljenost {forward:.3f} m, '
                                 f'{angle:.1f} deg od sredine slike]')
                print(f'  marker {label}: ({t.x:+.4f}, {t.y:+.4f}, '
                      f'{t.z:+.4f}) m{extra}', flush=True)
            if not seen:
                print('  marker se ne dekodira. Pogledaj prikaz '
                      f'"Slika - {"lijeva" if side == "left" else "desna"} '
                      'ruka" u RViz-u: ako je marker vidljiv, problem je u '
                      'dekodiranju; ako nije, u nisanu.', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
