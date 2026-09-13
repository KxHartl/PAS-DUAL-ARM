"""Drive both arms to a named posture and exit.

  ros2 run pas_dual_arm_scripts set_posture ARM_CARRY_V2

Needed before anything moves the base: after spawn every arm joint sits at 0 and
the robot is 2.28 m wide, so it cannot fit through the 1.0 m doorways.

Also reports how far the arms sit above the lidar plane. The 360 deg scan is at
z = 0.2086 m in base_footprint and the lowest geometry of ARM_CARRY_V2 is around
0.26 m, so the margin is small. An arm dipping into the scan plane would put a
phantom obstacle into every SLAM frame - exactly the failure the two decorative
SICK laser housings used to cause (see robot.urdf.xacro section 1).
"""

import subprocess
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from pas_dual_arm_scripts.postures import move_to_posture, posture_names

SCAN_PLANE_Z = 0.2086  # virtual_base_laser_link in base_footprint
# Links that reach lowest in the tucked posture.
WATCH_LINKS = (
    'left_forearm_link', 'left_spherical_wrist_2_link', 'left_bracelet_link',
    'right_forearm_link', 'right_spherical_wrist_2_link', 'right_bracelet_link',
)


class SetPosture(Node):
    def __init__(self):
        super().__init__('set_posture')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def report_scan_clearance(self):
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
        lowest = None
        for link in WATCH_LINKS:
            try:
                z = self.buf.lookup_transform(
                    'base_footprint', link, rclpy.time.Time()).transform.translation.z
            except Exception:
                continue
            if lowest is None or z < lowest[1]:
                lowest = (link, z)
        if lowest is None:
            self.get_logger().warn('could not read arm TFs; scan clearance unchecked')
            return
        link, z = lowest
        margin = z - SCAN_PLANE_Z
        msg = (f'lowest arm link {link} at z = {z:.3f} m, '
               f'lidar plane at {SCAN_PLANE_Z:.4f} m, margin {margin:+.3f} m')
        # Link origins are not the lowest point of the geometry, so a small
        # positive margin is a warning, not a pass.
        if margin < 0.10:
            self.get_logger().warn(f'{msg} - arms may enter the scan plane')
        else:
            self.get_logger().info(msg)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if not args:
        print(f'usage: set_posture <name>\nknown: {", ".join(posture_names())}',
              file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = SetPosture()
    try:
        # Ensure DetachableJoint is released so the box sits freely
        subprocess.run(
            ['ign', 'topic', '-t', '/aruco_box/detach',
             '-m', 'ignition.msgs.Empty', '-p', 'unused: true'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        ok = move_to_posture(node, args[0])
        if ok:
            node.report_scan_clearance()
    except KeyError as exc:
        node.get_logger().error(str(exc))
        ok = False
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
