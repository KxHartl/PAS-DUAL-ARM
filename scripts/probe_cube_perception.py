#!/usr/bin/env python3
"""Read marker and depth estimates without commanding any robot motion."""

import rclpy

from pas_dual_arm_scripts.main_task import MainTask


def main():
    rclpy.init()
    node = MainTask()
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.2)
        marker = node.confirm_box(timeout=20.0, samples=5)
        if marker is None:
            raise SystemExit('No fresh ArUco marker')
        depth = node.measure_box(marker, timeout=12.0)
        if depth is None:
            raise SystemExit('Depth box measurement rejected')
        center, length, height, axis = depth
        print(f'MARKER_CENTER {marker.x:.4f} {marker.y:.4f} {marker.z:.4f}', flush=True)
        print(f'DEPTH_CENTER {center.x:.4f} {center.y:.4f} {center.z:.4f}', flush=True)
        print(f'EXTENT {length:.4f} {height:.4f} AXIS {axis}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
