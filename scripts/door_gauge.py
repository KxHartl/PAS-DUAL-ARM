#!/usr/bin/env python3
"""Show a doorway of a given width around the robot in RViz.

Publishes two wall slabs leaving a centred gap, so a posture can be checked
against a real opening by eye. The walls live in base_footprint, i.e. they move
with the robot and stay centred on it.

    ./scripts/run_native.sh python3 scripts/door_gauge.py            # 1.00 m
    ./scripts/run_native.sh python3 scripts/door_gauge.py --sirina 0.9

In RViz add a MarkerArray display on /door_gauge (the checked-in display.rviz
already has one).
"""
import argparse

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

FRAME = 'base_footprint'
WALL_T = 0.10      # wall thickness (X)
WALL_W = 1.20      # how far each slab extends outward (Y)
WALL_H = 2.10      # wall height (Z)


def slab(mid, name, x, y, sx, sy, sz, rgba):
    m = Marker()
    m.header.frame_id = FRAME
    m.ns = 'door_gauge'
    m.id = mid
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = sz / 2.0
    m.pose.orientation.w = 1.0
    m.scale = Vector3(x=sx, y=sy, z=sz)
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    m.lifetime = Duration(sec=0)
    m.text = name
    return m


def build(width, x_offset):
    half = width / 2.0
    left = slab(0, 'left', x_offset, half + WALL_W / 2.0,
                WALL_T, WALL_W, WALL_H, (0.85, 0.25, 0.25, 0.55))
    right = slab(1, 'right', x_offset, -(half + WALL_W / 2.0),
                 WALL_T, WALL_W, WALL_H, (0.85, 0.25, 0.25, 0.55))

    label = Marker()
    label.header.frame_id = FRAME
    label.ns = 'door_gauge'
    label.id = 2
    label.type = Marker.TEXT_VIEW_FACING
    label.action = Marker.ADD
    label.pose.position.x = x_offset
    label.pose.position.z = WALL_H + 0.15
    label.pose.orientation.w = 1.0
    label.scale.z = 0.16
    label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
    label.text = f'otvor {width * 100:.0f} cm'
    return MarkerArray(markers=[left, right, label])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sirina', type=float, default=1.00,
                    help='sirina otvora u metrima (zadano 1.00)')
    ap.add_argument('--x', type=float, default=0.0,
                    help='pomak zida po X u odnosu na robota (zadano 0 = oko robota)')
    args = ap.parse_args()

    rclpy.init()
    node = Node('door_gauge')
    qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, '/door_gauge', qos)
    msg = build(args.sirina, args.x)
    node.create_timer(1.0, lambda: pub.publish(msg))
    pub.publish(msg)
    node.get_logger().info(
        f'otvor {args.sirina * 100:.0f} cm centriran oko robota '
        f'(unutarnji rubovi na y = +/-{args.sirina / 2:.3f} m), topic /door_gauge')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
