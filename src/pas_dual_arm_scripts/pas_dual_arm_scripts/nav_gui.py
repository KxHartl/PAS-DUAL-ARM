#!/usr/bin/env python3
"""Button panel for the room navigator, plus a live view of what it is doing.

The navigation test is meant to be driven by hand: send the robot somewhere with
RViz's "2D Goal Pose", then press a button and watch it go to a room on its own.
Typing a `ros2 topic pub` line for the second half is a poor way to run a demo
and a worse way to hit STOP, so this is the panel.

Started by nav2.launch.py unless gui:=false; run it on its own with
`ros2 run pas_dual_arm_scripts nav_gui`.

Buttons publish on /room_navigator/goto; the readout is /room_navigator/status,
the zone graph from /nav_zones, the localised pose from TF and the side
clearance straight off /scan_filtered - the same number the navigator aborts on,
so what the robot is deciding is visible while it decides it.
"""

import json
import math
import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from pas_dual_arm_scripts.room_navigator import HALF_LENGTH, HALF_WIDTH

COLOURS = {
    'idle': '#4a5568', 'driving': '#2b6cb0', 'arrived': '#276749',
    'aborted': '#9b2c2c', 'failed': '#9b2c2c', 'cancelled': '#975a16',
}


class NavGuiNode(Node):
    def __init__(self):
        super().__init__('nav_gui')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goto = self.create_publisher(String, '/room_navigator/goto', 10)
        self.status = {'state': 'idle', 'detail': 'waiting for the navigator'}
        self.graph = None
        self.scan = None
        self.create_subscription(String, '/room_navigator/status', self._on_status, latched)
        self.create_subscription(String, '/nav_graph', self._on_graph, latched)
        self.create_subscription(LaserScan, '/scan_filtered', self._on_scan,
                                 qos_profile_sensor_data)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status = {'state': 'idle', 'detail': msg.data}

    def _on_graph(self, msg):
        self.graph = json.loads(msg.data)

    def _on_scan(self, msg):
        self.scan = msg

    def send(self, room):
        self.goto.publish(String(data=room))
        self.get_logger().info(f'requested: {room}')

    def pose(self):
        try:
            tf = self._tf.lookup_transform('map', 'base_footprint',
                                           rclpy.time.Time()).transform
        except Exception:
            return None
        q = tf.rotation
        return (tf.translation.x, tf.translation.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def clearance(self):
        """Gap beside the robot, left and right, in metres."""
        scan = self.scan
        if scan is None:
            return None
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        good = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        if not good.any():
            return None
        xs, ys = ranges[good] * np.cos(angles[good]), ranges[good] * np.sin(angles[good])
        beside = np.abs(xs) < HALF_LENGTH
        left, right = ys[beside & (ys > 0)], ys[beside & (ys < 0)]
        return ((left.min() if left.size else float('inf')) - HALF_WIDTH,
                (-right.max() if right.size else float('inf')) - HALF_WIDTH)


class NavGui:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('PAS dual arm - room navigation')
        self.root.geometry('560x430')
        pad = {'padx': 8, 'pady': 5}

        frame = ttk.Frame(self.root, padding=12)
        frame.pack(fill='both', expand=True)

        ttk.Label(frame, text='Pošalji robota', font=('TkDefaultFont', 12, 'bold')) \
            .grid(row=0, column=0, columnspan=3, sticky='w', **pad)

        self.buttons = {}
        for column, (room, text) in enumerate((('blue', 'PLAVA soba\n(pred stol)'),
                                               ('red', 'CRVENA soba\n(pred stol)'),
                                               ('home', 'HOME\n(sredina sobe)'))):
            button = tk.Button(frame, text=text, height=3, width=16,
                               command=lambda r=room: self.node.send(r))
            button.grid(row=1, column=column, **pad)
            self.buttons[room] = button

        stop = tk.Button(frame, text='STOP', height=2, bg='#c53030', fg='white',
                         font=('TkDefaultFont', 11, 'bold'),
                         command=lambda: self.node.send('stop'))
        stop.grid(row=2, column=0, columnspan=3, sticky='ew', **pad)

        self.state = tk.Label(frame, text='idle', font=('TkDefaultFont', 11, 'bold'),
                              fg='white', bg=COLOURS['idle'], anchor='w', padx=10, pady=6)
        self.state.grid(row=3, column=0, columnspan=3, sticky='ew', **pad)

        self.detail = ttk.Label(frame, text='', wraplength=510, justify='left')
        self.detail.grid(row=4, column=0, columnspan=3, sticky='w', **pad)

        ttk.Separator(frame, orient='horizontal').grid(row=5, column=0, columnspan=3,
                                                       sticky='ew', pady=6)
        self.readout = ttk.Label(frame, text='', justify='left',
                                 font=('TkFixedFont', 9))
        self.readout.grid(row=6, column=0, columnspan=3, sticky='w', **pad)

        for column in range(3):
            frame.columnconfigure(column, weight=1)
        self.refresh()

    def refresh(self):
        status = self.node.status
        state = status.get('state', 'idle')
        self.state.configure(text=state.upper(), bg=COLOURS.get(state, '#4a5568'))
        self.detail.configure(text=status.get('detail', ''))

        lines = []
        pose = self.node.pose()
        lines.append(f'pose      : ({pose[0]:+.2f}, {pose[1]:+.2f}) m, '
                     f'{math.degrees(pose[2]):+7.1f} deg' if pose
                     else 'pose      : no map -> base_footprint (AMCL not localised?)')

        clearance = self.node.clearance()
        if clearance is None:
            lines.append('clearance : no /scan_filtered')
        else:
            left, right = clearance
            lines.append(f'clearance : left {_cm(left)}  right {_cm(right)}')

        graph = self.node.graph
        if graph is None:
            lines.append('zones     : waiting for /nav_graph')
        else:
            rooms = ', '.join(room['name'] for room in graph['rooms'])
            lines.append(f'rooms     : {rooms}')
            for door in graph['doors']:
                cx, cy = door['centre']
                lines.append(f'door      : ({cx:+.2f}, {cy:+.2f}) '
                             f'{door["width"]:.2f} m wide, normal {door["normal"]}, '
                             f'joins {"-".join(str(r) for r in door["rooms"])}')
            for warning in graph.get('warnings', []):
                lines.append(f'WARN      : {warning}')

        busy = state == 'driving'
        for button in self.buttons.values():
            button.configure(state='disabled' if busy else 'normal')

        self.readout.configure(text='\n'.join(lines))
        self.root.after(200, self.refresh)


def _cm(value):
    return '  --  ' if not math.isfinite(value) else f'{value * 100:6.1f} cm'


def main():
    rclpy.init()
    node = NavGuiNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    gui = NavGui(node)
    try:
        gui.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
