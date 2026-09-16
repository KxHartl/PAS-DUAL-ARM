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
import time
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
    'waiting': '#b7791f', 'running': '#2b6cb0', 'success': '#276749',
}


class NavGuiNode(Node):
    def __init__(self):
        super().__init__('nav_gui')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goto = self.create_publisher(String, '/room_navigator/goto', 10)
        # The mission node waits for this before anything drives; the room name
        # tells it where the cube is. Separate from /room_navigator/goto on
        # purpose: that one drives the base right now, this one hands the whole
        # run over to main_task, which drives through the navigator itself.
        self.mission = self.create_publisher(String, '/mission/start', 10)
        self.status = {'state': 'idle', 'detail': 'waiting for the navigator'}
        self.task_status = {
            'step': 0, 'total_steps': 8, 'phase': 'Pripravan',
            'detail': 'Čeka se pokretanje misije.', 'state': 'idle'
        }
        self.task_history = []
        self.graph = None
        self.scan = None
        self.create_subscription(String, '/room_navigator/status', self._on_status, latched)
        self.create_subscription(String, '/mission/task_status', self._on_task_status, latched)
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

    def _on_task_status(self, msg):
        try:
            data = json.loads(msg.data)
            self.task_status = data
            t_str = time.strftime('%H:%M:%S')
            step = data.get('step', 0)
            total = data.get('total_steps', 8)
            phase = data.get('phase', '')
            state = data.get('state', 'running')
            detail = data.get('detail', '')
            step_tag = f"[{step}/{total}] " if (step and total) else ""
            log_line = f"[{t_str}] {step_tag}{phase} - {detail}\n"
            self.task_history.append((log_line, state))
            if len(self.task_history) > 100:
                self.task_history.pop(0)
        except Exception:
            pass

    def _on_graph(self, msg):
        self.graph = json.loads(msg.data)

    def _on_scan(self, msg):
        self.scan = msg

    def send(self, room):
        self.goto.publish(String(data=room))
        self.get_logger().info(f'requested: {room}')

    def start_mission(self, room):
        self.mission.publish(String(data=room))
        self.get_logger().info(f'mission requested: go for the cube in {room}')

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
        self.root.title('PAS dual arm - Upravljanje i status misije')
        self.root.geometry('620x720')
        pad = {'padx': 8, 'pady': 3}

        frame = ttk.Frame(self.root, padding=12)
        frame.pack(fill='both', expand=True)

        ttk.Label(frame, text='Pošalji robota', font=('TkDefaultFont', 11, 'bold')) \
            .grid(row=0, column=0, columnspan=3, sticky='w', **pad)

        self.buttons = {}
        for column, (room, text) in enumerate((('blue', 'PLAVA soba\n(pred stol)'),
                                               ('red', 'CRVENA soba\n(pred stol)'),
                                               ('home', 'HOME\n(sredina sobe)'))):
            button = tk.Button(frame, text=text, height=2, width=16,
                               command=lambda r=room: self.node.send(r))
            button.grid(row=1, column=column, **pad)
            self.buttons[room] = button

        # Hands the whole run to the mission node: it drives to the cube itself,
        # picks it up, carries it to the red room and places it. The buttons
        # above stay what they are - plain drives, nothing else.
        mission = tk.Button(frame, text='MISIJA: po kutiju', height=2,
                            bg='#276749', fg='white',
                            font=('TkDefaultFont', 11, 'bold'),
                            command=lambda: self.node.start_mission('blue'))
        mission.grid(row=2, column=0, columnspan=3, sticky='ew', **pad)

        stop = tk.Button(frame, text='STOP', height=1, bg='#c53030', fg='white',
                         font=('TkDefaultFont', 10, 'bold'),
                         command=lambda: self.node.send('stop'))
        stop.grid(row=3, column=0, columnspan=3, sticky='ew', **pad)

        self.state = tk.Label(frame, text='idle', font=('TkDefaultFont', 10, 'bold'),
                              fg='white', bg=COLOURS['idle'], anchor='w', padx=10, pady=4)
        self.state.grid(row=4, column=0, columnspan=3, sticky='ew', **pad)

        self.detail = ttk.Label(frame, text='', wraplength=580, justify='left')
        self.detail.grid(row=5, column=0, columnspan=3, sticky='w', **pad)

        # --- Task / Mission section ---
        ttk.Separator(frame, orient='horizontal').grid(row=6, column=0, columnspan=3,
                                                       sticky='ew', pady=5)
        ttk.Label(frame, text='Trenutni zadatak / Misija', font=('TkDefaultFont', 11, 'bold')) \
            .grid(row=7, column=0, columnspan=3, sticky='w', **pad)

        self.task_phase = tk.Label(frame, text='ČEKA POKRETANJE', font=('TkDefaultFont', 10, 'bold'),
                                   fg='white', bg=COLOURS['idle'], anchor='w', padx=10, pady=4)
        self.task_phase.grid(row=8, column=0, columnspan=3, sticky='ew', **pad)

        self.task_detail = ttk.Label(frame, text='Pripravan za rad.', wraplength=580, justify='left')
        self.task_detail.grid(row=9, column=0, columnspan=3, sticky='w', **pad)

        # Mini console / log
        ttk.Label(frame, text='Dnevnik koraka misije:', font=('TkDefaultFont', 9, 'italic')) \
            .grid(row=10, column=0, columnspan=3, sticky='w', padx=8, pady=(3, 0))

        log_frame = ttk.Frame(frame)
        log_frame.grid(row=11, column=0, columnspan=3, sticky='nsew', padx=8, pady=2)
        self.task_log_text = tk.Text(log_frame, height=5, bg='#1a202c', fg='#e2e8f0',
                                     font=('TkFixedFont', 8), wrap='word', state='disabled')
        log_scroll = ttk.Scrollbar(log_frame, orient='vertical', command=self.task_log_text.yview)
        self.task_log_text.configure(yscrollcommand=log_scroll.set)
        self.task_log_text.pack(side='left', fill='both', expand=True)
        log_scroll.pack(side='right', fill='y')

        self.task_log_text.tag_config('success', foreground='#68d391')
        self.task_log_text.tag_config('aborted', foreground='#fc8181')
        self.task_log_text.tag_config('waiting', foreground='#f6e05e')
        self.task_log_text.tag_config('running', foreground='#63b3ed')
        self.task_log_text.tag_config('idle', foreground='#a0aec0')
        self._rendered_history_count = 0

        # --- Navigation / Readout section ---
        ttk.Separator(frame, orient='horizontal').grid(row=12, column=0, columnspan=3,
                                                       sticky='ew', pady=5)
        self.readout = ttk.Label(frame, text='', justify='left',
                                 font=('TkFixedFont', 9))
        self.readout.grid(row=13, column=0, columnspan=3, sticky='w', **pad)

        for column in range(3):
            frame.columnconfigure(column, weight=1)
        self.refresh()

    def refresh(self):
        status = self.node.status
        state = status.get('state', 'idle')
        self.state.configure(text=f"NAV: {state.upper()}", bg=COLOURS.get(state, '#4a5568'))
        self.detail.configure(text=status.get('detail', ''))

        # Update mission task status
        task = self.node.task_status
        task_state = task.get('state', 'idle')
        step = task.get('step', 0)
        total = task.get('total_steps', 8)
        step_prefix = f"[{step}/{total}] " if (step and total) else ""
        phase_text = f"{step_prefix}{task.get('phase', '')}".strip() or "ČEKA POKRETANJE"
        self.task_phase.configure(
            text=phase_text,
            bg=COLOURS.get(task_state, '#4a5568')
        )
        self.task_detail.configure(text=task.get('detail', ''))

        # Append new history lines to log text widget
        if len(self.node.task_history) > self._rendered_history_count:
            self.task_log_text.configure(state='normal')
            for line, l_state in self.node.task_history[self._rendered_history_count:]:
                tag = l_state if l_state in ('success', 'aborted', 'waiting', 'running') else 'idle'
                self.task_log_text.insert(tk.END, line, tag)
            self.task_log_text.see(tk.END)
            self.task_log_text.configure(state='disabled')
            self._rendered_history_count = len(self.node.task_history)

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
