#!/usr/bin/env python3
"""Joint slider GUI with typed values in DEGREES (replaces joint_state_publisher_gui).

The stock joint_state_publisher_gui only offers drag sliders in radians. This one
shows, for every movable joint, an editable box (degrees for revolute/continuous
joints, millimetres for prismatic ones) next to the slider, so a posture can be
dialled in exactly and read off in units that mean something.

    ./scripts/run_native.sh python3 scripts/joint_gui.py

Publishes sensor_msgs/JointState on /joint_states at 30 Hz, exactly like the
stock GUI, so robot_state_publisher and RViz need no changes. Run only ONE of
the two GUIs at a time.
"""
import argparse
import ast
import math
import os
import re
import subprocess
import tkinter as tk
import xml.etree.ElementTree as ET
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

MOVABLE = ('revolute', 'continuous', 'prismatic')
CONT_LIMIT = math.pi          # range shown for continuous joints
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, 'src', 'pas_dual_arm_bringup', 'urdf', 'robot.urdf.xacro')
REGISTRY = os.path.join(ROOT, 'notes', '08_poze.md')


def load_posture(name):
    """Read NAME_LEFT / NAME_RIGHT joint dicts out of the posture registry."""
    if not os.path.exists(REGISTRY):
        return {}
    text = open(REGISTRY).read()
    out = {}
    for side in ('LEFT', 'RIGHT'):
        m = re.search(rf'^{re.escape(name)}_{side}\s*=\s*(\{{[^}}]*\}})',
                      text, re.M)
        if not m:
            continue
        for j, v in ast.literal_eval(m.group(1)).items():
            out[f'{side.lower()}_joint_{j}'] = float(v)
    return out

# Joints are grouped so the window reads like the robot, not like an XML dump.
GROUPS = [
    ('Lijeva ruka', lambda n: n.startswith('left_joint_')),
    ('Desna ruka', lambda n: n.startswith('right_joint_')),
    ('Lijeva hvataljka', lambda n: n.startswith('left_robotiq')),
    ('Desna hvataljka', lambda n: n.startswith('right_robotiq')),
    ('Torzo (klizaci)', lambda n: 'carriage' in n),
    ('Pan-tilt', lambda n: n.startswith('pan_tilt')),
    ('Ostalo', lambda n: True),
]


def robot_description(node):
    """URDF text: prefer the live topic, fall back to expanding the xacro."""
    got = []
    qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/robot_description',
                             lambda m: got.append(m.data), qos)
    end = node.get_clock().now().nanoseconds + 3e9
    while not got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if got:
        return got[0]
    node.get_logger().info('Nema /robot_description, prosirujem xacro...')
    return subprocess.run(['xacro', URDF], capture_output=True, text=True,
                          check=True).stdout


def parse_joints(urdf):
    """[(name, type, lower, upper)] for every movable, non-mimic joint."""
    out = []
    for j in ET.fromstring(urdf).findall('joint'):
        jtype = j.get('type')
        if jtype not in MOVABLE or j.find('mimic') is not None:
            continue
        lim = j.find('limit')
        if jtype == 'continuous' or lim is None:
            lo, hi = -CONT_LIMIT, CONT_LIMIT
        else:
            lo = float(lim.get('lower', -CONT_LIMIT))
            hi = float(lim.get('upper', CONT_LIMIT))
        out.append((j.get('name'), jtype, lo, hi))
    return out


class JointRow:
    """One joint: label, typed value, slider. Prismatic in mm, others in deg."""

    def __init__(self, parent, row, name, jtype, lo, hi):
        self.name = name
        self.prismatic = (jtype == 'prismatic')
        self.scale = 1000.0 if self.prismatic else 180.0 / math.pi
        self.unit = 'mm' if self.prismatic else '°'
        self.lo, self.hi = lo * self.scale, hi * self.scale
        self._syncing = False

        ttk.Label(parent, text=name, width=34, anchor='w').grid(
            row=row, column=0, sticky='w', padx=(4, 6), pady=1)

        # Start inside the limits: the carriages run 50-650 mm, so a plain 0
        # would put them below their lower stop.
        start = max(self.lo, min(self.hi, 0.0))
        self.var = tk.DoubleVar(value=start)
        self.entry = ttk.Entry(parent, width=9, justify='right')
        self.entry.insert(0, f'{start:.1f}')
        self.entry.grid(row=row, column=1, padx=2)
        self.entry.bind('<Return>', self._from_entry)
        self.entry.bind('<FocusOut>', self._from_entry)
        ttk.Label(parent, text=self.unit, width=3).grid(row=row, column=2)

        self.slider = ttk.Scale(parent, from_=self.lo, to=self.hi,
                                orient='horizontal', length=320,
                                variable=self.var, command=self._from_slider)
        self.slider.grid(row=row, column=3, padx=6, sticky='we')
        ttk.Label(parent, text=f'[{self.lo:.0f}, {self.hi:.0f}]', width=16,
                  foreground='#666').grid(row=row, column=4, sticky='w')

    def _from_slider(self, _=None):
        if self._syncing:
            return
        self._syncing = True
        self.entry.delete(0, tk.END)
        self.entry.insert(0, f'{self.var.get():.1f}')
        self._syncing = False

    def _from_entry(self, _=None):
        if self._syncing:
            return
        try:
            v = float(self.entry.get().replace(',', '.'))
        except ValueError:
            v = self.var.get()
        v = max(self.lo, min(self.hi, v))
        self._syncing = True
        self.var.set(v)
        self.entry.delete(0, tk.END)
        self.entry.insert(0, f'{v:.1f}')
        self._syncing = False

    def set(self, value_si):
        """Set from a value in URDF units (rad or m)."""
        v = max(self.lo, min(self.hi, value_si * self.scale))
        self._syncing = True
        self.var.set(v)
        self.entry.delete(0, tk.END)
        self.entry.insert(0, f'{v:.1f}')
        self._syncing = False

    @property
    def value_si(self):
        return self.var.get() / self.scale


class App:
    def __init__(self, node, joints, preset=None):
        self.preset = preset or {}
        self.node = node
        self.pub = node.create_publisher(JointState, '/joint_states', 10)
        self.root = tk.Tk()
        self.root.title('PAS dual-arm - zglobovi (stupnjevi)')
        self.rows = {}

        outer = ttk.Frame(self.root)
        outer.pack(fill='both', expand=True)
        canvas = tk.Canvas(outer, width=880, height=720, highlightthickness=0)
        sb = ttk.Scrollbar(outer, orient='vertical', command=canvas.yview)
        frame = ttk.Frame(canvas)
        frame.bind('<Configure>',
                   lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=frame, anchor='nw')
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        canvas.bind_all('<Button-4>', lambda e: canvas.yview_scroll(-2, 'units'))
        canvas.bind_all('<Button-5>', lambda e: canvas.yview_scroll(2, 'units'))

        r = 0
        used = set()
        for title, sel in GROUPS:
            members = [j for j in joints if j[0] not in used and sel(j[0])]
            if not members:
                continue
            used.update(j[0] for j in members)
            ttk.Label(frame, text=title, font=('TkDefaultFont', 10, 'bold')).grid(
                row=r, column=0, sticky='w', pady=(10, 2), padx=4)
            r += 1
            for name, jtype, lo, hi in members:
                self.rows[name] = JointRow(frame, r, name, jtype, lo, hi)
                r += 1

        bar = ttk.Frame(self.root)
        bar.pack(fill='x', pady=6)
        ttk.Button(bar, text='Sve na 0', command=self.zero).pack(side='left', padx=4)
        ttk.Button(bar, text='Zrcali lijevu -> desnu',
                   command=self.mirror).pack(side='left', padx=4)
        ttk.Button(bar, text='Ispisi pozu', command=self.dump).pack(side='left', padx=4)
        self.status = ttk.Label(bar, text='objavljujem /joint_states @30 Hz',
                                foreground='#357')
        self.status.pack(side='right', padx=8)

        self.root.protocol('WM_DELETE_WINDOW', self.quit)
        if self.preset:
            n = 0
            for name, value in self.preset.items():
                if name in self.rows:
                    self.rows[name].set(value)
                    n += 1
            self.status.config(text=f'ucitana spremljena poza ({n} zglobova)')
        else:
            self.adopt_current()
        self.tick()

    def adopt_current(self, timeout=2.0):
        """Take over the posture another publisher is currently showing, so
        switching from joint_state_publisher_gui does not snap the robot to 0."""
        seen = {}
        sub = self.node.create_subscription(
            JointState, '/joint_states',
            lambda m: seen.update(zip(m.name, m.position)), 10)
        end = self.node.get_clock().now().nanoseconds + timeout * 1e9
        while not seen and self.node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        n = 0
        for name, value in seen.items():
            if name in self.rows:
                self.rows[name].set(value)
                n += 1
        if n:
            self.status.config(text=f'preuzeta trenutna poza ({n} zglobova)')

    def zero(self):
        for row in self.rows.values():
            row.set(0.0)

    def mirror(self):
        """Copy the left arm onto the right one (same joint values)."""
        n = 0
        for j in range(1, 8):
            src, dst = f'left_joint_{j}', f'right_joint_{j}'
            if src in self.rows and dst in self.rows:
                self.rows[dst].set(self.rows[src].value_si)
                n += 1
        self.status.config(text=f'zrcaljeno {n} zglobova')

    def dump(self):
        left = {j: self.rows[f'left_joint_{j}'].value_si
                for j in range(1, 8) if f'left_joint_{j}' in self.rows}
        right = {j: self.rows[f'right_joint_{j}'].value_si
                 for j in range(1, 8) if f'right_joint_{j}' in self.rows}
        def fmt(d):
            return '{' + ', '.join(f'{k}: {v:.3f}' for k, v in sorted(d.items())) + '}'
        print(f'\nLEFT  = {fmt(left)}')
        print(f'RIGHT = {fmt(right)}')
        print('  (u stupnjevima: ' + ', '.join(
            f'j{k}={math.degrees(v):.0f}' for k, v in sorted(left.items())) + ')')
        self.status.config(text='poza ispisana u terminal')

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for name, row in self.rows.items():
            msg.name.append(name)
            msg.position.append(row.value_si)
        self.pub.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(33, self.tick)

    def quit(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--poza', help='ucitaj spremljenu pozu iz notes/08_poze.md '
                                   '(npr. ARM_CARRY_V2)')
    args = ap.parse_args()

    rclpy.init()
    node = Node('joint_gui')
    urdf = robot_description(node)
    joints = parse_joints(urdf)
    node.get_logger().info(f'{len(joints)} pomicnih zglobova')
    preset = load_posture(args.poza) if args.poza else None
    if args.poza:
        node.get_logger().info(
            f'poza {args.poza}: {len(preset)} zglobova'
            if preset else f'poza {args.poza} nije nadena u registru')
    app = App(node, joints, preset)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
