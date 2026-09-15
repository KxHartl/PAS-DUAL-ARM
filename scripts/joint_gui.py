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

With --sim it drives the SIMULATED robot instead: nothing is published on
/joint_states (Gazebo owns it), the sliders start from the real posture, and
the send buttons command the controllers - carriages, then grippers, then arms,
one after the other. "Posalji (MoveIt)" plans a collision-checked path to the
dialled joints; "Posalji direktno" hands them straight to the arm controllers,
which is what a pose touching the cube needs (MoveIt refuses a goal in contact)
and does NOT check collisions. "Provjeri" prints which pads touch the cube and
how wide the robot is right now.

    ./scripts/run_cube_isolated.sh python3 scripts/joint_gui.py --sim
    ./scripts/run_cube_isolated.sh python3 scripts/joint_gui.py --sim --poza MY_GRASP
"""
import argparse
import ast
import math
import os
import re
import subprocess
import sys
import tkinter as tk
import xml.etree.ElementTree as ET
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:          # contact read-out only; everything else works
    Contacts = None

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
    # Carriages and gripper opening, saved by capture_posture since 15. 9.
    # (older postures have none and leave those sliders as they are).
    for part in ('TORSO', 'GRIPPER'):
        m = re.search(rf'^{re.escape(name)}_{part}\s*=\s*(\{{[^}}]*\}})', text, re.M)
        if m:
            out.update({k: float(v) for k, v in ast.literal_eval(m.group(1)).items()})
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
        # Gen3 joints 1, 3, 5, 7 turn without stops, and the robot happily
        # reports 5.128 rad for one. Shown folded into +/-180 deg; what is SENT
        # is the equivalent angle nearest the live one (App._command), or a
        # fold would turn into a full extra rotation of the arm.
        self.continuous = (jtype == 'continuous')
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
        if self.continuous:
            value_si = math.atan2(math.sin(value_si), math.cos(value_si))
        v = max(self.lo, min(self.hi, value_si * self.scale))
        self._syncing = True
        self.var.set(v)
        self.entry.delete(0, tk.END)
        self.entry.insert(0, f'{v:.1f}')
        self._syncing = False

    @property
    def value_si(self):
        return self.var.get() / self.scale


ARM_JOINTS = {side: [f'{side}_joint_{j}' for j in range(1, 8)]
              for side in ('left', 'right')}
CARRIAGES = ['torso_left_carriage_joint', 'torso_right_carriage_joint']
KNUCKLE = {side: f'{side}_robotiq_85_left_knuckle_joint' for side in ('left', 'right')}
PADS = ('left_left', 'left_right', 'right_left', 'right_right')
DOOR = 1.0            # narrowest doorway on the route (notes/06_parametri)
CARRY_WIDTH = 0.840   # ARM_CARRY_V2, the posture validated through it (P-35)


class App:
    def __init__(self, node, joints, preset=None, sim=False, urdf=None):
        self.preset = preset or {}
        self.node = node
        self.sim = sim
        self.urdf = urdf
        self._mirror = None
        self._jog = None
        self.pub = (None if sim else
                    node.create_publisher(JointState, '/joint_states', 10))
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
        if sim:
            simbar = ttk.Frame(self.root)
            simbar.pack(fill='x', pady=(0, 6))
            ttk.Button(simbar, text='Posalji (MoveIt)',
                       command=lambda: self.send(direct=False)).pack(side='left', padx=4)
            ttk.Button(simbar, text='Posalji direktno (bez provjere sudara)',
                       command=lambda: self.send(direct=True)).pack(side='left', padx=4)
            ttk.Button(simbar, text='Preuzmi s robota',
                       command=self.adopt_live).pack(side='left', padx=4)
            ttk.Button(simbar, text='Provjeri', command=self.check).pack(side='left', padx=4)
        self._build_jog_panel(sim)
        self.status = ttk.Label(
            bar, text=('SIM: salje kontrolerima, ne objavljuje /joint_states'
                       if sim else 'objavljujem /joint_states @30 Hz'),
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
            # With --sim the window may open before the simulation publishes
            # joint states; filling the sliders from nothing would put zeros on
            # the Send button. Wait for the real robot instead.
            self.adopt_current(timeout=180.0 if sim else 2.0)
        if self.sim:
            self._start_sim()
        self._save_proc = None
        self._build_saved_window()
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

    # ----------------------------------------------------------- hand jogging
    def _build_jog_panel(self, sim):
        """Buttons that move a hand along the axes, and swivel the elbow."""
        jog = ttk.LabelFrame(self.root, text='Saka po osima  (baza: X naprijed, Y lijevo, Z gore)')
        jog.pack(fill='x', padx=4, pady=(0, 6))
        self.jog_side = tk.StringVar(value='both')
        self.jog_frame = tk.StringVar(value='base')
        self.jog_mm = tk.StringVar(value='5')
        self.jog_deg = tk.StringVar(value='2')
        self.jog_send = tk.BooleanVar(value=sim)
        top = ttk.Frame(jog)
        top.pack(fill='x', pady=2)
        for text, value in (('Lijeva', 'left'), ('Desna', 'right'), ('Obje simetricno', 'both')):
            ttk.Radiobutton(top, text=text, value=value,
                            variable=self.jog_side).pack(side='left', padx=4)
        ttk.Separator(top, orient='vertical').pack(side='left', fill='y', padx=8)
        for text, value in (('osi baze', 'base'), ('osi alata (Z = prema plohi)', 'tool')):
            ttk.Radiobutton(top, text=text, value=value,
                            variable=self.jog_frame).pack(side='left', padx=4)
        ttk.Label(top, text='korak').pack(side='left', padx=(12, 2))
        ttk.Entry(top, textvariable=self.jog_mm, width=5, justify='right').pack(side='left')
        ttk.Label(top, text='mm').pack(side='left', padx=(2, 8))
        ttk.Entry(top, textvariable=self.jog_deg, width=5, justify='right').pack(side='left')
        ttk.Label(top, text='°').pack(side='left', padx=2)
        moves = ttk.Frame(jog)
        moves.pack(fill='x', pady=2)
        for axis, minus, plus in (('x', '-X nazad', '+X naprijed'),
                                  ('y', '-Y desno', '+Y lijevo'),
                                  ('z', '-Z dolje', '+Z gore')):
            ttk.Button(moves, text=minus, width=11,
                       command=lambda a=axis: self.jog('move', a, -1)).pack(side='left', padx=2)
            ttk.Button(moves, text=plus, width=11,
                       command=lambda a=axis: self.jog('move', a, +1)).pack(side='left', padx=(2, 10))
        turns = ttk.Frame(jog)
        turns.pack(fill='x', pady=2)
        for axis in 'xyz':
            for sign, mark in ((-1, '-'), (+1, '+')):
                ttk.Button(turns, text=f'{mark}R{axis.upper()}', width=6,
                           command=lambda a=axis, g=sign: self.jog('turn', a, g)).pack(side='left', padx=2)
        ttk.Separator(turns, orient='vertical').pack(side='left', fill='y', padx=8)
        ttk.Button(turns, text='Lakat prema unutra (saka stoji)',
                   command=lambda: self.jog('elbow', None, +1)).pack(side='left', padx=2)
        ttk.Button(turns, text='Lakat prema van (saka stoji)',
                   command=lambda: self.jog('elbow', None, -1)).pack(side='left', padx=2)
        if sim:
            ttk.Checkbutton(turns, text='odmah posalji (direktno)',
                            variable=self.jog_send).pack(side='left', padx=12)
        ttk.Label(jog, foreground='#666', text=(
            'Obje simetricno: pomice se lijeva, desna je njeno zrcalo - +Y = obje van, '
            '-Y = obje prema sredini. Lakat: saka stoji, ruka se zakrene oko pravca '
            'rame-zapesce (jedini slobodan pomak kad je saka zamrznuta); korak u °.')).pack(anchor='w', padx=4)

    def _tools(self):
        """Kinematics for mirroring and jogging, built once from the URDF."""
        if self._mirror is None:
            import PyKDL as kdl
            from pas_dual_arm_scripts.force_model import ArmModel
            from pas_dual_arm_scripts.kinematics import ArmJog, Kinematics
            urdf = self.urdf or robot_description(self.node)
            kin = Kinematics(urdf)
            model = ArmModel(urdf, 'right')
            self._mirror = (kin, model, kdl.ChainIkSolverPos_LMA(model.chain, 1e-7, 500))
            self._jog = {side: ArmJog(urdf, side, kin) for side in ('left', 'right')}
        return self._mirror

    def _set_status(self, text):
        print(text, flush=True)
        self.status.config(text=text)

    def jog(self, kind, axis, sign):
        """One step of the selected hand(s); the rest of the arm follows.

        kind 'move' shifts the hand by the mm step along an axis, 'turn' turns
        it by the degree step about one, 'elbow' swivels the elbow with the
        hand held. With 'Obje simetricno' the left hand takes the step and the
        right one becomes its mirror image (camera kept on the same side).
        """
        import numpy as np
        from pas_dual_arm_scripts.kinematics import axis_rotation, mirror_right
        try:
            mm = float(self.jog_mm.get().replace(',', '.'))
            deg = float(self.jog_deg.get().replace(',', '.'))
        except ValueError:
            self._set_status('korak nije broj')
            return
        mode, frame = self.jog_side.get(), self.jog_frame.get()
        snap = None
        try:
            self._tools()
            q = {name: row.value_si for name, row in self.rows.items()}
            moved, elbow = [], None
            if mode == 'both':
                # How far the right hand is from the mirror of the left BEFORE
                # the step. "Both" puts it exactly there, so the first click
                # also squares up whatever asymmetry the arms had - measured 9 mm
                # after staging, from tracking - and it should say so.
                left_t = self._jog['left'].hand(q)[1]
                right_t = self._jog['right'].hand(q)[1]
                snap = float(np.linalg.norm(right_t - left_t * np.array([1.0, -1.0, 1.0])))
            for side in (('left',) if mode == 'both' else (mode,)):
                arm = self._jog[side]
                if kind == 'elbow':
                    joints, elbow = arm.elbow_swivel(q, sign * math.radians(deg))
                else:
                    R, t = arm.hand(q)
                    unit = np.eye(3)['xyz'.index(axis)]
                    if kind == 'move':
                        step = sign * mm / 1000.0 * (R @ unit if frame == 'tool' else unit)
                        joints = arm.solve(q, R, t + step)
                    else:
                        turn = axis_rotation(unit, sign * math.radians(deg))
                        joints = arm.solve(q, R @ turn if frame == 'tool' else turn @ R, t)
                q.update(zip(arm.names, joints))
                moved.append(side)
            if mode == 'both':
                right, _, _ = mirror_right(*self._mirror, q)
                if right is None:
                    raise ValueError('desna ruka ne moze u zrcalnu pozu')
                q.update({f'right_joint_{j + 1}': v for j, v in enumerate(right)})
                moved.append('right')
        except ValueError as exc:
            self._set_status(f'korak odbijen: {exc}')
            return
        for side in moved:
            for j in range(1, 8):
                self.rows[f'{side}_joint_{j}'].set(q[f'{side}_joint_{j}'])
        q = {name: row.value_si for name, row in self.rows.items()}
        base = self._mirror[0].place(q)['base_link'][1]
        hands = []
        for side in moved:
            t = self._jog[side].hand(q)[1] - base
            hands.append(f'{side} ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})')
        text = 'saka u base_link: ' + ', '.join(hands)
        if elbow is not None:
            text += (f' | lakat pomaknut ({elbow[0] * 1000:+.1f}, {elbow[1] * 1000:+.1f}, '
                     f'{elbow[2] * 1000:+.1f}) mm')
        if snap is not None and snap > 0.002:
            text += f' | desna poravnata u zrcalo lijeve (bila {snap * 1000:.0f} mm od zrcala)'
        self._set_status(text)
        if self.sim and self.jog_send.get():
            self.send(direct=True, minimum=0.8, preempt=True)

    # ------------------------------------------------------------ saved poses
    def _build_saved_window(self):
        """Second window: the postures in notes/08_poze.md, load or save."""
        win = tk.Toplevel(self.root)
        win.title('PAS dual-arm - spremljene poze')
        self.saved_list = tk.Listbox(win, height=16, width=34)
        self.saved_list.pack(fill='both', expand=True, padx=6, pady=6)
        self.saved_list.bind('<Double-Button-1>', lambda _e: self.load_saved())
        row = ttk.Frame(win)
        row.pack(fill='x', padx=6)
        ttk.Button(row, text='Ucitaj u klizace', command=self.load_saved).pack(side='left', padx=2)
        ttk.Button(row, text='Osvjezi popis', command=self.refresh_saved).pack(side='left', padx=2)
        save = ttk.Frame(win)
        save.pack(fill='x', padx=6, pady=6)
        self.save_name = tk.StringVar(value='GRASP_V1')
        ttk.Entry(save, textvariable=self.save_name, width=18).pack(side='left', padx=2)
        ttk.Button(save, text='Spremi trenutnu pozu', command=self.save_current).pack(side='left', padx=2)
        ttk.Label(win, foreground='#666', text=(
            'Ucitano ide samo u klizace - robot se mice tek na Posalji.'
            if self.sim else 'Ucitano se odmah prikazuje u RViz-u.')).pack(anchor='w', padx=6, pady=(0, 6))
        self.refresh_saved()

    def refresh_saved(self):
        names = []
        if os.path.exists(REGISTRY):
            for name in re.findall(r'^([A-Za-z0-9_]+)_LEFT\s*=\s*\{', open(REGISTRY).read(), re.M):
                if name not in names:
                    names.append(name)
        self.saved_list.delete(0, tk.END)
        for name in names:
            self.saved_list.insert(tk.END, name)

    def load_saved(self):
        chosen = self.saved_list.curselection()
        if not chosen:
            self.status.config(text='odaberi pozu u popisu')
            return
        name = self.saved_list.get(chosen[0])
        n = 0
        for joint, value in load_posture(name).items():
            if joint in self.rows:
                self.rows[joint].set(value)
                n += 1
        self.status.config(text=f'{name}: ucitano {n} zglobova u klizace')

    def save_current(self):
        """Record the robot's posture with capture_posture.py (width included)."""
        name = self.save_name.get().strip().upper()
        if not re.fullmatch(r'[A-Z][A-Z0-9_]*', name):
            self.status.config(text='ime: slova, brojke i _ (npr. GRASP_V1)')
            return
        if self._save_proc is not None and self._save_proc.poll() is None:
            self.status.config(text='spremanje vec traje')
            return
        script = os.path.join(ROOT, 'scripts', 'capture_posture.py')
        self._save_proc = subprocess.Popen(
            [sys.executable, script, name, 'spremljeno iz joint_gui'])
        self._save_label = name
        self.status.config(text=f'spremam {name} ...')

    def _poll_save(self):
        if self._save_proc is None or self._save_proc.poll() is None:
            return
        ok = self._save_proc.returncode == 0
        self.status.config(text=f'{self._save_label}: ' + ('spremljeno u notes/08_poze.md'
                                                          if ok else 'spremanje NIJE uspjelo (vidi terminal)'))
        self._save_proc = None
        self.refresh_saved()

    def zero(self):
        for row in self.rows.values():
            row.set(0.0)

    def mirror(self):
        """Right arm becomes the mirror image of the left one.

        This used to copy the joint values, which on this robot is not a
        mirror at all - the right arm is mounted mirrored, so the same angles
        send it somewhere else entirely (compare ARM_CARRY_V2). The left hand's
        pose is mirrored through the robot's centre plane instead and the right
        arm solved for it.
        """
        try:
            self._tools()
            q = {name: row.value_si for name, row in self.rows.items()}
            from pas_dual_arm_scripts.kinematics import mirror_right
            joints, pos_err, rot_err = mirror_right(*self._mirror, q)
        except Exception as exc:
            self.status.config(text=f'zrcaljenje nije uspjelo: {exc}')
            return
        if joints is None:
            self.status.config(text='zrcaljenje: IK za desnu ruku ne konvergira')
            return
        outside = [j + 1 for j, v in enumerate(joints)
                   if not (self.rows[f'right_joint_{j + 1}'].lo
                           <= math.degrees(v) <= self.rows[f'right_joint_{j + 1}'].hi)]
        for j, value in enumerate(joints):
            self.rows[f'right_joint_{j + 1}'].set(value)
        text = (f'zrcaljeno: desna saka {pos_err * 1000:.1f} mm od zrcalne '
                f'(orijentacija {rot_err:.4f})')
        if outside:
            text += f' - UPOZORENJE: zglobovi {outside} izvan granica, odrezani'
        self.status.config(text=text)

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
        self._poll_save()
        if self.sim:
            # ROS spins in its own thread (see _start_sim); only the label is
            # touched here, because Tk must stay on this thread.
            if self._status_text is not None:
                self.status.config(text=self._status_text)
                self._status_text = None
            self.root.after(100, self.tick)
            return
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for name, row in self.rows.items():
            msg.name.append(name)
            msg.position.append(row.value_si)
        self.pub.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(33, self.tick)

    # ----------------------------------------------------------- --sim mode
    def _start_sim(self):
        """Clients, live state and a spin thread for driving the simulation.

        The spin runs on its own thread because a 30 Hz spin_once in the Tk
        loop takes one callback per tick, and /tf alone arrives at ~1 kHz: the
        queue backs up and every TF lookup answers from seconds ago (the same
        starvation measured in main_task, P-19).
        """
        node = self.node
        self.live = {}
        self._status_text = None
        self._queue = []
        self._busy, self._gen = False, 0
        self._targets, self._minimum = {}, 3.0
        self._pad_box_t, self._pad_any_t = {}, {}
        node.create_subscription(
            JointState, '/joint_states',
            lambda m: self.live.update(zip(m.name, m.position)), 10)
        self.move = ActionClient(node, MoveGroup, '/move_action')
        self.arm = {side: ActionClient(node, FollowJointTrajectory,
                                       f'/{side}_arm_controller/follow_joint_trajectory')
                    for side in ('left', 'right')}
        self.grip = {side: ActionClient(node, GripperCommand,
                                        f'/{side}_gripper_controller/gripper_cmd')
                     for side in ('left', 'right')}
        self.torso = ActionClient(node, FollowJointTrajectory,
                                  '/torso_controller/follow_joint_trajectory')
        if Contacts is not None:
            for pad in PADS:
                node.create_subscription(
                    Contacts, f'/contact/{pad}_tip',
                    lambda m, k=pad: self._on_contact(k, m), 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, node, spin_thread=True)
        self.urdf = self.urdf or robot_description(node)
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def _say(self, text):
        print(text, flush=True)
        self._status_text = text

    def _on_contact(self, pad, msg):
        if not msg.contacts:
            return
        now = time.monotonic()
        self._pad_any_t[pad] = now
        for c in msg.contacts:
            names = (getattr(c.collision1, 'name', '') + '|'
                     + getattr(c.collision2, 'name', ''))
            if 'aruco_box' in names:
                self._pad_box_t[pad] = now
                break

    def adopt_live(self):
        n = 0
        for name, value in self.live.items():
            if name in self.rows:
                self.rows[name].set(value)
                n += 1
        self._say(f'preuzeta poza s robota ({n} zglobova)')

    def _changed(self, names, tol):
        return any(n in self.rows and abs(self._command(n) - self.live.get(n, 1e9)) > tol
                   for n in names)

    def send(self, direct, minimum=3.0, preempt=False):
        """Queue carriages -> grippers -> arms and start the first one.

        Every target is read HERE, on the Tk thread. The later steps are built
        from ROS callbacks on the spin thread, and Tk variables must not be
        touched from there. Each send carries a generation number, so a goal
        preempted by a newer jog step cannot report over it or cancel it.
        """
        if self._busy and not preempt:
            self._say('prethodna naredba jos traje')
            return
        self._targets = {name: self._command(name) for name in self.rows}
        self._minimum = minimum
        steps = []
        if self._changed(CARRIAGES, 0.001):
            steps.append(self._torso_goal)
        for side in ('left', 'right'):
            if self._changed([KNUCKLE[side]], 0.01):
                steps.append(lambda s=side: self._gripper_goal(s))
        if direct:
            for side in ('left', 'right'):
                steps.append(lambda s=side: self._arm_goal(s))
        else:
            steps.append(self._moveit_goal)
        self._gen += 1
        self._queue, self._busy = steps, True
        self._next(self._gen)

    def _next(self, gen):
        if gen != self._gen:
            return
        if not self._queue:
            self._busy = False
            return
        client, goal, label = self._queue.pop(0)()
        if not client.server_is_ready():
            self._queue, self._busy = [], False
            self._say(f'{label}: akcijski server nije dostupan')
            return
        self._say(f'{label}: saljem...')
        client.send_goal_async(goal).add_done_callback(
            lambda f: self._accepted(f, label, gen))

    def _accepted(self, future, label, gen):
        if gen != self._gen:
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self._queue, self._busy = [], False
            self._say(f'{label}: ODBIJENO')
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._finished(f, label, gen))

    def _finished(self, future, label, gen):
        if gen != self._gen:
            return
        response = future.result()
        ok = response is not None and response.status == GoalStatus.STATUS_SUCCEEDED
        detail = ''
        if response is not None and hasattr(response.result, 'error_code'):
            code = response.result.error_code
            detail = f' (kod {getattr(code, "val", code)})'
        if not ok:
            self._queue, self._busy = [], False
            hint = (' - MoveIt odbija cilj u dodiru s kockom; za takvu pozu '
                    'koristi "direktno"' if label == 'MoveIt ruke' else '')
            self._say(f'{label}: NEUSPJEH{detail}{hint}')
            return
        self._say(f'{label}: OK{detail}')
        self._next(gen)

    def _command(self, name):
        """What to send for `name`: for a continuous joint the angle equivalent
        to the slider that is NEAREST the live one, never a full turn away."""
        row = self.rows[name]
        target = row.value_si
        live = self.live.get(name)
        if row.continuous and live is not None:
            return live + math.atan2(math.sin(target - live), math.cos(target - live))
        return target

    def _duration(self, names, speed, minimum):
        delta = max((abs(self._targets[n] - self.live.get(n, self._targets[n]))
                     for n in names if n in self._targets), default=0.0)
        return max(minimum, delta / speed)

    @staticmethod
    def _trajectory_goal(names, positions, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.velocities = [0.0] * len(names)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1) * 1e9)
        goal.trajectory.points.append(point)
        return goal

    def _torso_goal(self):
        seconds = self._duration(CARRIAGES, 0.03, 4.0)
        goal = self._trajectory_goal(CARRIAGES, [self._targets[n] for n in CARRIAGES], seconds)
        return self.torso, goal, 'vodilice'

    def _gripper_goal(self, side):
        goal = GripperCommand.Goal()
        goal.command.position = float(self._targets[KNUCKLE[side]])
        goal.command.max_effort = 50.0
        return self.grip[side], goal, f'hvataljka {side}'

    def _arm_goal(self, side):
        names = ARM_JOINTS[side]
        # Both arms get the same duration, so they move together (P-26).
        seconds = self._duration(ARM_JOINTS['left'] + ARM_JOINTS['right'], 0.25, self._minimum)
        goal = self._trajectory_goal(names, [self._targets[n] for n in names], seconds)
        return self.arm[side], goal, f'ruka {side} (direktno)'

    def _moveit_goal(self):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'both_arms'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        constraints = Constraints(name='joint_gui')
        for name in ARM_JOINTS['left'] + ARM_JOINTS['right']:
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(self._targets[name]),
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        return self.move, goal, 'MoveIt ruke'

    def check(self):
        """Pads on the cube, robot width and hand positions, right now."""
        lines = ['', '=== PROVJERA POZE ===']
        now = time.monotonic()
        if Contacts is None:
            lines.append('  kontakti: ros_gz_interfaces nije dostupan')
        for pad in PADS:
            box, other = self._pad_box_t.get(pad), self._pad_any_t.get(pad)
            if box is not None and now - box < 0.5:
                state = 'KOCKA'
            elif other is not None and now - other < 0.5:
                state = 'dodiruje nesto drugo (ne kocku)'
            else:
                state = '-'
            lines.append(f'  jastucic {pad:12s} {state}')
        try:
            from pas_dual_arm_scripts.robot_extent import ExtentMeasurer
            extent = ExtentMeasurer(self.urdf)
            placed, stacked = 0, []
            for name, points in extent.links.items():
                try:
                    tf = self.tf.lookup_transform(extent.frame, name, rclpy.time.Time())
                except Exception:
                    continue
                placed += 1
                t, q = tf.transform.translation, tf.transform.rotation
                from pas_dual_arm_scripts.robot_extent import quat_matrix
                stacked.append(points @ quat_matrix([q.x, q.y, q.z, q.w]).T
                               + [t.x, t.y, t.z])
            if placed < len(extent.links):
                lines.append(f'  sirina: NEPOUZDANO - TF smjestio {placed}/{len(extent.links)} linkova')
            else:
                import numpy as np
                allp = np.vstack(stacked)
                width = float(allp[:, 1].max() - allp[:, 1].min())
                lines.append(
                    f'  sirina robota {width:.3f} m  (vrata {DOOR:.2f} m -> '
                    f'{"PROLAZI" if width < DOOR else "NE PROLAZI"}; '
                    f'ARM_CARRY_V2 {CARRY_WIDTH:.3f} m)')
        except Exception as exc:
            lines.append(f'  sirina: nije izmjerena ({exc})')
        for side in ('left', 'right'):
            try:
                t = self.tf.lookup_transform(
                    'base_link', f'{side}_end_effector_link',
                    rclpy.time.Time()).transform.translation
                lines.append(f'  saka {side:5s} ({t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f}) u base_link')
            except Exception:
                lines.append(f'  saka {side:5s} TF nedostupan')
        carriages = [self.live.get(n, float('nan')) for n in CARRIAGES]
        lines.append(f'  vodilice {carriages[0] * 1000:.1f} / {carriages[1] * 1000:.1f} mm')
        print('\n'.join(lines), flush=True)
        self.dump()
        self._say('provjera ispisana u terminal')

    def quit(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim', action='store_true',
                    help='upravljaj simuliranim robotom umjesto objave /joint_states')
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
    app = App(node, joints, preset, sim=args.sim, urdf=urdf)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
