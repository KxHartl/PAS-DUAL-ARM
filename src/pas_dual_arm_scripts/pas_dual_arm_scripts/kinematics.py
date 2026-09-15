"""Whole-robot forward kinematics from the URDF, and left/right arm mirroring.

No ROS graph and no MoveIt: everything here works from the URDF text alone, so
the same numbers come out offline (`scripts/grasp_width.py`), in the pose GUI
(`scripts/joint_gui.py`) and in a running node (`grasp_stage`).
"""

import math
import xml.etree.ElementTree as ET

import numpy as np

from pas_dual_arm_scripts.robot_extent import rpy_matrix

# Right = left with these joint signs gets the hand ORIENTATION exactly right
# but misses its position by 20-45 mm: the Gen3's small lateral link offsets do
# not flip with the joints. So it is only the seed for mirror_right's IK.
MIRROR_SIGNS = (-1, 1, -1, 1, -1, 1, -1)


class Kinematics:
    """Places every link of the robot for a given set of joint values.

    `force_model.ArmModel` covers one arm and is what the IK runs on; this
    places the WHOLE robot, because the width is decided by links that are not
    on the arm chain (the shoulder mounts) as often as by ones that are.
    """

    def __init__(self, xml):
        self.root = ET.fromstring(xml)
        self.joints = []
        for joint in self.root.findall('joint'):
            origin = joint.find('origin')
            axis = joint.find('axis')
            mimic = joint.find('mimic')
            self.joints.append(dict(
                mimic=None if mimic is None else (
                    mimic.get('joint'), float(mimic.get('multiplier', 1.0)),
                    float(mimic.get('offset', 0.0))),
                name=joint.get('name'), type=joint.get('type'),
                parent=joint.find('parent').get('link'),
                child=joint.find('child').get('link'),
                xyz=np.array([float(v) for v in ((origin.get('xyz') if origin
                              is not None else None) or '0 0 0').split()]),
                R=rpy_matrix(*[float(v) for v in ((origin.get('rpy') if origin
                               is not None else None) or '0 0 0').split()]),
                axis=np.array([float(v) for v in ((axis.get('xyz') if axis
                               is not None else None) or '1 0 0').split()])))
        children = {j['child'] for j in self.joints}
        self.base = next(link.get('name') for link in self.root.findall('link')
                         if link.get('name') not in children)
        self.limits = {}
        for joint in self.root.findall('joint'):
            limit = joint.find('limit')
            self.limits[joint.get('name')] = (
                (-np.pi, np.pi) if limit is None or joint.get('type') == 'continuous'
                else (float(limit.get('lower')), float(limit.get('upper'))))

    def place(self, positions):
        """{link: (rotation, translation)} in the base frame."""
        frames = {self.base: (np.eye(3), np.zeros(3))}
        pending = list(self.joints)
        while pending:
            rest, progressed = [], False
            for joint in pending:
                if joint['parent'] not in frames:
                    rest.append(joint)
                    continue
                parent_R, parent_t = frames[joint['parent']]
                R, t = joint['R'], joint['xyz']
                if joint['mimic'] is not None:
                    # A closed Robotiq is four mimic joints following one
                    # knuckle; without this every finger stays open.
                    source, gain, offset = joint['mimic']
                    value = gain * float(positions.get(source, 0.0)) + offset
                else:
                    value = float(positions.get(joint['name'], 0.0))
                length = np.linalg.norm(joint['axis'])
                unit = joint['axis'] / length if length else joint['axis']
                if joint['type'] in ('revolute', 'continuous'):
                    skew = np.array([[0, -unit[2], unit[1]],
                                     [unit[2], 0, -unit[0]],
                                     [-unit[1], unit[0], 0]])
                    R = R @ (np.eye(3) + np.sin(value) * skew
                             + (1 - np.cos(value)) * skew @ skew)
                elif joint['type'] == 'prismatic':
                    t = t + joint['R'] @ unit * value
                frames[joint['child']] = (parent_R @ R, parent_R @ t + parent_t)
                progressed = True
            pending = rest
            if not progressed:
                break
        return frames


def mirror_right(kin, right_model, ik, q, keep_camera_side=True):
    """Right-arm joints that mirror the left hand through the centre plane.

    A true mirror image puts the right WRIST CAMERA on the opposite side of the
    approach axis: both arms carry the camera the same way round (it is not
    mirrored hardware), and a reflection flips the tool y-axis. The face
    markers are raised for a camera ABOVE the axis, so the mirrored right hand
    lost its marker (stage run S3). With `keep_camera_side` the right hand is
    turned 180 deg about its own approach axis - joint 7, so joints 1-6 stay
    the exact mirror and the arm still looks symmetric. The two-finger Robotiq
    is symmetric under that turn, so the grasp itself does not change.

    `q` holds the left arm (and the carriages); `right_model` is the right
    `force_model.ArmModel` and `ik` a KDL position IK on its chain. The arm
    mounts mirror to 1 mm, and seeding from the sign-flipped left arm keeps the
    right elbow mirrored too (0.04-0.15 rad from the seed on test postures).

    Returns (joints, position error m, rotation error), joints None if the IK
    did not converge.
    """
    import PyKDL as kdl
    frames = kin.place(q)
    left_R, left_t = frames['left_end_effector_link']
    flip = np.diag([1.0, -1.0, 1.0])
    goal_R, goal_t = flip @ left_R @ flip, flip @ left_t
    if keep_camera_side:
        goal_R = goal_R @ np.diag([-1.0, -1.0, 1.0])     # 180 deg about tool z
    base_R, base_t = frames['right_base_link']
    local_R, local_t = base_R.T @ goal_R, base_R.T @ (goal_t - base_t)
    target = kdl.Frame(kdl.Rotation(*local_R.flatten()), kdl.Vector(*local_t))
    start, out = kdl.JntArray(7), kdl.JntArray(7)
    for i, sign in enumerate(MIRROR_SIGNS):
        start[i] = sign * q[f'left_joint_{i + 1}']
    if keep_camera_side:
        start[6] = start[6] + math.pi
    # LMA from the mirrored seed converges almost always; when it stalls, a
    # few nearby seeds recover it without leaving the mirrored branch (the
    # perturbation is small next to the 0.02-0.15 rad the solution moves).
    # The arm is redundant (7 joints, 6 constraints), and LMA knows nothing of
    # joint limits: from one seed it can land just past a limit while an
    # equally exact solution sits inside it (joint_gui test: right j6 past its
    # stop, 2 deg from the real right arm). So an out-of-limit answer only
    # counts if no seed finds one inside.
    names = [f'right_joint_{i + 1}' for i in range(7)]
    lower = [kin.limits.get(n, (-math.pi, math.pi))[0] for n in names]
    upper = [kin.limits.get(n, (-math.pi, math.pi))[1] for n in names]
    rng = np.random.default_rng(0)
    seeds = [None] + [rng.uniform(-0.15, 0.15, 7) for _ in range(16)]
    joints = None
    for jitter in seeds:
        trial = kdl.JntArray(7)
        for i in range(7):
            trial[i] = start[i] + (0.0 if jitter is None else jitter[i])
        if ik.CartToJnt(trial, target, out) < 0:
            continue
        found = [math.atan2(math.sin(out[i]), math.cos(out[i])) for i in range(7)]
        if all(lower[i] <= found[i] <= upper[i] for i in range(7)):
            joints = found
            break
        joints = joints or found
    if joints is None:
        return None, float('nan'), float('nan')
    check = dict(q)
    check.update({f'right_joint_{i + 1}': v for i, v in enumerate(joints)})
    right_R, right_t = kin.place(check)['right_end_effector_link']
    return (joints, float(np.linalg.norm(right_t - goal_t)),
            float(np.linalg.norm(right_R - goal_R)))


def _rotvec(R):
    """Rotation vector (axis * angle) of a rotation matrix."""
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(w)
    angle = math.atan2(s, np.trace(R) - 1.0)
    return np.zeros(3) if s < 1e-12 else w / s * angle


def axis_rotation(axis, angle):
    """Rotation matrix for `angle` about the unit vector `axis`."""
    a = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


class ArmJog:
    """Small Cartesian moves of one hand, and elbow swivel with the hand held.

    Resolved-rate steps on the arm's own Jacobian (force_model.resolved_rate)
    rather than a global IK: a global solver may answer a 5 mm request from a
    different branch - the same arm pose, a completely different arm (P-25) -
    while a Jacobian step can only move locally. Poses are in the robot frame
    (base_footprint; same axes as base_link: X ahead, Y left, Z up).
    """

    def __init__(self, xml, side, kin=None):
        import PyKDL as kdl
        from pas_dual_arm_scripts.force_model import ArmModel
        self.side = side
        self.kin = kin or Kinematics(xml)
        self.model = ArmModel(xml, side)
        self.fk = kdl.ChainFkSolverPos_recursive(self.model.chain)
        self.names = list(self.model.names)
        self.lower = [self.kin.limits.get(n, (-math.pi, math.pi))[0] for n in self.names]
        self.upper = [self.kin.limits.get(n, (-math.pi, math.pi))[1] for n in self.names]
        self._kdl = kdl

    def hand(self, q):
        """(R, t) of this side's end effector for joint values `q`."""
        return self.kin.place(q)[f'{self.side}_end_effector_link']

    def _local(self, joints):
        kdl = self._kdl
        arr, frame = kdl.JntArray(7), kdl.Frame()
        for i, v in enumerate(joints):
            arr[i] = float(v)
        self.fk.JntToCart(arr, frame)
        R = np.array([[frame.M[r, c] for c in range(3)] for r in range(3)])
        return R, np.array([frame.p[0], frame.p[1], frame.p[2]])

    def solve(self, q, goal_R, goal_t, max_jump=0.8):
        """Arm joints putting the hand at (goal_R, goal_t), from q's arm joints.

        Raises ValueError when it cannot converge, leaves the joint limits, or
        would have to swing the arm by more than `max_jump` rad to get there.
        """
        from pas_dual_arm_scripts.force_model import resolved_rate
        base_R, base_t = self.kin.place(q)[f'{self.side}_base_link']
        want_R, want_t = base_R.T @ goal_R, base_R.T @ (np.asarray(goal_t) - base_t)
        start = np.array([q[n] for n in self.names], dtype=float)
        x = start.copy()
        # Joints with stops (Gen3: 2, 4, 6). A pose picked by a planner can sit
        # a fraction of a degree from one - the S4 scan pose had joint 6 at
        # -119.7 of +/-120 deg - and then half the jog directions are refused.
        # The seventh joint is spare, so while the hand walks to its goal those
        # joints are also pushed off their stops, inside the null space where
        # the hand does not feel it.
        limited = [i for i in range(7)
                   if self.upper[i] - self.lower[i] < 2 * math.pi - 1e-3]
        # Just enough to stay off the stop. 20 deg was tried first: every 5 mm
        # step then swung the whole arm by ~30 deg on its way out of the margin.
        margin = math.radians(4.0)
        settled = 0
        for _ in range(600):
            R, t = self._local(x)
            dp, dr = want_t - t, _rotvec(want_R @ R.T)
            push = np.zeros(7)
            for i in limited:
                if x[i] - self.lower[i] < margin:
                    push[i] = margin - (x[i] - self.lower[i])
                elif self.upper[i] - x[i] < margin:
                    push[i] = -(margin - (self.upper[i] - x[i]))
            on_goal = np.linalg.norm(dp) < 1e-4 and np.linalg.norm(dr) < 1e-4
            if on_goal:
                settled += 1
                if np.max(np.abs(push)) < 1e-4 or settled > 300:
                    break
            # Walk in short strides so every step stays well inside the
            # linearisation: 2 mm / 0.02 rad at a time.
            dp *= min(1.0, 0.002 / max(np.linalg.norm(dp), 1e-12))
            dr *= min(1.0, 0.02 / max(np.linalg.norm(dr), 1e-12))
            jac = self.model.jacobian(x)
            step = resolved_rate(jac, np.concatenate([dp, dr]), max_step=0.3)
            if push.any():
                # Full gain is safe: projecting onto the null space never moves a
                # joint further than it was asked to. At 0.2 the push crept -
                # joint 6 has only a 0.28 share of the one null direction - and
                # was still pulling the elbow out long after the hand arrived.
                null = np.eye(7) - np.linalg.pinv(jac) @ jac
                step = step + null @ push
            x = x + step
        else:
            raise ValueError('ruka ne moze doci u tu pozu (nema konvergencije)')
        R, t = self._local(x)
        if np.linalg.norm(want_t - t) > 2e-4 or np.linalg.norm(_rotvec(want_R @ R.T)) > 2e-4:
            raise ValueError('ruka ne moze doci u tu pozu (nema konvergencije)')
        outside = [i + 1 for i in range(7) if not self.lower[i] - 1e-6 <= x[i] <= self.upper[i] + 1e-6
                   and self.upper[i] - self.lower[i] < 2 * math.pi - 1e-3]
        if outside:
            raise ValueError(f'zglob {outside} bi izasao iz granica')
        if np.max(np.abs(x - start)) > max_jump:
            raise ValueError('korak bi preokrenuo ruku - smanji korak')
        return list(x)

    def elbow_swivel(self, q, angle):
        """Swivel the elbow by `angle` (rad along the null space), hand held.

        With seven joints and the hand fixed in all six directions exactly one
        motion is left: the elbow turning about the shoulder-wrist line - the
        smallest right singular vector of the Jacobian. Which way that looks
        "up" depends entirely on the pose (in the scan pose it is almost
        horizontal: "elbow up" and "elbow down" both lowered it by 2-3 mm), so
        the sign is defined by what matters for a doorway instead: positive
        `angle` brings the elbow IN towards the robot's centre plane, negative
        takes it out. Both directions are solved and the one that actually
        does that is kept.

        Returns (joints, elbow displacement in metres as a 3-vector).
        """
        R0, t0 = self.hand(q)
        # Settle first: a pose within 4 deg of a joint stop is pushed off it by
        # every solve, and that push alone moved the elbow 11 mm OUT whichever
        # way it was asked to swivel (scan pose, joint 6 at -119.7 deg). So the
        # swivel starts from the settled pose and is judged against it.
        settled = dict(q)
        settled.update(zip(self.names, self.solve(q, R0, t0)))
        q = settled
        x0 = np.array([q[n] for n in self.names], dtype=float)
        null = np.linalg.svd(self.model.jacobian(x0))[2][-1]
        elbow = f'{self.side}_forearm_link'
        before = self.kin.place(q)[elbow][1]
        best = None
        for direction in (+1.0, -1.0):
            moved = dict(q)
            moved.update({n: v + direction * abs(angle) * d
                          for n, v, d in zip(self.names, x0, null)})
            try:
                joints = self.solve(moved, R0, t0, max_jump=abs(angle) + 0.3)
            except ValueError:
                continue
            check = dict(q)
            check.update(zip(self.names, joints))
            shift = self.kin.place(check)[elbow][1] - before
            inward = abs(before[1]) - abs(before[1] + shift[1])
            # A swivel that ends where it started is not a success: next to a
            # joint stop the null-space push hands the elbow straight back
            # (scan pose, "inward": 0.0 mm, reported as done).
            if (inward > 0) == (angle > 0) and abs(inward) > 0.0005 \
                    and (best is None or abs(inward) > best[2]):
                best = (joints, shift, abs(inward))
        if best is None:
            stops = [f'{i + 1} ({math.degrees(x0[i]):.0f} deg)'
                     for i in range(7)
                     if self.upper[i] - self.lower[i] < 2 * math.pi - 1e-3
                     and min(x0[i] - self.lower[i], self.upper[i] - x0[i]) < math.radians(6.0)]
            where = 'prema unutra' if angle > 0 else 'prema van'
            raise ValueError(f'lakat ne moze dalje {where}' +
                             (f' - zglob {", ".join(stops)} je na granicniku' if stops else ''))
        return best[0], best[1]


PAD_LINKS = ('robotiq_85_left_finger_tip_link', 'robotiq_85_right_finger_tip_link')


def level_approach(R):
    """The same hand orientation with its approach axis (tool Z) turned level.

    The wrist camera looks along the approach axis. GRASP_V3 points it 55 deg
    down, which is right for pressing and wrong for reading a marker on a
    vertical face, so the scan pose straightens only that pitch (user, 15. 9.).
    """
    z = R[:, 2]
    level = np.array([z[0], z[1], 0.0])
    level /= np.linalg.norm(level)
    axis = np.cross(z, level)
    s = np.linalg.norm(axis)
    if s < 1e-9:
        return R.copy()
    return axis_rotation(axis, math.atan2(s, float(z @ level))) @ R


def pad_contact_pose(kin, hulls, q_ref, side, face_centre, normal, interference, yaw=0.0):
    """Hand pose that keeps the reference orientation and puts the pads on a face.

    `q_ref` is the reference posture (GRASP_V3, the carriages as they are, the
    gripper as it will be); its hand orientation is kept, turned by `yaw` about
    Z to match the cube. The pads' collision geometry then decides the hand
    position: their innermost point sits `interference` inside the face plane,
    and their centre on the face centre along the face. `normal` points OUT of
    the face towards this hand; everything is in the robot (base_footprint)
    frame. Returns (R, p).
    """
    frames = kin.place(q_ref)
    R0, t0 = frames[f'{side}_end_effector_link']
    pads = np.vstack([hulls[f'{side}_{n}'] @ frames[f'{side}_{n}'][0].T + frames[f'{side}_{n}'][1]
                      for n in PAD_LINKS])
    R = axis_rotation([0.0, 0.0, 1.0], yaw) @ R0
    rel = ((pads - t0) @ R0) @ R.T          # pad geometry relative to the hand, turned
    n = np.asarray(normal, dtype=float) / np.linalg.norm(normal)
    c = np.asarray(face_centre, dtype=float)
    mid = rel.mean(axis=0)
    along = lambda v: v - (v @ n) * n       # noqa: E731 - the in-face components
    p = along(c) - along(mid) + (c @ n - interference - float((rel @ n).min())) * n
    return R, p
