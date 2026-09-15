"""Quasistatic Gen3 wrench estimation, with no simulator dependencies.

Wrench convention: force exerted BY the environment ON the tool, expressed
at the end-effector origin in the arm-base frame. Joint efforts are actuator
torques: tau = gravity - J.T @ external_wrench at rest.
"""
import xml.etree.ElementTree as ET

import numpy as np
import PyKDL as kdl


def frame(element):
    if element is None:
        return kdl.Frame.Identity()
    xyz = [float(v) for v in element.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in element.get('rpy', '0 0 0').split()]
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


class ArmModel:
    """KDL arm chain; off-chain masses (camera and open gripper) are included.

    Gripper branches are frozen at URDF zero, so callers must enforce open
    fingers. Gravity is supplied in the actual arm-base frame, not assumed Z.
    """

    def __init__(self, xml, side):
        root = ET.fromstring(xml)
        links = {e.get('name'): e for e in root.findall('link')}
        joints = {e.get('name'): e for e in root.findall('joint')}
        children = {}
        parent = {}
        for joint in joints.values():
            p, c = joint.find('parent').get('link'), joint.find('child').get('link')
            children.setdefault(p, []).append(joint)
            parent[c] = joint
        self.base = f'{side}_base_link'
        self.tip = f'{side}_end_effector_link'
        path = []
        current = self.tip
        while current != self.base:
            joint = parent[current]
            path.append(joint)
            current = joint.find('parent').get('link')
        path.reverse()
        path_names = {j.get('name') for j in path}

        def inertia(name):
            inertial = links[name].find('inertial')
            result = kdl.RigidBodyInertia.Zero()
            if inertial is not None:
                i = inertial.find('inertia')
                tensor = kdl.RotationalInertia(*[
                    float(i.get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')])
                local = kdl.RigidBodyInertia(
                    float(inertial.find('mass').get('value')), kdl.Vector.Zero(), tensor)
                result = frame(inertial.find('origin')) * local
            for joint in children.get(name, []):
                if joint.get('name') not in path_names:
                    result = result + frame(joint.find('origin')) * inertia(
                        joint.find('child').get('link'))
            return result

        self.chain = kdl.Chain()
        self.names = []
        for element in path:
            name, kind = element.get('name'), element.get('type')
            origin = frame(element.find('origin'))
            if kind == 'fixed':
                joint = kdl.Joint(name, kdl.Joint.Fixed)
            elif kind in ('revolute', 'continuous'):
                axis = kdl.Vector(*map(float, element.find('axis').get('xyz').split()))
                joint = kdl.Joint(name, origin.p, origin.M * axis, kdl.Joint.RotAxis)
                self.names.append(name)
            else:
                raise ValueError(f'Unsupported arm joint {name}: {kind}')
            child = element.find('child').get('link')
            self.chain.addSegment(kdl.Segment(child, joint, origin, inertia(child)))
        if len(self.names) != 7:
            raise ValueError('Expected a seven-joint Gen3 chain')
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

    def jacobian(self, positions):
        """Geometric Jacobian at `positions`, in the arm-base frame."""
        q = kdl.JntArray(7)
        for i, value in enumerate(positions):
            q[i] = float(value)
        jac = kdl.Jacobian(7)
        if self.jac_solver.JntToJac(q, jac) < 0:
            raise ValueError('KDL Jacobian failed')
        return np.array([[jac[r, c] for c in range(7)] for r in range(6)])

    def terms(self, positions, gravity):
        q = kdl.JntArray(7)
        for i, value in enumerate(positions):
            q[i] = float(value)
        jac = kdl.Jacobian(7)
        g = kdl.JntArray(7)
        if self.jac_solver.JntToJac(q, jac) < 0:
            raise ValueError('KDL Jacobian failed')
        if kdl.ChainDynParam(self.chain, kdl.Vector(*gravity)).JntToGravity(q, g) < 0:
            raise ValueError('KDL gravity failed')
        return (np.array([[jac[r, c] for c in range(7)] for r in range(6)]),
                np.array([g[i] for i in range(7)]))


def resolved_rate(jacobian, error, damping=0.005, length_scale=0.3, max_step=0.02):
    """Joint step that moves the tool by `error`, without leaving this branch.

    A global IK solver is the wrong tool for a half-millimetre correction: when
    its seeded solve does not converge it restarts from a random seed and hands
    back a different arm configuration, which is how a 0.5 mm squeeze step came
    back 3 rad away (notes/03_problemi/P-25). A damped least-squares Jacobian
    step is local by construction, so no branch change is possible.

    `error` is [dx, dy, dz, rx, ry, rz] in the ARM-BASE frame. Rotation is
    scaled by a characteristic arm length so the two halves are comparable.

    Damping trades accuracy for staying bounded near a singularity. Measured
    over 200 random Jacobians, worst case: 0.002 -> 1.0% of the requested motion
    lost, 0.005 -> 5.6%, 0.01 -> 16.7%, 0.05 -> 54%. 0.005 keeps the squeeze
    advancing while still bounded; the step size itself is set by how small the
    request is, not by the damping.
    """
    scale = np.diag([1., 1., 1., length_scale, length_scale, length_scale])
    a = scale @ np.asarray(jacobian, dtype=float)
    target = scale @ np.asarray(error, dtype=float)
    if not np.isfinite(a).all() or not np.isfinite(target).all():
        raise ValueError('non-finite resolved-rate input')
    step = a.T @ np.linalg.solve(a @ a.T + damping**2 * np.eye(6), target)
    if np.max(np.abs(step)) > max_step:
        raise ValueError(
            f'resolved-rate step {np.max(np.abs(step)):.4f} rad exceeds {max_step} rad')
    return step


def estimate(jacobian, gravity, measured, bias, damping=0.002,
             length_scale=0.3, condition_limit=100.0, residual_limit=0.2):
    """Damped least squares with dimensionless conditioning of force/moment.

    Scale moment by a characteristic arm length so condition numbers do not
    compare metres with radians. Reject unreliable solutions; never clamp
    invalid estimates into plausible forces.
    """
    scale = np.diag([1., 1., 1., length_scale, length_scale, length_scale])
    a = jacobian.T @ scale
    target = gravity - (np.asarray(measured) - np.asarray(bias))
    if not np.isfinite(a).all() or not np.isfinite(target).all():
        raise ValueError('non-finite joint data')
    u, singular, vt = np.linalg.svd(a, full_matrices=False)
    condition = singular[0] / max(singular[-1], 1e-12)
    if singular[-1] < 1e-8 or condition > condition_limit:
        raise ValueError(f'ill-conditioned arm pose ({condition:.1f})')
    wrench = scale @ (vt.T @ ((singular / (singular**2 + damping**2)) * (u.T @ target)))
    residual = float(np.linalg.norm(jacobian.T @ wrench - target))
    if residual > residual_limit:
        raise ValueError(f'torque model residual {residual:.3f} Nm')
    return wrench, condition, residual
