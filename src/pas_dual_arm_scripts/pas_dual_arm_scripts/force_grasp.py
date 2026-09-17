"""Bounded, camera/torque-driven squeeze. No Gazebo imports or subscriptions."""
from copy import deepcopy
import json
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .force_model import ArmModel, resolved_rate
from trajectory_msgs.msg import JointTrajectoryPoint


SIDES = ('left', 'right')


def squeeze_steps(forces, target, gain, dt, max_step=0.0005):
    """A single coupled width/centre correction, in inward coordinates.

    Common motion changes width; differential motion balances load. Bound each
    hand AND centre drift, rather than pushing one hand until a pad appears.
    """
    values = np.asarray(forces, dtype=float)
    if not np.isfinite(values).all() or target <= 0 or dt <= 0:
        raise ValueError('invalid force control input')
    error = target - values
    error[np.abs(error) < 0.15 * target] = 0.0
    common = float(error.mean()) * gain * dt
    balance = float(error[0] - error[1]) * gain * dt * 0.5
    balance = np.clip(balance, -max_step / 2, max_step / 2)
    steps = np.array([common + balance, common - balance])
    return np.clip(steps, -max_step, max_step)


class ForceGrasp:
    def __init__(self, node, qualification='', characterize=False):
        self.node = node
        self.report = None
        self.received = 0.
        self.characterize = characterize
        self.qualification = qualification
        defaults = dict(force_target=5.0, force_max=15.0, force_gain=0.0003,
                        force_step=0.0005, force_period=0.25, force_timeout=45.0,
                        force_past_face=0.01, force_lift_speed=0.005,
                        force_hold_seconds=30.0, force_max_slip=0.005,
                        force_max_tilt=0.0872665, force_vision_age=0.8,
                        force_status_age=0.15, force_future=0.05)
        self.cfg = {k: node.declare_parameter(k, v).value for k, v in defaults.items()}
        self.target = self.cfg['force_target']
        if not (0 < self.target < self.cfg['force_max']):
            raise ValueError('force target must be between zero and force_max')
        self.events = node.create_publisher(String, '/grasp/events', 10)
        node.create_subscription(String, '/grasp/estimator_status', self.status, 10)
        # Read the wrist markers from their own topic, not through /tf. Measured
        # 15. 9.: in a node doing nothing else the marker transform is 0.060 s
        # old, but read from inside the squeeze loop it was 0.73 s and the lag
        # grew to whatever limit was set. robot_state_publisher republishes /tf
        # at the joint rate - 1000 Hz under this profile - and a Python consumer
        # cannot drain that while the loop holds the GIL, even on its own thread.
        # This topic carries the same detection at 15 Hz, and the camera frame is
        # a FIXED child of the end-effector link, so the rest is a constant.
        self.marker = {}
        self.marker_to_tool = {}
        for side in SIDES:
            node.create_subscription(
                PoseStamped, f'/wrist_{side}/marker_pose',
                lambda msg, key=side: self.marker.__setitem__(key, msg), 20)
        self.description = None
        node.create_subscription(String, '/robot_description', self.set_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.zero_client = node.create_client(Trigger, '/grasp/zero_wrench')
        self.scene_get = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.scene_apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')
        self.initial_visual = None
        self.baseline_torso = None
        self.hand_targets = {}
        self.hand_origins = {}
        self.ages = []
        self.vision_ages = []
        self.models = {}
        self.phase = 'initializing'

    def status(self, msg):
        self.report = json.loads(msg.data)
        self.received = time.monotonic()

    def set_description(self, msg):
        self.description = msg.data

    def check_environment(self, require_qualification=True):
        deadline = time.monotonic() + 10.
        while not (self.description and self.report):
            if time.monotonic() > deadline:
                raise RuntimeError('robot description / wrench estimator unavailable')
            rclpy.spin_once(self.node, timeout_sec=0.02)
        if any('DetachableJoint' in p.get('name', '')
               for p in ET.fromstring(self.description).iter('plugin')):
            raise RuntimeError('force grasp requires a model without DetachableJoint')
        if require_qualification:
            self.check_qualification()

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def event(self, phase, **values):
        self.phase = phase
        for name, samples in (('status_age', self.ages), ('vision_age', self.vision_ages)):
            if not samples:
                continue
            recent = sorted(samples[-400:])
            values[name] = dict(
                n=len(recent), median=round(recent[len(recent) // 2], 4),
                p95=round(recent[int(0.95 * (len(recent) - 1))], 4),
                max=round(recent[-1], 4))
        self.events.publish(String(data=json.dumps(dict(stamp=self.now(), phase=phase, **values))))
        self.node.get_logger().info(f'Force grasp: {phase}')

    def wait(self, future, seconds=5.):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        if not future.done():
            raise RuntimeError('force-grasp service/action timed out')
        return future.result()

    def service(self, client, request):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError(f'missing service {client.srv_name}')
        return self.wait(client.call_async(request))

    def check_qualification(self):
        if self.characterize:
            return
        with open(self.qualification, encoding='utf-8') as stream:
            proof = json.load(stream)
        if not self.report or proof.get('fingerprint') != self.report['fingerprint']:
            raise RuntimeError('qualification does not match the running estimator/model')
        if proof.get('passed') is not True or proof.get('force_target') != self.target:
            raise RuntimeError('force estimator has not passed qualification at this target')
        mu = proof.get('friction_lower_bound', 0.)
        if mu <= 0 or 2 * mu * self.target < 2 * 0.3 * (9.81 + 0.05):
            raise RuntimeError('no verified friction/load margin for lifting')

    # Measured 15. 9. during a squeeze, over 400 reads: the marker TF as THIS
    # node reads it is median 0.32 s, p95 0.388 s, max 0.407 s old, while the
    # force estimate is 0.008 s. The old 0.4 s limit sat below the typical
    # reading, so the abort was a matter of when, not if. The sensor is not the
    # cause: the detector publishes every 0.066 s without a single gap, and
    # detector-to-subscriber latency is 22-43 ms. The extra ~0.25 s is this
    # node's own TF consumption - /tf carries robot_state_publisher at joint
    # rate, and `spin_once` runs one callback at a time, so the buffer trails.
    # 0.8 s keeps ~2x headroom over the measured worst case while still
    # catching a real loss of sight, whose age grows without bound.
    def transform(self, target, source, age=0.8):
        tf = self.node.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        # Same asymmetry as the force estimate: a TF stamped slightly ahead of
        # this node's clock has simply overtaken the /clock message for its time.
        reading = self.now() - stamp
        if source.endswith('_grasp_marker_frame'):
            self.vision_ages.append(reading)
        if not -self.cfg['force_future'] <= reading <= age:
            raise RuntimeError(f'stale TF {source}: {reading:+.3f} s (limit {age:.3f} s)')
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])

    def forces(self):
        if not self.report or time.monotonic() - self.received > 0.5:
            raise RuntimeError('estimator stream missing')
        forces = []
        for side in SIDES:
            arm = self.report['arms'][side]
            if not arm.get('valid'):
                raise RuntimeError(f'{side} invalid force: {arm.get("reason")}')
            # Say which of the two rejections happened. Reporting the estimator's
            # own reason for a staleness rejection reads as 'invalid force: ok'
            # and hides that the estimate was fine and merely arrived late.
            age = self.now() - arm.get('stamp', -1)
            self.ages.append(age)
            # A stamp a few ms AHEAD of this node's clock is delivery order, not
            # bad data: /clock and the data topics carry no ordering guarantee
            # between them, and the lead was measured at up to 19 ms here. Old
            # data is the direction that matters, and it keeps its own limit.
            if not -self.cfg['force_future'] <= age <= self.cfg['force_status_age']:
                raise RuntimeError(
                    f'{side} force estimate {age:+.3f} s old (window '
                    f'{-self.cfg["force_future"]:+.3f}..{self.cfg["force_status_age"]:+.3f} s)')
            _, rot = self.transform('base_link', f'{side}_base_link')
            # External force on the hand points opposite its inward approach.
            f = rot.apply(np.asarray(arm['wrench'][:3]))
            raw = rot.apply(np.asarray(arm['raw'][:3]))
            normal = self.headings[side]
            if np.linalg.norm(raw) > self.cfg['force_max']:
                raise RuntimeError(f'{side} force limit exceeded')
            forces.append(float(-np.dot(f, normal)))
        return forces

    def marker_in_tool(self, side):
        """Latest marker detection expressed in that hand's end-effector frame."""
        msg = self.marker.get(side)
        if msg is None:
            raise RuntimeError(f'{side} wrist has seen no marker')
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        reading = self.now() - stamp
        self.vision_ages.append(reading)
        if not -self.cfg['force_future'] <= reading <= self.cfg['force_vision_age']:
            raise RuntimeError(
                f'{side} marker {reading:+.3f} s old '
                f'(limit {self.cfg["force_vision_age"]:.3f} s)')
        offset, rotation = self.marker_to_tool[side]
        q = msg.pose.orientation
        p = msg.pose.position
        return (offset + rotation.apply([p.x, p.y, p.z]),
                rotation * Rotation.from_quat([q.x, q.y, q.z, q.w]))

    def vision(self):
        observed = {s: self.marker_in_tool(s) for s in SIDES}
        if self.initial_visual is not None:
            for side, (p, r) in observed.items():
                initial_p, initial_r = self.initial_visual[side]
                # Only tangential motion indicates slip; normal corrections
                # intentionally change distance between camera and box face.
                slip = np.linalg.norm((p - initial_p)[:2])  # tool +Z is approach
                tilt = (initial_r.inv() * r).magnitude()
                if slip > self.cfg['force_max_slip'] or tilt > self.cfg['force_max_tilt']:
                    raise RuntimeError(f'{side} visual slip={slip:.4f}m tilt={tilt:.3f}rad')
        return observed

    def allow_pads(self):
        request = GetPlanningScene.Request()
        request.components.components = request.components.ALLOWED_COLLISION_MATRIX
        response = self.service(self.scene_get, request)
        matrix = response.scene.allowed_collision_matrix
        self.original_matrix = deepcopy(matrix)
        pads = [f'{s}_robotiq_85_{f}_finger_tip_link' for s in SIDES for f in SIDES]
        for name in ['target_cube'] + pads:
            if name not in matrix.entry_names:
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_names.append(name)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        box = matrix.entry_names.index('target_cube')
        for pad in pads:
            i = matrix.entry_names.index(pad)
            matrix.entry_values[i].enabled[box] = True
            matrix.entry_values[box].enabled[i] = True
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.allowed_collision_matrix = matrix
        if not self.service(self.scene_apply, request).success:
            raise RuntimeError('cannot apply fingertip-only collision allowance')

    def zero(self, attempts=5):
        """Calibrate the force zero, waiting for the arms to actually stop.

        The estimator refuses a zero taken while an arm is still ringing, and it
        is right to: the bias would absorb the motion. Asking once right after
        the standoff was too early - measured 15. 9., the left arm was still at
        0.069-0.258 Nm of torque spread against the 0.05 Nm limit, and only
        settled to 0.004 Nm about 8 s after the move. Rather than guess a fixed
        pause, retry and let the estimator's own stability test decide when the
        arm is quiet.
        """
        self.event('zeroing')
        deadline = time.monotonic() + 120
        for attempt in range(1, attempts + 1):
            if not self.service(self.zero_client, Trigger.Request()).success:
                raise RuntimeError('zero service refused')
            start = self.now()
            while self.now() - start < 8 and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
                if not self.report or self.report['stamp'] <= start + 2.0:
                    continue
                arms = self.report['arms']
                if all(a.get('valid') for a in arms.values()):
                    self.check_qualification()
                    if max(abs(f) for f in self.forces()) > self.target * 0.2:
                        raise RuntimeError('nonzero pre-contact force')
                    return
                # The estimator gives up on an unstable window; ask again rather
                # than burn the whole budget waiting for a zero it abandoned.
                if any('unstable' in (a.get('reason') or '') for a in arms.values()):
                    break
            reasons = {s: a.get('reason') for s, a in self.report['arms'].items()}
            self.node.get_logger().warn(
                f'zero attempt {attempt}/{attempts} not settled: {reasons}')
        raise RuntimeError(f'force zero failed after {attempts} attempts: {self.report}')

    def step(self, inward, torso_targets=None):
        """Small synchronized moves, checked as a combined robot state."""
        positions = {}
        next_targets = {}
        for side, delta in zip(SIDES, inward):
            pose = deepcopy(self.hand_targets[side])
            pose.position.x += float(delta * self.headings[side][0])
            pose.position.y += float(delta * self.headings[side][1])
            offset = np.dot(np.array([pose.position.x, pose.position.y, pose.position.z])
                            - self.hand_origins[side], self.headings[side])
            if not -0.003 <= offset <= 0.02 + self.cfg['force_past_face']:
                raise RuntimeError(f'{side} approach correction limit')
            # A rising carriage changes the arm base, not its desired arm
            # configuration. Solve the squeeze at the CURRENT carriage height.
            actual = self.node._ee_pose(f'{side}_end_effector_link')
            if actual is None:
                raise RuntimeError('missing hand TF')
            pose.position.z = actual.position.z
            # Differential correction, not an IK call. MoveIt's solver is global:
            # when its seeded solve fails it restarts from a random seed, and a
            # 0.5 mm step came back 3 rad away on the left arm, which the branch
            # guard below then (correctly) refused to execute. Measured 15. 9.:
            # IK pose error is 0.00 mm, but the branch is not the current one.
            # See notes/03_problemi/P-25.
            error = np.empty(6)
            error[:3] = [pose.position.x - actual.position.x,
                         pose.position.y - actual.position.y,
                         0.0]
            error[3:] = (Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                             pose.orientation.z, pose.orientation.w])
                         * Rotation.from_quat([actual.orientation.x, actual.orientation.y,
                                               actual.orientation.z,
                                               actual.orientation.w]).inv()).as_rotvec()
            _, base = self.transform('base_link', f'{side}_base_link')
            model = self.models[side]
            here = np.array([self.node._joint[n][0] for n in model.names])
            step = resolved_rate(model.jacobian(here), np.concatenate(
                [base.inv().apply(error[:3]), base.inv().apply(error[3:])]))
            positions.update(dict(zip(model.names, here + step)))
            next_targets[side] = pose
        if torso_targets is not None:
            positions.update(dict(zip(self.torso_names, torso_targets)))
        # Check the joint midpoint and endpoint for BOTH moving arms/rails.
        for fraction in (0.5, 1.):
            request = GetStateValidity.Request()
            request.robot_state.joint_state.name = list(self.node._joint)
            request.robot_state.joint_state.position = [
                self.node._joint[n][0] + fraction * (positions.get(n, self.node._joint[n][0])
                                                    - self.node._joint[n][0])
                for n in request.robot_state.joint_state.name]
            if not self.service(self.validity, request).valid:
                raise RuntimeError('combined correction collides')
        specs = [(s, getattr(self.node, f'{s}_jtc'), [f'{s}_joint_{i}' for i in range(1, 8)])
                 for s in SIDES]
        if torso_targets is not None:
            specs.append(('torso', self.node.torso, self.torso_names))
        handles = []
        try:
            sends = []
            for side, client, names in specs:
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = names
                point = JointTrajectoryPoint()
                point.positions = [float(positions[n]) for n in names]
                if max(abs(positions[n] - self.node._joint[n][0]) for n in names) > 0.04:
                    raise RuntimeError(f'{side} IK branch/tracking jump')
                point.velocities = [0.] * len(names)
                point.time_from_start = rclpy.duration.Duration(seconds=self.cfg['force_period']).to_msg()
                goal.trajectory.points = [point]
                sends.append(client.send_goal_async(goal))
            rejected = False
            for send in sends:
                handle = self.wait(send)
                if handle is None or not handle.accepted:
                    rejected = True
                else:
                    handles.append(handle)
            if rejected:
                raise RuntimeError('correction rejected')
            results = [h.get_result_async() for h in handles]
            deadline = time.monotonic() + 5.
            while not all(f.done() for f in results):
                rclpy.spin_once(self.node, timeout_sec=0.01)
                self.forces()
                self.vision()
                if time.monotonic() > deadline:
                    raise RuntimeError('correction timed out')
            if any(f.result().status != GoalStatus.STATUS_SUCCEEDED for f in results):
                raise RuntimeError('correction execution failed')
            self.hand_targets = next_targets
        except Exception:
            for handle in handles:
                self.wait(handle.cancel_goal_async())
            raise

    def regulate(self, seconds, height=None):
        start, wall = self.now(), time.monotonic()
        stable_since = None
        while self.now() - start < self.cfg['force_timeout'] + seconds:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if time.monotonic() - wall > 4 * (self.cfg['force_timeout'] + seconds):
                raise RuntimeError('simulation stalled during force control')
            forces = self.forces()
            self.vision()
            balanced = all(abs(f - self.target) <= 0.2*self.target for f in forces)
            targets = None
            if height is not None:
                current = np.array([self.node._joint[n][0] for n in self.torso_names])
                progress = current - self.baseline_torso
                if abs(progress[0] - progress[1]) > 0.005:
                    raise RuntimeError('carriages desynchronized')
                destination = self.baseline_torso + height
                if np.any(destination < 0.05) or np.any(destination > 0.65):
                    raise RuntimeError('carriage stroke exceeded')
                if balanced:
                    targets = current + np.clip(destination-current,
                        -self.cfg['force_lift_speed']*self.cfg['force_period'],
                        self.cfg['force_lift_speed']*self.cfg['force_period'])
                reached = np.max(np.abs(current-destination)) < 0.002
            else:
                reached = True
            if balanced and reached:
                stable_since = self.now() if stable_since is None else stable_since
                if self.now() - stable_since >= seconds:
                    return
            else:
                stable_since = None
            self.step(squeeze_steps(forces, self.target, self.cfg['force_gain'],
                                    self.cfg['force_period'], self.cfg['force_step']), targets)
        raise RuntimeError('force/height did not settle within deadline')

    def run(self, contacts, no_lift=False):
        self.torso_names = ['torso_left_carriage_joint', 'torso_right_carriage_joint']
        self.baseline_torso = np.array([self.node._joint[n][0] for n in self.torso_names])
        self.headings = {}
        for side, goal in contacts.items():
            pose = self.node._ee_pose(f'{side}_end_effector_link')
            p = np.array([pose.position.x, pose.position.y, pose.position.z])
            direction = np.array([goal.position.x, goal.position.y, goal.position.z]) - p
            direction[2] = 0.
            span = float(np.linalg.norm(direction))
            self.node.get_logger().info(
                f'{side} measured standoff {span * 1000:.1f} mm to its contact pose')
            if abs(span - 0.02) > 0.005:
                raise RuntimeError(
                    f'{side} is {span * 1000:.1f} mm from its contact pose, '
                    'not 20 +/- 5 mm')
            self.headings[side] = direction / np.linalg.norm(direction)
            self.hand_targets[side], self.hand_origins[side] = pose, p
        # The squeeze corrects differentially, so it needs the arm chains.
        self.models = {s: ArmModel(self.description, s) for s in SIDES}
        # Camera-to-tool is a fixed joint, so read it once and never again.
        self.marker_to_tool = {
            s: self.transform(f'{s}_end_effector_link', f'{s}_wrist_cam_optical_frame',
                              age=5.0) for s in SIDES}
        self.zero()
        self.allow_pads()
        try:
            self.event('squeeze')
            self.regulate(2.)
            self.initial_visual = self.vision()
            if no_lift or self.characterize:
                self.event('characterization_complete')
                return
            for height in (0.02, 0.20):
                self.event('lift', height=height)
                self.regulate(2., height)
            self.event('hold')
            self.regulate(self.cfg['force_hold_seconds'], 0.20)
            self.event('lower')
            self.regulate(2., 0.)
            self.initial_visual = None
            self.event('release')
            for _ in range(6):
                self.step([-0.0005, -0.0005])
            self.event('complete')
        except Exception as exc:
            self.event('aborted', reason=str(exc))
            # Lower only while both sensing paths remain valid. There is no
            # blind descent or renewed inward search after sensor loss.
            try:
                self.forces()
                self.vision()
                self.regulate(1., 0.)
            except Exception as lower_error:
                self.node.get_logger().error(f'Controlled lowering unavailable: {lower_error}')
            raise
        finally:
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = True
            request.scene.allowed_collision_matrix = self.original_matrix
            self.service(self.scene_apply, request)
