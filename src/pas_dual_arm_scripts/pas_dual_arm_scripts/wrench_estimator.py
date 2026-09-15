"""Gen3 joint-torque estimator. Inputs deliberately exclude Gazebo contacts."""
import hashlib
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation

from .force_model import ArmModel, estimate


class WrenchEstimator(Node):
    def __init__(self):
        super().__init__('grasp_wrench_estimator')
        defaults = dict(max_age=0.15, max_future=0.05, max_joint_speed=0.08, max_bias=2.0,
                        damping=0.002, condition_limit=100.0, residual_limit=0.2,
                        filter_tau=0.1, effort_sign=1.0, gravity_frame='odom',
                        zero_seconds=2.0, max_zero_std=0.05)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if self.cfg['effort_sign'] not in (-1.0, 1.0):
            raise ValueError('effort_sign must be +1 or -1')
        self.models, self.samples, self.bias, self.filtered = {}, {}, {}, {}
        self.zero, self.zero_samples = None, {}
        self.last_processed = {}
        self.fingerprint = ''
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.wrench = {s: self.create_publisher(WrenchStamped, f'/grasp/{s}/wrench', 10)
                       for s in ('left', 'right')}
        self.status = self.create_publisher(String, '/grasp/estimator_status', 10)
        self.create_subscription(String, '/robot_description', self.description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_service(Trigger, '/grasp/zero_wrench', self.start_zero)
        self.create_timer(0.02, self.tick)

    def description(self, msg):
        if self.models:
            return
        try:
            self.models = {s: ArmModel(msg.data, s) for s in ('left', 'right')}
            self.fingerprint = hashlib.sha256(
                (msg.data + json.dumps(self.cfg, sort_keys=True)).encode()).hexdigest()
        except Exception as exc:
            self.get_logger().error(f'Cannot construct force model: {exc}')

    def joints(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for i, name in enumerate(msg.name):
            if i < len(msg.position) and i < len(msg.velocity) and i < len(msg.effort):
                self.samples[name] = (msg.position[i], msg.velocity[i], msg.effort[i], stamp,
                                      time.monotonic())
            else:
                self.samples.pop(name, None)

    def start_zero(self, _request, response):
        # Caller must have established a collision-free, stationary pregrasp.
        self.bias.clear()
        self.filtered.clear()
        self.zero_samples = {s: [] for s in ('left', 'right')}
        self.zero = self.get_clock().now().nanoseconds * 1e-9
        response.success = True
        response.message = 'Zero requested; wait for calibrated=true on estimator_status'
        return response

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        report = {'stamp': now, 'fingerprint': self.fingerprint, 'arms': {}}
        for side in ('left', 'right'):
            state = {'valid': False, 'calibrated': side in self.bias}
            try:
                model = self.models[side]
                data = np.array([self.samples[n][:4] for n in model.names])
                stamps = data[:, 3]
                age = now - stamps.min()
                # Stale data is a safety question; a stamp slightly AHEAD of this
                # node's clock is not. /clock and /joint_states are separate
                # topics with no ordering guarantee between them, so a joint
                # message stamped at t routinely arrives before the /clock
                # message for t. Measured on this sim (/clock at 1 kHz): the
                # stamp leads by up to 19 ms on 3.6% of messages. A 10 ms
                # tolerance rejected those as if the data were bad.
                if age > self.cfg['max_age']:
                    raise ValueError(f'joint stamp {age:.3f} s old')
                if age < -self.cfg['max_future']:
                    raise ValueError(f'joint stamp {-age:.3f} s in the future')
                if max(time.monotonic() - self.samples[n][4] for n in model.names) > 0.5:
                    raise ValueError('joint stream stopped')
                if not np.isfinite(data).all():
                    raise ValueError('non-finite joint state')
                if np.max(np.abs(data[:, 1])) > self.cfg['max_joint_speed']:
                    raise ValueError('moving too fast for quasistatic estimation')
                knuckle = self.samples[f'{side}_robotiq_85_left_knuckle_joint']
                if abs(knuckle[0]) > 0.025 or now - knuckle[3] > self.cfg['max_age']:
                    raise ValueError('gripper not confirmed open')
                transform = self.tf.lookup_transform(
                    model.base, self.cfg['gravity_frame'], rclpy.time.Time(),
                    timeout=Duration(seconds=0.0))
                q = transform.transform.rotation
                gravity = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0., 0., -9.81])
                jac, g = model.terms(data[:, 0], gravity)
                measured = data[:, 2] * self.cfg['effort_sign']
                stamp = float(stamps.min())
                fresh = stamp > self.last_processed.get(side, -1.)
                self.last_processed[side] = stamp
                state.update(stamp=stamp, measured=measured.tolist(), gravity=g.tolist())
                state['world_to_base'] = [q.x, q.y, q.z, q.w]
                if self.zero is not None and side not in self.bias:
                    if fresh:
                        self.zero_samples[side].append(measured - g)
                    if now - self.zero >= self.cfg['zero_seconds']:
                        samples = np.array(self.zero_samples[side])
                        if len(samples) < 20:
                            raise ValueError('too few independent zero samples')
                        if np.max(samples.std(axis=0)) > self.cfg['max_zero_std']:
                            raise ValueError('zero unstable; stop motion and retry')
                        bias = samples.mean(axis=0)
                        if np.max(np.abs(bias)) > self.cfg['max_bias']:
                            raise ValueError('zero bias too large: verify torque sign/model')
                        self.bias[side] = bias
                    if now - self.zero > self.cfg['zero_seconds'] + 1.0 and side not in self.bias:
                        self.zero = None
                if side not in self.bias:
                    raise ValueError('not calibrated')
                wrench, condition, residual = estimate(
                    jac, g, measured, self.bias[side], damping=self.cfg['damping'],
                    condition_limit=self.cfg['condition_limit'],
                    residual_limit=self.cfg['residual_limit'])
                if fresh:
                    previous, t = self.filtered.get(side, (wrench, stamp - 0.02))
                    dt = max(0., stamp - t)
                    alpha = dt / (self.cfg['filter_tau'] + dt)
                    self.filtered[side] = (previous + alpha * (wrench - previous), stamp)
                filtered = self.filtered.get(side, (wrench, stamp))[0]
                state.update(valid=True, calibrated=True, reason='ok', condition=condition,
                             residual=residual, raw=wrench.tolist(), wrench=filtered.tolist())
                msg = WrenchStamped()
                msg.header.frame_id = model.base
                msg.header.stamp = rclpy.time.Time(seconds=stamp).to_msg()
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = filtered[:3]
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = filtered[3:]
                self.wrench[side].publish(msg)
            except Exception as exc:
                state['reason'] = str(exc)
                self.filtered.pop(side, None)
            report['arms'][side] = state
        self.status.publish(String(data=json.dumps(report)))


def main():
    rclpy.init()
    node = WrenchEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
