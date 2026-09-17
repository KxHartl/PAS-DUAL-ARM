"""Read-only recorder. Simulator-only evidence never feeds the grasp controller.

Records raw contact forces/normals/depths and estimator output for offline
qualification. A zero wrench array is recorded as unavailable, not no force.
Contact message normals and forces are in the world frame in Fortress.
Fortress publishes nothing while a pad is free, so every message written here
is a real touch; 'other' names touches that are not the box.

Also records the wrist marker detections and the wrist force-torque sensors. Those carry the force the contact
messages do not (notes/03_problemi/P-42), and are the qualification's absolute
reference. Nothing in the control path may subscribe to either.
"""
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Wrench
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String


class Recorder(Node):
    def __init__(self):
        super().__init__('grasp_force_diagnostics')
        path = Path(self.declare_parameter('output', 'log/grasp-force.jsonl').value)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.output = path.open('x', encoding='utf-8')
        for side in ('left', 'right'):
            for finger in ('left', 'right'):
                key = f'{side}_{finger}'
                self.create_subscription(Contacts, f'/contact/{key}_tip',
                                         lambda msg, key=key: self.contact(key, msg), 10)
        for side in ('left', 'right'):
            self.create_subscription(Wrench, f'/ft/{side}_wrist',
                                     lambda msg, key=side: self.wrench(key, msg), 50)
        for side in ('left', 'right'):
            self.create_subscription(PoseStamped, f'/wrist_{side}/marker_pose',
                                     lambda msg, key=side: self.marker(key, msg), 20)
        for topic in ('estimator_status', 'events'):
            self.create_subscription(String, f'/grasp/{topic}',
                                     lambda msg, key=topic: self.write(key, json.loads(msg.data)), 10)

    def write(self, kind, data):
        self.output.write(json.dumps(dict(
            stamp=self.get_clock().now().nanoseconds * 1e-9, kind=kind, data=data)) + '\n')
        self.output.flush()

    def wrench(self, side, msg):
        # Wrist force-torque, in the joint's child frame. The real Gen3 has no
        # such sensor; this is evidence for the offline qualification only.
        self.write('wrist_ft', dict(
            side=side,
            force=[msg.force.x, msg.force.y, msg.force.z],
            torque=[msg.torque.x, msg.torque.y, msg.torque.z]))

    def marker(self, side, msg):
        # The wrist detector publishes only when it actually finds its marker, so
        # the GAPS between these records are the measurement: force_grasp aborts
        # when the marker TF is older than force_vision_age. Recording the pose
        # stamp separates 'the camera stopped' from 'the detector lost the
        # marker', which the abort message alone cannot tell apart.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        self.write('marker_pose', dict(side=side, stamp=stamp,
                                       position=[p.x, p.y, p.z]))

    def contact(self, key, msg):
        def xyz(v):
            return [v.x, v.y, v.z]
        entries, other = [], set()
        for contact in msg.contacts:
            a, b = contact.collision1.name, contact.collision2.name
            if ('aruco_box' in a) == ('aruco_box' in b):
                # Anything else pressing this pad (the table, the other hand)
                # is recorded, not dropped: a silent pad must mean a free pad,
                # and naming what is pressing turns a rejected sample into a
                # usable clue instead of an unexplained gap.
                other.add(f'{a} | {b}')
                continue
            force = []
            for wrench in contact.wrenches:
                # Choose force ON the hand, matching the estimator convention.
                hand = wrench.body_2_wrench if 'aruco_box' in a else wrench.body_1_wrench
                force.append(xyz(hand.force))
            entries.append(dict(collision1=a, collision2=b, forces=force,
                                normals=[xyz(n) for n in contact.normals],
                                positions=[xyz(p) for p in contact.positions],
                                depths=list(contact.depths)))
        self.write('contact', dict(pad=key, contacts=entries, other=sorted(other)))


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    finally:
        node.output.close()
        node.destroy_node()
        rclpy.shutdown()
