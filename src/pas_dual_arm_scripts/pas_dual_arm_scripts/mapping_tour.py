"""Scripted drive through the three rooms so slam_toolbox can map them.

  ros2 run pas_dual_arm_scripts mapping_tour

Route design is shaped entirely by P-11: localisation once jumped ~30 m because
the skid-steer base turns by sliding its wheels (P-10 sets mu2 = 0 so it can turn
at all), which makes wheel odometry lie about yaw and breaks scan matching.

Three consequences, all visible in the route below:

  * **No spinning to look around.** The lidar is a full 360 deg sensor with 12 m
    of usable range in 6 x 6 m rooms, so standing still already sees every wall.
    The old failure was a 360 deg scan-for-marker spin; there is no reason to
    repeat it.
  * **Come back in reverse, do not turn around.** Reversing along the path just
    driven costs no yaw slip at all. That takes the tour down to *two* in-place
    turns of 90 deg each, instead of one per direction change.
  * **Return to the origin twice.** Each return is a loop closure the graph can
    use to pull the three rooms into agreement.

The tour drives open-loop on odometry and never reads the map - slam_toolbox is
free to correct the map underneath it. What the tour does report, per leg, is
commanded vs odometry-measured motion, plus the map->odom correction at every
waypoint. That correction is the P-11 acceptance criterion: it should creep, not
jump. A jump > 0.2 m means the run failed even if the picture looks plausible.
"""

import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener

from pas_dual_arm_scripts.base_drive import BaseDriver
from pas_dual_arm_scripts.postures import (ARM_DRIVE, POSTURES, move_to_posture,
                                           switch_arm_hold)

# Legs are (kind, value, label). Distances in metres, angles in radians.
# Starting pose is the spawn pose: (0, 0) facing +X, in the HOME room.
# Doorways are at (0, -3) to BLUE and (3, 0) to RED, both 1.0 m wide.
#
# Stopping distances are set so the robot body keeps ~1 m from the furniture:
# the box sits at (0, -6.38) with its near face at y = -6.23, and place_table at
# (6.5, 0) with its near face at x = 6.2. The body reaches ~0.7 m ahead of
# base_link, so stopping the base 4.5 m out leaves ~1 m of air.
ROUTE = [
    ('pause', 3.0, 'seed HOME'),
    ('turn', -math.pi / 2, 'face the BLUE doorway (-Y)'),
    ('drive', 4.5, 'HOME -> through the BLUE doorway -> BLUE room'),
    ('pause', 3.0, 'seed BLUE'),
    ('drive', -4.5, 'reverse back to the origin (no turn, no yaw slip)'),
    ('pause', 2.0, 'loop closure at the origin'),
    ('turn', math.pi / 2, 'face the RED doorway (+X)'),
    ('drive', 4.5, 'HOME -> through the RED doorway -> RED room'),
    ('pause', 3.0, 'seed RED'),
    ('drive', -4.5, 'reverse back to the origin'),
    ('pause', 3.0, 'final loop closure'),
]

CHUNK = 0.5          # m per driving chunk, so the scan gate gets a say often
CLEAR_SECTOR = 0.44  # rad (+-25 deg) around the direction of travel


class MappingTour(BaseDriver, Node):
    def __init__(self):
        super().__init__('mapping_tour')
        self.declare_parameter('tuck_arms', True)
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('turn_rate', 0.25)
        self.declare_parameter('stop_distance', 0.6)
        self.declare_parameter('probe_distance', 0.0)
        self.declare_parameter('probe_turn_deg', 0.0)

        self.speed = self.get_parameter('speed').value
        self.turn_rate = self.get_parameter('turn_rate').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.probe_distance = float(self.get_parameter('probe_distance').value)
        self.probe_turn_deg = float(self.get_parameter('probe_turn_deg').value)

        self.init_base_drive()
        self._scan = None
        self._joints = {}
        self.create_subscription(LaserScan, '/scan_filtered', self._scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _scan_cb(self, msg):
        self._scan = msg

    def _joint_cb(self, msg):
        self._joints = dict(zip(msg.name, msg.position))

    def _posture_ok(self):
        for side, joints in POSTURES[ARM_DRIVE].items():
            for index, target in joints.items():
                joint = f'{side}_joint_{index}'
                actual = self._joints.get(joint)
                if actual is None:
                    return False
                error = math.atan2(math.sin(actual - target),
                                   math.cos(actual - target))
                if abs(error) > 0.15:
                    self.get_logger().error(
                        f'{joint} sagged by {error:.3f} rad; travel posture unsafe')
                    return False
        return True

    # ------------------------------------------------------------ safety gate
    def _clearance(self, forward=True):
        """Nearest obstacle in the sector we are about to drive into, or None.

        Returns None when no scan has arrived yet; the caller treats that as a
        reason to stop rather than to assume the way is clear.
        """
        deadline = time.monotonic() + 2.0
        self._scan = None
        while self._scan is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        scan = self._scan
        if scan is None:
            return None
        centre = 0.0 if forward else math.pi
        best = math.inf
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min:
                continue
            a = scan.angle_min + i * scan.angle_increment
            d = math.atan2(math.sin(a - centre), math.cos(a - centre))
            if abs(d) <= CLEAR_SECTOR:
                best = min(best, r)
        # All-infinite ranges mean open space, not a missing scan.
        return best

    # ------------------------------------------------------------------- legs
    def _run_drive(self, metres, label):
        """Drive `metres` in CHUNK steps, checking the way ahead between steps."""
        forward = metres >= 0
        remaining = abs(metres)
        travelled = 0.0
        while remaining > 1e-3:
            if not self._posture_ok():
                return None
            clear = self._clearance(forward)
            if clear is None:
                self.get_logger().error(f'{label}: no /scan_filtered - refusing to drive blind')
                return None
            if clear < self.stop_distance:
                self.get_logger().error(
                    f'{label}: obstacle at {clear:.2f} m '
                    f'(< {self.stop_distance:.2f} m) - stopping, tour aborted')
                return None
            step = min(CHUNK, remaining)
            moved = self.drive_distance(math.copysign(step, metres), self.speed)
            if moved is None:
                self.get_logger().error(f'{label}: no odometry - tour aborted')
                return None
            travelled += moved
            remaining -= step
        return travelled

    def _run_turn(self, radians, label):
        turned = self.turn_angle(radians, self.turn_rate)
        if turned is None:
            self.get_logger().error(f'{label}: no odometry - tour aborted')
        return turned

    # --------------------------------------------------------------- reporting
    def _map_odom(self):
        """The SLAM correction map->odom. Should creep, never jump (P-11)."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time()).transform.translation
        except Exception:
            return None
        return (t.x, t.y)

    def _report_correction(self, label):
        mo = self._map_odom()
        if mo is None:
            self.get_logger().warn(f'{label}: map->odom not available yet')
            return None
        self.get_logger().info(f'{label}: map->odom = ({mo[0]:+.3f}, {mo[1]:+.3f}) m')
        return mo

    def _settle(self, secs, label):
        """Hold still on SIM time so scan matching can converge before moving on."""
        self.get_logger().info(f'{label}: holding {secs:.1f} s')
        self.drive(0.0, 0.0, secs)

    # -------------------------------------------------------------------- tour
    def run(self):
        # Ensure DetachableJoint is released so the box sits freely on its table.
        # Ignition's DetachableJoint plugin defaults to attached at startup.
        self.get_logger().info('Ensuring aruco_box is detached from left wrist...')
        subprocess.run(
            ['ign', 'topic', '-t', '/aruco_box/detach',
             '-m', 'ignition.msgs.Empty', '-p', 'unused: true'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        if self.get_parameter('tuck_arms').value:
            if not move_to_posture(self, ARM_DRIVE, label='mapping tour', freeze=True):
                self.get_logger().error(
                    'could not tuck the arms; refusing to drive. After spawn the '
                    'arms are straight out and the robot is 2.28 m wide, which '
                    'does not fit through a 1.0 m doorway.')
                return False

        previous = self._report_correction('start')
        worst_jump = 0.0

        if self.probe_distance:
            if not 0.05 <= self.probe_distance <= 0.7:
                self.get_logger().error('probe_distance must be 0.05..0.7 m')
                return False
            self.get_logger().info(f'HOME straight probe: {self.probe_distance:.2f} m')
            moved = self._run_drive(self.probe_distance, 'HOME straight probe')
            if moved is None:
                return False
            self.get_logger().info(
                f'HOME straight probe completed: requested {self.probe_distance:.2f} m, '
                f'wheel odometry {moved:.2f} m')
            return abs(moved - self.probe_distance) <= 0.08

        if self.probe_turn_deg:
            if not 5.0 <= abs(self.probe_turn_deg) <= 90.0:
                self.get_logger().error('probe_turn_deg must be 5..90 degrees')
                return False
            radians = math.radians(self.probe_turn_deg)
            self.get_logger().info(f'HOME turn probe: {self.probe_turn_deg:+.1f} deg')
            turned = self._run_turn(radians, 'HOME turn probe')
            if turned is None:
                return False
            self.get_logger().info(
                f'HOME turn probe completed: requested {self.probe_turn_deg:+.1f} deg, '
                f'wheel odometry {math.degrees(turned):+.1f} deg')
            return abs(turned - radians) <= math.radians(3.0)

        for kind, value, label in ROUTE:
            if kind == 'pause':
                self._settle(value, label)
            elif kind == 'drive':
                self.get_logger().info(f'{label}: driving {value:+.2f} m')
                moved = self._run_drive(value, label)
                if moved is None:
                    return False
                self.get_logger().info(
                    f'{label}: commanded {abs(value):.2f} m, odometry {moved:.2f} m '
                    f'(error {moved - abs(value):+.2f} m)')
            elif kind == 'turn':
                self.get_logger().info(f'{label}: turning {math.degrees(value):+.1f} deg')
                turned = self._run_turn(value, label)
                if turned is None:
                    return False
                self.get_logger().info(
                    f'{label}: commanded {math.degrees(value):+.1f} deg, odometry '
                    f'{math.degrees(turned):+.1f} deg '
                    f'(error {math.degrees(turned - value):+.1f} deg)')
            else:
                self.get_logger().error(f'unknown leg kind {kind!r}')
                return False

            current = self._report_correction(label)
            if previous is not None and current is not None:
                jump = math.hypot(current[0] - previous[0], current[1] - previous[1])
                worst_jump = max(worst_jump, jump)
                if jump > 0.2:
                    self.get_logger().error(
                        f'{label}: map->odom jumped {jump:.3f} m in one leg - '
                        'this is the P-11 failure mode, the map is suspect')
                    return False
            if current is not None:
                previous = current

        self.get_logger().info(
            f'tour finished. Largest single-leg map->odom jump: {worst_jump:.3f} m '
            f'({"OK" if worst_jump <= 0.2 else "TOO LARGE - see P-11"})')
        self.get_logger().info('save the map with ./scripts/save_map.sh')
        return worst_jump <= 0.2


def main():
    rclpy.init()
    node = MappingTour()
    try:
        ok = node.run()
    except KeyboardInterrupt:
        if rclpy.ok():
            node._send_vel(0.0, 0.0)
        ok = False
    if getattr(node, '_arms_frozen', False) and rclpy.ok():
        switch_arm_hold(node, False)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
