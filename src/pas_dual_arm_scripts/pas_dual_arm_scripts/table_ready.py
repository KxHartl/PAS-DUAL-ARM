"""Stage the robot at the cube table: carriages up, wrists above the tabletop.

Both moves are ordinary trajectories on their own JointTrajectoryController -
the carriages are driven exactly like the arms. Which actuator profile is behind
them is a launch choice: the default position interface, or the effort interface
with a PID under `force_grasp:=true`. The point of this node is that it does not
trust the
action status: it reads `/joint_states` afterwards and reports the height the
carriages actually reached. That measurement is the open question in P-13.
"""

import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from pas_dual_arm_scripts.postures import POSTURES

CARRIAGES = ('torso_left_carriage_joint', 'torso_right_carriage_joint')


class TableReady(Node):
    def __init__(self):
        super().__init__('cube_table_ready')
        self.declare_parameter('carriage_height', 0.20)
        # Long enough that the carriages do not jerk on the way up.
        self.declare_parameter('carriage_seconds', 10)
        self.declare_parameter('arm_seconds', 8)
        self.positions = {}
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

    def on_joints(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def read(self, names, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self.positions for name in names):
                return [float(self.positions[name]) for name in names]
        return None

    def command(self, controller, names, positions, seconds):
        client = ActionClient(
            self, FollowJointTrajectory,
            f'/{controller}/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(f'{controller} action server unavailable')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start.sec = int(seconds)
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{controller} rejected the trajectory')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=seconds + 25.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error(f'{controller} result timed out')
            return False
        self.get_logger().info(
            f'{controller}: status={result.status} '
            f'error_code={result.result.error_code}')
        return result.status == GoalStatus.STATUS_SUCCEEDED

    def stage_carriages(self):
        height = float(self.get_parameter('carriage_height').value)
        seconds = int(self.get_parameter('carriage_seconds').value)
        before = self.read(CARRIAGES)
        if before is None:
            self.get_logger().error('no carriage positions on /joint_states')
            return False
        self.get_logger().info(
            f'carriages before: {before[0]:.4f} / {before[1]:.4f} m, '
            f'commanding {height:.3f} m over {seconds} s')
        reported = self.command('torso_controller', CARRIAGES,
                                [height, height], seconds)
        # The carriages keep creeping after the trajectory time runs out, so
        # wait until they actually stop before measuring. A fixed pause reported
        # the creep instead of the result: 3 s after the move the error still
        # reads ~3 mm, while the settled error is below 0.1 mm. The status above
        # is not the measurement, and neither is a reading taken too early.
        # Creep is slow, so compare across a window rather than between two
        # neighbouring samples: 1 mm over 10 s moves less than 0.02 mm between
        # consecutive reads and would pass a per-sample test while still moving.
        window = deque(maxlen=12)          # 3 s at 0.25 s per sample
        deadline = time.monotonic() + 40.0
        while time.monotonic() < deadline:
            settle = time.monotonic() + 0.25
            while time.monotonic() < settle:
                rclpy.spin_once(self, timeout_sec=0.05)
            current = self.read(CARRIAGES, timeout=1.0)
            if current is None:
                continue
            window.append(current)
            if len(window) == window.maxlen and all(
                    max(sample[i] for sample in window)
                    - min(sample[i] for sample in window) < 2e-5
                    for i in range(len(current))):
                break
        else:
            self.get_logger().warn('carriages never stopped creeping in 40 s')
        after = self.read(CARRIAGES, timeout=3.0) or [float('nan')] * 2
        errors = [height - value for value in after]
        self.get_logger().info(
            f'CARRIAGE MEASURED left={after[0]:.4f} right={after[1]:.4f} m '
            f'(commanded {height:.3f}, error {errors[0]:+.4f} / '
            f'{errors[1]:+.4f} m, action reported '
            f'{"SUCCEEDED" if reported else "FAILURE"})')
        if max(abs(e) for e in errors) > 0.01:
            self.get_logger().warn(
                'carriages did not reach the commanded height - this is the '
                'P-13 measurement; continuing so the run stays observable')
            return False
        return True

    def stage_arms(self):
        seconds = int(self.get_parameter('arm_seconds').value)
        ok = True
        for side in ('left', 'right'):
            joints = POSTURES['ARM_HOME'][side]
            ok = self.command(
                f'{side}_arm_controller',
                [f'{side}_joint_{index}' for index in range(1, 8)],
                [float(joints[index]) for index in range(1, 8)],
                seconds) and ok
        return ok


def main():
    rclpy.init()
    node = TableReady()
    try:
        carriages_ok = node.stage_carriages()
        arms_ok = node.stage_arms()
        node.get_logger().info(
            f'table-ready staging done (carriages={"ok" if carriages_ok else "SHORT"}, '
            f'arms={"ok" if arms_ok else "FAILED"})')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # On Ctrl-C the signal handler has already shut the context down, and
        # calling it again raises over the real exit reason.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
