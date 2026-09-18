#!/usr/bin/env python3
"""Drive a fixed open-loop figure so the yaw estimators can be compared.

The mission is not a good measuring stick for heading: it takes ten minutes,
fails a third of the time, and spends most of it standing still. This drives a
short sequence that contains the motions heading error comes from - straight
runs, turns, and the sideways motion the wheels cannot see - and nothing else.

Open loop on purpose. Nothing here needs to arrive anywhere; the point is that
Gazebo knows exactly where the robot went, so every estimator can be scored
against it afterwards from the same bag.

    ./scripts/run_native.sh python3 scripts/yaw_drive.py
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter

# (seconds, vx, vy, wz) in the robot's frame. Total 46 s of sim time.
LEGS = [
    (3.0,  0.00, 0.00, 0.00),   # settle, so the first samples are at rest
    (8.0,  0.25, 0.00, 0.00),   # 2.0 m straight: the doorway transit's length
    (4.0,  0.00, 0.00, 0.40),   # +92 deg
    (6.0,  0.25, 0.00, 0.00),   # 1.5 m straight on the new heading
    (4.0,  0.00, 0.20, 0.00),   # 0.8 m sideways: invisible to the wheels
    (4.0,  0.00, 0.00, -0.40),  # -92 deg, back to the first heading
    (8.0,  0.25, 0.00, 0.00),   # 2.0 m straight again
    (4.0,  0.15, 0.10, 0.15),   # everything at once, which is how it really drives
    (5.0,  0.00, 0.00, 0.00),   # stop, and let the estimators settle
]


class YawDrive(Node):
    def __init__(self):
        # Sim time, so the legs below cover the distances they claim to: the
        # simulation runs at about half real time, and on the wall clock each
        # leg would drive half as far as its comment says.
        super().__init__('yaw_drive', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.leg = 0
        self.started = None
        self.create_timer(0.05, self._tick)

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.started is None:
            self.started = now
            self.get_logger().info(f'leg 1/{len(LEGS)}')
        duration, vx, vy, wz = LEGS[self.leg]
        if now - self.started >= duration:
            self.leg += 1
            self.started = now
            if self.leg >= len(LEGS):
                self.pub.publish(Twist())
                self.get_logger().info('done')
                raise SystemExit(0)
            duration, vx, vy, wz = LEGS[self.leg]
            self.get_logger().info(
                f'leg {self.leg + 1}/{len(LEGS)}: vx={vx} vy={vy} wz={wz} for {duration}s')
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = YawDrive()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
