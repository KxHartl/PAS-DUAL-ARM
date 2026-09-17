#!/usr/bin/env python3
"""Find which link of the scan chain dies mid-run.

Run 17 Sep (auto mapping) froze at sim 8.69 s: slam_toolbox stopped refreshing
map -> odom, the collision monitor kept asking to transform a scan stamped
8.69 s, and the map stopped growing - three consumers of /scan_filtered, one
cause. scan_filter's own log is empty, so it was not dropping scans on a failed
transform; it stopped being fed. Physics, /clock and odom -> base_footprint kept
running throughout, and the Ignition render thread neither errored nor exited.

That leaves three candidates and the logs cannot separate them:

    Ignition gpu_lidar  ->  ros_gz bridge  ->  /scan  ->  scan_filter  ->  /scan_filtered

This node watches both ROS topics from the outside and, the moment /scan goes
quiet, asks Ignition directly whether it is still publishing. Whichever side is
silent is the side that broke.

Deliberately on the wall clock (use_sim_time is never set here): the failure
being measured stops sim-time-driven work, and a watchdog that freezes with its
subject reports nothing.

    ros2 run pas_dual_arm_scripts scan_watch     # if installed
    python3 scripts/scan_watch.py                # from the project root

Start it in its own terminal before launching the scenario; it waits.
"""
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

STALL = 2.0          # wall seconds of silence that count as a stall, not a hiccup
IGN_PROBE_TIMEOUT = 5.0


class ScanWatch(Node):
    def __init__(self):
        super().__init__('scan_watch')
        self._start = time.monotonic()
        self._count = {'scan': 0, 'filtered': 0}
        self._last = {'scan': None, 'filtered': None}
        self._stamp = {'scan': None, 'filtered': None}
        self._sim = None
        self._probed = False
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: self._on('scan', m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_filtered',
                                 lambda m: self._on('filtered', m), 10)
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        self.create_timer(1.0, self._tick)
        print('scan_watch: waiting for /scan and /scan_filtered '
              '(wall clock, 1 line per second)', flush=True)

    def _on(self, key, msg):
        self._count[key] += 1
        self._last[key] = time.monotonic()
        self._stamp[key] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_clock(self, msg):
        self._sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _age(self, key):
        return None if self._last[key] is None else time.monotonic() - self._last[key]

    def _tick(self):
        wall = time.monotonic() - self._start
        scan_hz, filt_hz = self._count['scan'], self._count['filtered']
        self._count['scan'] = self._count['filtered'] = 0
        sim = f'{self._sim:8.2f}' if self._sim is not None else '    n/a '
        parts = [f'[{wall:6.1f}s wall] sim {sim}',
                 f'/scan {scan_hz:3d} Hz', f'/scan_filtered {filt_hz:3d} Hz']
        for key, label in (('scan', 'scan'), ('filtered', 'filt')):
            age, stamp = self._age(key), self._stamp[key]
            if age is not None and age > STALL:
                parts.append(f'{label} SILENT {age:.1f} s (last stamp {stamp:.2f})')
        print('  '.join(parts), flush=True)

        scan_age = self._age('scan')
        if not self._probed and scan_age is not None and scan_age > STALL:
            self._probed = True
            self._probe_ignition()

    def _probe_ignition(self):
        """/scan is quiet on the ROS side. Is the simulator still producing it?"""
        print('\n--- /scan stalled; asking Ignition directly ---', flush=True)
        try:
            out = subprocess.run(['ign', 'topic', '-e', '-t', '/scan', '-n', '1'],
                                 capture_output=True, text=True, timeout=IGN_PROBE_TIMEOUT)
            alive = bool(out.stdout.strip())
        except subprocess.TimeoutExpired:
            alive = False
        except FileNotFoundError:
            print('  `ign` not on PATH - run this from the project environment '
                  '(scripts/run_native.sh)', flush=True)
            return
        if alive:
            print('  VERDICT: Ignition IS still publishing /scan.\n'
                  '           The break is the ros_gz bridge or DDS delivery, '
                  'not the simulated lidar.\n', flush=True)
        else:
            print('  VERDICT: Ignition is NOT publishing /scan either '
                  f'(silent for {IGN_PROBE_TIMEOUT:.0f} s).\n'
                  '           The break is inside the simulator: the sensor stopped '
                  'updating while physics ran on.\n', flush=True)


def main():
    rclpy.init()
    node = ScanWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
