#!/usr/bin/env python3
"""Velocity command selector + LiDAR safety filter in front of balance_node.

Sources (geometry_msgs/Twist, each dropped after its timeout):
  cmd_vel/teleop   operator (balance_cli, gamepad)       — wins whenever fresh
  cmd_vel/auto     autonomy (ArUco follow, avoidance, ...)
Output: cmd_vel (only while a source is fresh, so test scripts may still publish cmd_vel directly).

Safety: forward / backward speed is scaled down by the nearest LiDAR return inside the robot's
corridor (|y| < corridor_half_width) in the travel direction: full speed beyond slow_dist, 0 at
stop_dist. Turning is never limited. A stale scan caps |vx| at stale_vx_max.
Status (source, limit) on cmd_mux/status (std_msgs/String, 5 Hz).
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')
        p = self.declare_parameter
        self.teleop_to = p('teleop_timeout_s', 0.5).value
        self.auto_to = p('auto_timeout_s', 0.5).value
        self.stop_d = p('stop_dist_m', 0.35).value
        self.slow_d = p('slow_dist_m', 0.80).value
        self.half_w = p('corridor_half_width_m', 0.20).value
        self.self_r = p('self_radius_m', 0.20).value          # ignore returns from the robot itself
        self.fwd_deg = p('lidar_forward_deg', 0.0).value      # scan angle that points to the robot's front
        self.scan_to = p('scan_timeout_s', 0.5).value
        self.stale_vx = p('stale_vx_max', 0.1).value
        self.src = {'teleop': (None, 0.0), 'auto': (None, 0.0)}
        self.front = self.back = math.inf
        self.scan_t = 0.0
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.st_pub = self.create_publisher(String, 'cmd_mux/status', 10)
        self.create_subscription(Twist, 'cmd_vel/teleop', lambda m: self._in('teleop', m), 10)
        self.create_subscription(Twist, 'cmd_vel/auto', lambda m: self._in('auto', m), 10)
        self.create_subscription(LaserScan, 'scan', self._scan, qos_profile_sensor_data)
        self.status = 'idle'
        self.create_timer(0.2, lambda: self.st_pub.publish(String(data=self.status)))

    def _scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float64)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment - math.radians(self.fwd_deg)
        ok = np.isfinite(r) & (r > max(m.range_min, self.self_r)) & (r < m.range_max)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        lane = np.abs(y) < self.half_w
        f, b = x[lane & (x > 0)], -x[lane & (x < 0)]
        self.front = float(f.min()) if f.size else math.inf
        self.back = float(b.min()) if b.size else math.inf
        self.scan_t = time.monotonic()

    def _in(self, name, m):
        self.src[name] = (m, time.monotonic())
        self._out()

    def _out(self):
        now = time.monotonic()
        m, t = self.src['teleop']
        name = 'teleop'
        if m is None or now - t > self.teleop_to:
            m, t = self.src['auto']
            name = 'auto'
            if m is None or now - t > self.auto_to:
                self.status = 'idle'
                return
        vx, wz = m.linear.x, m.angular.z
        note = ''
        if now - self.scan_t > self.scan_to:
            vx = max(-self.stale_vx, min(self.stale_vx, vx))
            note = 'scan stale'
        else:
            d = self.front if vx > 0 else self.back if vx < 0 else math.inf
            k = min(1.0, max(0.0, (d - self.stop_d) / (self.slow_d - self.stop_d)))
            if k < 1.0:
                vx *= k
                note = f'obstacle {d:.2f} m x{k:.2f}'
        out = Twist()
        out.linear.x, out.angular.z = float(vx), float(wz)
        self.pub.publish(out)
        self.status = f'{name} vx {vx:+.2f} wz {wz:+.2f} front {self.front:.2f} back {self.back:.2f} {note}'


def main():
    rclpy.init()
    rclpy.spin(CmdMux())


if __name__ == '__main__':
    main()
