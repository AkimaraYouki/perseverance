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
import os
import time

import numpy as np
import rclpy
import rclpy.executors
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
        self.limit_on = p('lidar_limit', True).value          # false: pass commands through (scan only logged)
        log_dir = p('log_dir', os.path.expanduser('~/gen2_ws/logs/steps')).value
        self.front_ang = self.back_ang = float('nan')
        try:
            os.makedirs(log_dir, exist_ok=True)
            self.log = open(os.path.join(log_dir, time.strftime('%Y-%m-%d_%H%M%S') + '_mux.csv'), 'w', buffering=1)
            self.log.write('t,src,vx_in,wz_in,vx_out,front,front_deg,back,back_deg,scale,note\n')
        except OSError:
            self.log = None
        self.src = {'teleop': (None, 0.0), 'auto': (None, 0.0)}
        self.front = self.back = math.inf
        self.scan_t = 0.0
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.st_pub = self.create_publisher(String, 'cmd_mux/status', 10)
        self.create_subscription(Twist, 'cmd_vel/teleop', lambda m: self._in('teleop', m), 10)
        self.create_subscription(Twist, 'cmd_vel/auto', lambda m: self._in('auto', m), 10)
        self.create_subscription(LaserScan, 'scan', self._scan, qos_profile_sensor_data)
        self.status = 'idle'
        self.add_on_set_parameters_callback(self._set)   # `ros2 param set /cmd_mux lidar_limit false` works live
        self.create_timer(0.2, lambda: self.st_pub.publish(String(data=self.status)))

    def _set(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for q in params:
            if q.name == 'lidar_limit':
                self.limit_on = bool(q.value)
                self.get_logger().info(f'lidar_limit -> {self.limit_on}')
        return SetParametersResult(successful=True)

    def _scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float64)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment - math.radians(self.fwd_deg)
        ok = np.isfinite(r) & (r > max(m.range_min, self.self_r)) & (r < m.range_max)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        lane = np.abs(y) < self.half_w
        fm, bm = lane & (x > 0), lane & (x < 0)
        f, b = x[fm], -x[bm]
        self.front = float(f.min()) if f.size else math.inf
        self.back = float(b.min()) if b.size else math.inf
        aa = np.degrees(a[ok])
        self.front_ang = float(aa[fm][np.argmin(f)]) if f.size else float('nan')
        self.back_ang = float(aa[bm][np.argmin(b)]) if b.size else float('nan')
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
        vx_in, k = vx, 1.0
        note = ''
        if now - self.scan_t > self.scan_to:
            vx = max(-self.stale_vx, min(self.stale_vx, vx))
            note = 'scan stale'
        else:
            d = self.front if vx > 0 else self.back if vx < 0 else math.inf
            k = min(1.0, max(0.0, (d - self.stop_d) / (self.slow_d - self.stop_d)))
            if k < 1.0 and self.limit_on:
                vx *= k
                note = f'obstacle {d:.2f} m x{k:.2f}'
        out = Twist()
        out.linear.x, out.angular.z = float(vx), float(wz)
        self.pub.publish(out)
        self.status = f'{name} vx {vx:+.2f} wz {wz:+.2f} front {self.front:.2f} back {self.back:.2f} {note}'
        if self.log:
            self.log.write(f'{now:.3f},{name},{vx_in:.3f},{wz:.3f},{vx:.3f},{self.front:.3f},{self.front_ang:.1f},'
                           f'{self.back:.3f},{self.back_ang:.1f},{k:.2f},{note}\n')


def main():
    rclpy.init()
    try:
        rclpy.spin(CmdMux())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
