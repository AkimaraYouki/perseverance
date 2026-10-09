#!/usr/bin/env python3
"""Republish /scan as /scan_level only while the body is near level (2D SLAM assumes a level LiDAR).

A wheel-legged robot pitches 5-25 deg when it accelerates; the LiDAR plane then cuts the floor a
metre or two away and slam_toolbox would draw it as a wall. Scans taken with |pitch| or |roll| above
max_tilt_deg are dropped. The body attitude is read from the IMU shared memory (/dev/shm/gen2_imu,
written by iahrs_node) only when a scan arrives: subscribing /imu/data at 500 Hz in Python cost ~55 % CPU.
"""

import math
import mmap
import struct

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanGate(Node):
    def __init__(self):
        super().__init__('scan_gate')
        self.max_tilt = math.radians(self.declare_parameter('max_tilt_deg', 5.0).value)
        self.pitch = self.roll = 0.0
        self.n_in = self.n_out = 0
        self.pub = self.create_publisher(LaserScan, 'scan_level', qos_profile_sensor_data)
        self.shm = None
        self.create_subscription(LaserScan, 'scan', self._scan, qos_profile_sensor_data)
        self.create_timer(10.0, self._report)

    def _attitude(self):
        # ImuShm: uint32 seq @0, sample @8 = int64 rx_ns, int64 count, double q[4] (w x y z, body) @24
        try:
            if self.shm is None:
                with open('/dev/shm/gen2_imu', 'rb') as f:
                    self.shm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
            for _ in range(5):
                s0 = struct.unpack_from('<I', self.shm, 0)[0]
                w, x, y, z = struct.unpack_from('<4d', self.shm, 24)
                if s0 % 2 == 0 and struct.unpack_from('<I', self.shm, 0)[0] == s0:
                    self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
                    self.roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
                    return True
        except (OSError, ValueError):
            self.shm = None
        return False

    def _scan(self, m):
        self.n_in += 1
        if not self._attitude():
            self.get_logger().warn('IMU shared memory unavailable: passing scans unfiltered', throttle_duration_sec=10.0)
            self.pub.publish(m)
            return
        if abs(self.pitch) < self.max_tilt and abs(self.roll) < self.max_tilt:
            self.n_out += 1
            self.pub.publish(m)

    def _report(self):
        if self.n_in:
            self.get_logger().info(f'passed {self.n_out}/{self.n_in} scans (|tilt| < {math.degrees(self.max_tilt):.0f} deg)')
        self.n_in = self.n_out = 0


def main():
    rclpy.init()
    try:
        rclpy.spin(ScanGate())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
