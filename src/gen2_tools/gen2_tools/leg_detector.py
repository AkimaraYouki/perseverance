#!/usr/bin/env python3
"""Person (leg-pair) detection from the 2D LiDAR (C1 at ~12 cm sees shins / ankles).

/scan (raw, 10 Hz: every scan is used, also while the body pitches) -> own self-hitbox mask + floor cut (the plane
of a pitched LiDAR meets the floor at H/tan(pitch): returns beyond 85 % of that in the tilt direction are dropped;
body attitude from the IMU shared memory) -> split into clusters where neighbouring returns jump by more
than split_m -> keep leg-sized clusters (width leg_min..leg_max) -> pair legs whose centres are pair_min..pair_max
apart (a single leg also counts: legs often occlude each other) -> people candidates in the laser frame.
Simple nearest-neighbour tracks with a constant-velocity filter keep an id per candidate.

Out: people/legs (geometry_msgs/PoseArray, frame laser: one pose per person candidate, orientation = velocity heading),
     people/debug (std_msgs/String). Range limited to max_range_m.
"""

import math
import mmap
import struct
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Track:
    def __init__(self, tid, p, t):
        self.id, self.p, self.v, self.t, self.hits, self.seen = tid, np.array(p), np.zeros(2), t, 1, t

    def predict(self, t):
        return self.p + self.v * (t - self.t)

    def update(self, p, t):
        dt = max(1e-3, t - self.t)
        pred = self.predict(t)
        self.v = 0.7 * self.v + 0.3 * (np.array(p) - self.p) / dt
        self.p = 0.5 * pred + 0.5 * np.array(p)
        self.t = self.seen = t
        self.hits += 1


class LegDetector(Node):
    def __init__(self):
        super().__init__('leg_detector')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.split = p('split_m', 0.10)
        self.leg_min, self.leg_max = p('leg_min_m', 0.04), p('leg_max_m', 0.25)
        self.pair_min, self.pair_max = p('pair_min_m', 0.08), p('pair_max_m', 0.50)
        self.max_range = p('max_range_m', 4.0)
        self.gate = p('track_gate_m', 0.5)
        self.keep_s = p('track_keep_s', 1.0)
        self.tracks, self.next_id = [], 1
        self.hb = p('hitbox_laser_xy', [-0.144, 0.239, -0.164, 0.164])
        self.h_lidar = p('lidar_height_m', 0.37)        # above the floor when standing (wheel r + leg h + mount)
        self.shm = None
        self.pub = self.create_publisher(PoseArray, 'people/legs', 10)
        self.dbg = self.create_publisher(String, 'people/debug', 10)
        self.create_subscription(LaserScan, 'scan', self._scan, qos_profile_sensor_data)

    def _tilt(self):
        try:
            if self.shm is None:
                with open('/dev/shm/gen2_imu', 'rb') as f:
                    self.shm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
            w, x, y, z = struct.unpack_from('<4d', self.shm, 24)
            return (math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))),
                    math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
        except (OSError, ValueError):
            self.shm = None
            return 0.0, 0.0

    def _clusters(self, m):
        r = np.asarray(m.ranges, dtype=np.float64)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < self.max_range)
        xy = np.stack([r * np.cos(a), r * np.sin(a)], 1)
        hb = self.hb
        ok &= ~((xy[:, 0] > hb[0]) & (xy[:, 0] < hb[1]) & (xy[:, 1] > hb[2]) & (xy[:, 1] < hb[3]))
        pitch, roll = self._tilt()               # pitch + = nose down -> plane hits the floor ahead
        for ang, axis, sgn in ((pitch, 0, 1.0), (roll, 1, -1.0)):   # roll + = right side low -> floor on the right (-y)
            if abs(ang) > math.radians(1.0):
                d = 0.85 * self.h_lidar / math.tan(abs(ang))
                ok &= ~(sgn * math.copysign(1.0, ang) * xy[:, axis] > d)
        out, cur = [], []
        for i in range(len(r)):
            if not ok[i]:
                if cur:
                    out.append(cur)
                cur = []
                continue
            if cur and np.linalg.norm(xy[i] - xy[cur[-1]]) > self.split:
                out.append(cur)
                cur = []
            cur.append(i)
        if cur:
            out.append(cur)
        legs = []
        for c in out:
            if len(c) < 2:
                continue
            pts = xy[c]
            w = np.linalg.norm(pts[-1] - pts[0])
            if self.leg_min <= w <= self.leg_max:
                legs.append(pts.mean(0))
        return legs

    def _scan(self, m):
        t = time.monotonic()
        legs = self._clusters(m)
        used, people = set(), []
        for i in range(len(legs)):                       # pair legs (closest partner first)
            if i in used:
                continue
            best, bj = None, None
            for j in range(i + 1, len(legs)):
                if j in used:
                    continue
                d = np.linalg.norm(legs[i] - legs[j])
                if self.pair_min <= d <= self.pair_max and (best is None or d < best):
                    best, bj = d, j
            if bj is not None:
                used |= {i, bj}
                people.append((legs[i] + legs[bj]) / 2)
            else:
                people.append(legs[i])                   # single leg (the other hidden behind it)
        # tracks: greedy nearest neighbour
        free = list(range(len(people)))
        for tr in self.tracks:
            if not free:
                break
            pred = tr.predict(t)
            k = min(free, key=lambda q: np.linalg.norm(people[q] - pred))
            if np.linalg.norm(people[k] - pred) < self.gate:
                tr.update(people[k], t)
                free.remove(k)
        for k in free:
            self.tracks.append(Track(self.next_id, people[k], t))
            self.next_id += 1
        self.tracks = [tr for tr in self.tracks if t - tr.seen < self.keep_s]
        pa = PoseArray()
        pa.header = m.header
        txt = []
        for tr in self.tracks:
            if tr.hits < 3 or tr.seen != t:
                continue
            ps = Pose()
            ps.position.x, ps.position.y = float(tr.p[0]), float(tr.p[1])
            yaw = math.atan2(tr.v[1], tr.v[0])
            ps.orientation.z, ps.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            pa.poses.append(ps)
            txt.append(f'#{tr.id} {math.hypot(*tr.p):.2f} m @ {math.degrees(math.atan2(tr.p[1], tr.p[0])):+.0f} deg'
                       f' v {np.linalg.norm(tr.v):.2f}')
        self.pub.publish(pa)
        self.dbg.publish(String(data=f'legs {len(legs)} people {len(pa.poses)}: ' + ', '.join(txt)))


def main():
    rclpy.init()
    try:
        rclpy.spin(LegDetector())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
