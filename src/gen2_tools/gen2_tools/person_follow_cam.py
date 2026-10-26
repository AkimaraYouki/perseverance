#!/usr/bin/env python3
"""Camera-first person following (2026-10-25, desktop) — person_follow without the LiDAR leg detector.

Who: person_tracker (YOLO + ByteTrack + OSNet ReID) keeps the designated person as a track id and publishes
people/target at the camera rate (30 Hz). On follow/enable this node asks it to register the person nearest straight
ahead (people/target_cmd 'register'); no LiDAR leg track is needed any more (the old node locked only when a leg track
sat at the camera bearing, once per LiDAR scan — a person with a green box was often never locked).
Where: bearing from the camera; distance from the LiDAR points inside the person's box (bearing span), nearest cluster,
else from the feet row on the ground plane (wide lens: equidistant model). The target is kept in the odom frame with a
velocity, so OCCLUDED / SEARCH / HOLD / Nav2 avoidance work exactly as in person_follow (inherited).
Speed limit 0.7 m/s (user: follow / command maximum 0.7 m/s).

Run: ros2 run gen2_tools person_follow_cam   (node name person_follow: same services follow/enable, follow/hold,
     topic follow/status, cmd_vel/auto — GUI / pad / gestures unchanged). Start person_tracker instead of
     person_detector + leg_detector.
"""

import json
import math
import time

import numpy as np
import rclpy
import rclpy.executors
from gen2_msgs.msg import ControllerState
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from gen2_tools.person_follow import PersonFollow


class PersonFollowCam(PersonFollow):
    def __init__(self):
        super().__init__()
        self.cmd = self.create_publisher(String, 'people/target_cmd', 10)
        self.create_subscription(String, 'people/target', self._tg, 10)

    def _declare_extra(self, p):
        self.vx_max = p('cam_vx_max', 0.7)
        self.cam_model = p('camera_model', 'pinhole')     # pinhole (cam_fx) | equidistant (hfov_deg) | opencv_fisheye (K, D)
        self.hfov = math.radians(p('hfov_deg', 160.0))
        self.cam_K, self.cam_D = p('K', [0.0]), p('D', [0.0])  # checkerboard calibration (camera_cal.yaml, 2026-10-26)
        self.bm = None
        self.lidar_margin = math.radians(p('lidar_span_margin_deg', 2.0))
        self.lidar_max = p('lidar_max_m', 6.0)
        self.tg_msg, self.tg_new, self.tg_t = None, False, -1e9
        self.dist_src = '-'
        self.cam_w = 640

    # ------------------------------------------------------------------ inputs
    def _tg(self, m):
        self.tg_msg, self.tg_new, self.tg_t = json.loads(m.data), True, time.monotonic()
        self.cam_h = self.tg_msg.get('h', self.cam_h) or self.cam_h
        self.cam_w = self.tg_msg.get('w', self.cam_w) or self.cam_w
        if self.enabled:
            self._step()                                    # camera rate (30 Hz); the 20 Hz timer covers camera dropouts

    def _enable(self, req, res):
        res = super()._enable(req, res)
        if res.success:
            self.cmd.publish(String(data='register' if self.enabled else 'clear'))
        return res

    # ------------------------------------------------------------------ geometry
    def _lidar_dist(self, span):
        """Nearest LiDAR cluster inside the person's bearing span (from the camera) [m], or None."""
        if not len(self.scan_b):
            return None
        rel = self.scan_b - np.array([self.cam_x, 0.0])
        ang = np.arctan2(rel[:, 1], rel[:, 0])
        lo, hi = min(span) - self.lidar_margin, max(span) + self.lidar_margin
        r = np.hypot(rel[:, 0], rel[:, 1])
        m = (ang >= lo) & (ang <= hi) & (r < self.lidar_max) & (r > 0.15)
        if int(m.sum()) < 3:
            return None
        rs = np.sort(r[m])
        near = rs[rs <= rs[min(len(rs) - 1, max(2, len(rs) // 5))] + 0.25]      # the front cluster (legs / body)
        return float(np.median(near)) if len(near) >= 3 else None

    def _ground_dist(self, feet_row, u=None):
        """Camera -> feet distance on the ground plane from the feet pixel (lens model from camera_cal / parameters)."""
        if self.bm is None:
            from gen2_tools.tracking_core import BearingModel
            K = self.cam_K if len(self.cam_K) == 9 else None
            D = self.cam_D if len(self.cam_D) == 4 else None
            self.bm = BearingModel(self.cam_model, self.cam_w, self.cam_h, fx=self.cam_fx, hfov_deg=math.degrees(self.hfov), K=K, D=D)
        _, ray = self.bm.ray(self.cam_w / 2.0 if u is None else u, feet_row)
        dep = ray + self.body_pitch + self.cam_tilt
        if dep < math.radians(1.5):
            return None
        d = (self.leg_h + self.cam_h_off) / math.tan(dep)
        return d if 0.2 < d < self.cam_dmax else None

    # ------------------------------------------------------------------ main loop
    def _step(self):
        now = time.monotonic()
        st = self._poll(self._mode_sub, ControllerState)
        if st is not None:
            self.robot_mode = st.mode
            if st.h[0] > 0.05:
                self.leg_h, self.body_pitch = 0.5 * (st.h[0] + st.h[1]), st.pitch
        if self.enabled and self.robot_mode is not None and self.robot_mode != 'balance':
            self.get_logger().warn(f'robot {self.robot_mode}: follow off (press X again after START)')
            self.enabled, self.hold, self.target = False, False, None
            self._nav_cancel()
            self.cmd.publish(String(data='clear'))
            self.pub.publish(Twist())
        if not self.enabled:
            self._status('IDLE')
            return
        pose = self._pose()
        tgt = self.target
        j = self.tg_msg if (self.tg_new and now - self.tg_t < self.cam_to) else None
        self.tg_new = False
        if j is not None and j.get('box') is not None and j.get('state') in ('LOCK', 'REACQUIRED'):
            b = math.radians(j['bearing_deg'])
            d = self._lidar_dist([math.radians(a) for a in j['span_deg']])
            self.dist_src = 'lidar'
            if d is None:
                d, self.dist_src = self._ground_dist(j['feet_row'], 0.5 * (j['box'][0] + j['box'][2])), 'feet'
            if d is not None:
                pc = np.array([self.cam_x + d * math.cos(b), d * math.sin(b)])
                w = self._to_w(pose, pc)
                if tgt is None or j.get('state') == 'REACQUIRED' or j.get('id') != tgt.get('id'):
                    if tgt is None:
                        self.get_logger().info(f'locked person id {j.get("id")} at {d:.2f} m, {math.degrees(b):+.0f} deg ({self.dist_src})')
                    self.target = tgt = {'w': w, 'v': np.zeros(2), 't': now, 'side': 1.0, 'src': 'camera', 'cam_t': now,
                                         'hist': np.zeros(1), 'id': j.get('id')}
                else:
                    dt = now - tgt['t']
                    if dt > 1e-3:
                        v = (w - tgt['w']) / max(dt, 0.033)
                        tgt['v'] = 0.8 * tgt['v'] + 0.2 * v
                        n = float(np.linalg.norm(tgt['v']))
                        if n > 1.8:
                            tgt['v'] *= 1.8 / n
                    tgt['w'] = 0.6 * w + 0.4 * self._predict(tgt, now)    # light smoothing (box jitter on waves)
                    tgt['t'], tgt['cam_t'], tgt['src'] = now, now, f'camera+{self.dist_src}'
        self._control(pose, tgt, now)

    def _control(self, pose, tgt, now):
        """Same states / commands as person_follow (FOLLOW, OCCLUDED -> predicted place, SEARCH, HOLD, Nav2 around obstacles)."""
        t = Twist()
        lost = now - tgt['t'] if tgt is not None else 1e9
        if self.hold:
            state = 'HOLD'
        elif tgt is None:
            state = 'SEARCH'
            t.angular.z = 0.0                               # wait for the lock (person ahead); no spinning before
        elif lost < self.grace:
            state = 'FOLLOW'
        elif lost < self.occl_s:
            state = 'OCCLUDED'
        else:
            state = 'SEARCH'
            t.angular.z = math.copysign(self.search_wz, tgt['side'])
        pb = None
        if state in ('FOLLOW', 'OCCLUDED'):
            pw = self._predict(tgt, now)
            pb = self._to_b(pose, pw)
            if abs(pb[1]) > 0.1:
                tgt['side'] = math.copysign(1.0, pb[1])
            if self._blocked(pb):
                self.blocked_t = now
            if now - self.blocked_t < self.nav_hold and self._nav(pose, pw, pb, now):
                self._status(state + '+nav', pb)
                return
        self._nav_cancel()
        if pb is not None:
            x, y = pb
            b, d = math.atan2(y, x), math.hypot(x, y)
            k = 1.0 if state == 'FOLLOW' else 0.5
            if now - self.nav_fail_t < 1.0:
                k = 0.0
            t.angular.z = float(np.clip(self.k_yaw * b, -self.wz_max, self.wz_max))
            t.linear.x = float(np.clip(k * self.k_d * (d - self.d_ref) * max(0.0, math.cos(b)), -self.vx_max * 0.5, self.vx_max))
        self.pub.publish(t)
        self._status(state, pb)


def main():
    rclpy.init()
    n = PersonFollowCam()
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
