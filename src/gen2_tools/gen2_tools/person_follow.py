#!/usr/bin/env python3
"""Follow ONE designated person (competition: no markers, 2 people, hand signals stop / restart only).

Fusion: LiDAR leg tracks (people/legs, laser frame: distance + bearing, works without light) are confirmed as people
when a camera person box (people/camera, YOLO on the GPU) lies at the same bearing (+-bearing_tol). A camera person with
no leg track at its bearing (legs not seen, LiDAR down) is a camera-only candidate: bearing from the camera, distance
from the ankles' image row on the ground plane (camera height from the leg height, body pitch from controller/state). The designated
person is registered on enable: the confirmed person closest to straight ahead; its clothes (HSV histogram of the
box) are kept and slowly updated, and every later choice prefers the track that continues + the best appearance.
The target is kept in the odom frame (world-fixed) with a velocity estimate, so it stays valid while the robot
drives / turns and while the person is hidden (predicted along its walking direction, up to predict_s).

States: IDLE (not enabled) -> FOLLOW (seen within grace_s) -> OCCLUDED (lost < occluded_s: go to the predicted
        place) -> SEARCH (turn towards the side the person was last seen until seen + matched again).
        HOLD = stop hand signal: stand still.
Re-acquire: short gaps (< legs_only_s) also by LiDAR legs alone near the prediction; longer gaps need a camera
match (clothes >= appearance_min) within max_gate_m of the prediction, or anywhere with clothes >= reacquire_min.
Command: free corridor to the person (scan_level) -> direct: wz = k_yaw * bearing,
vx = k_dist * (distance - follow_dist) * cos(bearing) (responsive; MPPI only steps up from the measured speed and is
sluggish on a balancing robot). Obstacle in the corridor (robot width + corridor_margin_m, up to 0.45 m before the
person) -> Nav2 (nav_follow.launch): a path to 'follow_dist in front of the person' on the LiDAR costmap every
nav_replan_s, MPPI drives it around; kept nav_hold_s after the corridor is free again. After a controller abort only
turn towards the person for 1 s, then plan again. Both go to cmd_vel/auto (cmd_mux; the pad always wins).
Parameters in _LIVE (+ bearing_tol_deg) can be changed while the robot is disarmed (gen2_tools.live). Services: follow/enable (SetBool), follow/hold (SetBool: true =
stop, false = resume). Topic: follow/status (String).
"""

import json
import math
import time

import numpy as np
import rclpy
import rclpy.executors
from gen2_tools.live import Live
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from gen2_msgs.msg import ControllerState
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

LASER_X = -0.0774          # laser in base_link (desktop CAD)


class PersonFollow(Node):
    def __init__(self):
        super().__init__('person_follow')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.d_ref = p('follow_dist_m', 0.8)
        self.k_d, self.k_yaw = p('k_dist', 0.8), p('k_yaw', 1.5)
        self.vx_max, self.wz_max = p('vx_max', 0.83), p('wz_max', 1.0)   # 0.83 m/s = 3.0 km/h walking speed
        self.tol = math.radians(p('bearing_tol_deg', 10.0))
        self.cam_to = p('camera_timeout_s', 0.5)
        self.occl_s = p('occluded_s', 5.0)
        self.search_wz = p('search_wz', 0.5)
        self.app_min = p('appearance_min', 0.5)          # Bhattacharyya similarity: same person near the prediction
        self.app_reacq = p('reacquire_min', 0.65)        # ... anywhere (long loss)
        self.grace = p('grace_s', 0.5)                   # missed scans shorter than this stay FOLLOW
        self.legs_only = p('legs_only_s', 1.0)           # gaps up to this may be bridged by legs without the camera
        self.max_gate = p('max_gate_m', 2.5)
        self.pred_s = p('predict_s', 1.5)
        # camera-only people (no LiDAR legs at that bearing, or LiDAR down): distance from the ankles' (else the box
        # bottom's) image row on the ground plane: camera height = leg h + cam_h_off, depression = ray + body pitch + tilt
        self.cam_only = p('camera_only', True)
        self.cam_fx = p('cam_fx', 530.0)
        self.cam_tilt = math.radians(p('cam_tilt_deg', 0.0))
        self.cam_x = p('cam_x_m', 0.108)          # camera_link in base_link
        self.cam_h_off = p('cam_h_off_m', 0.081)  # camera above the leg height h (base 0.0655 + camera 0.0153)
        self.cam_dmax = p('cam_max_dist_m', 5.0)
        self.leg_h, self.body_pitch = 0.182, 0.0
        self.enabled, self.hold = False, False
        self.state = 'IDLE'
        self.cam, self.cam_t, self.cam_new, self.cam_new_match, self.cam_h = [], 0.0, False, False, 480
        self.legs, self.legs_new, self.legs_t = [], False, -1e9
        self.target = None                               # dict: w (odom xy), v (odom m/s), t, hist, side
        self.pub = self.create_publisher(Twist, 'cmd_vel/auto', 10)
        self.st = self.create_publisher(String, 'follow/status', 10)
        self.create_subscription(String, 'people/camera', self._cam, 10)
        self.create_subscription(PoseArray, 'people/legs', self._legs, 10)
        self.create_service(SetBool, 'follow/enable', self._enable)
        self.create_service(SetBool, 'follow/hold', self._hold)
        # Nav2 (nav_follow.launch: planner + MPPI on the LiDAR costmap) drives around obstacles when available
        self.use_nav = p('use_nav', True)
        self.nav_period = p('nav_replan_s', 0.5)
        # robot pose in odom: /odom is 100 Hz and every executor wake-up costs (measured 19 % CPU just for that; a TF
        # listener was 25 %) -> a subscription on a side node the executor never spins, polled for the newest in _pose()
        self.odom = (0.0, 0.0, 0.0)
        self._odom_node = rclpy.create_node('person_follow_odom')
        self._odom_sub = self._odom_node.create_subscription(Odometry, 'odom', lambda m: None, 1)
        # robot mode (controller/state, 100 Hz, polled the same way): follow switches itself off when the robot is
        # not balancing (disarmed / fell / fault) — no tracking, detectors go idle, X again after the next START
        self._mode_sub = self._odom_node.create_subscription(ControllerState, 'controller/state', lambda m: None, 1)
        self.robot_mode = None
        self.ac_plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.ac_follow = ActionClient(self, FollowPath, 'follow_path')
        self.nav_goal_h = None          # FollowPath goal handle
        self.nav_t = 0.0
        self.nav_busy = False
        self.nav_fail_t = -1e9
        self.margin = p('corridor_margin_m', 0.12)
        self.nav_hold = p('nav_hold_s', 2.0)
        self.blocked_t = -1e9           # last time the corridor to the person was blocked
        self.scan_b = np.zeros((0, 2))  # scan_level points in base_link
        self.create_subscription(LaserScan, 'scan_level', self._scan, qos_profile_sensor_data)          # controller gave up (boxed in / no progress): steer directly for a while
        live = dict(self._LIVE)
        live['bearing_tol_deg'] = lambda v: setattr(self, 'tol', math.radians(v))
        Live(self, live, always=('use_nav',))
        self.create_timer(0.05, self._step)

    # runtime-changeable (robot disarmed only, gen2_tools.live): limits / tracking / avoidance — no gains (k_dist, k_yaw)
    _LIVE = {'follow_dist_m': 'd_ref', 'vx_max': 'vx_max', 'wz_max': 'wz_max',
             'occluded_s': 'occl_s', 'search_wz': 'search_wz', 'appearance_min': 'app_min', 'use_nav': 'use_nav',
             'nav_replan_s': 'nav_period', 'reacquire_min': 'app_reacq', 'grace_s': 'grace',
             'legs_only_s': 'legs_only', 'max_gate_m': 'max_gate', 'predict_s': 'pred_s',
             'corridor_margin_m': 'margin', 'nav_hold_s': 'nav_hold'}

    # ------------------------------------------------------------------ inputs
    def _cam(self, m):
        j = json.loads(m.data)
        self.cam, self.cam_t, self.cam_new, self.cam_new_match = j['people'], time.monotonic(), True, True
        self.cam_h = j.get('h', self.cam_h) or self.cam_h

    def _legs(self, m):
        # PoseArray in the laser frame, ids are not carried: use positions (base frame)
        self.legs = [np.array([q.position.x + LASER_X, q.position.y]) for q in m.poses]
        self.legs_new, self.legs_t = True, time.monotonic()

    def _scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float64)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < 4.0)
        self.scan_b = np.stack([r[ok] * np.cos(a[ok]) + LASER_X, r[ok] * np.sin(a[ok])], 1)

    def _blocked(self, pb):
        """Obstacle in the straight corridor robot -> person (stopping 0.45 m before the person's legs)?"""
        d = float(np.hypot(*pb))
        if d < 0.6 or not len(self.scan_b):
            return False
        u = pb / d
        along = self.scan_b @ u
        lat = np.abs(self.scan_b[:, 0] * u[1] - self.scan_b[:, 1] * u[0])
        hit = (along > 0.05) & (along < d - 0.45) & (lat < 0.164 + self.margin)
        return int(hit.sum()) >= 3

    def _enable(self, req, res):
        st = self._poll(self._mode_sub, ControllerState)
        if st is not None:
            self.robot_mode = st.mode
        if req.data and self.robot_mode is not None and self.robot_mode != 'balance':
            res.success, res.message = False, f'follow refused: robot {self.robot_mode} (START first)'
            self.get_logger().warn(res.message)
            return res
        self.enabled = bool(req.data)
        self.hold = False
        self.target = None
        self.state = 'SEARCH' if self.enabled else 'IDLE'
        res.success, res.message = True, f'follow {"on: registering the person ahead" if self.enabled else "off"}'
        self.get_logger().info(res.message)
        if not self.enabled:
            self._nav_cancel()
            self.pub.publish(Twist())
        return res

    def _hold(self, req, res):
        self.hold = bool(req.data)
        if self.hold:
            self._nav_cancel()
        res.success, res.message = True, 'hold' if self.hold else 'resume'
        self.get_logger().info(res.message)
        return res

    # ------------------------------------------------------------------ fusion
    @staticmethod
    def _sim(a, b):
        return float(np.sum(np.sqrt(np.asarray(a) * np.asarray(b))))      # Bhattacharyya coefficient

    def _cam_dist(self, c):
        """Ground distance [m] from the camera to the person's feet, or None (no usable row / too far)."""
        rows = []
        k = c.get('kps')
        if k:
            rows = [q[1] for q in (k[15], k[16]) if q[2] > 0.5]       # ankles
        if not rows and c.get('box') and c['box'][3] < self.cam_h - 4:
            rows = [c['box'][3]]                                       # box bottom (feet inside the image)
        if not rows:
            return None
        dep = math.atan2(max(rows) - 0.5 * self.cam_h, self.cam_fx) + self.body_pitch + self.cam_tilt
        if dep < math.radians(1.5):
            return None
        d = (self.leg_h + self.cam_h_off) / math.tan(dep)
        return d if 0.2 < d < self.cam_dmax else None

    def _candidates(self):
        """People: LiDAR leg position + matching camera box (bearing within tol); camera boxes with no leg at their
        bearing become camera-only candidates (distance from the feet on the ground plane)."""
        fresh = time.monotonic() - self.cam_t < self.cam_to
        out, used = [], set()
        for pl in self.legs:
            b = math.atan2(pl[1], pl[0])
            best = None
            if fresh:
                for i, c in enumerate(self.cam):
                    d = abs(math.radians(c['bearing_deg']) - b)
                    if d < self.tol and (best is None or d < best[0]):
                        best = (d, c, i)
            if best:
                used.add(best[2])
            out.append({'p': pl, 'bearing': b, 'dist': float(np.hypot(*pl)), 'cam': best[1] if best else None, 'src': 'legs'})
        if self.cam_only and fresh:
            for i, c in enumerate(self.cam):
                if i in used:
                    continue
                d = self._cam_dist(c)
                if d is None:
                    continue
                b = math.radians(c['bearing_deg'])
                pc = np.array([self.cam_x + d * math.cos(b), d * math.sin(b)])
                out.append({'p': pc, 'bearing': math.atan2(pc[1], pc[0]), 'dist': float(np.hypot(*pc)), 'cam': c, 'src': 'camera'})
        return out

    # ------------------------------------------------------------------ frames
    def _poll(self, sub, typ):
        last = None
        with sub.handle:
            while True:
                r = sub.handle.take_message(typ, False)
                if r is None:
                    break
                last = r[0] if isinstance(r, tuple) else r
        return last

    def _pose(self):
        """Robot pose in odom (x, y, yaw), newest /odom; stays (0, 0, 0) without /odom (targets then robot-relative)."""
        last = self._poll(self._odom_sub, Odometry)
        if last is not None:
            q = last.pose.pose.orientation
            self.odom = (last.pose.pose.position.x, last.pose.pose.position.y,
                         math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        return self.odom

    @staticmethod
    def _to_w(pose, p):
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        return np.array([pose[0] + c * p[0] - s_ * p[1], pose[1] + s_ * p[0] + c * p[1]])

    @staticmethod
    def _to_b(pose, w):
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        dx, dy = w[0] - pose[0], w[1] - pose[1]
        return np.array([c * dx + s_ * dy, -s_ * dx + c * dy])

    def _predict(self, tgt, now):
        return tgt['w'] + tgt['v'] * min(now - tgt['t'], self.pred_s)

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
            self.pub.publish(Twist())
        if not self.enabled:
            self._status('IDLE')
            return
        pose = self._pose()
        tgt = self.target
        lidar_alive = now - self.legs_t < 0.3
        if self.legs_new or (not lidar_alive and self.cam_new_match):   # once per LiDAR scan; per camera frame without LiDAR
            self.legs_new = self.cam_new_match = False
            cands = self._candidates()
            for c in cands:
                c['w'] = self._to_w(pose, c['p'])
            if tgt is None:
                # register: confirmed person (camera + legs) closest to straight ahead, within 2.5 m
                conf = [c for c in cands if c['cam'] is not None and c['dist'] < 2.5]
                if conf:
                    pk = min(conf, key=lambda c: abs(c['bearing']))
                    self.target = tgt = {'w': pk['w'], 'v': np.zeros(2), 't': now, 'side': 1.0,
                                         'hist': np.array(pk['cam']['hsv_hist'])}
                    self.get_logger().info(f'registered person at {pk["dist"]:.2f} m, {math.degrees(pk["bearing"]):+.0f} deg')
            else:
                self._match(tgt, cands, now)
        # between LiDAR scans (10 Hz) keep the bearing fresh with the camera (30 Hz): same person = matching
        # clothes within the bearing tolerance -> rotate the target to the camera bearing, keep the LiDAR range
        if tgt is not None and self.cam_new and now - tgt['t'] < self.grace:
            self.cam_new = False
            pb = self._to_b(pose, tgt['w'])
            bt, dt_ = math.atan2(pb[1], pb[0]), float(np.hypot(*pb))
            best = None
            for c in self.cam:
                bc = math.radians(c['bearing_deg'])
                if abs(bc - bt) < self.tol and self._sim(c['hsv_hist'], tgt['hist']) >= self.app_min:
                    if best is None or abs(bc - bt) < abs(best - bt):
                        best = bc
            if best is not None:
                tgt['w'] = self._to_w(pose, (dt_ * math.cos(best), dt_ * math.sin(best)))
        t = Twist()
        lost = now - tgt['t'] if tgt is not None else 1e9
        if self.hold:
            state = 'HOLD'
        elif tgt is None:
            state = 'SEARCH'
            t.angular.z = self.search_wz
        elif lost < self.grace:
            state = 'FOLLOW'
        elif lost < self.occl_s:
            state = 'OCCLUDED'
        else:
            state = 'SEARCH'
            t.angular.z = math.copysign(self.search_wz, tgt['side'])     # turn to where the person went
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
                return                                     # the Nav2 controller publishes cmd_vel/auto
        self._nav_cancel()
        if pb is not None:
            x, y = pb
            b, d = math.atan2(y, x), math.hypot(x, y)
            k = 1.0 if state == 'FOLLOW' else 0.5             # slower towards the predicted place
            if now - self.nav_fail_t < 1.0:
                k = 0.0                                       # controller just gave up (blocked): only turn
            t.angular.z = float(np.clip(self.k_yaw * b, -self.wz_max, self.wz_max))
            t.linear.x = float(np.clip(k * self.k_d * (d - self.d_ref) * max(0.0, math.cos(b)), -self.vx_max * 0.5, self.vx_max))
        self.pub.publish(t)
        self._status(state, pb)

    def _match(self, tgt, cands, now):
        """Pick this scan's candidate for the target (odom frame) and update position / velocity / clothes."""
        lost = now - tgt['t']
        pred = self._predict(tgt, now)
        gate = min(self.max_gate, 0.4 + 1.5 * lost)        # how far the person may have walked since last seen
        best, bs = None, -1e9
        for c in cands:
            jump = float(np.linalg.norm(c['w'] - pred))
            if jump > gate:
                continue
            sim = self._sim(c['cam']['hsv_hist'], tgt['hist']) if c['cam'] else None
            if sim is None and lost > self.legs_only:
                continue                                     # long gap: legs alone could be anything (chair, someone)
            if sim is not None and sim < self.app_min:
                continue                                     # someone else (different clothes)
            s = -jump + (0.5 * sim if sim is not None else 0.0)
            if s > bs:
                best, bs = c, s
        reacq = False
        if best is None and lost > self.legs_only:          # lost for a while: the same clothes anywhere in view
            sims = [(self._sim(c['cam']['hsv_hist'], tgt['hist']), c) for c in cands if c['cam'] is not None]
            sims = [sc for sc in sims if sc[0] >= self.app_reacq]
            if sims:
                best, reacq = max(sims, key=lambda sc: sc[0])[1], True
        if best is None:
            return
        dt = now - tgt['t']
        if reacq or dt > 1.0:
            tgt['v'] = np.zeros(2)
            if lost > self.grace:
                self.get_logger().info(f're-acquired after {lost:.1f} s{" by clothes" if reacq else ""}')
        else:
            v = (best['w'] - tgt['w']) / max(dt, 0.05)
            tgt['v'] = 0.7 * tgt['v'] + 0.3 * v
            n = float(np.linalg.norm(tgt['v']))
            if n > 1.8:
                tgt['v'] *= 1.8 / n
        tgt['w'], tgt['t'] = best['w'], now
        if best['cam'] is not None:
            tgt['hist'] = 0.9 * tgt['hist'] + 0.1 * np.array(best['cam']['hsv_hist'])

    # ------------------------------------------------------------------ Nav2
    def _nav(self, pose, pw, pb, now):
        """Plan to 'follow_dist in front of the person' (odom frame) every nav_replan_s; False = drive directly."""
        if not self.use_nav or not self.ac_plan.server_is_ready() or not self.ac_follow.server_is_ready():
            return False
        if now - self.nav_fail_t < 1.0:
            return False
        d = float(np.hypot(*pb))
        if d < self.d_ref + 0.15:                          # close enough: just face the person (direct)
            return False
        if self.nav_busy or now - self.nav_t < self.nav_period:
            return self.nav_goal_h is not None
        u = (pw - np.array(pose[:2])) / d                  # robot -> person (unit, odom)
        g = PoseStamped()
        g.header.frame_id = 'odom'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = float(pw[0] - self.d_ref * u[0]), float(pw[1] - self.d_ref * u[1])
        yaw = math.atan2(u[1], u[0])
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        req = ComputePathToPose.Goal(goal=g, planner_id='GridBased', use_start=False)
        self.nav_busy, self.nav_t = True, now
        self.ac_plan.send_goal_async(req).add_done_callback(self._plan_accepted)
        return self.nav_goal_h is not None

    def _plan_accepted(self, fut):
        h = fut.result()
        if not h or not h.accepted:
            self.nav_busy = False
            return
        h.get_result_async().add_done_callback(self._plan_done)

    def _plan_done(self, fut):
        self.nav_busy = False
        res = fut.result().result
        if not res.path.poses or not self.enabled or self.hold:
            return
        g = FollowPath.Goal(path=res.path, controller_id='FollowPath', goal_checker_id='goal_checker')
        self.ac_follow.send_goal_async(g).add_done_callback(self._follow_accepted)

    def _follow_accepted(self, fut):
        h = fut.result()
        self.nav_goal_h = h if h and h.accepted else None
        if self.nav_goal_h is not None:
            h.get_result_async().add_done_callback(lambda f, h=h: self._follow_done(h, f.result().status))

    def _follow_done(self, h, status):
        if status == GoalStatus.STATUS_ABORTED and self.nav_goal_h is h:
            self.get_logger().warn('Nav2 controller aborted (blocked): turning to the person, plan again in 1 s')
            self.nav_goal_h, self.nav_fail_t = None, time.monotonic()

    def _nav_cancel(self):
        if self.nav_goal_h is not None:
            self.nav_goal_h.cancel_goal_async()
            self.nav_goal_h = None

    def _status(self, state, pb=None):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state
        s = state
        if pb is not None:
            s += f' target {math.hypot(*pb):.2f} m @ {math.degrees(math.atan2(pb[1], pb[0])):+.0f} deg'
        self.st.publish(String(data=s))


def main():
    rclpy.init()
    try:
        rclpy.spin(PersonFollow())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
