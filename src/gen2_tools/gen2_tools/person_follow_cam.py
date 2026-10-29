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

from gen2_tools.auto_step import AutoStep
from gen2_tools.person_follow import PersonFollow
from gen2_tools.step_watch import StepWatch
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8
from std_srvs.srv import SetBool


class PersonFollowCam(PersonFollow):
    _LIVE = dict(PersonFollow._LIVE, auto_step='auto_on', explore='ex_on', explore_s='ex_s', explore_ahead_m='ex_ahead',
                 explore_side_m='ex_side', explore_v='ex_v', explore_scan_deg='ex_scan_deg',
                 feet_follow='feet_on', feet_px='feet_px', feet_k='feet_k', front_stop_m='front_stop')
    _ALWAYS = PersonFollow._ALWAYS + ('auto_step', 'explore', 'feet_follow')     # 켜기/끄기는 균형 중에도 (GUI)
    def __init__(self):
        super().__init__()
        self.cmd = self.create_publisher(String, 'people/target_cmd', 10)
        self.create_subscription(String, 'people/target', self._tg, 10)
        # 자동 턱 (2026-10-27, Isaac 시험): 카메라 턱 -> 감속 -> 턱 감지 모드 (balance/step_mode) -> 점프 -> 다 올라가면 해제
        self.create_subscription(String, 'step/rays', self._rays, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Int8, 'balance/step_phase', self._phase, latched)
        self.step_cli = self.create_client(SetBool, 'balance/step_mode')

    def _declare_extra(self, p):
        self.vx_max = p('cam_vx_max', 0.7)
        self.cam_model = p('camera_model', 'pinhole')     # pinhole (cam_fx) | equidistant (hfov_deg) | opencv_fisheye (K, D)
        self.hfov = math.radians(p('hfov_deg', 160.0))
        self.cam_K, self.cam_D = p('K', [0.0]), p('D', [0.0])  # checkerboard calibration (camera_cal.yaml, 2026-10-26)
        self.cam_roll = p('cam_roll_deg', 0.0)                 # mount roll (cw HUD, 2026-10-27)
        self.bm = None
        # 발 픽셀 추종 (사용자 2026-10-29 "발은 수평선 아래로 유지하게끔 추종, 수평선부터 픽셀은 파라미터로"): FOLLOW 의 전진 속도를
        # 거리 (LiDAR / 발 / 예측, 튐) 대신 사람 발 행이 카메라 수평선 아래 feet_px 에 오게 정한다. 멀수록 발이 수평선 쪽으로
        # 올라감 -> 전진, 가까우면 내려감 (화면 밖 = 아래 끝) -> 멈춤 / 조금 후진. 수평선 = 몸 피치 + cam_tilt 로 렌즈 모델에서
        self.feet_on = p('feet_follow', False)
        self.feet_px = p('feet_px', 80.0)                  # [px] 목표: 발이 수평선 아래 이만큼 (로봇 렌즈: 100 = 0.8 m, 80 = 1.0 m, 55 = 1.5 m)
        self.feet_k = p('feet_k', 0.01)                    # [m/s per px] 50 px 모자라면 0.5 m/s
        self.feet_dv, self.feet_t = None, -1e9
        # 라이다 앞 정지 (사용자 2026-10-29: 미로를 사람 따라 통과, 사람이 너무 가깝거나 장애물이면 라이다로 멈추고 Nav2 로 피함):
        # 몸 폭 (+ corridor_margin_m) 앞쪽 통로에 front_stop_m 안의 라이다 점 3 개 이상이면 전진 0 (회전 · 후진은 됨).
        # 거리 추정 (라이다 묶음 · 발 · 예측 · 발 픽셀) 이 틀려도 사람 다리나 벽 앞에서 선다. 0 = 끔
        self.front_stop = p('front_stop_m', 0.5)
        self.front_hit = False
        self.lidar_margin = math.radians(p('lidar_span_margin_deg', 2.0))
        self.lidar_max = p('lidar_max_m', 6.0)
        self.tg_msg, self.tg_new, self.tg_t = None, False, -1e9
        self.dist_src = '-'
        self.cam_w = 640
        # auto_step_v 0.3 -> 0.7 (GCS 2026-10-28, game day): the forward command while the step-detect mode is on decides when
        # the re-run reaches the trigger. Robot logs this boot: command 0.3 -> trigger at the overshoot peak 0.56-0.58 m/s in
        # 12/18 runs (falls), 0.5 -> 9/20, 0.7 (= pad D-pad + full stick, the manual mode that works) -> 0/10 (0.32-0.37 m/s).
        # The robot sets the real speeds itself (approach 0.14, re-run 0.3); this only pushes forward.
        self.step_stop_m = p('auto_step_stop_m', 0.6)       # STEP: no forward push when the person is this close
        self.auto_on = p('auto_step', False)                 # 기본 꺼짐 (GUI / 파라미터로 켬, 균형 중에도: Live always)
        self.auto = AutoStep(slow_dist=p('auto_slow_dist_m', 1.5), arm_dist=p('auto_arm_dist_m', 0.5),
                             slow_v=p('auto_slow_v', 0.3), step_v=p('auto_step_v', 0.7), done_travel=p('auto_done_travel_m', 1.3),
                             false_m=p('auto_false_m', 0.8), land_stop_s=p('auto_land_stop_s', 0.8))   # false_m was 0.35: real step 0.3-0.4 m farther than the camera said (2026-10-29)
        self.auto_watch = StepWatch(None)
        self.step_d, self.step_on_sent, self.odom_s, self.odom_prev = None, False, 0.0, None
        self.rt_phase, self.t_fire = 0, -1e9
        self.v_meas = 0.0
        # 가려짐 탐색 (GCS 2026-10-28, 사용자 "장애물에 가려지면 자동으로 탐사해서 다시 붙기"): occluded_s 를 넘게 놓치면
        # 제자리 회전 (SEARCH) 전에 EXPLORE — 사람이 사라진 곳 + 가던 방향 explore_ahead_m 앞 (그다음 사라진 쪽 옆
        # explore_side_m) 으로 가서 (막히면 Nav2) 좌우를 훑는다. 재식별 (person_tracker REACQUIRED) 되면 바로 FOLLOW.
        self.ex_on = p('explore', True)
        self.ex_s = p('explore_s', 12.0)                     # occluded_s 뒤 이 시간까지 탐색, 그다음 SEARCH
        self.ex_ahead = p('explore_ahead_m', 1.0)            # 사라진 곳에서 가던 방향으로 더 갈 거리
        self.ex_side = p('explore_side_m', 0.6)              # 두 번째 지점: 사라진 쪽 (side) 으로 비킨 거리
        self.ex_v = p('explore_v', 0.4)                      # 탐색 중 최고 속도 [m/s]
        self.ex_scan_deg = p('explore_scan_deg', 70.0)       # 지점에 닿으면 좌우로 이만큼 훑기
        self.ex = None                                       # 탐색 계획 (사람을 놓친 순간마다 새로)
        self.dist_gate = p('dist_gate_m', 0.35)
        self.disagree_t = None                               # since when the predicted and the feet LiDAR clusters disagree               # 같은 사람인데 거리가 예측에서 이만큼 튀면 그 거리 버림

    # ------------------------------------------------------------------ inputs
    def _tg(self, m):
        self.tg_msg, self.tg_new, self.tg_t = json.loads(m.data), True, time.monotonic()
        self.cam_h = self.tg_msg.get('h', self.cam_h) or self.cam_h
        self.cam_w = self.tg_msg.get('w', self.cam_w) or self.cam_w
        if self.enabled:
            self._step()                                    # camera rate (30 Hz); the 20 Hz timer covers camera dropouts

    def _rays(self, m):
        """Step rays from person_tracker -> distances (camera height = leg h + offset, pitch = body + tilt) -> confirmed step."""
        if not self.auto_on:
            return
        rays = json.loads(m.data).get('rays', [])
        c = self.auto_watch.to_dist([(r[0], (r[1], r[2]), None) for r in rays], self.leg_h + self.cam_h_off,
                                    self.body_pitch + self.cam_tilt)
        self.step_d = self.auto_watch.feed([d for d, _, _ in c], self.odom_s)

    def _phase(self, m):
        if self.rt_phase == 2 and int(m.data) == 0:
            self.t_fire = time.monotonic()                   # run -> jump fired (no jump-phase topic yet: 1.2 s = in the air)
        self.rt_phase = int(m.data)

    def _step_mode(self, on):
        if on == self.step_on_sent or not self.step_cli.service_is_ready():
            return
        self.step_cli.call_async(SetBool.Request(data=bool(on)))
        self.step_on_sent = on
        self.get_logger().info(f'auto step: step-detect mode {"ON" if on else "OFF"} ({self.auto.state})')

    def _travel(self, pose):
        """Distance driven (odometry path length, + forward) for the step bookkeeping."""
        if self.odom_prev is not None:
            dx, dy = pose[0] - self.odom_prev[0], pose[1] - self.odom_prev[1]
            ds = math.hypot(dx, dy)
            self.odom_s += ds if dx * math.cos(pose[2]) + dy * math.sin(pose[2]) >= 0 else -ds
        self.odom_prev = pose[:2]

    def _enable(self, req, res):
        res = super()._enable(req, res)
        if res.success:
            self.cmd.publish(String(data='register' if self.enabled else 'clear'))
        return res

    # ------------------------------------------------------------------ geometry
    def _lidar_clusters(self, span):
        """LiDAR points inside the person's bearing span (from the camera), grouped by range (gap > 0.25 m) ->
        [(median range, points)] nearest first, groups of >= 2 points. 2026-10-28 GCS: legs often give only 2-4 points
        and the old 'front cluster' then merged them with the wall behind (person at 1.05 m read as 3.0 m)."""
        if not len(self.scan_b):
            return []
        rel = self.scan_b - np.array([self.cam_x, 0.0])
        ang = np.arctan2(rel[:, 1], rel[:, 0])
        lo, hi = min(span) - self.lidar_margin, max(span) + self.lidar_margin
        r = np.hypot(rel[:, 0], rel[:, 1])
        rs = np.sort(r[(ang >= lo) & (ang <= hi) & (r < self.lidar_max) & (r > 0.15)])
        out, start = [], 0
        for k in range(1, len(rs) + 1):
            if k == len(rs) or rs[k] - rs[k - 1] > 0.25:
                if k - start >= 2:
                    out.append((float(np.median(rs[start:k])), k - start))
                start = k
        return out

    def _lidar_dist(self, span, expect=None, tol=None):
        """Range of the LiDAR cluster in the span closest to expect (within tol, default dist_gate_m); without expect
        the nearest cluster. None if nothing fits."""
        cl = self._lidar_clusters(span)
        if not cl:
            return None
        if expect is None:
            return cl[0][0]
        tol = self.dist_gate if tol is None else tol
        best = min(cl, key=lambda c: abs(c[0] - expect))
        return best[0] if abs(best[0] - expect) < tol else None

    def _front_blocked(self):
        """LiDAR points straight ahead within front_stop_m (body width + corridor margin): no forward drive."""
        if self.front_stop <= 0 or not len(self.scan_b):
            return False
        x, y = self.scan_b[:, 0], self.scan_b[:, 1]
        return int(((x > 0.05) & (x < self.front_stop) & (np.abs(y) < 0.164 + self.margin)).sum()) >= 3

    def _front_stop(self, t):
        """Clamp a direct command (not Nav2) in place: forward -> 0 when the LiDAR sees something right ahead."""
        self.front_hit = t.linear.x > 0.0 and self._front_blocked()
        if self.front_hit:
            t.linear.x = 0.0
            self.sm_v = min(self.sm_v, 0.0)
        return t

    def _horizon_row(self, u):
        """Image row of the horizon (depression 0 with body pitch + cam_tilt) at column u; may lie outside the image."""
        pitch = self.body_pitch + self.cam_tilt
        lo, hi = -0.5 * self.cam_h, 1.5 * self.cam_h
        for _ in range(20):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if self._lens().ray(u, mid)[1] + pitch < 0 else (lo, mid)
        return 0.5 * (lo + hi)

    def _lens(self):
        if self.bm is None:
            from gen2_tools.tracking_core import BearingModel
            K = self.cam_K if len(self.cam_K) == 9 else None
            D = self.cam_D if len(self.cam_D) == 4 else None
            self.bm = BearingModel(self.cam_model, self.cam_w, self.cam_h, fx=self.cam_fx, hfov_deg=math.degrees(self.hfov), K=K, D=D,
                                   roll_deg=self.cam_roll)
        return self.bm

    def _ground_dist(self, feet_row, u=None):
        """Camera -> feet distance on the ground plane from the feet pixel (lens model from camera_cal / parameters)."""
        _, ray = self._lens().ray(self.cam_w / 2.0 if u is None else u, feet_row)
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
            self.v_meas = st.v
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
            # 같은 사람을 계속 볼 때 거리가 예측에서 dist_gate_m 넘게 튀면 버린다: 사람이 기둥·상자 모서리 뒤로 들어갈 때
            # 라이다가 모서리를 사람 거리로 잡는다 (GCS 2026-10-28 시험) → 발 위치 → 예측 거리 순. 방위는 카메라 그대로
            pc_of = lambda d_: np.array([self.cam_x + d_ * math.cos(b), d_ * math.sin(b)])  # noqa: E731
            same = tgt is not None and j.get('state') == 'LOCK' and j.get('id') == tgt.get('id')
            pred = self._predict(tgt, now) if same else None
            fits = lambda d_: d_ is not None and (pred is None or float(np.linalg.norm(self._to_w(pose, pc_of(d_)) - pred)) < self.dist_gate)  # noqa: E731,E501
            pp = self._to_b(pose, pred) if pred is not None else None
            span = [math.radians(a) for a in j['span_deg']]
            d_feet = self._ground_dist(j['feet_row'], 0.5 * (j['box'][0] + j['box'][2]))
            self.feet_dv = float(j['feet_row'] - self._horizon_row(0.5 * (j['box'][0] + j['box'][2])))   # 발 아래 끝이 화면 밖 = 큰 값
            self.feet_t = now
            # LiDAR cluster: near the prediction (same person) -> else near the camera's feet distance (also at the lock,
            # so the wall behind is not taken) -> else the feet distance -> else the prediction
            d = self._lidar_dist(span, None if pp is None else float(math.hypot(pp[0] - self.cam_x, pp[1])))
            self.dist_src = 'lidar'
            # a wrong lock (e.g. on the wall behind) would keep matching its own prediction: if the feet distance has its
            # own LiDAR cluster far from the predicted one for > 0.7 s, trust the feet one (a quick occluder edge is shorter)
            if d is not None and d_feet is not None and abs(d - d_feet) > max(0.6, 0.4 * d_feet):
                df = self._lidar_dist(span, d_feet, tol=max(self.dist_gate, 0.3 * d_feet))
                if df is not None:
                    self.disagree_t = self.disagree_t if self.disagree_t is not None else now
                    if now - self.disagree_t > 0.7:
                        self.get_logger().warn(f'distance re-anchored: LiDAR {d:.2f} m (prediction) -> {df:.2f} m (feet {d_feet:.2f} m)')
                        d, pred = df, None                      # accept the jump (fits() with pred None)
                        self.disagree_t = None
                else:
                    self.disagree_t = None
            else:
                self.disagree_t = None
            if not fits(d) and d_feet is not None:
                d = self._lidar_dist(span, d_feet, tol=max(self.dist_gate, 0.3 * d_feet))
            if not fits(d) and d_feet is not None and pred is None:
                d = None                                        # lock: no LiDAR cluster near the feet -> feet below
            if not fits(d):
                d, self.dist_src = d_feet, 'feet'
            if not fits(d) and pred is not None:
                d, self.dist_src = float(math.hypot(pp[0] - self.cam_x, pp[1])), 'pred'
            if d is not None:
                pc = np.array([self.cam_x + d * math.cos(b), d * math.sin(b)])
                w = self._to_w(pose, pc)
                if tgt is None or j.get('state') == 'REACQUIRED' or j.get('id') != tgt.get('id'):
                    if tgt is None:
                        self.get_logger().info(f'locked person id {j.get("id")} at {d:.2f} m, {math.degrees(b):+.0f} deg ({self.dist_src})')
                    self.target = tgt = {'w': w, 'v': np.zeros(2), 't': now, 'side': 1.0, 'src': 'camera', 'cam_t': now,
                                         'hist': np.zeros(1), 'id': j.get('id'), 'trail': []}
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
                    tr = tgt.setdefault('trail', [])               # 지나온 자리 (탐색 때 가던 방향)
                    tr.append((now, tgt['w'].copy()))
                    while tr and now - tr[0][0] > 2.0:
                        tr.pop(0)
        self._travel(pose)
        self._control(pose, tgt, now)

    def _auto(self, t, now):
        """Auto step on top of the follow command t (Twist), in place: SLOW caps vx, STEP = step-detect mode, vx forward, wz 0."""
        if not self.auto_on:
            if self.step_on_sent:
                self._step_mode(False)
            return
        v_meas = self.v_meas
        jumping = now - self.t_fire < 1.2
        clear = not self._blocked(np.array([1.0, 0.0]), person=False)       # LiDAR corridor 1 m ahead (a box is seen, a 6-8 cm step is not)
        vx, mode, state = self.auto.update(now, t.linear.x, self.step_d, v_meas, jumping, clear, self.odom_s)
        self._step_mode(mode)
        if state == 'STEP':
            t.angular.z = 0.0                                  # no turning while the robot finds / jumps the step
            t.linear.x = vx
        elif state == 'SLOW':
            t.linear.x = min(t.linear.x, vx)

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
        elif (self.ex_on and lost < self.occl_s + self.ex_s and not (self.auto_on and self.auto.state == 'STEP')
              and (self.ex is None or self.ex['phase'] != 'DONE' or self.ex['tgt_t'] != tgt['t'])):
            # (2026-10-28 desktop) never explore while the auto step holds step-detect mode ON: EXPLORE returned before
            # _auto(), so the robot drove around at 0.4 m/s with step mode still on (back off / jump at any block).
            # A step takes ~20 s and the person is often out of view during the jumps -> finish the step first.
            state = 'EXPLORE'
        else:
            state = 'SEARCH'
            t.angular.z = math.copysign(self.search_wz, tgt['side'])
        if state != 'EXPLORE' and (state != 'SEARCH' or tgt is None):
            self.ex = None
        if state == 'EXPLORE':
            if self._explore(pose, tgt, now, t):
                return                                       # Nav2 drives (cmd_vel/auto from its controller)
            self.pub.publish(self._front_stop(self._smooth(t, now)))
            return
        pb = None
        if state in ('FOLLOW', 'OCCLUDED'):
            pw = self._predict(tgt, now)
            pb = self._to_b(pose, pw)
            if abs(pb[1]) > 0.1:
                tgt['side'] = math.copysign(1.0, pb[1])
            if self._blocked(pb):
                self.blocked_t = now
            stepping = self.auto_on and self.auto.state in ('SLOW', 'STEP')   # the LiDAR 'obstacle' is the step itself
            # person far to the side (> nav_max_bearing_deg): no Nav2 — turn to them first or they leave the 124 deg view
            # (2026-10-28 GCS diag: under Nav2 the bearing ran 0 -> -58 deg, lost 7 s; +43..+53 deg in another run)
            side = abs(math.atan2(pb[1], pb[0])) > self.nav_max_b
            if not stepping and not side and now - self.blocked_t < self.nav_hold and self._nav(pose, pw, pb, now):
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
            if self.feet_on and state == 'FOLLOW' and now - self.feet_t < 0.3:   # 발 픽셀 추종 (feet_follow, 위 설명)
                t.linear.x = float(np.clip(k * self.feet_k * (self.feet_px - self.feet_dv) * max(0.0, math.cos(b)),
                                           -self.vx_max * 0.5, self.vx_max))
            if abs(b) > self.nav_max_b and now - self.blocked_t < self.nav_hold:
                t.linear.x = 0.0                                # blocked and the person far to the side: turn in place
        if state in ('FOLLOW', 'OCCLUDED') or (self.auto_on and self.auto.state == 'STEP'):
            if self.auto_on and self.auto.state == 'STEP' and state not in ('FOLLOW', 'OCCLUDED'):
                t.linear.x = self.auto.step_v                     # lost the person while jumping: finish the step first
            self._auto(t, time.monotonic())
        if self.auto_on and self.auto.state == 'STEP':
            # game day 2026-10-28 (user: manual D-pad + constant 0.7 forward works, follow + camera step does not): like the pad,
            # a constant forward command, no turning, no rate limit (the smoother turned 0 <-> 0.7 chatter into 0.3-0.5 = the
            # range where the re-run triggers at its 0.56-0.58 m/s overshoot), no Nav2. 0 only right next to the person.
            close = pb is not None and math.hypot(*pb) < self.step_stop_m
            t.linear.x, t.angular.z = (0.0 if close else float(t.linear.x)), 0.0   # _auto: step_v, 0 just after a landing
            self.sm_v, self.sm_w, self.sm_t = t.linear.x, 0.0, now
            self.pub.publish(t)
            self._status(state, pb)
            return
        cap = t.linear.x if self.auto_on and self.auto.state == 'SLOW' else None
        t = self._smooth(t, now)                           # accel / decel limits (PR #8) ...
        if cap is not None and t.linear.x > cap:           # ... never above the auto-step speed (SLOW 0.3)
            t.linear.x = self.sm_v = float(cap)
        self.pub.publish(self._front_stop(t))
        self._status(state, pb)

    def _explore(self, pose, tgt, now, t):
        """EXPLORE: go to the waypoints behind the occluder, then sweep left / right. Fills t (direct) or returns True
        when Nav2 drives. A plan is made once per loss (tgt['t'] = last time the person was seen)."""
        rob = np.array(pose[:2])
        if self.ex is None or self.ex['tgt_t'] != tgt['t']:
            last, v = np.array(tgt['w'], float), np.array(tgt['v'], float)
            tr = [(t_, w_) for t_, w_ in tgt.get('trail', []) if tgt['t'] - t_ <= 1.0]
            if len(tr) >= 3 and tr[-1][0] - tr[0][0] > 0.3:     # 마지막 1 s 궤적으로 가던 방향 (EMA 속도보다 덜 튐)
                v = (tr[-1][1] - tr[0][1]) / (tr[-1][0] - tr[0][0])
            spd = float(np.linalg.norm(v))
            u = v / spd if spd > 0.15 else (last - rob) / max(1e-3, float(np.linalg.norm(last - rob)))
            n = np.array([-u[1], u[0]]) * tgt['side']        # 사라진 쪽 (side: + = 로봇 왼쪽에서 마지막으로 봄)
            w1 = last + u * self.ex_ahead
            # 사라진 자리 (사람이 지나간 길이라 대체로 뚫림) → 가던 방향 앞 → 사라진 쪽 옆
            self.ex = {'tgt_t': tgt['t'], 'wps': [last, w1, w1 + n * self.ex_side], 'i': 0, 'phase': 'GO', 't0': now,
                       'scan_t': None, 'stuck_t': None}
            self.get_logger().info(f'explore: lost {now - tgt["t"]:.1f} s, last ({last[0]:.2f}, {last[1]:.2f}) v ({v[0]:+.2f}, {v[1]:+.2f}) -> go ({w1[0]:.2f}, {w1[1]:.2f}) then sweep')
        ex = self.ex
        if ex['phase'] == 'DONE':
            return False
        if ex['phase'] == 'GO':
            gw = ex['wps'][ex['i']]
            pb = self._to_b(pose, gw)
            d = float(np.hypot(*pb))
            if d < 0.35 and ex['i'] + 1 < len(ex['wps']) and ex['i'] == 0:   # 사라진 자리는 지나가는 점 (훑지 않음)
                ex['i'], ex['t0'] = 1, now
                return self._explore(pose, tgt, now, t)
            if d < 0.3 or now - ex['t0'] > 10.0:                # 닿음 / 너무 오래: 훑기
                ex['phase'], ex['scan_t'] = 'SCAN', now
                self._nav_cancel()
            else:
                blocked = self._blocked(pb, person=False) if d > 0.6 else False
                if blocked:
                    self.blocked_t = now
                if now - self.blocked_t < self.nav_hold and self._nav(pose, gw, pb, now, stand_off=0.0):
                    self._status('EXPLORE', pb, f'GO {ex["i"] + 1}/{len(ex["wps"])} nav')
                    return True
                self._nav_cancel()
                b = math.atan2(pb[1], pb[0])
                if blocked:                                     # Nav2 없음 / 실패: 15° 씩 비켜 뚫린 방향 (0.8 m 통로) 으로 돌아감
                    free = [b + k * math.radians(15) * sg for k in range(1, 7) for sg in (tgt['side'], -tgt['side'])
                            if not self._blocked(np.array([math.cos(b + k * math.radians(15) * sg),
                                                           math.sin(b + k * math.radians(15) * sg)]) * 1.25, person=False)]
                    if free:
                        b, blocked = free[0], False
                t.angular.z = float(np.clip(self.k_yaw * b, -self.wz_max, self.wz_max))
                t.linear.x = 0.0 if blocked else float(min(self.ex_v, 0.8 * d) * max(0.0, math.cos(b)))
                if blocked:                                     # 어느 쪽도 막힘: 2 s 뒤 훑기
                    ex['stuck_t'] = ex['stuck_t'] or now
                    if now - ex['stuck_t'] > 2.0:
                        ex['phase'], ex['scan_t'] = 'SCAN', now
                else:
                    ex['stuck_t'] = None
                self._status('EXPLORE', pb, f'GO {ex["i"] + 1}/{len(ex["wps"])}')
                return False
        # SCAN: 사라진 쪽으로 explore_scan_deg, 반대쪽으로 2 배, 가운데로 — search_wz 로
        wz = max(0.2, self.search_wz)
        t1 = math.radians(self.ex_scan_deg) / wz
        dt = now - ex['scan_t']
        side = tgt['side']
        t.linear.x = 0.0
        if dt < t1:
            t.angular.z = side * wz
        elif dt < 3 * t1:
            t.angular.z = -side * wz
        elif dt < 4 * t1:
            t.angular.z = side * wz
        else:
            t.angular.z = 0.0
            if ex['i'] + 1 < len(ex['wps']):
                ex['i'], ex['phase'], ex['t0'], ex['stuck_t'] = ex['i'] + 1, 'GO', now, None
            else:
                ex['phase'] = 'DONE'                           # 다 훑음: SEARCH (제자리 회전) 로
        self._status('EXPLORE', None, f'SCAN {ex["i"] + 1}/{len(ex["wps"])}')
        return False

    def _status(self, state, pb=None, ex_txt=None):
        """As person_follow (HUD reads 'target d m @ b deg'), + ' [AUTO <state> <step d>]' when the auto step is on."""
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state
        s = state
        if ex_txt is not None:                               # EXPLORE: 'EXPLORE GO 1/2 to 1.20 m @ +30 deg' (pb = waypoint, not the person)
            s += f' {ex_txt}' + (f' to {math.hypot(*pb):.2f} m @ {math.degrees(math.atan2(pb[1], pb[0])):+.0f} deg' if pb is not None else '')
        elif pb is not None:
            s += f' target {math.hypot(*pb):.2f} m @ {math.degrees(math.atan2(pb[1], pb[0])):+.0f} deg'
            if self.target is not None:
                s += f" [{self.target.get('src', 'camera')}]"
        if getattr(self, 'front_hit', False):
            s += ' [STOP lidar]'
        if getattr(self, 'feet_dv', None) is not None and time.monotonic() - self.feet_t < 0.5:
            s += f' [feet {self.feet_dv:.0f}/{self.feet_px:.0f}px' + (' ON' if self.feet_on else '') + ']'
        if getattr(self, 'auto_on', False):
            s += f' [AUTO {self.auto.state}' + (f' {self.step_d:.1f}m' if self.step_d else '') + ']'
        self.st.publish(String(data=s))


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
