"""휠-레그 제어기 — LQR(바퀴) + VMC(다리) + 상태 추정 + 속도 제한 + 들림 감지 + 점프 상태머신.

**제어기는 이것 하나다** (2026-10-03 사용자: "지형만 다르고 제일 정확한 제어기 하나만, 실기 이식 가능한 LQR VMC").
로봇 N 대를 배열로 한 번에 계산한다 (numpy): 실기·창은 N = 1, pv robust 는 수십 대, pv harsh 는 수천 대 — 모두 이 코드.
시뮬 전용 코드(Isaac)는 없다: 입력은 한 스텝의 센서·기구학 값 (Frame, 배열 앞 차원 = 로봇), 출력은 행동·다리 게인·자중 피드포워드.
실기 C++/Simulink 로 옮길 때 이 파일이 규격이다 (로봇 1 대 = 배열 길이 1, 마스크 = if).

단위·부호 (climb_test 와 같음):
  pitch = asin(g_x) (+ = 앞으로 숙임), roll = asin(g_y) (+ = 왼쪽이 낮음)
  바퀴 토크·속도: +y 규약 (+ = 앞으로 굴림), 고관절 토크: + = 다리를 펴며 몸을 받침
  행동 act[:, 0:2] = (다리 목표 높이 - h_ref) / 0.12, act[:, 2:4] = 바퀴 토크 / wheel_tau_max
"""
import collections
import math
import os
from dataclasses import dataclass

import numpy as np

import lqr_vmc

H_MIN, H_MAX = 0.1225, 0.2425
R_WHEEL = float(os.environ.get("WB_WHEEL_R", "0.070"))   # leg_map 과 같은 환경변수 (기본 140 mm)
W_WHEEL_MAX = 18.85                    # 바퀴 모터 한계 (명목, 제어기가 믿는 값)
HALF_TRACK = 0.094
DT = 0.005
DRIVE, RETRACT, EXTRACT, FLY, DESCEND, LAND = range(6)
PHASES = ("drive", "retract", "extract", "fly", "descend", "land")


@dataclass
class Frame:
    """한 스텝의 입력 (로봇 N 대, 배열 앞 차원 = 로봇). 참값 계열(truth_*)은 est='truth' 일 때만 쓴다."""
    t: float
    g_b: np.ndarray            # (N, 3) 투영 중력 (몸체 좌표, 참값)
    w_b: np.ndarray            # (N, 3) 몸체 각속도 (몸체 좌표, 참값) — 자이로
    h: np.ndarray              # (N, 2) 다리 높이 [L, R] (관절값)
    tau_hip: np.ndarray        # (N, 2) 고관절 토크 (+ = 펴며 받침)
    w_wheel_joint: np.ndarray  # (N, 2) 바퀴 관절 속도 (+y 규약, 엔코더)
    w_wheel_abs: np.ndarray    # (N, 2) 바퀴 절대 회전 (엔코더 + 4절 링크 회전, 실기는 기구학으로 계산)
    th_kin: np.ndarray         # (N,) 몸체 좌표의 무게중심 방향 각 atan2(x, z) (명목 질량 + 기구학)
    l_pend: np.ndarray         # (N,) 진자 길이 (명목)
    wx: np.ndarray             # (N,) 바퀴 중심 x (월드, 발동·기록용)
    wheel_z_min: np.ndarray    # (N,) 두 바퀴 중 낮은 바닥 높이 (기록용)
    yaw: np.ndarray            # (N,)
    sf: np.ndarray             # (N,) IMU 비력 크기 [g]
    truth_th: np.ndarray       # (N,) 참 진자각 (est='truth')
    truth_v: np.ndarray        # (N,) 참 바퀴축 속도 (est='truth')
    motor_scale: object = 1.0  # (N,) 바퀴 모터 한계 추정 / 명목 (실기: 배터리 전압 / 만충 전압)
    a_fwd: object = 0.0        # (N,) IMU 전후 가속 [m/s^2] (진행 방향, 턱 감지)


class RollPI:
    """좌우 다리 길이 차 Δ = hL - hR 의 roll PI (lqr_vmc.RollPI 의 배열판, 수식 같음). mask 인 로봇만 상태를 바꾼다."""

    def __init__(self, n, track=0.198):
        self.track, self.i = track, np.zeros(n)

    def reset(self, diff=0.0, mask=None):
        self.i = np.where(mask, diff, self.i) if mask is not None else np.broadcast_to(np.asarray(diff, float), self.i.shape).copy()

    def __call__(self, roll, dt, kp, ki, lim, freeze, leak, rate, kd, mask):
        e = self.track * np.sin(roll)
        i = np.where(freeze, self.i, self.i + ki * e * dt)
        i = np.clip(i * (1.0 - leak * dt), -lim, lim)
        self.i = np.where(mask, i, self.i)
        return np.clip(self.i + kp * e + kd * self.track * rate, -lim, lim)


def lqr_torque(lqr, l, x_err, v_err, th, thd):
    """τ = -K(l) [x - x_ref, v - v_ref, θ, θ'] 를 로봇마다 (진자 길이 l 로 K 보간, lqr_vmc.WheelLQR.torque 와 같음)."""
    K = np.stack([np.interp(l, lqr.l_grid, lqr.K[:, j]) for j in range(4)], axis=-1)
    return -(K[:, 0] * x_err + K[:, 1] * v_err + K[:, 2] * th + K[:, 3] * thd)


class WBController:
    def __init__(self, P, lqr: lqr_vmc.WheelLQR, m_pend: float, n: int = 1, seed: int = 0, edges=()):
        """P: TUNE 값 (P.key). n: 로봇 수. edges: 자동 점프 모서리 x [m] (모든 로봇 같은 코스, 없으면 자동 점프 안 함)."""
        self.P, self.lqr, self.m_pend, self.n, self.edges = P, lqr, m_pend, n, list(edges)
        self.rng = np.random.default_rng(seed)
        self.bias = self.rng.uniform(-1, 1, (n, 2)) * math.radians(P.imu_tilt_bias_deg)     # 로봇마다 IMU (pitch, roll) 치우침
        self.roll_pi = RollPI(n)
        self.q = collections.deque(maxlen=16)
        self.reset()

    def reset(self, mask=None):
        """mask: 다시 시작할 로봇 (없으면 전부). 센서 치우침은 그대로 (같은 로봇)."""
        n = self.n
        m = np.ones(n, bool) if mask is None else np.asarray(mask, bool)
        z = lambda name: np.where(m, 0.0, getattr(self, name, np.zeros(n)))  # noqa: E731
        for k in ("t_phase", "x_err", "t_un", "t_ld", "g_vf", "g_i", "g_ref", "bump_t", "bump_quiet", "vf", "rf",
                  "th_bias", "v_prev", "t_takeoff"):
            setattr(self, k, z(k))
        self.phase = np.where(m, DRIVE, getattr(self, "phase", np.zeros(n, int))).astype(int)
        self.next_edge = np.where(m, 0, getattr(self, "next_edge", np.zeros(n, int))).astype(int)
        self.h0 = np.where(m, self.P.idle_h, getattr(self, "h0", np.zeros(n)))
        self.air_pmin = np.where(m, 99.0, getattr(self, "air_pmin", np.full(n, 99.0)))
        self.lift_on = np.where(m, False, getattr(self, "lift_on", np.zeros(n, bool)))
        self.soft = np.where(m, False, getattr(self, "soft", np.zeros(n, bool)))
        old = getattr(self, "jumps", [[] for _ in range(n)])
        self.jumps = [[] if m[i] else old[i] for i in range(n)]
        self.roll_pi.reset(0.0, m)
        if mask is None:
            self.q.clear()
        self.dbg, self.jump_blocked = {}, [""] * n           # 화면·기록용 (제어에는 안 씀)

    # --- 센서 ------------------------------------------------------------------------------------
    def _sense(self, f: Frame):
        P, n = self.P, self.n
        p_ = np.arcsin(np.clip(f.g_b[:, 0], -1.0, 1.0))
        r_ = np.arcsin(np.clip(f.g_b[:, 1], -1.0, 1.0))
        if P.est == "sensors":
            nz = self.rng.normal
            tn, gn = math.radians(P.imu_tilt_noise_deg), P.imu_gyro_noise
            return dict(pitch=p_ + self.bias[:, 0] + nz(0, tn, n), roll=r_ + self.bias[:, 1] + nz(0, tn, n),
                        gx=f.w_b[:, 0] + nz(0, gn, n), gy=f.w_b[:, 1] + nz(0, gn, n), gz=f.w_b[:, 2] + nz(0, gn, n))
        return dict(pitch=p_, roll=r_, gx=f.w_b[:, 0].copy(), gy=f.w_b[:, 1].copy(), gz=f.w_b[:, 2].copy())

    def _state(self, f: Frame, S):
        """(진자각, 각속도, 진자 길이, 바퀴축 속도, yaw rate)."""
        P = self.P
        if P.est != "sensors":
            return f.truth_th, S["gy"], f.l_pend, f.truth_v, S["gz"]
        th = S["pitch"] + f.th_kin
        wabs = f.w_wheel_abs + self.rng.normal(0, P.enc_vel_noise, f.w_wheel_abs.shape) if P.enc_vel_noise > 0 else f.w_wheel_abs
        on = f.tau_hip >= P.contact_tau_min                       # 땅 짚은 바퀴만
        cnt = on.sum(1)
        v_raw = R_WHEEL * (wabs * on).sum(1) / np.maximum(cnt, 1)
        a = 1.0 - math.exp(-2 * math.pi * P.v_lpf_hz * DT) if P.v_lpf_hz > 0 else 1.0
        self.vf = np.where(cnt > 0, self.vf + a * (v_raw - self.vf), self.vf)   # 둘 다 뜸 -> 직전 추정 유지
        return th, S["gy"], f.l_pend, self.vf, S["gz"]

    # --- 한 스텝 ----------------------------------------------------------------------------------
    def step(self, f: Frame, vx, wz, h_ref, jump=False, h_mid=None):
        """-> (act (N,4), leg_kp (N,), leg_kd (N,), ff_force (N,) 다리 하나당 자중 보상 힘 [N], info).
        vx, wz: 명령 (N,) 또는 숫자. h_ref: 명령의 높이 기준 (auto = idle_h). jump: 조종자 점프 요청 (패드 Y).
        h_mid: 다리 높이 가운데 (패드 수동 높이, None = idle_h)."""
        P, t, n = self.P, f.t, self.n
        vec = lambda x, dt_=float: np.broadcast_to(np.asarray(x, dt_), (n,)).copy()  # noqa: E731
        vx, wz, h_ref, jump = vec(vx), vec(wz), vec(h_ref), vec(jump, bool)
        motor_scale, a_fwd = vec(f.motor_scale), vec(f.a_fwd)
        act = np.zeros((n, 4))
        S = self._sense(f)
        th, thd, l_p, v_now, wz_now = self._state(f, S)
        # 균형점 자동 보정: 멈춰 서 있고 흔들림이 작을 때 (참 진자각 = 0 이어야 하는 순간) 추정 진자각을 천천히 학습
        # (경사를 일정 속도로 오를 땐 앞으로 숙이는 게 정상이라 그걸 오차로 배우면 안 됨 — pv robust 경사로 2/8)
        acc_ = (v_now - self.v_prev) / DT; self.v_prev = v_now  # noqa: E702
        ad = ((P.bal_adapt > 0) & (self.phase == DRIVE) & ~self.lift_on & (np.abs(acc_) < 0.3) & (np.abs(thd) < 0.3)
              & (np.abs(v_now) < 0.05) & (np.abs(vx) < 0.02))
        lim = math.radians(P.bal_adapt_max_deg)
        self.th_bias = np.where(ad, np.clip(self.th_bias + P.bal_adapt * DT * (th - self.th_bias), -lim, lim), self.th_bias)
        th = th - self.th_bias
        w_max = W_WHEEL_MAX * motor_scale
        a_r = 1.0 - math.exp(-2 * math.pi * P.roll_rate_lpf_hz * DT) if P.roll_rate_lpf_hz > 0 else 1.0
        self.rf = self.rf + a_r * (-S["gx"] - self.rf)
        leg_kp, leg_kd = np.full(n, float(P.vmc_kp)), np.full(n, float(P.vmc_kd))

        # 점프 상태머신 (모서리 자동 발동, 또는 조종자 요청 jump) — 전이 조건은 모두 이번 스텝 시작 때의 단계로 본다
        ph, tp = self.phase.copy(), t - self.t_phase
        ne = len(self.edges)
        auto = (np.zeros(n, bool) if ne == 0 else
                (self.next_edge < ne) & (f.wx >= np.asarray(self.edges)[np.minimum(self.next_edge, ne - 1)] - P.trigger))
        blocked = jump & (ph == DRIVE) & ((v_now < P.jump_min_v) | (np.abs(wz_now) > P.jump_max_wz))   # 제자리·후진·회전 중 점프 막기
        self.jump_blocked = [""] * n
        for i in np.flatnonzero(blocked):
            self.jump_blocked[i] = f"{v_now[i] * 3.6:+.1f} km/h" if v_now[i] < P.jump_min_v else f"turning {wz_now[i]:+.1f} rad/s"
        jump = jump & ~blocked
        m0 = (ph == DRIVE) & ~self.lift_on & (auto | jump)
        m1 = (ph == RETRACT) & (tp >= P.t_retract)
        m2 = (ph == EXTRACT) & ((f.h.min(1) >= H_MAX - 0.008) | (tp >= 0.25))
        m3 = (ph == FLY) & (tp >= P.t_tuck)
        m4 = (ph == DESCEND) & (((tp > 0.04) & (np.abs(f.tau_hip).max(1) > P.contact_tau)) | (t - self.t_takeoff >= P.t_fly_max))
        m5 = (ph == LAND) & (tp >= P.land_s)
        for i in np.flatnonzero(m0):
            self.jumps[i].append(dict(edge=int(self.next_edge[i]), t_trigger=t, x_trigger=float(f.wx[i])))
        for i in np.flatnonzero(m2):
            self.jumps[i][-1].update(t_takeoff=t, x_takeoff=float(f.wx[i]), pitch_takeoff=round(math.degrees(S["pitch"][i]), 1))
        for i in np.flatnonzero(m4):
            self.jumps[i][-1].update(t_land=t, x_land=float(f.wx[i]), wheel_bottom=float(f.wheel_z_min[i]),
                                     pitch_land=round(math.degrees(S["pitch"][i]), 1), pitch_min_air=round(float(self.air_pmin[i]), 1),
                                     v_land=round(float(f.truth_v[i]), 2),                               # 몸(바퀴축) 전진 속도
                                     wheel_v_land=round(float(np.mean(f.w_wheel_abs[i])) * R_WHEEL, 2))   # 바퀴 둘레 속도
        self.h0 = np.where(m0, f.h.mean(1), self.h0)
        self.bump_t = np.where(m0, 0.0, self.bump_t)
        self.air_pmin = np.where(m2, 99.0, self.air_pmin)
        self.t_takeoff = np.where(m2, t, self.t_takeoff)
        self.soft = np.where(m3, True, np.where(m5, False, self.soft))
        self.next_edge = self.next_edge + m5
        self.bump_quiet = np.where(m5, t + 1.0, self.bump_quiet)                  # 착지 충격을 턱으로 보지 않게
        self.phase = np.select([m0, m1, m2, m3, m4, m5], [RETRACT, EXTRACT, FLY, DESCEND, LAND, DRIVE], ph)
        self.t_phase = np.where(m0 | m1 | m2 | m3 | m4 | m5, t, self.t_phase)
        if ne:   # 마지막 모서리 넘어 0.25 m 더 들어간 뒤 멈춘다 (모서리에 서면 굴러 내려옴, 0.4 면 1 m 평대를 지나침)
            vx = np.where((self.next_edge >= ne) & (f.wx >= self.edges[-1] + 0.25), 0.0, vx)
        ph = self.phase

        air = (ph == FLY) | (ph == DESCEND)
        self.air_pmin = np.where(air, np.minimum(self.air_pmin, np.degrees(S["pitch"])), self.air_pmin)
        ffF = np.zeros(n)
        ctl = (ph == DRIVE) | (ph == RETRACT) | (ph == EXTRACT) | (ph == LAND)
        th_ref = np.where(ph == RETRACT, math.radians(P.retract_lean), 0.0)
        vm = np.minimum(P.vmax_kmh / 3.6, P.vmax_motor_frac * w_max * R_WHEEL)
        g_vf = self.g_vf + (1.0 - math.exp(-2 * math.pi * P.speed_lpf_hz * DT)) * (v_now - self.g_vf)
        e = np.abs(g_vf) - vm
        g_i = np.clip(self.g_i + P.brake_ki * e * DT, 0.0, vm)
        v_lim = np.maximum(0.0, vm - (P.brake_kp * np.maximum(e, 0.0) + g_i))
        # 턱 감지 -> 감속 (climb_test TUNE bump_slow 설명). 브레이크 vm 은 그대로, 목표만 낮춘다
        if getattr(P, "bump_slow", False):
            a_f = a_fwd + self.rng.normal(0.0, P.imu_acc_noise, n)
            hit = (t >= self.bump_quiet) & ((np.abs(thd) > P.bump_rate) | (a_f < -P.bump_acc))
            bump_t = np.where(ph == DRIVE, np.where(hit, P.bump_hold_s, np.maximum(0.0, self.bump_t - DT)), 0.0)
        else:
            bump_t = np.zeros(n)
        v_lim = np.where(bump_t > 0.0, np.minimum(v_lim, P.bump_vmax), v_lim)
        if getattr(P, "turn_slow", False):                  # 돌 때 안쪽 바퀴를 느리게 (climb_test TUNE turn_slow)
            v_lim = np.minimum(v_lim, np.maximum(0.0, vm - HALF_TRACK * np.abs(wz)))
        tgt = np.clip(vx, -v_lim, v_lim)
        g_ref = self.g_ref + np.clip(tgt - self.g_ref, -P.accel_max * DT, P.accel_max * DT)
        v_ref = g_ref.copy()
        x_err = np.where(np.abs(vx) > v_lim + 1e-3, 0.0, self.x_err)
        if P.speed_guard < 1.0:
            ww = np.abs(f.w_wheel_joint).max(1) / w_max
            guard = ww > P.speed_guard
            cut = np.minimum(1.0, (ww - P.speed_guard) / (1.0 - P.speed_guard))
            v_ref = np.where(guard, np.where(v_now * vx >= 0, v_now * (1.0 - 0.6 * cut), vx), v_ref)
            x_err = np.where(guard, 0.0, x_err)
        x_err = np.clip(x_err + (v_now - v_ref) * DT, -0.3, 0.3)
        tau_w = lqr_torque(self.lqr, l_p, x_err, v_now - v_ref, th - th_ref, thd)
        self.dbg = dict(v_ref=v_ref, v_lim=v_lim, vm=vm, brake=v_lim < vm - 0.02, bump=bump_t > 0.0, x_err=x_err)
        if getattr(P, "turn_limit", True):                 # 바깥 바퀴 <= 모터 x wheel_margin 이 되게 회전 상한 (climb_test TUNE turn_limit)
            # 속도는 명령(스틱)으로 본다: 실제 속도로 보면 자갈길에서 추정이 튀어 상한이 출렁여 넘어짐 (stones_turn 16/16 -> 14/16,
            # 2026-09-30), 돌며 느려질수록 더 돌아 넘어짐 (fast_turn 7/16, 2026-09-28). 앞 스틱을 민 만큼 회전 상한이 낮아진다
            v_cmd = np.minimum(np.abs(vx), vm)
            wz_lim = np.maximum(0.5, (P.wheel_margin * w_max * R_WHEEL - v_cmd) / HALF_TRACK)
            wz = np.where(ctl, np.clip(wz, -wz_lim, wz_lim), wz)
        tau_y = P.yaw_kd * (wz - wz_now)
        act[:, 2] = np.where(ctl, np.clip((0.5 * tau_w - tau_y) / P.wheel_tau_max, -1.0, 1.0), 0.0)
        act[:, 3] = np.where(ctl, np.clip((0.5 * tau_w + tau_y) / P.wheel_tau_max, -1.0, 1.0), 0.0)
        self.g_vf, self.g_i, self.g_ref = (np.where(ctl, a, b) for a, b in ((g_vf, self.g_vf), (g_i, self.g_i), (g_ref, self.g_ref)))
        self.x_err = np.where(ctl, x_err, self.x_err)
        self.bump_t = np.where(ctl, bump_t, self.bump_t)

        # 들림 / 착지 감지 (부호 있는 고관절 토크 + IMU 비력)
        unl = f.tau_hip.max(1) < P.contact_tau_min
        ldd = (f.tau_hip.max(1) >= P.contact_tau_min) & (f.sf > P.land_sf_min)
        a_ = (ph == DRIVE) & ~self.lift_on
        b_ = (ph == DRIVE) & self.lift_on
        self.t_un = np.where(a_, np.where(unl, self.t_un + DT, 0.0), self.t_un)
        up = a_ & (self.t_un >= P.lift_detect_s)
        self.t_ld = np.where(b_, np.where(ldd, self.t_ld + DT, 0.0), np.where(up, 0.0, self.t_ld))
        down = b_ & (self.t_ld >= P.land_detect_s)
        self.lift_on = (self.lift_on | up) & ~down
        self.t_un = np.where(down, 0.0, self.t_un)
        self.x_err = np.where(down, 0.0, self.x_err)
        self.g_i, self.g_ref, self.g_vf = (np.where(down, 0.0, self.g_i), np.where(down, v_now, self.g_ref), np.where(down, v_now, self.g_vf))
        self.roll_pi.reset(0.0, down)

        lifted = (ph == DRIVE) & self.lift_on                    # 들린 동안: 균형 끔, 바퀴 감쇠, 다리 IDLE
        act[:, 2:] = np.where(lifted[:, None], np.clip(-P.lift_wheel_kd * f.w_wheel_joint / P.wheel_tau_max, -1.0, 1.0), act[:, 2:])
        act[:, 0:2] = np.where(lifted[:, None], ((P.idle_h - h_ref) / 0.12)[:, None], act[:, 0:2])
        self.roll_pi.reset(0.0, lifted)
        self.x_err = np.where(lifted, 0.0, self.x_err)
        self.g_i, self.g_ref, self.g_vf = (np.where(lifted, 0.0, x) for x in (self.g_i, self.g_ref, self.g_vf))
        drv = (ph == DRIVE) & ~self.lift_on
        roll_ref = np.clip(np.arctan(P.turn_lean * self.g_ref * wz / 9.81), -math.radians(20), math.radians(20))
        rl = S["roll"] - roll_ref                                 # 회전 중 안쪽으로 기울이기 (Ascento lean)
        airborne = f.tau_hip.min(1) < P.contact_tau_min
        dlt = self.roll_pi(rl, DT, P.roll_kp, P.roll_ki, P.level_max, freeze=airborne | (np.abs(np.degrees(rl)) > P.roll_freeze_deg),
                           leak=P.roll_leak, rate=self.rf, kd=P.roll_kd, mask=drv)
        hc = np.full(n, float(P.idle_h)) if h_mid is None else vec(h_mid)
        tl = np.clip(hc + 0.5 * dlt, H_MIN, H_MAX); tr = np.clip(hc - 0.5 * dlt, H_MIN, H_MAX)  # noqa: E702
        act[:, 0] = np.where(drv, (tl - h_ref) / 0.12, act[:, 0])
        act[:, 1] = np.where(drv, (tr - h_ref) / 0.12, act[:, 1])
        ffF = np.where(drv | (ph == LAND), 0.5 * self.m_pend * 9.81, ffF)
        jmp = (ph == RETRACT) | (ph == EXTRACT) | (ph == FLY) | (ph == DESCEND)
        self.roll_pi.reset(f.h[:, 0] - f.h[:, 1], jmp)

        # 점프 단계의 다리 목표와 공중 바퀴 pitch PD
        legT = (ph != DRIVE)
        if legT.any():
            tgt_h = np.select([ph == RETRACT, ph == EXTRACT, ph == FLY], [H_MIN, H_MAX, H_MIN], P.h_land)
            tgt_h = np.where(ph == RETRACT, self.h0 + (H_MIN - self.h0) * np.minimum(1.0, tp / (0.75 * P.t_retract)), tgt_h)
            act[:, 0:2] = np.where(legT[:, None], ((tgt_h - h_ref) / 0.12)[:, None], act[:, 0:2])
            pd_set = (FLY, DESCEND) if P.pd_from == "fly" or P.extract_wheels != "pd" else (EXTRACT, FLY, DESCEND)
            pd = np.isin(ph, pd_set)
            if P.air_ctrl:
                ref = np.select([ph == EXTRACT, ph == DESCEND], [P.extract_pitch, getattr(P, "land_pitch", P.air_pitch)], P.air_pitch)
                u = P.air_kp * (S["pitch"] - np.radians(ref)) + P.air_kd * S["gy"]
                act[:, 2:] = np.where(pd[:, None], np.clip(u / P.air_tau, -1.0, 1.0)[:, None], act[:, 2:])
            else:
                act[:, 2:] = np.where(pd[:, None], 0.0, act[:, 2:])
            leg_kp = np.where(legT & self.soft, P.land_kp, np.where(legT & (ph != LAND), P.leg_kp, leg_kp))
            leg_kd = np.where(legT & self.soft, P.land_kd, np.where(legT & (ph != LAND), P.leg_kd, leg_kd))

        # 바퀴 마찰 보상 (TUNE fric_comp_nm, 0 = 끔): 바퀴마다 fric_comp_nm x tanh(관절 속도 / fric_comp_w) 를 토크에 더한다.
        # 들린 동안(바퀴 감쇠)은 안 더함
        # fric_comp_static_nm: 바퀴가 거의 멈췄을 때 (1 - |tanh(w / fric_comp_w)|) 만큼 지령 토크 방향으로 더 — 정지 마찰 넘기기 (0 = 끔)
        # fric_comp_cmd_nm: 바퀴 속도와 상관없이 지령 방향으로 항상 더 — 드라이브가 작은 지령을 무시(데드밴드)하는 경우 (0 = 끔)
        fc, fs, fd = getattr(P, "fric_comp_nm", 0.0), getattr(P, "fric_comp_static_nm", 0.0), getattr(P, "fric_comp_cmd_nm", 0.0)
        if fc > 0 or fs > 0 or fd > 0:
            sv = np.tanh(f.w_wheel_joint / P.fric_comp_w)
            tcmd = act[:, 2:] * P.wheel_tau_max
            comp = fc * sv + fs * (1.0 - np.abs(sv)) * np.tanh(tcmd / 0.05)
            if fd > 0:
                comp = comp + fd * np.tanh(tcmd / P.fric_comp_cmd_w)
            act[:, 2:] = np.where(lifted[:, None], act[:, 2:], np.clip(act[:, 2:] + comp / P.wheel_tau_max, -1.0, 1.0))

        # 제어 지연 (+ 가끔 한 주기 더) — 로봇마다 따로
        if P.delay_ms > 0 or P.jitter_ms > 0:
            self.q.append(act.copy())
            dly = int(round(P.delay_ms / 5.0)) + (self.rng.random(n) < P.jitter_ms / 5.0)
            k = np.maximum(0, len(self.q) - 1 - dly)
            act = np.stack([self.q[k[i]][i] for i in range(n)]) if n < 64 else self._gather(k)
        return act, leg_kp, leg_kd, ffF, dict(th=th, thd=thd, v=v_now, pitch=S["pitch"], roll=S["roll"], **self.dbg)

    def _gather(self, k):
        Q = np.stack(self.q)                                       # (len, N, 4)
        return Q[k, np.arange(self.n)]
