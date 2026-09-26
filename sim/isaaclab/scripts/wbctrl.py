"""휠-레그 제어기 (로봇 1 대 단위) — LQR(바퀴) + VMC(다리) + 상태 추정 + 속도 제한 + 들림 감지 + 점프 상태머신.

climb_test.py 의 ctrl="lqr" 경로를 로봇마다 따로 돌 수 있게 떼어 낸 것 (robust_suite.py 가 여러 대를 동시에 돌린다).
시뮬 전용 코드(Isaac)는 없다: 입력은 한 스텝의 센서·기구학 값 (Frame), 출력은 행동·다리 게인·자중 피드포워드.
실기 C++/Simulink 로 옮길 때 이 파일이 규격이다.

단위·부호 (climb_test 와 같음):
  pitch = asin(g_x) (+ = 앞으로 숙임), roll = asin(g_y) (+ = 왼쪽이 낮음)
  바퀴 토크·속도: +y 규약 (+ = 앞으로 굴림), 고관절 토크: + = 다리를 펴며 몸을 받침
  행동 act[0:2] = (다리 목표 높이 - h_ref) / 0.12, act[2:4] = 바퀴 토크 / wheel_tau_max
"""
import collections
import math
from dataclasses import dataclass

import numpy as np

import lqr_vmc

H_MIN, H_MAX = 0.1225, 0.2425
R_WHEEL = 0.06
W_WHEEL_MAX = 18.85                    # 바퀴 모터 한계 (명목, 제어기가 믿는 값)
HALF_TRACK = 0.094
DT = 0.005


@dataclass
class Frame:
    """한 스텝의 입력 (로봇 1 대). 참값 계열(g_true 등)은 est='truth' 일 때와 잡음 얹을 때만 쓴다."""
    t: float
    g_b: np.ndarray            # 투영 중력 (몸체 좌표, 참값)
    w_b: np.ndarray            # 몸체 각속도 (몸체 좌표, 참값) — 자이로
    h: np.ndarray              # 다리 높이 [L, R] (관절값)
    tau_hip: np.ndarray        # 고관절 토크 [L, R] (+ = 펴며 받침)
    w_wheel_joint: np.ndarray  # 바퀴 관절 속도 [L, R] (+y 규약, 엔코더)
    w_wheel_abs: np.ndarray    # 바퀴 절대 회전 [L, R] (엔코더 + 4절 링크 회전, 실기는 기구학으로 계산)
    th_kin: float              # 몸체 좌표의 무게중심 방향 각 atan2(x, z) (명목 질량 + 기구학)
    l_pend: float              # 진자 길이 (명목)
    wx: float                  # 바퀴 중심 x (월드, 발동·기록용)
    wheel_z_min: float         # 두 바퀴 중 낮은 바닥 높이 (기록용)
    yaw: float
    sf: float                  # IMU 비력 크기 [g]
    truth_th: float            # 참 진자각 (est='truth')
    truth_v: float             # 참 바퀴축 속도 (est='truth')


class WBController:
    def __init__(self, P, lqr: lqr_vmc.WheelLQR, m_pend: float, seed: int, edges=()):
        """P: TUNE 값 (dict 처럼 P.key). edges: 자동 점프 모서리 x [m] (없으면 점프 안 함)."""
        self.P, self.lqr, self.m_pend, self.edges = P, lqr, m_pend, list(edges)
        self.rng = np.random.default_rng(seed)
        self.bias = self.rng.uniform(-1, 1, 2) * math.radians(P.imu_tilt_bias_deg)
        self.roll_pi = lqr_vmc.RollPI()
        self.q = collections.deque(maxlen=16)
        self.reset()

    def reset(self):
        self.phase, self.t_phase, self.next_edge, self.h0 = "drive", 0.0, 0, self.P.idle_h
        self.jumps = []
        self.x_err = 0.0
        self.lift = dict(on=False, t_un=0.0, t_ld=0.0)
        self.gov = dict(vf=0.0, i=0.0, ref=0.0)
        self.vf, self.rf = 0.0, 0.0
        self.roll_pi.reset()
        self.q.clear()
        self.soft = False

    # --- 센서 ------------------------------------------------------------------------------------
    def _sense(self, f: Frame):
        P = self.P
        p_ = math.asin(max(-1.0, min(1.0, float(f.g_b[0]))))
        r_ = math.asin(max(-1.0, min(1.0, float(f.g_b[1]))))
        if P.est == "sensors":
            n = self.rng.normal
            tn, gn = math.radians(P.imu_tilt_noise_deg), P.imu_gyro_noise
            return dict(pitch=p_ + self.bias[0] + n(0, tn), roll=r_ + self.bias[1] + n(0, tn),
                        gx=float(f.w_b[0]) + n(0, gn), gy=float(f.w_b[1]) + n(0, gn), gz=float(f.w_b[2]) + n(0, gn))
        return dict(pitch=p_, roll=r_, gx=float(f.w_b[0]), gy=float(f.w_b[1]), gz=float(f.w_b[2]))

    def _state(self, f: Frame, S):
        """(진자각, 각속도, 진자 길이, 바퀴축 속도, yaw rate)."""
        P = self.P
        if P.est != "sensors":
            return f.truth_th, S["gy"], f.l_pend, f.truth_v, S["gz"]
        th = S["pitch"] + f.th_kin
        wabs = [w + self.rng.normal(0, P.enc_vel_noise) for w in f.w_wheel_abs] if P.enc_vel_noise > 0 else list(f.w_wheel_abs)
        ws = [w for w, t_ in zip(wabs, f.tau_hip) if t_ >= P.contact_tau_min]      # 땅 짚은 바퀴만
        if ws:
            v_raw = R_WHEEL * sum(ws) / len(ws)
            a = 1.0 - math.exp(-2 * math.pi * P.v_lpf_hz * DT) if P.v_lpf_hz > 0 else 1.0
            self.vf += a * (v_raw - self.vf)
        return th, S["gy"], f.l_pend, self.vf, S["gz"]

    # --- 한 스텝 ----------------------------------------------------------------------------------
    def step(self, f: Frame, vx: float, wz: float, h_ref: float):
        """-> (act[4], leg_kp, leg_kd, ff_force[N] (다리 하나당 자중 보상 힘)). h_ref: 명령의 높이 기준 (auto = idle_h)."""
        P, t = self.P, f.t
        act = np.zeros(4)
        S = self._sense(f)
        th, thd, l_p, v_now, wz_now = self._state(f, S)
        a_r = 1.0 - math.exp(-2 * math.pi * P.roll_rate_lpf_hz * DT) if P.roll_rate_lpf_hz > 0 else 1.0
        self.rf += a_r * (-S["gx"] - self.rf)
        leg_kp, leg_kd = P.vmc_kp, P.vmc_kd

        # 점프 상태머신 (모서리 자동 발동)
        tp = t - self.t_phase
        LEG_T = {"retract": H_MIN, "extract": H_MAX, "fly": H_MIN, "descend": P.h_land, "land": P.h_land}
        if self.edges:
            if self.phase == "drive" and not self.lift["on"] and self.next_edge < len(self.edges) \
                    and f.wx >= self.edges[self.next_edge] - P.trigger:
                self.phase, self.t_phase, self.h0 = "retract", t, float(np.mean(f.h))
                self.jumps.append(dict(edge=self.next_edge, t_trigger=t, x_trigger=f.wx))
            elif self.phase == "retract" and tp >= P.t_retract:
                self.phase, self.t_phase = "extract", t
            elif self.phase == "extract" and (float(np.min(f.h)) >= H_MAX - 0.008 or tp >= 0.25):
                self.phase, self.t_phase = "fly", t
                self.jumps[-1].update(t_takeoff=t, x_takeoff=f.wx)
            elif self.phase == "fly" and tp >= P.t_tuck:
                self.phase, self.t_phase = "descend", t
                self.soft = True
            elif self.phase == "descend" and ((tp > 0.04 and float(np.max(np.abs(f.tau_hip))) > P.contact_tau)
                                              or t - self.jumps[-1]["t_takeoff"] >= P.t_fly_max):
                self.jumps[-1].update(t_land=t, x_land=f.wx, wheel_bottom=f.wheel_z_min)
                self.phase, self.t_phase = "land", t
            elif self.phase == "land" and tp >= P.land_s:
                self.soft = False
                self.phase, self.t_phase = "drive", t
                self.next_edge += 1
            if self.next_edge >= len(self.edges):
                vx = 0.0

        ffF = 0.0
        if self.phase in ("drive", "retract", "extract", "land"):
            th_ref = math.radians(P.retract_lean) if self.phase == "retract" else 0.0
            vm = P.vmax_kmh / 3.6
            g = self.gov
            g["vf"] += (1.0 - math.exp(-2 * math.pi * P.speed_lpf_hz * DT)) * (v_now - g["vf"])
            e = abs(g["vf"]) - vm
            g["i"] = max(0.0, min(vm, g["i"] + P.brake_ki * e * DT))
            v_lim = max(0.0, vm - (P.brake_kp * max(e, 0.0) + g["i"]))
            tgt = max(-v_lim, min(v_lim, vx))
            g["ref"] += max(-P.accel_max * DT, min(P.accel_max * DT, tgt - g["ref"]))
            v_ref = g["ref"]
            if abs(vx) > v_lim + 1e-3:
                self.x_err = 0.0
            ww = float(np.max(np.abs(f.w_wheel_joint))) / W_WHEEL_MAX
            if ww > P.speed_guard and P.speed_guard < 1.0:
                cut = min(1.0, (ww - P.speed_guard) / (1.0 - P.speed_guard))
                v_ref = v_now * (1.0 - 0.6 * cut) if v_now * vx >= 0 else vx
                self.x_err = 0.0
            self.x_err = max(-0.3, min(0.3, self.x_err + (v_now - v_ref) * DT))
            tau_w = self.lqr.torque(l_p, self.x_err, v_now - v_ref, th - th_ref, thd)
            wz_lim = max(0.5, (P.wheel_margin * W_WHEEL_MAX * R_WHEEL - abs(v_now)) / HALF_TRACK)
            wz = max(-wz_lim, min(wz_lim, wz))
            tau_y = P.yaw_kd * (wz - wz_now)
            act[2] = max(-1.0, min(1.0, (0.5 * tau_w - tau_y) / P.wheel_tau_max))
            act[3] = max(-1.0, min(1.0, (0.5 * tau_w + tau_y) / P.wheel_tau_max))

        # 들림 / 착지 감지 (부호 있는 고관절 토크 + IMU 비력)
        unl = float(np.max(f.tau_hip)) < P.contact_tau_min
        ldd = float(np.max(f.tau_hip)) >= P.contact_tau_min and f.sf > P.land_sf_min
        L = self.lift
        if self.phase == "drive" and not L["on"]:
            L["t_un"] = L["t_un"] + DT if unl else 0.0
            if L["t_un"] >= P.lift_detect_s:
                L.update(on=True, t_ld=0.0)
        elif self.phase == "drive" and L["on"]:
            L["t_ld"] = L["t_ld"] + DT if ldd else 0.0
            if L["t_ld"] >= P.land_detect_s:
                L.update(on=False, t_un=0.0)
                self.x_err = 0.0; self.gov.update(i=0.0, ref=v_now, vf=v_now); self.roll_pi.reset(0.0)

        if self.phase == "drive" and L["on"]:                   # 들린 동안: 균형 끔, 바퀴 감쇠, 다리 IDLE
            act[2:] = np.clip(-P.lift_wheel_kd * np.asarray(f.w_wheel_joint) / P.wheel_tau_max, -1.0, 1.0)
            act[0] = act[1] = (P.idle_h - h_ref) / 0.12
            self.roll_pi.reset(0.0); self.x_err = 0.0; self.gov.update(i=0.0, ref=0.0, vf=0.0)
        elif self.phase == "drive":
            rl = S["roll"]
            airborne = float(np.min(f.tau_hip)) < P.contact_tau_min
            dlt = self.roll_pi(rl, DT, P.roll_kp, P.roll_ki, P.level_max,
                               freeze=airborne or abs(math.degrees(rl)) > P.roll_freeze_deg, leak=P.roll_leak,
                               rate=self.rf, kd=P.roll_kd)
            tl = min(H_MAX, max(H_MIN, P.idle_h + 0.5 * dlt)); tr = min(H_MAX, max(H_MIN, P.idle_h - 0.5 * dlt))
            act[0], act[1] = (tl - h_ref) / 0.12, (tr - h_ref) / 0.12
            ffF = 0.5 * self.m_pend * 9.81
        elif self.phase == "land":
            ffF = 0.5 * self.m_pend * 9.81
        else:
            self.roll_pi.reset(float(f.h[0] - f.h[1]))

        # 점프 단계의 다리 목표와 공중 바퀴 pitch PD
        if self.phase in LEG_T:
            tgt = LEG_T[self.phase]
            if self.phase == "retract":
                tgt = self.h0 + (H_MIN - self.h0) * min(1.0, tp / (0.75 * P.t_retract))
            act[0] = act[1] = (tgt - h_ref) / 0.12
            pd = ("fly", "descend") if P.pd_from == "fly" or P.extract_wheels != "pd" else ("extract", "fly", "descend")
            if self.phase in pd and P.air_ctrl:
                ref = P.extract_pitch if self.phase == "extract" else P.air_pitch
                u = P.air_kp * (S["pitch"] - math.radians(ref)) + P.air_kd * S["gy"]
                act[2] = act[3] = max(-1.0, min(1.0, u / P.air_tau))
            elif self.phase in pd:
                act[2:] = 0.0
            if self.soft:
                leg_kp, leg_kd = P.land_kp, P.land_kd
            elif self.phase != "land":
                leg_kp, leg_kd = P.leg_kp, P.leg_kd

        # 제어 지연 (+ 가끔 한 주기 더)
        if P.delay_ms > 0 or P.jitter_ms > 0:
            self.q.append(act.copy())
            dly = int(round(P.delay_ms / 5.0)) + (1 if self.rng.random() < P.jitter_ms / 5.0 else 0)
            act = self.q[max(0, len(self.q) - 1 - dly)]
        return act, leg_kp, leg_kd, ffF, dict(th=th, v=v_now, pitch=S["pitch"], roll=S["roll"])
