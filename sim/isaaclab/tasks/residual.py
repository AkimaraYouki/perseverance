"""잔차 RL: LQR(바퀴) + VMC(다리) 기본 제어기 위에 정책이 작은 보정만 더한다 (2026-09-26 사용자 결정).

기본 제어기 = scripts/wbctrl.py 의 주행 부분 (climb_test ctrl="lqr") 을 torch 로 벡터화한 것:
  상태 추정 (IMU 바이어스·잡음 + 기구학 + 땅 짚은 바퀴 속도), 균형점 보정(멈췄을 때), 모터 한계 따라 최고 속도,
  PI 브레이크 + 목표 변화율 제한, LQR (진자 길이별 게인 보간), 회전 P + 회전 한계, roll PI (멈춤·누설·필터 D),
  들림/착지 감지, 제어 지연 + 바퀴 토크 20 Hz LPF. 점프는 없다 (주행·험지만).
정책 행동 4 = [바퀴 토크 보정 L, R (x res_wheel_nm), 다리 높이 보정 L, R (x res_leg_m)], 각 ±1.
보정이 0 이면 기본 제어기 그대로 = 학습 시작점이 이미 대부분 서 있다.
"""
from __future__ import annotations

import math
from dataclasses import MISSING
from typing import TYPE_CHECKING

import numpy as np
import torch

from isaaclab.assets import Articulation
from isaaclab.managers import ActionTerm, ActionTermCfg, SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply_inverse

from . import cad

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

H_MIN, H_MAX = 0.1225, 0.2425
W_MAX0 = 18.85
HALF_TRACK = 0.094
TRACK = 0.198


def _lqr_table(m, I, m_w, I_w, R, q, r, l_grid):
    import sys, os
    sys.path.insert(0, os.path.expanduser("~/wheeled_biped_isaaclab/scripts"))
    import lqr_vmc
    ctl = lqr_vmc.WheelLQR(m, I, m_w, I_w, R, q=q, r=r, l_grid=l_grid)
    return ctl.K


class ResidualCtrlAction(ActionTerm):
    cfg: "ResidualCtrlActionCfg"
    _asset: Articulation

    def __init__(self, cfg: "ResidualCtrlActionCfg", env: "ManagerBasedRLEnv"):
        super().__init__(cfg, env)
        r = self._asset
        N, dev = self.num_envs, self.device
        self.leg_ids = r.find_joints(cad.LEG_JOINTS, preserve_order=True)[0]
        self.wheel_ids = r.find_joints(cad.WHEEL_JOINTS, preserve_order=True)[0]
        self.wheel_bodies = r.find_bodies(["l_wheel", "r_wheel"], preserve_order=True)[0]
        self.nonwheel = [i for i in range(r.num_bodies) if i not in self.wheel_bodies]
        self.hip_sign = torch.tensor([cad.M_SIGN["L"], cad.M_SIGN["R"]], device=dev)
        self.wsign = torch.tensor(cad.WHEEL_SIGN, device=dev)
        self.dt = env.step_dt
        # 명목 모델 (시작 DR 전에 읽는다 — 액션 항은 startup 이벤트보다 먼저 만들어진다)
        m = r.root_physx_view.get_masses()[0].clone().to(dev)
        self.m_nom = m
        self.m_pend = float(m[self.nonwheel].sum())
        d = r.data
        c0 = (d.body_com_pos_w[0, self.nonwheel] * m[self.nonwheel, None]).sum(0) / self.m_pend
        iyy = r.root_physx_view.get_inertias()[0][:, 4].to(dev)
        rel = d.body_com_pos_w[0, self.nonwheel] - c0
        I = float((iyy[self.nonwheel] + m[self.nonwheel] * (rel[:, 0] ** 2 + rel[:, 2] ** 2)).sum())
        self.l_grid = torch.linspace(0.12, 0.40, 15, device=dev)
        K = _lqr_table(self.m_pend, I, float(m[self.wheel_bodies].sum()), 2 * cad.WHEEL_IZZ, cad.R_WHEEL,
                       (cfg.lqr_qx, cfg.lqr_qv, cfg.lqr_qth, cfg.lqr_qthd), cfg.lqr_r, self.l_grid.cpu().numpy())
        self.K = torch.tensor(np.asarray(K), device=dev, dtype=torch.float32)          # (15, 4)
        z = lambda: torch.zeros(N, device=dev)  # noqa: E731
        self.x_err, self.vf, self.rf, self.v_prev, self.th_bias = z(), z(), z(), z(), z()
        self.gov_vf, self.gov_i, self.gov_ref, self.roll_i = z(), z(), z(), z()
        self.t_un, self.t_ld = z(), z()
        self.t_bump = torch.zeros(N, device=dev)                   # 턱 감속 남은 시간 [s]
        self.lift = torch.zeros(N, dtype=torch.bool, device=dev)
        self.bias = torch.zeros(N, 2, device=dev)
        self.motor_true = torch.ones(N, device=dev)
        self.motor_est = torch.ones(N, device=dev)
        # 명령 전달 (CAN 흉내): 스텝마다 명령을 큐에 넣고, 모터마다 도착 시각을 따로 정한다 (기본 지연 + 모터별 편차 +
        #   지터 + 프레임 손실 + 버스 정지). 물리 주기(2.5 ms)마다 모터별로 '도착한 것 중 가장 최근에 보낸 명령' 을 적용,
        #   아무것도 안 왔으면 직전 명령을 쥐고 있는다. 시각은 float64 (학습이 몇 시간 돌아도 2.5 ms 해상도 유지)
        self.Q = 16
        self.queue = torch.zeros(N, self.Q, 5, device=dev)
        self.arrive = torch.full((N, self.Q, 4), -1e9, device=dev, dtype=torch.float64)   # 모터 [바퀴L, 바퀴R, 고관절L, 고관절R]
        self.send_t = torch.full((self.Q,), -1e9, device=dev, dtype=torch.float64)
        self.head = 0
        self.t_ctrl, self.sub = 0.0, 0
        self.dt_phys = env.physics_dt
        self.d_base = torch.zeros(N, 4, device=dev, dtype=torch.float64)                 # 모터별 기본 지연 [s]
        self.bus_down = torch.zeros(N, dtype=torch.bool, device=dev)                      # 버스 정지 (버스트 손실) 중
        # 센서 지연: IMU (pitch, roll, 자이로 3, 비력) 와 모터 피드백 (바퀴 절대·관절 속도, 고관절 토크) 을 따로 늦춘다
        self.SR = 12                                                                     # 5 ms x 12 = 60 ms 기록
        self.sring = torch.zeros(N, self.SR, 12, device=dev)
        self.s_head = 0
        self.s_fresh = torch.ones(N, dtype=torch.bool, device=dev)
        self.d_imu = torch.zeros(N, device=dev)
        self.d_fb = torch.zeros(N, device=dev)
        self.ph_imu = torch.zeros(N, device=dev)                                          # 센서 샘플 위상 [s] (주기 샘플링)
        self.ph_fb = torch.zeros(N, device=dev)
        self.tau_f = torch.zeros(N, 2, device=dev)                # 바퀴 토크 LPF 상태
        self._raw = torch.zeros(N, 4, device=dev)
        self._prev_raw = torch.zeros(N, 4, device=dev)
        self.out = torch.zeros(N, 6, device=dev)                   # 모터에 걸린 명령 [τL, τR, hL, hR, ffL, ffR]
        self.base = torch.zeros(N, 4, device=dev)                  # 기본 제어기 출력 (관측용) [τL, τR, hL, hR]
        self.est = torch.zeros(N, 8, device=dev)                   # 관측용 내부값
        vl = r.actuators["wheels"].velocity_limit
        self.vlim0 = (vl.clone() if torch.is_tensor(vl) else torch.full((N, 2), float(vl), device=dev))
        self.reset(None)

    @property
    def action_dim(self) -> int:
        return 4

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw

    @property
    def processed_actions(self) -> torch.Tensor:
        return self.out

    # ------------------------------------------------------------------------------------------------
    def reset(self, env_ids=None):
        if env_ids is None:
            env_ids = slice(None)
        c = self.cfg
        for t_ in (self.x_err, self.vf, self.rf, self.v_prev, self.th_bias, self.gov_vf, self.gov_i, self.gov_ref,
                   self.roll_i, self.t_un, self.t_ld):
            t_[env_ids] = 0.0
        self.lift[env_ids] = False
        self.t_bump[env_ids] = 0.0
        self.tau_f[env_ids] = 0.0
        ff0 = 0.5 * self.m_pend * 9.81                              # 첫 명령이 도착하기 전: 바퀴 0, 다리 IDLE, 자중 보상
        self.queue[env_ids] = torch.tensor([0.0, 0.0, c.idle_h, c.idle_h, ff0], device=self.device)
        self.out[env_ids] = torch.tensor([0.0, 0.0, c.idle_h, c.idle_h, ff0, ff0], device=self.device)
        self.arrive[env_ids] = -1e9
        self.bus_down[env_ids] = False
        self.s_fresh[env_ids] = True
        self._raw[env_ids] = 0.0
        self._prev_raw[env_ids] = 0.0
        n = self.bias[env_ids].shape[0]
        dev = self.device
        # 실기 조건 무작위 (에피소드마다): IMU 바이어스, 바퀴 모터 한계(배터리), 제어 지연
        self.bias[env_ids] = (torch.rand(n, 2, device=dev) * 2 - 1) * math.radians(c.imu_tilt_bias_deg)
        kv = 1.0 - torch.rand(n, device=dev) * c.dr_motor
        self.motor_true[env_ids] = kv
        self.motor_est[env_ids] = kv * (1.0 + 0.02 * torch.randn(n, device=dev))
        wa = self._asset.actuators["wheels"]
        if torch.is_tensor(wa.velocity_limit):
            wa.velocity_limit[env_ids] = self.vlim0[env_ids] * kv[:, None]
            if hasattr(wa, "_vel_at_effort_lim") and torch.is_tensor(wa._vel_at_effort_lim):
                wa._vel_at_effort_lim[env_ids] = wa.velocity_limit[env_ids] * (1 + wa.effort_limit[env_ids] / wa._saturation_effort)
        self.d_base[env_ids] = ((c.delay_ms + (torch.rand(n, 4, device=dev) * 2 - 1) * c.delay_asym_ms) * 1e-3).clamp(min=0.0).double()
        self.d_imu[env_ids] = torch.rand(n, device=dev) * c.imu_delay_ms * 1e-3
        self.d_fb[env_ids] = torch.rand(n, device=dev) * c.fb_delay_ms * 1e-3
        self.ph_imu[env_ids] = torch.rand(n, device=dev) * c.imu_period_ms * 1e-3
        self.ph_fb[env_ids] = torch.rand(n, device=dev) * c.fb_period_ms * 1e-3

    @staticmethod
    def _age(t_now, lat_ms, d_rand, period_ms, phase):
        base = lat_ms * 1e-3 + d_rand
        if period_ms <= 0:
            return base
        P = period_ms * 1e-3
        return base + torch.remainder(t_now - base - phase, P)

    def _delayed(self, d_s, c0, c1):
        """센서 기록에서 로봇마다 d_s [s] 전 값 (스텝 사이 선형 보간)."""
        st = d_s / self.dt
        i0 = st.floor().long().clamp(max=self.SR - 2)
        f = (st - i0.float()).clamp(0.0, 1.0)[:, None]
        n_ = torch.arange(self.num_envs, device=self.device)
        a = self.sring[n_, (self.s_head - i0) % self.SR, c0:c1]
        b = self.sring[n_, (self.s_head - i0 - 1) % self.SR, c0:c1]
        return a * (1 - f) + b * f

    # ------------------------------------------------------------------------------------------------
    def _interp_K(self, l):
        g = self.l_grid
        l = l.clamp(g[0], g[-1])
        i = torch.clamp(torch.searchsorted(g, l.contiguous()), 1, len(g) - 1)
        w = ((l - g[i - 1]) / (g[i] - g[i - 1]))[:, None]
        return self.K[i - 1] * (1 - w) + self.K[i] * w

    def process_actions(self, actions: torch.Tensor):
        c, r, dt = self.cfg, self._asset, self.dt
        self._prev_raw[:] = self._raw
        self._raw[:] = actions
        res = actions.clamp(-1.0, 1.0)
        d = r.data
        dev = self.device
        cmd = self._env.command_manager.get_command(c.command_name)
        vx_c, wz_c = cmd[:, 0], cmd[:, 1]
        # --- 센서 (실기: IMU 자세·자이로, 관절·바퀴 엔코더, 고관절 전류) ---
        g = d.projected_gravity_b
        q = d.root_quat_w
        psi = torch.atan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))
        lat = torch.stack([-torch.sin(psi), torch.cos(psi), torch.zeros_like(psi)], 1)
        raw = torch.cat([torch.asin(g[:, 0:2].clamp(-1, 1)), d.root_ang_vel_b,
                         ((d.body_lin_acc_w[:, 0] + torch.tensor([0.0, 0.0, 9.81], device=dev)).norm(dim=1) / 9.81)[:, None],
                         (d.body_ang_vel_w[:, self.wheel_bodies] * lat[:, None]).sum(-1),
                         d.joint_vel[:, self.wheel_ids] * self.wsign,
                         d.applied_torque[:, self.leg_ids] * self.hip_sign], 1)             # (N, 12) 참값
        self.s_head = (self.s_head + 1) % self.SR
        self.sring[:, self.s_head] = raw
        if bool(self.s_fresh.any()):                                         # 에피소드 첫 스텝: 과거를 지금 값으로 채움
            self.sring[self.s_fresh] = raw[self.s_fresh][:, None, :].expand(-1, self.SR, -1)
            self.s_fresh[:] = False
        # 센서 나이 = 고정 지연 + 로봇별 추가 지연 + (주기 샘플링이면) 마지막 샘플 뒤 지난 시간. 샘플 사이엔 같은 값을 쥔다
        t_now = self.t_ctrl + dt
        imu = self._delayed(self._age(t_now, c.imu_latency_ms, self.d_imu, c.imu_period_ms, self.ph_imu), 0, 6)
        fb = self._delayed(self._age(t_now, c.fb_latency_ms, self.d_fb, c.fb_period_ms, self.ph_fb), 6, 12)
        tn = math.radians(c.imu_tilt_noise_deg)
        pitch = imu[:, 0] + self.bias[:, 0] + tn * torch.randn_like(imu[:, 0])
        roll = imu[:, 1] + self.bias[:, 1] + tn * torch.randn_like(imu[:, 0])
        gyro = imu[:, 2:5] + c.imu_gyro_noise * torch.randn_like(imu[:, 2:5])
        sf = imu[:, 5]
        wabs = fb[:, 0:2] + c.enc_vel_noise * torch.randn(self.num_envs, 2, device=dev)
        wj = fb[:, 2:4]
        tau_hip = fb[:, 4:6]                                                 # + = 다리를 펴며 몸을 받침
        # --- 상태 추정 ---
        ax = d.body_pos_w[:, self.wheel_bodies].mean(1)
        c_nom = (d.body_com_pos_w[:, self.nonwheel] * self.m_nom[None, self.nonwheel, None]).sum(1) / self.m_pend
        rb = quat_apply_inverse(q, c_nom - ax)
        th = pitch + torch.atan2(rb[:, 0], rb[:, 2])
        l_p = rb.norm(dim=1)
        thd = gyro[:, 1]
        loaded = tau_hip >= c.contact_tau_min
        nl = loaded.sum(1)
        v_raw = cad.R_WHEEL * (wabs * loaded).sum(1) / nl.clamp(min=1)
        a_v = 1.0 - math.exp(-2 * math.pi * c.v_lpf_hz * dt)
        self.vf = torch.where(nl > 0, self.vf + a_v * (v_raw - self.vf), self.vf)
        v = self.vf
        acc = (v - self.v_prev) / dt
        self.v_prev = v.clone()
        ad = (~self.lift) & (acc.abs() < 0.3) & (thd.abs() < 0.3) & (v.abs() < 0.05) & (vx_c.abs() < 0.02)
        lim = math.radians(c.bal_adapt_max_deg)
        self.th_bias = torch.where(ad, (self.th_bias + c.bal_adapt * dt * (th - self.th_bias)).clamp(-lim, lim), self.th_bias)
        th = th - self.th_bias
        # --- 속도 제한 + 브레이크 ---
        w_max = W_MAX0 * self.motor_est
        vm = torch.clamp(w_max * cad.R_WHEEL * c.vmax_motor_frac, max=c.vmax_kmh / 3.6)
        # 턱 감지 -> 감속 (bump_slow): 바퀴가 턱에 걸리면 pitch 가 튀고 몸이 급감속한다. 0.8 m/s 에선 바퀴가 이미 모터
        #   무부하 속도의 71~83 % 라 턱에서 LQR 이 달라는 토크를 모터가 못 낸다 (DC 모터: 속도가 오를수록 토크 한계가 준다).
        #   연달아 나오는 턱 (삼각형길) 은 첫 턱이 예고 -> bump_hold_s 동안 최고 속도를 bump_vmax 로.
        if c.bump_slow:
            a_f = (d.body_lin_acc_w[:, 0, :2] * torch.stack([torch.cos(psi), torch.sin(psi)], 1)).sum(1) \
                + c.imu_acc_noise * torch.randn_like(psi)
            hit = (thd.abs() > c.bump_rate) | (a_f < -c.bump_acc)
            self.t_bump = torch.where(hit, torch.full_like(self.t_bump, c.bump_hold_s), (self.t_bump - dt).clamp(min=0.0))
        #   (과속 브레이크 vm 은 그대로 두고 명령 목표만 낮춘다 — vm 을 낮추면 PI 브레이크가 '과속' 으로 보고 목표를 0.1 m/s 까지 급히 끌어내림)
        vb = torch.where(self.t_bump > 0, torch.full_like(vm, c.bump_vmax), torch.full_like(vm, 1e3))
        self.gov_vf += (1.0 - math.exp(-2 * math.pi * c.speed_lpf_hz * dt)) * (v - self.gov_vf)
        e = self.gov_vf.abs() - vm
        self.gov_i = torch.minimum(torch.clamp(self.gov_i + c.brake_ki * e * dt, min=0.0), vm)
        v_lim = torch.clamp(vm - (c.brake_kp * e.clamp(min=0.0) + self.gov_i), min=0.0)
        v_lim = torch.minimum(v_lim, vb)
        tgt = torch.maximum(torch.minimum(vx_c, v_lim), -v_lim)
        self.gov_ref += (tgt - self.gov_ref).clamp(-c.accel_max * dt, c.accel_max * dt)
        v_ref = self.gov_ref
        self.x_err = torch.where(vx_c.abs() > v_lim + 1e-3, torch.zeros_like(self.x_err), self.x_err)
        # 바퀴 속도 푸시백: 모터 한계의 speed_guard 를 넘으면 목표를 지금 속도보다 낮춰 뒤로 젖히며 감속 (climb_test 와 같음)
        ww = wj.abs().max(1).values / w_max
        push = (ww > c.speed_guard) & (c.speed_guard < 1.0)
        cut = ((ww - c.speed_guard) / max(1e-6, 1.0 - c.speed_guard)).clamp(0.0, 1.0)
        v_ref = torch.where(push, torch.where(v * vx_c >= 0, v * (1.0 - 0.6 * cut), vx_c), v_ref)
        self.x_err = torch.where(push, torch.zeros_like(self.x_err), self.x_err)
        self.x_err = (self.x_err + (v - v_ref) * dt).clamp(-0.3, 0.3)
        # --- LQR + 회전 ---
        K = self._interp_K(l_p)
        tau_w = -(K[:, 0] * self.x_err + K[:, 1] * (v - v_ref) + K[:, 2] * th + K[:, 3] * thd)
        if c.turn_limit:                   # 달릴 때 회전 한계 (창은 끔: 사람이 조심)
            wz_lim = torch.clamp((c.wheel_margin * w_max * cad.R_WHEEL - v.abs()) / HALF_TRACK, min=0.5)
            wz = torch.maximum(torch.minimum(wz_c, wz_lim), -wz_lim)
        else:
            wz = wz_c
        tau_y = c.yaw_kd * (wz - gyro[:, 2])
        tau = torch.stack([0.5 * tau_w - tau_y, 0.5 * tau_w + tau_y], 1)
        # --- 들림 / 착지 ---
        unl = tau_hip.max(1).values < c.contact_tau_min
        ldd = (tau_hip.max(1).values >= c.contact_tau_min) & (sf > c.land_sf_min)
        self.t_un = torch.where(~self.lift & unl, self.t_un + dt, torch.zeros_like(self.t_un))
        up = ~self.lift & (self.t_un >= c.lift_detect_s)
        self.t_ld = torch.where(self.lift & ldd, self.t_ld + dt, torch.zeros_like(self.t_ld))
        down = self.lift & (self.t_ld >= c.land_detect_s)
        self.lift = (self.lift | up) & ~down
        for t_ in (self.x_err, self.gov_i, self.roll_i):
            t_[down] = 0.0
        self.gov_ref[down] = v[down]; self.gov_vf[down] = v[down]
        # --- roll PI (좌우 다리 길이 차) ---
        a_r = 1.0 - math.exp(-2 * math.pi * c.roll_rate_lpf_hz * dt)
        self.rf += a_r * (-gyro[:, 0] - self.rf)
        # 회전 중 안쪽으로 기울이기 (Ascento lean 모드): 원심력 v*wz 를 받치려면 roll 목표 = atan(v wz / g)
        #   (로그 roll + = 왼쪽이 낮음, 왼쪽으로 돌면 wz > 0 -> 왼쪽을 낮춤). 수평으로 붙잡으면 바깥으로 넘어감
        roll_ref = torch.atan(c.turn_lean * v_ref * wz / 9.81).clamp(-math.radians(20), math.radians(20))   # 명령값으로 (측정은 흔들림)
        er = TRACK * torch.sin(roll - roll_ref)
        freeze = (tau_hip.min(1).values < c.contact_tau_min) | (roll.abs() > math.radians(c.roll_freeze_deg))
        self.roll_i = torch.where(freeze, self.roll_i, self.roll_i + c.roll_ki * er * dt)
        self.roll_i = (self.roll_i * (1.0 - c.roll_leak * dt)).clamp(-c.level_max, c.level_max)
        dlt = (self.roll_i + c.roll_kp * er + c.roll_kd * TRACK * self.rf).clamp(-c.level_max, c.level_max)
        h = torch.stack([c.idle_h + 0.5 * dlt, c.idle_h - 0.5 * dlt], 1).clamp(H_MIN, H_MAX)
        ff = torch.full((self.num_envs,), 0.5 * self.m_pend * 9.81, device=dev)
        # 들린 동안: 균형 끔, 바퀴 감쇠, 다리 IDLE, 적분 비움
        L = self.lift
        tau = torch.where(L[:, None], -c.lift_wheel_kd * wj, tau)
        h = torch.where(L[:, None], torch.full_like(h, c.idle_h), h)
        ff = torch.where(L, torch.zeros_like(ff), ff)
        for t_ in (self.roll_i, self.x_err, self.gov_i, self.gov_ref, self.gov_vf):
            t_[L] = 0.0
        tau = tau.clamp(-c.wheel_tau_max, c.wheel_tau_max)
        self.base[:] = torch.cat([tau, h], 1)
        # --- 정책 보정 (들린 동안은 끔) ---
        on = (~L).float()[:, None]
        tau = (tau + on * res[:, :2] * c.res_wheel_nm).clamp(-c.wheel_tau_max, c.wheel_tau_max)
        h = (h + on * res[:, 2:] * c.res_leg_m).clamp(H_MIN, H_MAX)
        # --- 명령 전달 (위 __init__ 설명) ---
        N = self.num_envs
        self.t_ctrl += dt
        self.head = (self.head + 1) % self.Q
        self.queue[:, self.head] = torch.cat([tau, h, ff[:, None]], 1)
        self.send_t[self.head] = self.t_ctrl
        jit = (torch.rand(N, 4, device=dev) < c.delay_extra_prob).double() * (c.jitter_ms * 1e-3)
        rec = torch.rand(N, device=dev) < 1.0 / max(1.0, c.loss_burst_steps)               # 버스 정지는 평균 loss_burst_steps 스텝
        self.bus_down = torch.where(self.bus_down, ~rec, torch.rand(N, device=dev) < c.loss_burst_prob)
        drop = (torch.rand(N, 4, device=dev) < c.loss_frame_prob) | self.bus_down[:, None]
        arr = self.t_ctrl + self.d_base + jit
        self.arrive[:, self.head] = torch.where(drop, torch.full_like(arr, float("inf")), arr)
        self.sub = 0
        # 관측용 내부값
        self.est[:] = torch.stack([th, v, v_ref, self.x_err, dlt, L.float(), self.motor_est, self.th_bias], 1)

    def apply_actions(self):
        r, c = self._asset, self.cfg
        # 이 물리 스텝 시각까지 도착한 명령 중 가장 최근에 보낸 것 (모터별). 없으면 쥐고 있던 명령 그대로
        now = self.t_ctrl + self.sub * self.dt_phys + 1e-7
        self.sub += 1
        ok = self.arrive <= now                                                          # (N, Q, 4)
        best = torch.where(ok, self.send_t[None, :, None], torch.full_like(self.arrive, -1e18)).argmax(1)   # (N, 4)
        n_ = torch.arange(self.num_envs, device=self.device)
        qq = self.queue
        new = torch.stack([qq[n_, best[:, 0], 0], qq[n_, best[:, 1], 1], qq[n_, best[:, 2], 2], qq[n_, best[:, 3], 3],
                           qq[n_, best[:, 2], 4], qq[n_, best[:, 3], 4]], 1)
        has = ok.any(1)[:, [0, 1, 2, 3, 2, 3]]
        self.out[:] = torch.where(has, new, self.out)
        a = 1.0 - math.exp(-2 * math.pi * c.wheel_lpf_hz * self._env.physics_dt)
        self.tau_f += a * (self.out[:, :2] - self.tau_f)
        M = cad.M_from_hj(self.out[:, 2:4])
        r.set_joint_position_target(M, joint_ids=self.leg_ids)
        Mc = r.data.joint_pos[:, self.leg_ids]
        leg_ff = self.hip_sign * self.out[:, 4:6] * cad.dh_from_M(Mc).to(torch.float32)
        r.set_joint_effort_target(leg_ff, joint_ids=self.leg_ids)
        r.set_joint_effort_target(self.tau_f * self.wsign, joint_ids=self.wheel_ids)

    # 관측 벡터 (정책·크리틱 공통 부분)
    def obs(self) -> torch.Tensor:
        c = self.cfg
        d = self._asset.data
        hj = cad.leg_state(self._asset)[0]
        return torch.cat([
            self.est[:, 0:1] * 5.0, self.est[:, 1:2], self.est[:, 2:3], self.est[:, 3:4] * 5.0,   # 진자각, 속도, 목표 속도, 거리 오차
            self.est[:, 4:5] * 10.0, self.est[:, 5:6], self.est[:, 6:7] - 1.0, self.est[:, 7:8] * 10.0,
            self.base[:, :2] / c.wheel_tau_max, (self.base[:, 2:] - c.idle_h) * 10.0,           # 기본 제어기 출력
            (hj - c.idle_h) * 10.0,
            (d.applied_torque[:, self.leg_ids] * self.hip_sign) / 5.0,
            self._raw.clamp(-1, 1),
        ], 1)


@configclass
class ResidualCtrlActionCfg(ActionTermCfg):
    class_type: type = ResidualCtrlAction
    asset_name: str = "robot"
    command_name: str = "base_velocity"
    res_wheel_nm: float = 1.5          # 바퀴 토크 보정 한계 (바퀴 하나)
    res_leg_m: float = 0.02            # 다리 높이 보정 한계
    # --- 기본 제어기 (climb_test TUNE 과 같은 값) ---
    lqr_qx: float = 2.0; lqr_qv: float = 5.0; lqr_qth: float = 100.0; lqr_qthd: float = 5.0; lqr_r: float = 1.0  # noqa: E702
    wheel_tau_max: float = 7.0
    yaw_kd: float = 0.5
    wheel_margin: float = 0.7
    turn_limit: bool = False
    speed_guard: float = 0.8
    vmax_kmh: float = 3.0
    vmax_motor_frac: float = 0.75
    brake_kp: float = 1.0; brake_ki: float = 5.5; speed_lpf_hz: float = 4.0; accel_max: float = 1.5  # noqa: E702  (창 TUNE 과 같게, 2026-09-27)
    bal_adapt: float = 0.3; bal_adapt_max_deg: float = 8.0  # noqa: E702
    idle_h: float = 0.1825
    roll_kp: float = 1.5; roll_ki: float = 15.0; roll_kd: float = 0.3; roll_rate_lpf_hz: float = 8.0  # noqa: E702
    roll_leak: float = 0.5; roll_freeze_deg: float = 20.0; level_max: float = 0.10  # noqa: E702
    contact_tau_min: float = 0.8
    bump_slow: bool = False            # 턱 감지 -> 감속 (위 설명). 가혹 평가 0.8 m/s 삼각형길 88 -> 94 %. 기본 끔 (사용자: 험지 안정성 우선)
    bump_rate: float = 1.5             # pitch 각속도 [rad/s] 가 넘거나
    bump_acc: float = 4.0              # 전후 감속 [m/s^2] 이 넘으면 턱
    bump_hold_s: float = 2.0
    bump_vmax: float = 0.45            # 감속 중 최고 속도 [m/s]
    imu_acc_noise: float = 0.2
    turn_lean: float = 1.0             # 회전 중 안쪽 기울기 비율 (1 = 원심력과 중력 합력 방향)
    lift_detect_s: float = 0.3; land_detect_s: float = 0.02; land_sf_min: float = 0.4; lift_wheel_kd: float = 0.05  # noqa: E702
    v_lpf_hz: float = 10.0
    wheel_lpf_hz: float = 20.0
    # --- 실기 조건 무작위 ---
    imu_tilt_noise_deg: float = 0.3
    imu_tilt_bias_deg: float = 0.5
    imu_gyro_noise: float = 0.01
    enc_vel_noise: float = 0.05
    delay_ms: float = 5.0              # 명령 기본 지연 [ms] (센서 -> 모터). 2.5 ms (물리 주기) 해상도
    delay_extra_prob: float = 0.4      # 모터·스텝마다 jitter_ms 만큼 더 늦을 확률 (= climb_test jitter_ms 2 / 5 ms)
    jitter_ms: float = 5.0
    delay_asym_ms: float = 1.0         # 모터별 기본 지연 편차 ± [ms] (에피소드마다)
    loss_frame_prob: float = 0.01      # 모터·스텝마다 명령 프레임이 빠질 확률 (모터는 직전 명령을 쥔다)
    loss_burst_prob: float = 0.002     # 스텝마다 버스가 멈출 확률 (네 모터 모두 명령이 안 감, 약 0.4 번/s)
    loss_burst_steps: float = 4.0      # 버스 정지 평균 길이 [스텝] (4 = 20 ms)
    imu_delay_ms: float = 5.0          # IMU 추가 지연 0 ~ 이 값 [ms] (로봇마다, 스텝 사이는 선형 보간)
    fb_delay_ms: float = 2.5           # 모터 피드백 (엔코더·토크) 추가 지연 0 ~ 이 값 [ms]
    imu_latency_ms: float = 0.0        # IMU 고정 지연 [ms] (iAHRS 내장 LPF 51.2 Hz 약 3 ms + USB 직렬)
    imu_period_ms: float = 0.0         # IMU 출력 주기 [ms] (0 = 매 스텝 새 값). iAHRS sp
    fb_latency_ms: float = 0.0         # 모터 피드백 고정 지연 [ms] (CAN 프레임 + 드라이버)
    fb_period_ms: float = 0.0          # 모터 피드백 업로드 주기 [ms] (0 = 매 스텝). CubeMars 서보 모드 1~500 Hz, 지금 50 Hz [측정]
    dr_motor: float = 0.15


# --- 관측 / 보상 --------------------------------------------------------------------------------------
def residual_obs(env: "ManagerBasedRLEnv", action_name: str = "ctrl") -> torch.Tensor:
    return env.action_manager.get_term(action_name).obs()


def _true_v_th(env, action_name="ctrl"):
    t = env.action_manager.get_term(action_name)
    d = t._asset.data
    q = d.root_quat_w
    psi = torch.atan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))
    fwd = torch.stack([torch.cos(psi), torch.sin(psi), torch.zeros_like(psi)], 1)
    v = (d.body_lin_vel_w[:, t.wheel_bodies].mean(1) * fwd).sum(1)
    return v, t


def privileged_obs(env: "ManagerBasedRLEnv", action_name: str = "ctrl") -> torch.Tensor:
    v, t = _true_v_th(env, action_name)
    d = t._asset.data
    return torch.cat([v[:, None], d.root_lin_vel_b, d.projected_gravity_b, d.root_ang_vel_b,
                      t.motor_true[:, None] - 1.0, t.bias * 10.0, (t.d_base.mean(1, keepdim=True) / t.dt).float()], 1)   # 명령 지연 [스텝]


def track_v_exp(env, std: float = 0.2, action_name: str = "ctrl"):
    v, t = _true_v_th(env, action_name)
    c = t.cfg
    vm = torch.clamp(W_MAX0 * t.motor_est * cad.R_WHEEL * c.vmax_motor_frac, max=c.vmax_kmh / 3.6)
    cmd = env.command_manager.get_command(c.command_name)[:, 0]
    tgt = torch.maximum(torch.minimum(cmd, vm), -vm)
    return torch.exp(-((v - tgt) ** 2) / std ** 2)


def track_wz_exp(env, std: float = 0.7, action_name: str = "ctrl"):
    t = env.action_manager.get_term(action_name)
    cmd = env.command_manager.get_command(t.cfg.command_name)[:, 1]
    return torch.exp(-((t._asset.data.root_ang_vel_w[:, 2] - cmd) ** 2) / std ** 2)


def upright_l2(env, action_name: str = "ctrl"):
    g = env.action_manager.get_term(action_name)._asset.data.projected_gravity_b
    return g[:, 0] ** 2 + g[:, 1] ** 2


def residual_l2(env, action_name: str = "ctrl"):
    return (env.action_manager.get_term(action_name)._raw.clamp(-1, 1) ** 2).sum(1)


def residual_rate_l2(env, action_name: str = "ctrl"):
    t = env.action_manager.get_term(action_name)
    return ((t._raw - t._prev_raw).clamp(-2, 2) ** 2).sum(1)


def wheel_saturation(env, frac: float = 0.9, action_name: str = "ctrl"):
    t = env.action_manager.get_term(action_name)
    w = (t._asset.data.joint_vel[:, t.wheel_ids]).abs().max(1).values / (W_MAX0 * t.motor_true)
    return torch.clamp(w - frac, min=0.0)
