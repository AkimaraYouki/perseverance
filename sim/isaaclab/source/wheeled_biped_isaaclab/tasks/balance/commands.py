"""휠-레그 명령: (vx, wz, 다리길이) — 오리(SummerProject)의 명령 구조를 옮겼다.

오리에서 가져온 것 (open_duck_mini_isaaclab/tasks/velocity/joystick_env.py):
  * 축별 균등 추첨 + **순수 단일축 명령**을 가중 추첨으로 섞는다.
    오리 기록: 축별 독립 균등으로만 뽑으면 "한 방향만" 명령이 거의 안 나와서,
    순수축으로 보는 6 방향 시험에서 25 개 버전 전부 좌/우 오차가 전진의 2~4 배였다.
  * 정지 명령 확률, 주기적 재추첨.
  * pin(): 측정·재생·조이스틱이 명령을 직접 쓸 때 재추첨이 덮어쓰지 않게 한다.

이 로봇은 옆으로 못 간다(두 바퀴, 비홀로노믹). 조종 축은 vx, wz, 높이다.
그래서 "6 방향" = 전진/후진, 좌회전/우회전, 올라가기/내려가기.

높이:
  명령 단위는 다리 관절값 = 고관절(모터축) ~ 바퀴중심 [m] (leg_map h 에서 바퀴반지름 뺀 값).
  목표(h_target)는 튀어도, 실제로 따라갈 기준(h_ref)은 max_height_rate 로만 움직인다.
  명령 텐서의 세 번째 값은 h_ref 다 — 정책이 보는 것도, 다리 액션의 기준도 이것이다.
  조이스틱에서는 스틱이 h_target 을 올리고 내리므로 실기와 같은 구조가 된다.

높이 모드 (2026-09-25 사용자, with_mode=True 일 때만 — 명령이 4 개가 된다: [vx, wz, h_ref, m]):
  m = 0  수동: 평균 높이 = 사용자 h_ref (촘촘히 추종). **평지 전용** (manual_flat_only).
  m = 1  자동: 평균 높이도 정책이 외란·지형에 맞춰 정한다. h_ref 는 공칭값(auto_height)으로 가고
               보상에서 느슨한 선호로만 쓴다 (rewards.gimbal_height_exp 의 std_auto).
  with_mode=False(평지 과제 기본)면 예전과 똑같이 3 개 — 기존 정책/스크립트 호환.
"""

from __future__ import annotations

from dataclasses import MISSING
from typing import TYPE_CHECKING, Sequence

import torch
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class WheelLegCommand(CommandTerm):
    cfg: "WheelLegCommandCfg"

    def __init__(self, cfg: "WheelLegCommandCfg", env: "ManagerBasedRLEnv"):
        super().__init__(cfg, env)
        self.robot = env.scene[cfg.asset_name]
        self._cad = "L_joint_M" in self.robot.joint_names       # CAD 폐루프 모델이면 모터각 -> 다리 관절값
        self._leg_ids = None if self._cad else self.robot.find_joints(cfg.leg_joint_names)[0]
        # vx, wz, h_ref (+ m: 0 수동 / 1 자동)
        n_cmd = 3 + int(cfg.with_mode) + int(cfg.with_jump)
        self._cmd = torch.zeros(self.num_envs, n_cmd, device=self.device)
        # 점프 (with_jump): 명령 마지막 칸 j. 버튼을 누르면 jump_pulse_s 동안 1. jump_window_s 동안 "점프 중" (보상 문).
        self.jump_timer = torch.full((self.num_envs,), 99.0, device=self.device)
        self.jump_trigger_d = torch.empty(self.num_envs, device=self.device).uniform_(*cfg.jump_trigger_dist)
        self.h_target = torch.full((self.num_envs,), cfg.default_height, device=self.device)
        self.pinned = False
        self.metrics["error_vx"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_wz"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_h"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        r = self.cfg.ranges
        return (f"WheelLegCommand: vx {r.lin_vel_x}, wz {r.ang_vel_z}, h {r.height}, "
                f"pure {self.cfg.pure_axis_prob} w{self.cfg.pure_axis_weights}, zero {self.cfg.zero_vel_prob}")

    @property
    def command(self) -> torch.Tensor:
        return self._cmd

    # --- 측정 / 조종용 ------------------------------------------------------
    def pin(self, on: bool = True):
        """재추첨을 멈춘다. 이후 set() 으로만 명령이 바뀐다."""
        self.pinned = bool(on)

    @property
    def jump_window(self) -> torch.Tensor:
        return self.jump_timer < self.cfg.jump_window_s

    def trigger_jump(self, env_ids=None):
        """조종자 버튼 (재생·조이스틱용). 쿨다운 중이면 무시."""
        ids = slice(None) if env_ids is None else env_ids
        ok = self.jump_timer[ids] > self.cfg.jump_cooldown_s
        self.jump_timer[ids] = torch.where(ok, torch.zeros_like(self.jump_timer[ids]), self.jump_timer[ids])

    def set(self, vx=None, wz=None, h=None, env_ids=None, mode=None):
        """mode: 0 수동 / 1 자동 (with_mode 일 때). 자동이면 h 는 무시하고 공칭 높이로 간다."""
        ids = slice(None) if env_ids is None else env_ids
        if mode is not None and self.cfg.with_mode:
            self._cmd[ids, 3] = mode
            if h is None:
                h = torch.where(self._cmd[ids, 3] > 0.5, self.cfg.auto_height, self.h_target[ids])
        if self.cfg.with_mode and h is not None:
            h = torch.where(self._cmd[ids, 3] > 0.5, torch.full_like(self.h_target[ids], self.cfg.auto_height),
                            torch.as_tensor(h, device=self.device, dtype=torch.float).expand_as(self.h_target[ids]))
        if vx is not None:
            self._cmd[ids, 0] = vx
        if wz is not None:
            self._cmd[ids, 1] = wz
        if h is not None:
            self.h_target[ids] = h

    # --- 내부 ---------------------------------------------------------------
    def _leg_h(self, ids=slice(None)):
        if self._cad:
            from .cad import leg_h_mean
            return leg_h_mean(self.robot)[ids]
        return self.robot.data.joint_pos[ids][:, self._leg_ids].mean(dim=1)

    def _update_metrics(self):
        dt = self._env.step_dt
        max_t = self.cfg.resampling_time_range[1]
        v = self.robot.data.root_com_lin_vel_b
        w = self.robot.data.root_ang_vel_b
        q = self._leg_h()
        self.metrics["error_vx"] += torch.abs(self._cmd[:, 0] - v[:, 0]) * dt / max_t
        self.metrics["error_wz"] += torch.abs(self._cmd[:, 1] - w[:, 2]) * dt / max_t
        self.metrics["error_h"] += torch.abs(self._cmd[:, 2] - q) * dt / max_t

    def _resample_command(self, env_ids: Sequence[int]):
        if self.pinned:
            return
        n = len(env_ids)
        r = self.cfg.ranges
        dev = self.device
        vx = torch.empty(n, device=dev).uniform_(*r.lin_vel_x)
        wz = torch.empty(n, device=dev).uniform_(*r.ang_vel_z)
        h = torch.empty(n, device=dev).uniform_(*r.height)
        # 순수 단일축: 0 = vx 만, 1 = wz 만, 2 = 높이만(주행 정지). 높이는 자세라서
        # vx/wz 순수축에서도 뽑힌 값을 그대로 둔다.
        if self.cfg.pure_axis_prob > 0.0:
            pure = torch.rand(n, device=dev) < self.cfg.pure_axis_prob
            w = torch.tensor(self.cfg.pure_axis_weights, device=dev, dtype=torch.float)
            axis = torch.multinomial(w / w.sum(), n, replacement=True)
            wz[pure & (axis == 0)] = 0.0
            vx[pure & (axis == 1)] = 0.0
            vx[pure & (axis == 2)] = 0.0
            wz[pure & (axis == 2)] = 0.0
        # 고속 코너링: 속도와 회전을 둘 다 크게 (2026-09-25 사용자 요청 — 코너에서 안쪽으로 기울기 학습)
        if self.cfg.fast_turn_prob > 0.0:
            ft = torch.rand(n, device=dev) < self.cfg.fast_turn_prob
            m = int(ft.sum())
            if m:
                sv = torch.where(torch.rand(m, device=dev) < 0.5, -1.0, 1.0)
                sw = torch.where(torch.rand(m, device=dev) < 0.5, -1.0, 1.0)
                vx[ft] = sv * torch.empty(m, device=dev).uniform_(*self.cfg.fast_turn_vx)
                wz[ft] = sw * torch.empty(m, device=dev).uniform_(*self.cfg.fast_turn_wz)
        zero = torch.rand(n, device=dev) < self.cfg.zero_vel_prob
        vx[zero] = 0.0
        wz[zero] = 0.0
        if self.cfg.with_mode:
            auto = torch.rand(n, device=dev) < self.cfg.auto_mode_prob
            if self.cfg.manual_flat_only:
                # 수동 높이 모드는 평지 전용 (사용자 2026-09-25). 거친 지형 위에서는 항상 자동.
                from .terrain import on_flat_tile
                ids_t = torch.as_tensor(env_ids, device=dev) if not isinstance(env_ids, slice) else torch.arange(self.num_envs, device=dev)
                auto |= ~on_flat_tile(self._env, ids_t)
            h[auto] = self.cfg.auto_height
            self._cmd[env_ids, 3] = auto.float()
        self._cmd[env_ids, 0] = vx
        self._cmd[env_ids, 1] = wz
        self.h_target[env_ids] = h

    def reset(self, env_ids: Sequence[int] | None = None):
        extras = super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self.jump_timer[ids] = 99.0
        # 기준 높이는 현재 다리에서 출발한다 (리셋 순간 목표로 순간이동하지 않게)
        self._cmd[ids, 2] = self._leg_h(ids)
        return extras

    def _update_command(self):
        step = self.cfg.max_height_rate * self._env.step_dt
        d = torch.clamp(self.h_target - self._cmd[:, 2], -step, step)
        self._cmd[:, 2] += d
        if self.cfg.with_jump:
            self._update_jump()

    def _update_jump(self):
        """학습 중 점프 발동: (1) 장애물 앞 자동 — 전진 명령 >= 0.2 m/s 이고 앞 d 지점이 발밑보다 3 cm 이상 높을 때.
        d 는 env 마다 jump_trigger_dist 에서 무작위라 일부러 이르거나 늦은 점프가 섞인다 (잘못 착지 -> 회복 학습, 사용자 요청).
        (2) 어디서나 무작위 (평지·험지 점프). 재생 중(pinned)에는 자동 발동을 끄고 trigger_jump() 로만."""
        dt = self._env.step_dt
        self.jump_timer += dt
        if not self.pinned:
            ready = self.jump_timer > self.cfg.jump_cooldown_s
            rnd = torch.rand(self.num_envs, device=self.device) < self.cfg.jump_rand_rate * dt
            auto = torch.zeros_like(ready)
            sens = self._env.scene.sensors.get("critic_scanner") if hasattr(self._env.scene, "sensors") else None
            if sens is not None:
                hits = sens.data.ray_hits_w                                   # (N, P, 3)
                rel = hits[..., :2] - sens.data.pos_w[:, None, :2]
                yaw = torch.atan2(2 * (self.robot.data.root_quat_w[:, 0] * self.robot.data.root_quat_w[:, 3]
                                       + self.robot.data.root_quat_w[:, 1] * self.robot.data.root_quat_w[:, 2]),
                                  1 - 2 * (self.robot.data.root_quat_w[:, 2] ** 2 + self.robot.data.root_quat_w[:, 3] ** 2))
                c, s_ = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
                xl = c * rel[..., 0] + s_ * rel[..., 1]
                yl = -s_ * rel[..., 0] + c * rel[..., 1]
                z = torch.nan_to_num(hits[..., 2], nan=-1e3, posinf=-1e3, neginf=-1e3)
                here = torch.where((xl.abs() < 0.1) & (yl.abs() < 0.15), z, torch.full_like(z, -1e3)).amax(dim=1)
                d = self.jump_trigger_d[:, None]
                ahead = torch.where((xl > d) & (xl < d + 0.1) & (yl.abs() < 0.15), z, torch.full_like(z, -1e3)).amax(dim=1)
                auto = (ahead - here > self.cfg.jump_step_thresh) & (self._cmd[:, 0] >= 0.2)
            fire = ready & (rnd | auto)
            if fire.any():
                self.jump_timer[fire] = 0.0
                self.jump_trigger_d[fire] = torch.empty(int(fire.sum()), device=self.device).uniform_(*self.cfg.jump_trigger_dist)
        self._cmd[:, -1] = (self.jump_timer < self.cfg.jump_pulse_s).float()


@configclass
class WheelLegCommandCfg(CommandTermCfg):
    class_type: type = WheelLegCommand
    asset_name: str = MISSING
    leg_joint_names: list = [".*_leg"]
    default_height: float = 0.1825
    max_height_rate: float = 0.10          # m/s. 전 구간(0.105 m)을 약 1 s 에
    pure_axis_prob: float = 0.35
    pure_axis_weights: tuple = (0.50, 0.35, 0.15)   # vx, wz, 높이 — 오리처럼 앞뒤 > 회전 > 나머지
    zero_vel_prob: float = 0.10
    fast_turn_prob: float = 0.0
    fast_turn_vx: tuple = (0.50, 0.85)     # |vx| [m/s]
    fast_turn_wz: tuple = (0.40, 1.00)     # |wz| [rad/s]
    with_mode: bool = False                # True 면 명령 4 번째 = 높이 모드 (0 수동 / 1 자동)
    auto_mode_prob: float = 0.0            # 재추첨 때 자동 모드 확률 (평지 타일에서)
    manual_flat_only: bool = False         # True: 거친 지형 타일 위에서는 항상 자동
    auto_height: float = 0.1825            # 자동 모드 공칭 높이 (다리 관절값, 범위 중앙)
    with_jump: bool = False                # True 면 명령 마지막 칸 = 점프 버튼 j (r6~)
    jump_pulse_s: float = 0.3              # 버튼 신호 길이
    jump_window_s: float = 0.7             # 점프 보상 문 (이륙~착지)
    jump_cooldown_s: float = 1.5
    jump_rand_rate: float = 0.05           # 초당 무작위 점프 확률 (평지·험지)
    jump_trigger_dist: tuple = (0.05, 0.40)  # 장애물 앞 자동 발동 거리 (무작위 = 이른/늦은 점프 섞기)
    jump_step_thresh: float = 0.03         # 앞이 발밑보다 이만큼 높으면 장애물

    @configclass
    class Ranges:
        lin_vel_x: tuple = (-0.45, 0.45)
        ang_vel_z: tuple = (-1.0, 1.0)
        height: tuple = (0.130, 0.235)     # 관절값. h(지면~고관절) = 190 ~ 295 mm

    ranges: Ranges = Ranges()
