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
        self._leg_ids, _ = self.robot.find_joints(cfg.leg_joint_names)
        self._cmd = torch.zeros(self.num_envs, 3, device=self.device)      # vx, wz, h_ref
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

    def set(self, vx=None, wz=None, h=None, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        if vx is not None:
            self._cmd[ids, 0] = vx
        if wz is not None:
            self._cmd[ids, 1] = wz
        if h is not None:
            self.h_target[ids] = h

    # --- 내부 ---------------------------------------------------------------
    def _update_metrics(self):
        dt = self._env.step_dt
        max_t = self.cfg.resampling_time_range[1]
        v = self.robot.data.root_lin_vel_b
        w = self.robot.data.root_ang_vel_b
        q = self.robot.data.joint_pos[:, self._leg_ids].mean(dim=1)
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
        zero = torch.rand(n, device=dev) < self.cfg.zero_vel_prob
        vx[zero] = 0.0
        wz[zero] = 0.0
        self._cmd[env_ids, 0] = vx
        self._cmd[env_ids, 1] = wz
        self.h_target[env_ids] = h

    def reset(self, env_ids: Sequence[int] | None = None):
        extras = super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        # 기준 높이는 현재 다리에서 출발한다 (리셋 순간 목표로 순간이동하지 않게)
        self._cmd[ids, 2] = self.robot.data.joint_pos[ids][:, self._leg_ids].mean(dim=1)
        return extras

    def _update_command(self):
        step = self.cfg.max_height_rate * self._env.step_dt
        d = torch.clamp(self.h_target - self._cmd[:, 2], -step, step)
        self._cmd[:, 2] += d


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

    @configclass
    class Ranges:
        lin_vel_x: tuple = (-0.45, 0.45)
        ang_vel_z: tuple = (-1.0, 1.0)
        height: tuple = (0.130, 0.235)     # 관절값. h(지면~고관절) = 190 ~ 295 mm

    ranges: Ranges = Ranges()
