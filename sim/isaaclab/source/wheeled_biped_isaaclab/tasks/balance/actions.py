"""바퀴/다리 액션 항 — 원시 액션을 ±1 로 자르고, 바퀴는 1차 저역통과(LPF)를 건다.

왜 래퍼(clip_actions)가 아니라 여기서 자르나:
  래퍼에서 자르면 환경은 잘린 값만 보게 되어, ±1 밖으로 나간 정책 평균에
  벌점을 줄 방법이 없다. 실제로 2026-09-25 학습은 원시 출력이 최대 194 까지
  밀려나 바퀴가 ±1.5 Nm 로 뱅뱅(진동)했다. 여기서 자르면 env.action_manager.action
  (원시)은 그대로 남아 rewards.action_out_of_range 로 벌할 수 있다.

LPF (바퀴 토크):
  y[k] = y[k-1] + a (u[k] - y[k-1]),  a = 1 - exp(-2 pi fc dt)
  fc 20 Hz, dt 5 ms 면 a = 0.466, 시정수 8 ms.
  **실기 제어기에도 같은 필터를 같은 주기로 넣어야 한다** (sim2real 일치).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Sequence

import torch
from isaaclab.envs.mdp.actions import joint_actions
from isaaclab.envs.mdp.actions.actions_cfg import JointEffortActionCfg, JointPositionActionCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class ClampedJointPositionAction(joint_actions.JointPositionAction):
    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        a = torch.clamp(actions, -1.0, 1.0)
        self._processed_actions = a * self._scale + self._offset
        if self.cfg.clip is not None:
            self._processed_actions = torch.clamp(
                self._processed_actions, min=self._clip[:, :, 0], max=self._clip[:, :, 1])


class FilteredJointEffortAction(joint_actions.JointEffortAction):
    cfg: "FilteredJointEffortActionCfg"

    def __init__(self, cfg, env: "ManagerBasedEnv"):
        super().__init__(cfg, env)
        dt = env.step_dt
        self._alpha = 1.0 - math.exp(-2.0 * math.pi * cfg.cutoff_hz * dt) if cfg.cutoff_hz > 0 else 1.0
        self._filtered = torch.zeros_like(self._raw_actions)

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        u = torch.clamp(actions, -1.0, 1.0) * self._scale + self._offset
        self._filtered = self._filtered + self._alpha * (u - self._filtered)
        self._processed_actions = self._filtered
        if self.cfg.clip is not None:
            self._processed_actions = torch.clamp(
                self._processed_actions, min=self._clip[:, :, 0], max=self._clip[:, :, 1])

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        if env_ids is None:
            self._filtered[:] = 0.0
        else:
            self._filtered[env_ids] = 0.0


@configclass
class ClampedJointPositionActionCfg(JointPositionActionCfg):
    class_type: type = ClampedJointPositionAction


@configclass
class FilteredJointEffortActionCfg(JointEffortActionCfg):
    class_type: type = FilteredJointEffortAction
    cutoff_hz: float = 20.0
