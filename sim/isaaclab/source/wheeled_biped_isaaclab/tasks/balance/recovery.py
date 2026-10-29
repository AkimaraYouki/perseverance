"""넘어짐 복구 (일어서기) 학습: 넘어진 자세에서 시작 -> 몸을 세워 잠깐 버티면 성공, 그 뒤는 LQR 이 받는다.

실기 흐름: 기울기 > 50 deg (LQR 포기) -> 이 정책 -> stood_up 조건 (수직 근처 + 거의 정지, hold_s 동안) -> LQR.
정책 입력은 실기 센서로 만들 수 있는 것만 (IMU 자세·자이로, 다리 엔코더·속도·전류, 바퀴 속도, 직전 행동).
행동: 다리 길이 절대 목표 (행정 전체) + 바퀴 토크 (피크 7 N·m 전체).
"""
from __future__ import annotations

import torch

from isaaclab.utils import configclass

from . import cad


# --- 행동 ---------------------------------------------------------------------
class CADLegAbsAction(cad.CADLegAction):
    """다리 목표 = 행정 가운데 + a x 반행정 (a 는 ±1 로 자름). 명령 높이와 무관 (복구 중엔 명령이 없다)."""

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        c = self.cfg
        mid, half = 0.5 * (c.h_min + c.h_max), 0.5 * (c.h_max - c.h_min)
        hj = mid + torch.clamp(actions, -1.0, 1.0) * half
        self._processed_actions = cad.M_from_hj(hj)


@configclass
class CADLegAbsActionCfg(cad.CADLegActionCfg):
    class_type: type = CADLegAbsAction


# --- 보상·종료 -----------------------------------------------------------------
def _up(env) -> torch.Tensor:
    """몸 z 축이 위를 향한 정도: 1 = 수직, 0 = 거꾸로 (projected_gravity_b z = -1 이 수직)."""
    g = env.scene["robot"].data.projected_gravity_b
    return (1.0 - g[:, 2]) * 0.5


def upright_sq(env) -> torch.Tensor:
    return _up(env) ** 2


def tilt(env) -> torch.Tensor:
    g = env.scene["robot"].data.projected_gravity_b
    return torch.acos(torch.clamp(-g[:, 2], -1.0, 1.0))


def stood_up(env, tilt_max: float = 0.2, rate_max: float = 1.0, hold_s: float = 0.3) -> torch.Tensor:
    """수직 근처 (tilt < tilt_max rad) + 몸 roll/pitch 각속도 < rate_max 를 hold_s 동안 유지 -> 성공 (LQR 인계)."""
    d = env.scene["robot"].data
    ok = (tilt(env) < tilt_max) & (d.root_ang_vel_b[:, :2].norm(dim=1) < rate_max)
    t = getattr(env, "_up_t", None)
    if t is None or t.shape[0] != env.num_envs:
        t = env._up_t = torch.zeros(env.num_envs, device=env.device)
    t[:] = torch.where(ok, t + env.step_dt, torch.zeros_like(t))
    return t >= hold_s


def reset_up_timer(env, env_ids: torch.Tensor):
    if getattr(env, "_up_t", None) is not None:
        env._up_t[env_ids] = 0.0


def fall_pitch(env) -> torch.Tensor:
    """관측/분석용: 몸 pitch (+ = 앞으로 기욺) [rad]."""
    g = env.scene["robot"].data.projected_gravity_b
    return torch.atan2(g[:, 0], -g[:, 2]).unsqueeze(1)


def root_height(env) -> torch.Tensor:
    """크리틱용 몸통 높이 [m] (평지)."""
    return (env.scene["robot"].data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]).unsqueeze(1)
