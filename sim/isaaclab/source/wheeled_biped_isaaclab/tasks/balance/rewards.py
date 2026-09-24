"""휠-레그 로봇 전용 보상 항.

Isaac Lab 기본 항으로 안 되는 두 가지만 여기 둔다.
  1) 다리 길이를 명령값에 맞추기 (몸체 높이가 아니라 관절값 기준)
  2) 좌우 다리 길이를 맞춰 롤 흔들림 억제
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def leg_length_target_l2(
    env: "ManagerBasedRLEnv",
    target: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", joint_names=[".*_leg"]),
) -> torch.Tensor:
    """두 다리 길이가 목표에서 벗어난 정도 (제곱합).

    몸체 높이(root z)를 직접 쓰지 않는 이유: 기울어져 있을 때 root z 는
    다리 길이와 어긋난다. 실기에서 측정 가능한 값도 모터 각도라
    관절값을 기준으로 두는 편이 sim2real 에 맞다.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    q = asset.data.joint_pos[:, asset_cfg.joint_ids]
    return torch.sum(torch.square(q - target), dim=1)


def leg_length_symmetry_l2(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", joint_names=[".*_leg"]),
) -> torch.Tensor:
    """좌우 다리 길이 차이. 평지에서는 0 이어야 한다."""
    asset: Articulation = env.scene[asset_cfg.name]
    q = asset.data.joint_pos[:, asset_cfg.joint_ids]
    return torch.square(q[:, 0] - q[:, 1])


def wheel_effort_l2(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", joint_names=[".*_wheel_joint"]),
) -> torch.Tensor:
    """바퀴 토크 제곱합. 마찰 한계(고무-대리석 mu 0.7 에서 0.82 Nm)를
    한참 넘는 토크를 내봐야 실기에서는 미끄러지기만 하므로 억제한다."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.applied_torque[:, asset_cfg.joint_ids]), dim=1)


def action_out_of_range(env: "ManagerBasedRLEnv", limit: float = 1.0) -> torch.Tensor:
    """원시 액션이 ±limit 밖으로 나간 양의 제곱합.

    액션은 액션 항 안에서 ±1 로 잘리므로 그 밖은 효과가 없는데, 벌이 없으면
    정책 평균이 계속 바깥으로 밀려나 출력이 ±1 에 붙은 뱅뱅 제어가 된다.
    """
    a = env.action_manager.action
    return torch.sum(torch.square(torch.clamp(torch.abs(a) - limit, min=0.0)), dim=1)


# --- 명령 추종 (commands.WheelLegCommand: [vx, wz, h_ref]) ------------------
def track_vx_exp(env: "ManagerBasedRLEnv", command_name: str, std: float,
                 asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """전진 속도 추종. 옆 미끄럼(vy)은 목표 0 으로 같이 본다."""
    asset: Articulation = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    v = asset.data.root_lin_vel_b
    err = torch.square(cmd[:, 0] - v[:, 0]) + torch.square(v[:, 1])
    return torch.exp(-err / std**2)


def track_wz_exp(env: "ManagerBasedRLEnv", command_name: str, std: float,
                 asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    return torch.exp(-torch.square(cmd[:, 1] - asset.data.root_ang_vel_b[:, 2]) / std**2)


def track_height_exp(env: "ManagerBasedRLEnv", command_name: str, std: float,
                     asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """두 다리 관절값이 높이 기준(h_ref)에 붙어 있는지. asset_cfg 는 params 로 넘길 것."""
    asset: Articulation = env.scene[asset_cfg.name]
    h = env.command_manager.get_command(command_name)[:, 2:3]
    q = asset.data.joint_pos[:, asset_cfg.joint_ids]
    return torch.exp(-torch.mean(torch.square(q - h), dim=1) / std**2)
