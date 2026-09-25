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
    v = asset.data.root_com_lin_vel_b   # COM 기준 (CAD 몸체 원점은 고관절에서 8 cm 떨어져 있다)
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
    if "L_joint_M" in asset.joint_names:          # CAD 폐루프 모델: 모터각 -> 다리 관절값
        from .cad import leg_state
        q, _ = leg_state(asset)
    else:
        q = asset.data.joint_pos[:, asset_cfg.joint_ids]
    return torch.exp(-torch.mean(torch.square(q - h), dim=1) / std**2)


# --- 자세: 앞뒤는 벌점, 좌우는 코너링 목표 기울기 추종 -----------------------
def pitch_l2(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """앞뒤 기울기만. 두 바퀴 균형에서 가감속 때 앞뒤로 기우는 건 피할 수 없어서 좌우와 분리한다."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.square(asset.data.projected_gravity_b[:, 0])


def roll_lean_target(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """코너링 목표 좌우 기울기의 projected_gravity y 성분 (= sin phi_des).

    원심력과 중력의 합력 방향에 몸을 맞춘다: tan(phi) = v * wz / g.
    +y 가 왼쪽이므로 전진하며 좌회전(v>0, wz>0)하면 구심가속도가 +y, 몸은 왼쪽(안쪽)으로 기운다.
    몸이 왼쪽으로 phi 기울면 projected_gravity_b.y = +sin(phi).
    실제 속도(오도메트리)와 자이로 z 를 쓴다 — 실기에서도 둘 다 있다.
    직진이나 제자리 회전이면 v * wz = 0 이라 목표는 수평이다.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    a_c = asset.data.root_com_lin_vel_b[:, 0] * asset.data.root_ang_vel_b[:, 2]
    return torch.sin(torch.atan(a_c / 9.81))


def roll_track_exp(env: "ManagerBasedRLEnv", std: float,
                   asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """좌우 기울기가 코너링 목표(직진이면 수평)에 붙어 있는지."""
    asset: Articulation = env.scene[asset_cfg.name]
    err = asset.data.projected_gravity_b[:, 1] - roll_lean_target(env, asset_cfg)
    return torch.exp(-torch.square(err) / std**2)
