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


def spin_in_place_exp(env: "ManagerBasedRLEnv", command_name: str, asset_cfg: SceneEntityCfg,
                      std: float = 0.5, vx_blend: tuple = (0.05, 0.15), wz_blend: tuple = (0.1, 0.3)) -> torch.Tensor:
    """정지 명령에서 회전할 때 두 바퀴가 서로 반대로 같은 크기로 돌게 한다 (2026-09-25 사용자 요청).

    두 바퀴 각속도 평균(+y 규약) = 차축 중점의 직진 성분. 0 이 아니면 한쪽 바퀴를 축으로 원을 그린다
    (측정: 0.5 rad/s 제자리 회전에서 L -0.32 / R +1.35 rad/s, 6 s 에 중심 24 cm 이동).

    **정지 명령 AND 회전 명령일 때만** 켜진다. 첫 판(L2 벌점, 정지면 항상)은 외란을 받아낼 때 필요한
    양쪽 바퀴 같은 방향 회전까지 벌해서 정책이 넘어지는 쪽을 택했다 (완주율 0.005 로 붕괴).
    그래서 켜지는 조건을 좁히고, 상한 있는 보상(0~1)으로 바꿨다 — 죽는 편이 이득이 되지 않는다.
    asset_cfg 에 바퀴 관절 (좌, 우 순서) 을 준다.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    w = asset.data.joint_vel[:, asset_cfg.joint_ids]
    if "L_joint_W" in asset.joint_names:          # CAD 폐루프 모델: 왼쪽 바퀴축이 -y
        from .cad import WHEEL_SIGN
        w = w * torch.tensor(WHEEL_SIGN, device=w.device)
    common = w.mean(dim=1)
    cmd = env.command_manager.get_command(command_name)

    def smooth(x, lo, hi):
        t = ((x - lo) / (hi - lo)).clamp(0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)

    gate = (1.0 - smooth(cmd[:, 0].abs(), *vx_blend)) * smooth(cmd[:, 1].abs(), *wz_blend)
    return gate * torch.exp(-torch.square(common) / std**2)


# --- 거친 지형: 짐벌처럼 몸통 고정 (terrain.py) -----------------------------------------------------
def gimbal_height_exp(env: "ManagerBasedRLEnv", command_name: str, std: float, sensor_cfg: SceneEntityCfg,
                      asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), std_auto: float | None = None) -> torch.Tensor:
    """몸통 높이를 **바퀴 밑이 아니라 몸통 아래 넓은 지면 평균** 기준으로 h_ref 에 맞춘다.

        오차 = (바퀴 접지 높이 - 지면 평균) + (다리 평균 - h_ref)

    바퀴가 턱 위(+2 cm)에 올라가면 다리를 2 cm 줄여야 오차 0 -> 몸통은 그 자리. 파인 곳이면 다리를 편다.
    평지에서는 첫 항이 0 이라 기존 track_height(다리 평균 = h_ref)와 같다.
    몸체 원점-고관절 간 기하 오프셋이 식에서 빠져서 보정 상수가 필요 없다.
    """
    from .terrain import ground_patch_mean, wheel_ground_mean

    asset: Articulation = env.scene[asset_cfg.name]
    h_ref = env.command_manager.get_command(command_name)[:, 2]
    if "L_joint_M" in asset.joint_names:
        from .cad import leg_state
        hj = leg_state(asset)[0].mean(dim=1)
    else:
        hj = asset.data.joint_pos[:, asset.find_joints(".*_leg")[0]].mean(dim=1)
    err = (wheel_ground_mean(asset) - ground_patch_mean(env, sensor_cfg)) + (hj - h_ref)
    cmd = env.command_manager.get_command(command_name)
    if std_auto is not None and cmd.shape[1] > 3:
        # 자동 모드(m=1): 평균 높이는 정책이 정한다 -> 공칭 높이는 느슨한 선호로만
        s = torch.where(cmd[:, 3] > 0.5, std_auto, std)
    else:
        s = std
    return torch.exp(-torch.square(err) / s**2)


def base_vz_world_l2(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """몸통(COM) 월드 수직속도. 요철을 넘을 때 몸이 튀는 것을 직접 벌한다."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.square(asset.data.root_com_lin_vel_w[:, 2])


def base_vertical_acc_exp(env: "ManagerBasedRLEnv", std: float,
                          asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """몸통 COM 수직가속도 (짐벌 지표, 승차감 ISO 2631 과 같은 축).

    속도가 아니라 가속도라서 경사를 일정하게 오르내리거나 높이를 천천히 바꾸는 건 벌하지 않고,
    요철에서 몸이 튀는 것만 잡는다. 두 모드 공통.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    az = asset.data.body_com_lin_acc_w[:, 0, 2]      # 0 = 루트(base_link)
    return torch.exp(-torch.square(az) / std**2)


def leg_action_rate_phys(env: "ManagerBasedRLEnv", scale_ratio: float) -> torch.Tensor:
    """다리 행동 변화를 **실제 다리 이동량** 기준으로 (행동 0, 1 = 다리 L, R).

    다리 권한을 0.03 -> 0.12 m 로 넓히면(scale_ratio 4) 같은 action_rate 벌점이 실제 다리 움직임으로는 1/16 이
    된다. m5100 에서 평지 수직가속 RMS 0.4 -> 1~3 m/s^2 (다리 떨림)로 나타났다. 예전 물리량 기준을 되살린다.
    """
    d = env.action_manager.action[:, :2] - env.action_manager.prev_action[:, :2]
    return torch.sum(torch.square(d), dim=1) * scale_ratio**2


def leg_stroke_margin(env: "ManagerBasedRLEnv", command_name: str, margin: float, h_min: float, h_max: float,
                      asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """자동 모드에서 다리가 행정 양끝 margin 안으로 붙는 것 (0~, 다리마다 (1 - 여유/margin)^2).

    m5100 자동 모드는 모든 지형에서 다리 119.7 mm (하한 122.5 아래, 처짐 포함)에 붙었다 — 낮을수록 안정하니
    제일 쉬운 답이지만, 하한에서는 더 줄일 수 없어 요철을 못 흡수한다 (파도 roll 7 deg). 수동 모드는 사용자
    높이라 끈다.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    if "L_joint_M" in asset.joint_names:
        from .cad import leg_state
        h = leg_state(asset)[0]
    else:
        h = asset.data.joint_pos[:, asset.find_joints(".*_leg")[0]]
    d = torch.minimum(h - h_min, h_max - h)
    pen = torch.sum(torch.square(torch.clamp(1.0 - d / margin, min=0.0)), dim=1)
    cmd = env.command_manager.get_command(command_name)
    if cmd.shape[1] > 3:
        pen = pen * (cmd[:, 3] > 0.5).float()
    return pen
