"""CAD 4절링크 폐루프 모델용 변환층 — 단순화(직선관절) 모델과 **정책 입출력을 똑같이** 맞춘다.

그래서 단순화 모델에서 학습한 정책을 그대로 올려 보고, 이어서(resume) 학습할 수 있다.

  정책 다리 액션  h(고관절~바퀴중심, m)  ->  모터각 M = theta_of_h(h + R) - theta0   (R 쪽은 부호 반대)
  정책 다리 관측  h, dh/dt               <-  M, dM/dt  (leg_map 표를 torch 로 보간)
  정책 바퀴       +y 축 기준 토크/속도    <-> CAD 바퀴축은 좌 -y, 우 +y  -> 왼쪽만 부호 반전
  몸체 속도       COM 기준 (CAD 몸체 원점은 고관절 중점에서 8 cm 떨어져 있어 회전 중 속도가 왜곡된다)

폐루프 모델 규칙 (2026-09-25 loop_check / view_cad 로 확인):
  * 모터는 explicit PD (PhysX implicit 드라이브는 Exclude 관절 구속력을 못 본다)
  * 수동 관절 I, K 한계 없음 (make_loop_urdf.py 가 뺀다)
  * 회전자 반사관성은 fix_urdf 가 링크 관성에 이미 넣었다 -> armature 0
"""

from __future__ import annotations

import math
import os
import sys
from typing import TYPE_CHECKING

import torch
from isaaclab.actuators import DCMotorCfg, IdealPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.assets import Articulation
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.envs.mdp.actions import joint_actions
from isaaclab.envs.mdp.actions.actions_cfg import JointEffortActionCfg, JointPositionActionCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.sim import UsdFileCfg
from isaaclab.sim.schemas import ArticulationRootPropertiesCfg, RigidBodyPropertiesCfg
from isaaclab.utils import configclass

sys.path.insert(0, os.path.expanduser("~/perseverance/sim/model"))
import leg_map  # noqa: E402

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

R_WHEEL = leg_map.R_WHEEL
THETA0 = math.radians(45.002)                 # CAD 영점 자세의 theta_kR (M = 0)
M_SIGN = {"L": +1.0, "R": -1.0}               # M 증가 = 다리 펴짐 (L), R 은 반대
WHEEL_SIGN = (-1.0, +1.0)                     # (L, R): CAD 바퀴축 -> 정책의 +y 규약
LEG_JOINTS = ["L_joint_M", "R_joint_M"]
WHEEL_JOINTS = ["L_joint_W", "R_joint_W"]
USD_PATH = os.path.expanduser("~/wheeled_biped_isaaclab/usd_loop/robot_simple.usd")

# --- leg_map 표 (theta -> 다리 관절값, dh/dtheta) -------------------------------
_TH = torch.linspace(leg_map.THETA_MIN - 0.08, leg_map.THETA_MAX + 0.08, 2001, dtype=torch.float64)
_HJ = torch.tensor([leg_map.h_of_theta(float(t)) - R_WHEEL for t in _TH], dtype=torch.float64)
_DH = torch.tensor([leg_map.dh_dtheta(float(t)) for t in _TH], dtype=torch.float64)
assert torch.all(_HJ[1:] > _HJ[:-1]), "h(theta) 가 단조가 아니다"


def _interp(x, xp, fp):
    xp = xp.to(x.device, x.dtype); fp = fp.to(x.device, x.dtype)
    idx = torch.clamp(torch.searchsorted(xp, x.contiguous()), 1, len(xp) - 1)
    x0, x1 = xp[idx - 1], xp[idx]; y0, y1 = fp[idx - 1], fp[idx]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def theta_from_M(M):                  # (N,2) [L,R] -> theta
    return THETA0 + M * torch.tensor([M_SIGN["L"], M_SIGN["R"]], device=M.device, dtype=M.dtype)


def M_from_theta(th):
    return (th - THETA0) * torch.tensor([M_SIGN["L"], M_SIGN["R"]], device=th.device, dtype=th.dtype)


def hj_from_M(M):
    return _interp(theta_from_M(M), _TH, _HJ)


def M_from_hj(hj):
    return M_from_theta(_interp(hj, _HJ, _TH))


def dh_from_M(M):
    return _interp(theta_from_M(M), _TH, _DH)


def leg_state(asset: Articulation):
    ids = asset.find_joints(LEG_JOINTS, preserve_order=True)[0]
    M = asset.data.joint_pos[:, ids]
    Md = asset.data.joint_vel[:, ids]
    sgn = torch.tensor([M_SIGN["L"], M_SIGN["R"]], device=M.device, dtype=M.dtype)
    return hj_from_M(M), dh_from_M(M) * Md * sgn


# --- 관측 ---------------------------------------------------------------------
def leg_pos_rel(env: "ManagerBasedRLEnv", default: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    h, _ = leg_state(env.scene[asset_cfg.name])
    return h - default


def joint_vel_like_simple(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """단순화 모델의 joint_vel_rel 과 같은 순서/의미: [dh_L, dh_R, w_wheel_L, w_wheel_R] (+y 규약)."""
    asset = env.scene[asset_cfg.name]
    _, hd = leg_state(asset)
    wid = asset.find_joints(WHEEL_JOINTS, preserve_order=True)[0]
    w = asset.data.joint_vel[:, wid] * torch.tensor(WHEEL_SIGN, device=hd.device)
    return torch.cat([hd, w], dim=1)


def leg_h_mean(asset: Articulation):
    h, _ = leg_state(asset)
    return h.mean(dim=1)


# --- 액션 ---------------------------------------------------------------------
class CADLegAction(joint_actions.JointPositionAction):
    """정책 다리 액션(명령 높이 + 0.03 m * a) -> 모터각 M 목표."""
    cfg: "CADLegActionCfg"

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        h = self._env.command_manager.get_command(self.cfg.command_name)[:, 2:3]
        hj = torch.clamp(h + torch.clamp(actions, -1.0, 1.0) * self.cfg.leg_scale, self.cfg.h_min, self.cfg.h_max)
        self._processed_actions = M_from_hj(hj)


@configclass
class CADLegActionCfg(JointPositionActionCfg):
    class_type: type = CADLegAction
    command_name: str = "base_velocity"
    leg_scale: float = 0.03
    h_min: float = 0.1225
    h_max: float = 0.2425


class CADWheelAction(joint_actions.JointEffortAction):
    """정책 바퀴 토크(+y 규약, 20 Hz LPF) -> CAD 바퀴축 부호."""
    cfg: "CADWheelActionCfg"

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._alpha = 1.0 - math.exp(-2.0 * math.pi * cfg.cutoff_hz * env.step_dt)
        self._filtered = torch.zeros_like(self._raw_actions)
        self._sign = torch.tensor(WHEEL_SIGN, device=self.device)

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        u = torch.clamp(actions, -1.0, 1.0) * self.cfg.torque_scale
        self._filtered = self._filtered + self._alpha * (u - self._filtered)
        self._processed_actions = self._filtered * self._sign

    def reset(self, env_ids=None):
        super().reset(env_ids)
        if env_ids is None:
            self._filtered[:] = 0.0
        else:
            self._filtered[env_ids] = 0.0


@configclass
class CADWheelActionCfg(JointEffortActionCfg):
    class_type: type = CADWheelAction
    cutoff_hz: float = 20.0
    torque_scale: float = 1.5


# --- 로봇 ---------------------------------------------------------------------
H0_JOINT = float(leg_map.h_of_theta(THETA0) - R_WHEEL)     # CAD 영점 자세 다리 관절값 (0.1365)

CAD_ROBOT_CFG = ArticulationCfg(
    spawn=UsdFileCfg(
        usd_path=USD_PATH,
        rigid_props=RigidBodyPropertiesCfg(
            disable_gravity=False, retain_accelerations=False, linear_damping=0.0, angular_damping=0.0,
            max_linear_velocity=100.0, max_angular_velocity=3600.0,   # deg/s (IsaacLab 단위)
            max_depenetration_velocity=1.0),
        articulation_props=ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=1),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, H0_JOINT + R_WHEEL + 0.01),
        joint_pos={".*": 0.0},        # CAD 영점 = 폐루프가 맞는 자세
        joint_vel={".*": 0.0},
    ),
    actuators={
        # 다리 모터: explicit PD. AK60-6 피크 9 Nm.
        "legs": IdealPDActuatorCfg(joint_names_expr=[".*_joint_M"], stiffness=60.0, damping=1.5,
                                   effort_limit=9.0, velocity_limit=24.4),
        "passive": ImplicitActuatorCfg(joint_names_expr=[".*_joint_[IK]"], stiffness=0.0, damping=0.02),
        # 바퀴: DC 모터 모델 (단순화 모델과 같은 값). 회전자 관성은 링크 izz 에 이미 있다 -> armature 0.
        "wheels": DCMotorCfg(joint_names_expr=[".*_joint_W"], saturation_effort=7.0, effort_limit=7.0,
                             velocity_limit=18.85, stiffness=0.0, damping=0.0),
    },
    soft_joint_pos_limit_factor=1.0,
)
