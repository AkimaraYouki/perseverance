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
from isaaclab.actuators import DCMotor, DCMotorCfg, IdealPDActuatorCfg, ImplicitActuatorCfg
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
# 로봇 USD (WB_WHEEL_R 로 고름):
#   70 mm = usd_loop          CAD export 7 + 바퀴 실측 (휠 + 고무 72.5 g, 고무 16.5 g, 2026-10-07) = 4.170 kg (기본). 형상은 6 과 같음
#           usd_loop_e7       CAD export 7 그대로 (질량 실측, 바퀴 CAD 70.2 g, 4.165 kg)
#           usd_loop_e6       CAD export 6 (바퀴 140 mm, 2026-09-27, 4.037 kg CAD 밀도) — WB_USD_DIR=usd_loop_e6 로 비교
#   60 mm = usd_loop_e5_R60   CAD export 5 (바퀴 120 mm, 이전 기본)
#   그 밖 = usd_loop_R{mm}    export 5 에서 바퀴 반지름·질량·관성만 바꾼 사본 (scripts/make_wheel_variant.py)
_R_MM = int(round(R_WHEEL * 1000))
_USD_DIR = os.environ.get("WB_USD_DIR") or {70: "usd_loop", 60: "usd_loop_e5_R60"}.get(_R_MM, f"usd_loop_R{_R_MM}")
USD_PATH = os.path.expanduser(f"~/wheeled_biped_isaaclab/{_USD_DIR}/robot_simple.usd")
# 바퀴 축 관성 (한 개) = 모터 회전자 반사관성 (fix_urdf: 157.33e-7 x 10^2 = 1.573e-3) + 바퀴 자체.
#   CAD: 120 mm 자체 1.82e-4 -> 1.755e-3, 140 mm 자체 2.661e-4 -> 1.839e-3. 140 mm 실측 질량 (고무 16.5 g + 휠 56.0 g, 메시 모양) 2.681e-4 -> 1.841e-3. 사본은 make_wheel_variant 와 같은 식
WHEEL_IZZ = {70: 1.8414e-3, 60: 1.755e-3}.get(_R_MM, 1.625e-3 + 1.3e-4 * (R_WHEEL / 0.060) ** 4)

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


def leg_vel(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """[dh_L, dh_R] [m/s] — joint_vel_like_simple 의 앞 두 칸. 잡음을 바퀴와 따로 주려고 나눴다."""
    return leg_state(env.scene[asset_cfg.name])[1]


def wheel_vel(env: "ManagerBasedRLEnv", asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """[w_L, w_R] [rad/s], +y 규약 — joint_vel_like_simple 의 뒤 두 칸."""
    asset = env.scene[asset_cfg.name]
    wid = asset.find_joints(WHEEL_JOINTS, preserve_order=True)[0]
    return asset.data.joint_vel[:, wid] * torch.tensor(WHEEL_SIGN, device=asset.device)


def leg_torque(env: "ManagerBasedRLEnv", scale: float = 5.0, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """고관절 모터 토크 [L, R] / scale. 부호: + = 다리를 펴는 쪽 (M_SIGN). 실기: 전류 x Kt."""
    asset = env.scene[asset_cfg.name]
    ids = asset.find_joints(LEG_JOINTS, preserve_order=True)[0]
    sgn = torch.tensor([M_SIGN["L"], M_SIGN["R"]], device=asset.device)
    return asset.data.applied_torque[:, ids] * sgn / scale


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


# --- 바퀴 모터 + 축 마찰 (2026-10-03 실측: AK45-10 은 0.4 A 이하 지령에 전혀 안 돈다) ----------------------
class DCMotorFric(DCMotor):
    """DC 모터 + 드라이브 데드밴드: |지령| <= deadband 이면 0 (드라이브가 작은 전류 지령을 무시하는 경우의 모델 —
    0.4 A 이하가 안 도는 원인이 마찰인지 이것인지 아직 모름). 기본 0 = DCMotor 와 같음.
    축 마찰은 여기서 안 한다 — PhysX 관절 마찰 (set_wheel_model) 이 정지 마찰을 솔버 안에서 푼다 (진짜로 붙어 안 돈다)."""

    def __init__(self, cfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        self.deadband = torch.full_like(self.computed_effort, cfg.deadband)

    def compute(self, control_action, joint_pos, joint_vel):
        e = control_action.joint_efforts
        if e is not None:
            control_action.joint_efforts = torch.where(e.abs() <= self.deadband, torch.zeros_like(e), e)
        return super().compute(control_action, joint_pos, joint_vel)


@configclass
class DCMotorFricCfg(DCMotorCfg):
    class_type: type = DCMotorFric
    deadband: float = 0.0    # [N·m] 이하 지령은 0


def set_wheel_model(robot, wheel_ids, fric_static, fric_dyn, viscous=0.0, deadband=0.0, armature=0.0):
    """바퀴 실측 모델 적용 (창·시험 세트·가혹 평가 공통).
    fric_static / fric_dyn [N·m]: PhysX 관절 마찰 (Isaac Sim 5.x: 정지 = 멈춰 있을 때 버티는 최대 토크, 운동 = 도는 동안 일정).
      숫자 또는 (환경, 바퀴) 텐서. 2026-10-03 확인 (scripts/diag_wheel_friction.py): 정지 0.45 에 0.3 N·m -> 안 돎,
      0.6 N·m -> (0.6 - 운동 0.3) / 관성 으로 가속 = N·m 단위가 맞다.
    armature [kg·m²]: 관절에 더하는 관성 (실측 출력축 2.0~2.6e-3 - 링크 izz WHEEL_IZZ)."""
    n, k = robot.num_instances, len(wheel_ids)
    full = lambda x: (torch.as_tensor(x, dtype=torch.float32).expand(n, k).clone() if torch.as_tensor(x).ndim < 2
                      else torch.as_tensor(x, dtype=torch.float32)).to(robot.device)  # noqa: E731
    fs, fd = full(fric_static), full(fric_dyn)
    fd = torch.minimum(fd, fs)                                         # PhysX: 정지 >= 운동 이어야 한다 (어기면 설정 자체를 거부, 2026-10-07)
    robot.write_joint_friction_coefficient_to_sim(fs, fd, full(viscous), joint_ids=wheel_ids)
    robot.actuators["wheels"].deadband[:] = full(deadband)                 # 숫자 또는 (환경, 바퀴) — 로봇마다
    if armature > 0:
        robot.write_joint_armature_to_sim(full(armature), joint_ids=wheel_ids)


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
        "wheels": DCMotorFricCfg(joint_names_expr=[".*_joint_W"], saturation_effort=7.0, effort_limit=7.0,
                                 velocity_limit=18.85, stiffness=0.0, damping=0.0),   # 마찰 기본 0 (set_wheel_model 로)
    },
    soft_joint_pos_limit_factor=1.0,
)


# --- 고관절 토크-속도 모델 (점프·턱 오르기 시험용, 2026-09-26) ---------------------------------------
# IdealPD 는 |tau| <= 9 만 자르고 속도가 오를 때 토크가 줄어드는 걸 모른다 -> 빠른 동작(점프)을 낙관한다.
# DCMotor = 같은 explicit PD + 4 사분면 토크-속도 직선. AK60-6 KV80 @ 24 V: 무부하 = 80 rpm/V x 24 V / 6 = 320 rpm
# = 33.5 rad/s (출력축) [계산, 역기전력 기준 — 실측 전], 피크 9 N·m.
AK60_NO_LOAD_RAD_S = 33.5
CAD_ROBOT_CFG_DCHIP = CAD_ROBOT_CFG.replace(
    actuators={
        **CAD_ROBOT_CFG.actuators,
        "legs": DCMotorCfg(joint_names_expr=[".*_joint_M"], stiffness=60.0, damping=1.5,
                           saturation_effort=9.0, effort_limit=9.0, velocity_limit=AK60_NO_LOAD_RAD_S),
    }
)
