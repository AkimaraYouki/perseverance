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
WHEEL_SELF_IZZ = {70: 2.6814e-4, 60: 1.82e-4}.get(_R_MM, 2.68e-4)   # 바퀴 자체 (고무 + 휠, 회전자 제외) — 백래시 모델용

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



class HipHostP:
    """실기 고관절 구조 (2026-10-08, 실기 balance_node hip_mode mit): P 항 kp·(M* − M) 은 200 Hz 호스트가 **지연된 피드백**으로
    계산해 t_ff 로 보내고, D 항만 드라이브가 빠르게 (여기선 물리 주기) 한다. 시뮬 기본 (IdealPD 가 P·D 모두 물리 주기, 지연 없음) 과 다르다.
    도구 (climb_test·robust_suite·harsh_eval) 가 TUNE hip_host_p 일 때 매 제어 스텝 apply() 를 부른다."""

    def __init__(self):
        import collections
        self.hist = collections.deque(maxlen=16)

    def reset(self):
        self.hist.clear()

    def apply(self, robot, leg_ids, legs_act, a_legs, h_ref, kp, kd, ff_joint, delay_steps, leg_scale=0.12, tau_max=None):
        """a_legs (N,2) 다리 행동 (wbctrl, 지연 큐 지난 것), h_ref (N,), kp/kd (N,), ff_joint (N,2) 자중 보상 관절 토크."""
        M = robot.data.joint_pos[:, leg_ids]
        self.hist.append(M.clone())
        M_fb = self.hist[max(0, len(self.hist) - 1 - int(delay_steps))]
        hj = torch.clamp(h_ref[:, None] + torch.clamp(a_legs, -1.0, 1.0) * leg_scale, 0.1225, 0.2425)
        tau_p = kp[:, None] * (M_from_hj(hj) - M_fb)
        legs_act.stiffness[:] = 0.0
        legs_act.damping[:] = kd[:, None].expand_as(legs_act.damping) if legs_act.damping.dim() == 2 else kd
        tau = ff_joint + tau_p
        if tau_max:                                      # 실기 balance_node: P 항 + 자중 FF 를 current_limit x Kt 로 자름 (kd 는 드라이브가 따로)
            tau = torch.clamp(tau, -tau_max, tau_max)
        robot.set_joint_effort_target(tau.to(torch.float32), joint_ids=leg_ids)

# --- 바퀴 모터 + 축 마찰 (2026-10-03 실측: AK45-10 은 0.4 A 이하 지령에 전혀 안 돈다) ----------------------
class DCMotorFric(DCMotor):
    """DC 모터 + 드라이브 데드밴드 (+ 선택: 감속기 백래시).
    데드밴드: |지령| <= deadband 이면 0 (서보 펌웨어의 0.5 A 문턱, 2026-10-06 판정). 기본 0.
    축 마찰은 여기서 안 한다 — PhysX 관절 마찰 (set_wheel_model) 이 정지 마찰을 솔버 안에서 푼다.
    백래시 (2026-10-08, backlash > 0): 회전자 (관성 rotor_J, 출력축 환산) 를 바퀴와 분리해 이 안에서 적분한다.
      토크 공백 모델: 맞물린 채 미는 동안은 τ_m 이 그대로 전달 (기어 강성 무한, 회전자 관성은 바퀴 링크 izz 에 그대로).
      토크 방향이 바뀌면 회전자가 유격 (출력축 b) 을 혼자 τ_m 으로 건너는 동안 바퀴엔 0 — 반전마다 수 ms 의 토크 공백.
      반대편 끝에 닿으면 비탄성으로 맞물림. 엔코더는 회전자 쪽이라 도구가 omega_r 를 바퀴 속도로 읽는다.
      (회전자를 떼어 강성 k 로 잇는 모델은 바퀴 링크가 가벼워져 2.5 ms 물리 스텝에서 발산 — 2026-10-08 98 Hz 발진)"""

    def __init__(self, cfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        self.deadband = torch.full_like(self.computed_effort, cfg.deadband)
        self.backlash = 0.0                              # [rad, 출력축] 전체 유격 (0 = 끔)
        self.gear_k, self.gear_c, self.rotor_J, self.dt, self.n_sub = 300.0, 0.05, 2.0e-3, 1.0 / 400, 10
        self.omega_r = torch.zeros_like(self.computed_effort)    # 회전자 속도 (관절 좌표, 출력축 환산)
        self.dphi = torch.zeros_like(self.computed_effort)       # 회전자 - 바퀴 각 (유격 포함)

    def reset(self, env_ids):
        super().reset(env_ids)
        self.omega_r[env_ids] = 0.0
        self.dphi[env_ids] = 0.0

    def compute(self, control_action, joint_pos, joint_vel):
        e = control_action.joint_efforts
        if e is not None:
            control_action.joint_efforts = torch.where(e.abs() <= self.deadband, torch.zeros_like(e), e)
        if self.backlash <= 0:
            return super().compute(control_action, joint_pos, joint_vel)
        out = super().compute(control_action, joint_pos, joint_vel)
        tau_m = self.applied_effort.clone()
        hb = 0.5 * self.backlash
        # 맞물림: 회전자가 유격 끝 (|dphi| = hb) 에서 그 방향으로 밀면 토크 전달 (회전자 관성은 바퀴 링크 izz 에 그대로)
        push = ((self.dphi >= hb - 1e-9) & (tau_m > 0)) | ((self.dphi <= -hb + 1e-9) & (tau_m < 0))
        # 유격 안 (또는 반대로 당김): 바퀴엔 0, 회전자 혼자 τ_m 으로 가속해 유격을 건넌다
        w_free = self.omega_r + self.dt * tau_m / self.rotor_J
        self.omega_r = torch.where(push, joint_vel, w_free)
        self.dphi = torch.where(push, self.dphi, (self.dphi + self.dt * (self.omega_r - joint_vel)).clamp(-hb, hb))
        hit = ~push & (self.dphi.abs() >= hb - 1e-9)                      # 반대편 끝에 닿음 = 다음 스텝부터 맞물림 (비탄성)
        self.omega_r = torch.where(hit, joint_vel, self.omega_r)
        tau = torch.where(push, tau_m, torch.zeros_like(tau_m))
        self.applied_effort = tau
        out.joint_efforts = tau
        return out


@configclass
class DCMotorFricCfg(DCMotorCfg):
    class_type: type = DCMotorFric
    deadband: float = 0.0    # [N·m] 이하 지령은 0


def set_wheel_model(robot, wheel_ids, fric_static, fric_dyn, viscous=0.0, deadband=0.0, armature=0.0,
                    backlash_deg=0.0, gear_k=300.0, gear_c=0.05, dt=1.0 / 400):
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
    wa = robot.actuators["wheels"]
    if backlash_deg > 0:                                                   # 반전 토크 공백 모델 (DCMotorFric)
        wa.backlash, wa.dt = math.radians(backlash_deg), dt
        wa.rotor_J = (WHEEL_IZZ - WHEEL_SELF_IZZ) + armature               # 유격을 건너는 회전자 (출력축 환산)
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
