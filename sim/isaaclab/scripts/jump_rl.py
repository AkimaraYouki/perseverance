"""점프 잔차 RL (2026-10-13): 공중 자세 제어 + 착지 뒤 균형 회복을 학습한다.

기본 제어 = scripts/wbctrl.py (실기 제어기 + 점프 상태기계, climb_test.py TUNE = 실기 기본값). 정책은 점프 단계
(RETRACT·EXTRACT·FLY·DESCEND·LAND) 와 착지 뒤 1 s 동안만 보정을 더한다 — 그 밖에선 보정 0 (= 지금 실기 제어기 그대로).
  행동 4 = [다리 높이 보정 L, R (±res_leg m), 바퀴 토크 보정 L, R (±res_wheel N·m)], 보정에도 실기와 같은 지연 (delay_ms) 이 걸린다.
환경 (로봇마다 무작위): 턱 높이 0–h_max (일부는 평지 점프), 접근 속도, 점프 시작 거리 (= 실기 '막힘 → 물러남' 재시도가 정하는 거리),
  고관절 실효 배율 (mit_pos, 0.5–1.0), 질량·무게중심·모터·마찰 오차, IMU 잡음 (wbctrl). 고관절 토크 상한 9 N·m (실기 요청과 같게).
학습:  isaaclab.sh -p scripts/_isaaclab_launch.py scripts/jump_rl.py --num_envs 2048 --iters 1500 --run jr1
평가:  ... scripts/jump_rl.py --eval logs/rsl_rl/wheeled_biped_jump/<run>/model_N.pt --num_envs 512   (정책 없이 기본 제어: --eval base)
"""
import argparse
import collections
import json
import math
import os
import re
import sys
import time
import types as _types

from isaaclab.app import AppLauncher

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def load_tune():
    src = open(os.path.join(HERE, "climb_test.py"), encoding="utf-8").read()
    i = src.index("TUNE = dict(")
    ns = {}
    exec(src[i:src.index("\n)\n", i) + 3], ns)
    return ns["TUNE"]


TUNE = load_tune()
ap = argparse.ArgumentParser()
ap.add_argument("--num_envs", type=int, default=2048)
ap.add_argument("--iters", type=int, default=1500)
ap.add_argument("--run", default="jr")
ap.add_argument("--resume", default=None, help="이어서 학습할 model_N.pt")
ap.add_argument("--eval", default=None, help="평가만: model_N.pt 또는 base (정책 없이 기본 제어)")
ap.add_argument("--eval_episodes", type=int, default=2048)
ap.add_argument("--export", default=None, help="--eval 모델을 이 폴더에 policy.onnx 로 내보내고 끝")
ap.add_argument("--h_min", type=float, default=0.04)
ap.add_argument("--h_max", type=float, default=0.10)
ap.add_argument("--h", type=float, default=None, help="턱 높이 고정 (평가용, 예: 0.06 / 0.08)")
ap.add_argument("--p_flat", type=float, default=0.15, help="평지 점프 비율")
ap.add_argument("--v_min", type=float, default=0.25, help="접근 속도 [m/s]")
ap.add_argument("--v_max", type=float, default=0.45)
ap.add_argument("--edge_min", type=float, default=0.75, help="출발점 ~ 모서리 (평지면 점프 지점 기준) [m]")
ap.add_argument("--edge_max", type=float, default=1.15)
ap.add_argument("--res_leg", type=float, default=0.04)
ap.add_argument("--res_wheel", type=float, default=2.5)
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--hip_eff_min", type=float, default=0.5, help="고관절 실효 배율 범위 (로봇 추정 ~0.6)")
ap.add_argument("--hip_eff_max", type=float, default=1.0)
ap.add_argument("--hip_cap_nm", type=float, default=9.0, help="실기 balance_node hip_pd_tau_max (명령 PD 상한) [N·m]")
ap.add_argument("--hip_robot_cap", type=int, default=1, help="1 = 실기처럼 명령 PD 를 9 N·m 로 자른 뒤 실효 배율 (실제 최대 9 x hip_eff). 0 = 옛 시뮬 (실효 배율 뒤 9 N·m)")
ap.add_argument("--dump", default=None, help="평가 때 판마다 기록 npz (결과, 턱, 이륙 거리·속도·gy, 바퀴 ω, 고관절 실효 ...)")
ap.add_argument("--retry", type=int, default=0, help="1 = 턱 감지 모드 평가: 조종자는 앞으로만, 점프는 wbctrl jump_retry 가 (막힘 -> 후진 -> 가속 -> 점프)")
ap.add_argument("--cma_space", default="jump", choices=("jump", "retry", "all"))
ap.add_argument("--cma", type=int, default=0, help="점프 상태기계 파라미터 CMA-ES 세대 수 (정책 없음, 로봇을 후보 수로 나눠 동시 평가)")
ap.add_argument("--cma_pop", type=int, default=16)
ap.add_argument("--cma_w_bottom", type=float, default=0.5, help="착지 때 다리가 끝까지 눌린 (스프링 바닥) 비율 벌점")
ap.add_argument("--cma_w_hub", type=float, default=0.3, help="허브 하중 평균이 cma_hub_ref 를 넘는 비율 벌점")
ap.add_argument("--cma_w_slip", type=float, default=0.3, help="접지 미끄럼 평균 [m/s] 벌점 (실기 파손 땐 1.4 m/s)")
ap.add_argument("--cma_hub_ref", type=float, default=350.0, help="[N] 옛 기본 파라미터의 착지 허브 하중 중간값 (평지 0.2-0.4 m/s)")
ap.add_argument("--cma_eps", type=int, default=1, help="세대마다 로봇 한 대당 에피소드 (처음 k 판만 센다)")
ap.add_argument("--debug", type=int, default=0, help="로봇 몇 대의 시간 흐름을 0.25 s 마다 출력")
for k, v in TUNE.items():
    if isinstance(v, bool):
        ap.add_argument(f"--{k}", type=lambda x: x.lower() in ("1", "true", "on", "yes"), default=v)
    elif isinstance(v, str):
        ap.add_argument(f"--{k}", default=v)
    elif v is not None:
        ap.add_argument(f"--{k}", type=float, default=v)
if TUNE.get("hip_w0") is None:
    ap.add_argument("--hip_w0", type=float, default=None, help="고관절 무부하 속도 [rad/s] (24 V 33.5, 6S 처짐 21 V 29.3). None = 33.5")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import gymnasium as gym  # noqa: E402
from tensordict import TensorDict  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab.assets import RigidObjectCfg  # noqa: E402
from isaaclab.terrains import TerrainImporterCfg  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance import cad  # noqa: E402

import lqr_vmc  # noqa: E402
import wbctrl  # noqa: E402

P = _types.SimpleNamespace(**{k: getattr(args, k, v) for k, v in TUNE.items()})
P.edge_stop = False
DT = 1.0 / 200.0
JUMP_PH = (wbctrl.RETRACT, wbctrl.EXTRACT, wbctrl.FLY, wbctrl.DESCEND, wbctrl.LAND)
AFTER_LAND_S = 1.0          # 착지 (LAND 끝) 뒤 보정·채점 시간
EP_S = 7.0 if "--retry" not in sys.argv else 22.0   # 턱 감지 모드는 천천히 접근 + 후진 + 재가속
BOX_L, BOX_W, BOX_T = 1.4, 2.0, 0.20

# --- 장면: 평면 + 로봇마다 키네마틱 턱 (높이를 리셋 때마다 바꾼다) -------------------------------------------------
TASK = "Isaac-WheeledBiped-CAD-Rough-Play-v0"
cfg = parse_env_cfg(TASK, device=args.device, num_envs=args.num_envs)
if args.hip == "dc":   # 2026-10-15: 실기 점프 (바퀴 5.6 cm) 보다 시뮬 (14–18 cm) 이 훨씬 높음 -> 고관절 토크-속도 직선 (climb_test 창과 같은 모델)
    cfg.scene.robot = cad.CAD_ROBOT_CFG_DCHIP.replace(prim_path="{ENV_REGEX_NS}/Robot")
    cfg.scene.robot.actuators["legs"].velocity_limit = float(getattr(args, "hip_w0", None) or cad.AK60_NO_LOAD_RAD_S)   # TUNE hip / hip_w0 (climb_test 와 같음, 기본 dc)
cfg.seed = args.seed
cfg.scene.env_spacing = 4.0
cfg.scene.terrain = TerrainImporterCfg(
    prim_path="/World/ground", terrain_type="plane", collision_group=-1,
    physics_material=sim_utils.RigidBodyMaterialCfg(friction_combine_mode="multiply", restitution_combine_mode="multiply",
                                                    static_friction=1.0, dynamic_friction=1.0))
cfg.scene.step = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Step",
    spawn=sim_utils.CuboidCfg(size=(BOX_L, BOX_W, BOX_T), collision_props=sim_utils.CollisionPropertiesCfg(),
                              rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                              mass_props=sim_utils.MassPropertiesCfg(mass=50.0),
                              physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0)),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(2.0, 0.0, -0.5)))
cfg.episode_length_s = 1e6                                                 # 리셋은 여기서 직접
for grp in (cfg.rewards, cfg.curriculum, cfg.terminations):
    for n_ in list(vars(grp)):
        if not n_.startswith("_"):
            setattr(grp, n_, None)
if hasattr(cfg.observations, "critic"):
    cfg.observations.critic = None
if hasattr(cfg.scene, "critic_scanner"):
    cfg.scene.critic_scanner = None
e = cfg.events
for n_ in ("base_mass", "base_com", "push"):
    if hasattr(e, n_):
        setattr(e, n_, None)
e.reset_base.params["pose_range"] = {}
e.reset_base.params["velocity_range"] = {}
_p = cfg.scene.robot.init_state.pos
cfg.scene.robot.init_state.pos = (_p[0], _p[1], _p[2] + P.spawn_z)
cfg.decimation = max(1, round(P.physics_hz / 200.0))
cfg.sim.dt = 1.0 / (200.0 * cfg.decimation)
env = gym.make(TASK, cfg=cfg).unwrapped
dev = env.device
N = env.num_envs
robot, step_obj = env.scene["robot"], env.scene["step"]
cmd_term = env.command_manager.get_term("base_velocity")
cmd_term.pin(True)
cmd_term.cfg.auto_height = cmd_term.cfg.default_height = P.idle_h
env.action_manager.get_term("wheels").cfg.torque_scale = P.wheel_tau_max
env.action_manager.get_term("wheels")._alpha = 1.0
leg_ids = robot.find_joints(cad.LEG_JOINTS, preserve_order=True)[0]
wheel_ids = robot.find_joints(cad.WHEEL_JOINTS, preserve_order=True)[0]
wheel_bodies = robot.find_bodies(["l_wheel", "r_wheel"], preserve_order=True)[0]
hip_sign = torch.tensor([cad.M_SIGN["L"], cad.M_SIGN["R"]], device=dev)
wsign = torch.tensor(cad.WHEEL_SIGN, device=dev)
nonwheel = [i for i in range(robot.num_bodies) if i not in wheel_bodies]
legs_act, wheels_act = robot.actuators["legs"], robot.actuators["wheels"]
R = cad.R_WHEEL
to = lambda x: x.detach().cpu().numpy()  # noqa: E731

env.reset()
# --- 명목 모델 + LQR (harsh_eval 과 같은 계산) -----------------------------------------------------------------
m_nom = robot.root_physx_view.get_masses().clone()[0].to(dev)
m_pend = float(m_nom[nonwheel].sum())
d = robot.data
c0 = (d.body_com_pos_w[0, nonwheel] * m_nom[nonwheel, None]).sum(0) / m_pend
iyy = robot.root_physx_view.get_inertias()[0][:, 4].to(dev)
rel = d.body_com_pos_w[0, nonwheel] - c0
I_pend = float((iyy[nonwheel] + m_nom[nonwheel] * (rel[:, 0] ** 2 + rel[:, 2] ** 2)).sum())
lqr = lqr_vmc.WheelLQR(m_pend, I_pend, float(m_nom[wheel_bodies].sum()), 2 * (cad.WHEEL_IZZ + P.wheel_armature), R,
                       q=(P.lqr_qx, P.lqr_qv, P.lqr_qth, P.lqr_qthd), r=P.lqr_r, table=P.lqr_table or None)

# --- 로봇 오차 (로봇마다, 학습 내내 고정) — harsh_eval 과 같은 분포 + 고관절 실효 배율 ----------------------------------
g = np.random.default_rng(args.seed)
km = 1.0 + g.uniform(-1, 1, N) * P.dr_mass
dxz = g.uniform(-1, 1, (N, 2)) * P.dr_com_cm / 100.0
kv = 1.0 - g.uniform(0, 1, N) * P.dr_motor
kf = 1.0 + g.uniform(-1, 1, (N, 2)) * P.dr_fric
kd_ = 1.0 + g.uniform(-1, 1, (N, 2)) * P.dr_fric_dyn
hip_eff = torch.tensor(g.uniform(args.hip_eff_min, args.hip_eff_max, N), device=dev, dtype=torch.float32)
if args.hip_robot_cap:   # 실기 balance_node hip_pd_tau_max: 명령 kp·err + |ff| <= 9 -> 드라이브 실효 배율 뒤 실제 <= 9 x hip_eff (2026-10-13 실기 안 뜸)
    legs_act.effort_limit[:] = args.hip_cap_nm * hip_eff[:, None]
view = robot.root_physx_view
com_b_nom = view.get_coms().clone()[..., :3].to(dev)
masses, coms = view.get_masses().clone(), view.get_coms().clone()
masses[:, 0] *= torch.tensor(km, dtype=masses.dtype)
masses[:, wheel_bodies] += float(os.environ.get("WB_WHEEL_DM_G", "0")) / 1000.0   # 바퀴 실제 질량 오차 [g] (제어기 명목은 그대로, 2026-10-13 새 바퀴 ±10 g 분석)
coms[:, 0, 0] += torch.tensor(dxz[:, 0], dtype=coms.dtype); coms[:, 0, 2] += torch.tensor(dxz[:, 1], dtype=coms.dtype)  # noqa: E702
view.set_masses(masses, torch.arange(N)); view.set_coms(coms, torch.arange(N))  # noqa: E702
vlim = wheels_act.velocity_limit.clone() if torch.is_tensor(wheels_act.velocity_limit) else torch.full((N, 2), float(wheels_act.velocity_limit))
wheels_act.velocity_limit = vlim.to(dev) * torch.tensor(kv, device=dev, dtype=torch.float32)[:, None]
if hasattr(wheels_act, "_vel_at_effort_lim"):
    wheels_act._vel_at_effort_lim = wheels_act.velocity_limit * (1 + wheels_act.effort_limit / wheels_act._saturation_effort)
motor_est = kv * (1.0 + g.normal(0, 0.02, N))
cad.set_wheel_model(robot, wheel_ids, torch.tensor(P.wheel_fric_nm * kf), torch.tensor(P.wheel_fric_dyn_nm * kd_), P.wheel_visc,
                    torch.tensor(P.wheel_deadband_nm * kf), P.wheel_armature, backlash_deg=P.wheel_backlash_deg,
                    gear_k=P.wheel_gear_k, gear_c=P.wheel_gear_c, dt=float(cfg.sim.dt), tau_gain=getattr(P, 'wheel_tau_gain', 1.0))
mass_true = masses.to(dev)
CTRL = wbctrl.WBController(P, lqr, m_pend, n=N, seed=args.seed + 1000)
origins = env.scene.env_origins.clone()


class JumpEnv:
    """rsl_rl VecEnv: 로봇 N 대, 한 에피소드 = 한 번의 점프 (출발 → 접근 → 점프 → 착지 뒤 1 s 서기)."""

    num_actions = 4

    def __init__(self):
        self.num_envs, self.device, self.cfg = N, dev, {}
        self.max_episode_length = int(EP_S / DT)
        self.episode_length_buf = torch.zeros(N, dtype=torch.long, device=dev)
        self.rng = np.random.default_rng(args.seed + 7)
        self.h = np.zeros(N); self.edge = np.zeros(N); self.dtrig = np.zeros(N); self.vcmd = np.zeros(N)  # noqa: E702
        self.jumped = np.zeros(N, bool); self.landed_t = np.full(N, np.nan); self.t = np.zeros(N)  # noqa: E702
        self.gt = 0.0                                                    # 제어기 전역 시간 (단계 타이머는 이것의 차이)
        self.res = np.zeros((N, 4)); self.resq = collections.deque([np.zeros((N, 4))] * 4, maxlen=4)  # noqa: E702
        self.base = np.zeros((N, 4)); self.x_prev = np.zeros(N)  # noqa: E702
        self.stats = collections.defaultdict(list)
        self.eplog = []; self.prelog = []  # noqa: E702
        self.cnt = np.zeros((N, 4))                                       # 로봇별 success / short / fell / timeout (CMA)
        self.land = np.zeros((N, 4))                                      # CMA: 로봇별 (착지 수, 다리 바닥 닿음 수, 허브 하중 합, 접지 미끄럼 합) — 세는 판만
        self.zb0 = np.full(N, np.nan); self.zbmax = np.zeros(N); self.rise = []   # 몸통 (base) 높이: 점프 시작 때 / 공중 최고 -> 상승 [m]  # noqa: E702
        self.ex_t = np.zeros(N); self.dh_max = np.zeros(N); self.h_last = np.zeros(N)   # EXTRACT 길이, 다리 펴는 속도 최대  # noqa: E702
        self.d_desc = np.zeros(N); self.fwd_fall = []                     # DESCEND 길이 [s], 넘어진 방향 (+ = 앞)  # noqa: E702
        self.gy_to = np.full(N, np.nan); self.vex = np.zeros(N); self.wex = np.zeros(N); self.sat_t = np.zeros(N)   # 이륙 gy, EXTRACT 최대 v·|ω|, 공중 바퀴 포화 시간 (2026-10-14 실기 뒤로 넘어짐)  # noqa: E702
        self.slip_td = np.full(N, np.nan); self.wair = np.zeros(N)        # 접지 순간 |바퀴 둘레 속도 - 몸 속도| [m/s], 공중 바퀴 |ω| 최대  # noqa: E702
        self.td = []; self.hmin = np.full(N, 9.0); self.h_td = np.full(N, np.nan); self.tqmax = np.zeros(N)  # 착지 순간 다리 길이·눌림·고관절 토크  # noqa: E702
        self.fmax = np.zeros(N); self.hub = []                           # 착지 바퀴 축 하중 최대 [N] (바퀴 관절 반력, 2026-10-13 바퀴 파손)  # noqa: E702
        self.zmax = np.zeros(N); self.clear = []                         # 점프 뒤 바퀴 바닥 최고 높이 (지면 기준) -> 떴나  # noqa: E702
        self.ep_n = np.zeros(N, int); self.ep_k = 1 << 30                 # CMA: 로봇마다 처음 ep_k 판만 센다 (빨리 끝나는 넘어짐 쏠림 방지)  # noqa: E702
        self.f = None
        self._reset(np.ones(N, bool))
        self.obs = self._observe()

    # --- 리셋: 로봇 기본 자세 (공중 스폰), 새 턱 높이·거리·속도, 제어기·보정 상태 -----------------------------------------
    def _reset(self, m):
        ids = np.flatnonzero(m)
        if len(ids) == 0:
            return
        idt = torch.tensor(ids, device=dev, dtype=torch.long)
        env._reset_idx(idt)
        n = len(ids)
        flat = self.rng.random(n) < args.p_flat
        h = self.rng.uniform(args.h_min, args.h_max, n) if args.h is None else np.full(n, args.h)
        self.h[ids] = np.where(flat & (args.h is None), 0.0, h)
        self.edge[ids] = self.rng.uniform(args.edge_min, args.edge_max, n)
        self.dtrig[ids] = self.rng.uniform(0.22, 0.40, n)
        self.vcmd[ids] = self.rng.uniform(args.v_min, args.v_max, n)
        pose = torch.zeros(n, 7, device=dev)
        pose[:, 0] = origins[idt, 0] + torch.tensor(self.edge[ids] + BOX_L / 2, device=dev, dtype=torch.float32)
        pose[:, 1] = origins[idt, 1]
        pose[:, 2] = torch.tensor(self.h[ids] - BOX_T / 2, device=dev, dtype=torch.float32)
        pose[:, 3] = 1.0
        step_obj.write_root_pose_to_sim(pose, idt)
        CTRL.reset(m)
        self.jumped[ids] = False; self.landed_t[ids] = np.nan; self.t[ids] = 0.0  # noqa: E702
        self.res[ids] = 0.0
        for q in self.resq:
            q[ids] = 0.0
        self.base[ids] = 0.0
        self.x_prev[ids] = 0.0
        self.episode_length_buf[idt] = 0

    # --- 센서 프레임 (harsh_eval 과 같은 계산) -------------------------------------------------------------------
    def _frame(self):
        d = robot.data
        q = d.root_quat_w
        psi = torch.atan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))
        fwd = torch.stack([torch.cos(psi), torch.sin(psi), torch.zeros_like(psi)], 1)
        lat = torch.stack([-torch.sin(psi), torch.cos(psi), torch.zeros_like(psi)], 1)
        h = cad.leg_state(robot)[0]
        tau = d.applied_torque[:, leg_ids] * hip_sign
        wj = d.joint_vel[:, wheel_ids] * wsign
        wabs = (d.body_ang_vel_w[:, wheel_bodies] * lat[:, None]).sum(-1)
        if wheels_act.backlash > 0:
            wr = wheels_act.omega_r * wsign
            wabs = wabs + (wr - wj); wj = wr  # noqa: E702
        ax = d.body_pos_w[:, wheel_bodies].mean(1)
        c_pos = d.body_link_pos_w[:, nonwheel] + quat_apply(d.body_link_quat_w[:, nonwheel].reshape(-1, 4),
                                                            com_b_nom[:, nonwheel].reshape(-1, 3)).reshape(N, len(nonwheel), 3)
        c_nom = (c_pos * m_nom[None, nonwheel, None]).sum(1) / m_pend
        rb = quat_apply_inverse(q, c_nom - ax)
        c_tru = (d.body_com_pos_w[:, nonwheel] * mass_true[:, nonwheel, None]).sum(1) / mass_true[:, nonwheel].sum(1, keepdim=True)
        r_t = c_tru - ax
        acc = d.body_lin_acc_w[:, 0]
        return wbctrl.Frame(t=self.gt, g_b=to(d.projected_gravity_b), w_b=to(d.root_ang_vel_b), h=to(h), tau_hip=to(tau), w_wheel_joint=to(wj),
                            w_wheel_abs=to(wabs), th_kin=to(torch.atan2(rb[:, 0], rb[:, 2])), l_pend=to(rb.norm(dim=1)),
                            wx=to(ax[:, 0] - origins[:, 0]), wheel_z_min=to(d.body_pos_w[:, wheel_bodies, 2].min(1).values - R),
                            yaw=to(psi), sf=to((acc + torch.tensor([0.0, 0.0, 9.81], device=dev)).norm(dim=1) / 9.81),
                            truth_th=to(torch.atan2((r_t * fwd).sum(1), r_t[:, 2])),
                            truth_v=to((d.body_lin_vel_w[:, wheel_bodies].mean(1) * fwd).sum(1)), motor_scale=motor_est,
                            a_fwd=to((acc * fwd).sum(1)))

    def _active(self):
        """보정이 켜지는 로봇: 점프 단계 또는 착지 뒤 AFTER_LAND_S 안."""
        ph = CTRL.phase
        after = ~np.isnan(self.landed_t) & (self.t - np.nan_to_num(self.landed_t, nan=-9) < AFTER_LAND_S)
        return np.isin(ph, JUMP_PH) | after

    def _observe(self):
        f = self.f = self._frame()
        ph = CTRL.phase
        oh = np.eye(6)[ph]
        tp = np.clip(self.gt - CTRL.t_phase, 0, 1.0)
        tl = np.where(np.isnan(self.landed_t), -1.0, np.clip(self.t - np.nan_to_num(self.landed_t), 0, AFTER_LAND_S))
        pitch = np.arcsin(np.clip(f.g_b[:, 0], -1, 1)); roll = np.arcsin(np.clip(f.g_b[:, 1], -1, 1))  # noqa: E702
        nz = lambda s, *sh: self.rng.normal(0, s, sh if sh else (N,))  # noqa: E731
        dist = np.clip(self.edge - f.wx + nz(0.03), -0.5, 1.5)            # 실기: 막힘 지점·오도메트리로 아는 거리 (잡음 3 cm)
        pol = np.concatenate([
            oh, tp[:, None], tl[:, None], self._active()[:, None].astype(float),
            (pitch + nz(0.005))[:, None], (roll + nz(0.005))[:, None], (f.w_b + nz(0.02, N, 3)) * 0.25,
            (f.h - P.idle_h) / 0.06, f.w_wheel_joint / 19.0, CTRL.vf[:, None], dist[:, None],
            (self.h + nz(0.005))[:, None] * 10.0, self.base, self.res], 1)
        z_top = f.wheel_z_min - self.h
        crit = np.concatenate([pol, f.truth_th[:, None], f.truth_v[:, None], z_top[:, None], (f.wx - self.edge)[:, None],
                               hip_eff.cpu().numpy()[:, None], km[:, None], dxz, kv[:, None]], 1)
        t_ = lambda a: torch.tensor(a, device=dev, dtype=torch.float32)  # noqa: E731
        return TensorDict({"policy": t_(pol), "critic": t_(crit)}, batch_size=[N])

    def get_observations(self):
        return self.obs

    def step(self, actions):
        f = self.f
        a_pol = np.clip(actions.detach().cpu().numpy(), -1.0, 1.0)
        act_on = self._active()
        # 점프 요청: 모서리 앞 dtrig 에서 한 번 (실기 = 막힘 → 물러남 → 다시 달려 그 거리에서)
        want = ~self.jumped & (f.wx >= self.edge - self.dtrig) & (CTRL.phase == wbctrl.DRIVE) & (not args.retry)
        ph_before = CTRL.phase.copy()                                     # 단계 전이는 CTRL.step 안에서 일어난다
        vc = np.where(self.t < 1.0, 0.0, self.vcmd) if args.retry else self.vcmd     # 턱 감지 모드: 첫 1 s 는 서서 안정 (공중 스폰 착지 출렁임)
        a, kp, kd, ffF, info = CTRL.step(f, vc, np.zeros(N), P.idle_h, jump=want)
        started = (ph_before == wbctrl.DRIVE) & (CTRL.phase == wbctrl.RETRACT)
        self.jumped |= started
        self.base = a.copy()
        res = np.where(act_on[:, None], a_pol, 0.0)
        self.resq.append(res)                                             # 보정에도 실기 지연
        r_d = self.resq[max(0, len(self.resq) - 1 - int(round(P.delay_ms / 5.0)))]
        acts = a.copy()
        acts[:, 0:2] += r_d[:, 0:2] * args.res_leg / 0.12
        acts[:, 2:4] += r_d[:, 2:4] * args.res_wheel / P.wheel_tau_max
        wc = P.wheel_cmd_max_nm / P.wheel_tau_max
        acts[:, 2:4] = np.clip(acts[:, 2:4], -wc, wc)
        M = robot.data.joint_pos[:, leg_ids]
        ffj = hip_sign * torch.tensor(ffF, device=dev, dtype=torch.float32)[:, None] * cad.dh_from_M(M).to(torch.float32)
        legs_act.stiffness[:] = torch.tensor(kp, device=dev, dtype=torch.float32)[:, None] * hip_eff[:, None]
        legs_act.damping[:] = torch.tensor(kd, device=dev, dtype=torch.float32)[:, None] * hip_eff[:, None]
        robot.set_joint_effort_target(ffj * hip_eff[:, None], joint_ids=leg_ids)
        cmd_term.set(torch.tensor(self.vcmd, device=dev, dtype=torch.float32), torch.zeros(N, device=dev),
                     torch.full((N,), P.idle_h, device=dev), mode=torch.ones(N, device=dev))
        env.step(torch.tensor(acts, device=dev, dtype=torch.float32))
        self.t += DT
        self.gt += DT
        self.episode_length_buf += 1
        landed_now = (ph_before == wbctrl.LAND) & (CTRL.phase == wbctrl.DRIVE) & np.isnan(self.landed_t)
        self.landed_t[landed_now] = self.t[landed_now]
        # --- 보상 ---------------------------------------------------------------------------------------------
        self.obs = self._observe()
        f = self.f
        pitch_t = f.truth_th
        tilt = np.degrees(np.arccos(np.clip(-f.g_b[:, 2], -1, 1)))
        dx = f.wx - self.x_prev; self.x_prev = f.wx.copy()  # noqa: E702
        wsat = np.maximum(0.0, np.abs(f.w_wheel_joint).max(1) / (18.85 * kv) - 0.85)
        dres = res - self.res; self.res = res  # noqa: E702
        on_top = f.wheel_z_min >= self.h - 0.015
        upright = tilt < 15.0
        rew = (np.where(act_on, -2.0 * pitch_t ** 2 - 0.02 * f.w_b[:, 1] ** 2, 0.0)
               + 2.0 * np.clip(dx, -0.01, 0.01) * (f.wx < self.edge + 0.6)
               - 3.0 * wsat - 0.05 * (res ** 2).sum(1) - 0.2 * (dres ** 2).sum(1))
        after = ~np.isnan(self.landed_t) & (self.t - np.nan_to_num(self.landed_t, nan=9e9) >= AFTER_LAND_S)
        success = after & on_top & upright & (f.wx >= self.edge + 0.05)
        short = after & ~success
        fell = tilt > 50.0
        timeout = self.episode_length_buf.cpu().numpy() >= self.max_episode_length
        rew += 30.0 * success - 15.0 * short - 20.0 * fell + 0.5 * (self.jumped & on_top & (self.h > 0.01) & (f.wx > self.edge))   # 턱 위에 올라가 있으면 매 스텝 (2026-10-13 jr2: jr1 은 덜 뛰어 턱 앞 착지로 수렴)
        done = success | short | fell | timeout
        cm = done & (self.ep_n < self.ep_k)
        if args.dump and done.any():
            oc = np.where(success, 0, np.where(short, 1, np.where(fell, 2, 3)))
            he = hip_eff.cpu().numpy()
            for i in np.flatnonzero(done & ~self.jumped):        # 점프 전에 끝난 판: 결과, 재시도 단계, 시간, 모서리 거리, pitch
                self.prelog.append((oc[i], CTRL.rt_phase[i], self.t[i], self.edge[i] - f.wx[i], f.truth_th[i], self.h[i], CTRL.n_retry[i]))
            for i in np.flatnonzero(done & self.jumped):
                self.eplog.append((oc[i], self.h[i], self.to_d[i], self.to_v[i], self.gy_to[i], self.to_w[i], self.to_p[i], he[i],
                                   self.vex[i], self.wex[i], self.sat_t[i], f.truth_th[i], self.vcmd[i]))
        self.zmax = np.where(np.isin(CTRL.phase, (wbctrl.EXTRACT, wbctrl.FLY, wbctrl.DESCEND)), np.maximum(self.zmax, f.wheel_z_min), self.zmax)
        if done.any():
            self.clear.extend(self.zmax[done & self.jumped].tolist())
        self.zmax[done] = 0.0
        hm = f.h.mean(1)
        td_now = (ph_before == wbctrl.DESCEND) & (CTRL.phase == wbctrl.LAND)          # 접지 판정 순간
        self.h_td = np.where(td_now, hm, self.h_td)
        # 닿기 직전 (바퀴가 아직 지면·턱 윗면 위 3 mm 넘게 떠 있는 마지막 순간) 의 바퀴 둘레 속도 - 몸 속도.
        # 시뮬 강체 접촉은 닿는 1–2 ms 안에 마찰로 맞춰 버려서, 접지 판정 (고관절 토크) 순간 값은 작게 나온다 (2026-10-13 CMA 1: 0.08 m/s vs 실기 1.4)
        surf = np.where(f.wx > self.edge, self.h, 0.0)
        up = np.isin(CTRL.phase, (wbctrl.FLY, wbctrl.DESCEND)) & (f.wheel_z_min - surf > 0.003)
        self.slip_td = np.where(up, np.abs(f.w_wheel_abs.mean(1) * R - f.truth_v), self.slip_td)
        self.wair = np.where(np.isin(CTRL.phase, (wbctrl.FLY, wbctrl.DESCEND)), np.maximum(self.wair, np.abs(f.w_wheel_joint).max(1)), self.wair)
        to_now = (ph_before == wbctrl.EXTRACT) & (CTRL.phase == wbctrl.FLY)
        self.gy_to = np.where(to_now, f.w_b[:, 1], self.gy_to)
        self.to_d = np.where(to_now, self.edge - f.wx, getattr(self, "to_d", np.full(N, np.nan)))      # 이륙 때 바퀴축 ~ 모서리 [m]
        self.to_v = np.where(to_now, f.truth_v, getattr(self, "to_v", np.full(N, np.nan)))
        self.to_w = np.where(to_now, np.abs(f.w_wheel_joint).max(1), getattr(self, "to_w", np.full(N, np.nan)))
        self.to_p = np.where(to_now, f.truth_th, getattr(self, "to_p", np.full(N, np.nan)))
        ex = CTRL.phase == wbctrl.EXTRACT
        self.vex = np.where(ex, np.maximum(self.vex, f.truth_v), self.vex)
        self.wex = np.where(ex, np.maximum(self.wex, np.abs(f.w_wheel_joint).max(1)), self.wex)
        zb = robot.data.root_pos_w[:, 2].cpu().numpy()
        st_ = (ph_before == wbctrl.DRIVE) & (CTRL.phase == wbctrl.RETRACT)
        self.zb0 = np.where(st_, zb, self.zb0)
        self.zbmax = np.where(np.isin(CTRL.phase, (wbctrl.FLY, wbctrl.DESCEND)), np.maximum(self.zbmax, zb), self.zbmax)
        if done.any():
            ok_ = done & ~np.isnan(self.zb0) & (self.zbmax > 0)
            self.rise.extend((self.zbmax[ok_] - self.zb0[ok_]).tolist())
        self.zb0[done] = np.nan; self.zbmax[done] = 0.0
        self.d_desc += (CTRL.phase == wbctrl.DESCEND) * DT
        self.ex_t += ex * DT
        self.dh_max = np.where(ex, np.maximum(self.dh_max, (hm - self.h_last) / DT), self.dh_max); self.h_last = hm  # noqa: E702
        self.sat_t += (np.isin(CTRL.phase, (wbctrl.FLY, wbctrl.DESCEND)) & (np.abs(f.w_wheel_joint).max(1) > 17.0)) * DT
        inl = CTRL.phase == wbctrl.LAND
        self.hmin = np.where(inl, np.minimum(self.hmin, hm), self.hmin)
        self.tqmax = np.where(inl | td_now, np.maximum(self.tqmax, np.abs(f.tau_hip).max(1)), self.tqmax)
        for i in np.flatnonzero(done & ~np.isnan(self.h_td)):
            self.td.append((self.h_td[i], self.hmin[i], self.tqmax[i], self.slip_td[i], self.wair[i], self.gy_to[i], self.vex[i], self.wex[i], self.sat_t[i], self.d_desc[i], self.ex_t[i], self.dh_max[i]))
        lm = cm & ~np.isnan(self.h_td)
        self.land[lm, 0] += 1
        self.land[lm, 1] += self.hmin[lm] < wbctrl.H_MIN + 0.005                      # 다리를 끝까지 눌러 스프링이 바닥에 닿음
        self.land[lm, 3] += self.slip_td[lm]
        self.fwd_fall.extend((f.truth_th[done & fell] > 0).tolist()); self.d_desc[done] = 0.0; self.ex_t[done] = 0.0; self.dh_max[done] = 0.0  # noqa: E702
        self.gy_to[done] = np.nan; self.vex[done] = 0.0; self.wex[done] = 0.0; self.sat_t[done] = 0.0  # noqa: E702
        self.h_td[done] = np.nan; self.hmin[done] = 9.0; self.tqmax[done] = 0.0; self.slip_td[done] = np.nan; self.wair[done] = 0.0  # noqa: E702
        try:   # 정강이 -> 바퀴 관절 반력 = 허브가 받는 힘. 공중·착지·착지 뒤 0.5 s 의 최대 (물리 스텝 중 마지막 값만 보므로 실제 최대보다 낮을 수 있음)
            fw = robot.data.body_incoming_joint_wrench_b[:, wheel_bodies, :3].norm(dim=-1).max(1).values.cpu().numpy()
            win = np.isin(CTRL.phase, (wbctrl.FLY, wbctrl.DESCEND, wbctrl.LAND)) | (~np.isnan(self.landed_t) & (self.t - np.nan_to_num(self.landed_t) < 0.5))
            self.fmax = np.where(win, np.maximum(self.fmax, fw), self.fmax)
            if done.any():
                self.hub.extend(self.fmax[done & self.jumped].tolist())
            self.land[lm, 2] += self.fmax[lm]
            self.fmax[done] = 0.0
        except Exception as e:   # noqa: BLE001
            if not getattr(self, "_hub_warn", False):
                print("[허브 하중] 못 잼:", e, flush=True); self._hub_warn = True  # noqa: E702
        self.cnt[cm] += np.stack([success, short, fell, timeout & ~(success | short | fell)], 1)[cm]
        self.ep_n += done
        for k_, v_ in (("success", success), ("short", short), ("fell", fell), ("timeout", timeout & ~(success | short | fell))):
            if done.any():
                self.stats[k_].append(float(v_[done].sum()))
        extras = {"time_outs": torch.tensor(timeout & ~(success | short | fell), device=dev),
                  "log": {"/jump/success": float(success.sum()), "/jump/fell": float(fell.sum()), "/jump/short": float(short.sum())}}
        if args.debug and int(round(self.gt / DT)) % 50 == 0:
            for i in range(args.debug):
                print(f"  t{self.t[i]:5.2f} #{i} ph{CTRL.phase[i]} x{f.wx[i]:+.2f} edge{self.edge[i]:.2f} h{self.h[i]:.2f} v{f.truth_v[i]:+.2f} "
                      f"vf{CTRL.vf[i]:+.2f} tilt{tilt[i]:4.0f} zmin{f.wheel_z_min[i]:+.3f} jumped{int(self.jumped[i])} land{self.landed_t[i]:.2f}", flush=True)
            if self.gt > 8.0:
                raise SystemExit(0)
        if done.any():
            self._reset(done)
            self.obs = self._observe()
        t_ = lambda a: torch.tensor(a, device=dev, dtype=torch.float32)  # noqa: E731
        return self.obs, t_(rew), torch.tensor(done, device=dev, dtype=torch.long), extras


venv = JumpEnv()
print(f"[점프 RL] 로봇 {N} 대, 관측 policy {venv.obs['policy'].shape[1]} critic {venv.obs['critic'].shape[1]}, 행동 4, "
      f"턱 {args.h_min:.2f}–{args.h_max:.2f} m (평지 {args.p_flat:.0%}), 고관절 실효 0.5–1.0, 지연 {P.delay_ms:.0f} ms", flush=True)

# --- 점프 상태기계 CMA-ES: 결과 = 로봇 tune.* 값 그대로 (2026-10-13, RL 잔차는 0 % 로 실패) ------------------------
CMA_SPACE = [   # 이름, 하한, 상한 (climb_test TUNE 기본값에서 시작). 2026-10-14 CMA 7 에서 상·하한에 붙은 것 넓힘
    ("t_retract", 0.12, 0.40), ("retract_lean", -3.0, 12.0), ("extract_pitch", -8.0, 12.0), ("t_tuck", 0.04, 0.25),
    ("air_kp", 0.0, 60.0), ("air_kd", 0.0, 10.0), ("air_pitch", -15.0, 4.0), ("land_pitch", -6.0, 10.0),
    ("land_kp", 8.0, 60.0), ("land_kd", 0.2, 4.0), ("land_s", 0.10, 0.70), ("h_land", 0.13, 0.2425),
    ("contact_tau", 0.3, 5.0), ("leg_kp", 30.0, 120.0), ("leg_kd", 0.2, 4.0),
    ("spin_match_k", 0.0, 0.8), ("air_w_max", 6.0, 19.0)]   # touch_dh 는 CMA 9 에서 나빠짐 (2026-10-15) -> 탐색 안 함                      # 2026-10-13 실기 바퀴 파손 뒤 추가
if args.cma_space == "all":     # 점프 + 턱 감지 + 신전 바퀴 속도 상한 (2026-10-20)
    CMA_SPACE = CMA_SPACE + [("retry_approach_v", 0.06, 0.2), ("retry_block_s", 0.05, 0.5), ("retry_back_m", 0.15, 0.9),
                             ("retry_v", 0.15, 0.3), ("trigger", 0.08, 0.5), ("retry_stall_frac", 0.15, 0.8), ("ex_w_max", 5.0, 19.0)]
    # 사용자: 점프 전 속도 0.2–0.3 m/s -> 명령 범위로만 (retry_vmax 속도 보호 상한 0.3 은 넘어짐 14 -> 50 %, 2026-10-20 비교)
if args.cma_space == "retry":   # 턱 감지 모드 파라미터만 (점프 묶음은 고정)
    CMA_SPACE = [("retry_approach_v", 0.06, 0.25), ("retry_block_s", 0.15, 0.6), ("retry_back_m", 0.15, 0.9),
                 ("retry_v", 0.2, 0.6), ("trigger", 0.08, 0.5)]


def cma_eval(cands, k_eps):
    """후보 K 개를 로봇 N 대에 번갈아 (i % K) 나눠 동시 평가, 로봇마다 처음 k_eps 판. 반환 (K, 4) 비율 success/short/fell/timeout, 판 수."""
    K = len(cands); grp = np.arange(N) % K
    for j, (k, _, _) in enumerate(CMA_SPACE):
        setattr(P, k, np.array([c[j] for c in cands])[grp])
    venv._reset(np.ones(N, bool)); venv.obs = venv._observe(); venv.cnt[:] = 0; venv.land[:] = 0; venv.ep_n[:] = 0; venv.ep_k = k_eps  # noqa: E702
    zero = torch.zeros(N, 4, device=dev)
    for s in range(k_eps * venv.max_episode_length + 10):
        venv.step(zero)
        if s % 50 == 49 and (venv.ep_n >= k_eps).all():
            break
    c = np.stack([venv.cnt[grp == j].sum(0) for j in range(K)])
    la = np.stack([venv.land[grp == j].sum(0) for j in range(K)])
    land = np.stack([la[:, 1] / np.maximum(1, la[:, 0]), la[:, 2] / np.maximum(1, la[:, 0]), la[:, 3] / np.maximum(1, la[:, 0])], 1)   # 바닥 닿음 비율, 허브 하중 평균 [N], 접지 미끄럼 평균 [m/s]
    return c / np.maximum(1, c.sum(1, keepdims=True)), c.sum(1), land


def cma_score(r, land):
    # 성공 − 0.7 × 넘어짐 (턱 앞 착지 short 는 0) − 착지 벌점 (2026-10-13: 1 차 CMA 는 다리를 접은 채 착지 -> 매번 바닥 닿음, 허브 하중 1.4 배 -> 바퀴 파손 위험)
    # 2026-10-13 실기 파손 로그: 접지 미끄럼 1.4 m/s (바퀴 -0.9 vs 몸 +0.5) -> 미끄럼 벌점
    return (r[:, 0] - 0.7 * r[:, 2] - args.cma_w_bottom * land[:, 0] - args.cma_w_hub * np.maximum(0.0, land[:, 1] / args.cma_hub_ref - 1.0)
            - args.cma_w_slip * land[:, 2])


if args.cma:
    lo = np.array([b[1] for b in CMA_SPACE]); hi = np.array([b[2] for b in CMA_SPACE])  # noqa: E702
    x0 = np.array([float(getattr(P, k)) for k, _, _ in CMA_SPACE])
    dec = lambda z: lo + (hi - lo) * np.clip(z, 0, 1)                    # noqa: E731
    D, lam = len(CMA_SPACE), args.cma_pop - 1                             # 마지막 칸 = 지금 평균 (기록용)
    mu = lam // 2; w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1)); w /= w.sum(); mueff = 1 / (w ** 2).sum()  # noqa: E702
    cc, cs = (4 + mueff / D) / (D + 4 + 2 * mueff / D), (mueff + 2) / (D + mueff + 5)
    c1 = 2 / ((D + 1.3) ** 2 + mueff); cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((D + 2) ** 2 + mueff))
    ds = 1 + 2 * max(0, np.sqrt((mueff - 1) / (D + 1)) - 1) + cs; chiN = np.sqrt(D) * (1 - 1 / (4 * D) + 1 / (21 * D * D))  # noqa: E702
    m = (x0 - lo) / (hi - lo); sig = 0.15; C = np.eye(D); pc = np.zeros(D); ps = np.zeros(D)  # noqa: E702
    out = os.path.join(os.path.dirname(HERE), "logs", "jump_cma", time.strftime("%Y-%m-%d_%H-%M-%S") + "_" + args.run)
    os.makedirs(out, exist_ok=True)
    best = (-9.0, None, None); rng = np.random.default_rng(args.seed)  # noqa: E702
    print(f"[점프 CMA] 파라미터 {D}, 후보 {lam}+평균, 세대 {args.cma}, 로봇 {N} -> {out}", flush=True)
    for gen in range(args.cma):
        t0 = time.time()
        ev, B = np.linalg.eigh(C); Dd = np.sqrt(np.maximum(ev, 1e-12))  # noqa: E702
        zs = rng.standard_normal((lam, D)); ys = zs @ (B * Dd).T; xs = m + sig * ys  # noqa: E702
        r, ne, la = cma_eval([dec(x) for x in xs] + [dec(m)], args.cma_eps)
        sc = cma_score(r, la)
        if sc[-1] > best[0]:
            best = (sc[-1], dec(m), r[-1], la[-1])
        o = np.argsort(-sc[:lam]); yw = (w[:, None] * ys[o[:mu]]).sum(0)  # noqa: E702
        m = np.clip(m + sig * yw, 0, 1)
        Ci = B @ np.diag(1 / Dd) @ B.T
        ps = (1 - cs) * ps + np.sqrt(cs * (2 - cs) * mueff) * Ci @ yw
        hs = np.linalg.norm(ps) / np.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) < (1.4 + 2 / (D + 1)) * chiN
        pc = (1 - cc) * pc + hs * np.sqrt(cc * (2 - cc) * mueff) * yw
        C = (1 - c1 - cmu) * C + c1 * (np.outer(pc, pc) + (1 - hs) * cc * (2 - cc) * C) + cmu * (w[:, None] * ys[o[:mu]]).T @ ys[o[:mu]]
        sig = min(0.5, sig * np.exp((cs / ds) * (np.linalg.norm(ps) / chiN - 1)))
        rec = dict(gen=gen, sigma=round(float(sig), 4), mean_score=round(float(sc[-1]), 3), mean_rates=r[-1].round(3).tolist(),
                   top_score=round(float(sc[o[0]]), 3), mean_bottom=round(float(la[-1, 0]), 3), mean_hub=round(float(la[-1, 1]), 1), mean_slip=round(float(la[-1, 2]), 3), eps=int(ne.min()), s=round(time.time() - t0, 1),
                   mean=dict(zip([k for k, _, _ in CMA_SPACE], dec(m).round(4).tolist())))
        with open(os.path.join(out, "gens.jsonl"), "a") as fh:
            fh.write(json.dumps(rec) + "\n")
        print(f"[CMA {gen:3d}] 평균후보 성공 {r[-1, 0]:.0%} 넘어짐 {r[-1, 2]:.0%} 바닥 {la[-1, 0]:.0%} 허브 {la[-1, 1]:.0f} N 미끄럼 {la[-1, 2]:.2f} 점수 {sc[-1]:+.3f} | 1등 {sc[o[0]]:+.3f} | "
              f"sigma {sig:.3f} | {ne.min()} 판 {time.time() - t0:.0f} s", flush=True)
    # 검증: 기본 vs 최고 (평지 / 6 cm / 8 cm), 각 조건 로봇 절반씩
    bx = dict(zip([k for k, _, _ in CMA_SPACE], best[1].round(4).tolist()))
    json.dump(dict(best=bx, score=best[0], rates=best[2].tolist(), land=best[3].tolist()), open(os.path.join(out, "best.json"), "w"), indent=1)
    for nm, h_, pf in (("평지", None, 1.0), ("턱 6 cm", 0.06, 0.0), ("턱 8 cm", 0.08, 0.0)):
        args.h, args.p_flat = h_, pf
        r, ne, la = cma_eval([x0, best[1]], 3)
        line = (f"[검증 {nm}] 기본 성공 {r[0, 0]:.1%} 넘어짐 {r[0, 2]:.1%} 바닥 {la[0, 0]:.0%} 허브 {la[0, 1]:.0f} N 미끄럼 {la[0, 2]:.2f} | "
                f"최고 성공 {r[1, 0]:.1%} 넘어짐 {r[1, 2]:.1%} 바닥 {la[1, 0]:.0%} 허브 {la[1, 1]:.0f} N 미끄럼 {la[1, 2]:.2f} ({ne.min()} 판)")
        print(line, flush=True)
        with open(os.path.join(out, "verify.txt"), "a") as fh:
            fh.write(line + "\n")
    print("[점프 CMA] 끝 best =", bx, flush=True)
    os._exit(0)

if args.eval:
    pol = None
    if args.eval != "base":
        from rsl_rl.runners import OnPolicyRunner
        from wheeled_biped_isaaclab.agents.rsl_rl_ppo_cfg import ResidualPPORunnerCfg
        tc = ResidualPPORunnerCfg(); tc.obs_groups = {"policy": ["policy"], "critic": ["critic"]}  # noqa: E702
        from isaaclab_rl.rsl_rl import handle_deprecated_rsl_rl_cfg
        import importlib.metadata as _md
        tc = handle_deprecated_rsl_rl_cfg(tc, _md.version("rsl-rl-lib"))
        runner = OnPolicyRunner(venv, tc.to_dict(), log_dir=None, device=dev)
        runner.load(args.eval)
        pol = runner.get_inference_policy(device=dev)
        if args.export:
            from isaaclab_rl.rsl_rl import export_policy_as_onnx
            pn = runner.alg.policy
            norm = getattr(pn, "actor_obs_normalizer", None)
            os.makedirs(args.export, exist_ok=True)
            export_policy_as_onnx(pn, normalizer=norm, path=args.export, filename="policy.onnx")
            print(f"[내보냄] {args.export}/policy.onnx", flush=True)
            os._exit(0)
    venv.stats.clear()
    n_done, t0 = 0, time.time()
    with torch.inference_mode():
        obs = venv.get_observations()
        while n_done < args.eval_episodes:
            act = pol(obs) if pol is not None else torch.zeros(N, 4, device=dev)
            obs, _, done, _ = venv.step(act)
            n_done += int(done.sum())
    tot = {k: sum(v) for k, v in venv.stats.items()}
    s = sum(tot.values())
    print(f"[점프 평가] {args.eval} | 턱 {args.h if args.h is not None else f'{args.h_min}-{args.h_max}'} | 에피소드 {s:.0f} | "
          + "  ".join(f"{k} {v / max(1, s):.1%}" for k, v in sorted(tot.items())) + f" | {time.time() - t0:.0f} s"
          + (f" | 바퀴 최고 {np.median(venv.clear) * 100:.1f} cm (중간값, 95 % {np.percentile(venv.clear, 95) * 100:.1f}, 최대 {np.max(venv.clear) * 100:.1f}), 1 cm 넘게 뜸 {np.mean(np.array(venv.clear) > 0.01):.0%}" if venv.clear else "")
          + (f" | 몸통 상승 (점프 시작 대비) 중간 {np.median(venv.rise) * 100:.1f} cm, 95 % {np.percentile(venv.rise, 95) * 100:.1f}, 최대 {np.max(venv.rise) * 100:.1f}" if venv.rise else "")
          + (f" | 착지 허브 하중 중간 {np.median(venv.hub):.0f} N, 95 % {np.percentile(venv.hub, 95):.0f} N, 최대 {np.max(venv.hub):.0f} N"
             f" (정지 때 바퀴 하나 {(float(m_nom.sum()) - float(m_nom[wheel_bodies].sum())) * 9.81 / 2:.0f} N)" if venv.hub else "")
          + ((lambda a: f" | 접지 순간 다리 {np.median(a[:, 0]) * 1000:.0f} mm (h_land {np.mean(P.h_land) * 1000:.0f}, 최소 {wbctrl.H_MIN * 1000:.0f}), "
             f"착지 중 최저 {np.median(a[:, 1]) * 1000:.0f} mm, 눌림 {np.median(a[:, 0] - a[:, 1]) * 1000:.0f} mm, 바닥 닿음 {np.mean(a[:, 1] < wbctrl.H_MIN + 0.005):.0%}, "
             f"고관절 최대 {np.median(a[:, 2]):.1f} N·m, 닿기 직전 미끄럼 중간 {np.median(a[:, 3]):.2f} m/s (95 % {np.percentile(a[:, 3], 95):.2f}), "
             f"공중 바퀴 |ω| 최대 중간 {np.median(a[:, 4]):.1f} rad/s, 17 넘음 {np.mean(a[:, 4] > 17):.0%}, "
             f"이륙 gy 중간 {np.nanmedian(a[:, 5]):+.2f} rad/s (5 % {np.nanpercentile(a[:, 5], 5):+.2f}), EXTRACT 최대 v {np.median(a[:, 6]):.2f} m/s · |ω| {np.median(a[:, 7]):.1f}, "
             f"공중 포화 {np.median(a[:, 8]) * 1000:.0f} ms, DESCEND {np.median(a[:, 9]) * 1000:.0f} ms, EXTRACT {np.median(a[:, 10]) * 1000:.0f} ms (펴는 속도 최대 {np.median(a[:, 11]):.2f} m/s)"
             + (f", 넘어짐 중 앞으로 {np.mean(venv.fwd_fall):.0%}" if venv.fwd_fall else ""))(np.array(venv.td)) if venv.td else ""), flush=True)
    if args.dump:
        np.save(os.path.expanduser(args.dump), np.array(venv.eplog, float))
        np.save(os.path.expanduser(args.dump).replace(".npy", "_pre.npy"), np.array(venv.prelog, float))
        print("[dump]", args.dump, len(venv.eplog), "판 (결과 0성공 1짧음 2넘어짐 3시간, h, to_d, to_v, gy_to, to_w, to_pitch, hip_eff, ex_vmax, ex_wmax, sat_t, 끝 pitch, vcmd)", flush=True)
    os._exit(0)                                                          # app.close() 가 가끔 멈춤

from isaaclab_rl.rsl_rl import handle_deprecated_rsl_rl_cfg  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
import importlib.metadata as _md  # noqa: E402
from wheeled_biped_isaaclab.agents.rsl_rl_ppo_cfg import ResidualPPORunnerCfg  # noqa: E402

tc = ResidualPPORunnerCfg()
tc.experiment_name = "wheeled_biped_jump"
tc.num_steps_per_env = 48
tc.max_iterations = args.iters
tc.save_interval = 50
tc.policy.init_noise_std = 0.5
tc.obs_groups = {"policy": ["policy"], "critic": ["critic"]}
tc = handle_deprecated_rsl_rl_cfg(tc, _md.version("rsl-rl-lib"))
log_dir = os.path.join(os.path.dirname(HERE), "logs", "rsl_rl", "wheeled_biped_jump", time.strftime("%Y-%m-%d_%H-%M-%S") + "_" + args.run)
os.makedirs(log_dir, exist_ok=True)
runner = OnPolicyRunner(venv, tc.to_dict(), log_dir=log_dir, device=dev)
if args.resume:
    runner.load(args.resume)
print(f"[점프 RL] 학습 {args.iters} iter -> {log_dir}", flush=True)
runner.learn(num_learning_iterations=args.iters, init_at_random_ep_len=False)
print("[점프 RL] 학습 끝", flush=True)
os._exit(0)
