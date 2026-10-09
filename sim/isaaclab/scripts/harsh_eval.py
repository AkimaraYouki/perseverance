"""가혹 평가: 실기 제어기 wbctrl (창·pv robust 와 같은 코드, 로봇 N 대 배열) 로 로봇 수천 대 x 20 s. **지형만 다르다**
(2026-10-03 사용자: "지형만 다르고 제일 정확한 제어기 하나만"). 예전엔 GPU 복제 제어기(tasks/balance/residual.py)를 썼다.

로봇 쪽 (climb_test.py TUNE 그대로, pv robust 와 같음): 제어기·게인, IMU 잡음·치우침, 바퀴 엔코더 잡음, 제어 지연 + 지터,
  몸통 질량·무게중심·바퀴 모터 한계 오차 (로봇마다), 바퀴 축 마찰·관성 (로봇 실측 2026-10-03). --harsh 배율이 잡음·오차에 곱해진다.
  넘어짐 = 50 deg 넘게 기욺 (pv robust 와 같음), 넘어지면 힘을 뺀다.
환경 쪽 (이 평가만): 지형 (난이도 행 0~9, 열마다 종류), 지면 마찰 μ (--mu, 기본 0.5~1.0), 밀기 4~8 s 마다 ±0.3 m/s (--no_push 로 끔).
명령: course = 출발 방향으로 --vx (불연속 지형은 --vx_slow), 방향 유지 = TUNE heading_kp. random = 환경 명령 (후진·회전·정지).
결과: 출발 지형별 성공률 + 95 % 신뢰구간 (Wilson), 난이도별, 넘어진 곳 지형 기준 (머문 시간당 넘어짐 -> 20 s 환산).
아직 없음 (residual.py 에만 있던 실기 조건, 공통 센서 모델로 옮길 것): IMU 샘플 주기·지연, 모터별 CAN 지연·프레임 손실.

    pv harsh [--num_envs 4096 --harsh 1.5 --level 9 --suite rough --fric_comp_nm 0.25 (TUNE 값 덮어쓰기)]
"""
import argparse
import json
import math
import os
import types as _types

from isaaclab.app import AppLauncher

HERE = os.path.dirname(os.path.abspath(__file__))


def load_tune():
    src = open(os.path.join(HERE, "climb_test.py"), encoding="utf-8").read()
    i = src.index("TUNE = dict(")
    ns = {}
    exec(src[i:src.index("\n)\n", i) + 3], ns)
    return ns["TUNE"]


TUNE = load_tune()
ap = argparse.ArgumentParser()
ap.add_argument("--num_envs", type=int, default=4096)
ap.add_argument("--seconds", type=float, default=20.0)
ap.add_argument("--harsh", type=float, default=1.0, help="로봇 잡음·오차 배율 (1 = TUNE 그대로) + 밀기·지면 마찰 하한")
ap.add_argument("--mu", type=float, nargs=2, default=None, metavar=("MIN", "MAX"), help="지면 마찰 범위 (없으면 하한 1 - 0.5 x 배율)")
ap.add_argument("--level", type=int, default=None, help="난이도 행 고정 (없으면 0~9 고르게)")
ap.add_argument("--cmd", choices=("random", "course"), default="course")
ap.add_argument("--vx", type=float, default=0.4)
ap.add_argument("--vx_slow", type=float, default=0.4, help="불연속 지형 (--slow) 에서의 속도 [m/s] (course)")
ap.add_argument("--slow", default="blocks,step_down", help="천천히 달릴 불연속 지형 (쉼표, 빈 문자열 = 없음)")
ap.add_argument("--yaw0", action="store_true", help="모두 +x 로 출발 (삼각형 길을 정면으로). 없으면 방향 무작위 (비스듬히)")
ap.add_argument("--no_push", action="store_true")
ap.add_argument("--tag", default="")
ap.add_argument("--seed", type=int, default=1, help="로봇 무작위 (전후 비교는 같은 시드로)")
ap.add_argument("--terrains", default=None, help="이 지형만 (쉼표), 예: oneside")
ap.add_argument("--suite", choices=("all", "rough", "slope"), default="all",
                help="all = 대회 지형 포함 10 종, rough = 자동 주행 험지 12 종, slope = 평평한 직선 경사로 (오르막·내리막)")
ap.add_argument("--slope_max", type=float, default=1.0, help="--suite slope: 난이도 9 의 기울기 (1.0 = 45 deg)")
for k, v in TUNE.items():                                              # TUNE 값 덮어쓰기 (pv robust 와 같은 문법)
    if any(a.dest == k for a in ap._actions):
        continue
    if isinstance(v, bool):
        ap.add_argument(f"--{k}", type=lambda x: x.lower() in ("1", "true", "on", "yes"), default=v)
    elif isinstance(v, str) or v is None:
        ap.add_argument(f"--{k}", default=v)
    else:
        ap.add_argument(f"--{k}", type=float, default=v)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import gymnasium as gym  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab.terrains import TerrainGeneratorCfg  # noqa: E402,F401
from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg  # noqa: E402
from isaaclab.terrains.height_field.utils import height_field_to_mesh  # noqa: E402
from isaaclab.terrains.trimesh.mesh_terrains_cfg import MeshPlaneTerrainCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance import terrain as T  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance import cad  # noqa: E402

import lqr_vmc  # noqa: E402
import terrain_gen  # noqa: E402
import wbctrl  # noqa: E402

H = args.harsh
P = _types.SimpleNamespace(**{k: (getattr(args, k) if k != "seconds" else v) for k, v in TUNE.items()})
for k in ("imu_tilt_noise_deg", "imu_tilt_bias_deg", "imu_gyro_noise", "enc_vel_noise", "imu_acc_noise", "dr_mass", "dr_com_cm", "dr_fric", "dr_fric_dyn"):
    setattr(P, k, getattr(P, k) * H)
P.dr_motor = min(0.6, P.dr_motor * H)

_cnt = [0]


@height_field_to_mesh
def _stones(difficulty, c):
    nx, ny = int(c.size[0] / c.horizontal_scale), int(c.size[1] / c.horizontal_scale)
    h = c.h_range[0] + difficulty * (c.h_range[1] - c.h_range[0])
    _cnt[0] += 1
    # oneside: 0.2 m 차선 (바퀴 간격) 하나 건너 하나에만 돌 -> 한쪽 바퀴만 돌 위. 돌은 자르지 않음
    #   (차선 가장자리를 깎으면 12 cm 돌에서 옆면이 62 deg 가 되어 비스듬히 지나면 못 넘는 벽이 됨)
    z = terrain_gen.make_heights("lane_stones" if c.kind == "oneside" else c.kind, nx, ny, c.horizontal_scale, h, 0,
                                 seed=1000 + _cnt[0])
    return np.rint(z / c.vertical_scale).astype(np.int16)


@height_field_to_mesh
def _ramp(difficulty, c):
    """평평한 직선 경사로: 칸 가운데(출발) 앞 1 m 부터 +x 로 기울기 slope 인 평면 (y 방향 기울기 0). down 이면 내리막."""
    nx, ny = int(c.size[0] / c.horizontal_scale), int(c.size[1] / c.horizontal_scale)
    slope = c.slope_range[0] + difficulty * (c.slope_range[1] - c.slope_range[0])
    x = np.arange(nx) * c.horizontal_scale
    z = slope * np.clip(x - (c.size[0] / 2 + 1.0), 0.0, None) * (-1.0 if c.down else 1.0)
    return np.rint(np.repeat(z[:, None], ny, 1) / c.vertical_scale).astype(np.int16)


@configclass
class RampCfg(HfTerrainBaseCfg):
    function = _ramp
    slope_range: tuple = (0.0, 1.0)
    down: bool = False


@configclass
class StonesCfg(HfTerrainBaseCfg):
    function = _stones
    kind: str = "stones"
    h_range: tuple = (0.03, 0.12)


R = T.ROUGH_TERRAINS_CFG.sub_terrains
EVAL_TERRAINS = T.ROUGH_TERRAINS_CFG.replace(
    horizontal_scale=0.05, num_rows=10, num_cols=9, curriculum=True,      # 열 = 종류 하나씩 (돌이 0.1 m 칸이면 뭉개짐)
    sub_terrains={
        "flat": MeshPlaneTerrainCfg(proportion=0.1),
        "stones": StonesCfg(proportion=0.1, kind="stones", h_range=(0.03, 0.12), **T._HF),
        "oneside": StonesCfg(proportion=0.1, kind="oneside", h_range=(0.03, 0.12), **T._HF),
        "ridges_cad": T.StaggeredRidgesTerrainCfg(proportion=0.1, height_range=(0.02, 0.09), base=0.5, period=0.5, lane_w=0.5, **T._HF),
        "ridges_narrow": T.StaggeredRidgesTerrainCfg(proportion=0.1, height_range=(0.02, 0.09), base=0.35, period=0.6, lane_w=0.2, **T._HF),
        # 가운데에서 바깥으로 달리므로: 역피라미드(R["slope_down"]) = 오르막, 피라미드(R["slope_up"]) = 내리막 (2026-09-27 전엔 이름이 반대였음)
        "slope_up": R["slope_down"].replace(proportion=0.1, slope_range=(0.0, 0.30)),
        "slope_down": R["slope_up"].replace(proportion=0.1, slope_range=(0.0, 0.30)),
        "step_down": T.MeshPyramidStairsTerrainCfg(proportion=0.1, step_height_range=(0.02, 0.09), step_width=1.0,
                                                   platform_width=2.0, border_width=0.0),
        # bumps (흩어진 네모 턱) 뺌 — 사용자 2026-10-03: "맨날 걸려 넘어지던 네모난 둔턱 빼"
        "gravel": R["gravel"].replace(proportion=0.1),
    },
)

if args.suite == "rough":                # 자동 주행 험지 (사용자: 삼각형길은 조종으로, 자갈·파도길은 자동으로 -> 험지 주행 안정성 우선)
    EVAL_TERRAINS = EVAL_TERRAINS.replace(num_cols=11, sub_terrains={
        "flat": MeshPlaneTerrainCfg(proportion=1.0),
        "stones": StonesCfg(proportion=1.0, kind="stones", h_range=(0.03, 0.12), **T._HF),
        "oneside": StonesCfg(proportion=1.0, kind="oneside", h_range=(0.03, 0.12), **T._HF),
        "gravel": R["gravel"].replace(proportion=1.0),                  # 10 cm 칸 무작위 ±1~3 cm
        "rough": R["rough"].replace(proportion=1.0),                    # 25 cm 칸 0.5~3.5 cm
        "rugged": R["rugged"].replace(proportion=1.0),                  # 2 m 기복 + 40 cm 요철 + 자갈
        "wave_long": R["wave_long"].replace(proportion=1.0),            # 파도 0~10 cm, 4 개
        "wave_short": R["wave_short"].replace(proportion=1.0),          # 파도 0~6 cm, 8 개
        # bumps (1~4 cm 네모 턱 0.3~1 m) 뺌 — 사용자 2026-10-03
        "blocks": StonesCfg(proportion=1.0, kind="gravel", h_range=(0.01, 0.04), **T._HF),   # 10 cm 블록 0~1~4 cm, 수직 모서리 (영상에서 넘어진 지형)
        # 가운데에서 바깥으로 달리므로: 역피라미드(R["slope_down"]) = 오르막, 피라미드(R["slope_up"]) = 내리막 (2026-09-27 전엔 이름이 반대였음)
        "slope_up": R["slope_down"].replace(proportion=1.0, slope_range=(0.0, 0.30)),
        "slope_down": R["slope_up"].replace(proportion=1.0, slope_range=(0.0, 0.30)),
    })
if args.suite == "slope":               # 가운데 평대(2 m)에서 바깥으로 달린다. 역피라미드 = 오르막, 피라미드 = 내리막
    # 평평한 직선 경사로 (+x). 피라미드 경사는 축에서 조금만 벗어나도 옆 기울기가 커서 로봇이 경사 아래로 돌아 옆으로 달림
    #   (0.4 m/s 10~15 deg 에서 꼭대기 도달 35 %, 방향 틀어짐 중앙 36 deg) -> --yaw0 과 함께 쓴다
    EVAL_TERRAINS = EVAL_TERRAINS.replace(num_cols=2, sub_terrains={
        "uphill": RampCfg(proportion=1.0, slope_range=(0.0, args.slope_max), down=False, border_width=0.0),
        "downhill": RampCfg(proportion=1.0, slope_range=(0.0, args.slope_max), down=True, border_width=0.0),
    })
if args.terrains:
    keep = args.terrains.split(",")
    EVAL_TERRAINS.sub_terrains = {k_: v_.replace(proportion=1.0 / len(keep)) for k_, v_ in EVAL_TERRAINS.sub_terrains.items() if k_ in keep}
TASK = "Isaac-WheeledBiped-CAD-Rough-Play-v0"                      # pv robust 와 같은 환경 (다리 위치 + 바퀴 토크 행동)
cfg = parse_env_cfg(TASK, device=args.device, num_envs=args.num_envs)
cfg.scene.terrain.terrain_generator = EVAL_TERRAINS
cfg.seed = args.seed
cfg.scene.terrain.max_init_terrain_level = 9 if args.level is None else args.level
cfg.episode_length_s = args.seconds + 10.0
for grp in (cfg.rewards, cfg.curriculum, cfg.terminations):            # 학습용 항목 없음, 넘어짐은 직접 센다 (pv robust 와 같음)
    for n_ in list(vars(grp)):
        if not n_.startswith("_"):
            setattr(grp, n_, None)
if hasattr(cfg.observations, "critic"):
    cfg.observations.critic = None
if hasattr(cfg.scene, "critic_scanner"):
    cfg.scene.critic_scanner = None
e = cfg.events
e.base_mass = None; e.base_com = None                                  # 로봇 오차는 아래에서 TUNE 으로 (pv robust 와 같음)  # noqa: E702
mu_r = tuple(args.mu) if args.mu else (max(0.2, 1.0 - 0.5 * H), 1.0)
from isaaclab.managers import EventTermCfg as _Ev, SceneEntityCfg as _Ent  # noqa: E402
import isaaclab.envs.mdp as _mdp  # noqa: E402
# 환경 조건 (Play 환경은 꺼 둔다 -> 학습 환경과 같은 정의로 다시 켬): 지면 마찰 (로봇마다), 밀기
e.wheel_friction = _Ev(func=_mdp.randomize_rigid_body_material, mode="startup",
                       params={"asset_cfg": _Ent("robot", body_names=".*"), "static_friction_range": mu_r, "dynamic_friction_range": mu_r,
                               "restitution_range": (0.0, 0.0), "num_buckets": 64, "make_consistent": True})
e.push = None if args.no_push else _Ev(func=_mdp.push_by_setting_velocity, mode="interval", interval_range_s=(4.0, 8.0),
                                         params={"velocity_range": {"x": (-0.3 * H, 0.3 * H), "y": (-0.3 * H, 0.3 * H)}})
e.reset_base.params["pose_range"] = {"x": (-0.3, 0.3), "y": (-0.3, 0.3), "yaw": (0.0, 0.0) if args.yaw0 or args.suite == "slope" else (-3.14, 3.14)}
if args.suite == "slope":
    e.reset_base.params["pose_range"]["x"] = (0.0, 0.0)
e.reset_base.params["velocity_range"] = {}
_p = cfg.scene.robot.init_state.pos
cfg.scene.robot.init_state.pos = (_p[0], _p[1], _p[2] + P.spawn_z)    # 공중 스폰 (pv robust 와 같음)
cfg.decimation = max(1, round(P.physics_hz / 200.0))
cfg.sim.dt = 1.0 / (200.0 * cfg.decimation)
env = gym.make(TASK, cfg=cfg).unwrapped
dev = env.device
N = env.num_envs
robot = env.scene["robot"]
cmd_term = env.command_manager.get_term("base_velocity")
if args.cmd == "course":
    cmd_term.pin(True)
cmd_term.cfg.auto_height = cmd_term.cfg.default_height = P.idle_h
env.action_manager.get_term("wheels").cfg.torque_scale = P.wheel_tau_max   # 바퀴 행동 = 토크 / wheel_tau_max
env.action_manager.get_term("wheels")._alpha = 1.0                          # 바퀴 20 Hz 필터는 wbctrl 안 (wheel_lpf_hz)
leg_ids = robot.find_joints(cad.LEG_JOINTS, preserve_order=True)[0]
print(f"[모델] {cad.USD_PATH}  총질량 {float(robot.data.default_mass[0].sum()):.3f} kg", flush=True)
wheel_ids = robot.find_joints(cad.WHEEL_JOINTS, preserve_order=True)[0]
wheel_bodies = robot.find_bodies(["l_wheel", "r_wheel"], preserve_order=True)[0]
hip_sign = torch.tensor([cad.M_SIGN["L"], cad.M_SIGN["R"]], device=dev)
wsign = torch.tensor(cad.WHEEL_SIGN, device=dev)
nonwheel = [i for i in range(robot.num_bodies) if i not in wheel_bodies]
legs_act, wheels_act = robot.actuators["legs"], robot.actuators["wheels"]
HIP = cad.HipHostP()
R = cad.R_WHEEL

# --- 명목 모델 (제어기가 믿는 값) + LQR — pv robust 와 같은 계산 ----------------------------------------------
obs, _ = env.reset()
mass_nom = robot.root_physx_view.get_masses().clone()
m_nom = mass_nom[0].to(dev)
m_pend = float(m_nom[nonwheel].sum())
d = robot.data
c0 = (d.body_com_pos_w[0, nonwheel] * m_nom[nonwheel, None]).sum(0) / m_pend
iyy = robot.root_physx_view.get_inertias()[0][:, 4].to(dev)
rel = d.body_com_pos_w[0, nonwheel] - c0
I_pend = float((iyy[nonwheel] + m_nom[nonwheel] * (rel[:, 0] ** 2 + rel[:, 2] ** 2)).sum())
lqr = lqr_vmc.WheelLQR(m_pend, I_pend, float(m_nom[wheel_bodies].sum()), 2 * (cad.WHEEL_IZZ + P.wheel_armature), R,
                       q=(P.lqr_qx, P.lqr_qv, P.lqr_qth, P.lqr_qthd), r=P.lqr_r, table=P.lqr_table or None)

# --- 로봇 오차 (로봇마다, pv robust 와 같은 분포) ---------------------------------------------------------------
g = np.random.default_rng(args.seed)
km = 1.0 + g.uniform(-1, 1, N) * P.dr_mass
dxz = g.uniform(-1, 1, (N, 2)) * P.dr_com_cm / 100.0
kv = 1.0 - g.uniform(0, 1, N) * P.dr_motor
kf = 1.0 + g.uniform(-1, 1, (N, 2)) * P.dr_fric                        # 정지 마찰·데드밴드
view = robot.root_physx_view
com_b_nom = view.get_coms().clone()[..., :3].to(dev)                # 모델 오차 전 링크 좌표 무게중심 (명목, 제어기용)
masses, coms = view.get_masses().clone(), view.get_coms().clone()
masses[:, 0] *= torch.tensor(km, dtype=masses.dtype)
coms[:, 0, 0] += torch.tensor(dxz[:, 0], dtype=coms.dtype); coms[:, 0, 2] += torch.tensor(dxz[:, 1], dtype=coms.dtype)  # noqa: E702
idx = torch.arange(N, device="cpu")
view.set_masses(masses, idx); view.set_coms(coms, idx)  # noqa: E702
vlim = wheels_act.velocity_limit.clone() if torch.is_tensor(wheels_act.velocity_limit) else torch.full((N, 2), float(wheels_act.velocity_limit))
wheels_act.velocity_limit = (vlim.to(dev) * torch.tensor(kv, device=dev, dtype=torch.float32)[:, None])
if hasattr(wheels_act, "_vel_at_effort_lim"):
    wheels_act._vel_at_effort_lim = wheels_act.velocity_limit * (1 + wheels_act.effort_limit / wheels_act._saturation_effort)
motor_est = kv * (1.0 + g.normal(0, 0.02, N))                          # 전압으로 추정한 모터 한계 (오차 2 %)
kd_ = 1.0 + g.uniform(-1, 1, (N, 2)) * P.dr_fric_dyn                   # 운동 마찰
cad.set_wheel_model(robot, wheel_ids, torch.tensor(P.wheel_fric_nm * kf), torch.tensor(P.wheel_fric_dyn_nm * kd_),
                    P.wheel_visc, torch.tensor(P.wheel_deadband_nm * kf), P.wheel_armature, backlash_deg=getattr(P, 'wheel_backlash_deg', 0.0), gear_k=getattr(P, 'wheel_gear_k', 300.0), gear_c=getattr(P, 'wheel_gear_c', 0.05), dt=float(cfg.sim.dt))
mass_true = masses.to(dev)
CTRL = wbctrl.WBController(P, lqr, m_pend, n=N, seed=args.seed + 1000)

# --- 지형 칸 ----------------------------------------------------------------------------------------------------
t_ = env.scene.terrain
if args.level is not None:
    t_.terrain_levels[:] = args.level
    t_.env_origins[:] = t_.terrain_origins[t_.terrain_levels, t_.terrain_types]
    obs, _ = env.reset()
types, levels = t_.terrain_types.clone(), t_.terrain_levels.clone()
names = list(EVAL_TERRAINS.sub_terrains.keys())
props = np.array([s.proportion for s in EVAL_TERRAINS.sub_terrains.values()])
cum = np.cumsum(props / props.sum())
col_name = [names[int(np.searchsorted(cum, cidx / EVAL_TERRAINS.num_cols + 0.001, side="right"))] for cidx in range(EVAL_TERRAINS.num_cols)]
slow_set = [x for x in args.slow.split(",") if x] if args.cmd == "course" else []
slow_np = np.array([col_name[int(ty)] in slow_set for ty in types.tolist()])
vx_course = np.where(slow_np, args.vx_slow, args.vx)
nr, nc = t_.terrain_origins.shape[:2]
org = t_.terrain_origins.reshape(-1, 3)[:, :2]
origins = env.scene.env_origins.clone()
home = origins[:, :2].clone()


def yaw_of(q):
    return torch.atan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))


fell = np.zeros(N, bool)
fall_t = np.full(N, np.nan)
done = np.zeros(N, bool)                                                # 첫 에피소드가 끝났나 (넘어짐 또는 칸 완주)
yaw0 = None
yaw_dev = np.zeros(N)
expo = np.zeros(nc); slow_t = np.zeros(nc); fall_at = np.zeros(nc)  # noqa: E702
RB = 200
ring = np.zeros((N, RB, 5))                                            # 최근 1 s [vx 명령, wz 명령, 바퀴 속도/한계, 들림, 기울기 deg]
snap = np.zeros((N, 5))
left_any = np.zeros(N, bool); left_t = np.zeros(N)  # noqa: E702
out_n = 0
dt = env.step_dt
to = lambda x: x.detach().cpu().numpy()  # noqa: E731
with torch.inference_mode():
    for k in range(int(args.seconds / dt)):
        t = k * dt
        d = robot.data
        q = d.root_quat_w
        psi = yaw_of(q)
        if yaw0 is None:
            yaw0 = psi.clone()
        fwd = torch.stack([torch.cos(psi), torch.sin(psi), torch.zeros_like(psi)], 1)
        lat = torch.stack([-torch.sin(psi), torch.cos(psi), torch.zeros_like(psi)], 1)
        h = cad.leg_state(robot)[0]
        tau = d.applied_torque[:, leg_ids] * hip_sign
        wj = d.joint_vel[:, wheel_ids] * wsign
        wabs = (d.body_ang_vel_w[:, wheel_bodies] * lat[:, None]).sum(-1)
        if wheels_act.backlash > 0:                                 # 엔코더는 회전자 쪽 (감속기 백래시 앞)
            wr = wheels_act.omega_r * wsign
            wabs = wabs + (wr - wj); wj = wr  # noqa: E702
        ax = d.body_pos_w[:, wheel_bodies].mean(1)
        # 명목 무게중심 (모델 오차 전 링크 좌표 COM 을 지금 링크 자세로) — 실기는 명목 표만 안다. 2026-10-07 전에는 오차가 반영된
        # body_com_pos_w 를 써서 제어기가 실제 무게중심 (±2 cm 오차 포함) 을 알고 있었다
        c_pos = d.body_link_pos_w[:, nonwheel] + quat_apply(d.body_link_quat_w[:, nonwheel].reshape(-1, 4),
                                                            com_b_nom[:, nonwheel].reshape(-1, 3)).reshape(N, len(nonwheel), 3)
        c_nom = (c_pos * m_nom[None, nonwheel, None]).sum(1) / m_pend
        rb = quat_apply_inverse(q, c_nom - ax)
        c_tru = (d.body_com_pos_w[:, nonwheel] * mass_true[:, nonwheel, None]).sum(1) / mass_true[:, nonwheel].sum(1, keepdim=True)
        r_t = c_tru - ax
        acc = d.body_lin_acc_w[:, 0]
        dpsi = torch.atan2(torch.sin(psi - yaw0), torch.cos(psi - yaw0))
        f = wbctrl.Frame(t=t, g_b=to(d.projected_gravity_b), w_b=to(d.root_ang_vel_b), h=to(h), tau_hip=to(tau), w_wheel_joint=to(wj),
                         w_wheel_abs=to(wabs), th_kin=to(torch.atan2(rb[:, 0], rb[:, 2])), l_pend=to(rb.norm(dim=1)),
                         wx=to(ax[:, 0] - origins[:, 0]), wheel_z_min=to(d.body_pos_w[:, wheel_bodies, 2].min(1).values - R), yaw=to(dpsi),
                         sf=to((acc + torch.tensor([0.0, 0.0, 9.81], device=dev)).norm(dim=1) / 9.81),
                         truth_th=to(torch.atan2((r_t * fwd).sum(1), r_t[:, 2])), truth_v=to((d.body_lin_vel_w[:, wheel_bodies].mean(1) * fwd).sum(1)),
                         motor_scale=motor_est, a_fwd=to((acc * fwd).sum(1)))
        if args.cmd == "course":                                           # 출발 방향으로 직진, 방향 유지 (pv robust 와 같은 식)
            vx = vx_course
            wz = np.clip(-P.heading_kp * f.yaw, -1.0, 1.0)
        else:
            cm = to(env.command_manager.get_command("base_velocity"))
            vx, wz = cm[:, 0], cm[:, 1]
        a, kp, kd, ffF, info = CTRL.step(f, vx, wz, P.idle_h)
        alive = ~fell
        acts = np.where(alive[:, None], a, 0.0)
        ff = np.where(alive, ffF, 0.0)
        cmd_term.set(torch.tensor(vx, device=dev, dtype=torch.float32), torch.zeros(N, device=dev), torch.full((N,), P.idle_h, device=dev),
                     mode=torch.ones(N, device=dev)) if args.cmd == "course" else None
        M = d.joint_pos[:, leg_ids]
        ffj = hip_sign * torch.tensor(ff, device=dev, dtype=torch.float32)[:, None] * cad.dh_from_M(M).to(torch.float32)
        kp_a, kd_a = np.where(alive, kp, 0.0), np.where(alive, kd, 0.0)
        if getattr(P, "hip_host_p", False):                         # 실기 고관절 구조: P 는 200 Hz 호스트 (지연 피드백), D 만 드라이브
            HIP.apply(robot, leg_ids, legs_act, torch.tensor(acts[:, 0:2], device=dev, dtype=torch.float32),
                      torch.full((N,), float(P.idle_h), device=dev), torch.tensor(kp_a, device=dev, dtype=torch.float32),
                      torch.tensor(kd_a, device=dev, dtype=torch.float32), ffj, round(P.delay_ms / 5.0), tau_max=P.hip_cmd_max_nm)
        else:
            legs_act.stiffness[:] = torch.tensor(kp_a, device=dev, dtype=torch.float32)[:, None] * P.hip_gain_eff   # mit_pos: PD 가 드라이브 안 (물리 주기)
            legs_act.damping[:] = torch.tensor(kd_a, device=dev, dtype=torch.float32)[:, None] * P.hip_gain_eff
            robot.set_joint_effort_target(ffj * P.hip_gain_eff, joint_ids=leg_ids)
        here = to(torch.cdist(d.root_pos_w[:, :2], org).argmin(1) % nc)    # 지금 서 있는 칸의 종류 (열)
        wfr = np.abs(f.w_wheel_joint).max(1) / (18.85 * kv)
        tilt = np.degrees(np.arccos(np.clip(-f.g_b[:, 2], -1, 1)))
        ring[:, k % RB] = np.stack([vx * np.ones(N), wz, wfr, CTRL.lift_on.astype(float), tilt], 1)
        yaw_dev = np.where(done, yaw_dev, np.maximum(yaw_dev, np.abs(f.yaw)))
        obs, _, _, _, _ = env.step(torch.tensor(acts, device=dev, dtype=torch.float32))
        live = ~done
        expo += np.bincount(here[live], minlength=nc) * dt
        slow_t += np.bincount(here[live & CTRL.dbg.get("bump", np.zeros(N, bool))], minlength=nc) * dt
        g_new = to(robot.data.projected_gravity_b)
        f_ = live & (np.degrees(np.arccos(np.clip(-g_new[:, 2], -1, 1))) > 50)          # 넘어짐 = 50 deg (pv robust 와 같음)
        fall_at += np.bincount(here[f_], minlength=nc)
        fall_t[f_] = (k + 1) * dt
        if f_.any():                        # 넘어지기 1.0~0.3 s 전 (넘어지는 순간은 바퀴가 따라 돌아 늘 포화라 원인이 아님)
            b = ring[f_][:, [(k - j) % RB for j in range(60, RB)]]
            snap[f_] = np.stack([b[:, 40, 0], b[:, 40, 1], b[:, :, 2].max(1), b[:, :, 3].max(1), b[:, 40, 4]], 1)
        left = live & (to((robot.data.root_pos_w[:, :2] - home).abs().max(1).values) > 3.7) & ~f_
        out_n += int(left.sum())
        left_t[left] = (k + 1) * dt
        left_any |= left
        done |= left                        # 자기 칸을 벗어남 = 그 칸 완주 (이웃 칸 벽에 부딪히는 것은 세지 않음)
        fell |= f_
        done |= f_
        if k % 100 == 99 and bool(done.all()):
            break


def wilson(s, n, z=1.96):
    if n == 0:
        return (float("nan"),) * 3
    p = s / n
    den = 1 + z * z / n
    ctr = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, max(0.0, ctr - half), min(1.0, ctr + half)


rows = {}
lv_np = levels.cpu().numpy()
for i in range(N):
    n_ = col_name[int(types[i])]
    rows.setdefault(n_, [0, 0]); rows[n_][0] += int(not fell[i]); rows[n_][1] += 1
title = (f"[가혹 평가] 제어기 wbctrl (실기 규격, 창·pv robust 와 같은 코드) | {N} 대 x {args.seconds:.0f} s | 로봇 조건 TUNE x{H} "
         f"(바퀴 마찰 {P.wheel_fric_nm:.2f}/{P.wheel_fric_dyn_nm:.2f} N·m, 보상 {P.fric_comp_nm:.2f}) | 지면 마찰 {mu_r[0]:.2f}~{mu_r[1]:.2f} | "
         f"지연 {P.delay_ms:.0f}+{P.jitter_ms:.0f} ms | 난이도 {args.level if args.level is not None else '0~9'} | "
         f"명령 {args.cmd}{f' {args.vx} m/s' if args.cmd == 'course' else ''}{f' (불연속 {args.vx_slow} m/s)' if slow_set else ''}"
         f"{' 정면' if args.yaw0 else ''}{' | 밀기 없음' if args.no_push else ''} {args.tag}")
print("\n" + title)
print(f"  {'지형':14s} {'성공':>11s}   {'성공률':>7s}  95% 신뢰구간")
out = {}
for n_ in names:
    if n_ not in rows:
        continue
    s_, c_ = rows[n_]
    p, lo, hi = wilson(s_, c_)
    out[n_] = dict(ok=s_, n=c_, p=p, lo=lo, hi=hi)
    print(f"  {n_:14s} {s_:5d}/{c_:<5d}   {100*p:6.1f} %  [{100*lo:5.1f}, {100*hi:5.1f}]")
S = int((~fell).sum())
p, lo, hi = wilson(S, N)
print(f"  {'합계':14s} {S:5d}/{N:<5d}   {100*p:6.1f} %  [{100*lo:5.1f}, {100*hi:5.1f}]", flush=True)
print("  난이도별: " + "  ".join(
    f"{lv}:{100*float((~fell[lv_np == lv]).mean()):.0f}%" for lv in range(nr) if int((lv_np == lv).sum()) > 0))
if args.suite == "slope":
    tp_np = types.cpu().numpy()
    print("  지형 x 난이도 (난이도 L = 기울기 %.2f x [L/10, (L+1)/10) ):" % args.slope_max)
    for j, n_ in enumerate(col_name):
        print(f"    {n_}")
        for lv in range(nr):
            m_ = (tp_np == j) & (lv_np == lv)
            if int(m_.sum()):
                lo_a = math.degrees(math.atan(args.slope_max * lv / nr)); hi_a = math.degrees(math.atan(args.slope_max * (lv + 1) / nr))  # noqa: E702
                la = left_any[m_]; lt = left_t[m_][la]  # noqa: E702
                sp = f"  완주 평균 {3.6 * 3.7 / float(np.median(lt)):4.2f} km/h (수평, 중앙값)" if len(lt) else ""
                print(f"       {lo_a:4.1f}~{hi_a:4.1f}deg 안넘어짐 {100*float((~fell[m_]).mean()):5.1f}% 완주 {100*float(la.mean()):5.1f}%{sp}")
yd = np.degrees(yaw_dev)
print(f"  출발 방향에서 벗어난 최대 각: 중앙 {np.median(yd):.0f} deg, 90 % {np.percentile(yd, 90):.0f} deg, 45 deg 넘음 {100*np.mean(yd > 45):.1f} %")
ft = fall_t[~np.isnan(fall_t)]
if len(ft):
    print(f"  넘어진 시각: 2 s 안 {int((ft < 2).sum())}, 2~5 s {int(((ft >= 2) & (ft < 5)).sum())}, 5 s 뒤 {int((ft >= 5).sum())}")
F = snap[fell]
print(f"  칸 완주 (벗어남) {out_n}/{N}")
if len(F):
    print(f"  넘어지기 1~0.3 s 전 ({len(F)}): 바퀴 속도 한계 90 %↑ {100*np.mean(F[:, 2] > 0.9):.0f} %, 회전 명령 |wz|>1.5 {100*np.mean(np.abs(F[:, 1]) > 1.5):.0f} %, "
          f"속도 명령 |vx|>0.6 {100*np.mean(np.abs(F[:, 0]) > 0.6):.0f} %, 정지 명령 {100*np.mean((np.abs(F[:, 0]) < 0.05) & (np.abs(F[:, 1]) < 0.05)):.0f} %, 들림 상태 {100*np.mean(F[:, 3] > 0.5):.0f} %")
print("  넘어진 곳 기준 (지형: 넘어짐 / 머문 로봇-분 -> 20 s 환산 성공률)")
for j in range(nc):
    m_ = float(expo[j]) / 60.0
    if m_ <= 0:
        continue
    lam = float(fall_at[j]) / (m_ * 60.0)
    sfr = float(slow_t[j]) / max(1e-9, float(expo[j]))
    out.setdefault(col_name[j], {})["at"] = dict(falls=int(fall_at[j]), robot_min=m_, p20=math.exp(-lam * 20), slow_frac=sfr)
    print(f"    {col_name[j]:14s} {int(fall_at[j]):4d} / {m_:6.0f} 분  -> {100*math.exp(-lam*20):5.1f} %" + (f"   턱 감속 {100*sfr:4.0f} % 시간" if sfr > 0 else ""))
res_dir = os.path.expanduser("~/pv_out/harsh"); os.makedirs(res_dir, exist_ok=True)
import datetime  # noqa: E402
json.dump(dict(title=title, controller="wbctrl", args=vars(args), tune={k: getattr(P, k) for k in TUNE}, by_terrain=out,
               total=dict(ok=S, n=N, p=p, lo=lo, hi=hi)),
          open(os.path.join(res_dir, f"{datetime.datetime.now():%m%d_%H%M%S}.json"), "w"), ensure_ascii=False, indent=1, default=str)
print("[가혹 평가 끝]", flush=True)
env.close()
app.close()
