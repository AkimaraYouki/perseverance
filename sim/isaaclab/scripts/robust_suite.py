"""강인성 시험 세트 — 한 시나리오를 로봇 N 대로 동시에 (대마다 다른 모델 오차·센서 잡음·지연) 돌려 통과율을 낸다.

제어기는 wbctrl.WBController (climb_test 의 ctrl="lqr" 과 같은 로직), 게인·설정은 climb_test.py 의 TUNE 블록을 그대로 읽는다
(창에서 쓰는 값 = 시험하는 값). 명령줄 --키 값 으로 TUNE 을 덮어쓸 수 있다.

    isaaclab.sh -p _isaaclab_launch.py robust_suite.py --scenario ridges --n 8 [--seed 100] [--roll_kd 0.3 ...]
    pv robust                 # 모든 시나리오를 차례로, 한 표로

시나리오 (통과 조건):
  flat_stop  평지 0.6 m/s 로 달리다 5 s 에 멈추기          넘어지지 않고 멈춤 (|v| < 0.15)
  turn       평지 0.6 m/s 로 달리며 2 s 마다 좌우 1 rad/s 슬랄롬   넘어지지 않음
  ridges     8 cm 엇갈린 삼각형길 (대회 CAD) 직선 0.4 m/s    넘어지지 않고 삼각형길 끝 (x >= 6.25 m)
  ramp       30 cm 경사로 오르내리기, 스틱 끝 (3 km/h)        넘어지지 않고 내리막 끝 (x >= 5.0 m)
  jump       8 cm 평대 자동 점프                              넘어지지 않고 윗면 (바퀴 바닥 >= 70 mm)
  hand10/20  0.3 m/s 로 달리다 손으로 15 cm 들어 10/20 deg 기울여 놓기   넘어지지 않음
"""
import argparse
import json
import math
import os
import sys
import time
import types

from isaaclab.app import AppLauncher

HERE = os.path.dirname(os.path.abspath(__file__))


def load_tune():
    src = open(os.path.join(HERE, "climb_test.py"), encoding="utf-8").read()
    i = src.index("TUNE = dict(")
    ns = {}
    exec(src[i:src.index("\n)\n", i) + 3], ns)
    return ns["TUNE"]


TUNE = load_tune()
SCEN = ("flat_stop", "turn", "ridges", "ramp", "jump", "hand10", "hand20")
ap = argparse.ArgumentParser()
ap.add_argument("--scenario", choices=SCEN, required=True)
ap.add_argument("--n", type=int, default=8)
ap.add_argument("--seed", type=int, default=100, help="로봇 i 의 무작위 시드 = seed + i")
ap.add_argument("--out", default=None)
ap.add_argument("--policy", default=None, help="잔차 RL 정책 (model_N.pt). 주면 학습 환경의 제어기(residual.py) + 정책으로 돈다 (점프 제외)")
for k, v in TUNE.items():
    if isinstance(v, bool):
        ap.add_argument(f"--{k}", type=lambda x: x.lower() in ("1", "true", "on", "yes"), default=v)
    elif isinstance(v, str):
        ap.add_argument(f"--{k}", default=v)
    else:
        ap.add_argument(f"--{k}", type=float, default=v)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
args.device = "cpu"                                                   # 로봇 여러 대도 CPU 물리가 빠르다 (파이썬 제어가 병목)
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import gymnasium as gym  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.terrains import TerrainImporterCfg  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance import cad  # noqa: E402

import lqr_vmc  # noqa: E402
import wbctrl  # noqa: E402

P = types.SimpleNamespace(**{k: getattr(args, k) for k in TUNE})
N, SC = args.n, args.scenario
OBST = os.path.expanduser("~/perseverance/sim/obstacles")
spec = dict(
    flat_stop=dict(sec=8.0, v=0.6, stop_at=5.0),
    turn=dict(sec=10.0, v=0.6, slalom=1.0),                          # 달리며 2 s 마다 좌우 1 rad/s 슬랄롬
    ridges=dict(sec=18.0, v=0.4, cad="course.stl", stop_x=6.8),     # 뒤에 이어진 ㅗ 턱(7.75 m) 앞에서 멈춘다
    ramp=dict(sec=12.0, v=P.vmax_kmh / 3.6, cad="arena.stl"),
    jump=dict(sec=8.0, v=P.v, box=(1.0, 2.0, 0.08), edges=[1.0]),
    hand10=dict(sec=9.0, v=0.3, hand=(3.0, 5.0, 10.0)),
    hand20=dict(sec=9.0, v=0.3, hand=(3.0, 5.0, 20.0)),
)[SC]

# --- 장면 -------------------------------------------------------------------------------------------
TASK = "Isaac-WheeledBiped-CAD-Residual-Play-v0" if args.policy else "Isaac-WheeledBiped-CAD-Rough-Play-v0"
if args.policy and SC == "jump":
    raise SystemExit("정책 모드는 점프 제외 (점프는 상태머신)")
cfg = parse_env_cfg(TASK, device="cpu", num_envs=N, use_fabric=True)
cfg.scene.env_spacing = 16.0
cfg.scene.terrain = TerrainImporterCfg(
    prim_path="/World/ground", terrain_type="plane", collision_group=-1,
    physics_material=sim_utils.RigidBodyMaterialCfg(friction_combine_mode="multiply", restitution_combine_mode="multiply",
                                                    static_friction=1.0, dynamic_friction=1.0))
cfg.scene.robot = cad.CAD_ROBOT_CFG_DCHIP.replace(prim_path="{ENV_REGEX_NS}/Robot")
_p = cfg.scene.robot.init_state.pos
cfg.scene.robot.init_state.pos = (_p[0], _p[1] + 0.08, _p[2] + P.spawn_z)   # 바퀴 가운데 = 환경 원점 y, 공중 스폰
for name in ("base_mass", "base_com", "push"):
    if hasattr(cfg.events, name):
        setattr(cfg.events, name, None)
cfg.events.reset_base.params["pose_range"] = {}
cfg.events.reset_base.params["velocity_range"] = {}
for grp in (cfg.rewards, cfg.curriculum, cfg.terminations):
    for n_ in list(vars(grp)):
        if not n_.startswith("_"):
            setattr(grp, n_, None)
if hasattr(cfg.observations, "critic") and not args.policy:
    cfg.observations.critic = None
if hasattr(cfg.scene, "critic_scanner") and not args.policy:
    cfg.scene.critic_scanner = None
if args.policy:                                                        # 학습 환경 제어기에 이 시험의 조건을 맞춘다
    for k_ in ("imu_tilt_noise_deg", "imu_tilt_bias_deg", "imu_gyro_noise", "enc_vel_noise", "delay_ms", "dr_motor"):
        setattr(cfg.actions.ctrl, k_, getattr(P, k_))
    cfg.actions.ctrl.delay_extra_prob = P.jitter_ms / 5.0
    cfg.actions.ctrl.dr_motor = 0.0                                   # 모터 오차는 이 시험이 로봇별로 준다 (아래)
cfg.decimation = max(1, round(P.physics_hz / 200.0))
cfg.sim.dt = 1.0 / (200.0 * cfg.decimation)
if "cad" in spec:
    from isaaclab.sim.converters import MeshConverter, MeshConverterCfg
    from isaaclab.sim.schemas import schemas_cfg
    conv = MeshConverter(MeshConverterCfg(
        asset_path=os.path.join(OBST, spec["cad"]), usd_dir=os.path.expanduser("~/pv_out/cad_obstacles"),
        usd_file_name=os.path.splitext(spec["cad"])[0] + ".usd", make_instanceable=False,
        collision_props=schemas_cfg.CollisionPropertiesCfg(collision_enabled=True),
        mesh_collision_props=schemas_cfg.TriangleMeshPropertiesCfg(), scale=(0.001,) * 3))
    cfg.scene.obst = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Obst", spawn=sim_utils.UsdFileCfg(usd_path=conv.usd_path))
if "box" in spec:
    x0, x1, hb = spec["box"]
    cfg.scene.obst = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Obst",
        spawn=sim_utils.CuboidCfg(size=(x1 - x0, 3.0, hb), collision_props=sim_utils.CollisionPropertiesCfg(),
                                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=((x0 + x1) / 2, 0.0, hb / 2)))
env = gym.make(TASK, cfg=cfg).unwrapped
dev = env.device
robot = env.scene["robot"]
cmd = env.command_manager.get_term("base_velocity")
cmd.pin(True)
cmd.cfg.auto_height = cmd.cfg.default_height = P.idle_h
if not args.policy:
    wheel_term = env.action_manager.get_term("wheels")
    wheel_term.cfg.torque_scale = P.wheel_tau_max                    # 모든 바퀴 행동 = 토크 / wheel_tau_max
pol = None
if args.policy:
    sd_ = torch.load(args.policy, map_location=dev, weights_only=False)["model_state_dict"]
    ks_ = sorted({k.split(".")[1] for k in sd_ if k.startswith("actor.")}, key=int)
    layers_ = []
    for j_, k_ in enumerate(ks_):
        w_, b_ = sd_[f"actor.{k_}.weight"], sd_[f"actor.{k_}.bias"]
        lin_ = torch.nn.Linear(w_.shape[1], w_.shape[0]).to(dev); lin_.weight.data[:] = w_; lin_.bias.data[:] = b_
        layers_ += [lin_] + ([torch.nn.ELU()] if j_ < len(ks_) - 1 else [])
    mlp_ = torch.nn.Sequential(*layers_).eval()
    mu_, sg_ = sd_.get("actor_obs_normalizer._mean"), sd_.get("actor_obs_normalizer._std")
    pol = (lambda o: mlp_((o - mu_) / (sg_ + 1e-2))) if mu_ is not None else mlp_
leg_ids = robot.find_joints(cad.LEG_JOINTS, preserve_order=True)[0]
wheel_ids = robot.find_joints(cad.WHEEL_JOINTS, preserve_order=True)[0]
wheel_bodies = robot.find_bodies(["l_wheel", "r_wheel"], preserve_order=True)[0]
hip_sign = torch.tensor([cad.M_SIGN["L"], cad.M_SIGN["R"]], device=dev)
wsign = torch.tensor(cad.WHEEL_SIGN, device=dev)
nonwheel = [i for i in range(robot.num_bodies) if i not in wheel_bodies]
legs_act, wheels_act = robot.actuators["legs"], robot.actuators["wheels"]
R = cad.R_WHEEL
origins = env.scene.env_origins.clone()

# --- 명목 모델 (제어기가 믿는 값) + LQR ----------------------------------------------------------------
obs, _ = env.reset()
mass_nom = robot.root_physx_view.get_masses().clone()                # (N, B) 복사!
m_nom = mass_nom[0].to(dev)
m_pend = float(m_nom[nonwheel].sum())
d = robot.data
c0 = (d.body_com_pos_w[0, nonwheel] * m_nom[nonwheel, None]).sum(0) / m_pend
iyy = robot.root_physx_view.get_inertias()[0][:, 4].to(dev)
rel = d.body_com_pos_w[0, nonwheel] - c0
I_pend = float((iyy[nonwheel] + m_nom[nonwheel] * (rel[:, 0] ** 2 + rel[:, 2] ** 2)).sum())
lqr = lqr_vmc.WheelLQR(m_pend, I_pend, float(m_nom[wheel_bodies].sum()), 2 * 1.755e-3, R,
                       q=(P.lqr_qx, P.lqr_qv, P.lqr_qth, P.lqr_qthd), r=P.lqr_r)

# --- 모델 오차 (로봇마다) ------------------------------------------------------------------------------
rngs = [np.random.default_rng(args.seed + i) for i in range(N)]
view = robot.root_physx_view
masses, coms = view.get_masses().clone(), view.get_coms().clone()
vlim = wheels_act.velocity_limit.clone() if torch.is_tensor(wheels_act.velocity_limit) else torch.full((N, 2), wheels_act.velocity_limit)
dr = []
for i, g in enumerate(rngs):
    km = 1.0 + g.uniform(-1, 1) * P.dr_mass
    dx, dz = g.uniform(-1, 1, 2) * P.dr_com_cm / 100.0
    kv = 1.0 - g.uniform(0, 1) * P.dr_motor
    masses[i, 0] *= km; coms[i, 0, 0] += dx; coms[i, 0, 2] += dz; vlim[i] *= kv
    dr.append(dict(mass=round(km, 3), com_x_cm=round(dx * 100, 2), com_z_cm=round(dz * 100, 2), wheel_motor=round(kv, 3)))
idx = torch.arange(N, device="cpu")
view.set_masses(masses, idx); view.set_coms(coms, idx)
wheels_act.velocity_limit = vlim.to(dev)
if hasattr(wheels_act, "_vel_at_effort_lim"):
    wheels_act._vel_at_effort_lim = wheels_act.velocity_limit * (1 + wheels_act.effort_limit / wheels_act._saturation_effort)
mass_true = masses.to(dev)
motor_est = [dr[i]["wheel_motor"] * (1.0 + rngs[i].normal(0, 0.02)) for i in range(N)]   # 전압으로 추정한 모터 한계 (오차 2 %)
if args.policy:                                                        # 학습 환경 제어기에도 같은 모터 오차·추정
    term_ = env.action_manager.get_term("ctrl")
    term_.motor_true[:] = torch.tensor([x["wheel_motor"] for x in dr], device=dev)
    term_.motor_est[:] = torch.tensor(motor_est, device=dev, dtype=torch.float32)
ctrls = [wbctrl.WBController(P, lqr, m_pend, seed=args.seed + 1000 + i, edges=spec.get("edges", ())) for i in range(N)]


def yaw_of(q):
    return torch.atan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))


# --- 실행 ----------------------------------------------------------------------------------------------
dt = env.step_dt
steps = int(spec["sec"] / dt)
fell_t = [None] * N
xmax = np.full(N, -9.0)
rec = [dict(pitch=[], roll=[], v=[]) for _ in range(N)]
grab = None
t0 = time.time()
with torch.inference_mode():
    for k in range(steps):
        t = k * dt
        d = robot.data
        g_b, w_b = d.projected_gravity_b.cpu().numpy(), d.root_ang_vel_b.cpu().numpy()
        q = d.root_quat_w
        psi = yaw_of(q)
        fwd = torch.stack([torch.cos(psi), torch.sin(psi), torch.zeros_like(psi)], 1)
        lat = torch.stack([-torch.sin(psi), torch.cos(psi), torch.zeros_like(psi)], 1)
        h = cad.leg_state(robot)[0]
        tau = d.applied_torque[:, leg_ids] * hip_sign
        wj = d.joint_vel[:, wheel_ids] * wsign
        wabs = (d.body_ang_vel_w[:, wheel_bodies] * lat[:, None]).sum(-1)
        ax = d.body_pos_w[:, wheel_bodies].mean(1)
        c_nom = (d.body_com_pos_w[:, nonwheel] * m_nom[None, nonwheel, None]).sum(1) / m_pend
        rb = quat_apply_inverse(q, c_nom - ax)
        th_kin = torch.atan2(rb[:, 0], rb[:, 2]); l_p = rb.norm(dim=1)
        c_tru = (d.body_com_pos_w[:, nonwheel] * mass_true[:, nonwheel, None]).sum(1) / mass_true[:, nonwheel].sum(1, keepdim=True)
        r_t = c_tru - ax
        th_true = torch.atan2((r_t * fwd).sum(1), r_t[:, 2])
        v_true = (d.body_lin_vel_w[:, wheel_bodies].mean(1) * fwd).sum(1)
        sf = (d.body_lin_acc_w[:, 0] + torch.tensor([0.0, 0.0, 9.81], device=dev)).norm(dim=1) / 9.81
        wx = (ax[:, 0] - origins[:, 0]).cpu().numpy()
        wzmin = (d.body_pos_w[:, wheel_bodies, 2].min(1).values - R).cpu().numpy()
        to = lambda x: x.cpu().numpy()  # noqa: E731
        h_, tau_, wj_, wabs_, thk_, lp_, tht_, vt_, sf_, psi_ = map(to, (h, tau, wj, wabs, th_kin, l_p, th_true, v_true, sf, psi))

        vx = 0.0 if ("stop_at" in spec and t >= spec["stop_at"]) else spec["v"]
        if pol is not None:                                           # 정책 모드: 명령만 주고 제어는 학습 환경 액션 항이
            vxs = torch.tensor([0.0 if ("stop_x" in spec and wx[i] >= spec["stop_x"]) else vx for i in range(N)], device=dev)
            if "slalom" in spec:
                wzs = torch.full((N,), spec["slalom"] * (1.0 if int(t // 2.0) % 2 == 0 else -1.0), device=dev)
            else:
                wzs = torch.clamp(-P.heading_kp * psi, -1.0, 1.0)
            cmd.set(vxs, wzs, torch.full((N,), P.idle_h, device=dev), mode=torch.ones(N, device=dev))
            for i in range(N):
                if fell_t[i] is None:
                    rec[i]["pitch"].append(math.degrees(math.asin(max(-1.0, min(1.0, float(g_b[i][0]))))))
                    rec[i]["roll"].append(math.degrees(math.asin(max(-1.0, min(1.0, float(g_b[i][1]))))))
                    rec[i]["v"].append(float(vt_[i])); xmax[i] = max(xmax[i], wx[i])
            act_t = pol(obs["policy"])
        acts = np.zeros((N, 4)); kps = np.zeros((N, 2)); kds = np.zeros((N, 2)); ff = np.zeros(N)
        for i in (range(N) if pol is None else ()):
            if fell_t[i] is not None:
                continue
            vx_i = 0.0 if ("stop_x" in spec and wx[i] >= spec["stop_x"]) else vx
            if "slalom" in spec:
                wz = spec["slalom"] * (1.0 if int(t // 2.0) % 2 == 0 else -1.0)
            else:
                wz = max(-1.0, min(1.0, -P.heading_kp * float(psi_[i])))
            f = wbctrl.Frame(t=t, g_b=g_b[i], w_b=w_b[i], h=h_[i], tau_hip=tau_[i], w_wheel_joint=wj_[i], w_wheel_abs=wabs_[i],
                             th_kin=float(thk_[i]), l_pend=float(lp_[i]), wx=float(wx[i]), wheel_z_min=float(wzmin[i]),
                             yaw=float(psi_[i]), sf=float(sf_[i]), truth_th=float(tht_[i]), truth_v=float(vt_[i]),
                             motor_scale=motor_est[i])
            a, kp, kd, ffF, info = ctrls[i].step(f, vx_i, wz, P.idle_h)
            acts[i], kps[i], kds[i], ff[i] = a, kp, kd, ffF
            rec[i]["pitch"].append(math.degrees(info["pitch"])); rec[i]["roll"].append(math.degrees(info["roll"]))
            rec[i]["v"].append(float(vt_[i]))
            xmax[i] = max(xmax[i], wx[i])
        if pol is None:
            cmd.set(torch.full((N,), float(vx), device=dev), torch.zeros(N, device=dev), torch.full((N,), P.idle_h, device=dev),
                    mode=torch.ones(N, device=dev))
            legs_act.stiffness[:] = torch.tensor(kps, device=dev, dtype=torch.float32)
            legs_act.damping[:] = torch.tensor(kds, device=dev, dtype=torch.float32)
            M = d.joint_pos[:, leg_ids]
            robot.set_joint_effort_target(hip_sign * torch.tensor(ff, device=dev, dtype=torch.float32)[:, None]
                                          * cad.dh_from_M(M).to(torch.float32), joint_ids=leg_ids)
        if "hand" in spec:                                          # 가상 손 (전 로봇 동시)
            tg, tr, roll_deg = spec["hand"]
            if tg <= t < tr:
                if grab is None:
                    grab = dict(t0=t, p=d.root_pos_w.clone())
                s_ = min(1.0, (t - grab["t0"]) / 0.5)
                tgt = grab["p"].clone(); tgt[:, 2] += s_ * 0.15
                F = 400.0 * (tgt - d.root_pos_w) - 40.0 * d.root_lin_vel_w
                F[:, 2] += mass_true.sum(1) * 9.81
                phi = -torch.asin(d.projected_gravity_b[:, 1].clamp(-1, 1)); th_ = torch.asin(d.projected_gravity_b[:, 0].clamp(-1, 1))
                wbb = d.root_ang_vel_b
                tb = torch.stack([20.0 * (math.radians(roll_deg) * s_ - phi) - 2.0 * wbb[:, 0], 20.0 * (0.0 - th_) - 2.0 * wbb[:, 1],
                                  -1.0 * wbb[:, 2]], 1)
                robot.set_external_force_and_torque(F[:, None], quat_apply(q, tb)[:, None], body_ids=[0], is_global=True)
            elif t >= tr and grab is not None:
                robot.set_external_force_and_torque(torch.zeros(N, 1, 3, device=dev), torch.zeros(N, 1, 3, device=dev), body_ids=[0])
                grab = None
        obs, _, _, _, _ = env.step(act_t if pol is not None else torch.tensor(acts, device=dev, dtype=torch.float32))
        gz = robot.data.projected_gravity_b[:, 2].cpu().numpy()
        for i in range(N):                                          # 넘어짐 = 50 deg 넘게 기욺
            if fell_t[i] is None and math.degrees(math.acos(max(-1.0, min(1.0, -float(gz[i]))))) > 50:
                fell_t[i] = round(t, 3)

# --- 판정 ------------------------------------------------------------------------------------------------
d = robot.data
v_end = (d.body_lin_vel_w[:, wheel_bodies].mean(1)[:, 0]).cpu().numpy()
wzmin_end = (d.body_pos_w[:, wheel_bodies, 2].min(1).values - R).cpu().numpy()
rows = []
for i in range(N):
    ok = fell_t[i] is None
    if SC == "flat_stop":
        ok = ok and len(rec[i]["v"]) > 100 and abs(float(np.mean(rec[i]["v"][-100:]))) < 0.15   # 마지막 0.5 s 평균 속도
    elif SC == "ridges":
        ok = ok and xmax[i] >= 6.2
    elif SC == "ramp":
        ok = ok and xmax[i] >= 5.0
    elif SC == "jump":
        ok = ok and wzmin_end[i] >= 0.07
    pr = np.abs(rec[i]["pitch"][200:]) if len(rec[i]["pitch"]) > 200 else np.abs(rec[i]["pitch"])
    rr = np.abs(rec[i]["roll"][200:]) if len(rec[i]["roll"]) > 200 else np.abs(rec[i]["roll"])
    rows.append(dict(i=i, ok=bool(ok), fell_t=fell_t[i], xmax=round(float(xmax[i]), 2), dr=dr[i],
                     pitch95=round(float(np.percentile(pr, 95)), 1) if len(pr) else None,
                     roll95=round(float(np.percentile(rr, 95)), 1) if len(rr) else None,
                     jumps=ctrls[i].jumps if pol is None else []))
npass = sum(r["ok"] for r in rows)
wall = time.time() - t0
print(f"\n[{SC}] 통과 {npass}/{N}   (시뮬 {spec['sec']:.0f} s, 실제 {wall:.0f} s)", flush=True)
for r in rows:
    print(f"  #{r['i']} {'통과' if r['ok'] else '실패'}  넘어짐 {r['fell_t']}  x최대 {r['xmax']:.2f}  pitch95 {r['pitch95']}  roll95 {r['roll95']}  "
          f"| 질량 x{r['dr']['mass']} 무게중심 x{r['dr']['com_x_cm']:+.1f} z{r['dr']['com_z_cm']:+.1f} cm 모터 x{r['dr']['wheel_motor']}", flush=True)
out = args.out or os.path.expanduser(f"~/pv_out/robust/{SC}.json")
os.makedirs(os.path.dirname(out), exist_ok=True)
json.dump(dict(scenario=SC, n=N, seed=args.seed, npass=npass, tune={k: getattr(args, k) for k in TUNE}, rows=rows),
          open(out, "w"), ensure_ascii=False, default=str)
print(f"SUITE {SC} {npass}/{N}", flush=True)
env.close()
app.close()
