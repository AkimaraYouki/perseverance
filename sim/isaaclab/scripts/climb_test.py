"""대회 장애물 넘기 시험 — Ascento (arXiv 2005.11435, IV-B) 식 점프 상태머신. 점프는 강화학습이 아니다.

균형·주행은 학습된 균형 정책(관측 25, r3~r5)이 한다. 점프 구간만 상태머신이 다리(와 공중 바퀴)를 덮어쓴다:
  retract  다리 접기(웅크림) — 궤적(램프)으로. 한 번에 접으면 바퀴가 들린다. 균형은 정책이 계속 (Ascento: 균형 제어 켜 둔 채 접는다)
  extract  양다리 최대 신장(이륙). 다 펴지면 fly
  fly      지면 균형 제어 끔. 다리는 원래 게인으로 최대 접기 (바퀴를 모서리 위로) t_tuck 동안.
           바퀴는 반작용 휠로 몸통 pitch 만 잡는다 (Ascento 에 없는 우리 추가: 이륙 때 생긴 pitch 회전을
           안 잡으면 공중에서 -190 deg/s 로 뒤로 넘어간다, 2026-09-26 시험). air_ctrl=False 면 바퀴 토크 0
  descend  바퀴는 fly 와 같음. 다리는 부드러운 스프링-댐퍼(land_kp/kd)로 착지 길이까지 내려 충격 행정을 확보.
           고관절 토크가 contact_tau 를 넘으면 접지 (Ascento 와 같은 감지 — 실기에서도 전류로 된다)
  land     정책이 바퀴 재개, 다리는 부드러운 게인 그대로 land_s 동안, 그 뒤 원래 게인
발동: 바퀴 중심이 다음 모서리 앞 trigger 에 오면 (조종자 버튼 대신). 2 단 계단은 모서리마다 한 번씩 = 연속 점프.

    obstacle plateau   평대 (높이 step_h, 윗면 length)
    obstacle stairs2   2 단 계단 (한 단 step_h, 단 폭 tread)
    --mode stance        한 바퀴 서기 위험 시험 (평지),  --mode none  점프 없이 달리기

튜닝 값은 아래 TUNE 에 모여 있다.
    pv jump [--obstacle stairs2 ...]      창으로 보기     pv jump --record    mp4 로 (헤드리스)
"""
import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

# ================================ 튜닝 — 여기 값만 바꾸고 실행 ================================
# `pv jump` 로 창을 띄워 본다. 한 번만 바꿔 보려면 명령줄 `--이름 값` (예: pv jump --trigger 0.35)
TUNE = dict(
    # --- 접근·발동 ---
    v=0.8,               # 접근 속도 [m/s]. 바퀴 한계 18.85 rad/s x R 0.06 = 1.13 m/s 라 0.8 이 여유 있는 최대
    trigger=0.38,        # 바퀴 중심이 모서리 앞 이 거리에 오면 발동 [m]. 발동~이륙 사이 v 0.8 에서 약 0.24 m 간다
    # --- 1 retract: 웅크림 (정책이 균형) ---
    t_retract=0.12,      # [s]. 앞 75 % 동안 램프로 접고 나머지는 유지. 한 번에 접으면 바퀴가 들린다
    # --- 2 extract: 신전 = 이륙 (바퀴는 pitch PD) ---
    extract_pitch=3.0,   # 신전 중 pitch 목표 [deg, + = 앞으로 숙임]. 뒤로 젖힌 채 밀면 전진 속도를 잃는다 (-7 deg 에서 0.4 m/s)
    # --- 3 fly / 4 descend: 공중 (바퀴 = 반작용 휠) ---
    t_tuck=0.12,         # 이륙 뒤 다리를 최대로 접어 두는 시간 [s]. 그 뒤 착지 길이로 부드럽게 편다
    air_ctrl=True,       # False = Ascento 원형 (공중 바퀴 토크 0) -> 뒤로 -190 deg/s 로 넘어갔다
    pd_from="extract",   # 바퀴 pitch PD 를 켜는 단계: extract | fly
    air_pitch=-10.0,     # 공중 pitch 목표 [deg]. 음수 = 뒤로 젖힘 -> 몸 아래 바퀴가 앞으로 나간다
    air_kp=30.0,         # [N·m/rad, 바퀴 하나]
    air_kd=3.0,          # [N·m·s/rad, 바퀴 하나]
    air_tau=7.0,         # PD 구간 바퀴 토크 한계 [N·m] (AK45-10 피크. 평소 정책은 1.5)
    # --- 5 land: 착지 ---
    h_land=0.16,         # 착지 다리 길이 [m] (행정 0.1225 ~ 0.2425)
    land_kp=20.0,        # 착지 스프링 [N·m/rad] (평소 60)
    land_kd=1.0,         # 착지 댐퍼 [N·m·s/rad] (평소 1.5)
    land_s=0.30,         # 부드러운 게인 유지 시간 [s]
    contact_tau=2.0,     # 접지 판정 고관절 토크 [N·m] (descend 0.04 s 뒤부터)
    t_fly_max=0.45,      # 이륙 뒤 이 시간 안에 접지를 못 느끼면 강제로 land [s]
    # --- 장애물 ---
    obstacle="plateau",  # plateau (평대) | stairs2 (2 단 계단)
    step_h=0.08,         # 턱(한 단) 높이 [m]
    length=0.35,         # 평대 윗면 길이 [m]
    tread=0.35,          # 2 단 계단 단 폭 [m]
    edge=1.0,            # 첫 모서리 x [m]
    # --- 기타 ---
    hip="dc",            # dc (토크-속도 모델) | ideal
    hip_w0=None,         # 고관절 무부하 속도 [rad/s] (24 V 33.5, 6S 처짐 21 V 29.3). None = 33.5
    seconds=8.0,
)
POLICY = "logs/rsl_rl/wheeled_biped_balance/2026-09-25_18-24-20_rough_v3/model_7099.pt"   # 관측 25 균형 정책 (r3)
# ==========================================================================================

CHOICES = dict(obstacle=("plateau", "stairs2"), pd_from=("extract", "fly"), hip=("dc", "ideal"))
ap = argparse.ArgumentParser()
ap.add_argument("--policy", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", POLICY))
ap.add_argument("--mode", choices=("jump", "stance", "none"), default="jump")
for k, v in TUNE.items():
    if isinstance(v, bool):
        ap.add_argument(f"--{k}", type=lambda x: x.lower() in ("1", "true", "on", "yes"), default=v)
    elif isinstance(v, str):
        ap.add_argument(f"--{k}", choices=CHOICES.get(k), default=v)
    else:
        ap.add_argument(f"--{k}", type=float, default=v)
ap.add_argument("--record", default=None, metavar="DIR")
ap.add_argument("--out", default=None)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
if args.record:
    args.enable_cameras = True
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.terrains import TerrainImporterCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance import cad  # noqa: E402

TASK = "Isaac-WheeledBiped-CAD-Rough-Play-v0"
R = cad.R_WHEEL
H_MIN, H_MAX = 0.1225, 0.2425
H_CRUISE = 0.18

cfg = parse_env_cfg(TASK, num_envs=1, use_fabric=True)
cfg.scene.terrain = TerrainImporterCfg(
    prim_path="/World/ground", terrain_type="plane", collision_group=-1,
    physics_material=sim_utils.RigidBodyMaterialCfg(friction_combine_mode="multiply", restitution_combine_mode="multiply",
                                                    static_friction=1.0, dynamic_friction=1.0))
if args.hip == "dc":
    cfg.scene.robot = cad.CAD_ROBOT_CFG_DCHIP.replace(prim_path="{ENV_REGEX_NS}/Robot")
    if args.hip_w0:
        cfg.scene.robot.actuators["legs"].velocity_limit = args.hip_w0
cfg.events.reset_base.params["pose_range"] = {}
cfg.events.reset_base.params["velocity_range"] = {}
cfg.events.base_mass = None
cfg.events.base_com = None
cfg.terminations.time_out = None


def box(name, x0, x1, h):
    """정적 상자 (rigid body 없음 = 고정 충돌체). 폭 3 m 라 옆으로 비껴갈 일은 없다."""
    return AssetBaseCfg(
        prim_path=f"/World/{name}",
        spawn=sim_utils.CuboidCfg(size=(x1 - x0, 3.0, h), collision_props=sim_utils.CollisionPropertiesCfg(),
                                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.45, 0.35))),
        init_state=AssetBaseCfg.InitialStateCfg(pos=((x0 + x1) / 2, -0.08, h / 2)))


edges, top = [], 0.0
if args.mode in ("jump", "none"):
    if args.obstacle == "plateau":
        cfg.scene.obstacle = box("obstacle", args.edge, args.edge + args.length, args.step_h)
        edges, top = [args.edge], args.step_h
    else:
        end = args.edge + args.tread + 0.8
        cfg.scene.obstacle = box("tier1", args.edge, end, args.step_h)
        cfg.scene.obstacle2 = box("tier2", args.edge + args.tread, end, 2 * args.step_h)
        edges, top = [args.edge, args.edge + args.tread], 2 * args.step_h
if True:                                                              # 옆에서 로봇을 따라가는 카메라 (창·녹화 공통)
    cfg.viewer.origin_type = "asset_root"; cfg.viewer.asset_name = "robot"; cfg.viewer.env_index = 0
    cfg.viewer.eye = (0.2, -1.4, 0.35); cfg.viewer.lookat = (0.0, 0.0, 0.1); cfg.viewer.resolution = (1280, 720)
env = gym.make(TASK, cfg=cfg, render_mode="rgb_array" if args.record else None).unwrapped
dev = env.device
robot = env.scene["robot"]
cmd = env.command_manager.get_term("base_velocity")
cmd.pin(True)


def load_policy(path):
    try:
        return torch.jit.load(path, map_location=dev).eval()
    except RuntimeError:
        pass
    sd = torch.load(path, map_location=dev, weights_only=False)["actor_state_dict"]
    layers = []
    for i in range(4):
        w = sd[f"mlp.{2*i}.weight"]
        lin = torch.nn.Linear(w.shape[1], w.shape[0]).to(dev)
        lin.weight.data[:] = w; lin.bias.data[:] = sd[f"mlp.{2*i}.bias"]
        layers += [lin] + ([torch.nn.ELU()] if i < 3 else [])
    mlp = torch.nn.Sequential(*layers).eval()
    mean, std = sd["obs_normalizer._mean"], sd["obs_normalizer._std"]
    return lambda x: mlp((x - mean) / (std + 1e-2))


policy = load_policy(args.policy)
leg_ids = robot.find_joints(cad.LEG_JOINTS, preserve_order=True)[0]
wheel_bodies = robot.find_bodies(["l_wheel", "r_wheel"], preserve_order=True)[0]
hip_sign = torch.tensor([cad.M_SIGN["L"], cad.M_SIGN["R"]], device=dev)
wheel_ids = robot.find_joints(cad.WHEEL_JOINTS, preserve_order=True)[0]
wheel_term = env.action_manager.get_term("wheels")
scale0 = wheel_term.cfg.torque_scale
wsign = torch.ones(2, device=dev)                                  # 바퀴 관절축 -> 세계 +y (전진 구름) 부호. 달릴 때 잰다
legs_act = robot.actuators["legs"]
kp0, kd0 = legs_act.stiffness.clone(), legs_act.damping.clone()


def soft_legs(on):
    legs_act.stiffness[:] = args.land_kp if on else kp0
    legs_act.damping[:] = args.land_kd if on else kd0


RESTART = ("obstacle", "step_h", "length", "tread", "edge", "hip", "hip_w0")   # 장면을 다시 만들어야 해서 재시작 필요
CLI_KEYS = {k for k in TUNE if f"--{k}" in sys.argv}                        # 명령줄로 준 값은 파일보다 우선


def reload_tune():
    """이 파일의 TUNE 블록을 다시 읽어 args 에 반영한다 (창을 띄운 채 파일만 저장하면 다음 시도부터 적용)."""
    try:
        src = open(os.path.abspath(__file__), encoding="utf-8").read()
        i = src.index("TUNE = dict(")
        ns = {}
        exec(src[i:src.index("\n)\n", i) + 3], ns)
    except Exception as e:                                             # 저장 도중이거나 문법 오류면 이전 값 유지
        print(f"[튜닝] TUNE 읽기 실패, 이전 값 유지: {e}", flush=True)
        return
    for k, v in ns["TUNE"].items():
        if k in CLI_KEYS or getattr(args, k, None) == v:
            continue
        if k in RESTART:
            print(f"[튜닝] {k} = {v} 는 재시작해야 적용된다 (지금 {getattr(args, k)})", flush=True)
            continue
        print(f"[튜닝] {k}: {getattr(args, k)} -> {v}", flush=True)
        setattr(args, k, v)


dt = env.step_dt
LOOP = not args.headless and not args.record                          # 창으로 볼 때는 계속 반복
rec = None
if args.record:
    import datetime
    import imageio.v2 as imageio
    os.makedirs(args.record, exist_ok=True)
    rec_path = os.path.join(args.record, f"ascento_{args.mode}_{args.obstacle}_{int(args.step_h*1000)}mm_{datetime.datetime.now():%H%M%S}.mp4")
    rec = imageio.get_writer(rec_path, fps=50, codec="libx264", quality=8, macro_block_size=8)
    rec_every = max(1, round(1.0 / (dt * 50)))


def episode():
    global wsign
    soft_legs(False)
    wheel_term.cfg.torque_scale = scale0
    obs, _ = env.reset()
    phase, t_phase, next_edge, h0 = "drive", 0.0, 0, H_CRUISE
    log, jumps = [], []
    fell, fell_t = False, None
    LEG_TARGET = {"retract": H_MIN, "extract": H_MAX, "fly": H_MIN, "descend": args.h_land, "land": args.h_land}
    for k in range(int(args.seconds / dt)):
        if not app.is_running():
            return None
        t = k * dt
        d = robot.data
        wheel_pos = d.body_pos_w[0, wheel_bodies]                 # (2, 3)
        wx = float(wheel_pos[:, 0].mean())
        h_now = cad.leg_state(robot)[0][0]                         # (2,) 다리 길이
        tau = d.applied_torque[0, leg_ids] * hip_sign
        # --- 상태머신 --------------------------------------------------------------------------------
        vx = args.v
        if args.mode == "jump":
            tp = t - t_phase
            if phase == "drive" and next_edge < len(edges) and wx >= edges[next_edge] - args.trigger:
                phase, t_phase, h0 = "retract", t, float(h_now.mean())
                wv = d.joint_vel[0, wheel_ids]
                if float(wv.abs().min()) > 1.0:
                    wsign = torch.sign(wv)
                jumps.append(dict(edge=next_edge, t_trigger=round(t, 3), x_trigger=round(wx, 3)))
            elif phase == "retract" and tp >= args.t_retract:
                phase, t_phase = "extract", t
                if args.air_ctrl and args.pd_from == "extract":
                    wheel_term.cfg.torque_scale = args.air_tau
            elif phase == "extract" and (float(h_now.min()) >= H_MAX - 0.008 or tp >= 0.25):
                phase, t_phase = "fly", t
                jumps[-1]["t_takeoff"], jumps[-1]["x_takeoff"] = round(t, 3), round(wx, 3)
                if args.air_ctrl:
                    wheel_term.cfg.torque_scale = args.air_tau
            elif phase == "fly" and tp >= args.t_tuck:
                phase, t_phase = "descend", t
                soft_legs(True)
            elif phase == "descend" and ((tp > 0.04 and float(tau.abs().max()) > args.contact_tau)
                                         or t - jumps[-1]["t_takeoff"] >= args.t_fly_max):
                jumps[-1].update(t_land=round(t, 3), x_land=round(wx, 3), contact="torque" if tp > 0.04 and
                                 float(tau.abs().max()) > args.contact_tau else "timeout",
                                 wheel_bottom_mm=round((float(wheel_pos[:, 2].min()) - R) * 1000, 1))
                phase, t_phase = "land", t
                wheel_term.cfg.torque_scale = scale0
            elif phase == "land" and tp >= args.land_s:
                soft_legs(False)
                phase, t_phase = "drive", t
                next_edge += 1
            if next_edge >= len(edges):
                vx = 0.0                                           # 다 올라가면 멈춘다 (윗면 20–50 cm)
        elif args.mode == "stance":
            vx = 0.0
            phase = "stand" if t < 1.0 else ("lean" if t < 2.0 else "lift")
        cmd.set(torch.tensor([vx], device=dev), torch.tensor([0.0], device=dev), torch.tensor([H_CRUISE], device=dev),
                mode=torch.tensor([0.0], device=dev))
        act = policy(obs["policy"]).clone()
        h_ref = float(cmd.command[0, 2])
        if args.mode == "jump" and phase in LEG_TARGET:
            tgt = LEG_TARGET[phase]
            if phase == "retract":
                tgt = h0 + (H_MIN - h0) * min(1.0, (t - t_phase) / (0.75 * args.t_retract))
            act[0, :2] = (tgt - h_ref) / 0.12
            pd_phases = ("fly", "descend") if args.pd_from == "fly" else ("extract", "fly", "descend")
            if phase in pd_phases:                                 # 지면 균형 제어 끔
                if not args.air_ctrl:
                    act[0, 2:] = 0.0
                else:                                              # 바퀴 +y 토크 u -> 몸통에 -u. 뒤로 젖혀지면(pitch<0) 바퀴를 감속
                    pitch = math.asin(max(-1.0, min(1.0, float(d.projected_gravity_b[0, 0]))))
                    ref = args.extract_pitch if phase == "extract" else args.air_pitch
                    u = args.air_kp * (pitch - math.radians(ref)) + args.air_kd * float(d.root_ang_vel_b[0, 1])
                    act[0, 2:] = wsign * max(-1.0, min(1.0, u / args.air_tau))
        elif args.mode == "stance" and phase != "stand":
            s = min(1.0, (t - 1.0) / 1.0)
            hl, hr = H_CRUISE + 0.045 * s, H_CRUISE - 0.045 * s     # 좌우 길이차 9 cm -> 약 24 deg 기울기
            if phase == "lift":
                hl = H_MIN                                         # 왼쪽 다리를 최대로 접어 바퀴를 든다
            act[0, 0], act[0, 1] = (hl - h_ref) / 0.12, (hr - h_ref) / 0.12
        obs, _, term, _, _ = env.step(act)
        if bool(term[0]) and not fell:
            fell, fell_t = True, t
            if LOOP:
                break                                              # 창 모드: 넘어지면 바로 다음 시도
        # --- 기록 ------------------------------------------------------------------------------------
        g = d.projected_gravity_b[0]
        mvel = d.joint_vel[0, leg_ids] * hip_sign
        log.append(dict(t=t, phase=phase, wx=wx, wz_l=float(wheel_pos[0, 2]), wz_r=float(wheel_pos[1, 2]),
                        bz=float(d.root_com_pos_w[0, 2]), bx=float(d.root_com_pos_w[0, 0]),
                        pitch=math.degrees(math.asin(max(-1, min(1, float(g[0]))))),
                        roll=math.degrees(math.asin(max(-1, min(1, float(g[1]))))),
                        hl=float(h_now[0]), hr=float(h_now[1]), tau_l=float(tau[0]), tau_r=float(tau[1]),
                        w_l=float(mvel[0]), w_r=float(mvel[1]),
                        tw_l=float(d.applied_torque[0, wheel_ids[0]] * wsign[0]), vw_l=float(d.joint_vel[0, wheel_ids[0]] * wsign[0])))
        if rec is not None and k % rec_every == 0:
            rec.append_data(np.asarray(env.render())[..., :3])
    return judge(log, jumps, fell, fell_t)


def judge(L, jumps, fell, fell_t):
    tau_max = max(max(abs(r["tau_l"]), abs(r["tau_r"])) for r in L)
    w_max = max(max(abs(r["w_l"]), abs(r["w_r"])) for r in L)
    res = dict(mode=args.mode, tune={k: getattr(args, k) for k in TUNE}, wsign=wsign.tolist(),
               fell=fell, fell_t=fell_t, tau_hip_max_nm=tau_max, hip_speed_max_rad_s=w_max,
               current_max_a_kt060=tau_max / 0.5994, current_max_a_kt081=tau_max / 0.81, jumps=jumps)
    if args.mode in ("jump", "none"):
        last = L[-int(1.0 / dt):]
        res["wheel_bottom_max_mm"] = float(max(min(r["wz_l"], r["wz_r"]) - R for r in L) * 1000)
        res["final_wheel_bottom_mm"] = float(np.mean([min(r["wz_l"], r["wz_r"]) - R for r in last]) * 1000)
        res["success"] = (not fell) and res["final_wheel_bottom_mm"] > (top - 0.01) * 1000
        air = [r for r in L if r["phase"] in ("extract", "fly", "descend", "land")]
        res["pitch_range_in_jump_deg"] = [float(min(r["pitch"] for r in air)), float(max(r["pitch"] for r in air))] if air else None
        res["final_x"] = L[-1]["wx"]
    else:
        lift = [r for r in L if r["phase"] == "lift"]
        ok = [r for r in lift if abs(r["roll"]) < 40]
        res["left_wheel_lift_mm"] = float(max(r["wz_l"] - R for r in lift) * 1000) if lift else None
        res["stance_time_s"] = (fell_t - 2.0) if fell else (len(ok) * dt)
    return res, L


def summary(n, res):
    """창 모드용 한 줄: 성공, 넘어짐, 점프마다 이륙·착지 x (모서리 기준 mm), 착지 순간 바퀴 바닥 높이, 공중 pitch 범위."""
    js = "  ".join(f"[{j['edge']+1}] 이륙 {1000*(j.get('x_takeoff', float('nan'))-edges[j['edge']]):+.0f} 착지 "
                   f"{1000*(j.get('x_land', float('nan'))-edges[j['edge']]):+.0f} mm ({j.get('contact', '-')}, 바닥 {j.get('wheel_bottom_mm', '-')} mm)"
                   for j in res["jumps"])
    pr = res.get("pitch_range_in_jump_deg")
    print(f"#{n} {'성공' if res.get('success') else '실패'}"
          + (f"  넘어짐 {res['fell_t']:.2f} s" if res["fell"] else "")
          + (f"  pitch {pr[0]:.0f}~{pr[1]:.0f} deg" if pr else "") + f"  {js}", flush=True)


n = 0
with torch.inference_mode():
    while True:
        n += 1
        if LOOP and n > 1:
            reload_tune()
        r = episode()
        if r is None:
            break
        res, L = r
        if LOOP:
            summary(n, res)
            continue
        break

if not LOOP and r is not None:
    if rec is not None:
        rec.close()
    print("\n" + json.dumps(res, ensure_ascii=False, indent=1), flush=True)
    summary(1, res)
    out = args.out or os.path.expanduser(f"~/pv_out/climb/ascento_{args.mode}_{args.obstacle}_{int(args.step_h*1000)}mm.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(dict(result=res, log=L[:: max(1, len(L) // 2000)]), open(out, "w"), ensure_ascii=False)
    print(f"저장: {out}" + (f"\n영상: {rec_path}" if rec is not None else ""), flush=True)
env.close()
app.close()
