"""대회 장애물(8 cm 평대) 넘기 가능성 시험 — 균형은 학습된 정책이 바퀴로, 다리만 스크립트로 덮어쓴다.

    --mode jump    접근 -> 웅크림 -> 최대 신전(이륙) -> 공중 수축 -> 착지 -> 정책에 돌려줌
    --mode stance  "한쪽씩 올리기" 의 핵심 위험만: 무게를 오른쪽 바퀴 위로 옮기고 왼쪽 다리를 들어 몇 초 버티나 (평지)
    --mode none    장애물 앞에서 그냥 달려 본다 (굴러서는 못 넘는다는 확인용)

기록: 바퀴 최대 높이, 모서리 통과 여유, 고관절 토크·속도·추정 전류(Kt 0.60 / 0.81), 공중 pitch, 성공 여부.
고관절은 기본으로 토크-속도 모델(cad.CAD_ROBOT_CFG_DCHIP). --hip ideal 이면 학습 때와 같은 IdealPD.

    isaaclab.sh -p _isaaclab_launch.py climb_test.py --policy <model_N.pt> --mode jump [--height 0.08 --length 0.35 --v 0.4]
"""
import argparse
import json
import math
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--policy", required=True)
ap.add_argument("--mode", choices=("jump", "stance", "none"), default="jump")
ap.add_argument("--step_h", type=float, default=0.08, help="장애물 높이 [m]")
ap.add_argument("--length", type=float, default=0.35, help="장애물 윗면 길이 [m]")
ap.add_argument("--edge", type=float, default=1.0, help="장애물 앞 모서리 x [m]")
ap.add_argument("--v", type=float, default=0.4, help="접근 속도 [m/s]")
ap.add_argument("--trigger", type=float, default=0.10, help="바퀴 중심이 모서리 앞 이 거리에 오면 점프 시작 [m]")
ap.add_argument("--t_crouch", type=float, default=0.25)
ap.add_argument("--t_air", type=float, default=0.20, help="공중 수축 유지 시간 [s]")
ap.add_argument("--h_land", type=float, default=0.16, help="착지 다리 길이 [m]")
ap.add_argument("--hip", choices=("dc", "ideal"), default="dc")
ap.add_argument("--seconds", type=float, default=8.0)
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
cfg.events.reset_base.params["pose_range"] = {}
cfg.events.reset_base.params["velocity_range"] = {}
cfg.events.base_mass = None
cfg.events.base_com = None
cfg.terminations.time_out = None
if args.mode in ("jump", "none"):
    # 정적 상자 (rigid body 없음 = 고정 충돌체). 폭 3 m 라 로봇이 옆으로 비껴갈 일은 없다.
    cfg.scene.obstacle = AssetBaseCfg(
        prim_path="/World/obstacle",
        spawn=sim_utils.CuboidCfg(size=(args.length, 3.0, args.step_h),
                                  collision_props=sim_utils.CollisionPropertiesCfg(),
                                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.45, 0.35))),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(args.edge + args.length / 2, -0.08, args.step_h / 2)))
if args.record:
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

rec = None
if args.record:
    import datetime
    import imageio.v2 as imageio
    os.makedirs(args.record, exist_ok=True)
    rec_path = os.path.join(args.record, f"climb_{args.mode}_{int(args.step_h*1000)}mm_{datetime.datetime.now():%H%M%S}.mp4")
    rec = imageio.get_writer(rec_path, fps=50, codec="libx264", quality=8, macro_block_size=8)
    rec_every = max(1, round(1.0 / (env.step_dt * 50)))

dt = env.step_dt
obs, _ = env.reset()
phase, t_phase = "approach", 0.0
log = []
fell = False
with torch.inference_mode():
    for k in range(int(args.seconds / dt)):
        t = k * dt
        d = robot.data
        wheel_pos = d.body_pos_w[0, wheel_bodies]                 # (2, 3)
        wx = float(wheel_pos[:, 0].mean())
        h_now = cad.leg_state(robot)[0][0]                         # (2,) 다리 관절값
        # --- 명령과 다리 덮어쓰기 -------------------------------------------------------------------
        vx, h_target = args.v, None
        if args.mode == "jump":
            if phase == "approach" and wx >= args.edge - args.trigger:
                phase, t_phase = "crouch", t
            elif phase == "crouch" and t - t_phase >= args.t_crouch:
                phase, t_phase = "launch", t
            elif phase == "launch" and (float(h_now.min()) >= H_MAX - 0.008 or t - t_phase >= 0.25):
                phase, t_phase = "retract", t
            elif phase == "retract" and t - t_phase >= args.t_air:
                phase, t_phase = "land", t
            elif phase == "land" and t - t_phase >= 0.4:
                phase, t_phase = "policy", t
            h_target = {"crouch": H_MIN, "launch": H_MAX, "retract": H_MIN, "land": args.h_land}.get(phase)
            if phase == "policy" and t - t_phase > 1.0:
                vx = 0.0                                           # 올라간 뒤 멈춘다 (윗면 20–50 cm)
        elif args.mode == "stance":
            vx = 0.0
            if t < 1.0:
                phase = "stand"
            elif t < 2.0:                                          # 1 s 동안 오른쪽 바퀴 위로 무게 옮기기 (왼쪽 다리 길게, 오른쪽 짧게)
                phase = "lean"
            else:
                phase = "lift"
        cmd.set(torch.tensor([vx], device=dev), torch.tensor([0.0], device=dev), torch.tensor([H_CRUISE], device=dev),
                mode=torch.tensor([0.0], device=dev))
        act = policy(obs["policy"]).clone()
        h_ref = float(cmd.command[0, 2])
        if args.mode == "jump" and h_target is not None:
            act[0, :2] = (h_target - h_ref) / 0.12
        elif args.mode == "stance" and phase != "stand":
            s = min(1.0, (t - 1.0) / 1.0)
            hl, hr = H_CRUISE + 0.045 * s, H_CRUISE - 0.045 * s     # 좌우 길이차 9 cm -> 약 24 deg 기울기
            if phase == "lift":
                hl = H_MIN                                         # 왼쪽 다리를 최대로 접어 바퀴를 든다
            act[0, 0], act[0, 1] = (hl - h_ref) / 0.12, (hr - h_ref) / 0.12
        obs, _, term, _, _ = env.step(act)
        if bool(term[0]) and not fell:
            fell = True
            fell_t = t
        # --- 기록 ------------------------------------------------------------------------------------
        g = d.projected_gravity_b[0]
        tau = d.applied_torque[0, leg_ids] * hip_sign
        mvel = d.joint_vel[0, leg_ids] * hip_sign
        log.append(dict(t=t, phase=phase, wx=wx, wz_l=float(wheel_pos[0, 2]), wz_r=float(wheel_pos[1, 2]),
                        bz=float(d.root_com_pos_w[0, 2]), pitch=math.degrees(math.asin(max(-1, min(1, float(g[0]))))),
                        roll=math.degrees(math.asin(max(-1, min(1, float(g[1]))))),
                        hl=float(h_now[0]), hr=float(h_now[1]), tau_l=float(tau[0]), tau_r=float(tau[1]),
                        w_l=float(mvel[0]), w_r=float(mvel[1])))
        if rec is not None and k % rec_every == 0:
            rec.append_data(np.asarray(env.render())[..., :3])

if rec is not None:
    rec.close()

# --- 판정 -----------------------------------------------------------------------------------------
L = log
bottom = np.array([min(r["wz_l"], r["wz_r"]) - R for r in L])        # 두 바퀴 중 낮은 쪽 바닥 높이
tau_max = max(max(abs(r["tau_l"]), abs(r["tau_r"])) for r in L)
w_max = max(max(abs(r["w_l"]), abs(r["w_r"])) for r in L)
res = dict(mode=args.mode, hip=args.hip, height=args.step_h, length=args.length, v=args.v, trigger=args.trigger,
           fell=fell, tau_hip_max_nm=tau_max, hip_speed_max_rad_s=w_max,
           current_max_a_kt060=tau_max / 0.5994, current_max_a_kt081=tau_max / 0.81)
if args.mode in ("jump", "none"):
    on_top = [r for r in L[-int(1.0 / dt):] if args.edge + 0.02 < r["wx"] < args.edge + args.length
              and min(r["wz_l"], r["wz_r"]) - R > args.step_h - 0.01]
    res["success"] = (not fell) and len(on_top) > 0.8 * int(1.0 / dt)
    res["wheel_bottom_max_mm"] = float(bottom.max() * 1000)
    cross = [r for r in L if abs(r["wx"] - args.edge) < 0.03]           # 모서리 위를 지날 때
    res["edge_clearance_mm"] = float(min(min(r["wz_l"], r["wz_r"]) - R - args.step_h for r in cross) * 1000) if cross else None
    air = [r for r in L if r["phase"] in ("launch", "retract", "land")]
    res["pitch_range_in_jump_deg"] = [float(min(r["pitch"] for r in air)), float(max(r["pitch"] for r in air))] if air else None
    res["final_x"] = L[-1]["wx"]
else:
    lift = [r for r in L if r["phase"] == "lift"]
    ok = [r for r in lift if abs(r["roll"]) < 40]
    res["roll_at_lift_start_deg"] = lift[0]["roll"] if lift else None
    res["left_wheel_lift_mm"] = float(max(r["wz_l"] - R for r in lift) * 1000) if lift else None
    res["stance_time_s"] = (fell_t - 2.0) if fell else (len(ok) * dt)
    res["roll_drift_deg"] = [float(min(r["roll"] for r in lift)), float(max(r["roll"] for r in lift))] if lift else None
print("\n" + json.dumps(res, ensure_ascii=False, indent=1), flush=True)
out = args.out or os.path.expanduser(f"~/pv_out/climb/{args.mode}_{args.hip}_{int(args.step_h*1000)}mm.json")
os.makedirs(os.path.dirname(out), exist_ok=True)
json.dump(dict(result=res, log=L[:: max(1, len(L) // 2000)]), open(out, "w"), ensure_ascii=False)
print(f"저장: {out}" + (f"\n영상: {rec_path}" if rec is not None else ""), flush=True)
env.close()
app.close()
