"""넘어짐 복구 환경 점검: 행동 0 / 무작위로 5 s -> 떨어진 뒤 자세 분포, 성공 판정, 보상 항 크기.

    isaaclab.sh -p recovery_probe.py [--num_envs 4096] [--mode zero|random|policy --policy model.pt]
결과: 넘어진 방향 (시작 자세: 앞 / 뒤 / 옆) x 기울기별 성공률 (tilt<11.5 deg + 각속도<1 을 0.3 s), 걸린 시간.
"""
import argparse
import math

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--num_envs", type=int, default=512)
ap.add_argument("--mode", choices=("zero", "random", "policy", "const"), default="zero")
ap.add_argument("--act", default="0,0,0,0", help="const: 고정 행동 [다리L, 다리R, 바퀴L, 바퀴R] (±1)")
ap.add_argument("--act_after", type=float, default=1.5, help="const: 이 시각부터 고정 행동 (그 전엔 0 = 떨어져 멈추기)")
ap.add_argument("--pitch", default="", help="시작 pitch 범위 'lo,hi' [rad] (기본: 환경 설정)")
ap.add_argument("--roll", default="", help="시작 roll 범위 'lo,hi' [rad]")
ap.add_argument("--policy", default="")
ap.add_argument("--seconds", type=float, default=5.0)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import gymnasium as gym  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

TASK = "Isaac-WheeledBiped-CAD-Recovery-v0"
cfg = parse_env_cfg(TASK, device=args.device, num_envs=args.num_envs)
cfg.episode_length_s = args.seconds + 1.0
cfg.terminations.stood_up = None                   # 끝까지 지켜본다 (성공 판정은 따로 센다)
cfg.rewards.success = None
if args.pitch:
    cfg.events.reset_base.params["pose_range"]["pitch"] = tuple(float(v) for v in args.pitch.split(","))
if args.roll:
    cfg.events.reset_base.params["pose_range"]["roll"] = tuple(float(v) for v in args.roll.split(","))
env = gym.make(TASK, cfg=cfg).unwrapped
from wheeled_biped_isaaclab.tasks.balance import recovery  # noqa: E402

obs, _ = env.reset()
if args.mode == "policy":
    from policy_io import load_actor
    pol = load_actor(args.policy, env.device)
N, dt = env.num_envs, env.step_dt
d = env.scene["robot"].data
p0 = recovery.fall_pitch(env)[:, 0].clone()
g0 = d.projected_gravity_b.clone()
roll0 = torch.atan2(-g0[:, 1], -g0[:, 2])
first_ok = torch.full((N,), float("nan"), device=env.device)
up_t = torch.zeros(N, device=env.device)
act = torch.zeros(N, env.action_manager.total_action_dim, device=env.device)
deg = lambda x: x.cpu().numpy() * 180 / math.pi  # noqa: E731
az0 = torch.atan2(g0[:, 1], g0[:, 0])                  # 몸 좌표에서 아래 방향: 0 = 앞으로 넘어짐, ±180 = 뒤, ±90 = 옆
tilt0 = recovery.tilt(env).clone()
for k in range(int(args.seconds / dt)):
    if args.mode == "random":
        act = torch.rand_like(act) * 2 - 1
    elif args.mode == "const":
        act[:] = torch.tensor([float(v) for v in args.act.split(",")], device=env.device) if (k + 1) * dt >= args.act_after else 0.0
        if k % int(0.5 / dt) == 0:
            tl = deg(recovery.tilt(env)); hh = recovery.root_height(env)[:, 0].cpu().numpy()
            print(f"    t {k*dt:3.1f} s  기울기 중앙 {np.median(tl):5.1f} deg (10~90% {np.percentile(tl,10):5.1f}~{np.percentile(tl,90):5.1f})"
                  f"  몸통 높이 중앙 {np.median(hh)*100:4.1f} cm", flush=True)
    elif args.mode == "policy":
        with torch.inference_mode():
            act = pol(obs["policy"])
    obs, *_ = env.step(act)
    ok = (recovery.tilt(env) < 0.2) & (d.root_ang_vel_b[:, :2].norm(dim=1) < 1.0)
    up_t = torch.where(ok, up_t + dt, torch.zeros_like(up_t))
    first_ok = torch.where(torch.isnan(first_ok) & (up_t >= 0.3), torch.full_like(first_ok, (k + 1) * dt), first_ok)
    if k == int(1.5 / dt):
        rest_p = recovery.fall_pitch(env)[:, 0].clone(); g = d.projected_gravity_b
        rest_r = torch.atan2(-g[:, 1], -g[:, 2]).clone(); rest_h = recovery.root_height(env)[:, 0].clone()
print(f"\n[복구 점검] 행동 {args.mode}, {N} 대, 5 s", flush=True)
print(f"  시작 pitch {deg(p0).min():.0f}~{deg(p0).max():.0f} deg, roll {deg(roll0).min():.0f}~{deg(roll0).max():.0f}", flush=True)
rp, rr, rh = deg(rest_p), deg(rest_r), rest_h.cpu().numpy()
print("  1.5 s 뒤 (떨어져 멈춘 자세):", flush=True)
for lo, hi in ((-180, -60), (-60, -20), (-20, 20), (20, 60), (60, 180)):
    m = (rp >= lo) & (rp < hi)
    if m.sum():
        print(f"    pitch {lo:4d}~{hi:4d}: {m.sum():4d} 대  |roll| 중앙 {np.median(np.abs(rr[m])):5.1f} deg  몸통 높이 중앙 {np.median(rh[m])*100:5.1f} cm", flush=True)
ok = ~torch.isnan(first_ok)
print(f"  '섰다' 판정 (tilt<11.5 deg, 각속도<1, 0.3 s): {int(ok.sum())}/{N} 대, 걸린 시간 중앙 "
      f"{float(first_ok[ok].median()) if ok.any() else float('nan'):.2f} s", flush=True)
azd, t0d = deg(az0), deg(tilt0)
okn, ft = ok.cpu().numpy(), first_ok.cpu().numpy()
print("  시작 자세별 (방향 x 기울기): 성공률, 걸린 시간 중앙", flush=True)
for dn, dm in (("앞", np.abs(azd) < 45), ("뒤", np.abs(azd) > 135), ("옆", (np.abs(azd) >= 45) & (np.abs(azd) <= 135))):
    row = []
    for lo, hi in ((0, 30), (30, 60), (60, 90), (90, 181)):
        m = dm & (t0d >= lo) & (t0d < hi)
        if m.sum():
            tt = ft[m & okn]
            row.append(f"{lo:3d}~{hi:3d}deg {100*okn[m].mean():5.1f}% ({m.sum():4d}) {np.median(tt) if len(tt) else float('nan'):4.2f}s")
    print(f"    {dn}: " + "  |  ".join(row), flush=True)
print("[복구 점검 끝]", flush=True)
env.close()
app.close()
