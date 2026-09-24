"""6방향 × 높이 측정 — 오리 gait_compare / odm measure 의 휠-레그판.

명령을 pin 해서 조건마다 고정한다 (학습 때 재추첨이 덮어쓰지 않게).
조건: {정지, 전진, 후진, 좌회전, 우회전} × {낮음, 중간, 높음} + {올라가기, 내려가기}.
조건마다 --repeats 개 env 를 병렬로 돌린다.

판정 순위 (오리와 같다): 6방향 추종 > 안정성 > 효율.
  추종    vx/wz 평균 오차, 높이 오차, 높이 전환 90 % 도달 시간
  안정성  낙상 수, pitch 표준편차, roll 최대
  효율    바퀴 토크 RMS, 토크 부호 반전(진동) 횟수

    isaaclab.sh -p _isaaclab_launch.py measure6.py --headless --policy <exported/policy.pt> [--hard]
    --hard : 학습 STAGE 2 조건 (외란 push, 관측 잡음, 질량/COM 무작위). WB_STAGE=2 와 같이 쓸 것.
"""
import argparse
import json
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--policy", required=True)
ap.add_argument("--repeats", type=int, default=8)
ap.add_argument("--seconds", type=float, default=10.0)
ap.add_argument("--settle", type=float, default=2.0, help="이 시간 이후만 평균 [s]")
ap.add_argument("--vx", type=float, default=0.30)
ap.add_argument("--wz", type=float, default=0.80)
ap.add_argument("--vx_fast", type=float, default=0.80, help="고속 조건 vx [m/s] (0.83 = 3 km/h)")
ap.add_argument("--hard", action="store_true")
ap.add_argument("--out", default=None)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

R_WHEEL = 0.060
task = "Isaac-WheeledBiped-Balance-v0" if args.hard else "Isaac-WheeledBiped-Balance-Play-v0"

# 조건 (이름, vx, wz, h_시작, h_끝)
H = {"낮음": 0.135, "중간": 0.1825, "높음": 0.230}
CONDS = []
for hn, h in H.items():
    CONDS += [(f"정지/{hn}", 0.0, 0.0, h, h), (f"전진/{hn}", args.vx, 0.0, h, h),
              (f"후진/{hn}", -args.vx, 0.0, h, h), (f"좌회전/{hn}", 0.0, args.wz, h, h),
              (f"우회전/{hn}", 0.0, -args.wz, h, h)]
CONDS += [("올라가기", 0.0, 0.0, H["낮음"], H["높음"]), ("내려가기", 0.0, 0.0, H["높음"], H["낮음"])]
# 고속 (3 km/h 목표) — 코너에서 안쪽으로 기우는지(lean vs 목표 atan(v*wz/g))도 본다
hm = H["중간"]
CONDS += [("고속전진", args.vx_fast, 0.0, hm, hm), ("고속후진", -args.vx_fast, 0.0, hm, hm),
          ("고속좌코너", args.vx_fast, args.wz, hm, hm), ("고속우코너", args.vx_fast, -args.wz, hm, hm)]
SWITCH_T = 3.0   # 높이 전환 조건은 이 시각에 목표를 바꾼다

N = len(CONDS) * args.repeats
cfg = parse_env_cfg(task, num_envs=N, use_fabric=True)
cfg.terminations.time_out = None
if not args.hard:
    cfg.observations.policy.enable_corruption = False
env = gym.make(task, cfg=cfg).unwrapped
dev = env.device
robot = env.scene["robot"]
cmd = env.command_manager.get_term("base_velocity")
cmd.pin(True)
policy = torch.jit.load(args.policy, map_location=dev).eval()
leg_ids = robot.find_joints(".*_leg")[0]
wheel_ids = robot.find_joints(".*_wheel_joint")[0]

ci = torch.arange(N, device=dev) // args.repeats
t_vx = torch.tensor([c[1] for c in CONDS], device=dev)[ci]
t_wz = torch.tensor([c[2] for c in CONDS], device=dev)[ci]
h_a = torch.tensor([c[3] for c in CONDS], device=dev)[ci]
h_b = torch.tensor([c[4] for c in CONDS], device=dev)[ci]

dt = env.step_dt
T = int(args.seconds / dt)
S = int(args.settle / dt)
obs, _ = env.reset()
alive = torch.ones(N, dtype=torch.bool, device=dev)
rec = {k: [] for k in ("vx", "wz", "hq", "href", "pitch", "roll", "lean", "tau", "alive")}
with torch.inference_mode():
    for k in range(T):
        t = k * dt
        h_now = torch.where(torch.tensor(t >= SWITCH_T, device=dev), h_b, h_a)
        cmd.set(t_vx, t_wz, h_now)
        obs, _, term, trunc, _ = env.step(policy(obs["policy"]))
        alive &= ~term
        d = robot.data
        g = d.projected_gravity_b
        rec["vx"].append(d.root_lin_vel_b[:, 0].clone())
        rec["wz"].append(d.root_ang_vel_b[:, 2].clone())
        rec["hq"].append(d.joint_pos[:, leg_ids].mean(1).clone())
        rec["href"].append(cmd.command[:, 2].clone())
        rec["pitch"].append(torch.rad2deg(torch.asin(g[:, 0].clamp(-1, 1))))
        rec["roll"].append(torch.rad2deg(torch.asin((-g[:, 1]).clamp(-1, 1))))
        # lean: + = 왼쪽으로 기움 (projected_gravity y = +sin). 목표는 rewards.roll_lean_target 과 같은 식
        rec["lean"].append(torch.rad2deg(torch.asin(g[:, 1].clamp(-1, 1))))
        rec["tau"].append(d.applied_torque[:, wheel_ids].clone())
        rec["alive"].append(alive.clone())
A = {k: torch.stack(v).cpu().numpy() for k, v in rec.items()}   # (T, N, ...)

rows = []
for c, (name, vx, wz, ha, hb) in enumerate(CONDS):
    sl = slice(c * args.repeats, (c + 1) * args.repeats)
    al = A["alive"][:, sl]
    falls = int((~al[-1]).sum())
    m = al[S:]                                     # 넘어진 뒤 샘플은 뺀다
    def mean(x):
        x = x[S:, sl]
        return float((x * m).sum() / max(m.sum(), 1))
    vx_a, wz_a = mean(A["vx"]), mean(A["wz"])
    herr = mean(np.abs(A["hq"] - A["href"]))
    p = A["pitch"][S:, sl]
    pitch_sd = float(np.sqrt(((p - mean(A["pitch"])) ** 2 * m).sum() / max(m.sum(), 1)))
    roll_max = float(np.abs(A["roll"][S:, sl][m]).max()) if m.any() else float("nan")
    tau = A["tau"][S:, sl]                          # (t, r, 2)
    tau_rms = float(np.sqrt((tau ** 2 * m[..., None]).sum() / max(m.sum() * 2, 1)))
    flips = float((np.diff(np.sign(tau), axis=0) != 0).sum() / max(m.sum() * 2, 1) / dt)
    t90 = None
    if ha != hb:
        s0 = int(SWITCH_T / dt)
        hq = A["hq"][s0:, sl].mean(1)
        frac = (hq - ha) / (hb - ha)
        idx = np.nonzero(frac >= 0.9)[0]
        t90 = float(idx[0] * dt) if len(idx) else float("nan")
    lean = mean(A["lean"])
    lean_tgt = float(np.degrees(np.arctan(vx_a * wz_a / 9.81)))
    rows.append(dict(cond=name, cmd_vx=vx, cmd_wz=wz, vx=vx_a, wz=wz_a, err_vx=abs(vx_a - vx),
                     lean=lean, lean_tgt=lean_tgt,
                     err_wz=abs(wz_a - wz), err_h_mm=herr * 1000, t90_s=t90, falls=falls,
                     pitch_sd=pitch_sd, roll_max=roll_max, tau_rms=tau_rms, flips_per_s=flips))

print("\n" + "=" * 110)
print(f"  6방향 × 높이 측정 — {'어려운 조건(push·잡음·DR)' if args.hard else '기본 조건'}, "
      f"조건당 {args.repeats} env, {args.seconds:.0f} s (앞 {args.settle:.0f} s 제외)")
print(f"  정책 {args.policy}")
print("=" * 110)
print(f"{'조건':12s}{'명령vx':>8}{'실제vx':>8}{'명령wz':>8}{'실제wz':>8}{'높이오차':>9}{'90%도달':>8}"
      f"{'낙상':>6}{'pitchσ°':>9}{'roll최대°':>10}{'토크RMS':>9}{'반전/s':>8}")
print("-" * 110)
for r in rows:
    t90 = f"{r['t90_s']:.2f}s" if r["t90_s"] is not None else "-"
    print(f"{r['cond']:12s}{r['cmd_vx']:8.2f}{r['vx']:8.3f}{r['cmd_wz']:8.2f}{r['wz']:8.3f}"
          f"{r['err_h_mm']:7.1f}mm{t90:>8}{r['falls']:>4}/{args.repeats}{r['pitch_sd']:9.2f}"
          f"{r['lean']:+8.2f}{r['lean_tgt']:+7.2f}{r['tau_rms']:9.3f}{r['flips_per_s']:8.1f}")
mv = [r for r in rows if r["cmd_vx"] != 0]
mw = [r for r in rows if r["cmd_wz"] != 0]
print("-" * 110)
print(f"  추종  vx 오차 평균 {np.mean([r['err_vx'] for r in mv]):.3f} m/s ({100*np.mean([r['err_vx'] for r in mv])/args.vx:.0f} %)"
      f" | wz 오차 평균 {np.mean([r['err_wz'] for r in mw]):.3f} rad/s ({100*np.mean([r['err_wz'] for r in mw])/args.wz:.0f} %)"
      f" | 높이 오차 평균 {np.mean([r['err_h_mm'] for r in rows]):.1f} mm")
print(f"  좌우  |기울기-목표| 평균 {np.mean([abs(r['lean']-r['lean_tgt']) for r in rows]):.2f}° (직진·정지는 목표 0°, 코너는 atan(v*wz/g))")
print(f"  안정  낙상 {sum(r['falls'] for r in rows)}/{N} | pitch σ 최대 {max(r['pitch_sd'] for r in rows):.2f}°")
print(f"  효율  바퀴 토크 RMS 평균 {np.mean([r['tau_rms'] for r in rows]):.3f} Nm | 토크 반전 최대 {max(r['flips_per_s'] for r in rows):.1f} 회/s")
out = args.out or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(args.policy))),
                               f"measure6{'_hard' if args.hard else ''}.json")
json.dump(dict(policy=args.policy, hard=args.hard, rows=rows), open(out, "w"), ensure_ascii=False, indent=1)
print(f"\n저장: {out}")
env.close()
app.close()
