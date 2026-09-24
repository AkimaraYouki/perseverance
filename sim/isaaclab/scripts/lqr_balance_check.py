"""VMC+LQR 게인 스케줄 Isaac 검증.

perseverance/sim/control/gains.yaml 의 K(h) 다항식을 그대로 써서 바퀴 토크를 낸다.
다리는 시뮬에서 위치제어(직선관절)라 VMC 자체는 여기서 검증하지 않는다 —
고정 높이 5개에서 직진 LQR 만 본다. 속도 커맨드는 무시(x_d = v_d = 0).

    WB_STAGE=1 ... lqr_balance_check.py --gains <gains.yaml>   # 쉬운 조건
    WB_STAGE=2 ... lqr_balance_check.py --gains <gains.yaml>   # ±8.6° 시작, 외란, 질량/COM 랜덤화
"""
import argparse
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--gains", default=os.path.expanduser("~/perseverance/sim/control/gains.yaml"))
ap.add_argument("--repeats", type=int, default=16)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import sys  # noqa: E402

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


def log(*a):
    print(*a, flush=True)


g = yaml.safe_load(open(args.gains))["lqr"]["poly_p0_p1_p2"]
P = torch.tensor([g["theta"], g["theta_dot"], g["x"], g["x_dot"]])      # (4,3)

R_WHEEL = 0.060
HEIGHTS = [0.1825, 0.2125, 0.2425, 0.2725, 0.3025]                      # h = 지면~고관절
N = len(HEIGHTS) * args.repeats
DT = 1.0 / 200.0
HORIZON = int(20.0 / DT)
STAGE = int(os.environ.get("WB_STAGE", "1"))

cfg = parse_env_cfg("Isaac-WheeledBiped-Balance-v0", num_envs=N, use_fabric=True)
cfg.terminations.time_out = None
env = gym.make("Isaac-WheeledBiped-Balance-v0", cfg=cfg).unwrapped
robot = env.scene["robot"]
dev = env.device
P = P.to(dev)

WHEEL_SCALE = float(cfg.actions.wheels.scale)
LEG_SCALE = float(cfg.actions.legs.scale)
LEG_OFFSET = float(cfg.actions.legs.offset)
h = torch.tensor([hh for hh in HEIGHTS for _ in range(args.repeats)], device=dev)
leg_act = ((h - R_WHEEL) - LEG_OFFSET) / LEG_SCALE
log(f"[정보] STAGE {STAGE}, env {N}, 바퀴 스케일 {WHEEL_SCALE} Nm, 다리 액션 범위 "
    f"{leg_act.min().item():.2f}~{leg_act.max().item():.2f}")
if leg_act.abs().max() > 1.0:
    log("[경고] 다리 액션이 ±1 을 넘는다 — 해당 높이는 도달하지 못하고 포화된다")

H3 = torch.stack([torch.ones_like(h), h, h * h], dim=1)                # (N,3)
K = H3 @ P.T                                                           # (N,4)

env.reset()
x = torch.zeros(N, device=dev)
alive = torch.full((N,), float(HORIZON), device=dev)
dead = torch.zeros(N, dtype=torch.bool, device=dev)
max_th = torch.zeros(N, device=dev)
max_tau = torch.zeros(N, device=dev)
sat = torch.zeros(N, device=dev)

for step in range(HORIZON):
    th = torch.asin(robot.data.projected_gravity_b[:, 0].clamp(-1, 1))
    thd = robot.data.root_ang_vel_b[:, 1]
    xd = robot.data.root_lin_vel_b[:, 0]
    s = torch.stack([th, thd, x, xd], dim=1)
    T = -(K * s).sum(dim=1)                                            # 두 바퀴 합 [Nm]
    per = T / 2.0
    a = per / WHEEL_SCALE
    sat += (a.abs() > 1.0).float()
    act = torch.zeros((N, env.action_space.shape[1]), device=dev)
    act[:, 0] = leg_act
    act[:, 1] = leg_act
    act[:, 2] = a.clamp(-1, 1)
    act[:, 3] = a.clamp(-1, 1)
    _, _, term, trunc, _ = env.step(act)
    x = x + xd * DT
    live = ~dead
    max_th = torch.where(live, torch.maximum(max_th, th.abs()), max_th)
    max_tau = torch.where(live, torch.maximum(max_tau, per.abs()), max_tau)
    newly = (term | trunc) & live
    alive[newly] = float(step + 1)
    dead |= (term | trunc)
    x = torch.where(term | trunc, torch.zeros_like(x), x)
    if step % 1000 == 0:
        log(f"  step {step}  생존 {int((~dead).sum())}/{N}")

log("")
log("=" * 78)
log(f"  LQR 게인 스케줄 검증 — STAGE {STAGE}, 높이별 {args.repeats}개, 20 s")
log("=" * 78)
log(f"{'h[mm]':>7}{'완주':>8}{'평균생존s':>10}{'최대기울기°':>12}{'최대토크Nm/바퀴':>16}{'포화비율':>10}")
log("-" * 78)
allok = True
for i, hh in enumerate(HEIGHTS):
    sl = slice(i * args.repeats, (i + 1) * args.repeats)
    done = int((alive[sl] >= HORIZON).sum())
    allok &= done == args.repeats
    log(f"{hh*1000:7.1f}{done:>5}/{args.repeats:<3}{alive[sl].mean().item()*DT:10.2f}"
        f"{torch.rad2deg(max_th[sl]).max().item():12.2f}{max_tau[sl].max().item():16.3f}"
        f"{(sat[sl] / alive[sl]).mean().item()*100:9.1f}%")
log("-" * 78)
log(f"  => {'전 높이 완주' if allok else '완주 못한 높이 있음'}")
log("=" * 78)
env.close()
app.close()
