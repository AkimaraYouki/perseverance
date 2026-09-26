"""잔차 RL 환경 평가: 보정 0 (기본 LQR+VMC 만) 또는 학습된 정책으로 N 대를 T 초 돌려 지형별 넘어짐 비율.

    isaaclab.sh -p _isaaclab_launch.py residual_eval.py [--policy model_N.pt] [--num_envs 1024 --seconds 20 --level 9]
"""
import argparse
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--policy", default=None, help="없으면 보정 0 (기본 제어기만)")
ap.add_argument("--num_envs", type=int, default=1024)
ap.add_argument("--seconds", type=float, default=20.0)
ap.add_argument("--level", type=int, default=None, help="모든 로봇을 이 지형 난이도 행에 (없으면 학습 초기값)")
ap.add_argument("--no_push", action="store_true")
ap.add_argument("--delay_extra", type=float, default=None, help="지연 한 주기 더인 로봇 비율")
ap.add_argument("--friction", type=float, nargs=2, default=None)
ap.add_argument("--ctrl", nargs="*", default=[], metavar="KEY=VAL", help="기본 제어기 설정 덮어쓰기 (예: turn_lean=-1)")
ap.add_argument("--cmd", type=float, nargs=2, default=None, metavar=("VX", "WZ"), help="명령 고정")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import torch  # noqa: E402
import gymnasium as gym  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

TASK = "Isaac-WheeledBiped-CAD-Residual-v0"
cfg = parse_env_cfg(TASK, device=args.device, num_envs=args.num_envs)
cfg.curriculum.terrain_levels = None
if args.no_push:
    cfg.events.push = None
for kv in args.ctrl:
    k_, v_ = kv.split("=")
    setattr(cfg.actions.ctrl, k_, float(v_))
if args.delay_extra is not None:
    cfg.actions.ctrl.delay_extra_prob = args.delay_extra
if args.friction is not None:
    cfg.events.wheel_friction.params["static_friction_range"] = tuple(args.friction)
    cfg.events.wheel_friction.params["dynamic_friction_range"] = tuple(args.friction)
if args.cmd is not None:
    c_ = cfg.commands.base_velocity
    c_.ranges.lin_vel_x = (args.cmd[0], args.cmd[0]); c_.ranges.ang_vel_z = (args.cmd[1], args.cmd[1])
    c_.zero_vel_prob = c_.fast_turn_prob = c_.pure_axis_prob = 0.0
if args.level is not None:
    cfg.scene.terrain.max_init_terrain_level = args.level
env = gym.make(TASK, cfg=cfg).unwrapped
dev = env.device
pol = None
if args.policy:
    sd = torch.load(args.policy, map_location=dev, weights_only=False)["model_state_dict"]
    ks = sorted({k.split(".")[1] for k in sd if k.startswith("actor.")}, key=int)
    layers = []
    for j, k in enumerate(ks):
        w, b = sd[f"actor.{k}.weight"], sd[f"actor.{k}.bias"]
        lin = torch.nn.Linear(w.shape[1], w.shape[0]).to(dev); lin.weight.data[:] = w; lin.bias.data[:] = b
        layers += [lin] + ([torch.nn.ELU()] if j < len(ks) - 1 else [])
    mlp = torch.nn.Sequential(*layers).eval()
    if "actor_obs_normalizer._mean" in sd:
        mu, sg = sd["actor_obs_normalizer._mean"], sd["actor_obs_normalizer._std"]
        pol = lambda o: mlp((o - mu) / (sg + 1e-2))  # noqa: E731
    else:
        pol = mlp
if args.level is not None:
    t = env.scene.terrain
    t.terrain_levels[:] = args.level
    t.env_origins[:] = t.terrain_origins[t.terrain_levels, t.terrain_types]
obs, _ = env.reset()
types = env.scene.terrain.terrain_types.clone()
names = list(cfg.scene.terrain.terrain_generator.sub_terrains.keys())
props = [s.proportion for s in cfg.scene.terrain.terrain_generator.sub_terrains.values()]
fell = torch.zeros(env.num_envs, dtype=torch.bool, device=dev)
steps = int(args.seconds / env.step_dt)
with torch.inference_mode():
    for k in range(steps):
        a = pol(obs["policy"]) if pol else torch.zeros(env.num_envs, 4, device=dev)
        obs, _, term, trunc, _ = env.step(a)
        fell |= term & ~trunc
# 지형 종류 = 열 번호 -> 비율로 구간을 나눈 이름 (TerrainGenerator 와 같은 규칙)
import numpy as np  # noqa: E402
ncol = cfg.scene.terrain.terrain_generator.num_cols
cum = np.cumsum(np.array(props) / sum(props))
col_name = [names[int(np.searchsorted(cum, c / ncol + 0.001, side="right"))] for c in range(ncol)]
print(f"\n[평가] {'정책 ' + os.path.basename(args.policy) if args.policy else '보정 0 (기본 LQR+VMC)'}, "
      f"{env.num_envs} 대 x {args.seconds:.0f} s, 난이도 {args.level if args.level is not None else '초기'}")
by = {}
for i in range(env.num_envs):
    n = col_name[int(types[i])]
    by.setdefault(n, [0, 0]); by[n][0] += int(fell[i]); by[n][1] += 1
for n, (f, c) in sorted(by.items()):
    print(f"  {n:14s} 넘어짐 {f:4d}/{c:<4d} ({100*f/c:5.1f} %)  -> 버팀 {100*(1-f/c):5.1f} %")
tot = int(fell.sum())
print(f"  합계           넘어짐 {tot}/{env.num_envs} -> 버팀 {100*(1-tot/env.num_envs):.1f} %", flush=True)
env.close()
app.close()
