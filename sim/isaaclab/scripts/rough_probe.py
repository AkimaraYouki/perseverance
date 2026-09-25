"""거친 지형 측정 — "몸통이 짐벌처럼 제자리에 고정되는가".

지형 종류 x 난이도마다 로봇을 타일 중앙 근처에 놓고 같은 명령을 준다.
    0-3 s  정지 (높이 0.1825)
    3-9 s  전진 0.5 m/s          <- 요철 통과: 짐벌 지표는 여기서 (4-9 s)
    9-12 s 정지                   <- 울퉁불퉁한 곳에서 제자리 유지 (9.5-12 s 이동거리)
    12-15 s 제자리 회전 1.0 rad/s

짐벌 지표
    바닥σ   바퀴 접지 높이 - 몸통 아래 지면 평균(60x40 cm)  의 표준편차 = 바퀴가 겪는 요철
    몸통σ   몸체 COM 높이  - 같은 지면 평균                  의 표준편차 = 몸통이 따라 움직인 양
    격리율  몸통σ / 바닥σ.  1 = 몸통이 바퀴 따라 그대로 오르내림,  0 = 완벽한 짐벌
    az      몸통 COM 수직가속도 RMS [m/s^2] (승차감),  vz  수직속도 RMS
    전달률  몸통 수직가속 RMS / 바퀴축 수직가속 RMS — **짐벌 주 지표** (사용자: 고정이 아니라 부드럽게 오르내리기).
            1 이면 바퀴 충격이 그대로 몸통에, 작을수록 다리가 걸러 준다. 격리율(변위)은 참고용.
    roll    좌우 기울기 RMS (목표 0),  pitchσ  앞뒤 흔들림,  h평균  주행 중 다리 평균 (자동 모드가 고른 높이)

높이 모드 (commands.py): 수동 = h 명령(--h) 추종, 자동 = 정책이 높이를 정함 (공칭 CAD 기본자세).
--policy 에 체크포인트(model_N.pt)나 TorchScript(policy.pt) 둘 다 줄 수 있다.

학습에 쓴 지형(seed 42)이 아니라 다른 seed 로 새로 만든 지형에서 잰다.

    isaaclab.sh -p _isaaclab_launch.py rough_probe.py --policy <policy.pt> [--levels 1 4 7 9] [--seed 7]
"""
import argparse
import json
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--policy", required=True)
ap.add_argument("--levels", type=int, nargs="+", default=[1, 4, 7, 9], help="지형 행 (난이도 약 (행+0.5)/10)")
ap.add_argument("--repeats", type=int, default=3)
ap.add_argument("--seed", type=int, default=7, help="측정용 지형 seed (학습은 42)")
ap.add_argument("--vx", type=float, default=0.5)
ap.add_argument("--wz", type=float, default=1.0)
ap.add_argument("--h", type=float, default=0.1825, help="수동 모드 높이 명령")
ap.add_argument("--modes", nargs="+", default=["manual", "auto"], choices=["manual", "auto"])
ap.add_argument("--manual_terrains", nargs="+", default=["flat"],
                help="수동 모드를 잴 지형 (사용자: 수동은 평지용). all 이면 전부")
ap.add_argument("--out", default=None)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance.terrain import ground_patch_mean, wheel_ground_mean  # noqa: E402

task = "Isaac-WheeledBiped-CAD-Rough-Play-v0"
torch.manual_seed(0)
np.random.seed(0)

# 열 -> 지형 종류 (TerrainGenerator._generate_curriculum_terrains 와 같은 규칙)
cfg0 = parse_env_cfg(task, num_envs=1)
gen = cfg0.scene.terrain.terrain_generator
names = list(gen.sub_terrains.keys())
prop = np.array([gen.sub_terrains[k].proportion for k in names])
prop = prop / prop.sum()
col_type = [int(np.min(np.where(c / gen.num_cols + 0.001 < np.cumsum(prop))[0])) for c in range(gen.num_cols)]
first_col = {names[t]: col_type.index(t) for t in range(len(names))}

CASES = [(n, lv, md) for md in args.modes for n in names for lv in args.levels
         if md == "auto" or "all" in args.manual_terrains or n in args.manual_terrains]
N = len(CASES) * args.repeats
cfg = parse_env_cfg(task, num_envs=N, use_fabric=True)
cfg.scene.terrain.terrain_generator = gen.replace(seed=args.seed)
cfg.terminations.time_out = None
cfg.events.reset_base.params["pose_range"] = {"x": (-0.4, 0.4), "y": (-0.4, 0.4), "yaw": (0.0, 0.0)}
cfg.events.reset_base.params["velocity_range"] = {}
env = gym.make(task, cfg=cfg).unwrapped
dev = env.device
robot = env.scene["robot"]
scan = SceneEntityCfg("height_scanner")
cmd = env.command_manager.get_term("base_velocity")
cmd.pin(True)
def load_policy(path):
    try:
        return torch.jit.load(path, map_location=dev).eval()
    except RuntimeError:
        pass
    sd = torch.load(path, map_location=dev, weights_only=False)["actor_state_dict"]
    dims = [sd[f"mlp.{i}.weight"].shape for i in (0, 2, 4, 6)]
    layers = []
    for i, (o, n_in) in enumerate(dims):
        lin = torch.nn.Linear(n_in, o).to(dev)
        lin.weight.data[:] = sd[f"mlp.{2*i}.weight"]
        lin.bias.data[:] = sd[f"mlp.{2*i}.bias"]
        layers += [lin] + ([torch.nn.ELU()] if i < 3 else [])
    mlp = torch.nn.Sequential(*layers).eval()
    mean, std = sd["obs_normalizer._mean"], sd["obs_normalizer._std"]
    return lambda x: mlp((x - mean) / (std + 1e-2))     # rsl_rl EmpiricalNormalization, 결정적(평균) 행동


policy = load_policy(args.policy)
wheel_ids = robot.find_bodies(["l_wheel", "r_wheel"], preserve_order=True)[0]

terr = env.scene.terrain
case_i = torch.arange(N, device=dev) // args.repeats
rows = torch.tensor([lv for _, lv, _ in CASES], device=dev)[case_i]
cols = torch.tensor([first_col[n] for n, _, _ in CASES], device=dev)[case_i]
mode_v = torch.tensor([1.0 if md == "auto" else 0.0 for _, _, md in CASES], device=dev)[case_i]
terr.terrain_levels[:] = rows
terr.terrain_types[:] = cols
terr.env_origins[:] = terr.terrain_origins[rows, cols]

dt = env.step_dt
PLAN = [(0.0, 0.0, 0.0), (3.0, args.vx, 0.0), (9.0, 0.0, 0.0), (12.0, 0.0, args.wz)]
T_END = 15.0
obs, _ = env.reset()
alive = torch.ones(N, dtype=torch.bool, device=dev)
keys = ("t", "body", "ground", "vz", "az", "azw", "rollrate", "roll", "pitch", "vx", "xy", "hleg", "alive")
rec = {k: [] for k in keys}
hv = torch.full((N,), args.h, device=dev)
with torch.inference_mode():
    for k in range(int(T_END / dt)):
        t = k * dt
        vx = wz = 0.0
        for t0, a, b in PLAN:
            if t >= t0:
                vx, wz = a, b
        cmd.set(torch.full((N,), vx, device=dev), torch.full((N,), wz, device=dev), hv, mode=mode_v)
        obs, _, term, _, _ = env.step(policy(obs["policy"]))
        alive &= ~term
        d = robot.data
        g = d.projected_gravity_b
        T0 = ground_patch_mean(env, scan)
        rec["t"].append(torch.full((N,), t, device=dev))
        rec["body"].append(d.root_com_pos_w[:, 2] - T0)   # 몸체 COM (원점은 고관절에서 옆으로 8 cm -> 기울기가 섞인다)
        rec["ground"].append(wheel_ground_mean(robot) - T0)
        rec["vz"].append(d.root_com_lin_vel_w[:, 2].clone())
        rec["az"].append(d.body_com_lin_acc_w[:, 0, 2].clone())
        rec["azw"].append(d.body_com_lin_acc_w[:, wheel_ids, 2].mean(dim=1))   # 두 바퀴축 수직가속 평균
        rec["rollrate"].append(d.root_ang_vel_b[:, 0].clone())
        rec["hleg"].append(cmd._leg_h().clone())
        rec["roll"].append(torch.rad2deg(torch.asin(g[:, 1].clamp(-1, 1))))
        rec["pitch"].append(torch.rad2deg(torch.asin(g[:, 0].clamp(-1, 1))))
        rec["vx"].append(d.root_com_lin_vel_b[:, 0].clone())
        rec["xy"].append(d.root_pos_w[:, :2].clone())
        rec["alive"].append(alive.clone())
A = {k: torch.stack(v).cpu().numpy() for k, v in rec.items()}
tt = A["t"][:, 0]
drive = (tt >= 4.0) & (tt < 9.0)
i_s0, i_s1 = int(np.searchsorted(tt, 9.5)), int(np.searchsorted(tt, 12.0)) - 1

out_rows = []
for c, (name, lv, md) in enumerate(CASES):
    sl = slice(c * args.repeats, (c + 1) * args.repeats)
    al = A["alive"][:, sl]
    ok = al[-1]                              # 끝까지 산 개체만 지표에 쓴다
    r = dict(terrain=name, level=lv, mode=md, falls=int((~ok).sum()), n=args.repeats)
    if ok.any():
        def seg(key):
            return A[key][drive][:, sl][:, ok]
        body_sd = float(seg("body").std(axis=0).mean())
        ground_sd = float(seg("ground").std(axis=0).mean())
        r.update(body_sd_mm=body_sd * 1e3, ground_sd_mm=ground_sd * 1e3,
                 ground_mean_mm=float(seg("ground").mean() * 1e3),     # 평지면 0 이어야 한다 (바퀴 접지 - 지면 평균)
                 isolation=body_sd / ground_sd if ground_sd > 0.003 else None,
                 vz_rms=float(np.sqrt((seg("vz") ** 2).mean())),
                 az_rms=float(np.sqrt((seg("az") ** 2).mean())),
                 azw_rms=float(np.sqrt((seg("azw") ** 2).mean())),
                 # 가속도 전달률: 바퀴축이 받은 수직 충격 중 몸통까지 온 비율 (사용자 정의 "짐벌 = 부드럽게")
                 acc_trans=float(np.sqrt((seg("az") ** 2).mean()) / max(np.sqrt((seg("azw") ** 2).mean()), 1e-6)),
                 rollrate_rms=float(np.degrees(np.sqrt((seg("rollrate") ** 2).mean()))),
                 h_mean_mm=float(seg("hleg").mean() * 1e3),
                 roll_rms=float(np.sqrt((seg("roll") ** 2).mean())),
                 pitch_sd=float(seg("pitch").std(axis=0).mean()),
                 vx=float(seg("vx").mean()),
                 stop_drift_cm=float(np.linalg.norm(A["xy"][i_s1, sl][ok] - A["xy"][i_s0, sl][ok], axis=1).mean() * 100))
    out_rows.append(r)

print("\n" + "=" * 124)
print(f"  거친 지형 측정  정책 {args.policy}")
print(f"  지형 seed {args.seed} (학습 42), 조건당 {args.repeats} 개체, 전진 {args.vx} m/s 구간(4-9 s)에서 짐벌 지표."
      f" 수동 높이 명령 {args.h} m")
print("=" * 124)


def f(x, fmt):
    return format(x, fmt) if x is not None else "-"


for md in args.modes:
    print(f"[{'수동' if md == 'manual' else '자동'} 모드]")
    print(f"{'지형':12s}{'행':>3}{'낙상':>6}{'실제vx':>8}{'바닥σ':>9}{'몸통σ':>9}{'격리율':>8}{'전달률':>8}{'azRMS':>8}{'vzRMS':>8}"
          f"{'rollRMS°':>10}{'pitchσ°':>9}{'h평균':>8}{'정지중이동':>11}")
    print("-" * 124)
    for r in out_rows:
        if r["mode"] != md:
            continue
        print(f"{r['terrain']:12s}{r['level']:>3}{r['falls']:>4}/{r['n']}"
              f"{f(r.get('vx'), '8.3f')}{f(r.get('ground_sd_mm'), '7.1f')}mm{f(r.get('body_sd_mm'), '7.1f')}mm"
              f"{f(r.get('isolation'), '8.2f')}{f(r.get('acc_trans'), '8.2f')}{f(r.get('az_rms'), '8.2f')}{f(r.get('vz_rms'), '8.3f')}"
              f"{f(r.get('roll_rms'), '10.2f')}{f(r.get('pitch_sd'), '9.2f')}{f(r.get('h_mean_mm'), '6.0f')}mm"
              f"{f(r.get('stop_drift_cm'), '9.1f')}cm")
    rs = [r for r in out_rows if r["mode"] == md]
    iso = [r["isolation"] for r in rs if r.get("isolation") is not None]
    print("-" * 124)
    print(f"  낙상 {sum(r['falls'] for r in rs)}/{len(rs) * args.repeats} | 격리율 평균 {np.mean(iso):.2f} (요철 조건 {len(iso)}개)"
          f" | 가속도 전달률 평균 {np.nanmean([r.get('acc_trans', np.nan) for r in rs]):.2f}"
          f" | az RMS 평균 {np.nanmean([r.get('az_rms', np.nan) for r in rs]):.2f} m/s^2"
          f" | roll RMS 평균 {np.nanmean([r.get('roll_rms', np.nan) for r in rs]):.2f}°"
          f" | 정지중 이동 평균 {np.nanmean([r.get('stop_drift_cm', np.nan) for r in rs]):.1f} cm\n")
out = args.out or os.path.splitext(args.policy)[0] + f"_rough_s{args.seed}.json"
json.dump(dict(policy=args.policy, seed=args.seed, rows=out_rows), open(out, "w"), ensure_ascii=False, indent=1)
print(f"\n저장: {out}", flush=True)
env.close()
app.close()
