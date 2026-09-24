"""정책 재생 + 모터/IMU 로그.

exported/policy.pt (관측 정규화 포함 jit) 를 돌리면서 매 제어 스텝(200 Hz)마다 기록한다.
  모터: 관절 위치/속도, 액션(원시), 토크 지령, 실제 적용 토크, 관절 한계
  IMU : roll/pitch/yaw, projected gravity, 몸체 각속도, 몸체 선속도, 높이
  기타: 속도 커맨드, 종료 플래그

출력 (--out 디렉터리):
  log.npz          전 env 전 채널
  env0.csv         env 0 만 CSV
  summary.txt      진단 요약 (포화, 한계, 진동 주파수 등)
  plot.png         env 0 시계열

    WB_STAGE=2 isaaclab.sh -p _isaaclab_launch.py play_log.py --policy <exported/policy.pt> [--headless]
"""
import argparse
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--policy", required=True)
ap.add_argument("--num_envs", type=int, default=8)
ap.add_argument("--seconds", type=float, default=20.0)
ap.add_argument("--out", default=None)
ap.add_argument("--task", default="Isaac-WheeledBiped-Balance-Play-v0")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
app = AppLauncher(args).app

import datetime  # noqa: E402

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


def log(*a):
    print(*a, flush=True)


out = args.out or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(args.policy))),
                               "playlog_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
os.makedirs(out, exist_ok=True)

cfg = parse_env_cfg(args.task, num_envs=args.num_envs, use_fabric=True)
env = gym.make(args.task, cfg=cfg).unwrapped
robot = env.scene["robot"]
dev = env.device
policy = torch.jit.load(args.policy, map_location=dev).eval()

names = robot.joint_names
lim = robot.data.joint_pos_limits[0].cpu().numpy()
soft = robot.data.soft_joint_pos_limits[0].cpu().numpy()
vlim = robot.data.joint_vel_limits[0].cpu().numpy() if hasattr(robot.data, "joint_vel_limits") else None
elim = robot.data.joint_effort_limits[0].cpu().numpy() if hasattr(robot.data, "joint_effort_limits") else None
log("[관절]", names)
for i, n in enumerate(names):
    log(f"  {n:16s} 위치한계 [{lim[i,0]: .4g}, {lim[i,1]: .4g}]  soft [{soft[i,0]: .4g}, {soft[i,1]: .4g}]"
        + (f"  속도한계 {vlim[i]:.4g}" if vlim is not None else "")
        + (f"  토크한계 {elim[i]:.4g}" if elim is not None else ""))
wheel_scale = float(cfg.actions.wheels.scale)
dt = env.step_dt
T = int(args.seconds / dt)
log(f"[설정] STAGE {os.environ.get('WB_STAGE', '1')}, env {args.num_envs}, dt {dt:.4f} s, {T} step, 바퀴 스케일 {wheel_scale}")

keys = ["t", "cmd", "action", "q", "qd", "tau_cmd", "tau_applied", "rpy", "grav", "angvel", "linvel", "height", "done"]
buf = {k: [] for k in keys}


def quat_to_rpy(q):  # (w,x,y,z)
    w, x, y, z = q.unbind(-1)
    roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = torch.asin((2 * (w * y - z * x)).clamp(-1, 1))
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return torch.stack([roll, pitch, yaw], -1)


obs, _ = env.reset()
with torch.inference_mode():
    for k in range(T):
        a = policy(obs["policy"])
        obs, _, term, trunc, _ = env.step(a)
        d = robot.data
        buf["t"].append(np.full(args.num_envs, (k + 1) * dt))
        buf["cmd"].append(env.command_manager.get_command("base_velocity").cpu().numpy())
        buf["action"].append(a.cpu().numpy())
        buf["q"].append(d.joint_pos.cpu().numpy())
        buf["qd"].append(d.joint_vel.cpu().numpy())
        buf["tau_cmd"].append(d.computed_torque.cpu().numpy())
        buf["tau_applied"].append(d.applied_torque.cpu().numpy())
        buf["rpy"].append(quat_to_rpy(d.root_quat_w).cpu().numpy())
        buf["grav"].append(d.projected_gravity_b.cpu().numpy())
        buf["angvel"].append(d.root_ang_vel_b.cpu().numpy())
        buf["linvel"].append(d.root_lin_vel_b.cpu().numpy())
        buf["height"].append(d.root_pos_w[:, 2].cpu().numpy())
        buf["done"].append((term | trunc).cpu().numpy())

A = {k: np.stack(v, 0) for k, v in buf.items()}   # (T, N, ...)
np.savez_compressed(os.path.join(out, "log.npz"), joint_names=np.array(names), joint_limits=lim,
                    wheel_scale=wheel_scale, dt=dt, **A)

# ---- env0 CSV
wi = [i for i, n in enumerate(names) if "wheel" in n]
li = [i for i, n in enumerate(names) if n.endswith("_leg")]
cols = ["t", "cmd_vx", "cmd_wz"] + [f"act{i}" for i in range(A["action"].shape[-1])]
for n in names:
    cols += [f"{n}_q", f"{n}_qd", f"{n}_tau_cmd", f"{n}_tau"]
cols += ["roll", "pitch", "yaw", "wx", "wy", "wz", "vx", "vy", "vz", "height", "done"]
rows = []
for k in range(T):
    r = [A["t"][k, 0], A["cmd"][k, 0, 0], A["cmd"][k, 0, 2], *A["action"][k, 0]]
    for j in range(len(names)):
        r += [A["q"][k, 0, j], A["qd"][k, 0, j], A["tau_cmd"][k, 0, j], A["tau_applied"][k, 0, j]]
    r += [*A["rpy"][k, 0], *A["angvel"][k, 0], *A["linvel"][k, 0], A["height"][k, 0], float(A["done"][k, 0])]
    rows.append(r)
np.savetxt(os.path.join(out, "env0.csv"), np.array(rows), delimiter=",", header=",".join(cols), comments="", fmt="%.6g")

# ---- 진단 요약
S = []
p = S.append
p(f"정책 {args.policy}")
p(f"STAGE {os.environ.get('WB_STAGE', '1')}, env {args.num_envs}, {args.seconds:.0f} s, dt {dt:.4f}")
p(f"낙상/리셋 횟수 (전 env) {int(A['done'].sum())}")
act = A["action"]
p("\n[액션 포화]  |a| >= 0.99 비율 (클립 전 원시 액션)")
for j in range(act.shape[-1]):
    p(f"  act{j}: 평균|a| {np.abs(act[...,j]).mean():.3f}  최대 {np.abs(act[...,j]).max():.2f}  포화 {100*(np.abs(act[...,j])>=0.99).mean():.1f} %")
p("\n[바퀴]")
for j in wi:
    tc, ta, qd = A["tau_cmd"][..., j], A["tau_applied"][..., j], A["qd"][..., j]
    flips = (np.diff(np.sign(ta), axis=0) != 0).mean() / dt
    p(f"  {names[j]}: 토크 지령 |평균| {np.abs(tc).mean():.3f} 최대 {np.abs(tc).max():.3f} Nm | 적용 최대 {np.abs(ta).max():.3f} Nm"
      f" | 지령≠적용 {100*(np.abs(tc-ta)>1e-3).mean():.1f} % | 속도 |최대| {np.abs(qd).max():.2f} rad/s"
      f" (한계 {vlim[j] if vlim is not None else 18.8:.1f}) | 토크 부호 반전 {flips:.1f} 회/s")
    x = qd[:, 0] - qd[:, 0].mean()
    f = np.fft.rfftfreq(len(x), dt); P = np.abs(np.fft.rfft(x)) ** 2
    top = f[1:][np.argsort(P[1:])[-3:][::-1]]
    p(f"      env0 바퀴속도 주요 주파수 {', '.join(f'{v:.2f}' for v in top)} Hz")
    p(f"      위치 범위 env0 {A['q'][:,0,j].min():.2f} ~ {A['q'][:,0,j].max():.2f} rad (한계 [{lim[j,0]:.3g},{lim[j,1]:.3g}])")
p("\n[다리]")
for j in li:
    q = A["q"][..., j]
    at_lo = (q <= soft[j, 0] + 1e-3).mean(); at_hi = (q >= soft[j, 1] - 1e-3).mean()
    p(f"  {names[j]}: {q.min()*1000:.1f} ~ {q.max()*1000:.1f} mm  (soft {soft[j,0]*1000:.1f}~{soft[j,1]*1000:.1f})  하한 접촉 {100*at_lo:.1f} %  상한 접촉 {100*at_hi:.1f} %")
p("\n[IMU]")
rpy = np.degrees(A["rpy"])
p(f"  pitch 평균 {rpy[...,1].mean():+.2f}°  표준편차 {rpy[...,1].std():.2f}°  |최대| {np.abs(rpy[...,1]).max():.2f}°")
p(f"  roll  |최대| {np.abs(rpy[...,0]).max():.2f}°")
wy = A["angvel"][..., 1]
p(f"  pitch 각속도 표준편차 {wy.std():.3f} rad/s")
x = rpy[:, 0, 1] - rpy[:, 0, 1].mean()
f = np.fft.rfftfreq(len(x), dt); P = np.abs(np.fft.rfft(x)) ** 2
p(f"  env0 pitch 주요 주파수 {', '.join(f'{v:.2f}' for v in f[1:][np.argsort(P[1:])[-3:][::-1]])} Hz")
p("\n[속도 추종]")
p(f"  vx 오차 |평균| {np.abs(A['linvel'][...,0]-A['cmd'][...,0]).mean():.3f} m/s,  wz 오차 |평균| {np.abs(A['angvel'][...,2]-A['cmd'][...,2]).mean():.3f} rad/s")
open(os.path.join(out, "summary.txt"), "w").write("\n".join(S) + "\n")
log("\n".join(S))

# ---- 그림
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = A["t"][:, 0]
    fig, ax = plt.subplots(6, 1, figsize=(12, 15), sharex=True)
    ax[0].plot(t, rpy[:, 0, 1], label="pitch [deg]"); ax[0].plot(t, rpy[:, 0, 0], label="roll [deg]"); ax[0].legend(); ax[0].grid()
    for j in wi:
        ax[1].plot(t, A["qd"][:, 0, j], label=f"{names[j]} vel [rad/s]")
        ax[2].plot(t, A["tau_cmd"][:, 0, j], label=f"{names[j]} tau cmd"); ax[2].plot(t, A["tau_applied"][:, 0, j], "--", label=f"{names[j]} tau applied")
    ax[1].legend(); ax[1].grid(); ax[2].set_ylabel("Nm"); ax[2].legend(); ax[2].grid()
    for j in range(act.shape[-1]):
        ax[3].plot(t, act[:, 0, j], label=f"act{j}")
    ax[3].axhline(1, c="k", lw=.5); ax[3].axhline(-1, c="k", lw=.5); ax[3].legend(); ax[3].grid()
    for j in li:
        ax[4].plot(t, A["q"][:, 0, j] * 1000, label=f"{names[j]} [mm]")
    ax[4].legend(); ax[4].grid()
    ax[5].plot(t, A["cmd"][:, 0, 0], label="cmd vx"); ax[5].plot(t, A["linvel"][:, 0, 0], label="vx")
    ax[5].plot(t, A["cmd"][:, 0, 2], label="cmd wz"); ax[5].plot(t, A["angvel"][:, 0, 2], label="wz")
    ax[5].legend(); ax[5].grid(); ax[5].set_xlabel("t [s]")
    fig.tight_layout(); fig.savefig(os.path.join(out, "plot.png"), dpi=90)
except Exception as e:
    log("그림 실패:", e)
log(f"\n저장: {out}")
env.close()
app.close()
