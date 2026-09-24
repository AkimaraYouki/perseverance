"""PD 밸런싱 가능성 검사 — 모델 문제인가 RL 문제인가를 가른다.

RL 정책을 빼고, 기울기에 대한 손튜닝 PD 로 바퀴 토크만 준다.
  - 20 s 를 버티면 모델은 멀쩡하고 RL 설정이 문제다.
  - 어떤 게인으로도 못 버티면 모델/물리가 문제다.

부호는 유도하지 않고 양쪽 다 돌려서 살아남는 쪽을 찾는다.
바퀴 마찰 한계도 같이 실측한다(슬립률).
"""
import argparse

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import sys, traceback  # noqa: E402
import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402

DT = 1.0 / 200.0
HORIZON = int(20.0 / DT)          # 20 s = 4000 step

# (kp, kd) 격자 x 부호
GAINS = [(a, b) for a in (2.0, 5.0, 10.0, 20.0, 40.0) for b in (0.2, 0.5, 1.0, 2.0)]
CASES = [(kp, kd, s) for (kp, kd) in GAINS for s in (+1.0, -1.0)]


from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

def log(*a):
    print(*a, flush=True)
    sys.stdout.flush()


log("[단계] cfg 파싱")
cfg = parse_env_cfg("Isaac-WheeledBiped-Balance-v0", num_envs=len(CASES), use_fabric=True)
cfg.terminations.time_out = None          # 20 s 를 다 돌려본다
log("[단계] env 생성")
try:
    env = gym.make("Isaac-WheeledBiped-Balance-v0", cfg=cfg).unwrapped
except Exception:
    traceback.print_exc()
    sys.stdout.flush(); raise
log("[단계] env 준비됨")

kp = torch.tensor([c[0] for c in CASES], device=env.device)
kd = torch.tensor([c[1] for c in CASES], device=env.device)
sg = torch.tensor([c[2] for c in CASES], device=env.device)

robot = env.scene["robot"]
# 바퀴 액션 스케일은 env 설정에서 직접 읽는다. (예전엔 2.0 을 하드코딩해서,
# 설정이 1.5 로 바뀐 뒤의 결과표 게인이 실제보다 4/3 배 크게 적혀 있었다.)
WHEEL_SCALE = float(cfg.actions.wheels.scale)
log(f"[정보] 바퀴 액션 스케일 {WHEEL_SCALE} Nm/unit")
wheel_ids, wheel_names = robot.find_joints(".*_wheel_joint")
log(f"[정보] 바퀴 관절 {wheel_names} -> {wheel_ids}")
log(f"[정보] 총질량 {robot.root_physx_view.get_masses()[0].sum().item():.4f} kg")

log("[단계] reset")
obs, _ = env.reset()
log("[단계] 루프 시작")
alive = torch.zeros(len(CASES), device=env.device)
done_mask = torch.zeros(len(CASES), dtype=torch.bool, device=env.device)
max_slip = torch.zeros(len(CASES), device=env.device)

for step in range(HORIZON):
    g = robot.data.projected_gravity_b          # (N,3), 기울면 xy 성분이 생긴다
    w = robot.data.root_ang_vel_b               # (N,3)
    # 2026-09-24 축 회전 후: 전진 +x, 바퀴축 y, 불안정축 pitch(y).
    # 기우는 양은 projected_gravity 의 x 성분, 각속도는 y 성분.
    tilt = g[:, 0]
    rate = w[:, 1]
    tau = sg * (kp * tilt + kd * rate)          # [Nm]
    act = torch.zeros((len(CASES), env.action_space.shape[1]), device=env.device)
    act[:, 2] = (tau / WHEEL_SCALE).clamp(-1.0, 1.0)   # 적용 토크 = tau [Nm], ±WHEEL_SCALE 에서 포화
    act[:, 3] = act[:, 2]

    obs, rew, term, trunc, info = env.step(act)
    if step % 500 == 0:
        log(f"  step {step}  생존 {int((~done_mask).sum())}/{len(CASES)}")

    # 슬립: 바퀴 접지속도 vs 몸체 속도
    wv = robot.data.joint_vel[:, wheel_ids].mean(dim=1) * 0.06
    bv = robot.data.root_lin_vel_b[:, 1]
    max_slip = torch.maximum(max_slip, (wv + bv).abs())

    newly = (term | trunc) & (~done_mask)
    alive[newly] = step + 1
    done_mask |= (term | trunc)
    if done_mask.all():
        break

alive[~done_mask] = HORIZON

log("")
log("=" * 70)
print("  PD 밸런싱 검사 — 20 s(4000 step) 완주 여부")
print("=" * 70)
log(f"{'kp':>6}{'kd':>7}{'부호':>6}{'생존 step':>11}{'생존 s':>9}{'최대슬립 m/s':>14}")
log("-" * 70)
order = sorted(range(len(CASES)), key=lambda i: -alive[i].item())
for i in order:
    kpv, kdv, sv = CASES[i]
    log(f"{kpv:>6.1f}{kdv:>7.2f}{int(sv):>6}"
          f"{int(alive[i].item()):>11}{alive[i].item()*DT:>9.2f}{max_slip[i].item():>14.3f}")
best = order[0]
print()
log(f"  최고: kp={CASES[best][0]} kd={CASES[best][1]} sign={int(CASES[best][2])} "
      f"-> {alive[best].item()*DT:.2f} s / 20.00 s")
if alive[best].item() >= HORIZON:
    log("  => 판정: 모델은 세울 수 있다. 문제는 RL 설정 쪽.")
else:
    log("  => 판정: 손튜닝 PD 로도 못 세운다. 모델/물리를 의심해야 한다.")
log("=" * 70)

env.close()
app.close()
