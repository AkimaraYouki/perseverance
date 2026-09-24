"""정책 재생 + 조종: 조이스틱 또는 6방향 순환. (오리 play_fixed_cmd.py 구조)

    --joystick [DEV]   Xbox 패드로 실시간 조종 (/dev/input/js0)
                       왼스틱 세로=전후, 왼스틱 가로=회전, 오른스틱 세로=높이, A=비상정지, B=높이 기본값
    --cycle            정지 → 전진 → 후진 → 좌회전 → 우회전 → 올라가기 → 내려가기 를 --hold 초씩
    (둘 다 없으면)     --vx --wz --h 고정 명령

명령은 pin 해서 학습용 재추첨이 덮어쓰지 않게 한다. 카메라는 로봇을 따라간다.

    isaaclab.sh -p _isaaclab_launch.py play_joy.py --policy <exported/policy.pt> --cycle
"""
import argparse
import os
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--policy", required=True)
ap.add_argument("--task", default="Isaac-WheeledBiped-Balance-Play-v0")
ap.add_argument("--num_envs", type=int, default=1)
ap.add_argument("--joystick", nargs="?", const="/dev/input/js0", default=None, metavar="DEV")
ap.add_argument("--cycle", action="store_true")
ap.add_argument("--hold", type=float, default=4.0)
ap.add_argument("--vx", type=float, default=0.0)
ap.add_argument("--wz", type=float, default=0.0)
ap.add_argument("--h", type=float, default=None, help="다리 관절값 [m] (고관절~바퀴중심). 기본=명령 기본값")
ap.add_argument("--height_rate", type=float, default=0.10, help="스틱 끝까지 밀 때 높이 변화 [m/s]")
ap.add_argument("--seconds", type=float, default=1e9)
ap.add_argument("--cam", type=float, nargs=3, default=(1.0, -1.0, 0.5))
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from wheeled_biped_isaaclab import joystick_input as J  # noqa: E402

R_WHEEL = 0.060

cfg = parse_env_cfg(args.task, num_envs=args.num_envs, use_fabric=True)
cfg.viewer.origin_type = "asset_root"
cfg.viewer.asset_name = "robot"
cfg.viewer.env_index = 0
cfg.viewer.eye = tuple(args.cam)
cfg.viewer.lookat = (0.0, 0.0, 0.1)
env = gym.make(args.task, cfg=cfg).unwrapped
dev = env.device
cmd = env.command_manager.get_term("base_velocity")
rng = cmd.cfg.ranges
cmd.pin(True)
policy = torch.jit.load(args.policy, map_location=dev).eval()
h_lo, h_hi = rng.height
h0 = args.h if args.h is not None else cmd.cfg.default_height

pad = None
if args.joystick:
    pad = J.Gamepad(args.joystick)
    pad.poll()
    print(f"[패드] {args.joystick} 배치: {pad.layout}", flush=True)

h_mid = 0.5 * (h_lo + h_hi)
CYCLE = [  # (이름, vx, wz, h)
    ("정지", 0.0, 0.0, h_mid), ("전진", 0.3, 0.0, h_mid), ("후진", -0.3, 0.0, h_mid),
    ("좌회전", 0.0, 0.8, h_mid), ("우회전", 0.0, -0.8, h_mid),
    ("올라가기", 0.0, 0.0, h_hi), ("내려가기", 0.0, 0.0, h_lo), ("복귀", 0.0, 0.0, h_mid),
]

obs, _ = env.reset()
h_t = h0
cmd.set(0.0, 0.0, h_t)
dt = env.step_dt
t0 = time.time()
k = 0
last_print = 0.0
label = ""
with torch.inference_mode():
    while app.is_running() and k * dt < args.seconds:
        sim_t = k * dt
        if pad is not None:
            pad.poll()
            vx, wz, dh, estop, reset_h = J.command_from_gamepad(pad, rng.lin_vel_x, rng.ang_vel_z)
            h_t = min(h_hi, max(h_lo, h_t + dh * args.height_rate * dt))
            if reset_h:
                h_t = cmd.cfg.default_height
            label = "비상정지" if estop else ("패드" if pad.connected else "패드 끊김→정지")
            if not pad.connected:
                vx = wz = 0.0
        elif args.cycle:
            i = int(sim_t // args.hold) % len(CYCLE)
            label, vx, wz, h_t = CYCLE[i]
        else:
            vx, wz = args.vx, args.wz
            label = "고정"
        cmd.set(vx, wz, h_t)
        obs, _, term, trunc, _ = env.step(policy(obs["policy"]))
        r = env.scene["robot"]
        if time.time() - last_print > 0.5:
            last_print = time.time()
            q = r.data.joint_pos[0, :2].mean().item()
            v = r.data.root_lin_vel_b[0, 0].item()
            w = r.data.root_ang_vel_b[0, 2].item()
            g = r.data.projected_gravity_b[0]
            pitch = torch.rad2deg(torch.asin(g[0].clamp(-1, 1))).item()
            print(f"[{label:6s}] 명령 vx {vx:+.2f} wz {wz:+.2f} h {(h_t+R_WHEEL)*1000:5.1f}mm | "
                  f"실제 vx {v:+.2f} wz {w:+.2f} h {(q+R_WHEEL)*1000:5.1f}mm pitch {pitch:+5.1f}°"
                  + ("  [넘어짐→리셋]" if bool(term[0]) else ""), flush=True)
        k += 1
        # 실시간 맞추기 (시뮬이 더 빠르면 기다린다)
        lag = k * dt - (time.time() - t0)
        if lag > 0:
            time.sleep(lag)
env.close()
app.close()
