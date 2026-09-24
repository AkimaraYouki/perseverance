"""정책 재생 + 조종: 조이스틱 또는 6방향 순환. (오리 play_fixed_cmd.py 구조)

    --joystick [DEV]   Xbox 패드로 실시간 조종 (/dev/input/js0)
                       왼스틱 세로=전후, 오른스틱 가로=조향, RT/LT=높이 올림/내림, A=비상정지, B=높이 기본값
                       카메라: 십자키 좌우=로봇 주위 회전, 십자키 위아래=카메라 높이,
                               LB=축소(멀리) RB=확대(가까이), Y=카메라 초기화
    --cycle            정지 → 전진 → 후진 → 좌회전 → 우회전 → 올라가기 → 내려가기 를 --hold 초씩
    (둘 다 없으면)     --vx --wz --h 고정 명령

명령은 pin 해서 학습용 재추첨이 덮어쓰지 않게 한다. 카메라는 로봇을 따라간다.
뷰포트 오른쪽 위에 HUD (오리 play_fixed_cmd.py --hud 를 옮긴 것): 명령·실제·추종률,
높이, 기울기, 자이로, 바퀴 떨림 %, 바퀴 토크(마찰한계 대비), 최근 3 초 그래프. --no-hud 로 끈다.

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
ap.add_argument("--pad", choices=("auto", "classic", "modern"), default="auto",
                help="패드 축 배치 강제. 자동 판별이 모호할 때 (연결 순간 트리거를 잡고 있으면 생긴다). "
                     "유선 Xbox(xpad 드라이버) = classic")
ap.add_argument("--cycle", action="store_true")
ap.add_argument("--hold", type=float, default=4.0)
ap.add_argument("--vx", type=float, default=0.0)
ap.add_argument("--wz", type=float, default=0.0)
ap.add_argument("--h", type=float, default=None, help="다리 관절값 [m] (고관절~바퀴중심). 기본=명령 기본값")
ap.add_argument("--height_rate", type=float, default=0.10, help="스틱 끝까지 밀 때 높이 변화 [m/s]")
ap.add_argument("--seconds", type=float, default=1e9)
ap.add_argument("--cam", type=float, nargs=3, default=(1.0, -1.0, 0.5), help="초기 카메라 오프셋 (로봇 기준) [m]")
ap.add_argument("--no-hud", dest="hud", action="store_false")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from wheeled_biped_isaaclab import joystick_input as J  # noqa: E402

R_WHEEL = 0.060
FRICTION_NM = 0.82          # 바퀴 하나 마찰 한계 추정 (HUD 에서 % 로 보여준다)

# 패드 추가 입력 (고전 xpad 기준. 최신 hid/블루투스는 버튼 번호가 다를 수 있다)
DPAD_X, DPAD_Y = 6, 7
BTN_Y, BTN_LB, BTN_RB = 3, 4, 5

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

# ── 카메라 (로봇 기준 궤도) ──────────────────────────────────────────────
import math  # noqa: E402

cam0 = (math.atan2(args.cam[1], args.cam[0]), math.hypot(args.cam[0], args.cam[1]), args.cam[2])
cam = list(cam0)            # [방위각 rad, 수평거리 m, 높이 m]
vcc = getattr(env, "viewport_camera_controller", None)


def apply_cam():
    if vcc is None:
        return
    yaw, d, hz = cam
    vcc.update_view_location(eye=(d * math.cos(yaw), d * math.sin(yaw), hz), lookat=(0.0, 0.0, 0.12))


# ── HUD (오리 play_fixed_cmd.py 에서 옮김) ──────────────────────────────
HIST = 150
hist = {k: [0.0] * HIST for k in ("vx", "wz", "h", "pitch", "chat")}
hud = None
plots = {}
if args.hud:
    try:
        import omni.kit.viewport.utility as vp_utils
        import omni.ui as ui

        # 한글 글꼴은 omni.ui 에서 깨져서(2026-09-25 확인) HUD 는 영어 + DejaVu Sans Mono.
        font = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
        fst = {"font_size": 15, "color": 0xFFFFFFFF, "font": font}
        frame = vp_utils.get_active_viewport_window().get_frame("wb_hud")
        with frame:
            with ui.HStack():
                ui.Spacer()
                with ui.VStack(width=ui.Pixel(380)):
                    ui.Spacer(height=8)
                    with ui.ZStack():
                        ui.Rectangle(style={"background_color": 0xB0000000, "border_radius": 6})
                        with ui.VStack(spacing=2):
                            ui.Spacer(height=8)
                            hud = ui.Label("", style=fst, alignment=ui.Alignment.LEFT_TOP)
                            ui.Spacer(height=6)
                            for key, lab, col in (("vx", "vx", 0xFF44AAFF), ("wz", "yaw", 0xFFFF8844),
                                                  ("h", "h err", 0xFF44FF88), ("pitch", "pitch", 0xFF44FFFF),
                                                  ("chat", "chatter", 0xFF4444FF)):
                                with ui.HStack(height=ui.Pixel(26)):
                                    ui.Spacer(width=8)
                                    ui.Label(lab, width=ui.Pixel(80), style=fst)
                                    plots[key] = ui.Plot(ui.Type.LINE, -1.0, 1.0, *([0.0] * HIST),
                                                         width=ui.Pixel(260), height=ui.Pixel(24),
                                                         style={"color": col, "background_color": 0x30FFFFFF})
                            ui.Spacer(height=8)
                    ui.Spacer()
                ui.Spacer(width=12)
        print(f"[HUD] 활성 (font {os.path.basename(font)})", flush=True)
    except Exception as e:  # GUI 없이 돌 때 등
        print(f"[HUD] 비활성: {type(e).__name__} {e}", flush=True)


def push(key, val):
    h = hist[key]
    h.append(max(-1.0, min(1.0, val)))
    del h[0]


chat_win = []               # 바퀴 토크 변화 부호 — 오리의 CHATTER 와 같은 정의 (50 % = 백색잡음)
prev_tau = None
falls = 0

pad = None
if args.joystick:
    pad = J.Gamepad(args.joystick)
    pad.poll()
    if args.pad != "auto":
        pad.axis_right_x = J.AXIS_RIGHT_X_CLASSIC if args.pad == "classic" else J.AXIS_RIGHT_X_MODERN
        pad.layout = f"{args.pad} (강제 지정)"
        pad._layout_done = True
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
            # 카메라 — 십자키: 둘러보기, LB/RB: 확대/축소 (사용자 지정)
            dcam = False
            dx = pad.axis(DPAD_X)
            if dx:
                cam[0] += dx * 1.2 * dt; dcam = True                       # 좌우 회전
            dy = pad.axis(DPAD_Y)
            if dy:
                cam[2] = min(3.0, max(0.05, cam[2] - dy * 0.6 * dt)); dcam = True   # 위 = 높이 올림
            if pad.button(BTN_LB) or pad.button(BTN_RB):
                z = 0.8 * dt if pad.button(BTN_LB) else -0.8 * dt
                cam[1] = min(4.0, max(0.3, cam[1] * (1.0 + z))); dcam = True       # LB 멀리, RB 가까이
            if pad.button(BTN_Y):
                cam[:] = list(cam0); dcam = True
            if dcam:
                apply_cam()
        elif args.cycle:
            i = int(sim_t // args.hold) % len(CYCLE)
            label, vx, wz, h_t = CYCLE[i]
        else:
            vx, wz = args.vx, args.wz
            label = "고정"
        cmd.set(vx, wz, h_t)
        obs, _, term, trunc, _ = env.step(policy(obs["policy"]))
        r = env.scene["robot"]
        if bool(term[0]):
            falls += 1
        # --- HUD 지표 (env 0) ---
        g = r.data.projected_gravity_b[0]
        pitch_d = math.degrees(math.atan2(float(g[0]), -float(g[2])))
        roll_d = math.degrees(math.atan2(-float(g[1]), -float(g[2])))
        v_now = float(r.data.root_lin_vel_b[0, 0]); w_now = float(r.data.root_ang_vel_b[0, 2])
        q_now = float(r.data.joint_pos[0, :2].mean())
        tau = r.data.applied_torque[0, 2:4].clone()
        if prev_tau is not None:
            chat_win.append(torch.sign(tau - prev_tau))
            if len(chat_win) > 200:
                del chat_win[0]
        prev_tau = tau
        chat = 0.5
        if len(chat_win) > 2:
            S = torch.stack(chat_win)
            chat = float((S[1:] * S[:-1] < 0).float().mean())
        push("vx", v_now / 0.45); push("wz", w_now / 1.0); push("h", (q_now - float(cmd.command[0, 2])) / 0.03)
        push("pitch", pitch_d / 20.0); push("chat", (chat - 0.5) * 2.0)
        if hud is not None and k % 5 == 0:
            tmax = float(tau.abs().max())
            mode_en = {"패드": "PAD", "비상정지": "E-STOP", "패드 끊김→정지": "PAD LOST -> STOP", "고정": "FIXED",
                       "정지": "STOP", "전진": "FORWARD", "후진": "BACKWARD", "좌회전": "TURN LEFT",
                       "우회전": "TURN RIGHT", "올라가기": "RAISE", "내려가기": "LOWER", "복귀": "RETURN"}.get(label, label)
            trk_v = f"{100*v_now/vx:4.0f}%" if abs(vx) > 1e-3 else "  --"
            trk_w = f"{100*w_now/wz:4.0f}%" if abs(wz) > 1e-3 else "  --"
            hud.text = "\n".join([
                f"MODE      {mode_en}",
                f"CMD       vx {vx:+.2f}  wz {wz:+.2f}  h {(h_t+R_WHEEL)*1000:5.1f} mm",
                f"ACTUAL    vx {v_now:+.2f}  wz {w_now:+.2f}  h {(q_now+R_WHEEL)*1000:5.1f} mm",
                f"TRACKING  vx {trk_v}   wz {trk_w}   h err {(q_now-float(cmd.command[0,2]))*1000:+5.1f} mm",
                f"SPEED     {abs(v_now)*3.6:4.2f} km/h",
                "",
                f"TILT      pitch {pitch_d:+6.2f} deg  roll {roll_d:+6.2f} deg",
                f"GYRO      y {float(r.data.root_ang_vel_b[0,1]):+5.2f}  z {w_now:+5.2f} rad/s",
                f"CHATTER   {chat*100:4.1f} %  (50% = noise)",
                f"WHEEL TAU {tmax:4.2f} Nm  ({100*tmax/FRICTION_NM:3.0f}% of friction {FRICTION_NM})",
                f"FALLS     {falls}",
            ])
            for kk, pl in plots.items():
                pl.set_data(*hist[kk])
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
