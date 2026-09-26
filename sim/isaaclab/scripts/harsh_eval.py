"""가혹 평가: 기본 LQR+VMC (보정 0, 또는 --policy) 를 로봇 수천 대로 20 s, 실기 조건 전부 + 강도 배율.

GPU 벡터 제어기 (tasks/balance/residual.py, 창·시험 세트와 같은 로직) 로 돈다. 로봇마다 첫 에피소드에서 넘어졌는지만 센다.
실기 조건 (로봇·에피소드마다 무작위, --harsh 배율):
  IMU 기울기 잡음 0.3 deg, 바이어스 ±0.5 deg, 자이로 잡음 0.01 rad/s, 바퀴 엔코더 잡음 0.05 rad/s
  제어 지연 --delay_ms (기본 5) + 매 스텝 40 % 확률로 한 주기 더 (지터)
  몸통 질량 ±15 %, 무게중심 x·z ±2 cm (y ±1), 바퀴 모터 한계 0~-15 % (제어기 추정 오차 2 %)
  마찰 0.5~1.0 (배율이 크면 하한이 내려감), 밀기 4~8 s 마다 ±0.3 m/s, 무작위 명령 (후진·급회전·정지)
지형 (난이도 행 0~9, 열마다 종류): flat, stones (둥근 돌 3~12 cm), oneside (0.2 m 차선마다 돌/평지 -> 한쪽 바퀴만 돌),
  ridges_cad (대회 삼각형 2~9 cm), ridges_narrow (좁은 차선), slope_up/down (0~0.30 = 17 deg), step_down (2~9 cm 턱),
  bumps, gravel (자잘한 요철)
결과: 출발 지형별 20 s 성공률 + 95 % 신뢰구간 (Wilson), 난이도별 성공률,
  넘어진 곳 지형 기준 (로봇이 이웃 칸으로 넘어가므로): 지형마다 머문 시간당 넘어짐 -> 20 s 환산 성공률.

    isaaclab.sh -p _isaaclab_launch.py harsh_eval.py [--num_envs 2048 --harsh 1.5 --delay_ms 10 --level 9 --cmd straight]
"""
import argparse
import json
import math
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--num_envs", type=int, default=4096)
ap.add_argument("--seconds", type=float, default=20.0)
ap.add_argument("--harsh", type=float, default=1.0, help="센서 잡음·바이어스·모델 오차·모터·밀기·마찰 배율")
ap.add_argument("--delay_ms", type=float, default=5.0)
ap.add_argument("--mu", type=float, nargs=2, default=None, metavar=("MIN", "MAX"), help="바퀴 마찰 범위 (없으면 --harsh 로: 하한 1 - 0.5 x 배율)")
ap.add_argument("--jitter_prob", type=float, default=0.4, help="매 스텝 한 주기 더 늦을 확률")
ap.add_argument("--level", type=int, default=None, help="난이도 행 고정 (없으면 0~9 고르게)")
ap.add_argument("--cmd", choices=("random", "course"), default="course",
                help="course = 칸 가운데서 바깥으로 --vx 직진 (대회처럼 앞으로 달림), random = 무작위 명령 (후진·회전·정지)")
ap.add_argument("--vx", type=float, default=0.4)
ap.add_argument("--vx_slow", type=float, default=0.4, help="불연속 지형 (--slow) 에서의 속도 [m/s] (course)")
ap.add_argument("--slow", default="bumps,blocks,step_down", help="천천히 달릴 불연속 지형 (쉼표, 빈 문자열 = 없음)")
ap.add_argument("--yaw0", action="store_true", help="모두 +x 로 출발 (삼각형 길을 정면으로). 없으면 방향 무작위 (비스듬히)")
ap.add_argument("--no_push", action="store_true")
ap.add_argument("--policy", default=None)
ap.add_argument("--tag", default="")
ap.add_argument("--seed", type=int, default=1, help="로봇 무작위 (전후 비교는 같은 시드로)")
ap.add_argument("--hw", choices=("none", "now", "plan"), default="none",
                help="실기 통신·센서 추정 (docs/lab-meeting 하드웨어·통신 문서 기준): now = 모터 피드백 500 Hz·IMU 200 Hz·일반 커널, "
                     "plan = 피드백 500 Hz·IMU 500 Hz·실시간 스레드. 명령·손실 값도 바뀜 (--ctrl 로 덮어쓰기 가능)")
ap.add_argument("--terrains", default=None, help="이 지형만 (쉼표), 예: oneside")
ap.add_argument("--suite", choices=("all", "rough"), default="all", help="rough = 자동 주행 험지 11 종 (돌·자갈·파도·요철·경사)")
ap.add_argument("--trace", type=int, default=0, help="넘어진 로봇 N 대의 넘어지기 전 2 s 기록을 npz 로")
ap.add_argument("--trace_min_t", type=float, default=2.5, help="이 시각 뒤 넘어짐만 추적 (앞은 출발 착지)")
ap.add_argument("--ctrl", nargs="*", default=[], metavar="KEY=VAL", help="기본 제어기 설정 덮어쓰기")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import gymnasium as gym  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab.terrains import TerrainGeneratorCfg  # noqa: E402
from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg  # noqa: E402
from isaaclab.terrains.height_field.utils import height_field_to_mesh  # noqa: E402
from isaaclab.terrains.trimesh.mesh_terrains_cfg import MeshPlaneTerrainCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance import terrain as T  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance import cad  # noqa: E402

import terrain_gen  # noqa: E402

_cnt = [0]


@height_field_to_mesh
def _stones(difficulty, c):
    nx, ny = int(c.size[0] / c.horizontal_scale), int(c.size[1] / c.horizontal_scale)
    h = c.h_range[0] + difficulty * (c.h_range[1] - c.h_range[0])
    _cnt[0] += 1
    # oneside: 0.2 m 차선 (바퀴 간격) 하나 건너 하나에만 돌 -> 한쪽 바퀴만 돌 위. 돌은 자르지 않음
    #   (차선 가장자리를 깎으면 12 cm 돌에서 옆면이 62 deg 가 되어 비스듬히 지나면 못 넘는 벽이 됨)
    z = terrain_gen.make_heights("lane_stones" if c.kind == "oneside" else c.kind, nx, ny, c.horizontal_scale, h, 0,
                                 seed=1000 + _cnt[0])
    return np.rint(z / c.vertical_scale).astype(np.int16)


@configclass
class StonesCfg(HfTerrainBaseCfg):
    function = _stones
    kind: str = "stones"
    h_range: tuple = (0.03, 0.12)


R = T.ROUGH_TERRAINS_CFG.sub_terrains
EVAL_TERRAINS = T.ROUGH_TERRAINS_CFG.replace(
    horizontal_scale=0.05, num_rows=10, num_cols=10, curriculum=True,     # 열 = 종류 하나씩 (돌이 0.1 m 칸이면 뭉개짐)
    sub_terrains={
        "flat": MeshPlaneTerrainCfg(proportion=0.1),
        "stones": StonesCfg(proportion=0.1, kind="stones", h_range=(0.03, 0.12), **T._HF),
        "oneside": StonesCfg(proportion=0.1, kind="oneside", h_range=(0.03, 0.12), **T._HF),
        "ridges_cad": T.StaggeredRidgesTerrainCfg(proportion=0.1, height_range=(0.02, 0.09), base=0.5, period=0.5, lane_w=0.5, **T._HF),
        "ridges_narrow": T.StaggeredRidgesTerrainCfg(proportion=0.1, height_range=(0.02, 0.09), base=0.35, period=0.6, lane_w=0.2, **T._HF),
        "slope_up": R["slope_up"].replace(proportion=0.1, slope_range=(0.0, 0.30)),
        "slope_down": R["slope_down"].replace(proportion=0.1, slope_range=(0.0, 0.30)),
        "step_down": T.MeshPyramidStairsTerrainCfg(proportion=0.1, step_height_range=(0.02, 0.09), step_width=1.0,
                                                   platform_width=2.0, border_width=0.0),
        "bumps": R["bumps"].replace(proportion=0.1),
        "gravel": R["gravel"].replace(proportion=0.1),
    },
)

if args.suite == "rough":                # 자동 주행 험지 (사용자: 삼각형길은 조종으로, 자갈·파도길은 자동으로 -> 험지 주행 안정성 우선)
    EVAL_TERRAINS = EVAL_TERRAINS.replace(num_cols=12, sub_terrains={
        "flat": MeshPlaneTerrainCfg(proportion=1.0),
        "stones": StonesCfg(proportion=1.0, kind="stones", h_range=(0.03, 0.12), **T._HF),
        "oneside": StonesCfg(proportion=1.0, kind="oneside", h_range=(0.03, 0.12), **T._HF),
        "gravel": R["gravel"].replace(proportion=1.0),                  # 10 cm 칸 무작위 ±1~3 cm
        "rough": R["rough"].replace(proportion=1.0),                    # 25 cm 칸 0.5~3.5 cm
        "rugged": R["rugged"].replace(proportion=1.0),                  # 2 m 기복 + 40 cm 요철 + 자갈
        "wave_long": R["wave_long"].replace(proportion=1.0),            # 파도 0~10 cm, 4 개
        "wave_short": R["wave_short"].replace(proportion=1.0),          # 파도 0~6 cm, 8 개
        "bumps": R["bumps"].replace(proportion=1.0),                    # 1~4 cm 턱 0.3~1 m
        "blocks": StonesCfg(proportion=1.0, kind="gravel", h_range=(0.01, 0.04), **T._HF),   # 10 cm 블록 0~1~4 cm, 수직 모서리 (영상에서 넘어진 지형)
        "slope_up": R["slope_up"].replace(proportion=1.0, slope_range=(0.0, 0.30)),
        "slope_down": R["slope_down"].replace(proportion=1.0, slope_range=(0.0, 0.30)),
    })
if args.terrains:
    keep = args.terrains.split(",")
    EVAL_TERRAINS.sub_terrains = {k_: v_.replace(proportion=1.0 / len(keep)) for k_, v_ in EVAL_TERRAINS.sub_terrains.items() if k_ in keep}
TASK = "Isaac-WheeledBiped-CAD-Residual-v0"
cfg = parse_env_cfg(TASK, device=args.device, num_envs=args.num_envs)
cfg.scene.terrain.terrain_generator = EVAL_TERRAINS
cfg.seed = args.seed
cfg.scene.terrain.max_init_terrain_level = 9 if args.level is None else args.level
cfg.curriculum.terrain_levels = None
cfg.episode_length_s = args.seconds
H = args.harsh
c = cfg.actions.ctrl
c.imu_tilt_noise_deg *= H; c.imu_tilt_bias_deg *= H; c.imu_gyro_noise *= H; c.enc_vel_noise *= H
c.dr_motor = min(0.6, c.dr_motor * H); c.delay_ms = args.delay_ms; c.delay_extra_prob = args.jitter_prob
c.delay_asym_ms *= H; c.loss_frame_prob *= H; c.loss_burst_prob *= H; c.imu_delay_ms *= H; c.fb_delay_ms *= H
HW = {   # 추정 근거: CAN 1 Mbit/s 부하 22 % [계산], CubeMars 업로드 50 Hz [측정] (1~500 Hz 설정 가능, 매뉴얼 5.2.1),
         # iAHRS 자이로 LPF 51.2 Hz (지연 약 3 ms) + USB 직렬 약 1 ms, 출력 sp 5 ms / 2 ms, 젯슨 일반 커널 제어 스레드 지연
    "now": dict(delay_ms=2.5, delay_asym_ms=0.5, delay_extra_prob=0.1, jitter_ms=2.5, loss_frame_prob=1e-4,
                loss_burst_prob=5e-4, loss_burst_steps=3.0, imu_latency_ms=4.0, imu_period_ms=5.0, imu_delay_ms=2.0,
                fb_latency_ms=1.0, fb_period_ms=2.0, fb_delay_ms=0.5),       # 업로드 50 Hz -> 500 Hz 로 바꿈 (사용자, 2026-09-27)
    "plan": dict(delay_ms=2.5, delay_asym_ms=0.5, delay_extra_prob=0.05, jitter_ms=2.5, loss_frame_prob=1e-5,
                 loss_burst_prob=1e-4, loss_burst_steps=2.0, imu_latency_ms=4.0, imu_period_ms=2.0, imu_delay_ms=1.0,
                 fb_latency_ms=1.0, fb_period_ms=2.0, fb_delay_ms=0.5),
}
if args.hw != "none":
    for k_, v_ in HW[args.hw].items():
        setattr(c, k_, v_)
for kv in args.ctrl:
    k_, v_ = kv.split("=")
    setattr(c, k_, type(getattr(c, k_))(float(v_)) if not isinstance(getattr(c, k_), bool) else v_.lower() in ("1", "true"))
e = cfg.events
e.base_mass.params["mass_distribution_params"] = (1.0 - 0.15 * H, 1.0 + 0.15 * H)
e.base_com.params["com_range"] = {"x": (-0.02 * H, 0.02 * H), "y": (-0.01 * H, 0.01 * H), "z": (-0.02 * H, 0.02 * H)}
mu_r = tuple(args.mu) if args.mu else (max(0.2, 1.0 - 0.5 * H), 1.0)
e.wheel_friction.params["static_friction_range"] = mu_r
e.wheel_friction.params["dynamic_friction_range"] = mu_r
if args.no_push:
    e.push = None
else:
    e.push.params["velocity_range"] = {"x": (-0.3 * H, 0.3 * H), "y": (-0.3 * H, 0.3 * H)}
if args.cmd == "course":
    cc = cfg.commands.base_velocity
    cc.ranges.lin_vel_x = (args.vx, args.vx); cc.ranges.ang_vel_z = (0.0, 0.0)
    cc.zero_vel_prob = cc.fast_turn_prob = cc.pure_axis_prob = 0.0
if args.yaw0:
    e.reset_base.params["pose_range"]["yaw"] = (0.0, 0.0)
env = gym.make(TASK, cfg=cfg).unwrapped
dev = env.device
pol = None
if args.policy:
    from policy_io import load_actor
    pol = load_actor(args.policy, dev)
t_ = env.scene.terrain
if args.level is not None:
    t_.terrain_levels[:] = args.level
    t_.env_origins[:] = t_.terrain_origins[t_.terrain_levels, t_.terrain_types]
obs, _ = env.reset()
types, levels = t_.terrain_types.clone(), t_.terrain_levels.clone()
names = list(EVAL_TERRAINS.sub_terrains.keys())
props = np.array([s.proportion for s in EVAL_TERRAINS.sub_terrains.values()])
cum = np.cumsum(props / props.sum())
col_name = [names[int(np.searchsorted(cum, cidx / EVAL_TERRAINS.num_cols + 0.001, side="right"))] for cidx in range(EVAL_TERRAINS.num_cols)]
slow_set = [x for x in args.slow.split(",") if x] if args.cmd == "course" else []
slow_mask = torch.tensor([col_name[int(ty)] in slow_set for ty in types.tolist()], device=dev)
cmd_term = env.command_manager.get_term("base_velocity")


def set_slow():
    if slow_set:
        cmd_term._cmd[slow_mask, 0] = args.vx_slow


set_slow()
robot0 = env.scene["robot"]
# 바퀴 정지 마찰 (로봇별). 재질은 몸체가 아니라 충돌 모양(shape) 마다 -> 바퀴 몸체의 모양 번호를 찾는다 (randomize_rigid_body_material 과 같은 방법)
try:
    _nsh = [robot0._physics_sim_view.create_rigid_body_view(lp).max_shapes for lp in robot0.root_physx_view.link_paths[0]]
    _off = np.cumsum([0] + _nsh)
    _wsh = [j for b_ in robot0.find_bodies(["l_wheel", "r_wheel"])[0] for j in range(_off[b_], _off[b_ + 1])]
    mu = robot0.root_physx_view.get_material_properties()[:, _wsh, 0].float().mean(1).cpu()
except Exception as ex_:                # 마찰 기록은 부가 정보 — 실패해도 평가는 계속
    print(f"  (바퀴 마찰 읽기 실패: {ex_})")
    mu = None
fell = torch.zeros(env.num_envs, dtype=torch.bool, device=dev)
fall_t = torch.full((env.num_envs,), float("nan"), device=dev)
done = torch.zeros(env.num_envs, dtype=torch.bool, device=dev)             # 첫 에피소드가 끝났나
nr, nc = t_.terrain_origins.shape[:2]
org = t_.terrain_origins.reshape(-1, 3)[:, :2]
robot = env.scene["robot"]
expo = torch.zeros(nc, device=dev)                                          # 종류별 머문 로봇-초
slow_t = torch.zeros(nc, device=dev)                                        # 종류별 턱 감속 중이던 로봇-초 (bump_slow)
fall_at = torch.zeros(nc, device=dev)                                       # 종류별 넘어진 횟수 (넘어진 곳 기준)
term_ = env.action_manager.get_term("ctrl")
snap = torch.zeros(env.num_envs, 5, device=dev)                             # 넘어지기 직전 [vx 명령, wz 명령, 바퀴 속도/한계, 들림, 기울기 deg]
W_MAX0 = 18.85
RB = 200                                                                    # 1 s 기록
ring = torch.zeros(env.num_envs, RB, 5, device=dev)
home = t_.env_origins[:, :2].clone()
out_n = 0
TR = 400                                                                    # 추적 2 s
TRN = ["wjL", "wjR", "wabsL", "wabsR", "hipL", "hipR", "tauL", "tauR", "hL", "hR", "pitch", "roll", "wz",
       "th_est", "v_est", "v_ref", "v_true", "lift", "hjL", "hjR", "wheel_zL", "wheel_zR"]
tring = torch.zeros(env.num_envs if args.trace else 1, TR, len(TRN), device=dev)
traces = []
with torch.inference_mode():
    for k in range(int(args.seconds / env.step_dt)):
        here = torch.cdist(robot.data.root_pos_w[:, :2], org).argmin(1) % nc  # 지금 서 있는 칸의 종류 (열)
        cm = env.command_manager.get_command("base_velocity")
        wfr = robot.data.joint_vel[:, term_.wheel_ids].abs().max(1).values / (W_MAX0 * term_.motor_true)
        tilt = torch.rad2deg(torch.acos((-robot.data.projected_gravity_b[:, 2]).clamp(-1, 1)))
        now = torch.stack([cm[:, 0], cm[:, 1], wfr, term_.lift.float(), tilt], 1)
        ring[:, k % RB] = now
        if args.trace:
            d_ = robot.data
            q_ = d_.root_quat_w
            psi_ = torch.atan2(2 * (q_[:, 0] * q_[:, 3] + q_[:, 1] * q_[:, 2]), 1 - 2 * (q_[:, 2] ** 2 + q_[:, 3] ** 2))
            lat_ = torch.stack([-torch.sin(psi_), torch.cos(psi_), torch.zeros_like(psi_)], 1)
            g_ = d_.projected_gravity_b
            hj_ = cad.leg_state(robot)[0].float()
            tring[:, k % TR] = torch.cat([
                d_.joint_vel[:, term_.wheel_ids] * term_.wsign,
                (d_.body_ang_vel_w[:, term_.wheel_bodies] * lat_[:, None]).sum(-1),
                d_.applied_torque[:, term_.leg_ids] * term_.hip_sign,
                term_.out[:, :2], term_.out[:, 2:4],
                torch.asin(g_[:, 0:1].clamp(-1, 1)), torch.asin(g_[:, 1:2].clamp(-1, 1)), d_.root_ang_vel_b[:, 2:3],
                term_.est[:, 0:3], (d_.root_lin_vel_b[:, 0:1]), term_.lift.float()[:, None], hj_,
                d_.body_pos_w[:, term_.wheel_bodies, 2] - cad.R_WHEEL,
            ], 1)
        a = pol(obs["policy"]) if pol else torch.zeros(env.num_envs, 4, device=dev)
        obs, _, term, trunc, _ = env.step(a)
        set_slow()
        live = ~done
        expo += torch.bincount(here[live], minlength=nc).float() * env.step_dt
        if hasattr(term_, "t_bump"):
            sl = live & (term_.t_bump > 0)
            slow_t += torch.bincount(here[sl], minlength=nc).float() * env.step_dt
        f_ = term & ~trunc & live
        fall_at += torch.bincount(here[f_], minlength=nc).float()
        fall_t[f_] = (k + 1) * env.step_dt
        if args.trace and f_.any() and len(traces) < args.trace and k * env.step_dt >= args.trace_min_t:
            for i_ in torch.nonzero(f_).flatten().tolist()[: args.trace - len(traces)]:
                traces.append(tring[i_, [(k - j) % TR for j in range(TR - 1, -1, -1)]].cpu().numpy())
        if f_.any():                        # 넘어지기 1.0~0.3 s 전 (넘어지는 순간은 바퀴가 따라 돌아 늘 포화라 원인이 아님)
            b = ring[f_][:, [(k - j) % RB for j in range(60, RB)]]          # (n, 140, 5)
            snap[f_] = torch.stack([b[:, 40, 0], b[:, 40, 1], b[:, :, 2].max(1).values, b[:, :, 3].max(1).values,
                                    b[:, 40, 4]], 1)
        left = ~done & ((robot.data.root_pos_w[:, :2] - home).abs().max(1).values > 3.7) & ~f_
        out_n += int(left.sum())
        done |= left                        # 자기 칸을 벗어남 = 그 칸 완주 (이웃 칸 벽에 부딪히는 것은 세지 않음)
        fell |= f_
        done |= term | trunc
        if k % 100 == 99 and bool(done.all()):
            break



def wilson(s, n, z=1.96):
    if n == 0:
        return (float("nan"),) * 3
    p = s / n
    den = 1 + z * z / n
    ctr = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, max(0.0, ctr - half), min(1.0, ctr + half)


rows = {}
levels = levels.cpu(); fell = fell.cpu()
for i in range(env.num_envs):
    n_ = col_name[int(types[i])]
    rows.setdefault(n_, [0, 0]); rows[n_][0] += int(not fell[i]); rows[n_][1] += 1
title = (f"[가혹 평가] {'정책 ' + os.path.basename(args.policy) if args.policy else '기본 LQR+VMC'} | {env.num_envs} 대 x {args.seconds:.0f} s | "
         f"{'실기 추정 ' + args.hw + ' | ' if args.hw != 'none' else ''}강도 x{H} 마찰 {mu_r[0]:.2f}~{mu_r[1]:.2f} 지연 {args.delay_ms:.0f} ms + 지터 {args.jitter_prob:.0%} | 난이도 {args.level if args.level is not None else '0~9'} | "
         f"명령 {args.cmd}{f' {args.vx} m/s' if args.cmd == 'course' else ''}{f' (불연속 {args.vx_slow} m/s)' if slow_set else ''}{' 정면' if args.yaw0 else ''}{' | 밀기 없음' if args.no_push else ''} {args.tag}")
print("\n" + title)
print(f"  {'지형':14s} {'성공':>11s}   {'성공률':>7s}  95% 신뢰구간")
out = {}
for n_ in names:
    if n_ not in rows:
        continue
    s_, c_ = rows[n_]
    p, lo, hi = wilson(s_, c_)
    out[n_] = dict(ok=s_, n=c_, p=p, lo=lo, hi=hi)
    print(f"  {n_:14s} {s_:5d}/{c_:<5d}   {100*p:6.1f} %  [{100*lo:5.1f}, {100*hi:5.1f}]")
S, N = int((~fell).sum()), env.num_envs
p, lo, hi = wilson(S, N)
print(f"  {'합계':14s} {S:5d}/{N:<5d}   {100*p:6.1f} %  [{100*lo:5.1f}, {100*hi:5.1f}]", flush=True)
print("  난이도별: " + "  ".join(
    f"{lv}:{100*float((~fell[levels == lv]).float().mean()):.0f}%" for lv in range(nr) if int((levels == lv).sum()) > 0))
bins = [0.0, 0.35, 0.5, 0.65, 0.8, 1.01]
if mu is not None:
    print("  바퀴 마찰별: " + "  ".join(
    f"{lo_:.2f}~{min(hi_, 1.0):.2f}: {100*float((~fell[(mu >= lo_) & (mu < hi_)]).float().mean()):.1f}% ({int(((mu >= lo_) & (mu < hi_)).sum())})"
    for lo_, hi_ in zip(bins[:-1], bins[1:]) if int(((mu >= lo_) & (mu < hi_)).sum()) > 0))
ft = fall_t.cpu().numpy(); ft = ft[~np.isnan(ft)]
if len(ft):
    print(f"  넘어진 시각: 2 s 안 {int((ft < 2).sum())}, 2~5 s {int(((ft >= 2) & (ft < 5)).sum())}, 5 s 뒤 {int((ft >= 5).sum())}")
F = snap[fell.to(dev)].cpu().numpy()
print(f"  칸 완주 (벗어남) {out_n}/{env.num_envs}")
if len(F):
    print(f"  넘어지기 1~0.3 s 전 ({len(F)}): 바퀴 속도 한계 90 %↑ {100*np.mean(F[:, 2] > 0.9):.0f} %, 회전 명령 |wz|>1.5 {100*np.mean(np.abs(F[:, 1]) > 1.5):.0f} %, "
          f"속도 명령 |vx|>0.6 {100*np.mean(np.abs(F[:, 0]) > 0.6):.0f} %, 정지 명령 {100*np.mean((np.abs(F[:, 0]) < 0.05) & (np.abs(F[:, 1]) < 0.05)):.0f} %, 들림 상태 {100*np.mean(F[:, 3] > 0.5):.0f} %")
print(f"  넘어진 곳 기준 ({'지형':s}: 넘어짐 / 머문 로봇-분 -> 20 s 환산 성공률)")
for j in range(nc):
    m_ = float(expo[j]) / 60.0
    if m_ <= 0:
        continue
    lam = float(fall_at[j]) / (m_ * 60.0)
    sfr = float(slow_t[j]) / max(1e-9, float(expo[j]))
    out.setdefault(col_name[j], {})["at"] = dict(falls=int(fall_at[j]), robot_min=m_, p20=math.exp(-lam * 20), slow_frac=sfr)
    print(f"    {col_name[j]:14s} {int(fall_at[j]):4d} / {m_:6.0f} 분  -> {100*math.exp(-lam*20):5.1f} %" + (f"   턱 감속 {100*sfr:4.0f} % 시간" if sfr > 0 else ""))
res_dir = os.path.expanduser("~/pv_out/harsh"); os.makedirs(res_dir, exist_ok=True)
import datetime  # noqa: E402
json.dump(dict(title=title, args=vars(args), by_terrain=out, total=dict(ok=S, n=N, p=p, lo=lo, hi=hi)),
          open(os.path.join(res_dir, f"{datetime.datetime.now():%m%d_%H%M%S}.json"), "w"), ensure_ascii=False, indent=1)
if traces:
    tp = os.path.join(res_dir, f"trace_{datetime.datetime.now():%m%d_%H%M%S}.npz")
    np.savez(tp, x=np.stack(traces), names=np.array(TRN), dt=env.step_dt)
    print(f"  추적 {len(traces)} 대 -> {tp}")
print("[가혹 평가 끝]", flush=True)
env.close()
app.close()
