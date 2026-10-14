#!/usr/bin/env python3
"""Golden data for wb_core: runs sim/isaaclab/scripts/wbctrl.py (the spec) on synthetic frames, N = 1,
est = sensors with all noise / delay set to 0, no jump edges. Writes lqr.csv and golden.csv next to this file.
GOLDEN_ROBOT=1 -> golden_robot.csv (robot additions). GOLDEN_JUMP=1 -> golden_jump.csv (robot additions + jump requests,
the operator jump path of the state machine: RETRACT, EXTRACT, FLY, DESCEND, LAND, back to DRIVE; refused requests too)."""
import math
import os
import sys
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '../../..'))
sys.path.insert(0, os.path.join(WS, 'sim/isaaclab/scripts'))
# GOLDEN_SPIN=1 -> golden_jump_spin.csv: the jump case with spin_match_k 0.25 + air_w_max 5 (wbctrl from WBCTRL_DIR, e.g. a
# `git show origin/desktop/jump-rl:sim/isaaclab/scripts/wbctrl.py` copy, until that wbctrl is merged)
SPIN = os.environ.get('GOLDEN_SPIN') == '1'
if os.environ.get('WBCTRL_DIR'):
    sys.path.insert(0, os.environ['WBCTRL_DIR'])
import lqr_vmc  # noqa: E402
import wbctrl  # noqa: E402

P = SimpleNamespace(
    est='sensors', imu_tilt_noise_deg=0.0, imu_tilt_bias_deg=0.0, imu_gyro_noise=0.0, enc_vel_noise=0.0,
    imu_acc_noise=0.0, v_lpf_hz=10.0, contact_tau_min=0.8, bal_adapt=0.3, bal_adapt_max_deg=8.0,
    roll_rate_lpf_hz=8.0, vmc_kp=60.0, vmc_kd=1.0, vmax_kmh=3.5, vmax_motor_frac=0.75, speed_lpf_hz=4.0,
    brake_kp=1.0, brake_ki=5.5, accel_max=1.5, speed_guard=0.8, bump_slow=False, turn_slow=True,
    turn_limit=True, wheel_margin=0.85, yaw_kd=0.5, wheel_tau_max=7.0, land_sf_min=0.4, lift_detect_s=0.3,
    land_detect_s=0.02, lift_wheel_kd=0.05, turn_lean=1.0, roll_kp=1.5, roll_ki=15.0, roll_kd=0.3,
    roll_leak=0.5, level_max=0.10, roll_freeze_deg=20.0, idle_h=0.1825, fric_comp_nm=0.11, fric_comp_w=0.5,
    fric_comp_static_nm=0.0, fric_comp_cmd_nm=0.0, fric_comp_cmd_w=0.05, wheel_lpf_hz=20.0, db_comp='sigma',
    db_comp_nm=0.65, db_comp_eps=0.05, delay_ms=0.0, jitter_ms=0.0,
    # jump state machine params (never triggered: no edges, jump=False)
    trigger=0.3, jump_min_v=0.2, jump_max_wz=1.0, t_retract=0.25, t_tuck=0.12, contact_tau=2.0, t_fly_max=0.6,
    land_s=0.3, retract_lean=3.0, h_land=0.1825, land_kp=20.0, land_kd=1.0, leg_kp=60.0, leg_kd=1.5,
    pd_from='extract', extract_wheels='pd', air_ctrl=True, air_pitch=-5.0, land_pitch=0.0, extract_pitch=0.0,
    air_kp=30.0, air_kd=5.0, air_tau=7.0, bump_rate=1.5, bump_acc=4.0, bump_hold_s=2.0, bump_vmax=0.45)
M_PEND = 4.02
lqr = lqr_vmc.WheelLQR(M_PEND, 0.0293, 0.145, 2 * (1.8414e-3 + 4.5e-4), wbctrl.R_WHEEL)
JUMP = os.environ.get('GOLDEN_JUMP') == '1' or SPIN
ROBOT = os.environ.get('GOLDEN_ROBOT') == '1' or JUMP
if ROBOT:   # robot additions (2026-10-09): S-curve, x_leak, lean_ff, x_hold_moving off, yaw integral
    P.jerk_max, P.accel_max, P.x_hold_moving, P.x_leak_moving, P.lean_ff = 3.0, 0.8, False, 0.5, 1.0
    P.yaw_kd, P.yaw_ki, P.yaw_i_max = 0.15, 1.0, 0.5
if SPIN:
    P.spin_match_k, P.air_w_max = 0.25, 5.0
ctl = wbctrl.WBController(P, lqr, M_PEND, n=1, seed=0)

with open(os.path.join(HERE, 'lqr.csv'), 'w') as f:
    for l, K in zip(lqr.l_grid, lqr.K):
        f.write(f'{l!r},{K[0]!r},{K[1]!r},{K[2]!r},{K[3]!r}\n')

rng = np.random.default_rng(1)
N = 3000
cols = ['t', 'gx_b', 'gy_b', 'gz_b', 'wx', 'wy', 'wz_b', 'hL', 'hR', 'tauL', 'tauR', 'wjL', 'wjR', 'waL', 'waR',
        'th_kin', 'l_pend', 'sf', 'motor_scale', 'vx', 'wzc', 'h_ref', 'h_mid',
        'a0', 'a1', 'a2', 'a3', 'kp', 'kd', 'ff', 'th', 'v', 'lifted'] + (['jump', 'phase'] if JUMP else [])
pitch = roll = 0.0
w = np.zeros(2)
rows = []
for k in range(N):
    t = k * wbctrl.DT
    pitch += rng.normal(0, 0.004); pitch *= 0.995
    roll += rng.normal(0, 0.003); roll *= 0.99
    seg = (k // 300) % 10
    vx = [0.0, 0.0, 0.3, 0.8, 1.2, -0.4, 0.0, 0.5, 0.0, 0.2][seg]
    wzc = [0.0, 0.0, 0.0, 1.0, 2.5, 0.0, -1.5, 0.5, 0.0, 0.0][seg]
    w += rng.normal(0, 0.3, 2); w *= 0.98
    if seg == 4:
        w += 0.6            # drive wheels towards the speed guard
    lifted = seg == 6 and (k % 300) < 200          # unloaded legs -> lift detection, then landing
    tau = np.array([0.1, -0.2]) if lifted else 2.0 + rng.normal(0, 0.3, 2)
    jump = False
    if JUMP:
        # jump case: rolling forward ~0.45 m/s (v >= jump_min_v) in segments 2, 3, 7; request on step 40 of every segment
        # (segments 0, 1, 5, 6, 8 refuse it: slow, reversing or lifted). Landing: hip torque low in the air, a spike at touch-down
        # in odd segments (contact_tau path), none in even ones (t_fly_max path). Legs reach H_MAX in segment 3 (early take-off).
        kk = k % 300
        if seg in (2, 3, 7):
            w = np.array([6.4, 6.4]) + rng.normal(0, 0.2, 2)
        jump = kk == 40
        if seg in (2, 3, 7) and 100 < kk < 220:
            tau = 0.5 + rng.normal(0, 0.1, 2)                     # unloaded legs in the air
            if seg % 2 == 1 and kk > 170:
                tau = np.array([3.0, 2.6])                       # touch-down spike

    if seg == 8 and (k % 300) > 150:
        tau[1] = 0.3                                # one wheel unloaded (airborne flag, roll freeze)
    g = np.array([math.sin(pitch), math.sin(roll) * math.cos(pitch), math.cos(roll) * math.cos(pitch)])
    wb = rng.normal(0, 0.2, 3)
    h = 0.1825 + rng.normal(0, 0.01, 2)
    if JUMP and seg == 3 and 95 < (k % 300) < 130:
        h = np.array([0.2400, 0.2390])                         # fully extended -> EXTRACT ends on height
    th_kin = 0.02 + rng.normal(0, 0.005)
    l_pend = 0.22 + 0.05 * math.sin(k / 400)
    sf = 1.0 + rng.normal(0, 0.05) if not (seg == 6 and (k % 300) < 30) else 0.2
    ms = 1.0 if seg != 7 else 0.85
    h_mid = -1.0 if seg != 9 else 0.15
    f = wbctrl.Frame(t=t, g_b=g[None], w_b=wb[None], h=h[None], tau_hip=tau[None], w_wheel_joint=w[None].copy(),
                     w_wheel_abs=(w + rng.normal(0, 0.05, 2))[None], th_kin=np.array([th_kin]),
                     l_pend=np.array([l_pend]), wx=np.array([0.0]), wheel_z_min=np.array([0.0]), yaw=np.array([0.0]),
                     sf=np.array([sf]), truth_th=np.array([0.0]), truth_v=np.array([0.0]), motor_scale=np.array([ms]))
    act, kp, kd, ff, info = ctl.step(f, vx, wzc, 0.1825, jump=jump, h_mid=None if h_mid < 0 else h_mid)
    rows.append([t, *g, *wb, *h, *tau, *f.w_wheel_joint[0], *f.w_wheel_abs[0], th_kin, l_pend, sf, ms, vx, wzc,
                 0.1825, h_mid, *act[0], kp[0], kd[0], ff[0], info['th'][0], info['v'][0], float(ctl.lift_on[0])]
                + ([float(jump), float(ctl.phase[0])] if JUMP else []))
name = 'golden_jump_spin.csv' if SPIN else 'golden_jump.csv' if JUMP else 'golden_robot.csv' if ROBOT else 'golden.csv'
with open(os.path.join(HERE, name), 'w') as fo:
    fo.write(','.join(cols) + '\n')
    for r in rows:
        fo.write(','.join(repr(float(x)) for x in r) + '\n')
if JUMP:
    ph = [int(r[-1]) for r in rows]
    print(f'golden_jump: {N} steps, requests {sum(r[-2] for r in rows):.0f}, phase steps',
          {n: ph.count(i) for i, n in enumerate(('DRIVE', 'RETRACT', 'EXTRACT', 'FLY', 'DESCEND', 'LAND'))},
          'jumps', sum(1 for a, b in zip(ph, ph[1:]) if a == 0 and b == 1))
else:
    print(f'golden: {N} steps, lifted steps {sum(r[-1] for r in rows):.0f}')
