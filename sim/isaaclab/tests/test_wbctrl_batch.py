# wbctrl 배열판: 로봇 N 대를 한 번에 = 1 대씩 따로 (잡음 끄고 같은 입력) — 창(N=1)·pv robust·pv harsh 가 같은 제어기임을 지킨다 (Isaac 불필요)
import os, sys, types
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import lqr_vmc, wbctrl as W
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "climb_test.py"), encoding="utf-8").read()
i0 = src.index("TUNE = dict("); ns = {}; exec(src[i0:src.index("\n)\n", i0) + 3], ns)
T = dict(ns["TUNE"]); T.update(imu_tilt_noise_deg=0.0, imu_tilt_bias_deg=0.0, imu_gyro_noise=0.0, enc_vel_noise=0.0, jitter_ms=0.0,
                              imu_acc_noise=0.0, bump_slow=True, turn_limit=True, fric_comp_nm=0.25)
P = types.SimpleNamespace(**T)
lqr = lqr_vmc.WheelLQR(3.9, 0.03, 0.28, 0.0046, W.R_WHEEL)
N, edges, rng = 5, [1.0, 2.0], np.random.default_rng(3)
batch = W.WBController(P, lqr, 3.9, n=N, seed=1, edges=edges)
singles = [W.WBController(P, lqr, 3.9, n=1, seed=1, edges=edges) for _ in range(N)]
wx, spd, maxd, njump, nlift = np.zeros(N), rng.uniform(0.2, 0.8, N), 0.0, 0, 0
for k in range(1500):
    t = k * W.DT; wx += spd * W.DT
    low = ((k // 150) % 4 == 3) & (np.arange(N) % 2 == 0)
    tau = rng.uniform(1.0, 5.0, (N, 2)); tau[low] = rng.uniform(-0.5, 0.5, (low.sum(), 2))
    g = rng.normal(0, 0.05, (N, 3)); g[:, 2] = -1.0
    fr = dict(g_b=g, w_b=rng.normal(0, 0.3, (N, 3)), h=rng.uniform(0.13, 0.23, (N, 2)), tau_hip=tau, w_wheel_joint=rng.normal(0, 8, (N, 2)),
              w_wheel_abs=rng.normal(5, 4, (N, 2)), th_kin=rng.normal(0, 0.05, N), l_pend=rng.uniform(0.15, 0.3, N), wx=wx.copy(),
              wheel_z_min=rng.uniform(0, 0.1, N), yaw=rng.normal(0, 0.2, N), sf=rng.uniform(0.2, 1.5, N), truth_th=rng.normal(0, 0.05, N),
              truth_v=rng.normal(0.5, 0.2, N), motor_scale=rng.uniform(0.85, 1.0, N), a_fwd=rng.normal(0, 3, N))
    vx, wz, jump, hm = rng.uniform(-1.2, 1.2, N), rng.uniform(-3, 3, N), rng.random(N) < 0.01, rng.uniform(0.15, 0.2, N)
    a, kp, kd, ff, _ = batch.step(W.Frame(t=t, **fr), vx, wz, P.idle_h, jump=jump, h_mid=hm)
    for i in range(N):
        f1 = W.Frame(t=t, **{k_: v[i:i + 1] for k_, v in fr.items()})
        a1, kp1, kd1, ff1, _ = singles[i].step(f1, vx[i], wz[i], P.idle_h, jump=jump[i], h_mid=hm[i])
        maxd = max(maxd, np.abs(a[i] - a1[0]).max(), abs(kp[i] - kp1[0]), abs(kd[i] - kd1[0]), abs(ff[i] - ff1[0]))
    nlift += int(batch.lift_on.sum())
njump = sum(len(j) for j in batch.jumps)
same_j = all(batch.jumps[i] == singles[i].jumps[0] for i in range(N))
ok = maxd < 1e-9 and same_j and njump > 0 and nlift > 0
print(("PASS" if ok else "FAIL"), f"N={N} 한 번에 = 1 대씩: 최대 차 {maxd:.1e}, 점프 {njump} 번 기록 같음 {same_j}, 들림 로봇-스텝 {nlift}")
sys.exit(0 if ok else 1)
