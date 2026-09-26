"""harsh_eval --trace 결과 (넘어지기 전 2 s) 요약: 넘어짐 유형별로 몇 대인지.

    python3 trace_why.py ~/pv_out/harsh/trace_*.npz [--show 3]
"""
import sys

import numpy as np

f = sys.argv[1]
show = int(sys.argv[sys.argv.index("--show") + 1]) if "--show" in sys.argv else 0
z = np.load(f)
X, names, dt = z["x"], list(z["names"]), float(z["dt"])
ix = {n: i for i, n in enumerate(names)}
W = 18.85
n = len(X)
T = X.shape[1]
t = (np.arange(T) - T) * dt                                    # 넘어진 때 = 0
print(f"{n} 대, {T*dt:.1f} s 기록")
rows = []
for x in X:
    g = lambda k: x[:, ix[k]]
    unl = (g("hipL") < 0.8) | (g("hipR") < 0.8)                # 한쪽이라도 안 받침
    one = (g("hipL") < 0.8) ^ (g("hipR") < 0.8)                 # 딱 한쪽만
    w = np.maximum(np.abs(g("wjL")), np.abs(g("wjR"))) / W
    first_sat = np.argmax(w > 0.8) if (w > 0.8).any() else None
    pitch, roll = np.degrees(g("pitch")), np.degrees(g("roll"))
    tilt_start = np.argmax(np.hypot(pitch, roll) > 15) if (np.hypot(pitch, roll) > 15).any() else T - 1
    win = slice(max(0, tilt_start - 100), max(tilt_start, 1))  # 15 deg 넘기 전 0.5 s
    slip = np.maximum(np.abs(g("wabsL") * 0.06 - g("v_true")), np.abs(g("wabsR") * 0.06 - g("v_true")))
    s_on = np.argmax(slip > 0.3) if (slip[:tilt_start] > 0.3).any() else None
    rows.append(dict(
        slip_first=s_on is not None, slip_lead=(tilt_start - s_on) * dt if s_on is not None else np.nan,
        one_unl=float(one[win].mean()), any_unl=float(unl[win].mean()),
        sat_before_tilt=first_sat is not None and first_sat < tilt_start,
        sat_lead=(tilt_start - first_sat) * dt if first_sat is not None and first_sat < tilt_start else np.nan,
        dir="앞" if pitch[-1] > 0 and abs(pitch[-1]) > abs(roll[-1]) else "뒤" if abs(pitch[-1]) > abs(roll[-1]) else "옆",
        lift=bool((g("lift")[win] > 0.5).any()),
        dv=float(np.abs(g("v_est") - g("v_true"))[win].max()),
        wdiff=float(np.abs(g("wabsL") - g("wabsR"))[win].max()),
        tau_sat=float((np.maximum(np.abs(g("tauL")), np.abs(g("tauR"))) > 0.95 * np.abs(np.r_[g("tauL"), g("tauR")]).max())[win].mean()),
        hdiff=float(np.abs(g("hL") - g("hR"))[win].max()),
        roll_max=float(np.abs(roll[win]).max()) if win.stop > win.start else 0.0,
    ))
R = {k: np.array([r[k] for r in rows]) for k in rows[0]}
print(f"넘어지는 방향: 앞 {np.sum(R['dir']=='앞')}, 뒤 {np.sum(R['dir']=='뒤')}, 옆 {np.sum(R['dir']=='옆')}")
print(f"기울기 15 deg 전 0.5 s 동안: 한쪽 바퀴만 안 받침 비율 중앙 {np.median(R['one_unl'])*100:.0f} %, "
      f"그런 구간이 있던 로봇 {np.mean(R['one_unl'] > 0)*100:.0f} %")
print(f"바퀴 속도 80 % 넘은 게 기울기보다 먼저 {np.mean(R['sat_before_tilt'])*100:.0f} % (앞선 시간 중앙 {np.nanmedian(R['sat_lead']):.2f} s)")
print(f"바퀴 미끄럼 (바퀴 둘레 속도 - 몸 속도 > 0.3 m/s) 이 기울기 15 deg 보다 먼저: {np.mean(R['slip_first'])*100:.0f} % "
      f"(앞선 시간 중앙 {np.nanmedian(R['slip_lead']):.2f} s)")
print(f"들림 판정 {np.mean(R['lift'])*100:.0f} %, 속도 추정 오차 최대 중앙 {np.median(R['dv']):.2f} m/s, 좌우 바퀴 절대 회전 차 중앙 {np.median(R['wdiff']):.1f} rad/s")
print(f"좌우 다리 명령 차 최대 중앙 {np.median(R['hdiff'])*100:.1f} cm, roll 최대 중앙 {np.median(R['roll_max']):.1f} deg")
for i in range(min(show, n)):
    x = X[i]
    print(f"\n--- {i} (마지막 1 s, 0.05 s 간격) t  pitch roll | hipL hipR | wjL wjR (한계비) | tauL tauR | hL hR | v_est v_true lift")
    for k in range(T - 200, T, 10):
        r_ = {nm: x[k, ix[nm]] for nm in names}
        print(f"{t[k]:+.2f} {np.degrees(r_['pitch']):+6.1f} {np.degrees(r_['roll']):+6.1f} | {r_['hipL']:+5.1f} {r_['hipR']:+5.1f} | "
              f"{r_['wjL']/W:+.2f} {r_['wjR']/W:+.2f} | {r_['tauL']:+5.2f} {r_['tauR']:+5.2f} | {r_['hL']:.3f} {r_['hR']:.3f} | "
              f"{r_['v_est']:+.2f} {r_['v_true']:+.2f} {int(r_['lift'])}")
