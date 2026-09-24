#!/usr/bin/env python3
"""Isaac 폐루프 모델(loop_check.py 로그)을 asd.py 기구학으로 검증한다.

asd.calculate_kinematics(CAD 치수, theta_kR) 가 주는 점 M, P, I, K, W 로부터
  크랭크 각   psi_c = atan2(I - M)
  생크 각     psi_s = atan2(W - I)       (K-I-W 는 한 강체)
  로커 각     psi_r = atan2(K - P)
를 만들고, Isaac 관절값과 비교한다 (Isaac 관절은 CAD 영점에서 0 이므로 영점 대비 변화량끼리 본다).
  M 관절 = psi_c 변화           = theta_kR - theta0
  I 관절 = (psi_s - psi_c) 변화
  K 관절 = (psi_r - psi_s) 변화
  다리    = h(asd) - R  vs  Isaac 고관절~바퀴중심 수직거리
그리고 각 점에서 asd 의 전달각 mu 를 같이 보인다 (설계 기준 30 deg 이상).
부호(Isaac 관절축 방향)는 두 가지 중 잘 맞는 쪽을 고른다 — 고른 결과를 출력한다.

    python3 verify_loop_with_asd.py <loop_check 로그>
"""
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path.home() / "PycharmProjects/PythonProject"))
import asd  # noqa: E402

CAD = [0.10272, 0.10072, 0.08000, 0.17098, 0.08000]   # L1..L5 (leg_map 과 같은 CAD 실측)
THETA0 = np.radians(45.002)                          # CAD 영점의 theta_kR


def angles(theta, sign):
    k = asd.calculate_kinematics(CAD, np.atleast_1d(theta), assembly_sign=sign)
    psi_c = np.arctan2(k["y_I"] - k["y_M"], k["x_I"] - k["x_M"])
    psi_s = np.arctan2(k["y_W"] - k["y_I"], k["x_W"] - k["x_I"])
    psi_r = np.arctan2(k["y_K"] - k["y_P"], k["x_K"] - k["x_P"])
    mu = np.degrees(np.arccos(np.clip(k["cos_mu"], -1, 1)))
    return psi_c, psi_s, psi_r, k["h_R_actual"], mu


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


rows = []
for line in open(sys.argv[1], encoding="utf-8"):
    m = re.search(r"모든관절 \[([^\]]+)\]", line)
    if m:
        v = [float(x) for x in m.group(1).split(",")]
        rows.append({"M": v[0], "I": v[2], "K": v[4]})
    m2 = re.search(r"다리\(고관절~바퀴\)\s+([\d.]+) mm", line)
    if m2 and rows and "leg" not in rows[-1]:
        rows[-1]["leg"] = float(m2.group(1))
rows = [r for r in rows if "leg" in r]
M = np.radians([r["M"] for r in rows]); I = np.radians([r["I"] for r in rows]); K = np.radians([r["K"] for r in rows])
legI = np.array([r["leg"] for r in rows])

best = None
for sign in (+1.0, -1.0):          # 조립 분기
    for sM in (+1, -1):            # Isaac M 축 방향
        th = THETA0 + sM * M
        c0, s0, r0, _, _ = angles(THETA0, sign)
        c, s, r, h, mu = angles(th, sign)
        dI = wrap((s - c) - (s0 - c0)[0]); dK = wrap((r - s) - (r0 - s0)[0])
        legA = (h - asd.R) * 1000
        for sI in (+1, -1):
            for sK in (+1, -1):
                e = (np.abs(np.degrees(sI * dI - I)).max(), np.abs(np.degrees(sK * dK - K)).max(), np.abs(legA - legI).max())
                score = e[0] + e[1] + e[2]
                if best is None or score < best[0]:
                    best = (score, sign, sM, sI, sK, e, th, dI * sI, dK * sK, legA, mu)
_, sign, sM, sI, sK, e, th, dI, dK, legA, mu = best
print(f"asd.py 조립분기 {sign:+.0f}, 축부호 M {sM:+d} I {sI:+d} K {sK:+d}  (asd.PHI_MP_FIXED {np.degrees(asd.PHI_MP_FIXED):.0f} deg)")
print(f"{'theta_kR':>9} | {'I asd':>7} {'I isaac':>8} {'차':>6} | {'K asd':>7} {'K isaac':>8} {'차':>6} | {'다리 asd':>8} {'isaac':>7} {'차':>6} | {'전달각':>6}")
for k in range(len(rows)):
    inr = asd_ok = (np.radians(38.0) <= th[k] <= np.radians(97.8))
    print(f"{np.degrees(th[k]):8.2f}° | {np.degrees(dI[k]):7.2f} {np.degrees(I[k]):8.2f} {np.degrees(dI[k]-I[k]):+6.2f} | "
          f"{np.degrees(dK[k]):7.2f} {np.degrees(K[k]):8.2f} {np.degrees(dK[k]-K[k]):+6.2f} | {legA[k]:8.2f} {legI[k]:7.2f} {legA[k]-legI[k]:+6.2f} | {mu[k]:6.1f}°"
          + ("" if inr else "   (설계구간 밖)"))
print(f"\n최대 오차: I {e[0]:.3f} deg, K {e[1]:.3f} deg, 다리 {e[2]:.3f} mm   (Isaac 로그 각도는 0.1 deg 단위 반올림)")
print(f"전달각 범위 {mu.min():.1f} ~ {mu.max():.1f} deg (asd 설계 하한 {np.degrees(asd.MU_MIN):.0f} deg)")
