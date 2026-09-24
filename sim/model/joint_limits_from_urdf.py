#!/usr/bin/env python3
"""URDF 에서 직접 4절링크 폐루프를 풀어 관절 각도 한계를 뽑는다 -> joint_limits.json

onshape-to-robot export 는 모든 관절을 ±180° 로 내보낸다. 실제 사용 구간은
다리 높이 h ∈ [H_MIN, H_MAX] (leg_map) 에 해당하는 좁은 범위다.

CAD 를 다시 뽑으면 관절 로컬 프레임(영점)이 바뀔 수 있어서, 예전 값을 옮겨 쓰면 안 된다.
이 스크립트는 URDF 의 closing_*_P_1 (로커) == closing_*_P_2 (몸체) 구속을
수치로 풀면서 M 관절을 훑어, h 구간에 대응하는 M/I/K 각도 범위를 잰다.
관절 이름으로 결과를 쓰므로 fix_urdf.py 가 그대로 읽는다.

    python3 joint_limits_from_urdf.py <robot.urdf> [--out ../joint_limits.json]
"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).parent))
import leg_map  # noqa: E402

MARGIN = 0.05


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def rotz(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Chain:
    def __init__(self, urdf):
        r = ET.parse(urdf).getroot()
        self.J = {j.get("name"): j for j in r.findall("joint")}
        self.by_child = {j.find("child").get("link"): j for j in r.findall("joint")}

    def origin(self, j):
        o = j.find("origin")
        T = np.eye(4)
        T[:3, :3] = rpy_mat(*map(float, o.get("rpy").split()))
        T[:3, 3] = list(map(float, o.get("xyz").split()))
        return T

    def fk(self, link, q):
        """루트 기준 link 변환. q = {관절이름: 각도}. 회전관절 축은 +z 로 가정(export 형식)."""
        T = np.eye(4)
        chain = []
        while link in self.by_child:
            j = self.by_child[link]
            chain.append(j)
            link = j.find("parent").get("link")
        for j in reversed(chain):
            T = T @ self.origin(j)
            if j.get("type") in ("revolute", "continuous"):
                ax = np.array(list(map(float, j.find("axis").get("xyz").split())))
                assert np.allclose(ax, [0, 0, 1]), f"{j.get('name')} 축이 z 가 아님"
                R = np.eye(4)
                R[:3, :3] = rotz(q.get(j.get("name"), 0.0))
                T = T @ R
        return T


def sweep(ch, side):
    M, I, K, W = (f"{side}_joint_{s}" for s in "MIKW")
    p1, p2 = f"closing_{side}_joint_P_1", f"closing_{side}_joint_P_2"
    wheel = ch.J[W].find("child").get("link")
    hip = ch.J[M]

    def resid(x, qm):
        q = {M: qm, I: x[0], K: x[1]}
        return (ch.fk(p1, q)[:3, 3] - ch.fk(p2, q)[:3, 3])

    def solve(qm, guess):
        s = least_squares(resid, guess, args=(qm,), xtol=1e-12, ftol=1e-12)
        return s.x, np.linalg.norm(s.fun)

    def leg_len(q):
        # 고관절(모터축) -> 바퀴중심 의 수직(몸체 z) 거리 = h - R
        hz = ch.fk(ch.J[M].find("child").get("link"), {M: 0})[2, 3]
        wz = ch.fk(wheel, q)[2, 3]
        return hz - wz

    x0, err0 = solve(0.0, np.zeros(2))
    rows = [(0.0, *x0, leg_len({M: 0.0, I: x0[0], K: x0[1]}), err0)]
    for direction in (+1, -1):
        x = x0.copy()
        qm = 0.0
        while abs(qm) < np.pi:
            qm += direction * np.deg2rad(0.5)
            x, err = solve(qm, x)
            if err > 1e-6:
                break
            rows.append((qm, *x, leg_len({M: qm, I: x[0], K: x[1]}), err))
    rows.sort()
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--out", default=str(Path(__file__).parent.parent / "joint_limits.json"))
    a = ap.parse_args()
    ch = Chain(a.urdf)
    lo, hi = leg_map.H_MIN - leg_map.R_WHEEL, leg_map.H_MAX - leg_map.R_WHEEL
    out = {}
    for side in ("L", "R"):
        rows = sweep(ch, side)
        L0 = rows[np.argmin(np.abs(rows[:, 0]))][3]
        sel = rows[(rows[:, 3] >= lo - 1e-4) & (rows[:, 3] <= hi + 1e-4)]
        print(f"{side} 다리: CAD 영점 다리길이(고관절~바퀴중심) {L0*1000:.1f} mm "
              f"(h = {(L0 + leg_map.R_WHEEL)*1000:.1f} mm), 폐루프 잔차 최대 {rows[:, 4].max():.1e}")
        print(f"   도달 가능 {rows[:, 3].min()*1000:.1f} ~ {rows[:, 3].max()*1000:.1f} mm, "
              f"사용구간 {lo*1000:.1f} ~ {hi*1000:.1f} mm 에 해당하는 점 {len(sel)} 개")
        if len(sel) < 10 or sel[:, 3].min() > lo + 1e-3 or sel[:, 3].max() < hi - 1e-3:
            print("   [오류] 사용 구간을 다 덮지 못한다"); return 1
        for k, s in ((1, "M"), (2, "I"), (3, "K")):
            col = sel[:, k - 1] if s == "M" else sel[:, k - 1]
        for idx, s in ((0, "M"), (1, "I"), (2, "K")):
            v = sel[:, idx]
            out[f"{side}_joint_{s}"] = [round(float(v.min() - MARGIN), 4), round(float(v.max() + MARGIN), 4)]
            print(f"   {side}_joint_{s}: {np.degrees(v.min()):7.2f} ~ {np.degrees(v.max()):7.2f} deg")
        # 단조성: 다리 길이가 M 에 대해 단조여야 한 구간이 한 가지 해다
        d = np.diff(sel[:, 3])
        print(f"   다리길이 단조 {'예' if (np.all(d > 0) or np.all(d < 0)) else '아니오(주의)'}")
    Path(a.out).write_text(json.dumps({"margin": MARGIN, "source": Path(a.urdf).name,
                                       "limits": out}, indent=2))
    print(f"\n저장: {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
