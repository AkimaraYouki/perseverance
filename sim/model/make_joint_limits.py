"""4절링크 기구학에서 관절 각도 한계를 계산해 joint_limits.json 으로 낸다.

fix_urdf.py 가 이 파일을 읽어 URDF 에 넣는다. 링크 치수(leg_map.py)나
사용 구간(THETA_MIN/MAX)이 바뀌면 이걸 다시 돌리면 된다.

부호 규약:
  URDF 의 I, K 관절 축이 여기 각도 규약과 반대 방향이라 부호를 뒤집는다.
  좌우는 M, I 가 서로 반대이고 K 는 같다.
  이 표는 MuJoCo 닫힌고리 모델 실측과 대조해 확인했다 (M 오차 0.003 rad).
"""

import json
from pathlib import Path

import numpy as np

import leg_map as L

MARGIN = 0.05   # 한계에 끼지 않도록 두는 여유 [rad]


def _frames(theta):
    theta = np.asarray(theta, dtype=float)
    xP, yP = L.L2, 0.0
    xI, yI = L.L1 * np.cos(theta), L.L1 * np.sin(theta)
    dx, dy = xI - xP, yI - yP
    d = np.maximum(np.hypot(dx, dy), 1e-9)
    de = np.clip(d, abs(L.L5 - L.L3) + 1e-9, (L.L5 + L.L3) - 1e-9)
    a = (L.L5**2 - L.L3**2 + de**2) / (2 * de)
    hc = np.sqrt(np.maximum(L.L5**2 - a**2, 0.0))
    ux, uy = dx / d, dy / d
    mx, my = xP + a * ux, yP + a * uy
    xK = mx + L.ASSEMBLY_SIGN * (-uy) * hc
    yK = my + L.ASSEMBLY_SIGN * (ux) * hc
    vx, vy = xI - xK, yI - yK
    v = np.maximum(np.hypot(vx, vy), 1e-9)
    z = np.zeros_like(theta)
    return (z, z), (xP + z, yP + z), (xI, yI), (xK, yK), (xI + L.L4 * vx / v, yI + L.L4 * vy / v)


def _joint_angles(theta):
    """크랭크 절대각과, 크랭크->생크 / 생크->로커 상대각."""

    M, P, I, K, W = _frames(theta)
    ang = lambda p, q: np.arctan2(q[1] - p[1], q[0] - p[0])
    wrap = lambda x: (x + np.pi) % (2 * np.pi) - np.pi

    crank = ang(M, I)
    shank = ang(K, W)      # K-I-W 는 한 강체라 K->W 가 생크 방향
    rocker = ang(P, K)

    return crank, wrap(shank - crank), wrap(rocker - shank)


# URDF 관절 = 부호 x (Python 각도 - 영점 각도)
SIGN = {
    "R_joint_M": +1, "R_joint_I": -1, "R_joint_K": -1,
    "L_joint_M": -1, "L_joint_I": +1, "L_joint_K": -1,
}

# CAD 조립 영점에서의 theta_kR. URDF 관절값 0 이 이 자세다.
THETA_ZERO = np.deg2rad(45.002)


def main():

    c0, i0, k0 = _joint_angles(THETA_ZERO)
    th = np.linspace(L.THETA_MIN, L.THETA_MAX, 2001)
    c, i, k = _joint_angles(th)

    base = {"M": c - c0, "I": i - i0, "K": k - k0}

    out = {}
    for name, sgn in SIGN.items():
        v = sgn * base[name.split("_")[-1]]
        out[name] = [round(float(v.min()) - MARGIN, 4), round(float(v.max()) + MARGIN, 4)]

    path = Path(__file__).parent.parent / "joint_limits.json"
    path.write_text(json.dumps({"margin": MARGIN, "limits": out}, indent=2))

    print(f"{path} 생성  (여유 {MARGIN} rad 포함)")
    print(f"{'joint':>12} {'lower':>9} {'upper':>9} {'범위[deg]':>12}")
    for n, (lo, hi) in out.items():
        print(f"{n:>12} {lo:9.4f} {hi:9.4f} {np.degrees(hi-lo):11.1f}")


main()
