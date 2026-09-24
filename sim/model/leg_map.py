"""다리 모터 각도 theta_kR <-> 다리 높이 h 변환.

RL 정책은 다리를 "길이 h 를 갖는 직선 관절" 하나로 다루고,
실기와 시뮬(4절링크)은 모터 각도 theta 로 움직인다. 그 사이를 잇는다.

    정책 출력  h        -->  theta = theta_of_h(h)      -->  모터 위치 지령
    정책 출력  F (수직력) -->  tau   = F * dh_dtheta(th) -->  모터 토크 지령
    모터 상태  theta     -->  h     = h_of_theta(theta)  -->  정책 관측

치수는 CAD(export 4) 실측값이고, MuJoCo 닫힌고리 모델과 0.3 mm 이내로 일치한다.
"""

import numpy as np

# ============================================================
# CAD 실측 치수 [m]  (URDF 에서 평면 내 성분으로 뽑은 값)
# ============================================================

L1 = 0.10272   # |M-I|  크랭크 (모터가 직접 구동)
L2 = 0.10072   # |M-P|  그라운드
L3 = 0.08000   # |I-K|  커플러
L4 = 0.17098   # |I-W|  생크
L5 = 0.08000   # |P-K|  로커

R_WHEEL = 0.060   # 바퀴 반지름 (지름 120 mm)

ASSEMBLY_SIGN = -1.0   # 조립 분기

# 실제 사용 구간 [rad]
THETA_MIN = np.deg2rad(38.021)
THETA_MAX = np.deg2rad(97.789)


def _wheel_xy(theta):
    """모터축 M 을 원점으로 한 바퀴 중심 좌표. asd.py 와 같은 좌표 우선 방식."""

    theta = np.asarray(theta, dtype=float)

    # P 는 고정. 방향은 h 계산에 영향이 없으므로 0 으로 둔다.
    xP, yP = L2, 0.0

    xI = L1 * np.cos(theta)
    yI = L1 * np.sin(theta)

    dx, dy = xI - xP, yI - yP
    d = np.maximum(np.hypot(dx, dy), 1e-9)

    # |P-K| = L5, |I-K| = L3 의 원-원 교점
    d_eff = np.clip(d, abs(L5 - L3) + 1e-9, (L5 + L3) - 1e-9)
    a = (L5**2 - L3**2 + d_eff**2) / (2.0 * d_eff)
    hc = np.sqrt(np.maximum(L5**2 - a**2, 0.0))

    ux, uy = dx / d, dy / d
    mx, my = xP + a * ux, yP + a * uy

    xK = mx + ASSEMBLY_SIGN * (-uy) * hc
    yK = my + ASSEMBLY_SIGN * (ux) * hc

    # W 는 K->I 를 지나 I 에서 L4 만큼 더 간 점
    vx, vy = xI - xK, yI - yK
    v = np.maximum(np.hypot(vx, vy), 1e-9)

    return xI + L4 * vx / v, yI + L4 * vy / v


def h_of_theta(theta):
    """모터 각도 -> 다리 높이 (모터축에서 지면까지) [m]"""

    xW, yW = _wheel_xy(theta)

    return np.hypot(xW, yW) + R_WHEEL


def dh_dtheta(theta, eps=1e-6):
    """dh/dtheta [m/rad].  토크 <-> 수직력 변환 계수."""

    return (h_of_theta(np.asarray(theta) + eps) - h_of_theta(np.asarray(theta) - eps)) / (2 * eps)


_TH = np.linspace(THETA_MIN, THETA_MAX, 2001)
_H = h_of_theta(_TH)

H_MIN, H_MAX = float(_H.min()), float(_H.max())


def theta_of_h(h):
    """다리 높이 -> 모터 각도 [rad].  단조 구간이라 보간으로 충분하다."""

    return np.interp(np.clip(h, H_MIN, H_MAX), _H, _TH)


def tau_of_force(force, theta):
    """다리가 내야 할 수직력 [N] -> 모터 토크 [Nm]"""

    return np.asarray(force) * dh_dtheta(theta)


def force_of_tau(tau, theta):
    """모터 토크 [Nm] -> 다리 수직력 [N]"""

    return np.asarray(tau) / dh_dtheta(theta)


if __name__ == "__main__":
    print(f"h 범위 : {H_MIN*1000:.1f} ~ {H_MAX*1000:.1f} mm  (stroke {(H_MAX-H_MIN)*1000:.1f})")
    print(f"theta  : {np.rad2deg(THETA_MIN):.3f} ~ {np.rad2deg(THETA_MAX):.3f} deg")
    print()
    print(f"{'theta[deg]':>11} {'h[mm]':>9} {'dh/dth[mm/rad]':>15} {'1N->tau[mNm]':>13}")
    for t in np.linspace(THETA_MIN, THETA_MAX, 7):
        print(f"{np.rad2deg(t):11.2f} {h_of_theta(t)*1000:9.1f} "
              f"{dh_dtheta(t)*1000:15.2f} {tau_of_force(1.0,t)*1000:13.2f}")
    print()
    err = np.abs(theta_of_h(h_of_theta(_TH)) - _TH).max()
    print(f"왕복 변환 오차 (theta->h->theta) = {np.rad2deg(err)*3600:.3f} arcsec")
