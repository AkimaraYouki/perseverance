#!/usr/bin/env python3
"""VMC + LQR 게인 설계 — Liu & Wang (2024) 구조를 이 로봇 파라미터로 푼다.

논문: "Locomotion Control of a Bipedal Wheeled Robot Using Virtual Model Control and
Linear Quadratic Regulator Techniques", Emerging Science Innovation 4 (2024) 17-32.

구조 (논문 Fig.5):
  직진    LQR  u = -K(h) [theta, theta_dot, x - x_d, x_dot - v_d]  -> T_theta (두 바퀴 합)
  조향    PID  (delta_d - delta)                                   -> T_delta
  분배    T_L = 0.5 T_theta + 0.5 T_delta,  T_R = 0.5 T_theta - 0.5 T_delta   (논문 식 34)
  높이    F = F_d + PID(h_d - h),  F_d = M g / cos(theta)          (식 50)
          tau_hip = J^T F,  J = dh/dtheta_motor                    (식 48)
          -> 이 로봇은 J 가 leg_map.dh_dtheta 그대로다 (113~119 mm/rad).

K 는 다리 높이 h 마다 선형화해 풀고 (논문처럼 10 mm 간격), 원소별로
K_ij(h) = p0 + p1 h + p2 h^2 로 맞춘다 (식 40). 실기에서는 이 계수만 있으면 된다.

부호 규약 (Isaac Sim 에서 PD 로 확인한 것과 같다):
  전진 +x, 바퀴축 +y.  theta = +y 축 회전 = 윗부분이 +x 로 기움 (앞으로 기울면 +)
  theta ~= projected_gravity_b.x,  theta_dot = ang_vel_b.y
  양(+)의 바퀴 토크 -> 로봇 +x 전진, 몸체는 뒤로(-theta) 반작용.
  실기 IMU 에서는 이 정의에 맞게 축/부호를 먼저 맞출 것.

URDF 가 아직 확정 전이다. 파라미터는 전부 URDF 에서 읽으므로 URDF 가 바뀌면
이 스크립트만 다시 돌리면 된다.

    python3 vmc_lqr_design.py                         # 표 출력 + gains.yaml 생성
    python3 vmc_lqr_design.py --extra-mass 0.25       # 실측 총질량 = 모델 + 0.25 kg (기본값)
    python3 vmc_lqr_design.py --urdf ../model/robot_simple.urdf --out gains.yaml
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.linalg import solve_continuous_are, expm

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "model"))
import leg_map  # noqa: E402

G = 9.81

# 논문 식 51 의 가중치에서 직진 부분(theta, theta_dot, x, x_dot)과 T_theta 항.
Q_PAPER = np.diag([6000.0, 1.0, 1000.0, 100.0])
R_PAPER = np.array([[100.0]])


# --------------------------------------------------------------------- 모델
def load_urdf(path):
    r = ET.parse(path).getroot()
    links = {}
    for l in r.findall("link"):
        i = l.find("inertial")
        if i is None:
            continue
        e = i.find("inertia")
        g = lambda k: float(e.get(k))
        links[l.get("name")] = dict(
            m=float(i.find("mass").get("value")),
            com=np.array([float(v) for v in i.find("origin").get("xyz").split()]),
            I=np.array([[g("ixx"), g("ixy"), g("ixz")],
                        [g("ixy"), g("iyy"), g("iyz")],
                        [g("ixz"), g("iyz"), g("izz")]]),
        )
    joints = {j.get("name"): j for j in r.findall("joint")}
    return links, joints


def plant(links, joints, h, extra_mass, wheel_rotor):
    """다리 높이 h [m] 에서 논문 식 22~24 의 파라미터를 계산한다.

    h = 지면 ~ 고관절(모터축) 높이 (leg_map 정의). 고관절~바퀴중심 = h - R.
    추가 질량(배선·배터리 등 실측 차이)은 몸체 COM 에 얹는다.
    """
    R = leg_map.R_WHEEL
    base, car, wh = links["base"], links["r_carrier"], links["r_wheel"]

    hip_to_axle = h - R
    m_b = base["m"] + extra_mass
    z_b = hip_to_axle + base["com"][2]          # 몸체 COM 의 바퀴축 위 높이
    z_c = 0.0                                   # 캐리어 COM 은 바퀴축에 있다 (단순화 모델)
    m_c = 2.0 * car["m"]

    M = m_b + m_c                               # 바퀴 위 전체 (논문 M)
    L = (m_b * z_b + m_c * z_c) / M             # COM ~ 바퀴축 (논문 L)

    # 피치(y) 관성, COM 기준 (논문 I_theta)
    I_b = base["I"][1, 1] * (m_b / base["m"])
    I_theta = (I_b + m_b * (z_b - L) ** 2
               + 2.0 * car["I"][1, 1] + m_c * (z_c - L) ** 2)

    m_w = wh["m"]                               # 바퀴 하나 (논문 m)
    I_w = wh["I"][1, 1] + wheel_rotor           # 바퀴 스핀 + 감속기 반사 관성 (논문 I_w)

    yl = [float(j.find("origin").get("xyz").split()[1])
          for n, j in joints.items() if n.endswith("_leg")]
    D = abs(yl[0] - yl[1])                      # 바퀴 간격 (논문 D)
    return dict(M=M, L=L, I_theta=I_theta, m=m_w, I_w=I_w, R=R, D=D,
                total=M + 2 * m_w)


def linearize(p):
    """식 22, 23 을 theta=0 에서 선형화. 상태 [theta, theta_dot, x, x_dot], 입력 T = T_L + T_R."""
    M, L, I, m, Iw, R = p["M"], p["L"], p["I_theta"], p["m"], p["I_w"], p["R"]
    a = R * (M + 2 * m + 2 * Iw / R**2)   # 식 22 좌변 계수
    b = M * R * L                         # 식 22 의 theta_ddot 항
    c = I + M * L**2                      # 식 23 좌변 계수
    d = M * G * L                         # 식 23 의 중력 항
    e = M * L                             # 식 23 의 x_ddot 항
    #   a xdd + b thdd = T
    #   e xdd + c thdd = d th - T
    det = a * c - b * e
    A = np.zeros((4, 4))
    B = np.zeros((4, 1))
    A[0, 1] = 1.0
    A[2, 3] = 1.0
    A[1, 0] = a * d / det            # thdd = (a d th - (a + e) T) / det
    B[1, 0] = -(a + e) / det
    A[3, 0] = -b * d / det           # xdd  = ((c + b) T - b d th) / det
    B[3, 0] = (c + b) / det
    return A, B


def lqr(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


# --------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--urdf", default=str(HERE.parent / "model" / "robot_simple.urdf"))
    ap.add_argument("--extra-mass", type=float, default=0.25,
                    help="실측 총질량 - 모델 총질량 [kg] (2026-09-24 사용자 추정 +0.25)")
    ap.add_argument("--wheel-rotor", type=float, default=157.33e-7 * 10.0**2,
                    help="바퀴 모터 반사 관성 [kg m^2] (AK45-10 157.33 g cm^2 x 10^2)")
    ap.add_argument("--step-mm", type=float, default=10.0)
    ap.add_argument("--ctrl-hz", type=float, default=200.0, help="이산화 검증용 제어 주기")
    ap.add_argument("--friction-nm", type=float, default=0.82,
                    help="바퀴 하나의 마찰 한계 토크 [Nm]")
    ap.add_argument("--out", default=str(HERE / "gains.yaml"))
    args = ap.parse_args()

    links, joints = load_urdf(args.urdf)
    n = int(round((leg_map.H_MAX - leg_map.H_MIN) / (args.step_mm / 1000.0))) + 1
    hs = np.linspace(leg_map.H_MIN, leg_map.H_MAX, n)

    rows = []
    for h in hs:
        p = plant(links, joints, h, args.extra_mass, args.wheel_rotor)
        A, B = linearize(p)
        K = lqr(A, B, Q_PAPER, R_PAPER)
        ol = np.linalg.eigvals(A)
        cl = np.linalg.eigvals(A - B @ K)
        Ad = expm((A - B @ K) / args.ctrl_hz)           # 이산 폐루프 (ZOH 근사)
        rho = max(abs(np.linalg.eigvals(Ad)))
        # 5 deg 기울기 한 번에 요구하는 합토크 -> 바퀴 하나당
        t5 = abs(K[0, 0] * math.radians(5.0)) / 2.0
        # VMC 정적 피드포워드 (식 50, theta=0): 한 다리가 받는 힘과 고관절 토크
        th = leg_map.theta_of_h(h)
        F_leg = p["M"] * G / 2.0
        tau_hip = F_leg * leg_map.dh_dtheta(th)
        rows.append(dict(h=h, p=p, K=K[0], ol=ol, cl=cl, rho=rho, t5=t5,
                         theta=th, F_leg=F_leg, tau_hip=tau_hip))

    p0 = rows[0]["p"]
    print(f"URDF      {args.urdf}")
    print(f"총질량    {p0['total']:.3f} kg  (모델 + {args.extra_mass:.3f})   "
          f"바퀴 위 M {p0['M']:.3f} kg   바퀴간격 D {p0['D']*1000:.0f} mm   R {p0['R']*1000:.0f} mm")
    print(f"바퀴 I_w  {p0['I_w']:.4e} kg m^2 (스핀 + 반사 관성)")
    print(f"Q = diag{tuple(np.diag(Q_PAPER))}, R = {R_PAPER[0,0]}  (논문 식 51 직진 부분)\n")

    hdr = (f"{'h[mm]':>6} {'L[mm]':>6} {'Ith':>7} {'불안정극':>7} | "
           f"{'K_th':>7} {'K_thd':>7} {'K_x':>7} {'K_xd':>7} | {'폐루프 최저':>9} "
           f"{'|z|@Hz':>7} | {'5deg당 Nm/바퀴':>13} | {'모터각':>6} {'F/다리':>6} {'고관절Nm':>8}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        k = r["K"]
        print(f"{r['h']*1000:6.1f} {r['p']['L']*1000:6.1f} {r['p']['I_theta']:7.4f} "
              f"{max(r['ol'].real):7.2f} | {k[0]:7.2f} {k[1]:7.3f} {k[2]:7.3f} {k[3]:7.3f} | "
              f"{max(r['cl'].real):9.3f} {r['rho']:7.4f} | {r['t5']:13.3f} | "
              f"{math.degrees(r['theta']):6.1f} {r['F_leg']:6.1f} {r['tau_hip']:8.2f}")

    # 게인 스케줄 다항식 적합 (식 40)
    H = np.array([r["h"] for r in rows])
    Ks = np.array([r["K"] for r in rows])
    names = ["theta", "theta_dot", "x", "x_dot"]
    coeffs, fit_err = {}, 0.0
    for j, n in enumerate(names):
        c = np.polyfit(H, Ks[:, j], 2)[::-1]        # p0, p1, p2
        coeffs[n] = c
        pred = c[0] + c[1] * H + c[2] * H**2
        fit_err = max(fit_err, float(np.max(np.abs(pred - Ks[:, j]) / np.maximum(np.abs(Ks[:, j]), 1e-9))))

    print("\n게인 스케줄 K_j(h) = p0 + p1 h + p2 h^2   (h [m], u = -K x, T_theta = 두 바퀴 합 [Nm])")
    for n in names:
        c = coeffs[n]
        print(f"  K_{n:<10} p0={c[0]: .6e}  p1={c[1]: .6e}  p2={c[2]: .6e}")
    print(f"  최대 상대 적합오차 {fit_err*100:.3f} %")

    # 판정
    print("\n판정")
    ok_stable = all(max(r["cl"].real) < 0 for r in rows)
    ok_disc = all(r["rho"] < 1 for r in rows)
    worst_t5 = max(r["t5"] for r in rows)
    worst_hip = max(r["tau_hip"] for r in rows)
    print(f"  {'PASS' if ok_stable else 'FAIL'}  연속시간 폐루프 전 구간 안정")
    print(f"  {'PASS' if ok_disc else 'FAIL'}  {args.ctrl_hz:.0f} Hz 이산 폐루프 전 구간 안정")
    print(f"  {'PASS' if worst_t5 <= args.friction_nm else 'WARN'}  5 deg 기울기 요구 토크 "
          f"{worst_t5:.3f} Nm/바퀴  (마찰한계 {args.friction_nm} Nm)")
    print(f"  {'PASS' if worst_hip <= 3.0 else 'WARN'}  VMC 정적 고관절 토크 최대 {worst_hip:.2f} Nm "
          f"(AK60-6 정격 3 Nm)")

    # YAML 출력 (의존성 없이 손으로 쓴다)
    out = Path(args.out)
    with out.open("w", encoding="utf-8") as f:
        f.write("# vmc_lqr_design.py 생성. 손으로 고치지 말고 스크립트를 다시 돌릴 것.\n")
        f.write(f"# urdf: {Path(args.urdf).name}, extra_mass_kg: {args.extra_mass}\n")
        f.write("# 상태 [theta, theta_dot, x - x_d, x_dot - v_d], u = -K x = T_theta (두 바퀴 합, Nm)\n")
        f.write("# 분배: T_L = 0.5 T_theta + 0.5 T_delta, T_R = 0.5 T_theta - 0.5 T_delta\n")
        f.write("lqr:\n")
        f.write(f"  Q_diag: {list(map(float, np.diag(Q_PAPER)))}\n  R: {float(R_PAPER[0,0])}\n")
        f.write(f"  h_range_m: [{leg_map.H_MIN:.4f}, {leg_map.H_MAX:.4f}]\n")
        f.write("  poly_p0_p1_p2:\n")
        for n in names:
            c = coeffs[n]
            f.write(f"    {n}: [{c[0]:.8e}, {c[1]:.8e}, {c[2]:.8e}]\n")
        f.write("vmc:\n")
        f.write(f"  supported_mass_kg: {p0['M']:.4f}   # 바퀴 위 전체. F_d = M g / cos(theta), 다리당 절반\n")
        f.write("  jacobian: leg_map.dh_dtheta   # tau_hip = F_leg * dh_dtheta(theta_motor)\n")
        f.write("table:\n")
        for r in rows:
            k = r["K"]
            f.write(f"  - {{h: {r['h']:.4f}, K: [{k[0]:.5f}, {k[1]:.5f}, {k[2]:.5f}, {k[3]:.5f}], "
                    f"theta_motor_deg: {math.degrees(r['theta']):.3f}, tau_hip_ff_nm: {r['tau_hip']:.3f}}}\n")
    print(f"\n저장: {out}")
    return 0 if (ok_stable and ok_disc) else 1


if __name__ == "__main__":
    sys.exit(main())
