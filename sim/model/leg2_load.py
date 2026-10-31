"""다음 로봇 ("<" 2 링크 다리: 고관절 피치 - 무릎 피치 - 바퀴) 관절 토크와 안전 하중 (2026-10-31).

기준: 이전 로봇 (4 절 링크, AK60-6) 은 서 있을 때 고관절 1.94 N·m = 정격 3 N·m 의 65 % 로 발열 없이 썼다 (leg_map, 4.17 kg).
-> 연속 토크 한계 = 정격 x 0.65 (검증된 사용률), 순간 한계 = 피크 x 0.8.

경우 (정역학, 지면 반력 F = (Fx, Fz) 가 바퀴 중심에 걸린다고 봄, 다리·모터 자중의 모멘트는 무시 = 보수적):
  stand  : 두 다리로 서기, 바퀴가 고관절 바로 아래, 한 다리 Fz = m g / 2
  accel  : stand + 주행 가속 2 m/s² (Fx = m a / 2) -> 고관절에 Fx x 높이
  step   : 계단 한 발 서기, 바퀴가 고관절보다 앞으로 x_off, Fz = m g (한 다리가 전부)
  step_dyn: step x 1.5 (디딜 때·내려갈 때 감속 충격)
사용: python3 leg2_load.py
"""
import numpy as np

G = 9.81
MOTOR = {"AK80-6": (6.0, 12.0), "AK60-6": (3.0, 9.0), "AK45-10": (2.5, 7.0)}   # (정격, 피크) N·m
K_CONT, K_PEAK = 0.65, 0.8


def ik(l1, l2, x, z, knee_fwd=True):
    """고관절 원점, 바퀴 중심 (x 앞, z 아래 +) -> 허벅지 각 q1, 무릎 각 q2 (평면 2 링크)."""
    d = np.hypot(x, z)
    c2 = np.clip((d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1, 1)
    q2 = np.arccos(c2) * (1 if knee_fwd else -1)
    q1 = np.arctan2(x, z) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
    return q1, q2


def torques(l1, l2, x, z, fx, fz):
    """정역학 tau = J^T F. 반환 (|tau_hip|, |tau_knee|)."""
    q1, q2 = ik(l1, l2, x, z)
    # 바퀴 위치 (x 앞, z 아래): x = l1 sin q1 + l2 sin(q1+q2), z = l1 cos q1 + l2 cos(q1+q2)
    J = np.array([[l1 * np.cos(q1) + l2 * np.cos(q1 + q2), l2 * np.cos(q1 + q2)],
                  [-l1 * np.sin(q1) - l2 * np.sin(q1 + q2), -l2 * np.sin(q1 + q2)]])
    t = J.T @ np.array([fx, -fz])                     # 지면이 바퀴를 위로 민다 (z 아래 + 좌표라 -fz)
    return abs(t[0]), abs(t[1])


def stair(l1, l2, h, rise, m, x_off=0.03, k_dyn=1.3):
    """계단 한 사이클: 서기 (두 다리), 밀어올리기 (위 단에 올린 다리 하나가 h - rise 로 굽힌 채 몸 전체를 들어 올림)."""
    stand = torques(l1, l2, 0.0, h, 0.0, m * G / 2)
    push = torques(l1, l2, x_off, h - rise, 0.0, m * G)
    rms = tuple(np.sqrt(0.5 * push[i] ** 2 + 0.5 * stand[i] ** 2) for i in range(2))   # 반은 한 다리, 반은 두 다리
    return stand, push, rms, (push[0] * k_dyn, push[1] * k_dyn)


def safe_mass(l1, l2, h, rise, hip="AK80-6", knee="AK80-6"):
    """서기 연속 <= 정격 x 0.65 (이전 로봇 검증), 계단 오르는 동안 RMS <= 정격, 밀어올리기 순간 x 1.3 <= 피크 x 0.8."""
    stand, push, rms, dyn = stair(l1, l2, h, rise, 1.0)
    acc = torques(l1, l2, 0.0, h, 1.0, G / 2)          # 가속 2 m/s^2: Fx = m a / 2 = 1 N/kg
    out = {}
    for name, i, mot in (("hip", 0, hip), ("knee", 1, knee)):
        r, p = MOTOR[mot]
        out[name] = min(r * K_CONT / max(stand[i], acc[i], 1e-9), r / max(rms[i], 1e-9), p * K_PEAK / max(dyn[i], 1e-9))
    return min(out.values()), out


if __name__ == "__main__":
    for rise in (0.08, 0.17):
        print(f"\n== 계단 높이 {rise * 100:.0f} cm  (허벅지 = 정강이 = L, 서기 높이 h = 0.85 x 2L)")
        print(f"{'L':>5} {'h':>6} | 1 kg 당 무릎 N·m: 서기 / 밀어올리기 / RMS | 안전 질량 [kg]: 무릎 AK80-6 (고관절 AK80-6)")
        for L in (0.18, 0.20, 0.22, 0.25, 0.28):
            h = 2 * L * 0.85
            if h - rise < 0.35 * 2 * L:
                print(f"{L:5.2f} {h:6.3f} | 다리가 짧아 {rise * 100:.0f} cm 단에 못 올림"); continue
            stand, push, rms, dyn = stair(L, L, h, rise, 1.0)
            m, lim = safe_mass(L, L, h, rise)
            print(f"{L:5.2f} {h:6.3f} | {stand[1]:5.2f} / {push[1]:5.2f} / {rms[1]:5.2f} | {lim['knee']:5.1f}  ({lim['hip']:.1f})")
