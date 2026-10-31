"""다음 로봇 ("<" 2 링크 다리: 고관절 피치 - 무릎 피치 - 바퀴) 관절 토크와 안전 하중 (2026-10-31).

기준은 asd.py (Gen2 4 절 링크 설계, ~/PycharmProjects/PythonProject/asd.py) 와 같다:
  평지 홀딩 (속도 0 으로 버팀, 발열 최악) <= 정격 x 0.88 (asd 권장 4.5 kg 때 88 %, 100 % 넘으면 셧다운 위험)
  등반 순간 (계단 한 칸 1~2 s, 열용량으로 흡수) <= 피크 x 0.5 (asd TAU_KR_MAX 4.5 = AK60-6 피크의 50 %)
Gen2 실기 (4.17 kg) 는 이 기준 안에서 발열 없이 돌았다.

질량은 몸통과 다리를 나눠 본다 (토크는 관절보다 아래 / 위 질량이 다르게 걸린다):
  m_body  : 고관절 피치 축 위 전부 (프레임, 배터리, Jetson, 센서, 요/롤 모터, 고관절 피치 모터 — 축 위라 고관절 피치 레버 0)
  다리 한 짝: 허벅지 링크, 무릎 모터 (무릎 축), 정강이 링크, 바퀴 뭉치 (바퀴 모터 + 바퀴, 바퀴 중심)
정역학, "<" 다리 (허벅지 = 정강이 = L), 바퀴 중심이 고관절 바로 아래 (균형 중 무게중심이 바퀴 위):
  무릎 = 바퀴에서 수평 d = sqrt(L^2 - (h/2)^2) 앞.
  고관절 토크: 지면 반력은 레버 0 -> 다리 자기 무게만 (허벅지·정강이 d/2, 무릎 모터 d)
  무릎 토크  : 지면 반력 x d - 정강이·바퀴 뭉치 무게 (같은 쪽) = g d (지지하는 몸통 몫 + 허벅지 + 무릎 모터 + 정강이/2)
등반: 위 단에 올린 다리 하나가 h - rise 로 굽힌 채 몸통 + 반대 다리를 전부 받음 (asd LOAD_FACTOR 2 에 해당).
사용: python3 leg2_load.py
"""
import numpy as np

G = 9.81
MOTOR = {"AK80-6": (6.0, 12.0, 0.485), "AK60-6": (3.0, 9.0, 0.380), "AK45-10": (2.5, 7.0, 0.262), "AK40-10": (1.3, 4.1, 0.190)}
K_HOLD, K_CLIMB = 0.88, 0.5

# 다리 한 짝 (추정, CAD 나오면 교체) [kg]
LEG = dict(thigh=0.15, knee_motor=MOTOR["AK80-6"][2], shank=0.12, wheel=MOTOR["AK40-10"][2] + 0.12)


def lever(L, h):
    return np.sqrt(max(L * L - (h / 2) ** 2, 0.0))


def hip_knee(L, h, m_carry, leg=LEG):
    """m_carry = 이 다리가 받치는 몸통 (+ 반대 다리) 질량. 반환 (고관절, 무릎) N·m."""
    d = lever(L, h)
    hip = G * d * (leg["thigh"] / 2 + leg["knee_motor"] + leg["shank"] / 2)
    knee = G * d * (m_carry + leg["thigh"] + leg["knee_motor"] + leg["shank"] / 2)
    return hip, knee


def safe_body(L, h, rise, knee="AK80-6", hip="AK80-6", leg=LEG):
    """평지 홀딩·등반 기준을 만족하는 최대 몸통 질량 [kg] (무릎·고관절 각각)."""
    m_leg = sum(leg.values())
    d_h, d_c = lever(L, h), lever(L, h - rise)
    other = G * (leg["thigh"] + leg["knee_motor"] + leg["shank"] / 2)          # 다리 자기 몫 (몸통 0 일 때)
    out = {}
    r, p, _ = MOTOR[knee]
    hold = (r * K_HOLD / (G * d_h) - other / G) * 2                            # 무릎 = g d (m_body/2 + ...)
    climb = p * K_CLIMB / (G * d_c) - other / G - m_leg                        # 무릎 = g d (m_body + m_leg + ...)
    out["knee_hold"], out["knee_climb"] = hold, climb
    rh, ph, _ = MOTOR[hip]
    hp = hip_knee(L, h - rise, 0.0, leg)[0]
    out["hip_ok"] = hp <= ph * K_CLIMB and hip_knee(L, h, 0.0, leg)[0] <= rh * K_HOLD
    return out


if __name__ == "__main__":
    m_leg = sum(LEG.values())
    print(f"다리 한 짝 {m_leg:.2f} kg (허벅지 {LEG['thigh']}, 무릎 AK80-6 {LEG['knee_motor']}, 정강이 {LEG['shank']}, 바퀴 뭉치 {LEG['wheel']:.2f})")
    print(f"기준: 평지 홀딩 <= 정격 6 x {K_HOLD} = {6 * K_HOLD:.2f} N·m, 등반 <= 피크 12 x {K_CLIMB} = {12 * K_CLIMB:.1f} N·m (asd.py 와 같음)")
    for rise in (0.10, 0.17):
        print(f"\n== 계단 {rise * 100:.0f} cm (서기 h = 0.85 x 2L, 등반 때 올린 다리 h - {rise * 100:.0f} cm)")
        print(f"{'L':>5} {'h':>6} | 몸통 상한 [kg]: 평지 홀딩 / 등반 | 총중량 상한 | 고관절")
        for L in (0.18, 0.20, 0.22, 0.25, 0.28):
            h = 2 * L * 0.85
            if h - rise < 0.6 * L:
                print(f"{L:5.2f} {h:6.3f} | 다리가 짧음"); continue
            s = safe_body(L, h, rise)
            mb = min(s["knee_hold"], s["knee_climb"])
            print(f"{L:5.2f} {h:6.3f} | {s['knee_hold']:5.1f} / {s['knee_climb']:5.1f} | {mb + 2 * m_leg:5.1f} kg (몸통 {mb:.1f}) | {'OK' if s['hip_ok'] else '초과'}")
    mb = 4.2
    print(f"\n예: 몸통 {mb} kg (프레임 1.2, 배터리 0.7, Jetson·RealSense 0.4, 센서 0.15, 요/롤 AK60-6 x2 0.76, 고관절 AK80-6 x2 0.97) + 다리 2 짝 = {mb + 2 * m_leg:.1f} kg")
    for L in (0.20, 0.22):
        h = 2 * L * 0.85
        hold = hip_knee(L, h, mb / 2)
        for rise in (0.10, 0.17):
            climb = hip_knee(L, h - rise, mb + m_leg)
            print(f"  L {L:.2f}: 평지 홀딩 무릎 {hold[1]:.2f} N·m ({hold[1] / 6 * 100:.0f} % 정격), 고관절 {hold[0]:.2f} | "
                  f"{rise * 100:.0f} cm 등반 무릎 {climb[1]:.2f} N·m ({climb[1] / 12 * 100:.0f} % 피크), 고관절 {climb[0]:.2f}")
