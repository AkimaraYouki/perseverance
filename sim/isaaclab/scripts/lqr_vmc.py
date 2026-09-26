"""고전 제어기: 바퀴 LQR (앞뒤 균형·속도) + 다리 VMC (높이 스프링-댐퍼 + 자중 보상 + roll 수평).

Ascento (arXiv 2005.11435 III, IV-A) 의 구성을 따른다:
  * 바퀴: 두 바퀴 + 역진자 몸체 모델을 선형화한 LQR, 다리 높이(진자 길이 l)마다 게인을 구해 보간한다.
  * 다리: 가상 스프링-댐퍼 (논문은 착지·복구에만. 여기선 주행 중에도 = VMC). 좌우 roll 은 다리 길이 차로 잡는다
    (논문 VI 의 lean 모드처럼 다리 각을 따로 움직여 β 를 만든다 — 여기선 β = 0 으로).
numpy 만 쓴다 (Isaac python 에 scipy 가 없다). 실기에도 그대로 옮길 수 있게 입력은 IMU·관절각·바퀴 속도로 만든 상태.

모델 (평면, 두 바퀴 합침). x = 바퀴 진행 거리, θ = 바퀴축->몸체 무게중심 벡터가 수직과 이루는 각 (+ = 앞으로 기욺),
τ = 두 바퀴 토크 합 (+ = 앞으로 굴림, 몸체엔 -τ):
    (m_w + I_w/R^2 + m) x'' + m l θ''          = τ / R
    m l x''             + (I + m l^2) θ'' - m g l θ = -τ
"""
import numpy as np

G = 9.81


def lqr_gain(A, B, Q, R):
    """연속시간 LQR K (u = -K x). 해밀토니안 고유분해로 Riccati 를 푼다."""
    n = A.shape[0]
    Ri = np.linalg.inv(R)
    H = np.block([[A, -B @ Ri @ B.T], [-Q, -A.T]])
    w, V = np.linalg.eig(H)
    Vs = V[:, w.real < 0]
    P = np.real(Vs[n:] @ np.linalg.inv(Vs[:n]))
    return Ri @ B.T @ P


def wip_model(m, l, I, m_w, I_w, R):
    """상태 [x, x', θ, θ'], 입력 τ (두 바퀴 합)."""
    M = np.array([[m_w + I_w / R**2 + m, m * l], [m * l, I + m * l**2]])
    Mi = np.linalg.inv(M)
    g_th = Mi @ np.array([0.0, m * G * l])
    b_u = Mi @ np.array([1.0 / R, -1.0])
    A = np.zeros((4, 4)); A[0, 1] = 1.0; A[2, 3] = 1.0
    A[1, 2], A[3, 2] = g_th
    B = np.zeros((4, 1)); B[1, 0], B[3, 0] = b_u
    return A, B


class WheelLQR:
    """진자 길이 l 격자마다 K 를 미리 구해 두고 선형 보간 (Ascento: 다리 높이 10 개)."""

    def __init__(self, m, I, m_w, I_w, R, q=(2.0, 5.0, 100.0, 5.0), r=1.0, l_grid=np.linspace(0.12, 0.40, 15)):
        self.l_grid = l_grid
        self.K = np.array([lqr_gain(*wip_model(m, l, I, m_w, I_w, R), np.diag(q), np.array([[r]]))[0] for l in l_grid])

    def gain(self, l):
        return np.array([np.interp(l, self.l_grid, self.K[:, j]) for j in range(4)])

    def torque(self, l, x_err, v_err, th, thd):
        """τ = -K [x - x_ref, v - v_ref, θ, θ']."""
        return float(-self.gain(l) @ np.array([x_err, v_err, th, thd]))


class RollPI:
    """좌우 다리 길이 차 Δ = hL - hR. roll (로그 규약: asin(g_y) > 0 = 왼쪽이 낮다) 이 0 이 되게 PI."""

    def __init__(self, track=0.198):
        self.track, self.i = track, 0.0

    def reset(self, diff=0.0):
        self.i = diff

    def __call__(self, roll, dt, kp, ki, lim, freeze=False, leak=0.0, rate=0.0, kd=0.0):
        """freeze: 적분 멈춤 (한 바퀴가 떠서 다리를 움직여도 roll 이 안 바뀔 때 — 안 멈추면 적분이 한계까지 차서
        착지 순간 반대로 넘기고, 반대 바퀴가 뜨며 번갈아 '뜀박질' 한다, 2026-09-26 패드 기록).
        leak [1/s]: 적분을 0 쪽으로 천천히 새게 해 한계에 붙어 있지 않게."""
        e = self.track * np.sin(roll)                  # roll 을 없애는 데 필요한 길이 차 (작은 각)
        if not freeze:
            self.i += ki * e * dt
        self.i = float(np.clip(self.i * (1.0 - leak * dt), -lim, lim))
        return float(np.clip(self.i + kp * e + kd * self.track * rate, -lim, lim))   # kd: roll 각속도 감쇠
