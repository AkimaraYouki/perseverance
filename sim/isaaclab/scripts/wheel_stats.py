"""바퀴 모터 선정용 통계 (2026-10-30 사용자: 바퀴 모터 AK40-10 vs 큰 BLDC + FOC 고민 -> 필요한 토크·속도를 시뮬에서).

harsh_eval / jump_rl 에서 환경 변수 WB_WHEEL_STATS=<경로.npz> 가 있으면 켜진다 (없으면 아무것도 안 함).
매 제어 주기 (5 ms) 살아 있는 로봇의 바퀴 2 개마다: 지령 토크 |tau| [N·m] (모터가 내야 할 값, 지령 상한 뒤), 바퀴 속도 |w| [rad/s],
기계 출력 tau·w [W]. 단계별 (DRIVE / 웅크림·신전 / 공중 / 착지) 로 나눠 히스토그램, 토크-속도 2 차원 히스토그램,
로봇마다 1 s / 10 s 이동 RMS 토크의 최댓값 (발열 = 연속 토크 비교용).
"""

import numpy as np

T_MAX, T_BIN = 10.0, 0.02
W_MAX, W_BIN = 80.0, 0.25
GROUPS = ("drive", "jump", "air", "land")
PH2G = np.array([0, 1, 1, 2, 2, 3])          # wbctrl DRIVE, RETRACT, EXTRACT, FLY, DESCEND, LAND


class WheelStats:
    def __init__(self, path, n, dt):
        self.path, self.dt = path, dt
        nt, nw = int(T_MAX / T_BIN), int(W_MAX / W_BIN)
        self.ht = np.zeros((len(GROUPS), nt)); self.hw = np.zeros((len(GROUPS), nw)); self.hp = np.zeros((len(GROUPS), 400))  # noqa: E702
        self.h2 = np.zeros((len(GROUPS), int(W_MAX), int(T_MAX / 0.1)))
        self.e1 = np.zeros((n, 2)); self.e10 = np.zeros((n, 2))   # noqa: E702  이동 평균 tau^2 (1 s, 10 s)
        self.m1 = np.zeros(n); self.m10 = np.zeros(n); self.tmax = np.zeros(n); self.wmax = np.zeros(n)  # noqa: E702
        self.k1, self.k10 = dt / 1.0, dt / 10.0
        self.ve = np.zeros(n); self.vmax = np.zeros(n); self.kv = dt / 0.5   # noqa: E702  도달 속도 (0.5 s 평균의 최대, 전진)
        self.steps = 0

    def add(self, tau, w, alive, phase=None, v=None):
        tau, w = np.abs(np.asarray(tau, float)), np.abs(np.asarray(w, float))
        alive = np.asarray(alive, bool)
        if v is not None:
            self.ve += self.kv * (np.asarray(v, float) - self.ve)
            self.vmax = np.where(alive, np.maximum(self.vmax, self.ve), self.vmax)
        self.e1 += self.k1 * (tau ** 2 - self.e1); self.e10 += self.k10 * (tau ** 2 - self.e10)  # noqa: E702
        self.steps += 1
        if self.steps * self.dt > 1.0:
            self.m1 = np.where(alive, np.maximum(self.m1, np.sqrt(self.e1).max(1)), self.m1)
        if self.steps * self.dt > 10.0:
            self.m10 = np.where(alive, np.maximum(self.m10, np.sqrt(self.e10).max(1)), self.m10)
        self.tmax = np.where(alive, np.maximum(self.tmax, tau.max(1)), self.tmax)
        self.wmax = np.where(alive, np.maximum(self.wmax, w.max(1)), self.wmax)
        g = np.zeros(len(alive), int) if phase is None else PH2G[np.asarray(phase, int)]
        for gi in range(len(GROUPS)):
            m = alive & (g == gi)
            if not m.any():
                continue
            t_, w_ = tau[m].ravel(), w[m].ravel()
            self.ht[gi] += np.bincount(np.minimum((t_ / T_BIN).astype(int), self.ht.shape[1] - 1), minlength=self.ht.shape[1])
            self.hw[gi] += np.bincount(np.minimum((w_ / W_BIN).astype(int), self.hw.shape[1] - 1), minlength=self.hw.shape[1])
            self.hp[gi] += np.bincount(np.minimum((t_ * w_ / 0.5).astype(int), 399), minlength=400)
            iw, it = np.minimum(w_.astype(int), self.h2.shape[1] - 1), np.minimum((t_ / 0.1).astype(int), self.h2.shape[2] - 1)
            np.add.at(self.h2[gi], (iw, it), 1)

    @staticmethod
    def _pct(h, b, ps=(50, 95, 99, 99.9)):
        c = np.cumsum(h)
        if c[-1] == 0:
            return [float("nan")] * (len(ps) + 1)
        top = (np.flatnonzero(h)[-1] + 1) * b
        return [float((np.searchsorted(c, p / 100 * c[-1]) + 1) * b) for p in ps] + [float(top)]

    def report(self):
        lines = ["[바퀴 통계] 단계 | 샘플 (바퀴·5 ms) | |tau| N·m p50/p95/p99/p99.9/최대 | |w| rad/s p50/p95/p99/p99.9/최대 | 출력 W p99/최대"]
        for gi, gname in enumerate(GROUPS):
            n = self.ht[gi].sum()
            if n == 0:
                continue
            t_, w_, p_ = self._pct(self.ht[gi], T_BIN), self._pct(self.hw[gi], W_BIN), self._pct(self.hp[gi], 0.5)
            lines.append(f"  {gname:5s} | {int(n):>10d} | " + "/".join(f"{x:.2f}" for x in t_) + " | " + "/".join(f"{x:.1f}" for x in w_)
                         + f" | {p_[2]:.0f}/{p_[4]:.0f}")
        q = lambda a: "/".join(f"{np.percentile(a, p):.2f}" for p in (50, 95, 99, 100))   # noqa: E731
        lines.append(f"  로봇마다 최대 1 s RMS 토크 p50/p95/p99/최대 {q(self.m1)} N·m, 10 s RMS {q(self.m10)} N·m")
        lines.append(f"  로봇마다 최대 |tau| {q(self.tmax)} N·m, 최대 |w| {q(self.wmax)} rad/s")
        if self.vmax.any():
            lines.append("  도달 속도 (0.5 s 평균 최대, 전진) p10/p50/p90/최대 " + "/".join(f"{np.percentile(self.vmax, p):.2f}" for p in (10, 50, 90, 100)) + " m/s")
        env = []
        for iw in range(self.h2.shape[1]):
            col = self.h2[:, iw].sum(0)
            if col.sum() >= 200:
                env.append(f"{iw}:{self._pct(col, 0.1, (99,))[0]:.1f}")
        lines.append("  속도 [rad/s] 별 p99 토크 [N·m] (전 단계): " + " ".join(env))
        return "\n".join(lines)

    def save(self):
        np.savez(self.path, vmax=self.vmax, ht=self.ht, hw=self.hw, hp=self.hp, h2=self.h2, m1=self.m1, m10=self.m10, tmax=self.tmax, wmax=self.wmax,
                 t_bin=T_BIN, w_bin=W_BIN, groups=np.array(GROUPS))
        print(self.report(), flush=True)
