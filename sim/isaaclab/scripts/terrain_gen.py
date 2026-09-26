"""시험용 요철 높이맵 (창 모드 climb_test 의 obstacle="gen", 시험 세트 공통). Isaac 의존 없음 (numpy).

좌표: 높이맵 [x, y], 한 칸 hs [m]. 로봇은 x 로 달린다. 두 바퀴 가운데 선 = y 가운데.
  stones   둥근 돌 (가우시안, 높이 0.5h~h, 폭 σ = h~1.5h -> 경사 최대 약 30 deg), m^2 당 약 25 개 = 실제 자갈길에 가까움
  gravel   10 cm 격자마다 0~h 무작위 높이 (블록 잔해). 이웃 칸 사이 수직 턱이 h 까지 — 바퀴 반지름(6 cm) 근처면 물리적으로 못 넘음
  bumps    0.2~0.5 m 크기 턱을 m^2 당 약 1.2 개, 높이 0.5h~h
  waves    2D 파도 (파장 1.2 x 1.6 m), 최고 h
  oneside  gravel 을 왼쪽 차선 (y > 가운데) 에만 -> 왼쪽 바퀴만 요철, 두 바퀴가 늘 다른 높이
  lane_stones  stones 를 0.2 m (바퀴 간격) 차선 하나 건너 하나에만 (돌 중심만 가름, 돌 모양은 그대로 -> 자른 벽 없음)
출발 앞 i0 칸까지 평지, 그 뒤 0.5 m 에 걸쳐 높이를 0 -> 1 배로 (갑자기 h 높이 벽이 되지 않게).
"""
import numpy as np

KINDS = ("stones", "gravel", "bumps", "waves", "oneside", "oneside_stones", "lane_stones")


def make_heights(kind: str, nx: int, ny: int, hs: float, h: float, i0: int, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    x = np.arange(nx)[:, None] * hs
    y = np.arange(ny)[None, :] * hs
    z = np.zeros((nx, ny))
    if kind in ("stones", "oneside_stones", "lane_stones"):
        n = int(nx * ny * hs * hs * 25)
        for _ in range(n):
            cx, cy = rng.uniform(0, nx * hs), rng.uniform(0, ny * hs)
            if kind == "lane_stones" and int(cy // 0.2) % 2:
                continue
            hh, sg = rng.uniform(0.5 * h, h), rng.uniform(1.0, 1.5) * h
            r = int(3 * sg / hs) + 1
            ix, iy = int(cx / hs), int(cy / hs)
            x0_, x1_, y0_, y1_ = max(0, ix - r), min(nx, ix + r), max(0, iy - r), min(ny, iy + r)
            if x1_ <= x0_ or y1_ <= y0_:
                continue
            xx = (np.arange(x0_, x1_)[:, None] * hs - cx) ** 2
            yy = (np.arange(y0_, y1_)[None, :] * hs - cy) ** 2
            z[x0_:x1_, y0_:y1_] = np.maximum(z[x0_:x1_, y0_:y1_], hh * np.exp(-(xx + yy) / (2 * sg * sg)))
        if kind == "oneside_stones":
            z[:, : ny // 2] = 0.0
    elif kind in ("gravel", "oneside"):
        cell = max(1, int(round(0.1 / hs)))
        g = rng.uniform(0.0, h, ((nx + cell - 1) // cell, (ny + cell - 1) // cell))
        z = np.kron(g, np.ones((cell, cell)))[:nx, :ny]
        if kind == "oneside":
            z[:, : ny // 2] = 0.0                                   # 오른쪽 차선은 평지
    elif kind == "bumps":
        for _ in range(int(nx * ny * hs * hs * 1.2)):
            l_, w_ = rng.uniform(0.2, 0.5, 2)
            cx, cy = rng.uniform(0, nx * hs), rng.uniform(0, ny * hs)
            hh = rng.uniform(0.5 * h, h)
            ix0, ix1 = max(0, int((cx - l_ / 2) / hs)), max(0, int((cx + l_ / 2) / hs))
            iy0, iy1 = max(0, int((cy - w_ / 2) / hs)), max(0, int((cy + w_ / 2) / hs))
            z[ix0:ix1, iy0:iy1] = np.maximum(z[ix0:ix1, iy0:iy1], hh)
    elif kind == "waves":
        z = 0.5 * h * (1.0 + np.sin(2 * np.pi * x / 1.2) * np.cos(2 * np.pi * y / 1.6))
    else:
        raise ValueError(kind)
    ramp = np.clip((x[:, 0] - i0 * hs) / 0.5, 0.0, 1.0)
    return z * ramp[:, None]
