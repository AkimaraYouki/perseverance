"""jump_rl --trace_envs 기록 -> 접근·후진·재가속·점프 궤적 그림 (PNG).

    python3 plot_jump_trace.py TRACE.npy OUT.png [--title 제목] [--n 4]

위: 시간-위치 (x 와 바퀴 바닥 높이, 턱 윤곽, 단계 색).  가운데: 옆에서 본 바퀴 바닥 경로 (x-z).  아래: 속도·pitch.
열 순서는 jump_rl.py 의 trace (gt t wx zw zb ph pitch v edge h stage rt done res h1 nst leg).
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for f_ in ("/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf", "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"):
    try:
        font_manager.fontManager.addfont(f_)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=f_).get_name()
        break
    except Exception:  # noqa: BLE001
        pass
plt.rcParams["axes.unicode_minus"] = False

COLS = "gt t wx zw zb ph pitch v edge h stage rt done res h1 nst leg".split()
C = {c: i for i, c in enumerate(COLS)}
PH = {0: ("주행", "#9aa5b1"), 1: ("웅크림", "#e0a800"), 2: ("신전", "#e8590c"), 3: ("공중", "#c92a2a"), 4: ("하강", "#862e9c"), 5: ("착지", "#1971c2")}
RT = {1: "후진", 2: "재가속"}
RES = {0: "성공", 1: "턱 앞", 2: "넘어짐", 3: "시간"}

ap = argparse.ArgumentParser()
ap.add_argument("trace")
ap.add_argument("out")
ap.add_argument("--title", default="")
ap.add_argument("--n", type=int, default=4)
a = ap.parse_args()
T = np.load(a.trace)

eps = []                       # (로봇, 시작 스텝, 끝 스텝) — 각 로봇의 첫 판
for e in range(T.shape[1]):
    d = np.flatnonzero(T[:, e, C["done"]] > 0)
    if len(d):
        eps.append((e, 0, int(d[0])))
# 결과가 섞이게 고르기: 성공 위주 + 실패 있으면 하나
eps.sort(key=lambda x: T[x[2], x[0], C["res"]])
pick = [p for p in eps if T[p[2], p[0], C["res"]] == 0][: max(1, a.n - 1)] + [p for p in eps if T[p[2], p[0], C["res"]] != 0][:1]

fig, axs = plt.subplots(len(pick), 3, figsize=(17, 3.6 * len(pick)), squeeze=False)
for row, (e, s0, s1) in zip(axs, pick):
    seg = T[s0:s1 + 1, e]
    t, x, zw, ph, rt = seg[:, C["t"]], seg[:, C["wx"]], seg[:, C["zw"]], seg[:, C["ph"]].astype(int), seg[:, C["rt"]].astype(int)
    v, pitch = seg[:, C["v"]], np.degrees(seg[:, C["pitch"]])
    h1, nst, e0 = seg[0, C["h1"]], int(seg[0, C["nst"]]), seg[0, C["edge"]]
    res = RES.get(int(seg[-1, C["res"]]), "?")
    # 턱 윤곽 (첫 모서리 e0, 단 높이 h1, 2 단이면 1 m 뒤 2 h1)
    xs = [x.min() - 0.1, e0, e0]
    zs = [0, 0, h1]
    if nst == 2:
        xs += [e0 + 1.0, e0 + 1.0, max(x.max(), e0 + 1.4) + 0.1]
        zs += [h1, 2 * h1, 2 * h1]
    else:
        xs += [max(x.max(), e0 + 0.5) + 0.1]
        zs += [h1]
    # 1) 시간-위치
    ax = row[0]
    for k in range(len(t) - 1):
        ax.plot(t[k:k + 2], x[k:k + 2], color=PH[ph[k]][1], lw=2.2)
    for k in np.flatnonzero(np.diff(rt) != 0):
        if rt[k + 1] in RT:
            ax.axvline(t[k + 1], color="#495057", ls=":", lw=1)
            ax.text(t[k + 1], ax.get_ylim()[1] if False else x.max(), RT[rt[k + 1]], fontsize=8, rotation=90, va="top")
    ax.axhline(e0, color="k", lw=0.8, ls="--")
    if nst == 2:
        ax.axhline(e0 + 1.0, color="k", lw=0.8, ls="--")
    ax.set_xlabel("시간 [s]"); ax.set_ylabel("바퀴축 x [m]")  # noqa: E702
    ax.set_title(f"로봇 {e}: {'2 단' if nst == 2 else '단'} {h1*100:.0f} cm → {res}", fontsize=10)
    # 2) 옆에서 본 경로
    ax = row[1]
    ax.fill_between(xs, zs, -0.02, step=None, color="#dee2e6")
    ax.plot(xs, zs, color="k", lw=1)
    for k in range(len(t) - 1):
        ax.plot(x[k:k + 2], zw[k:k + 2], color=PH[ph[k]][1], lw=2.2)
    ax.set_xlabel("x [m]"); ax.set_ylabel("바퀴 바닥 z [m]")  # noqa: E702
    ax.set_ylim(-0.02, max(zs) + 0.18)
    ax.set_title("옆에서 본 바퀴 경로 (색 = 단계)", fontsize=10)
    # 3) 속도·pitch
    ax = row[2]
    ax.plot(t, v, color="#1864ab", lw=1.5, label="속도 [m/s]")
    ax.axhline(0.3, color="#1864ab", lw=0.6, ls=":")
    ax2 = ax.twinx()
    ax2.plot(t, pitch, color="#c92a2a", lw=1.2, label="pitch [deg]")
    for k in range(len(t) - 1):
        if ph[k] != 0:
            ax.axvspan(t[k], t[k + 1], color=PH[ph[k]][1], alpha=0.12, lw=0)
    ax.set_xlabel("시간 [s]"); ax.set_ylabel("속도 [m/s]", color="#1864ab"); ax2.set_ylabel("pitch [deg]", color="#c92a2a")  # noqa: E702
    ax.set_title("속도 (점선 0.3 m/s) · 몸 기울기", fontsize=10)
hand = [plt.Line2D([], [], color=c, lw=3, label=n) for n, c in PH.values()]
fig.legend(handles=hand, loc="upper center", ncol=6, frameon=False, bbox_to_anchor=(0.5, 1.0))
if a.title:
    fig.suptitle(a.title, y=1.03, fontsize=13)
fig.tight_layout(rect=(0, 0, 1, 0.97))
fig.savefig(a.out, dpi=110, bbox_inches="tight")
print("저장", a.out)
