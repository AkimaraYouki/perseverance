"""히트박스 그림: CAD z 투영 (서기·앉기·최고) + 합집합 + 앞뒤좌우 여유 히트박스 + LiDAR. make_footprint.py --npz 출력을 읽는다.
    python3 scripts/plot_hitbox.py <npz> <out.png>"""
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for f in ("NanumGothic", "Noto Sans CJK KR", "NanumBarunGothic"):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = f
        break
plt.rcParams["axes.unicode_minus"] = False
INK, MUTED, SURF = "#0b0b0b", "#6b6a63", "#fcfcfb"
POSE = [("high", "최고 (h 0.2425)", "#1baf7a"), ("sit", "앉기 (h 0.1255)", "#eb6834"), ("stand_idle", "서기 (h 0.1825)", "#2a78d6")]

z = np.load(sys.argv[1])
hb, ub, lz = z["hitbox"], z["union"], z["laser"]
fig, ax = plt.subplots(figsize=(8.2, 7.4), facecolor=SURF)
ax.set_facecolor(SURF)
cm = 100.0   # 그림은 cm
for key, lab, col in POSE:
    P = z[key]
    ax.scatter(P[:, 0] * cm, P[:, 1] * cm, s=1.0, color=col, alpha=0.25, linewidths=0, rasterized=True)
    ax.plot([], [], ls="none", marker="o", ms=6, color=col, label=lab)
# 합집합 박스 (여유 없음)
ax.add_patch(plt.Rectangle((ub[0] * cm, ub[2] * cm), (ub[1] - ub[0]) * cm, (ub[3] - ub[2]) * cm, fill=False, ec=INK, lw=1.4,
                           label="CAD 합집합 (세 자세)"))
# 히트박스
ax.add_patch(plt.Rectangle((hb[0] * cm, hb[2] * cm), (hb[1] - hb[0]) * cm, (hb[3] - hb[2]) * cm, fc="#2a78d6", alpha=0.07, ec="none"))
ax.add_patch(plt.Rectangle((hb[0] * cm, hb[2] * cm), (hb[1] - hb[0]) * cm, (hb[3] - hb[2]) * cm, fill=False,
                           ec="#184f95", lw=2.2, ls=(0, (6, 3)), label="히트박스 (+5 cm)"))
# LiDAR
ax.plot(lz[0] * cm, lz[1] * cm, marker="o", ms=9, mfc=SURF, mec=INK, mew=2, ls="none", label="LiDAR 광학 중심")
ax.annotate("", xy=(lz[0] * cm + 6, lz[1] * cm), xytext=(lz[0] * cm, lz[1] * cm), arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.5))
ax.text(lz[0] * cm + 6.5, lz[1] * cm + 0.8, "0° (실측: 앞)", fontsize=9, color=INK)
# 실기가 자기 몸으로 본 범위 (10-09 로봇 보고): r 0.08–0.33 m, -22…-67°
a = np.radians(np.linspace(-67, -22, 60))
wedge = np.concatenate([np.c_[0.08 * np.cos(a), 0.08 * np.sin(a)], np.c_[0.33 * np.cos(a[::-1]), 0.33 * np.sin(a[::-1])]])
ax.fill((wedge[:, 0] + lz[0]) * cm, (wedge[:, 1] + lz[1]) * cm, fc="none", ec=MUTED, hatch="////", lw=1.0,
        label="실기에서 '자기 몸'으로 본 점 (10-09 보고)")
# 치수
ax.text((hb[0] + hb[1]) / 2 * cm, hb[3] * cm + 1.2, f"{(hb[1] - hb[0]) * cm:.1f} cm", ha="center", fontsize=10, color=INK)
ax.text(hb[1] * cm + 1.0, 0, f"{(hb[3] - hb[2]) * cm:.1f} cm", va="center", rotation=90, fontsize=10, color=INK)
for xv, ha in ((hb[0], "right"), (hb[1], "left")):
    ax.text(xv * cm + (-0.6 if ha == "right" else 0.6), hb[2] * cm - 2.2, f"x {xv * cm:+.1f}", ha=ha, fontsize=8.5, color=MUTED)
ax.text(hb[0] * cm - 0.6, hb[2] * cm, f"y {hb[2] * cm:+.1f}", ha="right", va="bottom", fontsize=8.5, color=MUTED)
ax.text(hb[0] * cm - 0.6, hb[3] * cm, f"y {hb[3] * cm:+.1f}", ha="right", va="top", fontsize=8.5, color=MUTED)
ax.annotate("앞 (+x)", xy=(30, -30), xytext=(21, -30), fontsize=10, color=INK, va="center", arrowprops=dict(arrowstyle="-|>", color=INK))
ax.set_aspect("equal")
ax.set_xlim(-30, 40); ax.set_ylim(-36, 24)
ax.set_xlabel("x [cm] (base_link, 앞 +)", color=INK); ax.set_ylabel("y [cm] (왼쪽 +)", color=INK)
ax.grid(True, color="#e6e5df", lw=0.6); ax.set_axisbelow(True)
for s in ax.spines.values():
    s.set_color("#c3c2b7")
ax.tick_params(colors=MUTED)
lg = ax.legend(loc="lower left", fontsize=8.5, frameon=True)
lg.get_frame().set_edgecolor("#c3c2b7")
ax.set_title("C-WANG 자기 몸 히트박스 — CAD z 투영 + 앞뒤좌우 5 cm (위에서 본 모습)", fontsize=12, color=INK, loc="left")
fig.tight_layout()
fig.savefig(sys.argv[2], dpi=150, facecolor=SURF)
print("[저장]", sys.argv[2])
