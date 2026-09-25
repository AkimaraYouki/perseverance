"""rough_probe.py 결과 JSON 두 개 (전 / 후) -> 마크다운 비교표.

    python3 probe_compare.py before.json after.json [--names 기준선 학습후]
"""
import argparse
import json

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("before")
ap.add_argument("after")
ap.add_argument("--names", nargs=2, default=["전", "후"])
a = ap.parse_args()
B, A = (json.load(open(p))["rows"] for p in (a.before, a.after))
_keys = {(r["terrain"], r["level"], r["mode"]) for r in A}
B = [r for r in B if (r["terrain"], r["level"], r["mode"]) in _keys]      # 같은 조건끼리만 비교
MODES = [m for m in ("manual", "auto") if any(r["mode"] == m for r in A)]
TER = list(dict.fromkeys(r["terrain"] for r in A))
KEYS = [("falls", "낙상", "{:.0f}"), ("isolation", "격리율", "{:.2f}"), ("az_rms", "수직가속 RMS [m/s²]", "{:.2f}"),
        ("roll_rms", "roll RMS [°]", "{:.2f}"), ("pitch_sd", "pitch σ [°]", "{:.2f}"),
        ("stop_drift_cm", "정지 중 밀림 [cm]", "{:.1f}"), ("vx", "실제 vx (0.5 명령)", "{:.3f}"),
        ("h_mean_mm", "주행 중 다리 h [mm]", "{:.0f}")]


def agg(rows, mode, terrain=None, key="falls"):
    rs = [r for r in rows if r["mode"] == mode and (terrain is None or r["terrain"] == terrain)]
    if key == "falls":
        return sum(r["falls"] for r in rs), sum(r["n"] for r in rs)
    v = [r[key] for r in rs if r.get(key) is not None]
    return float(np.mean(v)) if v else None


for m in MODES:
    print(f"\n#### {'수동' if m == 'manual' else '자동'} 모드 — 전체\n")
    print(f"| 지표 | {a.names[0]} | {a.names[1]} |\n|---|---|---|")
    for k, lab, fmt in KEYS:
        if k == "falls":
            (fb, nb), (fa, na) = agg(B, m, key=k), agg(A, m, key=k)
            print(f"| {lab} | {fb}/{nb} | {fa}/{na} |")
        else:
            vb, va = agg(B, m, key=k), agg(A, m, key=k)
            print(f"| {lab} | {fmt.format(vb) if vb is not None else '-'} | {fmt.format(va) if va is not None else '-'} |")
    print(f"\n지형별 (난이도 평균) — 낙상 / 격리율 / roll RMS / 정지 중 밀림, {a.names[0]} → {a.names[1]}\n")
    print("| 지형 | 낙상 | 격리율 | roll RMS [°] | 밀림 [cm] |\n|---|---|---|---|---|")
    for t in TER:
        (fb, nb), (fa, na) = agg(B, m, t, "falls"), agg(A, m, t, "falls")
        cells = []
        for k, fmt in (("isolation", "{:.2f}"), ("roll_rms", "{:.2f}"), ("stop_drift_cm", "{:.1f}")):
            vb, va = agg(B, m, t, k), agg(A, m, t, k)
            cells.append(f"{fmt.format(vb) if vb is not None else '-'} → {fmt.format(va) if va is not None else '-'}")
        print(f"| {t} | {fb}/{nb} → {fa}/{na} | " + " | ".join(cells) + " |")
