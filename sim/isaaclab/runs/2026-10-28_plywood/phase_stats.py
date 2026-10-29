import sys, numpy as np
cols = "gt t wx zw zb ph pitch v edge h stage rt done res h1 nst leg".split(); C = {c: i for i, c in enumerate(cols)}
T = np.load(sys.argv[1]); out = {k: [] for k in ("EXTRACT ms", "FLY 시작 pitch", "DESCEND 시작 pitch", "LAND 시작 pitch", "LAND 시작 h", "착지뒤 최소 v")}
for e in range(T.shape[1]):
    ph = T[:, e, C["ph"]].astype(int); p = np.degrees(T[:, e, C["pitch"]]); t = T[:, e, C["t"]]; v = T[:, e, C["v"]]; h = T[:, e, C["leg"]]
    for k in np.flatnonzero((ph[:-1] == 1) & (ph[1:] == 2)):
        j = k + 1 + np.flatnonzero(ph[k + 1:] != 2)[0] if (ph[k + 1:] != 2).any() else None
        if j: out["EXTRACT ms"].append((t[j] - t[k + 1]) * 1000)
    for k in np.flatnonzero((ph[:-1] == 2) & (ph[1:] == 3)): out["FLY 시작 pitch"].append(p[k + 1])
    for k in np.flatnonzero((ph[:-1] == 3) & (ph[1:] == 4)): out["DESCEND 시작 pitch"].append(p[k + 1])
    for k in np.flatnonzero((ph[:-1] == 4) & (ph[1:] == 5)): out["LAND 시작 pitch"].append(p[k + 1]); out["LAND 시작 h"].append(h[k + 1]); out["착지뒤 최소 v"].append(v[k + 1:k + 200].min())
print(" | ".join(f"{k} {np.median(v):.3g}" for k, v in out.items() if v))
