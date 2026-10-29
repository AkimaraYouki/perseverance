"""균형 게인 (LQR 성분 배율 4) + 롤 게인 4 CMA-ES — 실기 플랜트 험지 (2026-10-24). Isaac 없이 도는 구동기: 세대마다 harsh_eval 한 번.

    python3 gain_cma.py --gens 20 --plant "--wheel_tau_gain 0.83 ..."

세대마다 후보 = 표본 (pop - 2) + 지금 평균 + '지금 실기 게인' (고정, 기준). 로봇 N 대를 후보 i % C 로 나눔 (지형 칸이 연속 블록이라 고르게).
점수 = 성공률 - 3 x 고관절 진동 보호 발동률 - w_chat x max(0, 떨림 - 1.15 x 기준 후보 떨림)
  (진동 보호 = 실기 balance_node 판정 흉내, 떨림 = 좌우 고관절 하중 차 1 Hz 고역 RMS — 롤 CMA 에서 kp 2.9 가 4 Hz 로 떨었던 것 막기).
LQR 배율은 실기 이득 표 (--plant 의 lqr_table) 에 곱함. 롤 시작점 = 롤1.
"""
import argparse
import json
import os
import subprocess
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SPACE = [("lqr_mx", 0.6, 1.5), ("lqr_mv", 0.6, 1.5), ("lqr_mth", 0.7, 1.4), ("lqr_mthd", 0.6, 1.4),
         ("roll_kp", 0.5, 2.0), ("roll_ki", 0.0, 10.0), ("roll_kd", 0.0, 0.4), ("roll_rate_lpf_hz", 2.0, 16.0)]
X0 = dict(lqr_mx=1.0, lqr_mv=1.0, lqr_mth=1.0, lqr_mthd=1.0, roll_kp=1.5, roll_ki=4.0, roll_kd=0.2, roll_rate_lpf_hz=8.0)   # 롤1
NOW = dict(lqr_mx=1.0, lqr_mv=1.0, lqr_mth=1.0, lqr_mthd=1.0, roll_kp=1.0, roll_ki=8.0, roll_kd=0.1, roll_rate_lpf_hz=4.0)  # 지금 실기

ap = argparse.ArgumentParser()
ap.add_argument("--gens", type=int, default=20)
ap.add_argument("--pop", type=int, default=11, help="후보 수 (표본 pop-2 + 평균 + 지금)")
ap.add_argument("--num_envs", type=int, default=2048)
ap.add_argument("--plant", default="")
ap.add_argument("--w_chat", type=float, default=0.3)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--run", default="gain")
a = ap.parse_args()

IL = os.path.expanduser("~/Desktop/IsaacLab/isaaclab.sh")
out = os.path.join(os.path.dirname(HERE), "logs", "gain_cma", time.strftime("%Y-%m-%d_%H-%M-%S") + "_" + a.run)
os.makedirs(out, exist_ok=True)
lo = np.array([s[1] for s in SPACE]); hi = np.array([s[2] for s in SPACE])  # noqa: E702
dec = lambda z: lo + (hi - lo) * np.clip(z, 0, 1)   # noqa: E731
names = [s[0] for s in SPACE]


def evaluate(xs, tag, seed):
    """후보 목록 (마지막 = 기준 '지금') -> 후보마다 [점수, 성공, 진동 보호, 떨림, 지형별 성공 dict]."""
    C = len(xs)
    cf = os.path.join(out, f"cands_{tag}.json"); rf = os.path.join(out, f"rows_{tag}.json")
    json.dump([dict(zip(names, map(float, x))) for x in xs], open(cf, "w"))
    for _ in range(2):                                     # Isaac 시작 segfault 면 한 번 더
        if os.path.exists(rf):
            break
        with open(os.path.join(out, f"log_{tag}.txt"), "w") as fh:
            subprocess.run(["env", "-u", "PYTHONPATH", IL, "-p", "scripts/_isaaclab_launch.py", "scripts/harsh_eval.py",
                            "--num_envs", str(a.num_envs), "--cands", cf, "--rows_out", rf, "--seed", str(seed)] + a.plant.split(),
                           cwd=os.path.dirname(HERE), stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, timeout=2400)
    rows = json.load(open(rf))
    ok = np.zeros(C); osc = np.zeros(C); n = np.zeros(C); ch = [[] for _ in range(C)]; ter = [{} for _ in range(C)]  # noqa: E702
    for r in rows:
        c = r["i"] % C
        ok[c] += not r["fell"]; osc[c] += r["osc"]; n[c] += 1  # noqa: E702
        if r["chat"] is not None and not r["fell"]:
            ch[c].append(r["chat"])
        t_ = ter[c].setdefault(r["terrain"], [0, 0]); t_[0] += not r["fell"]; t_[1] += 1  # noqa: E702
    succ, oscr = ok / np.maximum(n, 1), osc / np.maximum(n, 1)
    chat = np.array([np.median(v) if v else 9.0 for v in ch])
    score = succ - 3.0 * oscr - a.w_chat * np.maximum(0.0, chat - 1.15 * chat[-1])
    return score, succ, oscr, chat, ter


D, lam = len(SPACE), a.pop - 2
mu = lam // 2; w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1)); w /= w.sum(); mueff = 1 / (w ** 2).sum()  # noqa: E702
cc, cs = (4 + mueff / D) / (D + 4 + 2 * mueff / D), (mueff + 2) / (D + mueff + 5)
c1 = 2 / ((D + 1.3) ** 2 + mueff); cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((D + 2) ** 2 + mueff))
ds = 1 + 2 * max(0, np.sqrt((mueff - 1) / (D + 1)) - 1) + cs; chiN = np.sqrt(D) * (1 - 1 / (4 * D) + 1 / (21 * D * D))  # noqa: E702
m = (np.array([X0[k] for k in names]) - lo) / (hi - lo); sig = 0.15; C_ = np.eye(D); pc = np.zeros(D); ps = np.zeros(D)  # noqa: E702
rng = np.random.default_rng(a.seed); best = (-9.0, None, None)  # noqa: E702
xnow = np.array([NOW[k] for k in names])
print(f"[게인 CMA] 파라미터 {D}, 후보 {lam}+평균+지금, 세대 {a.gens}, 로봇 {a.num_envs} -> {out}", flush=True)
for gen in range(a.gens):
    t0 = time.time()
    ev, B = np.linalg.eigh(C_); Dd = np.sqrt(np.maximum(ev, 1e-12))  # noqa: E702
    zs = rng.standard_normal((lam, D)); ys = zs @ (B * Dd).T; xs = m + sig * ys  # noqa: E702
    sc, su, os_, ch, _ = evaluate([dec(x) for x in xs] + [dec(m), xnow], f"g{gen:02d}", seed=1 + gen)
    rel = sc[-2] - sc[-1]                                       # 평균 - 지금 (같은 로봇·지형이라 노이즈가 줄어든 비교)
    if rel > best[0]:
        best = (rel, dec(m), dict(score=float(sc[-2]), succ=float(su[-2]), osc=float(os_[-2]), chat=float(ch[-2]), now_succ=float(su[-1])))
    o = np.argsort(-sc[:lam]); yw = (w[:, None] * ys[o[:mu]]).sum(0)  # noqa: E702
    m = np.clip(m + sig * yw, 0, 1)
    Ci = B @ np.diag(1 / Dd) @ B.T
    ps = (1 - cs) * ps + np.sqrt(cs * (2 - cs) * mueff) * Ci @ yw
    hs = np.linalg.norm(ps) / np.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) < (1.4 + 2 / (D + 1)) * chiN
    pc = (1 - cc) * pc + hs * np.sqrt(cc * (2 - cc) * mueff) * yw
    C_ = (1 - c1 - cmu) * C_ + c1 * (np.outer(pc, pc) + (1 - hs) * cc * (2 - cc) * C_) + cmu * (w[:, None] * ys[o[:mu]]).T @ ys[o[:mu]]
    sig = min(0.5, sig * np.exp((cs / ds) * (np.linalg.norm(ps) / chiN - 1)))
    rec = dict(gen=gen, sigma=round(float(sig), 4), mean_succ=round(float(su[-2]), 4), now_succ=round(float(su[-1]), 4),
               mean_osc=round(float(os_[-2]), 4), mean_chat=round(float(ch[-2]), 3), now_chat=round(float(ch[-1]), 3),
               top=round(float(sc[o[0]]), 4), s=round(time.time() - t0, 1), mean=dict(zip(names, dec(m).round(4).tolist())))
    with open(os.path.join(out, "gens.jsonl"), "a") as fh:
        fh.write(json.dumps(rec) + "\n")
    print(f"[게인 CMA {gen:3d}] 평균 성공 {su[-2]:.1%} (지금 {su[-1]:.1%}) 진동 {os_[-2]:.1%} 떨림 {ch[-2]:.2f} (지금 {ch[-1]:.2f}) | "
          f"1등 점수 {sc[o[0]]:+.3f} | sigma {sig:.3f} | {time.time() - t0:.0f} s", flush=True)
bx = dict(zip(names, best[1].round(4).tolist()))
json.dump(dict(best=bx, rel=best[0], stats=best[2]), open(os.path.join(out, "best.json"), "w"), indent=1)
print("[게인 CMA] 끝 best =", bx, flush=True)
# 검증: 지금 / 롤1 / best — 로봇 2 배, 새 시드 (후보마다 같은 로봇·지형)
a.num_envs *= 2
sc, su, os_, ch, ter = evaluate([np.array([X0[k] for k in names]), np.array([bx[k] for k in names]), xnow], "verify", seed=777)
with open(os.path.join(out, "verify.txt"), "w") as fh:
    for nm, i in (("지금", 2), ("롤1", 0), ("best", 1)):
        tl = " ".join(f"{k} {v[0]}/{v[1]}" for k, v in sorted(ter[i].items()))
        line = f"[검증 {nm}] 성공 {su[i]:.1%} 진동 보호 {os_[i]:.1%} 떨림 {ch[i]:.2f} | {tl}"
        print(line, flush=True); fh.write(line + "\n")  # noqa: E702
