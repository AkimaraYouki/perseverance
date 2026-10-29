"""롤 게인 + 다리 접지 반사 CMA-ES (2026-10-24). Isaac 없이 도는 구동기: 세대마다 robust_suite --scenario rollmix 한 번.

    python3 roll_cma.py --gens 25 --reps 3 [--plant "--wheel_tau_gain 0.83 ..."]

세대마다 후보 C (= pop, 마지막은 지금 평균) x 지형 7 종 (ROLLMIX) x reps 대 = 로봇 N 대를 한 번에.
점수 (후보마다, 그 로봇들 평균) = 통과 - 넘어짐 - 0.02 x max(0, 최대 롤 - 5°) - 3 x 고관절 진동 보호 발동
  (진동 보호 = 실기 balance_node 판정 흉내: 요철에서 진동으로 오판해 주저앉는 일 없게).
jump_rl.py 의 CMA 와 같은 갱신식, 탐색은 [0, 1] 정규화 공간.
"""
import argparse
import json
import os
import subprocess
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SPACE = [("roll_kp", 0.5, 2.0), ("roll_ki", 0.0, 12.0), ("roll_kd", 0.0, 0.5), ("roll_rate_lpf_hz", 2.0, 20.0),
         ("drop_tau", 0.2, 1.2), ("drop_s", 0.005, 0.06), ("drop_rate", 0.5, 3.0)]
X0 = dict(roll_kp=1.5, roll_ki=4.0, roll_kd=0.2, roll_rate_lpf_hz=8.0, drop_tau=0.5, drop_s=0.02, drop_rate=1.5)   # A + 반사
NKIND = 8

ap = argparse.ArgumentParser()
ap.add_argument("--gens", type=int, default=25)
ap.add_argument("--pop", type=int, default=10, help="후보 수 (마지막 = 평균)")
ap.add_argument("--reps", type=int, default=3, help="지형 종류마다 후보 하나당 로봇 수")
ap.add_argument("--plant", default="", help="robust_suite 에 넘길 플랜트 인자")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--run", default="roll")
ap.add_argument("--smoke", action="store_true", help="평가 한 번만 (지형·후보 배치 확인)")
ap.add_argument("--calib", action="store_true", help="지금 / A / A+반사 / 옛 best 의 떨림 지표만 재기")
ap.add_argument("--chat_ref", type=float, default=9.0, help="떨림 벌점 기준 [N·m] (지금 설정 수준)")
ap.add_argument("--w_chat", type=float, default=2.0, help="떨림 벌점 무게 [1/N·m]")
a = ap.parse_args()

IL = os.path.expanduser("~/Desktop/IsaacLab/isaaclab.sh")
out = os.path.join(os.path.dirname(HERE), "logs", "roll_cma", time.strftime("%Y-%m-%d_%H-%M-%S") + "_" + a.run)
os.makedirs(out, exist_ok=True)
lo = np.array([s[1] for s in SPACE]); hi = np.array([s[2] for s in SPACE])  # noqa: E702
dec = lambda z: lo + (hi - lo) * np.clip(z, 0, 1)   # noqa: E731
names = [s[0] for s in SPACE]


def evaluate(xs, tag):
    """후보 목록 -> (점수, 통과율, 넘어짐률, 진동 보호 비율, 최대 롤 중간값) 후보마다."""
    C = len(xs); N = C * NKIND * a.reps  # noqa: E702
    cf = os.path.join(out, f"cands_{tag}.json"); rf = os.path.join(out, f"res_{tag}.json")
    json.dump([dict(zip(names, map(float, x))) for x in xs], open(cf, "w"))
    for _ in range(2):                                     # Isaac 시작 segfault 면 한 번 더
        if os.path.exists(rf):
            break
        with open(os.path.join(out, f"log_{tag}.txt"), "w") as fh:
            subprocess.run(["env", "-u", "PYTHONPATH", IL, "-p", "scripts/_isaaclab_launch.py", "scripts/robust_suite.py",
                            "--scenario", "rollmix", "--n", str(N), "--cands", cf, "--out", rf, "--seed", str(100 + 1000 * a.seed)]
                           + a.plant.split(), cwd=os.path.dirname(HERE), stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                           timeout=1800)
    rows = json.load(open(rf))["rows"]
    S = np.zeros((C, 6)); cnt = np.zeros(C)  # noqa: E702
    rm = [[] for _ in range(C)]; ch = [[] for _ in range(C)]  # noqa: E702
    for r in rows:
        c = r["i"] % C
        osc = r.get("osc_t") is not None; fell = r["fell_t"] is not None; rmax = r.get("roll_max") or 0.0
        chat = r.get("chat")
        if chat is not None and not fell:
            ch[c].append(chat)
        S[c, 0] += float(r["ok"]) - float(fell) - 0.02 * max(0.0, rmax - 5.0) - 3.0 * float(osc)
        S[c, 1] += float(r["ok"]); S[c, 2] += float(fell); S[c, 3] += float(osc)  # noqa: E702
        rm[c].append(rmax); cnt[c] += 1
    S[:, :4] /= np.maximum(cnt, 1)[:, None]
    S[:, 4] = [np.median(v) if v else 0.0 for v in rm]
    S[:, 5] = [np.median(v) if v else 9.0 for v in ch]          # 좌우 고관절 하중 차 떨림 (1 Hz 고역 RMS, N·m)
    S[:, 0] -= a.w_chat * np.maximum(0.0, S[:, 5] - a.chat_ref)   # 지금 수준보다 떨면 벌점
    return S


if a.calib:
    R_ = [dict(roll_kp=1.0, roll_ki=8.0, roll_kd=0.1, roll_rate_lpf_hz=4.0, drop_tau=0.0, drop_s=0.02, drop_rate=1.5), dict(X0, drop_tau=0.0), dict(X0),
          dict(roll_kp=2.9435, roll_ki=4.7684, roll_kd=0.0447, roll_rate_lpf_hz=8.663, drop_tau=0.6131, drop_s=0.0098, drop_rate=1.7533)]
    S = evaluate([np.array([d[k] for k in names]) for d in R_], "calib")
    for nm, s_ in zip(("지금", "A", "A+반사", "옛best"), S):
        print(f"{nm}: 통과 {s_[1]:.0%} 넘어짐 {s_[2]:.0%} 진동 {s_[3]:.0%} 최대롤 {s_[4]:.1f} 떨림 {s_[5]:.3f}", flush=True)
    raise SystemExit
if a.smoke:
    S = evaluate([np.array([X0[k] for k in names]), np.array([X0[k] if k != "drop_tau" else 0.0 for k in names])], "smoke")
    print(S.round(3)); raise SystemExit
D, lam = len(SPACE), a.pop - 1
mu = lam // 2; w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1)); w /= w.sum(); mueff = 1 / (w ** 2).sum()  # noqa: E702
cc, cs = (4 + mueff / D) / (D + 4 + 2 * mueff / D), (mueff + 2) / (D + mueff + 5)
c1 = 2 / ((D + 1.3) ** 2 + mueff); cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((D + 2) ** 2 + mueff))
ds = 1 + 2 * max(0, np.sqrt((mueff - 1) / (D + 1)) - 1) + cs; chiN = np.sqrt(D) * (1 - 1 / (4 * D) + 1 / (21 * D * D))  # noqa: E702
m = (np.array([X0[k] for k in names]) - lo) / (hi - lo); sig = 0.15; C_ = np.eye(D); pc = np.zeros(D); ps = np.zeros(D)  # noqa: E702
rng = np.random.default_rng(a.seed); best = (-9.0, None)  # noqa: E702
print(f"[롤 CMA] 파라미터 {D}, 후보 {lam}+평균, 세대 {a.gens}, 로봇 {a.pop * NKIND * a.reps} -> {out}", flush=True)
for gen in range(a.gens):
    t0 = time.time()
    ev, B = np.linalg.eigh(C_); Dd = np.sqrt(np.maximum(ev, 1e-12))  # noqa: E702
    zs = rng.standard_normal((lam, D)); ys = zs @ (B * Dd).T; xs = m + sig * ys  # noqa: E702
    S = evaluate([dec(x) for x in xs] + [dec(m)], f"g{gen:02d}")
    sc = S[:, 0]
    if sc[-1] > best[0]:
        best = (sc[-1], dec(m), S[-1])
    o = np.argsort(-sc[:lam]); yw = (w[:, None] * ys[o[:mu]]).sum(0)  # noqa: E702
    m = np.clip(m + sig * yw, 0, 1)
    Ci = B @ np.diag(1 / Dd) @ B.T
    ps = (1 - cs) * ps + np.sqrt(cs * (2 - cs) * mueff) * Ci @ yw
    hs = np.linalg.norm(ps) / np.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) < (1.4 + 2 / (D + 1)) * chiN
    pc = (1 - cc) * pc + hs * np.sqrt(cc * (2 - cc) * mueff) * yw
    C_ = (1 - c1 - cmu) * C_ + c1 * (np.outer(pc, pc) + (1 - hs) * cc * (2 - cc) * C_) + cmu * (w[:, None] * ys[o[:mu]]).T @ ys[o[:mu]]
    sig = min(0.5, sig * np.exp((cs / ds) * (np.linalg.norm(ps) / chiN - 1)))
    rec = dict(gen=gen, sigma=round(float(sig), 4), mean_score=round(float(sc[-1]), 3), top_score=round(float(sc[o[0]]), 3),
               mean_pass=round(float(S[-1, 1]), 3), mean_fell=round(float(S[-1, 2]), 3), mean_osc=round(float(S[-1, 3]), 3),
               mean_rollmax=round(float(S[-1, 4]), 1), s=round(time.time() - t0, 1), mean=dict(zip(names, dec(m).round(4).tolist())))
    with open(os.path.join(out, "gens.jsonl"), "a") as fh:
        fh.write(json.dumps(rec) + "\n")
    print(f"[롤 CMA {gen:3d}] 평균후보 통과 {S[-1, 1]:.0%} 넘어짐 {S[-1, 2]:.0%} 진동 {S[-1, 3]:.0%} 최대롤 {S[-1, 4]:.1f}° 점수 {sc[-1]:+.3f} | "
          f"1등 {sc[o[0]]:+.3f} | sigma {sig:.3f} | {time.time() - t0:.0f} s", flush=True)
bx = dict(zip(names, best[1].round(4).tolist()))
json.dump(dict(best=bx, score=best[0], stats=best[2].tolist()), open(os.path.join(out, "best.json"), "w"), indent=1)
print("[롤 CMA] 끝 best =", bx, flush=True)
# 검증: 지금 / A / A+반사 / best 를 같은 지형 섞음으로 (reps 2 배)
REF = [("지금", dict(roll_kp=1.0, roll_ki=8.0, roll_kd=0.1, roll_rate_lpf_hz=4.0, drop_tau=0.0, drop_s=0.02, drop_rate=1.5)),
       ("A", dict(X0, drop_tau=0.0)), ("A+반사", dict(X0)), ("best", bx)]
a.reps *= 2
S = evaluate([np.array([d[k] for k in names]) for _, d in REF], "verify")
with open(os.path.join(out, "verify.txt"), "w") as fh:
    for (nm, _), s_ in zip(REF, S):
        line = f"[검증 {nm}] 점수 {s_[0]:+.3f} 통과 {s_[1]:.1%} 넘어짐 {s_[2]:.1%} 진동 보호 {s_[3]:.1%} 최대롤 중간 {s_[4]:.1f}°"
        print(line, flush=True); fh.write(line + "\n")  # noqa: E702
