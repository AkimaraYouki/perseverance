"""실기 제어기 (gen2_control) 용 모델 표 — balance_tables.yaml (Isaac 불필요, URDF 기구학).

URDF (sim/model/cad_export7_robot.urdf = 시뮬 기본 모델 usd_loop 와 같은 질량·형상) 에서 4절 링크를 직접 푼다:
크랭크 각 θ 를 주면 고리 (로커 P 점 = 몸통 P 점) 가 닫히도록 수동 관절 I, K 를 뉴턴법으로 구하고,
바퀴 중심·다리 링크 질량중심·정강이 회전을 base_link 좌표로 낸다. 확인: |바퀴 중심| + R 이 leg_map.h_of_theta 와 맞는지,
M = 0 에서 구한 LQR 관성이 시뮬 build_lqr 값 (Isaac) 과 맞는지.
LQR 이득은 climb_test.build_lqr() 와 같은 식 (CAD 영점 자세 M = 0).
로봇 요청: comms/to-desktop/2026-10-07_robot-controller-port-request.md, ..._robot-balance-tables-format.md

    python3 scripts/make_balance_tables.py [--out ~/perseverance/sim/model/balance_tables.yaml]
"""
import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.expanduser("~/perseverance/sim/model")
sys.path.insert(0, HERE)
sys.path.insert(0, MODEL)
import leg_map  # noqa: E402
import lqr_vmc  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--urdf", default=os.path.join(MODEL, "cad_export7_robot.urdf"))
ap.add_argument("--loop", default=os.path.join(MODEL, "cad_export7_loop_closure.json"))
ap.add_argument("--out", default=os.path.join(MODEL, "balance_tables.yaml"))
ap.add_argument("--step_deg", type=float, default=0.5)
ap.add_argument("--I_pend", type=float, default=0.02912,
                help="LQR 진자 관성 [kg·m²]. 기본 = 시뮬 build_lqr 이 Isaac 에서 쓰는 값 (PhysX 링크 관성 yy). 0 이면 URDF 로 계산")
args = ap.parse_args()

src = open(os.path.join(HERE, "climb_test.py")).read()
i0 = src.index("TUNE = dict(")
ns = {}
exec(src[i0:src.index("\n)\n", i0) + 3], ns)
T = ns["TUNE"]
THETA0 = math.radians(45.002)                                      # cad.THETA0
WHEEL_IZZ = 1.8414e-3                                              # cad.WHEEL_IZZ (R 70 mm, 바퀴 실측)
M_SIGN = {"L": 1.0, "R": -1.0}


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_mat(a, q):
    a = np.asarray(a, float) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def tf(R, p):
    Tm = np.eye(4); Tm[:3, :3] = R; Tm[:3, 3] = p
    return Tm


root = ET.parse(args.urdf).getroot()
J = {}
for j in root.findall("joint"):
    o = j.find("origin"); a = j.find("axis")
    J[j.get("name")] = dict(parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                            xyz=[float(v) for v in o.get("xyz").split()], rpy=[float(v) for v in o.get("rpy").split()],
                            axis=[float(v) for v in a.get("xyz").split()] if a is not None else [0, 0, 1])
LINK = {}
for l in root.findall("link"):
    i = l.find("inertial")
    if i is None:
        continue
    I_ = i.find("inertia").attrib
    LINK[l.get("name")] = dict(m=float(i.find("mass").get("value")), com=np.array([float(v) for v in i.find("origin").get("xyz").split()]),
                               I=np.array([[float(I_["ixx"]), float(I_["ixy"]), float(I_["ixz"])],
                                           [float(I_["ixy"]), float(I_["iyy"]), float(I_["iyz"])],
                                           [float(I_["ixz"]), float(I_["iyz"]), float(I_["izz"])]]))
LOOP = json.load(open(args.loop))


def jt(name, q=0.0):
    j = J[name]
    return tf(rpy_mat(*j["rpy"]), j["xyz"]) @ tf(axis_mat(j["axis"], q), [0, 0, 0])


def leg_fk(S, qM, qI, qK):
    """S = 'L' | 'R'. 반환: base_link 기준 각 링크 4x4."""
    s = S.lower()
    Tc = jt(f"{S}_joint_M", qM)
    Ts = Tc @ jt(f"{S}_joint_I", qI)
    Tr = Ts @ jt(f"{S}_joint_K", qK)
    Tw = Ts @ jt(f"{S}_joint_W", 0.0)
    return {f"{s}_crank": Tc, f"{s}_shank": Ts, f"{s}_rocker": Tr, f"{s}_wheel": Tw}


def loop_res(S, qM, qI, qK):
    Tr = leg_fk(S, qM, qI, qK)[f"{S.lower()}_rocker"]
    p = Tr @ np.r_[LOOP[S]["p_rocker"], 1.0]
    return p[:3] - np.asarray(LOOP[S]["p_body"])


def solve(S, qM, guess):
    q = np.array(guess, float)
    for _ in range(50):
        r = loop_res(S, qM, *q)
        if np.linalg.norm(r) < 1e-11:
            break
        Jm = np.zeros((3, 2))
        for k in range(2):
            dq = np.zeros(2); dq[k] = 1e-7
            Jm[:, k] = (loop_res(S, qM, *(q + dq)) - r) / 1e-7
        q = q - np.linalg.lstsq(Jm, r, rcond=None)[0]
    return q, float(np.linalg.norm(loop_res(S, qM, *q)))


def com_w(Tm, name):
    return (Tm @ np.r_[LINK[name]["com"], 1.0])[:3]


# ---- 스윕: M = 0 에서 출발해 양쪽으로 (이전 해를 초깃값으로, 조립 분기 유지) -------------------------------
th_deg = np.arange(38.0, 97.8 + 1e-9, args.step_deg)
if th_deg[-1] < 97.8 - 1e-6:
    th_deg = np.append(th_deg, 97.8)
leg_links = ("crank", "shank", "rocker")
out = {}
for S in ("L", "R"):
    s = S.lower()
    q0, res0 = solve(S, 0.0, (0.0, 0.0))
    R_ref = leg_fk(S, 0.0, *q0)[f"{s}_shank"][:3, :3]
    sols = {}
    for direction in (+1, -1):
        q = q0.copy()
        grid = sorted([d for d in th_deg if (d - math.degrees(THETA0)) * direction >= 0], key=lambda d: abs(d - math.degrees(THETA0)))
        for d in grid:
            q, res = solve(S, (math.radians(d) - THETA0) * M_SIGN[S], q)
            sols[d] = (q.copy(), res)
    rows = []
    for d in th_deg:
        qM = (math.radians(d) - THETA0) * M_SIGN[S]
        q, res = sols[d]
        F = leg_fk(S, qM, *q)
        wheel = F[f"{s}_wheel"][:3, 3]
        ms = [LINK[f"{s}_{k}"]["m"] for k in leg_links]
        lc = sum(m * com_w(F[f"{s}_{k}"], f"{s}_{k}") for m, k in zip(ms, leg_links)) / sum(ms)
        Rrel = F[f"{s}_shank"][:3, :3] @ R_ref.T                  # 기준 자세 대비 회전 (시상면 -> y 둘레)
        phi = math.atan2(Rrel[0, 2], Rrel[2, 2])                    # + = +y 둘레 (x 를 -z 쪽으로)
        rows.append(dict(th=math.radians(d), wheel=wheel, legcom=lc, phi=phi, res=res))
    out[S] = rows

th = np.array([r["th"] for r in out["L"]])
wheel = np.array([r["wheel"][[0, 2]] for r in out["L"]])
legc = np.array([r["legcom"][[0, 2]] for r in out["L"]])
phi = np.unwrap([r["phi"] for r in out["L"]])
dphi = np.gradient(phi, th, edge_order=2)
dh = np.array([leg_map.dh_dtheta(t) for t in th])
res_max = max(r["res"] for S in "LR" for r in out[S])
mir_w = np.abs(np.array([r["wheel"][[0, 2]] for r in out["R"]]) - wheel).max()
mir_c = np.abs(np.array([r["legcom"][[0, 2]] for r in out["R"]]) - legc).max()
mir_p = np.abs(np.gradient(np.unwrap([r["phi"] for r in out["R"]]), th, edge_order=2) - dphi).max()
h_err = np.abs(np.hypot(wheel[:, 0], wheel[:, 1]) + leg_map.R_WHEEL - np.array([leg_map.h_of_theta(t) for t in th])).max()
dh_num = np.gradient(np.hypot(wheel[:, 0], wheel[:, 1]), th, edge_order=2)

# ---- LQR: build_lqr() 와 같은 식 (M = 0, 링크 COM 은 base 좌표, iyy = 링크 자기 좌표 yy 성분) ----------------
F0 = {"base_link": np.eye(4)}
for S in "LR":
    q0, _ = solve(S, 0.0, (0.0, 0.0))
    F0.update(leg_fk(S, 0.0, *q0))
nonwheel = ["base_link"] + [f"{s}_{k}" for s in "lr" for k in leg_links]
m_pend = sum(LINK[n]["m"] for n in nonwheel)
c = sum(LINK[n]["m"] * com_w(F0[n], n) for n in nonwheel) / m_pend
I_sim = sum(LINK[n]["I"][1, 1] + LINK[n]["m"] * ((com_w(F0[n], n) - c)[0] ** 2 + (com_w(F0[n], n) - c)[2] ** 2) for n in nonwheel)
m_w = LINK["l_wheel"]["m"] + LINK["r_wheel"]["m"]
I_w = 2 * (WHEEL_IZZ + T["wheel_armature"])
I_use = args.I_pend if args.I_pend > 0 else I_sim
lqr = lqr_vmc.WheelLQR(m_pend, I_use, m_w, I_w, leg_map.R_WHEEL, q=(T["lqr_qx"], T["lqr_qv"], T["lqr_qth"], T["lqr_qthd"]), r=T["lqr_r"])
body_com = LINK["base_link"]["com"]
leg_mass = sum(LINK[f"l_{k}"]["m"] for k in leg_links)
print(f"[LQR] m_pend {m_pend:.4f} kg, I {I_use:.5f} kg·m² (URDF 계산 {I_sim:.5f}), m_w {m_w:.4f}, I_w {I_w:.6f}, K(0.25) {np.round(lqr.gain(0.25), 3).tolist()}")
print(f"[확인] 고리 잔차 최대 {res_max*1e6:.3f} µm | 좌우 거울: 바퀴 {mir_w*1e3:.4f} mm, 다리 COM {mir_c*1e3:.4f} mm, dφ/dθ {mir_p:.5f}"
      f" | |바퀴|+R vs leg_map {h_err*1e3:.3f} mm, dh/dθ 수치 vs leg_map {np.abs(dh_num - dh).max()*1e3:.3f} mm/rad")
for d in (38.0, 45.0, 67.5, 97.8):
    k = int(np.argmin(np.abs(np.degrees(th) - d)))
    print(f"  θ {d:5.1f}: 바퀴 ({wheel[k,0]*1e3:6.1f}, {wheel[k,1]*1e3:7.1f}) mm, 다리 COM ({legc[k,0]*1e3:6.1f}, {legc[k,1]*1e3:6.1f}) mm,"
          f" φ {math.degrees(phi[k]):6.2f}°, dφ/dθ {dphi[k]:.4f}, dh/dθ {dh[k]*1e3:.1f} mm/rad")


def fl(a, nd=6):
    return "[" + ", ".join(f"{v:.{nd}f}" for v in np.asarray(a).ravel()) + "]"


yaml = f"""# 실기 제어기 (gen2_control balance_node) 모델 표 — wheeled_biped_isaaclab/scripts/make_balance_tables.py 가 만든다 (손으로 고치지 말 것)
# 모델: {os.path.basename(args.urdf)} (CAD export 7 + 바퀴 실측, 시뮬 usd_loop 와 같음), 총 {sum(v['m'] for v in LINK.values()):.4f} kg, 바퀴 반지름 R {leg_map.R_WHEEL} m
# LQR: climb_test.build_lqr() 와 같은 식 — CAD 영점 자세 (M = 0, θ 45.002°) 에서 m_pend {m_pend:.4f} kg (바퀴 뺀 전체), I {I_use:.5f} kg·m² (시뮬 Isaac 값, URDF 로 풀면 {I_sim:.5f}),
#      바퀴 두 개 m_w {m_w:.4f} kg, I_w = 2·(WHEEL_IZZ {WHEEL_IZZ} + armature {T['wheel_armature']}) = {I_w:.6f} kg·m²,
#      Q = diag({T['lqr_qx']}, {T['lqr_qv']}, {T['lqr_qth']}, {T['lqr_qthd']}), R = {T['lqr_r']}.  τ (두 바퀴 합) = -K [x_err, v_err, θ, θ̇], l 사이 선형 보간
# 기구학: URDF 4절 링크를 θ {th_deg[0]}…{th_deg[-1]}° 마다 풀어 (고리 잔차 최대 {res_max*1e6:.3f} µm) base_link 좌표 (x 앞, z 위, 원점 = 고관절 축) 로 낸 L 다리.
#      R 다리 (θ 같을 때) 와의 차: 바퀴 {mir_w*1e3:.4f} mm, 다리 COM {mir_c*1e3:.4f} mm, dφ/dθ {mir_p:.5f} -> 같은 표를 각 다리 θ 로 쓴다.
#      |바퀴 중심| + R vs leg_map.h_of_theta: {h_err*1e3:.3f} mm.
# dphi_shank_dtheta: 정강이 (바퀴가 붙은 링크) 의 몸통 대비 y 둘레 회전 / 크랭크 θ. + = 바퀴 굴림 +y 와 같은 방향
#      -> w_wheel_abs = w_wheel_joint (+y) + 자이로 y + dphi(θ)·θ̇   (시뮬: 바퀴 링크 월드 각속도 · 옆축)
balance:
  ros__parameters:
    model:
      lqr_l: {fl(lqr.l_grid, 4)}
      lqr_k: {fl(lqr.K, 6)}
      body_mass: {LINK['base_link']['m']:.4f}
      body_com_xz: [{body_com[0]:.6f}, {body_com[2]:.6f}]
      leg_mass: {leg_mass:.4f}
      wheel_mass: {LINK['l_wheel']['m']:.4f}
      theta_rad: {fl(th, 6)}
      wheel_xz: {fl(wheel, 6)}
      leg_com_xz: {fl(legc, 6)}
      dphi_shank_dtheta: {fl(dphi, 5)}
      dh_dtheta: {fl(dh, 6)}
"""
open(args.out, "w").write(yaml)
print(f"[저장] {args.out}  θ {len(th)} 점, LQR {len(lqr.l_grid)} 점")
