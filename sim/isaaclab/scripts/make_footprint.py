"""로봇 발자국 (z 투영) · LiDAR 높이 단면 · 센서 장착 위치 · 4절 링크 수동 관절 표 — 실기 cmd_mux 자기 몸 마스크·RViz 용.

URDF (sim/model/cad_export7_robot.urdf) 메시를 4절 링크 기구학 (make_balance_tables 와 같은 풀이) 으로 자세마다 놓고
base_link 좌표 (x 앞, y 왼, z 위, 원점 = 두 고관절 축 가운데) 로 낸다. 몸통 기울기 0 (몸통 좌표 그대로).

    python3 scripts/make_footprint.py --assets "<export_(7b)_fixed/assets>" [--out ~/perseverance/sim/model/footprint.yaml]
"""
import argparse
import importlib.util
import io
import contextlib
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--assets", default="/home/parksuho/Desktop/휠 레그드 로봇/export_(7b)_fixed/assets")
ap.add_argument("--out", default=os.path.expanduser("~/perseverance/sim/model/footprint.yaml"))
ap.add_argument("--band_mm", type=float, default=15.0, help="LiDAR 높이 단면 두께 ± [mm]")
args = ap.parse_args()

sys.argv = ["x", "--out", "/tmp/_bt_unused.yaml"]
spec = importlib.util.spec_from_file_location("mbt", os.path.join(HERE, "make_balance_tables.py"))
mbt = importlib.util.module_from_spec(spec)
with contextlib.redirect_stdout(io.StringIO()):
    spec.loader.exec_module(mbt)
leg_map = mbt.leg_map

root = ET.parse(mbt.args.urdf).getroot()
VIS = {}
for l in root.findall("link"):
    for v in l.findall("visual"):
        m = v.find("geometry/mesh")
        if m is None:
            continue
        o = v.find("origin")
        VIS.setdefault(l.get("name"), []).append((os.path.basename(m.get("filename")),
                                                  [float(x) for x in o.get("xyz").split()], [float(x) for x in o.get("rpy").split()]))
_cache = {}


def stl(name):
    if name not in _cache:
        b = open(os.path.join(args.assets, name), "rb").read()
        n = struct.unpack("<I", b[80:84])[0]
        a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        _cache[name] = np.unique(a["v"].reshape(-1, 3).astype(float).round(5), axis=0)
    return _cache[name]


def hull(P):
    """2-D 볼록 껍질 (monotone chain), 반시계."""
    P = sorted(set(map(tuple, np.round(P, 4))))
    if len(P) < 3:
        return np.array(P)
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return np.array(lo[:-1] + up[:-1])


def simplify(H, n=16):
    """꼭짓점이 많으면 넓이를 가장 적게 잃는 점부터 빼서 n 개로 (바깥 껍질 그대로라 마스크가 커지지 않게 — 조금 줄 수 있음)."""
    H = list(map(tuple, H))
    while len(H) > n:
        best, bi = None, None
        for i in range(len(H)):
            a, b, c = np.array(H[i - 1]), np.array(H[i]), np.array(H[(i + 1) % len(H)])
            area = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2
            if best is None or area < best:
                best, bi = area, i
        H.pop(bi)
    return np.array(H)


def pose_points(theta):
    """θ (크랭크, 양 다리 같게) 자세의 모든 메시 꼭짓점 (base_link 좌표) 과 관절값."""
    F, joints = {"base_link": np.eye(4)}, {}
    for S in "LR":
        qM = (theta - mbt.THETA0) * mbt.M_SIGN[S]
        q0 = (0.0, 0.0)
        # M = 0 에서 θ 쪽으로 이어 풀기 (조립 분기 유지)
        steps = np.linspace(0, qM, 40)
        q = np.array(q0)
        for x in steps:
            q, res = mbt.solve(S, x, q)
        F.update(mbt.leg_fk(S, qM, *q))
        joints[S] = dict(M=qM, I=float(q[0]), K=float(q[1]), res=res)
    pts = []
    for link, vis in VIS.items():
        if link not in F:
            continue
        for mesh, xyz, rpy in vis:
            Tv = F[link] @ mbt.tf(mbt.rpy_mat(*rpy), xyz)
            V = stl(mesh)
            pts.append((Tv[:3, :3] @ V.T).T + Tv[:3, 3])
    return np.vstack(pts), joints


laser = next(j for j in root.findall("joint") if j.find("child").get("link") == "laser")
z_laser = float(laser.find("origin").get("xyz").split()[2])
poses = {"stand_idle": 0.1825, "sit": 0.1255, "high": 0.2425}
out, txt = {}, []
for name, hj in poses.items():
    th = float(leg_map.theta_of_h(hj + leg_map.R_WHEEL))
    P, joints = pose_points(th)
    H = simplify(hull(P[:, :2]), 16)
    band = P[np.abs(P[:, 2] - z_laser) <= args.band_mm / 1000.0]
    Hs = simplify(hull(band[:, :2]), 12) if len(band) >= 3 else np.zeros((0, 2))
    out[name] = dict(h=hj, theta_deg=math.degrees(th), M_deg=math.degrees(th - mbt.THETA0), box=[P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max()],
                     z=[P[:, 2].min(), P[:, 2].max()], hull=H, slice=Hs, slice_box=[band[:, 0].min(), band[:, 0].max(), band[:, 1].min(), band[:, 1].max()] if len(band) else None,
                     joints=joints)
    print(f"{name:10s} h {hj:.4f} θ {math.degrees(th):5.1f}° | 박스 x {P[:,0].min():+.3f}…{P[:,0].max():+.3f} y {P[:,1].min():+.3f}…{P[:,1].max():+.3f} "
          f"z {P[:,2].min():+.3f}…{P[:,2].max():+.3f} m | 꼭짓점 {len(H)} | LiDAR 높이 단면 점 {len(band)}"
          + (f" 박스 x {band[:,0].min():+.3f}…{band[:,0].max():+.3f} y {band[:,1].min():+.3f}…{band[:,1].max():+.3f}" if len(band) else ""))

# 4절 링크 수동 관절 표 (RViz joint_states): θ 38…97.8°
tab = []
for d in np.arange(38.0, 97.81, 1.0):
    th = math.radians(d)
    row = [d]
    for S in "LR":
        qM = (th - mbt.THETA0) * mbt.M_SIGN[S]
        q = np.array((0.0, 0.0))
        for x in np.linspace(0, qM, 30):
            q, _ = mbt.solve(S, x, q)
        row += [qM, q[0], q[1]]
    tab.append(row)
tab = np.array(tab)

fl = lambda a, nd=4: "[" + ", ".join(f"{v:.{nd}f}" for v in np.asarray(a).ravel()) + "]"  # noqa: E731
sens = {}
for j in root.findall("joint"):
    c = j.find("child").get("link")
    if j.get("type") == "fixed" and j.find("parent").get("link") == "base_link" and c in ("imu_link", "gps_link", "laser", "camera_link"):
        o = j.find("origin")
        sens[c] = ([float(x) for x in o.get("xyz").split()], [math.degrees(float(x)) for x in o.get("rpy").split()])
y = ["# 로봇 발자국 · 센서 위치 · 4절 링크 관절 표 — wheeled_biped_isaaclab/scripts/make_footprint.py 가 만든다 (손으로 고치지 말 것)",
     f"# 모델 {os.path.basename(mbt.args.urdf)} 메시 전부 (나사 포함), 몸통 기울기 0. 좌표 base_link: x 앞, y 왼, z 위 [m], 원점 = 두 고관절 축 (L/R_joint_M) 가운데.",
     f"# hull: z 투영 볼록 다각형 (반시계, 16 점 이하). slice: LiDAR 광학 중심 높이 z {z_laser:.4f} ± {args.band_mm:.0f} mm 안의 메시 점 볼록 다각형.",
     "# 바퀴 접지 = 바퀴 중심 z - 0.070.",
     "footprint:"]
for name, o in out.items():
    y += [f"  {name}:   # 다리 관절값 h {o['h']} m (고관절~바퀴 중심 {o['h']+0.07:.4f}), 크랭크 θ {o['theta_deg']:.1f}°, M {o['M_deg']:+.1f}°",
          f"    box_xy: {fl(o['box'])}   # [x_min, x_max, y_min, y_max]",
          f"    z_range: {fl(o['z'])}",
          f"    hull_xy: {fl(o['hull'])}   # x0, y0, x1, y1, ...",
          f"    slice_box_xy: {fl(o['slice_box']) if o['slice_box'] else '[]'}",
          f"    slice_hull_xy: {fl(o['slice'])}"]
y += ["sensors:   # base_link 기준 xyz [m], rpy [deg] (CAD 메이트 커넥터 frame_*)"]
for c, (xyz, rpy) in sens.items():
    y.append(f"  {c}: {{xyz: {fl(xyz)}, rpy_deg: {fl(rpy, 2)}}}")
y += ["four_bar:   # RViz joint_states: θ (크랭크) -> 관절값 [rad]. M = (θ - 45.002°)·부호 (L +1, R -1). I, K 는 수동 관절 (URDF L/R_joint_I, L/R_joint_K)",
      f"  theta_deg: {fl(tab[:, 0], 1)}",
      f"  L_M: {fl(tab[:, 1])}", f"  L_I: {fl(tab[:, 2])}", f"  L_K: {fl(tab[:, 3])}",
      f"  R_M: {fl(tab[:, 4])}", f"  R_I: {fl(tab[:, 5])}", f"  R_K: {fl(tab[:, 6])}"]
open(args.out, "w").write("\n".join(y) + "\n")
print("[저장]", args.out)
