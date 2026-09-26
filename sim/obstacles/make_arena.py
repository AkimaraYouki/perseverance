"""Onshape 경기장 (Part Studio 전체를 STL zip 으로 내보낸 것) -> 시뮬 경기장 한 덩어리 arena.stl.

make_course.py 와 달리 파트를 다시 늘어놓지 않는다: Part Studio 안의 배치를 그대로 쓴다.
  * Onshape 좌표: Z 위, 로봇이 출발할 때 보는 방향 = --heading (기본 +Y)
  * --start X Y  : 출발점 (CAD mm, 두 바퀴 축 가운데 바로 아래 바닥). 이 점이 시뮬 원점이 된다
  * 바닥판을 그렸다면 윗면이 Z = 0 이어야 한다 (시뮬 바닥 평면과 겹친다). 안 그려도 된다
  * 단위 mm (다른 단위면 --unit)

    python3 make_arena.py "~/Downloads/Part Studio 3.zip" --start 0 -3000 --heading +Y
    pv jump --joystick --obstacle cad --cad_file ~/perseverance/sim/obstacles/arena.stl
"""
import argparse
import glob
import io
import math
import os
import struct
import zipfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def read_stl(data: bytes):
    if data[:5] == b"solid" and b"facet" in data[:1000]:
        v = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors="ignore").splitlines() if ln.strip().startswith("vertex")]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack("<I", data[80:84])[0]
    return np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"].reshape(n, 3, 3).astype(float)


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"arena from make_arena.py".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            nrm = np.cross(t[1] - t[0], t[2] - t[0]); nrm = nrm / (np.linalg.norm(nrm) + 1e-12)
            f.write(struct.pack("<3f", *nrm) + b"".join(struct.pack("<3f", *v) for v in t) + b"\0\0")


ap = argparse.ArgumentParser()
ap.add_argument("src", help="Onshape STL zip, 또는 STL 들이 든 폴더")
ap.add_argument("--start", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"), help="출발점 CAD 좌표 [mm]")
ap.add_argument("--heading", default="+Y", help="출발할 때 로봇이 보는 CAD 방향: +X -X +Y -Y 또는 각도 [deg, +X 기준 반시계]")
ap.add_argument("--unit", type=float, default=1.0, help="CAD 단위 -> mm (mm 면 1)")
ap.add_argument("--out", default=os.path.join(HERE, "arena.stl"))
a = ap.parse_args()

src = os.path.expanduser(a.src)
parts = []
if src.endswith(".zip"):
    with zipfile.ZipFile(src) as z:
        parts = [(n, read_stl(z.read(n))) for n in z.namelist() if n.lower().endswith(".stl")]
else:
    parts = [(os.path.basename(p), read_stl(open(p, "rb").read())) for p in sorted(glob.glob(os.path.join(src, "*.stl")))]
if not parts:
    raise SystemExit("STL 이 없다")

hd = {"+X": 0.0, "+Y": 90.0, "-X": 180.0, "-Y": 270.0}.get(a.heading.upper(), None)
psi = math.radians(float(a.heading) if hd is None else hd)
c, s = math.cos(psi), math.sin(psi)
out = []
for name, t in parts:
    t = t * a.unit
    x, y = t[..., 0] - a.start[0], t[..., 1] - a.start[1]
    t = np.stack([c * x + s * y, -s * x + c * y, t[..., 2]], axis=-1)      # 출발 방향 -> 시뮬 +X
    lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
    print(f"{name}: X {lo[0]:.0f}~{hi[0]:.0f}  Y {lo[1]:.0f}~{hi[1]:.0f}  Z {lo[2]:.0f}~{hi[2]:.0f} mm  ({len(t)} 삼각형)")
    out.append(t)
T = np.concatenate(out)
under = T[..., 2].min()
if under < -1.0:
    print(f"주의: Z {under:.0f} mm 까지 바닥 아래로 내려간 파트가 있다 (바닥판이면 괜찮다. 장애물이면 밑면을 Z = 0 에)")
write_stl(a.out, T)
print(f"-> {a.out}  파트 {len(parts)} 개, 삼각형 {len(T)} 개. 출발점 = 원점, 출발 방향 = +X")
