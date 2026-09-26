"""Onshape 에서 뽑은 장애물 STL 들을 시뮬 규약(+X 전진, 원점 = 출발점 바닥, Z = 0 바닥)으로 돌려 한 코스로 잇는다.

Onshape 파트는 원점 중심, 달리는 방향이 +Y 라서: (x, y, z) -> (y, -x, z) 로 돌린 뒤, 밑면을 Z = 0 에 놓고,
COURSE 순서대로 앞 장애물 끝에서 gap 만큼 띄워 X 로 늘어놓는다. 좌우 가운데(CAD X = 0) 는 바퀴 가운데선(시뮬 Y = 0).

    python3 make_course.py            # -> course.stl (mm)
    pv jump --joystick --obstacle cad --cad_file ~/perseverance/sim/obstacles/course.stl
"""
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
START = 1000.0                                          # 출발점에서 첫 장애물까지 [mm]
COURSE = [("obs1_ridges.stl", 0.0), ("obs2_pyramid.stl", 1500.0)]   # (파일, 앞 장애물과의 간격 mm)


def load(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    return np.array([struct.unpack("<12f", b[84 + i * 50:84 + i * 50 + 48]) for i in range(n)])[:, 3:].reshape(n, 3, 3)


def save(path, tris):
    with open(path, "wb") as f:
        f.write(b"course from make_course.py".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            nrm = np.cross(t[1] - t[0], t[2] - t[0])
            nrm = nrm / (np.linalg.norm(nrm) + 1e-12)
            f.write(struct.pack("<3f", *nrm) + b"".join(struct.pack("<3f", *v) for v in t) + b"\0\0")


out, x = [], START
for name, gap in COURSE:
    t = load(os.path.join(HERE, name))
    t = np.stack([t[..., 1], -t[..., 0], t[..., 2]], axis=-1)          # Onshape +Y 전진 -> 시뮬 +X
    t[..., 2] -= t[..., 2].min()                                        # 밑면 Z = 0
    x += gap
    t[..., 0] += x - t[..., 0].min()
    lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
    print(f"{name}: X {lo[0]:.0f}~{hi[0]:.0f}  Y {lo[1]:.0f}~{hi[1]:.0f}  높이 {hi[2]:.0f} mm")
    x = hi[0]
    out.append(t)
save(os.path.join(HERE, "course.stl"), np.concatenate(out))
print(f"-> {os.path.join(HERE, 'course.stl')}  (코스 끝 X {x:.0f} mm)")
