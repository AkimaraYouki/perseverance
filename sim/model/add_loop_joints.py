"""USD 에 4절링크 폐루프 관절을 단다 — Isaac Sim 5.1 rig_closed_loop_structures 방식.

make_loop_urdf.py 가 만든 loop_closure.json 을 읽어, 몸체(base_link)와 로커 사이에
회전관절을 만들고 "Exclude From Articulation" 을 켠다. 관절 트리는 그대로 두고
이 관절만 솔버가 최대좌표 구속으로 따로 푼다 (구동·한계 없음).

관절 프레임: 회전축을 관절 X 축으로 둔다. CAD 영점 자세에서 두 프레임이 월드에서
완전히 겹치도록 로커 쪽 회전을 몸체 쪽에서 유도한다 (축 둘레 각도만 자유).

    isaaclab.sh -p add_loop_joints.py <robot.usd> <loop_closure.json>
"""
import json
import sys

from isaaclab.app import AppLauncher

app = AppLauncher(headless=True).app

import numpy as np  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdPhysics  # noqa: E402

usd_path, js_path = sys.argv[1], sys.argv[2]
loops = json.load(open(js_path))
stage = Usd.Stage.Open(usd_path)
root = stage.GetDefaultPrim()
rp = root.GetPath()
print("root", rp, flush=True)


def rot_x_to(axis):
    """관절 X 축을 axis 로 보내는 회전행렬 (축 둘레 자유도는 임의로 고정)."""
    x = np.asarray(axis, float); x /= np.linalg.norm(x)
    tmp = np.array([0, 0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0, 0])
    y = np.cross(tmp, x); y /= np.linalg.norm(y)
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def quat(R):
    q = Gf.Matrix3d(*R.T.flatten().tolist()).ExtractRotation().GetQuat()  # Gf 는 행벡터 규약 -> 전치
    return Gf.Quatf(q.GetReal(), Gf.Vec3f(*q.GetImaginary()))


jroot = rp.AppendChild("joints")
for side, d in loops.items():
    R_bw = np.array(d["R_body_world"]); R_rw = np.array(d["R_rocker_world"])
    R0 = rot_x_to(d["axis_body"])                 # 몸체 로컬에서의 관절 프레임
    R1 = R_rw.T @ R_bw @ R0                       # 로커 로컬에서의 같은 프레임 (CAD 영점)
    # 검산: 로커 쪽 관절 X 축이 로커 로컬 축과 평행해야 한다
    par = abs(float(np.dot(R1[:, 0], np.asarray(d["axis_rocker"]) / np.linalg.norm(d["axis_rocker"]))))
    path = jroot.AppendChild(f"loop_{side}")
    # 구면 관절로 닫는다. 평면 4절링크의 폐루프에 필요한 구속은 평면 내 위치 2 개뿐인데,
    # 회전관절은 5 개(위치 3 + 회전 2)를 걸어 과잉구속이 된다. CAD 축이 미세하게만 어긋나도
    # 중복 구속끼리 싸워 모터가 90 Nm 를 넣어도 목표에서 12~15 deg 뒤처졌다 (2026-09-25 loop_check).
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    j = UsdPhysics.SphericalJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([rp.AppendChild(d["body"])])
    j.CreateBody1Rel().SetTargets([rp.AppendChild(d["rocker"])])
    j.CreateAxisAttr("X")
    j.CreateConeAngle0LimitAttr(-1.0)   # 음수 = 제한 없음
    j.CreateConeAngle1LimitAttr(-1.0)
    j.CreateLocalPos0Attr(Gf.Vec3f(*d["p_body"]))
    j.CreateLocalRot0Attr(quat(R0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*d["p_rocker"]))
    j.CreateLocalRot1Attr(quat(R1))
    j.CreateExcludeFromArticulationAttr(True)
    print(f"loop_{side} (spherical): {d['body']} <-> {d['rocker']}  exclude=True  축 평행도 {par:.6f}", flush=True)
stage.GetRootLayer().Save()
print("저장", usd_path, flush=True)
app.close()
