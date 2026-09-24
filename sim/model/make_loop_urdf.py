#!/usr/bin/env python3
"""CAD URDF(4절링크 폐루프 포함) -> Isaac Sim 용 robot_loop.urdf + loop_closure.json

Isaac Sim 5.1 폐루프 튜토리얼(rig_closed_loop_structures) 방식:
  URDF 는 트리만 표현하므로 트리로 가져오고, 폐루프 관절은 USD 에서
  "Exclude From Articulation" 회전관절로 따로 단다 (add_loop_joints.py).

이 스크립트가 하는 일:
  1. onshape-to-robot 의 closing_*_P_1 (로커) / closing_*_P_2 (몸체) 프레임과 _z 프레임에서
     폐루프 점 P 와 회전축을 몸체·로커 로컬 좌표로 뽑아 loop_closure.json 에 쓴다.
     CAD 영점 자세에서 두 P 가 월드에서 일치하는지(조립 잔차)도 검사한다.
  2. closing_* 링크/관절을 지운다 (질량 없는 링크는 PhysX 관절 트리에서 문제를 낸다).
  3. 충돌체를 단순화한다. CAD 는 보이는 메시 전부(몸체 93 개)가 충돌체라 관절부에서 링크끼리
     겹친다. 바퀴 = 원통(R 60, 폭 30), 몸체 = 상자, 다리 링크 = 없음. 모양(visual)은 CAD 그대로.

    python3 make_loop_urdf.py <export_(N)_fixed/robot.urdf> [--out robot_loop.urdf]
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def origin(j):
    o = j.find("origin")
    T = np.eye(4)
    T[:3, :3] = rpy_mat(*map(float, o.get("rpy", "0 0 0").split()))
    T[:3, 3] = list(map(float, o.get("xyz", "0 0 0").split()))
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--out", default=str(Path(__file__).parent / "robot_loop.urdf"))
    a = ap.parse_args()
    tree = ET.parse(a.src)
    root = tree.getroot()
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}

    def world(link):          # CAD 영점(모든 관절 0) 자세에서 루트 기준 변환
        T = np.eye(4)
        chain = []
        while link in by_child:
            chain.append(by_child[link]); link = by_child[link].find("parent").get("link")
        for j in reversed(chain):
            T = T @ origin(j)
        return T

    loops = {}
    for side in ("L", "R"):
        p1, p2 = f"closing_{side}_joint_P_1", f"closing_{side}_joint_P_2"
        rocker = by_child[p1].find("parent").get("link")
        body = by_child[p2].find("parent").get("link")
        W1, W2 = world(p1), world(p2)
        W1z, W2z = world(p1 + "_z"), world(p2 + "_z")
        res = float(np.linalg.norm(W1[:3, 3] - W2[:3, 3]))
        ax_w = W2z[:3, 3] - W2[:3, 3]; ax_w /= np.linalg.norm(ax_w)
        ax_w1 = W1z[:3, 3] - W1[:3, 3]; ax_w1 /= np.linalg.norm(ax_w1)
        Wr, Wb = world(rocker), world(body)
        # 로컬 좌표 (점, 축)
        pb = (np.linalg.inv(Wb) @ np.r_[W2[:3, 3], 1])[:3]
        pr = (np.linalg.inv(Wr) @ np.r_[W1[:3, 3], 1])[:3]
        ab = Wb[:3, :3].T @ ax_w
        ar = Wr[:3, :3].T @ ax_w
        loops[side] = dict(body=body, rocker=rocker, p_body=pb.tolist(), p_rocker=pr.tolist(),
                           axis_body=ab.tolist(), axis_rocker=ar.tolist(),
                           # 로커 기준 몸체 회전 (CAD 영점) — USD 관절 프레임 맞출 때 쓴다
                           R_body_world=Wb[:3, :3].tolist(), R_rocker_world=Wr[:3, :3].tolist(),
                           assembly_residual_m=res,
                           axis_parallel=float(abs(np.dot(ax_w, ax_w1))))
        print(f"{side}: 몸체 {body} P {np.round(pb*1000,2)} mm 축 {np.round(ab,3)} | 로커 {rocker} P {np.round(pr*1000,2)} mm 축 {np.round(ar,3)}"
              f" | 조립잔차 {res*1000:.3f} mm, 두 축 평행도 {abs(np.dot(ax_w, ax_w1)):.6f}")

    # closing_* 제거
    for l in list(root.findall("link")):
        if l.get("name").startswith("closing_"):
            root.remove(l)
    for j in list(root.findall("joint")):
        if j.find("child").get("link").startswith("closing_"):
            root.remove(j)

    # 충돌체 단순화
    for l in root.findall("link"):
        for c in l.findall("collision"):
            l.remove(c)
        n = l.get("name")
        i = l.find("inertial")
        com = i.find("origin").get("xyz") if i is not None else "0 0 0"
        if n.endswith("_wheel"):       # 회전축 = 로컬 z (W 관절 축 0 0 1) — 원통 기본축과 같다
            c = ET.SubElement(l, "collision"); ET.SubElement(c, "origin", {"xyz": com, "rpy": "0 0 0"})
            g = ET.SubElement(c, "geometry"); ET.SubElement(g, "cylinder", {"radius": "0.06", "length": "0.03"})
        elif n == "base_link":
            c = ET.SubElement(l, "collision"); ET.SubElement(c, "origin", {"xyz": com, "rpy": "0 0 0"})
            g = ET.SubElement(c, "geometry"); ET.SubElement(g, "box", {"size": "0.15 0.19 0.13"})
    root.set("name", "wheeled_biped_loop")
    ET.indent(tree, space="  ")
    tree.write(a.out, encoding="utf-8", xml_declaration=True)
    js = Path(a.out).with_name("loop_closure.json")
    js.write_text(json.dumps(loops, indent=1))
    print(f"\n저장: {a.out}\n      {js}")


main()
