"""RL/제어 설계용 단순화 URDF 생성.

4절링크를 직선 관절 하나로 대체한다. Isaac 은 닫힌 고리를 잘 못 다루므로
학습 모델에서는 고리를 아예 없앤다. 정책이 내는 다리 길이 h 는
leg_map.py 로 모터 각도 theta 로 바꿔 실기/MuJoCo 에 넣는다.

    base_link ─(prismatic, 아래로)─ 캐리어 ─(continuous)─ 바퀴
              └(fixed) imu_link, gps_link, laser, camera_link  (CAD 프레임 그대로)

입력은 fix_urdf.py 를 거친 onshape-to-robot export 다 (export (5) 부터, X 전진 / Z 위,
L = +y 쪽, 루트 = base_link). 원점은 두 고관절(모터축)의 중점으로 옮긴다.

질량 배분은 MuJoCo 닫힌고리 모델에서 실측한 참여도를 쓴다.
크랭크 0.54, 로커 0.35, 생크 0.96, 바퀴 1.00 만큼만 다리를 따라 움직인다.
나머지는 고관절 위치에 몸체 질량으로 붙인다.
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import leg_map

HERE = Path(__file__).parent
DEFAULT_SRC = HERE.parent / "export_(5)_fixed" / "robot.urdf"
OUT = HERE / "robot_simple.urdf"
R = leg_map.R_WHEEL

# MuJoCo 실측 참여도
PARTICIPATION = {"crank": 0.5385, "rocker": 0.3483, "shank": 0.9592, "wheel": 1.0}

# 바퀴 회전자 반사 관성. 단순화 URDF 에는 넣지 않는다 —
# robot_cfg.py 가 armature 로 관절에 직접 넣기 때문에 양쪽에 넣으면 이중 계상이다.
# fix_urdf.py 가 CAD 바퀴 izz 에 더해 둔 것을 여기서 뺀다.
WHEEL_ROTOR_REFLECTED = 157.33e-7 * 10.0**2   # 1.5733e-3 kg m^2

SENSOR_FRAMES = ("imu_link", "gps_link", "laser", "camera_link")


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


class Src:
    def __init__(self, path):
        r = ET.parse(path).getroot()
        self.L = {l.get("name"): l for l in r.findall("link")}
        self.J = {j.get("name"): j for j in r.findall("joint")}
        self.by_child = {j.find("child").get("link"): j for j in r.findall("joint")}

    def inertial(self, name):
        i = self.L[name].find("inertial")
        o = i.find("origin")
        assert all(abs(float(v)) < 1e-9 for v in o.get("rpy", "0 0 0").split()), f"{name} 관성 프레임 회전됨"
        e = i.find("inertia")
        g = lambda k: float(e.get(k))
        I = np.array([[g("ixx"), g("ixy"), g("ixz")],
                      [g("ixy"), g("iyy"), g("iyz")],
                      [g("ixz"), g("iyz"), g("izz")]])
        return float(i.find("mass").get("value")), np.array(list(map(float, o.get("xyz").split()))), I

    def pose(self, link):
        """관절 0 자세에서 base_link 기준 link 의 (R, p)."""
        T = np.eye(4)
        chain = []
        while link in self.by_child:
            j = self.by_child[link]
            chain.append(j)
            link = j.find("parent").get("link")
        for j in reversed(chain):
            o = j.find("origin")
            A = np.eye(4)
            A[:3, :3] = rpy_mat(*map(float, o.get("rpy").split()))
            A[:3, 3] = list(map(float, o.get("xyz").split()))
            T = T @ A
        return T[:3, :3], T[:3, 3]


def tensor_attrs(I):
    return {"ixx": I[0, 0], "ixy": I[0, 1], "ixz": I[0, 2], "iyy": I[1, 1], "iyz": I[1, 2], "izz": I[2, 2]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(DEFAULT_SRC))
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args()
    S = Src(a.src)

    # --- 고관절 (다리 모터축) 위치 → 새 원점
    hipL = S.pose(S.J["L_joint_M"].find("child").get("link"))[1]
    hipR = S.pose(S.J["R_joint_M"].find("child").get("link"))[1]
    hip_c = 0.5 * (hipL + hipR)
    hip_y = 0.5 * abs(hipL[1] - hipR[1])
    assert hipL[1] > hipR[1], "L 다리가 +y 쪽이 아니다 — export 좌우 규약 확인"

    # --- 질량
    m_body, com_body, I_body = S.inertial("base_link")
    m_crank = S.inertial("r_crank")[0]
    m_rocker = S.inertial("r_rocker")[0]
    m_shank, _, I_shank_local = S.inertial("r_shank")
    m_wheel, _, I_wheel_cad = S.inertial("r_wheel")

    eff = (m_crank * PARTICIPATION["crank"] + m_rocker * PARTICIPATION["rocker"]
           + m_shank * PARTICIPATION["shank"])
    m_carrier = eff
    left_over = m_crank + m_rocker + m_shank - eff          # 다리 하나당 몸체에 붙는 몫
    m_base = m_body + 2 * left_over

    # 몸체 COM: CAD 몸체 COM + 남은 다리 질량을 각 고관절에
    com_base = (m_body * com_body + left_over * (hipL + hipR)) / m_base
    # 몸체 관성: CAD 몸체 관성(자기 COM 기준) + 고관절 점질량의 평행축 항
    def pa(m, d):
        return m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    I_base = (I_body + pa(m_body, com_body - com_base)
              + pa(left_over, hipL - com_base) + pa(left_over, hipR - com_base))

    # 캐리어 관성: 생크 텐서를 몸체 좌표로 돌리고 질량비로 축소 (지배적 부재)
    Rs = S.pose("r_shank")[0]
    I_carrier = (Rs @ I_shank_local @ Rs.T) * (m_carrier / m_shank)

    # 캐리어 COM 보정: 실제 다리 부재(생크 등)는 고관절-바퀴 선보다 뒤에 있다.
    # 캐리어 질량을 선 위에 두면 전체 COM 이 CAD 와 반대쪽(앞)으로 가서 정적
    # 기울기 부호가 실기와 뒤집힌다. CAD 영점 자세의 전체 COM x 에 맞춘다.
    tot_m, tot_c = 0.0, np.zeros(3)
    for n, l in S.L.items():
        if l.find("inertial") is None:
            continue
        m_, c_, _ = S.inertial(n)
        if m_ <= 0:
            continue
        Rn, pn = S.pose(n)
        tot_m += m_
        tot_c += m_ * (pn + Rn @ c_)
    com_cad = tot_c / tot_m
    # sum m x 일치: m_base*cx_base + 2*m_carrier*dx + 2*m_wheel*x_wheel(=고관절 x) = M*cx_cad
    x_w = hip_c[0]
    dx_car = (tot_m * com_cad[0] - m_base * com_base[0] - 2 * m_wheel * x_w) / (2 * m_carrier) - x_w
    self_check = (m_base * com_base[0] + 2 * m_carrier * (x_w + dx_car) + 2 * m_wheel * x_w) / (m_base + 2 * m_carrier + 2 * m_wheel)

    # 바퀴: 스핀축 = 몸체 y. CAD 바퀴 텐서는 로컬 z 가 스핀축.
    I_spin_cad = I_wheel_cad[2, 2] - WHEEL_ROTOR_REFLECTED
    I_trans = 0.5 * (I_wheel_cad[0, 0] + I_wheel_cad[1, 1])
    I_wheel = np.diag([I_trans, I_spin_cad, I_trans])

    # --- URDF 작성 (원점 = 고관절 중점)
    robot = ET.Element("robot", {"name": "wheeled_biped_simple"})

    def add_link(name, mass, I, com=(0, 0, 0), shape=None):
        l = ET.SubElement(robot, "link", {"name": name})
        if mass is not None:
            i = ET.SubElement(l, "inertial")
            ET.SubElement(i, "origin", {"xyz": " ".join(f"{v:.6g}" for v in com), "rpy": "0 0 0"})
            ET.SubElement(i, "mass", {"value": f"{mass:.6f}"})
            ET.SubElement(i, "inertia", {k: f"{v:.6e}" for k, v in tensor_attrs(I).items()})
        if shape is not None:
            kind, size, oxyz, orpy = shape
            for tag in ("visual", "collision"):
                e = ET.SubElement(l, tag)
                ET.SubElement(e, "origin", {"xyz": oxyz, "rpy": orpy})
                g = ET.SubElement(e, "geometry")
                if kind == "box":
                    ET.SubElement(g, "box", {"size": size})
                else:
                    r_, ln = size
                    ET.SubElement(g, "cylinder", {"radius": f"{r_:g}", "length": f"{ln:g}"})
        return l

    c = com_base - hip_c
    # 몸체 충돌체: CAD 외형을 감싸는 상자를 COM 높이에 둔다 (접지는 바퀴로만 한다)
    add_link("base_link", m_base, I_base, com=c,
             shape=("box", "0.15 0.19 0.13", f"0 0 {c[2]:.4f}", "0 0 0"))

    for side, sy in (("l", +1), ("r", -1)):
        leg_mid = (leg_map.H_MIN + leg_map.H_MAX) / 2 - R
        add_link(f"{side}_carrier", m_carrier, I_carrier, com=(dx_car, 0.0, 0.0),
                 shape=("cyl", (0.018, 0.10), f"0 0 {leg_mid/2:.4f}", "0 0 0"))
        add_link(f"{side}_wheel", m_wheel, I_wheel,
                 shape=("cyl", (R, 0.030), "0 0 0", "1.5708 0 0"))

        j = ET.SubElement(robot, "joint", {"name": f"{side}_leg", "type": "prismatic"})
        ET.SubElement(j, "parent", {"link": "base_link"})
        ET.SubElement(j, "child", {"link": f"{side}_carrier"})
        ET.SubElement(j, "origin", {"xyz": f"0 {sy*hip_y:.6g} 0", "rpy": "0 0 0"})
        ET.SubElement(j, "axis", {"xyz": "0 0 -1"})
        ET.SubElement(j, "limit", {
            "lower": f"{leg_map.H_MIN - R:.6f}", "upper": f"{leg_map.H_MAX - R:.6f}",
            "effort": f"{9.0 / leg_map.dh_dtheta(leg_map.THETA_MIN):.1f}", "velocity": "1.0"})

        jw = ET.SubElement(robot, "joint", {"name": f"{side}_wheel_joint", "type": "continuous"})
        ET.SubElement(jw, "parent", {"link": f"{side}_carrier"})
        ET.SubElement(jw, "child", {"link": f"{side}_wheel"})
        ET.SubElement(jw, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(jw, "axis", {"xyz": "0 1 0"})
        ET.SubElement(jw, "limit", {"effort": "7.0", "velocity": "18.8"})

    # 센서 프레임: CAD 위치/자세를 새 원점 기준으로 옮긴다
    frames = []
    for name in SENSOR_FRAMES:
        if name not in S.by_child:
            continue
        o = S.by_child[name].find("origin")
        xyz = np.array(list(map(float, o.get("xyz").split()))) - hip_c
        ET.SubElement(robot, "link", {"name": name})
        jf = ET.SubElement(robot, "joint", {"name": f"{name}_joint", "type": "fixed"})
        ET.SubElement(jf, "parent", {"link": "base_link"})
        ET.SubElement(jf, "child", {"link": name})
        ET.SubElement(jf, "origin", {"xyz": " ".join(f"{v:.6g}" for v in xyz), "rpy": o.get("rpy")})
        frames.append((name, xyz, o.get("rpy")))

    t = ET.ElementTree(robot)
    ET.indent(t, space="  ")
    t.write(a.out, encoding="utf-8", xml_declaration=True)

    tot = m_base + 2 * (m_carrier + m_wheel)
    print(f"{Path(a.out).name} 생성  (입력 {a.src})")
    print(f"  base_link {m_base*1000:8.1f} g   COM (고관절 중점 기준) "
          f"x {c[0]*1000:+.1f}  y {c[1]*1000:+.1f}  z {c[2]*1000:+.1f} mm")
    print(f"  carrier   {m_carrier*1000:8.1f} g  x2   COM x {dx_car*1000:+.1f} mm (바퀴축 기준, CAD 전체 COM 맞춤)")
    print(f"  전체 COM x (고관절 중점 기준): CAD {(com_cad[0]-hip_c[0])*1000:+.1f} mm, 단순화 {(self_check-hip_c[0])*1000:+.1f} mm  (CAD 영점 자세 기준)")
    print(f"  wheel     {m_wheel*1000:8.1f} g  x2   스핀 iyy {I_spin_cad:.4e} (회전자 {WHEEL_ROTOR_REFLECTED:.4e} 는 armature 로 별도)")
    print(f"  합계      {tot*1000:8.1f} g")
    print(f"  고관절 간격 {2*hip_y*1000:.1f} mm  (L +y {hip_y*1000:.1f}, R -y)")
    print(f"  좌표규약  전진 +x, 가로 +y(왼쪽), 바퀴축 y, 불안정축 pitch(y)")
    for n, xyz, rpy in frames:
        print(f"  프레임 {n:12s} xyz {np.round(xyz*1000,1)} mm  rpy {rpy}")
    print(f"\n  다리 관절 범위 {(leg_map.H_MIN-R)*1000:.1f} ~ {(leg_map.H_MAX-R)*1000:.1f} mm (모터축~바퀴중심)")
    print(f"  = 다리 높이 h  {leg_map.H_MIN*1000:.1f} ~ {leg_map.H_MAX*1000:.1f} mm")


main()
