"""RL 학습용 단순화 URDF 생성.

4절링크를 직선 관절 하나로 대체한다. Isaac 은 닫힌 고리를 잘 못 다루므로
학습 모델에서는 고리를 아예 없앤다. 정책이 내는 다리 길이 h 는
leg_map.py 로 모터 각도 theta 로 바꿔 실기/MuJoCo 에 넣는다.

    몸체 ─(prismatic, 아래로)─ 캐리어 ─(continuous)─ 바퀴

질량 배분은 MuJoCo 닫힌고리 모델에서 실측한 참여도를 쓴다.
크랭크 0.54, 로커 0.35, 생크 0.96, 바퀴 1.00 만큼만 다리를 따라 움직인다.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import leg_map

SRC = Path(__file__).parent.parent / "export_(4)_fixed" / "robot.urdf"
OUT = Path(__file__).parent / "robot_simple.urdf"

HIP_Y = 0.081          # 몸체 중심에서 좌우 고관절까지 [m] (가로 = y)
R = leg_map.R_WHEEL

# ---------------------------------------------------------------------------
# 좌표 규약 (2026-09-24 수정)
#
# CAD 내보내기 프레임은 바퀴가 x=+-0.081 에 있고 회전축도 x 였다. 즉 이 로봇은
# y 방향으로 굴러간다. 그런데 IsaacLab 의 이동 과제는 전부 +x 를 전진으로 보고
# 커맨드도 lin_vel_x 다. 그대로 두면 정책이 갈 수 없는 방향으로 가라는 지령을
# 받게 되고, 실기에서도 부호 실수가 나기 쉽다.
#
# 그래서 모델 전체를 z 축으로 +90 deg 돌려 전진을 +x 로 맞춘다.
#   고관절  x=+-0.081  ->  y=+-0.081
#   바퀴축  x          ->  y
#   불안정축(넘어지는 축) = pitch = y
# 관성 텐서도 같이 돌려야 한다 (아래 rot_z90).
# ---------------------------------------------------------------------------


def rot_z90(I):
    """z 축 +90 deg 회전 후의 관성 텐서. I' = R I R^T, R = Rz(90).

    성분으로 풀면:  ixx'=iyy, iyy'=ixx, izz'=izz,
                   ixy'=-ixy, ixz'=-iyz, iyz'=ixz
    """
    return {
        "ixx": I["iyy"], "iyy": I["ixx"], "izz": I["izz"],
        "ixy": -I["ixy"], "ixz": -I["iyz"], "iyz": I["ixz"],
    }


# 바퀴 회전자 반사 관성. URDF 에는 넣지 않는다 —
# robot_cfg.py 가 armature 로 관절에 직접 넣기 때문에 양쪽에 넣으면 이중 계상이다.
# (기존 모델은 CAD izz 에 이미 포함된 채로 armature 까지 더해져 있었다.)
WHEEL_ROTOR_REFLECTED = 157.33e-7 * 10.0**2   # 1.5733e-3 kg m^2

# MuJoCo 실측 참여도
PARTICIPATION = {"crank": 0.5385, "rocker": 0.3483, "shank": 0.9592, "wheel": 1.0}


def inertial(el):
    i = el.find("inertial")
    m = float(i.find("mass").get("value"))
    ine = i.find("inertia")
    return m, {k: float(ine.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}


def main():
    src = ET.parse(SRC).getroot()
    L = {l.get("name"): l for l in src.findall("link")}

    m_body, I_body = inertial(L["r_body"])
    m_crank, _ = inertial(L["r_crank"])
    m_rocker, _ = inertial(L["r_rocker"])
    m_shank, I_shank = inertial(L["r_shank"])
    m_wheel, I_wheel = inertial(L["r_wheel"])

    # 다리를 따라 움직이는 유효 질량
    eff = (
        m_crank * PARTICIPATION["crank"]
        + m_rocker * PARTICIPATION["rocker"]
        + m_shank * PARTICIPATION["shank"]
    )
    m_carrier = eff
    # 나머지는 몸체에 붙는다
    m_body_total = m_body + 2 * (m_crank + m_rocker + m_shank - eff)

    # 몸체 관성은 질량 증가분만큼 비례 확대 (근사).
    # 추가된 질량이 고관절 근처에 있어 실제보다 약간 작게 잡힌다.
    k = m_body_total / m_body
    I_body = {kk: v * k for kk, v in I_body.items()}

    # 캐리어 관성은 생크 것을 쓴다 (지배적)
    I_carrier = {kk: v * (m_carrier / m_shank) for kk, v in I_shank.items()}

    robot = ET.Element("robot", {"name": "wheeled_biped_simple"})
    ET.SubElement(robot, "!--", {})  # placeholder removed below

    def add_link(name, mass, I, com=(0, 0, 0), shape=None):
        """shape = (kind, size, origin_xyz, origin_rpy) 또는 None.

        RL 학습에는 충돌체가 반드시 있어야 한다. 바퀴는 실제 반지름 그대로,
        몸체와 캐리어는 관성만 맞춘 단순 도형으로 둔다 (접촉은 바퀴에서만 일어난다).
        """
        l = ET.SubElement(robot, "link", {"name": name})
        i = ET.SubElement(l, "inertial")
        ET.SubElement(i, "origin", {"xyz": " ".join(f"{v:g}" for v in com), "rpy": "0 0 0"})
        ET.SubElement(i, "mass", {"value": f"{mass:.6f}"})
        ET.SubElement(i, "inertia", {k2: f"{v:.6e}" for k2, v in I.items()})

        if shape is None:
            return l

        kind, size, oxyz, orpy = shape
        for tag in ("visual", "collision"):
            e = ET.SubElement(l, tag)
            ET.SubElement(e, "origin", {"xyz": oxyz, "rpy": orpy})
            g = ET.SubElement(e, "geometry")
            if kind == "box":
                ET.SubElement(g, "box", {"size": size})
            else:
                r, ln = size
                ET.SubElement(g, "cylinder", {"radius": f"{r:g}", "length": f"{ln:g}"})
        return l

    # 몸체: CAD 외형을 감싸는 상자. 접지는 바퀴로만 하므로 정확도는 덜 중요하다.
    add_link("base", m_body_total, rot_z90(I_body), com=(0, 0, 0.029),
             shape=("box", "0.15 0.19 0.13", "0 0 0.029", "0 0 0"))

    for side, sx in (("r", +1), ("l", -1)):

        # 캐리어: 다리를 나타내는 가는 원통. 절반 지점에 중심이 오게 둔다.
        leg_mid = (leg_map.H_MIN + leg_map.H_MAX) / 2 - R
        add_link(f"{side}_carrier", m_carrier, rot_z90(I_carrier),
                 shape=("cyl", (0.018, 0.10), f"0 0 {leg_mid/2:.4f}", "0 0 0"))
        # 바퀴: 실제 반지름 60 mm, 폭 30 mm. 회전축이 y 라 x 축 기준 90도 눕힌다.
        #
        # CAD 텐서는 스핀축이 z 였다(izz 가 큰 값). 단순화 모델의 회전축은 y 이므로
        # 스핀 성분을 y 로 옮긴다. 그리고 CAD izz 에 섞여 있던 회전자 반사 관성을
        # 빼서 armature 와의 이중 계상을 없앤다.
        I_spin_cad = I_wheel["izz"] - WHEEL_ROTOR_REFLECTED   # 1.817e-4
        I_trans = 0.5 * (I_wheel["ixx"] + I_wheel["iyy"])     # 9.504e-5
        I_wheel_rot = {"ixx": I_trans, "iyy": I_spin_cad, "izz": I_trans,
                       "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}
        add_link(f"{side}_wheel", m_wheel, I_wheel_rot,
                 shape=("cyl", (R, 0.030), "0 0 0", "1.5708 0 0"))

        # 다리 길이 관절: 고관절에서 아래로. 관절값 = 모터축~바퀴중심 거리
        j = ET.SubElement(robot, "joint", {"name": f"{side}_leg", "type": "prismatic"})
        ET.SubElement(j, "parent", {"link": "base"})
        ET.SubElement(j, "child", {"link": f"{side}_carrier"})
        ET.SubElement(j, "origin", {"xyz": f"0 {sx*HIP_Y:g} 0", "rpy": "0 0 0"})
        ET.SubElement(j, "axis", {"xyz": "0 0 -1"})
        ET.SubElement(j, "limit", {
            "lower": f"{leg_map.H_MIN - R:.6f}",
            "upper": f"{leg_map.H_MAX - R:.6f}",
            # 다리가 낼 수 있는 수직력 = 모터 피크토크 / (dh/dtheta)
            "effort": f"{9.0 / leg_map.dh_dtheta(leg_map.THETA_MIN):.1f}",
            "velocity": "1.0",
        })

        jw = ET.SubElement(robot, "joint", {"name": f"{side}_wheel_joint", "type": "continuous"})
        ET.SubElement(jw, "parent", {"link": f"{side}_carrier"})
        ET.SubElement(jw, "child", {"link": f"{side}_wheel"})
        ET.SubElement(jw, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(jw, "axis", {"xyz": "0 1 0"})
        ET.SubElement(jw, "limit", {"effort": "7.0", "velocity": "18.8"})

    robot.remove(robot.find("!--"))
    t = ET.ElementTree(robot)
    ET.indent(t, space="  ")
    t.write(OUT, encoding="utf-8", xml_declaration=True)

    print(f"{OUT.name} 생성")
    print(f"  base      {m_body_total*1000:8.1f} g")
    print(f"  carrier   {m_carrier*1000:8.1f} g  x2")
    print(f"  wheel     {m_wheel*1000:8.1f} g  x2   izz={I_wheel['izz']:.3e} (회전자 포함)")
    print(f"  합계      {(m_body_total+2*(m_carrier+m_wheel))*1000:8.1f} g")
    print(f"  좌표규약  전진 +x, 가로 +y, 바퀴축 y, 불안정축 pitch(y)")
    print(f"  바퀴관성  스핀(iyy) {I_spin_cad:.4e}  횡 {I_trans:.4e}  "
          f"(회전자 {WHEEL_ROTOR_REFLECTED:.4e} 는 armature 로 별도)")
    print(f"\n  다리 관절 범위 {(leg_map.H_MIN-R)*1000:.1f} ~ {(leg_map.H_MAX-R)*1000:.1f} mm (모터축~바퀴중심)")
    print(f"  = 다리 높이 h  {leg_map.H_MIN*1000:.1f} ~ {leg_map.H_MAX*1000:.1f} mm")
    print(f"  다리 최대 수직력 {9.0/leg_map.dh_dtheta(leg_map.THETA_MIN):.1f} N (모터 피크 9 Nm 기준)")


main()
