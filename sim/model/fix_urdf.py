#!/usr/bin/env python3
"""
onshape-to-robot 로 내보낸 URDF 후처리.

CAD 에서는 고칠 수 없는 것들을 내보내기 후에 한 번 돌려서 바로잡는다.
Onshape 는 모터 스펙을 모르고, 메이트에는 "무한회전" 개념이 없기 때문에
export 를 다시 해도 아래 항목들은 계속 같은 상태로 나온다.

  1) 바퀴 조인트 revolute(+-180deg) -> continuous (위치 한계 제거)
  2) 토크 / 속도 한계를 실제 모터 값으로
  3) 바퀴 링크 이름을 나사 이름 -> r_wheel / l_wheel

사용법
    python3 fix_urdf.py "export (4).zip"     # zip 을 풀고 그 안의 robot.urdf 수정
    python3 fix_urdf.py some/robot.urdf      # urdf 직접 수정 (.bak 백업 생성)

여러 번 돌려도 결과가 같다 (멱등).
"""

import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import zipfile

# ============================================================
# 모터 사양
# ============================================================

# 다리 모터 CubeMars AK60-6 V3.0 KV80
#   피크 토크 9 Nm
#   정격속도 233 rpm (24V) = 24.4 rad/s,  490 rpm (48V) = 51.3 rad/s
LEG_EFFORT = 9.0
LEG_VELOCITY = 24.4

# AK60-6 데이터시트에서 회전자 관성을 확인하면 여기에 넣는다 (6:1).
# None 이면 다리 쪽 보정은 건너뛴다.
LEG_ROTOR_INERTIA = 243.5e-7
LEG_GEAR_RATIO = 6.0

# 휠 모터 CubeMars AK45-10 KV75
#   피크 토크 7 Nm
#   무부하 180 rpm = 18.8 rad/s
WHEEL_EFFORT = 7.0
WHEEL_VELOCITY = 18.8

# 회전자 관성 (데이터시트 "관성 모멘트" 157.33 gcm^2 = 157.33e-7 kg m^2).
#
# 감속기를 거치면 출력축에서 본 실효 관성이 감속비의 제곱만큼 커진다.
#   157.33e-7 x 10^2 = 1.573e-3 kg m^2
# 바퀴 자체 관성이 1.82e-4 라 회전자 쪽이 8.7 배 크다.
# 이걸 빼면 바퀴 관성을 1/10 로 잡는 셈이라 밸런싱 모델이 크게 어긋난다.
WHEEL_ROTOR_INERTIA = 157.33e-7
WHEEL_GEAR_RATIO = 10.0

# 수동 조인트 (4절링크가 구속하므로 구동력 없음).
# velocity 는 0 으로 두면 움직이지 못하는 것으로 해석하는 도구가 있어 넉넉히 준다.
PASSIVE_EFFORT = 0.0
PASSIVE_VELOCITY = 100.0


# ============================================================
# 관절 각도 한계 [rad]
#
# 내보낸 URDF 는 모든 관절이 +-180deg 로 나온다. 실제로는 4절링크가
# 훨씬 좁은 범위만 움직이고, 범위를 안 걸면 시뮬레이터가 조립 분기를
# 뒤집어 링크가 관통하거나 +-180deg 에 붙어 멈추는 일이 생긴다.
# (Isaac 에서 실제로 I 관절이 +-175deg 에 걸려 멈추는 현상이 있었다.)
#
# 아래 값은 MuJoCo 닫힌고리 모델을 설계 구간
# theta_kR 38.021 ~ 97.789 deg 로 훑어서 실측한 것이다.
# CAD 영점의 theta_kR = 45.002 deg.
#
# 주의: 이 값은 이 CAD 치수에 묶여 있다. 링크 길이가 바뀌면 다시 재야 한다.
# 좌우 M, I 는 부호가 반대이고 K 는 양쪽이 같다.
# 값은 model/joint_limits_from_urdf.py 가 이 URDF 의 폐루프를 직접 풀어
# joint_limits.json 으로 내놓는다 (관절 이름 기준). CAD 를 다시 뽑으면 관절 로컬
# 프레임이 바뀔 수 있으므로 반드시 다시 돌린다. (2026-09-24 export (5) 부터:
# 좌우 이름이 ROS 규약으로 바뀌어 L = +y 쪽이다. 아래 기본값은 옛 이름 기준이라 쓰지 말 것.)
# 파일이 없으면 아래 기본값(현재 CAD 기준)을 쓴다.
JOINT_MARGIN = 0.05

JOINT_RANGE = {
    "R_joint_M": (-0.1718, 0.9713),
    "R_joint_I": (-0.1872, 1.2675),
    "R_joint_K": (-1.5940, 0.2123),
    "L_joint_M": (-0.9713, 0.1718),
    "L_joint_I": (-1.2675, 0.1872),
    "L_joint_K": (-1.5940, 0.2123),
}

_lim_file = Path(__file__).parent / "joint_limits.json"

if _lim_file.exists():
    JOINT_RANGE = {
        k: tuple(v) for k, v in json.loads(_lim_file.read_text())["limits"].items()
    }


# ============================================================
# 조인트 이름 규칙  (onshape-to-robot 내보내기 기준)
#
#   R_joint_M / L_joint_M   다리 모터 (몸체 -> 크랭크)
#   R_joint_I / L_joint_I   수동 (크랭크 -> 생크)
#   R_joint_K / L_joint_K   수동 (생크 -> 로커)
#   R_joint_W / L_joint_W   휠 모터 (생크 -> 바퀴)
# ============================================================

WHEEL_SUFFIX = "_joint_W"
LEG_SUFFIX = "_joint_M"
PASSIVE_SUFFIXES = ("_joint_I", "_joint_K")


def side_prefix(joint_name):
    """R_joint_W -> 'r',  L_joint_W -> 'l'"""

    head = joint_name.split("_", 1)[0].lower()

    return head if head in ("r", "l") else None


def set_limit(joint, effort, velocity, keep_range=True):
    """limit 태그의 effort/velocity 와 각도 한계를 갱신.

    keep_range=False 면 위치 한계를 제거한다 (continuous 용).
    """

    limit = joint.find("limit")

    if limit is None:
        limit = ET.SubElement(joint, "limit")

    limit.set("effort", f"{effort:g}")
    limit.set("velocity", f"{velocity:g}")

    if not keep_range:
        limit.attrib.pop("lower", None)
        limit.attrib.pop("upper", None)
        return limit

    rng = JOINT_RANGE.get(joint.get("name"))

    if rng is not None:
        limit.set("lower", f"{rng[0]:.4f}")
        limit.set("upper", f"{rng[1]:.4f}")

    return limit


def sanitize_asset_names(root, urdf_path):
    """USD prim 이름은 숫자로 시작할 수 없다.

    Isaac Sim 의 URDF 임포터는 메시 파일 이름으로 prim 경로를 만드는데,
    '2inch_lcd.stl' 같은 이름이 있으면 </visuals/r_body/2inch_lcd> 가
    잘못된 경로가 되어 "Used null prim" 으로 임포트가 통째로 실패한다.
    (재질 이름은 임포터가 알아서 고쳐주지만 메시 prim 은 고쳐주지 않는다.)

    숫자로 시작하는 에셋 파일과 그 참조에 'm' 을 붙인다.
    """

    assets_dir = urdf_path.parent / "assets"

    renamed = []
    done = {}

    for mesh in root.iter("mesh"):

        ref = mesh.get("filename")

        if ref is None:
            continue

        name = ref.rsplit("/", 1)[-1]

        if not name[:1].isdigit():
            continue

        if name not in done:
            new_name = "m" + name
            src = assets_dir / name

            if src.exists():
                src.rename(assets_dir / new_name)

            done[name] = new_name
            renamed.append(f"  에셋 이름: {name} -> {new_name}")

        mesh.set("filename", ref.replace(name, done[name]))

    # 재질 이름도 숫자로 시작하면 정리해 둔다 (임포터가 고치지만 이름이 예측 가능해진다)
    for tag in ("material",):
        for el in root.iter(tag):
            n = el.get("name")
            if n and n[:1].isdigit():
                el.set("name", "m" + n)

    return renamed


def rename_link(root, old, new):
    """링크 이름과 이를 참조하는 모든 조인트의 parent/child 를 함께 바꾼다."""

    for link in root.findall("link"):
        if link.get("name") == old:
            link.set("name", new)

    for joint in root.findall("joint"):
        for tag in ("parent", "child"):
            ref = joint.find(tag)
            if ref is not None and ref.get("link") == old:
                ref.set("link", new)


def add_rotor_inertia(root, joint, rotor_inertia, ratio, already):
    """감속기 출력축에서 본 회전자 실효 관성(I x N^2)을 구동 링크에 더한다.

    URDF 에는 MuJoCo 의 armature 같은 항목이 없어서 링크 관성에 직접 넣는다.
    회전자 질량은 이미 모터가 붙은 링크에 들어가 있으므로 질량은 건드리지 않고
    회전축 성분만 키운다.
    """

    if rotor_inertia is None:
        return None

    child = joint.find("child").get("link")

    if child in already:
        return None

    axis = joint.find("axis")
    axis_xyz = axis.get("xyz") if axis is not None else "0 0 1"

    comp = {"1 0 0": "ixx", "0 1 0": "iyy", "0 0 1": "izz"}.get(
        " ".join(f"{float(v):g}" for v in axis_xyz.split())
    )

    if comp is None:
        return f"  [건너뜀] {joint.get('name')}: 회전축이 좌표축과 안 맞음 ({axis_xyz})"

    link = next((l for l in root.findall("link") if l.get("name") == child), None)

    if link is None or link.find("inertial") is None:
        return f"  [건너뜀] {child}: inertial 없음"

    inertial = link.find("inertial")

    rpy = inertial.find("origin").get("rpy", "0 0 0")

    if any(abs(float(v)) > 1e-6 for v in rpy.split()):
        return f"  [건너뜀] {child}: 관성 프레임이 회전되어 있음 (rpy={rpy})"

    inertia = inertial.find("inertia")

    old = float(inertia.get(comp))
    refl = rotor_inertia * ratio**2
    new = old + refl

    inertia.set(comp, f"{new:.6e}")

    already.add(child)

    return (
        f"  {child}.{comp}: {old:.4e} -> {new:.4e}"
        f"  (회전자 {rotor_inertia:.3e} x {ratio:g}^2 = {refl:.4e} 추가)"
    )


def fix(urdf_path):

    urdf_path = Path(urdf_path)

    # 주석(부품 이름)을 살리기 위해 comment 를 보존하는 파서를 쓴다
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))

    tree = ET.parse(urdf_path, parser=parser)

    root = tree.getroot()

    # 관성은 더하는 연산이라 두 번 적용되면 안 된다.
    # 이미 보정한 링크를 옆 파일에 기록해 두고 건너뛴다.
    state_path = urdf_path.parent / ".fix_urdf_state.json"

    already = set()

    if state_path.exists():
        already = set(json.loads(state_path.read_text()).get("rotor_inertia_done", []))

    changes = []

    # --------------------------------------------------------
    # 0)  USD 가 못 읽는 에셋 이름 정리 (숫자로 시작 금지)
    # --------------------------------------------------------
    changes += sanitize_asset_names(root, urdf_path)

    # 로봇 이름 앞뒤 공백 제거 (config.json 의 "wheeled_biped " 가 그대로 들어온다)
    if root.get("name") != root.get("name", "").strip():
        root.set("name", root.get("name").strip())
        changes.append(f"  로봇 이름 공백 제거: {root.get('name')!r}")

    # 루트 링크 -> base_link (ROS 관례. CAD 파트 이름이 r_body / l_body 로 바뀌어 왔다)
    children = {j.find("child").get("link") for j in root.findall("joint")}
    roots = [l.get("name") for l in root.findall("link") if l.get("name") not in children]
    if len(roots) == 1 and roots[0] != "base_link":
        rename_link(root, roots[0], "base_link")
        changes.append(f"  루트 링크: {roots[0]} -> base_link")

    for joint in root.findall("joint"):

        name = joint.get("name")

        # --------------------------------------------------------
        # 1) + 3)  바퀴: continuous 로 바꾸고 링크 이름 정리
        # --------------------------------------------------------
        if name.endswith(WHEEL_SUFFIX):

            if joint.get("type") != "continuous":
                changes.append(f"  {name}: type {joint.get('type')} -> continuous")
                joint.set("type", "continuous")

            side = side_prefix(name)
            child = joint.find("child")

            if side and child is not None:
                old = child.get("link")
                new = f"{side}_wheel"

                if old != new:
                    rename_link(root, old, new)
                    changes.append(f"  링크 이름: {old[:38]}... -> {new}")

            msg = add_rotor_inertia(
                root, joint, WHEEL_ROTOR_INERTIA, WHEEL_GEAR_RATIO, already
            )
            if msg:
                changes.append(msg)

            set_limit(joint, WHEEL_EFFORT, WHEEL_VELOCITY, keep_range=False)
            changes.append(
                f"  {name}: 위치한계 제거, effort {WHEEL_EFFORT}, velocity {WHEEL_VELOCITY}"
            )

        # --------------------------------------------------------
        # 2)  다리 모터
        # --------------------------------------------------------
        elif name.endswith(LEG_SUFFIX):

            msg = add_rotor_inertia(
                root, joint, LEG_ROTOR_INERTIA, LEG_GEAR_RATIO, already
            )
            if msg:
                changes.append(msg)

            lim = set_limit(joint, LEG_EFFORT, LEG_VELOCITY)
            changes.append(
                f"  {name}: effort {LEG_EFFORT}, velocity {LEG_VELOCITY}"
                f", 각도 {lim.get('lower')} ~ {lim.get('upper')} rad"
            )

        # --------------------------------------------------------
        # 2)  수동 조인트
        # --------------------------------------------------------
        elif any(name.endswith(s) for s in PASSIVE_SUFFIXES):

            lim = set_limit(joint, PASSIVE_EFFORT, PASSIVE_VELOCITY)
            changes.append(
                f"  {name}: effort {PASSIVE_EFFORT} (수동)"
                f", 각도 {lim.get('lower')} ~ {lim.get('upper')} rad"
            )

    ET.indent(tree, space="  ")

    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)

    state_path.write_text(json.dumps({"rotor_inertia_done": sorted(already)}, indent=2))

    return changes


def main():

    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)

    target = Path(sys.argv[1]).expanduser()

    if not target.exists():
        print(f"파일이 없습니다: {target}")
        sys.exit(1)

    if target.suffix.lower() == ".zip":

        out_dir = target.with_suffix("")
        out_dir = out_dir.parent / (out_dir.name.replace(" ", "_") + "_fixed")

        if out_dir.exists():
            shutil.rmtree(out_dir)

        with zipfile.ZipFile(target) as z:
            z.extractall(out_dir)

        urdf = out_dir / "robot.urdf"
        print(f"압축 해제: {out_dir}")

    else:
        urdf = target
        shutil.copy(urdf, urdf.with_suffix(".urdf.bak"))
        print(f"백업: {urdf.with_suffix('.urdf.bak')}")

    if not urdf.exists():
        print(f"robot.urdf 를 찾을 수 없습니다: {urdf}")
        sys.exit(1)

    changes = fix(urdf)

    print(f"\n수정 완료: {urdf}\n")
    for c in changes:
        print(c)


if __name__ == "__main__":
    main()
