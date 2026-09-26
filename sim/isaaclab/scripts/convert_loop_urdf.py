"""폐루프 CAD URDF (make_loop_urdf.py 결과) -> USD. usd_loop/config.yaml 과 같은 설정.

다음 단계: add_loop_joints.py 로 4절링크 폐루프 관절을 단다.

    isaaclab.sh -p convert_loop_urdf.py <robot_loop.urdf> <usd_dir>
"""
import sys

from isaaclab.app import AppLauncher

urdf, out = sys.argv[1], sys.argv[2]
app = AppLauncher(headless=True).app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402

cfg = UrdfConverterCfg(
    asset_path=urdf, usd_dir=out, usd_file_name="robot_simple.usd", force_usd_conversion=True, make_instanceable=True,
    fix_base=False, root_link_name=None, link_density=0.0, merge_fixed_joints=True,
    convert_mimic_joints_to_normal_joints=False, joint_drive=None, collider_type="convex_hull", self_collision=False,
    replace_cylinders_with_capsules=False, collision_from_visuals=False,
)
conv = UrdfConverter(cfg)
print(f"\n[USD] {conv.usd_path}", flush=True)
app.close()
