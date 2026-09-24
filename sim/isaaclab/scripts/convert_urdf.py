"""단순화 URDF -> USD 변환.

model/robot_simple.urdf 는 4절링크를 직선 관절로 바꾼 학습용 모델이다.
닫힌 고리가 없으므로 merge_fixed_joints 를 켜도 잃을 프레임이 없다.
"""

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

DEFAULT_URDF = Path.home() / "Desktop/휠 레그드 로봇/model/robot_simple.urdf"

parser = argparse.ArgumentParser()
parser.add_argument("--urdf", default=str(DEFAULT_URDF))
parser.add_argument("--out", default=str(Path.home() / "wheeled_biped_isaaclab/usd"))
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True

app = AppLauncher(args).app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

cfg = UrdfConverterCfg(
    asset_path=args.urdf,
    usd_dir=args.out,
    usd_file_name="robot_simple.usd",
    fix_base=False,
    merge_fixed_joints=True,
    force_usd_conversion=True,
    joint_drive=None,
)
conv = UrdfConverter(cfg)
print(f"\n[USD] {conv.usd_path}\n")
app.close()
