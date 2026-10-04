"""PhysX 관절 마찰 단위 확인 (2026-10-03): 로봇을 공중에 붙잡고 바퀴에 일정 토크 -> 0.5 s 뒤 바퀴 속도.
정지 마찰 0.45 를 N·m 로 받는다면: 토크 0.3 은 안 돌고, 0.6 은 (0.6 - 운동 마찰) / 관성 으로 가속한다.
    ~/Desktop/IsaacLab/isaaclab.sh -p scripts/_isaaclab_launch.py scripts/diag_wheel_friction.py --headless
"""
import argparse

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
args.device = "cpu"                                    # pv robust 와 같음 (env 도 cpu)
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance import cad  # noqa: E402

# (정지 마찰, 운동 마찰, 드라이브 데드밴드 [N·m], 지령 전류 [A]) 환경마다. 토크 = 전류 x Kt 1.27 (AK45-10 출력축)
KT = 1.27
AMPS = (0.30, 0.35, 0.40, 0.45, 0.50)
MODELS = {"마찰 모델 x0.9 (정지 0.51 / 운동 0.25)": (0.57 * 0.9, 0.25, 0.0),
          "마찰 모델 x1.1 (정지 0.63 / 운동 0.25)": (0.57 * 1.1, 0.25, 0.0),
          "데드밴드 모델 x0.9 (0.51, 마찰 0.1)": (0.1, 0.1, 0.57 * 0.9),
          "데드밴드 모델 x1.1 (0.63, 마찰 0.1)": (0.1, 0.1, 0.57 * 1.1)}
CASES = [(st_, dy_, db_, a_ * KT) for (st_, dy_, db_) in MODELS.values() for a_ in AMPS]
N = len(CASES)
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.terrains import TerrainImporterCfg  # noqa: E402

cfg = parse_env_cfg("Isaac-WheeledBiped-CAD-Rough-Play-v0", device="cpu", num_envs=N, use_fabric=True)
cfg.scene.env_spacing = 4.0
cfg.scene.terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane", collision_group=-1)
env = gym.make("Isaac-WheeledBiped-CAD-Rough-Play-v0", cfg=cfg).unwrapped
robot = env.scene["robot"]
wid = robot.find_joints(cad.WHEEL_JOINTS, preserve_order=True)[0]
env.reset()
st = torch.tensor([[c[0]] * 2 for c in CASES]); dy = torch.tensor([[c[1]] * 2 for c in CASES])
robot.write_joint_friction_coefficient_to_sim(st, dy, 0.0, joint_ids=wid)
robot.actuators["wheels"].deadband[:] = torch.tensor([[c[2]] * 2 for c in CASES])
root = robot.data.default_root_state.clone(); root[:, :3] += env.scene.env_origins; root[:, 2] += 0.5   # 공중
sim, dt = env.sim, env.physics_dt
tq = torch.tensor([[c[3]] * 2 for c in CASES])
wa = robot.actuators["wheels"]
for k in range(int(0.5 / dt)):
    robot.write_root_state_to_sim(root)
    robot.set_joint_effort_target(tq, joint_ids=wid)
    robot.write_data_to_sim()
    sim.step(render=False)
    robot.update(dt)
    if k == int(0.2 / dt):
        v02 = robot.data.joint_vel[:, wid[0]].clone()
v05 = robot.data.joint_vel[:, wid[0]]
print("바퀴를 띄우고 지령 전류를 0.5 s 동안 걸었을 때 바퀴 속도 [rad/s] (0.2 s / 0.5 s) — 0 이면 안 돎")
print(f"  {'모델':40s}" + "".join(f"{a_:>14.2f} A" for a_ in AMPS))
for m_i, name in enumerate(MODELS):
    cells = [f"{float(v02[m_i * len(AMPS) + j_]):6.1f} /{float(v05[m_i * len(AMPS) + j_]):5.1f}" for j_ in range(len(AMPS))]
    print(f"  {name:40s}" + "".join(f"{c_:>16s}" for c_ in cells))
env.close()
app.close()
