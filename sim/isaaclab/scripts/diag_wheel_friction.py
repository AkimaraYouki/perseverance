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

# (정지 마찰, 운동 마찰, 걸어 준 토크) 환경마다
CASES = [(0.0, 0.0, 0.3), (0.45, 0.30, 0.3), (0.45, 0.30, 0.6), (0.45, 0.30, 1.2), (4.5, 3.0, 0.3)]
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
root = robot.data.default_root_state.clone(); root[:, :3] += env.scene.env_origins; root[:, 2] += 0.5   # 공중
sim, dt = env.sim, env.physics_dt
tq = torch.tensor([[c[2]] * 2 for c in CASES])
wa = robot.actuators["wheels"]
for k in range(int(0.5 / dt)):
    robot.write_root_state_to_sim(root)
    robot.set_joint_effort_target(tq, joint_ids=wid)
    robot.write_data_to_sim()
    sim.step(render=False)
    robot.update(dt)
    if k in (int(0.1 / dt), int(0.5 / dt) - 1):
        print(f"t {k*dt:.2f} s  바퀴 속도 [rad/s]:", [round(float(v), 2) for v in robot.data.joint_vel[:, wid[0]]])
I = cad.WHEEL_IZZ
for (s_, d_, t_), v in zip(CASES, robot.data.joint_vel[:, wid[0]]):
    print(f"정지 {s_:.2f} 운동 {d_:.2f} 토크 {t_:.2f}: 0.5 s 뒤 {float(v):+7.2f} rad/s  "
          f"(마찰 없으면 {t_ / I * 0.5:.0f}, 운동 마찰을 N·m 로 빼면 {max(0.0, t_ - d_) / I * 0.5 if t_ > s_ else 0.0:.0f})")
env.close()
app.close()
