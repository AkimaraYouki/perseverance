"""CAD 4절링크 폐루프 모델 보기 (Isaac Sim GUI).

몸체를 공중에 고정하고 다리 모터(M)를 설계 전 구간(theta 38~97.5 deg)으로 천천히 오르내리게 한다.
바퀴는 천천히 돈다. 폐루프 모델 규칙(2026-09-25 loop_check 로 확인):
  * 모터는 explicit PD (implicit 드라이브는 Exclude 관절 구속력을 못 봐서 중력에 끌려 내려간다)
  * 수동 관절(I, K) 한계는 푼다 (폐루프와 겹치면 기구가 잠긴다)

    DISPLAY=:0 isaaclab.sh -p scripts/view_cad.py [--period 6] [--z 0.34]
"""
import argparse
import math
import os
import sys

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.expanduser("~/wheeled_biped_isaaclab/usd_loop/robot_simple.usd"))
ap.add_argument("--period", type=float, default=6.0, help="다리 한 번 오르내리는 주기 [s]")
ap.add_argument("--hz", type=float, default=1000.0, help="물리 주기 [Hz]")
ap.add_argument("--kp", type=float, default=60.0)
ap.add_argument("--kd", type=float, default=1.5)
ap.add_argument("--z", type=float, default=0.34, help="몸체 고정 높이 [m] (다리 최대로 펴도 바퀴가 바닥에 안 닿게)")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import IdealPDActuatorCfg, ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402

sys.path.insert(0, os.path.expanduser("~/perseverance/sim/model"))
import leg_map  # noqa: E402

# explicit PD 는 물리 스텝이 굵으면 불안정하다 (1/400 s + kp200/kd5 에서 크게 떨었다, 2026-09-25).
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / args.hz, render_interval=max(1, int(args.hz / 100))))
sim.set_camera_view(eye=(0.75, -0.65, 0.45), target=(0.05, -0.08, 0.22))
sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
sim_utils.DomeLightCfg(intensity=2500.0, color=(0.95, 0.95, 1.0)).func("/World/light", sim_utils.DomeLightCfg(intensity=2500.0))

robot = Articulation(ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=args.usd,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=True, enabled_self_collisions=False,
            solver_position_iteration_count=16, solver_velocity_iteration_count=4)),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, args.z), joint_pos={".*": 0.0}),
    actuators={
        "motor": IdealPDActuatorCfg(joint_names_expr=[".*_joint_M"], stiffness=args.kp, damping=args.kd, effort_limit=9.0),
        "passive": ImplicitActuatorCfg(joint_names_expr=[".*_joint_[IK]"], stiffness=0.0, damping=0.02),
        "wheel": ImplicitActuatorCfg(joint_names_expr=[".*_joint_W"], stiffness=0.0, damping=0.3),
    },
))
sim.reset()
names = robot.joint_names
lim = robot.data.joint_pos_limits.clone()
for n in names:
    if "_joint_I" in n or "_joint_K" in n:
        lim[:, names.index(n)] = torch.tensor([-math.pi, math.pi], device=lim.device)
robot.write_joint_position_limit_to_sim(lim)
iM = {s: names.index(f"{s}_joint_M") for s in "LR"}
iW = [names.index(n) for n in names if "_joint_W" in n]
print(f"[CAD 모델] 관절 {names}", flush=True)
print("[CAD 모델] 다리가 설계 전 구간을 오르내리고 바퀴가 돈다. 창을 닫으면 끝.", flush=True)

M_LO, M_HI = math.radians(-7.0), math.radians(52.5)   # theta 38 ~ 97.5 deg (설계 구간, 한계 여유 없이)
t, k = 0.0, 0
qd_hist = []
while app.is_running():
    s = 0.5 - 0.5 * math.cos(2 * math.pi * t / args.period)
    m = M_LO + (M_HI - M_LO) * s
    tgt = torch.zeros(1, robot.num_joints, device=sim.device)
    tgt[0, iM["L"]] = m
    tgt[0, iM["R"]] = -m
    robot.set_joint_position_target(tgt)
    vel = torch.zeros_like(tgt)
    vel[0, iW] = torch.tensor([3.0, -3.0], device=sim.device)
    robot.set_joint_velocity_target(vel)
    robot.write_data_to_sim()
    sim.step()
    robot.update(sim.cfg.dt)
    t += sim.cfg.dt
    k += 1
    qd_hist.append(float(robot.data.joint_vel[0, iM["L"]]))
    if k % int(args.hz) == 0:
        q = float(robot.data.joint_pos[0, iM["L"]])
        th = math.radians(45.002) + q
        print(f"  t {t:5.1f}s  모터 M {math.degrees(q):6.1f} deg (theta {math.degrees(th):5.1f})  "
              f"다리높이 h {leg_map.h_of_theta(th)*1000:5.1f} mm   떨림(모터 각속도 부호반전) {sum(1 for a, b in zip(qd_hist, qd_hist[1:]) if a*b < 0)} 회/s", flush=True)
        qd_hist.clear()
app.close()
