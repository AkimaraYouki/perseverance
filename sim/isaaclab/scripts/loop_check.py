"""CAD 폐루프 모델(usd_loop) 검증: 몸체 고정, 다리 모터 M 스윕.

- 폐루프 잔차: 몸체의 P 와 로커의 P 가 월드에서 얼마나 떨어지나 (exclude 관절은 솔버가 약하게 푼다)
- 다리 길이(고관절~바퀴중심 수직거리) vs leg_map (MuJoCo 폐루프로 0.3 mm 검증된 기구학)
"""
import json, math, os, sys, time
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True).app

import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.utils.math import quat_apply

sys.path.insert(0, os.path.expanduser("~/perseverance/sim/model"))
import leg_map  # noqa: E402

LOOP = json.load(open("/home/parksuho/Desktop/휠 레그드 로봇/export_(5)_fixed/loop_closure.json"))
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 800, gravity=(0.0, 0.0, 0.0) if os.environ.get("NOGRAV") else (0.0, 0.0, -9.81)))
cfg = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.expanduser("~/wheeled_biped_isaaclab/usd_loop/robot_simple.usd"),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=True, enabled_self_collisions=False,
            solver_position_iteration_count=int(os.environ.get("PIT","16")), solver_velocity_iteration_count=int(os.environ.get("VIT","4")))),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0, 0, 1.0), joint_pos={".*": 0.0}),
    actuators={
        "motor": ImplicitActuatorCfg(joint_names_expr=[".*_joint_M"], stiffness=float(os.environ.get("KP","200")), damping=float(os.environ.get("KD","5")), effort_limit_sim=float(os.environ.get("EFF","9.0"))),
        "passive": ImplicitActuatorCfg(joint_names_expr=[".*_joint_[IK]"], stiffness=0.0, damping=0.02),
        "wheel": ImplicitActuatorCfg(joint_names_expr=[".*_joint_W"], stiffness=0.0, damping=0.0),
    },
)
if os.environ.get("NOLOOP"):
    cfg.spawn.usd_path = os.path.expanduser("~/wheeled_biped_isaaclab/usd_loop_noloop/robot_simple.usd")
if os.environ.get("EXPLICIT"):
    # 모터를 명시적 PD 로 (토크를 계산해 관절 힘으로 넣는다). implicit 드라이브가 폐루프 구속을 못 보는지 확인용
    cfg.actuators["motor"] = IdealPDActuatorCfg(joint_names_expr=[".*_joint_M"], stiffness=float(os.environ.get("KP","200")),
                                                damping=float(os.environ.get("KD","5")), effort_limit=float(os.environ.get("EFF","9.0")))
robot = Articulation(cfg)
t0 = time.time()
sim.reset()
print(f"[reset] {time.time()-t0:.1f} s  관절 {robot.joint_names}", flush=True)
names = robot.joint_names
bi = {n: i for i, n in enumerate(robot.body_names)}
jm = {s: names.index(f"{s}_joint_M") for s in "LR"}

def body_pose(n):
    return robot.data.body_pos_w[0, bi[n]], robot.data.body_quat_w[0, bi[n]]

def world_pt(body, p):
    pos, q = body_pose(body)
    return pos + quat_apply(q.unsqueeze(0), torch.tensor([p], device=pos.device, dtype=pos.dtype))[0]

hipL = torch.tensor([0.08, 0.001, 0.0])  # URDF L_joint_M 원점 (base_link 기준)
print("관절 한계(deg):", [(n, round(math.degrees(float(a)),1), round(math.degrees(float(b)),1)) for n,(a,b) in zip(names, robot.data.joint_pos_limits[0].tolist())], flush=True)
print("질량(g):", [(n, round(float(m)*1000,1)) for n, m in zip(robot.body_names, robot.root_physx_view.get_masses()[0].tolist())], flush=True)
print("관성 대각(kg m2):", [(n, [round(float(x),6) for x in I[[0,4,8]]]) for n, I in zip(robot.body_names, robot.root_physx_view.get_inertias()[0])], flush=True)
if os.environ.get("MASSONLY"):
    app.close(); sys.exit()
print("PhysX 강성", robot.root_physx_view.get_dof_stiffnesses()[0].tolist(), flush=True)
print("PhysX 최대힘", robot.root_physx_view.get_dof_max_forces()[0].tolist(), flush=True)
if os.environ.get("FREE_IK"):
    lim = robot.data.joint_pos_limits.clone()
    for n in ("L_joint_I", "R_joint_I", "L_joint_K", "R_joint_K"):
        lim[:, names.index(n)] = torch.tensor([-math.pi, math.pi], device=lim.device)
    robot.write_joint_position_limit_to_sim(lim)
    print("수동관절 I/K 한계 해제", flush=True)
rows = []
for deg in np.linspace(-7.0, 52.5, 12):
    tgt = torch.zeros(1, robot.num_joints, device=sim.device)
    tgt[0, jm["L"]] = math.radians(deg); tgt[0, jm["R"]] = -math.radians(deg)
    robot.set_joint_position_target(tgt)
    for _ in range(400):
        robot.write_data_to_sim(); sim.step(); robot.update(sim.cfg.dt)
    res = {s: float(torch.linalg.norm(world_pt(LOOP[s]["body"], LOOP[s]["p_body"]) - world_pt(LOOP[s]["rocker"], LOOP[s]["p_rocker"]))) for s in "LR"}
    bpos, bq = body_pose("base_link")
    wl, _ = body_pose("l_wheel")
    hip_w = bpos + quat_apply(bq.unsqueeze(0), hipL.to(bpos.device).unsqueeze(0))[0]
    leg = float(hip_w[2] - wl[2])
    qL = math.degrees(float(robot.data.joint_pos[0, jm["L"]]))
    rows.append((deg, qL, res["L"] * 1000, res["R"] * 1000, leg * 1000))
    allq = [round(math.degrees(float(x)),1) for x in robot.data.joint_pos[0]]
    tau = [round(float(x),2) for x in robot.data.applied_torque[0]]
    print(f"   모든관절 {allq}  토크 {tau}", flush=True)
    print(f"M목표 {deg:6.1f}° 실제 {qL:6.1f}° | 폐루프 잔차 L {res['L']*1000:6.2f} R {res['R']*1000:6.2f} mm | 다리(고관절~바퀴) {leg*1000:6.1f} mm", flush=True)

# leg_map 과 비교: CAD 영점 M=0 이 theta 45.0 deg. 방향은 두 가지를 다 보고 맞는 쪽을 쓴다
qs = np.array([r[1] for r in rows]); legs = np.array([r[4] for r in rows])
best = None
for sgn in (+1, -1):
    th = np.radians(45.002 + sgn * qs)
    ok = (th >= leg_map.THETA_MIN - 1e-3) & (th <= leg_map.THETA_MAX + 1e-3)
    ref = np.array([(leg_map.h_of_theta(t) - leg_map.R_WHEEL) * 1000 for t in th])
    err = np.abs(ref - legs)[ok]
    if len(err) and (best is None or err.max() < best[1]):
        best = (sgn, err.max(), err.mean(), ref)
if best is None:
    print("leg_map 비교 불가 (theta 범위 밖)"); app.close(); sys.exit()
print(f"\nleg_map 대비 (theta = 45.0 {'+' if best[0]>0 else '-'} M): 최대 오차 {best[1]:.2f} mm, 평균 {best[2]:.2f} mm", flush=True)
print(f"폐루프 잔차 최대 L {max(r[2] for r in rows):.2f} mm, R {max(r[3] for r in rows):.2f} mm", flush=True)
app.close()
