"""Isaac Lab ArticulationCfg — 휠-레그 계단 로봇 (학습용 단순화 모델).

4절링크를 직선 관절 하나로 대체한 모델이다. Isaac/PhysX 의 articulation 은
트리 구조만 지원해서 닫힌 고리를 articulation 밖 조인트로 빼야 하는데,
실제로 해보니 자유낙하에서 다리가 한계각에 걸리고 몸체를 고정하면
sim.reset() 에서 진행이 없었다. 닫힌 고리는 MuJoCo 로 따로 검증했고
(asd.py 예측과 0.3 mm 이내 일치), 학습 모델에서는 고리를 아예 없앤다.

    몸체 ─(prismatic, 다리 길이)─ 캐리어 ─(continuous)─ 바퀴

정책이 내는 다리 길이 h 는 model/leg_map.py 로 모터 각도 theta 로 바꿔
실기에 보낸다. dh/dtheta 가 110~119 mm/rad 로 거의 일정해 변환이 선형에 가깝다.

액추에이터 값 출처 (CubeMars 데이터시트 + CAD 실측):

  다리  AK60-6 V3.0 KV80   피크 9 Nm, 6:1, 정격속도 233 rpm(24V)
        직선 관절이 받는 힘으로 환산: 9 Nm / 0.1132 m/rad = 79.5 N
        속도 한계: 24.4 rad/s * 0.1132 m/rad = 2.76 m/s (실질 무제한)

  바퀴  AK45-10 KV75       피크 7 Nm, 10:1, 무부하 180 rpm = 18.8 rad/s
        회전자 관성 157.33 g*cm^2 를 감속비 제곱(x100)으로 환산해
        armature 1.573e-3 kg*m^2 로 넣는다. 이걸 빼면 바퀴 관성을
        1/10 로 잡는 셈이라 밸런싱 특성이 크게 어긋난다.

  다리 armature 는 AK60-6 회전자 243.5 g*cm^2 를 6^2 배 한 8.766e-4 를
  직선 관절로 환산 (/ (dh/dtheta)^2) 한 값이다.
"""

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sim import UsdFileCfg
from isaaclab.sim.schemas import ArticulationRootPropertiesCfg, RigidBodyPropertiesCfg

# scripts/convert_urdf.py 가 만드는 위치
USD_PATH = os.path.expanduser("~/wheeled_biped_isaaclab/usd/robot_simple.usd")

# leg_map.py 의 h 범위에서 온 값 (모터축 ~ 바퀴중심 거리)
LEG_MIN = 0.1225
LEG_MAX = 0.2425
LEG_MID = 0.5 * (LEG_MIN + LEG_MAX)

# dh/dtheta 대표값 [m/rad] — 토크/힘 환산 계수
DH_DTHETA = 0.1132

WHEELED_BIPED_CFG = ArticulationCfg(
    spawn=UsdFileCfg(
        usd_path=USD_PATH,
        rigid_props=RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=100.0,
            max_angular_velocity=100.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # 다리 중간 길이에서 바퀴가 바닥에 닿는 높이 + 여유
        pos=(0.0, 0.0, LEG_MID + 0.060 + 0.01),
        joint_pos={".*_leg": LEG_MID, ".*_wheel_joint": 0.0},
        joint_vel={".*": 0.0},
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_leg"],
            effort_limit_sim=9.0 / DH_DTHETA,     # 79.5 N
            velocity_limit_sim=2.76,
            stiffness=900.0,
            damping=30.0,
            armature=8.766e-4 / DH_DTHETA**2,     # 0.0684 kg (직선 관절 환산)
        ),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*_wheel_joint"],
            effort_limit_sim=7.0,
            velocity_limit_sim=18.8,
            stiffness=0.0,                         # 토크 제어
            damping=0.1,
            armature=1.5733e-3,                    # 회전자 157.33 g*cm^2 x 10^2
        ),
    },
    soft_joint_pos_limit_factor=0.95,
)
