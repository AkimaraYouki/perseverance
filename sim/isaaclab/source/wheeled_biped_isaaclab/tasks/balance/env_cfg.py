"""휠-레그 로봇 균형 + 속도 추종 학습 환경.

행동 4개
    r_leg, l_leg          다리 길이 [m] — 위치 제어 (실기의 모터 내장 PD 에 대응)
    r_wheel, l_wheel      바퀴 토크 [Nm] — AK45-10 의 MIT 모드에 대응

관측에 몸체 선속도를 넣지 않는다. 실기에서 측정할 수 없는 값이라
학습 때 쓰면 sim2real 에서 그대로 깨진다. 각속도, 중력 방향, 관절 상태,
명령, 직전 행동만 쓴다.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab.envs.mdp as mdp
from wheeled_biped_isaaclab.robot_cfg import LEG_MAX, LEG_MID, LEG_MIN, WHEELED_BIPED_CFG

from . import rewards as custom_rewards
from .actions import CommandOffsetLegActionCfg, FilteredJointEffortActionCfg
from .commands import WheelLegCommandCfg

# 학습 목표 다리 길이. 범위 중앙에 둔다.
TARGET_LEG = LEG_MID


@configclass
class SceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            # multiply 결합이라 실효 마찰 = 지면 x 로봇 재질. 지면을 1.0 으로 두고 로봇(바퀴) 재질을
            # 0.5~0.8 에서 무작위로 뽑는다 (사용자 지정).
            # 예전 0.8/0.7 x 로봇 기본 0.5 = 실효 0.4/0.35 였다.
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
    )
    robot = WHEELED_BIPED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    # 장면 항목은 AssetBaseCfg 로 감싸야 한다. 스포너 설정을 바로 넣으면
    # InteractiveScene 이 "Unknown asset config type" 으로 거부한다.
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.95)),
    )


# ---------------------------------------------------------------------------
# 학습 스테이지
#
# 2026-09-22 첫 본학습(1500 iter)이 실패했다. 4096 env 를 1500 iter 돌리는
# 동안 time_out 이 단 한 번도 안 났고(= 20 s 를 버틴 개체가 전무),
# 에피소드 길이는 iter 100 의 226 step 을 정점으로 155 step(0.78 s)까지
# 퇴행했다. action std 는 iter 200 에 0.23 으로 붕괴해 탐색이 먼저 죽었다.
#
# 원인은 "서는 법을 배우기 전에 넘어지는 걸 받아내고 달리는 것까지" 요구한
# 과제 난이도다. 그래서 두 단계로 나눈다.
#
#   STAGE 1 (기본) — 순수 균형. 속도 커맨드 0, 거의 수직에서 시작,
#                     외란/도메인 랜덤화 끔. time_out 비율이 올라오는지만 본다.
#   STAGE 2        — 원래 과제. 속도 추종 + 외란 + DR. STAGE 1 정책에서
#                     이어서 학습한다(--resume).
#
# 환경변수로 고른다:  WB_STAGE=2 ./scripts/train.sh ...
# ---------------------------------------------------------------------------
STAGE = int(os.environ.get("WB_STAGE", "1"))


@configclass
class CommandsCfg:
    # (vx, wz, 다리높이). 구조는 오리(SummerProject)에서 가져왔다 — commands.py 참조.
    # 이름은 관측/보상 호환을 위해 base_velocity 로 둔다.
    base_velocity = WheelLegCommandCfg(
        asset_name="robot",
        resampling_time_range=(3.0, 6.0),
        default_height=TARGET_LEG,
        # STAGE 1: 주행 명령 0, 높이만 무작위 (높이별로 서는 법 먼저)
        zero_vel_prob=1.0 if STAGE == 1 else 0.10,
        pure_axis_prob=0.35,
        pure_axis_weights=(0.50, 0.35, 0.15),
        fast_turn_prob=0.0 if STAGE == 1 else 0.20,
        fast_turn_wz=(0.6, 2.0),
        ranges=WheelLegCommandCfg.Ranges(
            # 2026-09-25 사용자 요청: 3 km/h. 0.85 m/s = 바퀴 14.2 rad/s (무부하 18.85 의 75 %).
            # 그 이상은 균형 회복에 쓸 속도 여유가 거의 없다.
            lin_vel_x=(-0.85, 0.85),
            # 2026-09-25 사용자: 회전이 너무 느리다. 제자리 회전 시 바퀴 = 1.35 x wz 이라 이론상 14 rad/s 까지 되지만
            # 균형·마찰 여유를 두고 2.5 rad/s (143 deg/s).
            ang_vel_z=(-2.5, 2.5),
            height=(0.130, 0.235),
        ),
    )


@configclass
class ActionsCfg:
    # 원시 액션은 두 항 모두 안에서 ±1 로 자른다 (actions.py 설명 참조).
    # 다리 목표 = 높이 명령 h_ref + 0.03 m * a. 높이는 명령이, 균형용 미세조정은 정책이.
    legs = CommandOffsetLegActionCfg(
        asset_name="robot",
        joint_names=[".*_leg"],
        scale=0.03,
        command_name="base_velocity",
        clip={".*_leg": (LEG_MIN, LEG_MAX)},
    )
    wheels = FilteredJointEffortActionCfg(
        asset_name="robot",
        joint_names=[".*_wheel_joint"],
        # 바퀴 토크 1차 LPF. 실기 제어기에도 같은 필터를 넣을 것.
        cutoff_hz=20.0,
        # 마찰한계가 0.82 Nm 이라 2.0 Nm 는 액션 범위의 60% 가 죽은 구간이고,
        # 탐색 노이즈만 증폭시켰다. PD 검사에서 실제로 쓰인 토크는 0.85 Nm.
        # 1.5 면 여유는 남기고 노이즈는 25% 줄어든다.
        scale=1.5,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        # 다리만. 바퀴 회전각은 계속 커져서(연속 관절) 학습 때 못 본 값이 된다.
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.002, n_max=0.002),
                            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_leg"])})
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.5, n_max=0.5))
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    # --- 시작 조건 ---
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            # STAGE 1 은 균형의 흡인영역 안에서 출발시킨다. 8.6 deg + 0.3 rad/s
            # 로 시작하면 서는 법을 배우기 전에 회복부터 요구받는다.
            "pose_range": ({"x": (-0.3, 0.3), "y": (-0.3, 0.3), "yaw": (-3.14, 3.14),
                            "pitch": (-0.03, 0.03), "roll": (-0.02, 0.02)}
                           if STAGE == 1 else
                           {"x": (-0.3, 0.3), "y": (-0.3, 0.3), "yaw": (-3.14, 3.14),
                            "pitch": (-0.15, 0.15), "roll": (-0.08, 0.08)}),
            "velocity_range": ({"x": (-0.05, 0.05), "y": (-0.05, 0.05), "z": (0.0, 0.0),
                                "roll": (-0.05, 0.05), "pitch": (-0.05, 0.05),
                                "yaw": (-0.05, 0.05)}
                               if STAGE == 1 else
                               {"x": (-0.2, 0.2), "y": (-0.2, 0.2), "z": (-0.1, 0.1),
                                "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3),
                                "yaw": (-0.3, 0.3)}),
        },
    )
    reset_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={"position_range": (-0.02, 0.02), "velocity_range": (-0.1, 0.1)},
    )

    # --- 도메인 랜덤화 ---
    # 바퀴-바닥 마찰 0.5 ~ 0.8 (사용자 지정 2026-09-25, 처음 0.25~0.6 에서 변경). 두 단계 모두 켠다.
    wheel_friction = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.5, 0.8),
            "dynamic_friction_range": (0.5, 0.8),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
            "make_consistent": True,
        },
    )
    # 실기 무게가 아직 확정 전이고 배터리/배선으로 바뀔 수 있어 범위를 넓게 둔다.
    base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (-0.3, 0.5),
            "operation": "add",
        },
    )
    # 무게중심 편심 15 mm 를 실측했다. 배치가 바뀔 수 있으니 랜덤화한다.
    base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "com_range": {"x": (-0.02, 0.02), "y": (-0.03, 0.01), "z": (-0.01, 0.01)},
        },
    )
    push = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(4.0, 8.0),
        params={"velocity_range": {"x": (-0.4, 0.4), "y": (-0.4, 0.4)}},
    )


@configclass
class RewardsCfg:
    # --- 과제 ---
    # 오리 판정 순위: 6 방향 추종 > 안정성 > 효율. 요 추종이 약했으므로 1.0 -> 1.5.
    track_lin_vel = RewTerm(func=custom_rewards.track_vx_exp, weight=2.0,
                            params={"command_name": "base_velocity", "std": 0.25})
    track_ang_vel = RewTerm(func=custom_rewards.track_wz_exp, weight=1.5,
                            params={"command_name": "base_velocity", "std": 0.5})
    track_height = RewTerm(func=custom_rewards.track_height_exp, weight=1.0,
                           params={"command_name": "base_velocity", "std": 0.02,
                                   "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_leg"])})
    # (고정 목표 leg_length 항은 높이 명령 추종 track_height 로 대체했다.
    #  asset_cfg 는 반드시 params 로 — 기본 인자는 매니저가 해석하지 않는다.)

    # --- 균형 ---
    # 두 바퀴 로봇이라 자세 유지가 과제보다 먼저다. 가중치를 크게 준다.
    # 2026-09-25 사용자 요청: 좌우는 수평(코너에서는 안쪽으로 기울기), 앞뒤는 균형 때문에 어쩔 수 없음.
    # 예전 upright(flat_orientation_l2)는 앞뒤·좌우를 같이 벌해서 코너 기울기와 부딪혔다.
    upright = RewTerm(func=custom_rewards.pitch_l2, weight=-2.0)
    roll_track = RewTerm(func=custom_rewards.roll_track_exp, weight=1.5, params={"std": 0.03})
    ang_vel_xy = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.1)
    lin_vel_z = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.0)
    # (다리 좌우 대칭 벌점은 뺐다 — 코너에서 안쪽 다리를 줄이는 것과 정면으로 부딪힌다.
    #  직진에서 수평이면 대칭은 roll_track 이 알아서 맞춘다.)
    # Isaac Lab 은 모든 보상항에 step_dt(1/200 s)를 곱한다. 버티기의
    # 총 상금은 weight * episode_length_s 이므로 2.0 이면 20 s 완주에 +40.
    alive = RewTerm(func=mdp.is_alive, weight=2.0)

    # --- 매끄러움 / 실기 보호 ---
    # 2026-09-25: 바퀴 진동(±1.5 Nm 뱅뱅) 억제. 이전 -0.02 / -0.002.
    # wheel_effort: 양 바퀴 1.5 Nm 상시면 초당 -0.45, 균형 유지(0.2~0.5 Nm)면 무시할 수준.
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.1)
    # 같은 이유로 params 명시. 기본 인자였을 때는 다리 지지력(40~80 N)까지 세고 있었다.
    wheel_effort = RewTerm(func=custom_rewards.wheel_effort_l2, weight=-0.1,
                           params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_wheel_joint"])})
    action_range = RewTerm(func=custom_rewards.action_out_of_range, weight=-0.5)
    leg_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.002,
                      params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_leg"])})
    joint_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0,
                           params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_leg"])})
    # 마찬가지로 dt 가 곱해져서 weight=-50 은 실효 -0.25 에 불과했다
    # (첫 학습 로그의 Episode_Reward/terminated = -0.013 x 20 s = -0.26).
    # 같은 에피소드의 upright 누적 -0.45 보다 작아 죽는 쪽이 쌌다.
    terminated = RewTerm(func=mdp.is_terminated, weight=-200.0)


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # 45도 이상 기울면 회복 불가로 본다
    fell_over = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 0.8})
    too_low = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": 0.12})


@configclass
class WheeledBipedBalanceEnvCfg(ManagerBasedRLEnvCfg):
    scene: SceneCfg = SceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        if STAGE == 1:
            # 균형 자체를 못 잡는 단계에서 외란과 DR 은 신호를 가린다.
            self.events.push = None
            self.events.base_mass = None
            self.events.base_com = None

        self.decimation = 4
        self.episode_length_s = 20.0
        # 200 Hz 정책 / 800 Hz 물리. 실기 제어 루프 500 Hz~1 kHz 와 같은 자리.
        self.sim.dt = 1.0 / 800.0
        self.sim.render_interval = self.decimation
        self.viewer.eye = (2.0, 2.0, 1.2)


@configclass
class WheeledBipedBalanceEnvCfg_PLAY(WheeledBipedBalanceEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 3.0
        self.observations.policy.enable_corruption = False
        self.events.push = None
