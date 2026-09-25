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
    # 정지 명령 + 회전: 두 바퀴 반대로 같은 크기 (제자리 회전). 주행 중에는 꺼진다.
    spin_in_place = RewTerm(func=custom_rewards.spin_in_place_exp, weight=1.0,
                            params={"command_name": "base_velocity",
                                    "asset_cfg": SceneEntityCfg("robot", joint_names=["l_wheel_joint", "r_wheel_joint"],
                                                                preserve_order=True)})
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


# ---------------------------------------------------------------------------
# CAD 4절링크 폐루프 모델 (cad.py). 정책 입출력은 위와 똑같다 — 기존 정책을 올리고 이어서 학습할 수 있다.
# ---------------------------------------------------------------------------
from . import cad  # noqa: E402


@configclass
class WheeledBipedCADEnvCfg(WheeledBipedBalanceEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = cad.CAD_ROBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.actions.legs = cad.CADLegActionCfg(asset_name="robot", joint_names=cad.LEG_JOINTS,
                                                preserve_order=True, command_name="base_velocity")
        self.actions.wheels = cad.CADWheelActionCfg(asset_name="robot", joint_names=cad.WHEEL_JOINTS,
                                                    preserve_order=True, cutoff_hz=20.0, torque_scale=1.5)
        pol = self.observations.policy
        pol.joint_pos = ObsTerm(func=cad.leg_pos_rel, params={"default": LEG_MID},
                                noise=Unoise(n_min=-0.002, n_max=0.002))
        pol.joint_vel = ObsTerm(func=cad.joint_vel_like_simple, noise=Unoise(n_min=-0.5, n_max=0.5))
        rw = self.rewards
        rw.track_height.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=cad.LEG_JOINTS)
        rw.wheel_effort.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=cad.WHEEL_JOINTS)
        rw.spin_in_place.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=cad.WHEEL_JOINTS, preserve_order=True)
        rw.leg_vel.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=cad.LEG_JOINTS)
        rw.joint_limits.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=cad.LEG_JOINTS)
        # 관절 무작위 리셋은 폐루프를 깨뜨린다 (M 만 바꾸면 I, K 가 안 맞는다) -> CAD 영점에서 시작
        self.events.reset_joints = None
        self.commands.base_velocity.default_height = cad.H0_JOINT


@configclass
class WheeledBipedCADEnvCfg_PLAY(WheeledBipedCADEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 3.0
        self.observations.policy.enable_corruption = False
        self.events.push = None


# ---------------------------------------------------------------------------
# 거친 지형 + "짐벌처럼 몸통 고정" (terrain.py). CAD 모델, 정책 입출력은 그대로 (블라인드).
# 지형 높이는 보상/종료 판정에만 쓴다.
# ---------------------------------------------------------------------------
from isaaclab.managers import CurriculumTermCfg as CurrTerm  # noqa: E402

from . import terrain as rough  # noqa: E402


@configclass
class RoughSceneCfg(SceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=rough.ROUGH_TERRAINS_CFG,
        # 평지 정책에서 이어 받으므로 쉬운 3 단계(난이도 0~0.3)에서 출발
        max_init_terrain_level=2,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply", restitution_combine_mode="multiply",
            static_friction=1.0, dynamic_friction=1.0),
        debug_vis=False,
    )
    height_scanner = rough.HEIGHT_SCANNER_CFG
    critic_scanner = rough.CRITIC_SCANNER_CFG      # 크리틱 전용 (정책은 지형을 안 본다)


@configclass
class RoughObservationsCfg:
    """관측 v3 (2026-09-25): 정책 25 = 예전 20 + IMU 비력 3 + 고관절 토크 2. 새 항은 뒤에 붙여서
    이어 학습 때 첫 층에 0 열만 덧붙이면 된다 (scripts/add_obs_v3.py).
    다리 속도 잡음 0.5 -> 0.05 m/s (예전 값은 신호보다 커서 정책이 못 썼다). 바퀴 속도는 0.5 rad/s 유지."""

    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=cad.leg_pos_rel, params={"default": LEG_MID}, noise=Unoise(n_min=-0.002, n_max=0.002))
        leg_vel = ObsTerm(func=cad.leg_vel, noise=Unoise(n_min=-0.05, n_max=0.05))
        wheel_vel = ObsTerm(func=cad.wheel_vel, noise=Unoise(n_min=-0.5, n_max=0.5))
        actions = ObsTerm(func=mdp.last_action)
        imu_acc = ObsTerm(func=custom_rewards.imu_specific_force_g, noise=Unoise(n_min=-0.03, n_max=0.03))
        leg_torque = ObsTerm(func=cad.leg_torque, params={"scale": 5.0}, noise=Unoise(n_min=-0.04, n_max=0.04))

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """비대칭 크리틱: 정책 관측(잡음 없음) + 몸체 선속도 + 넓은 지형 스캔 117 점 = 145."""
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=cad.leg_pos_rel, params={"default": LEG_MID})
        leg_vel = ObsTerm(func=cad.leg_vel)
        wheel_vel = ObsTerm(func=cad.wheel_vel)
        actions = ObsTerm(func=mdp.last_action)
        imu_acc = ObsTerm(func=custom_rewards.imu_specific_force_g)
        leg_torque = ObsTerm(func=cad.leg_torque, params={"scale": 5.0})
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        height_scan = ObsTerm(func=rough.height_scan_rel, params={"sensor_cfg": SceneEntityCfg("critic_scanner")})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class RoughRewardsCfg(RewardsCfg):
    # 짐벌: 몸통 높이 = 넓은 지면 평균 + h_ref (바퀴 밑 요철은 다리가 흡수). 평지 track_height 를 대체한다.
    # 수동 모드 std 2 cm (사용자 높이에 붙인다), 자동 모드 std 5 cm (공칭 = CAD 기본자세, 느슨한 선호).
    gimbal_height = RewTerm(func=custom_rewards.gimbal_height_exp, weight=1.5,
                            params={"command_name": "base_velocity", "std": 0.02, "std_auto": 0.05,
                                    "sensor_cfg": SceneEntityCfg("height_scanner")})
    # 짐벌 본체: 몸통 수직가속도. 첫 판 std 3 m/s^2 은 너무 느슨했다 (기준선 az RMS 0.2~3 이 전부 0.6 이상 보상).
    ride = RewTerm(func=custom_rewards.base_vertical_acc_exp, weight=1.0, params={"std": 1.0})
    # 다리 권한 0.12 m 에 맞춘 다리 부드러움 (실제 이동량 기준 예전과 같게, 0.12/0.03 = 4)
    leg_rate = RewTerm(func=custom_rewards.leg_action_rate_phys, weight=-0.1, params={"scale_ratio": 4.0})
    # 정지 명령일 때 제자리 유지 (r3 에서 정지 중 밀림 수동 10 cm / 자동 4.4 cm)
    stand_still = RewTerm(func=custom_rewards.stand_still_exp, weight=1.0,
                          params={"command_name": "base_velocity", "std": 0.05})
    # 자동 모드: 다리를 행정 양끝 10 mm 안에 붙이지 않기 (요철 흡수 여유)
    stroke_margin = RewTerm(func=custom_rewards.leg_stroke_margin, weight=-1.0,
                            params={"command_name": "base_velocity", "margin": 0.010,
                                    "h_min": 0.1225, "h_max": 0.2425})
    # (몸통 수직속도 L2 는 뺐다 — 경사를 일정하게 오를 때(v x 경사)와 자동 모드 높이 변경까지 벌한다.
    #  평지의 lin_vel_z 도 끈다. 튀는 것은 아래 ride(수직가속도)가 잡는다.)


@configclass
class RoughCurriculumCfg:
    terrain_levels = CurrTerm(func=rough.terrain_levels_survival)


@configclass
class WheeledBipedCADRoughEnvCfg(WheeledBipedCADEnvCfg):
    scene: RoughSceneCfg = RoughSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: RoughObservationsCfg = RoughObservationsCfg()
    rewards: RoughRewardsCfg = RoughRewardsCfg()
    curriculum: RoughCurriculumCfg = RoughCurriculumCfg()

    def __post_init__(self):
        super().__post_init__()
        self.rewards.track_height = None
        self.rewards.lin_vel_z = None
        # CAD 부모가 넣은 joint_vel(4 칸 묶음)은 leg_vel / wheel_vel 로 나눠 대체했다
        self.observations.policy.joint_vel = None
        self.terminations.too_low = DoneTerm(func=rough.base_below_ground,
                                             params={"minimum_height": 0.12,
                                                     "sensor_cfg": SceneEntityCfg("height_scanner")})
        # 레이캐스트는 정책 주기(200 Hz)마다 한 번이면 된다
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        self.scene.critic_scanner.update_period = self.decimation * self.sim.dt
        # 높이 모드 (commands.py): 명령 = [vx, wz, h_ref, m]. 자동 공칭 = CAD 기본자세 (M = 0, 사용자 지정 IDLE)
        c = self.commands.base_velocity
        c.with_mode = True
        c.auto_mode_prob = 0.5          # 평지 타일에서. 거친 지형 타일 위에서는 항상 자동 (수동은 평지용)
        c.manual_flat_only = True
        c.auto_height = cad.H0_JOINT
        # 다리 권한: 어떤 h_ref 에서든 최소·최대 양끝까지 (사용자). 0.12 m = 전체 행정 0.1225~0.2425.
        # 예전 0.03 에서 이어받을 때는 add_mode_input.py 가 다리 출력을 1/4 로 환산한다.
        self.actions.legs.leg_scale = 0.12


@configclass
class WheeledBipedCADRoughEnvCfg_PLAY(WheeledBipedCADRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.observations.policy.enable_corruption = False
        self.events.push = None
        self.curriculum.terrain_levels = None
        self.scene.terrain.max_init_terrain_level = None


# ---------------------------------------------------------------------------
# r6: 점프 (조종자 버튼) + 대회 장애물 (2 단 계단, 엇갈린 삼각 턱) — 2026-09-26
# 명령 = [vx, wz, h_ref, m, j] (관측 26). 고관절은 토크-속도 모델(DC). 처음부터 학습.
# ---------------------------------------------------------------------------
@configclass
class JumpSceneCfg(RoughSceneCfg):
    l_wheel_scanner = rough.L_WHEEL_SCANNER_CFG
    r_wheel_scanner = rough.R_WHEEL_SCANNER_CFG


@configclass
class JumpRewardsCfg(RoughRewardsCfg):
    # 버튼 후 0.7 s 동안 바퀴가 바로 아래 지면에서 뜬 높이 (목표 10 cm). 8 cm 턱을 넘으려면 이만큼 떠야 한다.
    jump_clear = RewTerm(func=custom_rewards.jump_clearance, weight=4.0,
                         params={"command_name": "base_velocity", "target": 0.10})
    # 버튼 없이 뛰기 금지
    air_no_jump = RewTerm(func=custom_rewards.airtime_outside_jump, weight=-1.0,
                          params={"command_name": "base_velocity"})


@configclass
class WheeledBipedCADJumpEnvCfg(WheeledBipedCADRoughEnvCfg):
    scene: JumpSceneCfg = JumpSceneCfg(num_envs=4096, env_spacing=2.5)
    rewards: JumpRewardsCfg = JumpRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain.terrain_generator = rough.ROUGH_JUMP_TERRAINS_CFG
        self.scene.robot = cad.CAD_ROBOT_CFG_DCHIP.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.commands.base_velocity.with_jump = True
        # 점프 중에는 몸통 높이 유지·승차감·정지 유지·다리 부드러움을 끈다 (뛰려면 다리를 빠르게 펴야 한다)
        for name in ("gimbal_height", "ride", "stand_still", "leg_rate"):
            getattr(self.rewards, name).params["jump_cmd"] = "base_velocity"
        for sc in (self.scene.l_wheel_scanner, self.scene.r_wheel_scanner):
            sc.update_period = self.decimation * self.sim.dt


@configclass
class WheeledBipedCADJumpEnvCfg_PLAY(WheeledBipedCADJumpEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.observations.policy.enable_corruption = False
        self.events.push = None
        self.curriculum.terrain_levels = None
        self.scene.terrain.max_init_terrain_level = None


@configclass
class JumpRewardsCfgV7(JumpRewardsCfg):
    jump_takeoff = RewTerm(func=custom_rewards.jump_takeoff, weight=3.0,
                           params={"command_name": "base_velocity", "t_max": 0.3, "v_target": 1.2})


@configclass
class WheeledBipedCADJumpEnvCfgV7(WheeledBipedCADJumpEnvCfg):
    """r7: r6 + 이륙 유도 보상, 점프 높이 가중치 4 -> 6."""
    rewards: JumpRewardsCfgV7 = JumpRewardsCfgV7()

    def __post_init__(self):
        super().__post_init__()
        self.rewards.jump_clear.weight = 6.0


@configclass
class WheeledBipedCADJumpEnvCfgV7_PLAY(WheeledBipedCADJumpEnvCfgV7):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.observations.policy.enable_corruption = False
        self.events.push = None
        self.curriculum.terrain_levels = None
        self.scene.terrain.max_init_terrain_level = None


# ---------------------------------------------------------------------------
# j1: 점프 전용 정책 (JUMP_REFERENCES.md 의 B). 3 s 짧은 에피소드: 달리다(0.3~0.8 m/s) 턱 앞 무작위 거리에서 버튼 ->
# 점프 -> 착지 -> 안정. 2 단 계단 구덩이(가운데 6.9 m) 의 +x 벽을 향해 2.8~3.1 m 지점(벽까지 약 0.25~0.6 m)에서 출발.
# 평지 타일에서는 에피소드마다 한 번 무작위 버튼. 관측은 r6/r7 과 같은 26 -> r7 에서 이어 시작.
# ---------------------------------------------------------------------------
@configclass
class JumpSkillRewardsCfg(JumpRewardsCfgV7):
    jump_ref = RewTerm(func=custom_rewards.jump_leg_ref, weight=3.0, params={"command_name": "base_velocity", "sigma": 0.02})


@configclass
class WheeledBipedCADJumpSkillEnvCfg(WheeledBipedCADJumpEnvCfgV7):
    rewards: JumpSkillRewardsCfg = JumpSkillRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 3.0
        gen = rough.ROUGH_JUMP_TERRAINS_CFG
        self.scene.terrain.terrain_generator = gen.replace(
            num_cols=10, sub_terrains={"flat": gen.sub_terrains["flat"].replace(proportion=0.3),
                                       "stairs2_up": gen.sub_terrains["stairs2_up"].replace(proportion=0.7)})
        self.scene.terrain.max_init_terrain_level = 3
        self.events.reset_base.params["pose_range"] = {"x": (2.77, 3.12), "y": (-1.0, 1.0), "yaw": (-0.1, 0.1)}
        self.events.reset_base.params["velocity_range"] = {"x": (0.3, 0.8)}
        c = self.commands.base_velocity
        c.resampling_time_range = (100.0, 100.0)          # 에피소드 동안 명령 고정
        c.ranges.lin_vel_x = (0.3, 0.8)
        c.ranges.ang_vel_z = (0.0, 0.0)
        c.ranges.height = (0.18, 0.18)
        c.zero_vel_prob = 0.0
        c.pure_axis_prob = 0.0
        c.fast_turn_prob = 0.0
        c.auto_mode_prob = 0.0
        c.jump_rand_rate = 0.3                            # 평지 3 s 안 59 %, 계단 벽 전 조기 발동 약 16 % (나쁜 타이밍 표본)
        c.jump_cooldown_s = 5.0                           # 에피소드당 한 번
        self.events.push = None


@configclass
class WheeledBipedCADJumpSkillEnvCfg_PLAY(WheeledBipedCADJumpSkillEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.observations.policy.enable_corruption = False
        self.curriculum.terrain_levels = None
        self.scene.terrain.max_init_terrain_level = None
