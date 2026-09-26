"""rsl_rl PPO 설정.

이 체크아웃의 실제 API 를 확인하고 맞췄다 (isaaclab_rl 0.5.1,
IsaacLab 2.3.2). rl_cfg.py 에는 새 API 인 RslRlMLPModelCfg(actor/critic)와
구 API 인 RslRlPpoActorCriticCfg(policy)가 둘 다 있고,
IsaacLab 이 함께 배포하는 예제(go1/h1)는 policy 쪽을 쓴다.
배포 예제와 같은 형태를 따라간다 — 이 저장소가 검증한 조합이다.

행동 차원이 4개(다리 2, 바퀴 2)뿐이라 망을 크게 잡을 이유가 없다.
[128,128,128] 은 같은 규모 과제의 통상 크기이고, 실제 학습 곡선을 보고
조정하면 된다.
"""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class BalancePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    # --- 시간 규모 환산 (2026-09-24) ---
    # 이 환경은 정책이 200 Hz 로 돈다. IsaacLab 기본 하이퍼파라미터는 50 Hz
    # 기준이라 그대로 쓰면 시야가 4배 짧아진다. 실제로 세 번의 학습이
    # 전부 iter 100 근처에서 정점(226 / 722 / 846 step)을 찍고 무너졌다.
    # 아래 값은 전부 "50 Hz 기준 시간"을 200 Hz 로 환산한 것이다.
    #
    #   롤아웃 길이   24 step @50Hz = 0.48 s  ->  96 step @200Hz
    #   gamma 시야    100 step @50Hz = 2.0 s  ->  gamma 0.9975
    #   GAE 시야      17 step @50Hz = 0.34 s  ->  lam 0.9875
    num_steps_per_env = 96
    max_iterations = 800
    save_interval = 50
    experiment_name = "wheeled_biped_balance"
    empirical_normalization = False
    # 정책 출력을 ±1 로 자른다. IsaacLab 기본값은 None(자르지 않음)이라, 이게 없으면
    # 액션 스케일(바퀴 1.5 Nm, 다리 ±5 cm)이 한계 역할을 못 한다. 실제로 2026-09-24 학습은
    # 원시 액션이 최대 8.85 까지 나와 바퀴는 모터 한계 7 Nm 로 뱅뱅(약 10 Hz, 속도한계 18.8 rad/s),
    # 다리는 하한(122.5 mm)에 99.5 % 붙어 있었다 (scripts/play_log.py 로그).
    # 자르기는 액션 항 안(tasks/balance/actions.py)에서 한다. 래퍼에서 자르면 ±1 밖으로
    # 밀려난 정책 평균을 벌할 수 없어 뱅뱅 제어가 된다 (2026-09-25 원시 출력 최대 194).
    clip_actions = None

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_obs_normalization=True,
        critic_obs_normalization=True,
        actor_hidden_dims=[128, 128, 128],
        critic_hidden_dims=[128, 128, 128],
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        # 0.005 -> action std 가 iter 200 에 0.23 으로 붕괴(탐색 사망).
        # 0.02  -> 반대로 std 가 1.87 까지 자라 정책이 제 노이즈에 익사.
        #          (STAGE1 1500 iter 결과: 길이 715 step 에서 완전 정체)
        # PD 검사가 쓴 토크가 0.85 Nm 뿐이라 노이즈 여유가 거의 없다.
        entropy_coef=0.006,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.9975,   # 1 - dt/2.0s, dt = 1/200
        lam=0.9875,     # gamma*lam -> GAE 시야 0.34 s
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class RoughPPORunnerCfg(BalancePPORunnerCfg):
    """거친 지형 + 높이 모드 과제. 엔트로피 0.006 -> 0.002.

    2026-09-25 첫 판(0.006, Adam 모멘트 비움)에서 새 지형으로 넘어가자 행동 std 가 150 iter 동안
    전 차원에서 단조 증가(평균 0.31 -> 1.00)하고 완주율이 64 % -> 39 % 로 떨어졌다. 크리틱이 새 보상·지형에
    맞춰지기 전에는 추종 기울기가 잡음이라, 방향이 일정한 엔트로피 기울기만 Adam 에서 누적된다.
    """

    def __post_init__(self):
        super().__post_init__()
        # 0.002 로도 5050 -> 5100 사이 다리 std 0.25 -> 0.33 (실제 3 -> 4 cm) 로 다시 가팔라져서 0.001 (r3).
        # 그런데 0.001 로 이어 학습한 r3 는 std 0.19 까지 줄고, 작은 std 에서는 같은 평균 변화에도 KL 이 커서
        # adaptive 스케줄이 학습률을 하한 1e-5 까지 깎았다 (r3 끝, train_health 경고). r4 부터는 오리처럼
        # **처음부터** 학습하므로 평지에서 검증된 0.006 으로 되돌린다. 이어서 학습 시험은 --from 과 함께
        # `agent.algorithm.entropy_coef=0.001` 을 준다.
        self.algorithm.entropy_coef = 0.006


@configclass
class ResidualPPORunnerCfg(BalancePPORunnerCfg):
    """잔차 RL (LQR+VMC 위 보정). 보정 0 에서 시작해야 기본 제어기를 망치지 않는다 -> 초기 잡음 작게 (0.3).
    학습률은 고정 5e-4 (adaptive 가 1e-2 ~ 1e-5 를 오가며 퇴행한 교훈, r4/r5)."""

    def __post_init__(self):
        super().__post_init__()
        self.experiment_name = "wheeled_biped_residual"
        self.num_steps_per_env = 48
        self.max_iterations = 3000
        self.save_interval = 100
        self.policy.init_noise_std = 0.3
        self.policy.actor_hidden_dims = [128, 128]
        self.policy.critic_hidden_dims = [256, 256, 128]
        self.algorithm.schedule = "fixed"
        self.algorithm.learning_rate = 5.0e-4
        self.algorithm.entropy_coef = 0.002
