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
