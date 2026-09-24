"""균형 + 속도 추종 과제 등록.

설정 클래스를 여기서 import 하지 않고 문자열 경로로 등록한다.
이 소스 빌드 Isaac Sim 은 pxr(USD)을 Kit 런타임에서 제공해서
SimulationApp 이 뜨기 전에는 import 할 수 없다. env_cfg 를 여기서
직접 import 하면 isaaclab.sim -> pxr 로 이어져 앱 시작 전에 터진다.
IsaacLab 기본 태스크들도 같은 이유로 전부 문자열을 쓴다.
"""

import gymnasium as gym

gym.register(
    id="Isaac-WheeledBiped-Balance-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:WheeledBipedBalanceEnvCfg",
        "rsl_rl_cfg_entry_point": "wheeled_biped_isaaclab.agents.rsl_rl_ppo_cfg:BalancePPORunnerCfg",
    },
)

gym.register(
    id="Isaac-WheeledBiped-Balance-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:WheeledBipedBalanceEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": "wheeled_biped_isaaclab.agents.rsl_rl_ppo_cfg:BalancePPORunnerCfg",
    },
)
