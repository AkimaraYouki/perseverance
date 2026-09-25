"""거친 지형 학습 (2026-09-25 사용자: "울퉁불퉁, 웨이브 등 다양하게, ROBUST 하게").

목표 (사용자): **몸통이 제자리에 고정 — 짐벌처럼.** 바퀴가 요철을 타고 오르내려도 다리가 늘었다
줄었다 해서 몸통 높이와 좌우 수평은 그대로 두는 능동 서스펜션.

정책은 지형을 보지 않는다 (블라인드). 관측은 평지 과제와 똑같이 IMU + 관절뿐이라 실기로 그대로 옮겨진다.
지형 높이(레이캐스트)는 **보상과 종료 판정에만** 쓴다.

지형 종류 (행 = 난이도 0 -> 1, 열 = 종류):
  flat        평지 — 평지 성능을 잊지 않게
  rough       무작위 요철. 25 cm 격자에서 높이 ±(0.5 -> 3.5) cm
  wave_long   파장 2 m, x·y 성분 각각 ±(0 -> 5) cm (겹치면 피크-피크 20 cm), 경사 최대 9 deg
  wave_short  파장 1 m, 성분 ±(0 -> 3) cm, 경사 최대 10.7 deg
  slope_up / slope_down   피라미드 경사 0 -> 16.7 deg (tan 0.3, 오르막 / 내리막)
  bumps       턱과 파인 곳 1 -> 4 cm. 수직 벽 대신 10 cm 에 걸친 경사로 (바퀴 R 60 mm 가 마찰로
              넘을 수 있는 수직 턱은 약 1 cm 뿐이다: 필요 수평력/하중 = sqrt(2Rh-h^2)/(R-h), mu 0.65 에서 h = 1 cm)

계단(100 mm)은 여기 넣지 않았다. 바퀴 반지름보다 높아서 굴러서는 못 오른다 — 따로 다룬다.
"""

from __future__ import annotations

import copy
from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import RayCasterCfg, patterns
from isaaclab.terrains import TerrainGeneratorCfg
from isaaclab.terrains.height_field import hf_terrains
from isaaclab.terrains.height_field.hf_terrains_cfg import (
    HfDiscreteObstaclesTerrainCfg,
    HfInvertedPyramidSlopedTerrainCfg,
    HfPyramidSlopedTerrainCfg,
    HfRandomUniformTerrainCfg,
    HfWaveTerrainCfg,
)
from isaaclab.terrains.trimesh.mesh_terrains_cfg import MeshPlaneTerrainCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

R_WHEEL = 0.060


# --- 난이도에 따라 커지는 무작위 요철 -----------------------------------------------------------
# IsaacLab 의 random_uniform_terrain 은 difficulty 를 무시한다 (진폭 고정). 커리큘럼이 먹히게 감싼다.
def rough_scaled_terrain(difficulty: float, cfg: "RoughScaledTerrainCfg"):
    a = cfg.noise_amp[0] + difficulty * (cfg.noise_amp[1] - cfg.noise_amp[0])
    a = max(a, cfg.noise_step)
    c = copy.copy(cfg)
    c.noise_range = (-a, a)
    return hf_terrains.random_uniform_terrain(difficulty, c)


@configclass
class RoughScaledTerrainCfg(HfRandomUniformTerrainCfg):
    function = rough_scaled_terrain
    noise_range: tuple[float, float] = (0.0, 0.0)     # rough_scaled_terrain 이 채운다
    noise_amp: tuple[float, float] = (0.003, 0.02)


_HF = dict(border_width=0.25)

ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    seed=42,
    curriculum=True,
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=16,
    horizontal_scale=0.1,
    vertical_scale=0.002,       # 기본 5 mm 는 파도가 5 mm 계단으로 깎인다
    slope_threshold=None,       # 가파른 칸을 수직벽으로 바꾸지 않는다 (위 bumps 설명)
    use_cache=False,
    sub_terrains={
        "flat": MeshPlaneTerrainCfg(proportion=0.12),
        # 2026-09-25 첫 판(요철 ±2 cm, 파도 6/3 cm, 경사 0.25, 턱 3 cm)은 평지 정책이 84/84 버텼다
        # (rough_probe, 바퀴가 겪는 요철 σ 최대 6 mm) -> 강건성 학습이 안 된다. 키웠다.
        # (파도 amplitude 는 IsaacLab 정의상 성분 진폭의 2 배: h = amp/2 (cos y + sin x))
        "rough": RoughScaledTerrainCfg(proportion=0.22, noise_amp=(0.005, 0.035), noise_step=0.002,
                                       downsampled_scale=0.25, **_HF),
        "wave_long": HfWaveTerrainCfg(proportion=0.14, amplitude_range=(0.0, 0.10), num_waves=4, **_HF),
        "wave_short": HfWaveTerrainCfg(proportion=0.14, amplitude_range=(0.0, 0.06), num_waves=8, **_HF),
        "slope_up": HfPyramidSlopedTerrainCfg(proportion=0.10, slope_range=(0.0, 0.30), platform_width=2.0, **_HF),
        "slope_down": HfInvertedPyramidSlopedTerrainCfg(proportion=0.10, slope_range=(0.0, 0.30),
                                                        platform_width=2.0, **_HF),
        "bumps": HfDiscreteObstaclesTerrainCfg(proportion=0.18, obstacle_height_mode="choice",
                                               obstacle_height_range=(0.01, 0.04), obstacle_width_range=(0.3, 1.0),
                                               num_obstacles=40, platform_width=1.0, **_HF),
    },
)

# 몸통 아래 지면 평균 높이. 고관절 중점 중심 60 x 40 cm, 10 cm 간격 35 점.
# CAD 몸체 원점 기준 고관절 중점 = (0.08, -0.08) m (export 5 URDF: L_joint_M y +0.001, R_joint_M y -0.161).
# 이 넓이가 "짐벌"의 공간 저역통과 폭이다: 이보다 짧은 요철은 다리가 흡수하고, 긴 경사·파도는 몸이 따라간다.
HEIGHT_SCANNER_CFG = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link",
    offset=RayCasterCfg.OffsetCfg(pos=(0.08, -0.08, 20.0)),
    ray_alignment="yaw",
    pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[0.6, 0.4]),
    debug_vis=False,
    mesh_prim_paths=["/World/ground"],
)


# --- 지형 높이 도우미 ----------------------------------------------------------------------------
def ground_patch_mean(env: "ManagerBasedRLEnv", sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """몸통 아래 지면 평균 높이 (월드 z). 빗나간 광선(inf)은 뺀다."""
    z = env.scene.sensors[sensor_cfg.name].data.ray_hits_w[..., 2]
    ok = torch.isfinite(z)
    return torch.where(ok, z, 0.0).sum(dim=1) / ok.sum(dim=1).clamp(min=1)


def wheel_ground_mean(asset: Articulation) -> torch.Tensor:
    """두 바퀴 접지점 높이 평균 (= 바퀴 중심 z - R)."""
    ids = asset.find_bodies(["l_wheel", "r_wheel"], preserve_order=True)[0]
    return asset.data.body_pos_w[:, ids, 2].mean(dim=1) - R_WHEEL


# --- 커리큘럼 ------------------------------------------------------------------------------------
def terrain_levels_survival(env: "ManagerBasedRLEnv", env_ids, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """20 s 를 버티면 한 단계 험한 곳으로, 넘어지면 한 단계 쉬운 곳으로.

    IsaacLab 기본(terrain_levels_vel)은 "이동 거리"로 판정한다. 이 로봇은 정지·제자리 회전 명령이
    섞여 있어 거리로는 못 가른다.
    """
    terrain = env.scene.terrain
    up = env.termination_manager.time_outs[env_ids]
    down = env.termination_manager.terminated[env_ids]
    terrain.update_env_origins(env_ids, up, down)
    return torch.mean(terrain.terrain_levels.float())


# --- 종료 ----------------------------------------------------------------------------------------
def base_below_ground(env: "ManagerBasedRLEnv", minimum_height: float, sensor_cfg: SceneEntityCfg,
                      asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """몸체가 (그 자리 지면 기준으로) 너무 낮다 — 평지 과제의 root z < 0.12 를 지형 기준으로."""
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.root_pos_w[:, 2] - ground_patch_mean(env, sensor_cfg) < minimum_height
