"""거친 지형 학습 (2026-09-25 사용자: "울퉁불퉁, 웨이브 등 다양하게, ROBUST 하게").

목표 (사용자): **몸통이 제자리에 고정 — 짐벌처럼.** 바퀴가 요철을 타고 오르내려도 다리가 늘었다
줄었다 해서 몸통 높이와 좌우 수평은 그대로 두는 능동 서스펜션.

정책은 지형을 보지 않는다 (블라인드). 관측은 평지 과제와 똑같이 IMU + 관절뿐이라 실기로 그대로 옮겨진다.
지형 높이(레이캐스트)는 **보상과 종료 판정에만** 쓴다.

지형 종류 (행 = 난이도 0 -> 1, 열 = 종류):
  flat        평지 — 평지 성능을 잊지 않게, 수동 높이 모드는 여기서만
  gravel      자갈길: 10 cm 격자 무작위 높이 ±(1.0 -> 3.0) cm, 평활 없음 (사용자 목표 1 -> 3 cm)
  rugged      험지: 2 m 기복(0 -> 6 cm) + 40 cm 굵은 요철 ±(0.5 -> 3) cm + 10 cm 자갈 ±(0.5 -> 1.5) cm
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

import numpy as np

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


# --- 험지: 긴 기복 + 굵은 요철 + 자갈 요철을 겹친다 (2026-09-25 사용자: "자갈길이나 험지") ---------------------
from isaaclab.terrains.height_field.utils import height_field_to_mesh  # noqa: E402
from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg  # noqa: E402

_uniform_raw = hf_terrains.random_uniform_terrain.__wrapped__      # 메시 변환 전 높이 배열을 주는 원함수
_wave_raw = hf_terrains.wave_terrain.__wrapped__


@height_field_to_mesh
def rugged_terrain(difficulty: float, cfg: "RuggedTerrainCfg"):
    d = difficulty

    def lerp(r):
        return r[0] + d * (r[1] - r[0])

    w = copy.copy(cfg)
    w.amplitude_range = (lerp(cfg.wave_amp), lerp(cfg.wave_amp))
    w.num_waves = cfg.num_waves
    hf = _wave_raw(d, w).astype(np.int32)
    for amp, ds in ((cfg.coarse_amp, cfg.coarse_scale), (cfg.fine_amp, cfg.fine_scale)):
        u = copy.copy(cfg)
        a = max(lerp(amp), cfg.noise_step)
        u.noise_range, u.downsampled_scale = (-a, a), ds
        hf += _uniform_raw(d, u).astype(np.int32)
    return np.clip(hf, -32000, 32000).astype(np.int16)


@configclass
class RuggedTerrainCfg(HfTerrainBaseCfg):
    function = rugged_terrain
    wave_amp: tuple[float, float] = (0.0, 0.06)      # 파장 2 m 기복 (성분 진폭의 2 배, wave_terrain 정의)
    num_waves: int = 4
    coarse_amp: tuple[float, float] = (0.005, 0.030)  # 40 cm 격자 굵은 요철 ±
    coarse_scale: float = 0.4
    fine_amp: tuple[float, float] = (0.005, 0.015)    # 10 cm 격자 자갈 ± (2026-09-25 0.2~1.2 -> 0.5~1.5 cm)
    fine_scale: float = 0.1
    noise_step: float = 0.002


_HF = dict(border_width=0.25)

ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    seed=42,
    curriculum=True,
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.002,       # 기본 5 mm 는 파도가 5 mm 계단으로 깎인다
    slope_threshold=None,       # 가파른 칸을 수직벽으로 바꾸지 않는다 (위 bumps 설명)
    use_cache=False,
    sub_terrains={
        # 평지: 수동 높이 모드는 평지에서만 학습한다 (사용자: "수동은 평지용")
        "flat": MeshPlaneTerrainCfg(proportion=0.15),
        # 자갈길: 10 cm 격자 무작위 높이, 평활 없음. 2026-09-25 사용자: "요철이 너무 낮다, 1 cm -> 3 cm 까지 목표"
        # ±(0.4 -> 2.0) cm 에서 ±(1.0 -> 3.0) cm 로.
        "gravel": RoughScaledTerrainCfg(proportion=0.15, noise_amp=(0.010, 0.030), noise_step=0.002,
                                        downsampled_scale=0.1, **_HF),
        # 험지: 2 m 기복 + 40 cm 굵은 요철 + 10 cm 자갈을 겹침
        "rugged": RuggedTerrainCfg(proportion=0.15, **_HF),
        # 2026-09-25 첫 판(요철 ±2 cm, 파도 6/3 cm, 경사 0.25, 턱 3 cm)은 평지 정책이 84/84 버텼다
        # (rough_probe, 바퀴가 겪는 요철 σ 최대 6 mm) -> 강건성 학습이 안 된다. 키웠다.
        # (파도 amplitude 는 IsaacLab 정의상 성분 진폭의 2 배: h = amp/2 (cos y + sin x))
        "rough": RoughScaledTerrainCfg(proportion=0.12, noise_amp=(0.005, 0.035), noise_step=0.002,
                                       downsampled_scale=0.25, **_HF),
        "wave_long": HfWaveTerrainCfg(proportion=0.08, amplitude_range=(0.0, 0.10), num_waves=4, **_HF),
        "wave_short": HfWaveTerrainCfg(proportion=0.09, amplitude_range=(0.0, 0.06), num_waves=8, **_HF),
        "slope_up": HfPyramidSlopedTerrainCfg(proportion=0.07, slope_range=(0.0, 0.30), platform_width=2.0, **_HF),
        "slope_down": HfInvertedPyramidSlopedTerrainCfg(proportion=0.07, slope_range=(0.0, 0.30),
                                                        platform_width=2.0, **_HF),
        "bumps": HfDiscreteObstaclesTerrainCfg(proportion=0.12, obstacle_height_mode="choice",
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


# --- 지금 서 있는 타일이 평지인가 (수동 높이 모드는 평지 전용 — 사용자 2026-09-25) --------------------------
_TILE_CACHE = {}


def on_flat_tile(env: "ManagerBasedRLEnv", env_ids) -> torch.Tensor:
    """env_ids 로봇이 지금 평지 타일 위에 있는지. 평면 지형(plane)이면 전부 True.

    가장 가까운 타일 원점으로 열을 찾고, 열 -> 지형 종류는 TerrainGenerator 의 비율 규칙과 같다.
    """
    terrain = env.scene.terrain
    gen = getattr(terrain.cfg, "terrain_generator", None)
    if gen is None or terrain.cfg.terrain_type != "generator":
        return torch.ones(len(env_ids), dtype=torch.bool, device=env.device)
    key = id(terrain)
    if key not in _TILE_CACHE:
        import numpy as np
        names = list(gen.sub_terrains.keys())
        prop = np.array([gen.sub_terrains[n].proportion for n in names]); prop = prop / prop.sum()
        col_type = [int(np.min(np.where(c / gen.num_cols + 0.001 < np.cumsum(prop))[0])) for c in range(gen.num_cols)]
        flat_idx = names.index("flat") if "flat" in names else -1
        flat_col = torch.tensor([t == flat_idx for t in col_type], device=env.device)
        org = terrain.terrain_origins[..., :2].reshape(-1, 2)              # (rows*cols, 2)
        _TILE_CACHE[key] = (org, flat_col, gen.num_cols)
    org, flat_col, ncol = _TILE_CACHE[key]
    xy = env.scene["robot"].data.root_pos_w[env_ids, :2]
    idx = torch.cdist(xy, org).argmin(dim=1)
    return flat_col[idx % ncol]


# --- 크리틱 전용 (비대칭 액터-크리틱): 넓은 지형 스캔 ------------------------------------------------------
# 정책은 여전히 지형을 안 본다 (실기 이식). 크리틱만 1.2 x 0.8 m, 10 cm 간격 117 점을 본다.
CRITIC_SCANNER_CFG = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link",
    offset=RayCasterCfg.OffsetCfg(pos=(0.08, -0.08, 20.0)),
    ray_alignment="yaw",
    pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.2, 0.8]),
    debug_vis=False,
    mesh_prim_paths=["/World/ground"],
)


def height_scan_rel(env: "ManagerBasedRLEnv", sensor_cfg: SceneEntityCfg, offset: float = 0.25,
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """몸체 COM 높이 - 지면 높이 - offset, ±0.5 m 로 자름. 빗나간 광선은 0."""
    asset: Articulation = env.scene[asset_cfg.name]
    z = env.scene.sensors[sensor_cfg.name].data.ray_hits_w[..., 2]
    rel = asset.data.root_com_pos_w[:, 2:3] - z - offset
    return torch.nan_to_num(rel, nan=0.0, posinf=0.0, neginf=0.0).clamp(-0.5, 0.5)
