"""관측 v2 (정책 20, 크리틱 = 정책) 체크포인트 -> 관측 v3 (정책 25, 크리틱 145). 동작은 그대로 이어받는다.

v3 (tasks/balance/env_cfg.py RoughObservationsCfg)
  정책 25  = v2 20 (같은 순서) + IMU 비력 3 [g] + 고관절 토크 2 [/5 Nm]
  크리틱 145 = 정책 25 (잡음 없음) + 몸체 선속도 3 + 지형 스캔 117  (비대칭 액터-크리틱)

수술
  1. actor·critic 첫 층에 새 입력 수만큼 0 열을 **뒤에** 붙인다 -> 새 입력은 처음에 출력에 영향 없음.
  2. 정규화기: 새 칸은 사전값(평균, std)으로 채우고, count 를 1e6 으로 낮춰 새 통계가 몇 iter 안에 자리 잡게 한다
     (예전 count 그대로면 수억이라 새 칸 통계가 거의 안 움직인다).
  3. Adam 모멘트: 첫 층 두 개(옵티마이저 인덱스 1 actor, 9 critic)에 0 열을 붙여 보존한다.

    isaaclab.sh -p add_obs_v3.py <in model_N.pt> <out model_N.pt>
"""
import sys

import torch

src, dst = sys.argv[1], sys.argv[2]
ck = torch.load(src, map_location="cpu", weights_only=False)

# 새 칸 사전값 (평균, std)
ACT_NEW = [(0.0, 0.1), (0.0, 0.1), (1.0, 0.1),      # IMU 비력 x, y, z [g] — 가만히 있으면 (0, 0, 1)
           (0.0, 0.5), (0.0, 0.5)]                    # 고관절 토크 / 5 Nm
CRI_NEW = ACT_NEW + [(0.0, 0.5)] * 3 + [(0.0, 0.1)] * 117   # + 선속도 + 지형 스캔 (COM - 지면 - 0.25 m)
COUNT = 1.0e6


def extend(sd, new):
    n_old = sd["mlp.0.weight"].shape[1]
    assert n_old == 20, f"v2 체크포인트가 아니다 (입력 {n_old})"
    k = len(new)
    sd["mlp.0.weight"] = torch.cat([sd["mlp.0.weight"], torch.zeros(sd["mlp.0.weight"].shape[0], k)], dim=1)
    m = torch.tensor([[a for a, _ in new]]); s = torch.tensor([[b for _, b in new]])
    sd["obs_normalizer._mean"] = torch.cat([sd["obs_normalizer._mean"], m], dim=1)
    sd["obs_normalizer._std"] = torch.cat([sd["obs_normalizer._std"], s], dim=1)
    sd["obs_normalizer._var"] = torch.cat([sd["obs_normalizer._var"], s * s], dim=1)
    sd["obs_normalizer.count"] = torch.tensor(COUNT, dtype=sd["obs_normalizer.count"].dtype)
    return k


ka = extend(ck["actor_state_dict"], ACT_NEW)
kc = extend(ck["critic_state_dict"], CRI_NEW)
st = ck["optimizer_state_dict"]["state"]
for i, k in ((1, ka), (9, kc)):
    for key in ("exp_avg", "exp_avg_sq"):
        t = st[i][key]
        st[i][key] = torch.cat([t, torch.zeros(t.shape[0], k)], dim=1)
torch.save(ck, dst)
print(f"저장 {dst}: 정책 20 -> {20 + ka}, 크리틱 20 -> {20 + kc}, 정규화 count -> {COUNT:.0e}")
