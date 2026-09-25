"""평지 정책 체크포인트 -> 높이 모드 과제(CAD-Rough) 체크포인트. 동작은 그대로 이어받는다.

바뀌는 것 (tasks/balance/env_cfg.py WheeledBipedCADRoughEnvCfg)
  1. 관측 19 -> 20: 명령 [vx, wz, h_ref] 뒤(인덱스 9)에 모드 m 이 끼어든다.
       첫 층(actor·critic) 에 0 열 삽입 -> m 이 처음에는 출력에 영향 없음.
       정규화기 m 자리: 평균 0.5, std 0.49 -> 정규화값 ±1.
  2. 다리 행동 스케일 0.03 -> 0.12 m (최소·최대 양끝까지 닿게, 사용자 요청):
       같은 다리 목표를 내려면 a_new = a_old / 4
       -> actor 출력층 다리 행(0, 1)과 행동 std 를 1/4 로.
       관측 속 "직전 행동" 다리 두 칸(새 인덱스 16, 17)도 값이 1/4 이 되므로 정규화가 같은 값을 내도록
       mean' = mean/4,  std' + eps = (std + eps)/4   (rsl_rl: (x - mean) / (std + eps), eps = 0.01)
  3. Adam 모멘트는 비운다 (첫 층 모양이 바뀌었다).

    isaaclab.sh -p add_mode_input.py <in model_N.pt> <out model_N.pt>
"""
import sys

import torch

K = 0.25            # 0.03 / 0.12
EPS = 1e-2
INS = 9             # 모드 삽입 위치 (명령 3 칸 뒤)
LEG_ACT_OBS = (16, 17)   # 삽입 후 "직전 행동" 다리 두 칸

src, dst = sys.argv[1], sys.argv[2]
ck = torch.load(src, map_location="cpu", weights_only=False)


def insert_col(w, idx, val=0.0):
    return torch.cat([w[:, :idx], torch.full((w.shape[0], 1), val, dtype=w.dtype), w[:, idx:]], dim=1)


for name in ("actor_state_dict", "critic_state_dict"):
    sd = ck[name]
    assert sd["mlp.0.weight"].shape[1] == 19, "이미 변환된 체크포인트?"
    sd["mlp.0.weight"] = insert_col(sd["mlp.0.weight"], INS, 0.0)
    std_m = 0.49
    sd["obs_normalizer._mean"] = insert_col(sd["obs_normalizer._mean"], INS, 0.5)
    sd["obs_normalizer._std"] = insert_col(sd["obs_normalizer._std"], INS, std_m)
    sd["obs_normalizer._var"] = insert_col(sd["obs_normalizer._var"], INS, std_m**2)
    for j in LEG_ACT_OBS:
        m, s = sd["obs_normalizer._mean"][0, j].item(), sd["obs_normalizer._std"][0, j].item()
        s_new = (s + EPS) * K - EPS
        assert s_new > 0, (j, s)
        sd["obs_normalizer._mean"][0, j] = m * K
        sd["obs_normalizer._std"][0, j] = s_new
        sd["obs_normalizer._var"][0, j] = s_new**2

a = ck["actor_state_dict"]
a["mlp.6.weight"][:2] *= K
a["mlp.6.bias"][:2] *= K
a["distribution.std_param"][:2] *= K
ck["optimizer_state_dict"]["state"] = {}
torch.save(ck, dst)
print(f"저장 {dst}: 관측 19->20 (모드 @{INS}), 다리 행동 x{K}, 다리 std {a['distribution.std_param'][:2].tolist()}")
