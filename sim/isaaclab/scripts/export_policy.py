"""rsl_rl 체크포인트 -> 실기용 정책 파일 (TorchScript .pt + ONNX). Isaac Sim 을 띄우지 않는다.

정규화기((o - mean) / (std + 0.01))를 망 앞에 붙여서 내보낸다 -> 실기에서는 원시 관측 그대로 넣으면 된다.
출력은 결정적 행동(가우시안 평균), 자르기(±1)는 하지 않는다 — 실기 제어기가 시뮬 행동 항처럼 자른다.

    isaaclab.sh -p export_policy.py <model_N.pt> <출력 디렉터리>
"""
import json
import os
import sys

import torch


class Policy(torch.nn.Module):
    def __init__(self, sd):
        super().__init__()
        self.register_buffer("mean", sd["obs_normalizer._mean"].clone())
        self.register_buffer("std", sd["obs_normalizer._std"].clone())
        layers = []
        for i in range(4):
            w = sd[f"mlp.{2*i}.weight"]
            lin = torch.nn.Linear(w.shape[1], w.shape[0])
            lin.weight.data[:] = w
            lin.bias.data[:] = sd[f"mlp.{2*i}.bias"]
            layers += [lin] + ([torch.nn.ELU()] if i < 3 else [])
        self.mlp = torch.nn.Sequential(*layers)

    def forward(self, obs):
        return self.mlp((obs - self.mean) / (self.std + 1e-2))


src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
ck = torch.load(src, map_location="cpu", weights_only=False)
pol = Policy(ck["actor_state_dict"]).eval()
n_obs = pol.mean.shape[1]
ex = torch.zeros(1, n_obs)
torch.jit.script(pol).save(os.path.join(out, "policy.pt"))
torch.onnx.export(pol, ex, os.path.join(out, "policy.onnx"), input_names=["obs"], output_names=["actions"],
                  dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}}, opset_version=17)
meta = {"source": os.path.abspath(src), "iteration": ck.get("iter"), "n_obs": n_obs, "n_act": 4,
        "obs_layout": ("ang_vel_b(3) proj_gravity_b(3) cmd[vx,wz,h_ref,m](4) leg_h-0.1825(2) leg_hdot(2) "
                       "wheel_w(2) last_action(4) imu_specific_force_g(3) hip_torque/5Nm(2, + = extend)") if n_obs == 25 else
                      ("ang_vel_b(3) proj_gravity_b(3) cmd[vx,wz,h_ref,m](4) leg_h-0.1825(2) leg_hdot(2) "
                       "wheel_w(2) last_action(4)") if n_obs == 20 else
                      "ang_vel_b(3) proj_gravity_b(3) cmd[vx,wz,h_ref](3) leg_h-0.1825(2) leg_hdot(2) wheel_w(2) last_action(4)",
        "action_layout": "leg_L leg_R wheel_L wheel_R, clip to [-1,1] in the controller",
        "leg_scale_m": 0.12 if n_obs in (20, 25) else 0.03, "wheel_scale_nm": 1.5, "wheel_lpf_hz": 20.0,
        "control_hz": 200}
json.dump(meta, open(os.path.join(out, "policy_meta.json"), "w"), indent=1)
with torch.no_grad():
    print(f"내보냄 {out}: 관측 {n_obs}, iter {ck.get('iter')}, 영입력 출력 {pol(ex).numpy().round(4).tolist()}")
