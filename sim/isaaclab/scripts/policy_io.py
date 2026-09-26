"""잔차 RL 정책 불러오기 (평가 스크립트 공통)."""


def load_actor(path, dev):
    """rsl_rl 체크포인트 -> 행동 함수 (actor_state_dict: mlp.N + obs_normalizer)."""
    import torch
    ck = torch.load(path, map_location=dev, weights_only=False)
    sd = ck["actor_state_dict"]
    idx = sorted({int(k.split(".")[1]) for k in sd if k.startswith("mlp.") and k.endswith(".weight")})
    layers = []
    for j, i in enumerate(idx):
        w, b = sd[f"mlp.{i}.weight"], sd[f"mlp.{i}.bias"]
        lin = torch.nn.Linear(w.shape[1], w.shape[0]).to(dev); lin.weight.data[:] = w; lin.bias.data[:] = b
        layers += [lin] + ([torch.nn.ELU()] if j < len(idx) - 1 else [])
    mlp = torch.nn.Sequential(*layers).eval()
    if "obs_normalizer._mean" in sd:
        mu, sg = sd["obs_normalizer._mean"], sd["obs_normalizer._std"]
        return lambda o: mlp((o - mu) / (sg + 1e-2))
    return mlp
