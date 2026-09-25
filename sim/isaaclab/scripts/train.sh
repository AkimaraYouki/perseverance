#!/usr/bin/env bash
# IsaacLab 의 rsl_rl train.py 를 그대로 쓴다. 직접 구현하지 않는다.
#   ./scripts/train.sh --num_envs 4096 --headless
#
# 패키지 설치를 여기서 하지 않는 이유: isaaclab.sh 가 LD_LIBRARY_PATH 를
# Isaac 라이브러리로 바꿔 /usr/bin/uname 이 libstdc++ 를 못 찾고,
# pip 의 배포판 감지가 거기서 죽는다. 설치가 필요하면 아래처럼 직접 한다.
#   env -u PYTHONPATH -u LD_LIBRARY_PATH \
#     ~/Desktop/IsaacLab/_isaac_sim/kit/python/bin/python3 -m pip install -e .
#
# PYTHONPATH 는 건드리지 않는다. isaaclab.sh 가 자기 경로를 기존 값에
# 이어붙이는 구조라, 비우면 pxr(USD) 까지 사라진다.
set -euo pipefail
ISAACLAB_PATH="${ISAACLAB_PATH:-$HOME/Desktop/IsaacLab}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"$ISAACLAB_PATH/isaaclab.sh" -p "$ROOT/scripts/_isaaclab_launch.py" \
  "$ISAACLAB_PATH/scripts/reinforcement_learning/rsl_rl/train.py" \
  --task "${WB_TASK:-Isaac-WheeledBiped-Balance-v0}" "$@"
