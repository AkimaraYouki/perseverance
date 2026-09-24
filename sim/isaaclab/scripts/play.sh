#!/usr/bin/env bash
set -euo pipefail
ISAACLAB_PATH="${ISAACLAB_PATH:-$HOME/Desktop/IsaacLab}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$ISAACLAB_PATH/isaaclab.sh" -p "$ROOT/scripts/_isaaclab_launch.py" \
  "$ISAACLAB_PATH/scripts/reinforcement_learning/rsl_rl/play.py" \
  --task Isaac-WheeledBiped-Balance-Play-v0 "$@"
