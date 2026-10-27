#!/bin/bash
# Switch the balance tune overlay (gen2_control/config/balance_tune_<name>.yaml on top of balance.yaml) and restart
# gen2-balance so it loads. Only while the robot is disarmed / fault / not running.
#   gen2_tune.sh opt1      -> "최적1"
#   gen2_tune.sh default   -> balance.yaml only
#   gen2_tune.sh           -> show the current one
set -u
F=$HOME/.ros/gen2_tune
cur=$(cat "$F" 2>/dev/null || true)
if [ $# -eq 0 ]; then
  echo "tune overlay: ${cur:-default}"
  exit 0
fi
name=$1
share=${GEN2_WS:-$HOME/gen2_ws}/install/gen2_control/share/gen2_control/config
if [ "$name" != default ] && [ ! -f "$share/balance_tune_$name.yaml" ]; then
  echo "no such overlay: $share/balance_tune_$name.yaml" >&2
  exit 1
fi
set +u   # the ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$HOME/gen2_ws/install/setup.bash" >/dev/null 2>&1
set -u
export CYCLONEDDS_URI=file://$("$HOME/gen2_ws/install/gen2_bringup/lib/gen2_bringup/gen2_dds.sh" 2>/dev/null)
mode=$(timeout 6 ros2 topic echo --once /controller/state 2>/dev/null | awk '/^mode:/{print $2}')
case "$mode" in
  ''|disarmed|fault) ;;
  *) echo "robot is '$mode' — sit it down / disarm first (no restart while it stands)" >&2; exit 2 ;;
esac
mkdir -p "$(dirname "$F")"
if [ "$name" = default ]; then : > "$F"; else echo "$name" > "$F"; fi
sudo systemctl restart gen2-balance
echo "tune overlay: $name (was ${cur:-default}); gen2-balance restarted"
