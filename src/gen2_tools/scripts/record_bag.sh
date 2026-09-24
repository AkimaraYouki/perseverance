#!/bin/bash
# Record a gen2 experiment bag (mcap). Usage: record_bag.sh <experiment-name> [extra topics...]
# Convention (shared with the desktop sim): keep the robot STILL for the first 5 s
# (zero / static-lean trim). Topics that do not exist yet are picked up when they appear.
set -e
NAME=${1:?usage: record_bag.sh <experiment-name> [extra topics...]}; shift || true
OUT_DIR=${GEN2_BAG_DIR:-$HOME/bags}
mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/$(date +%Y%m%d_%H%M%S)_${NAME}"
TOPICS=(
  /motors/state /motors/command /motor_test/status          # motors (4 axes), commands, bench tests
  /imu/data /imu/data_raw /imu/raw                           # iAHRS: imu_link and raw sensor axes
  /controller/state                                          # VMC+LQR state, gains, torques, clamps
  /diagnostics /robot/state /cmd_vel /joy                    # CAN stats, safety state, operator input
  /power/compute /power/motor
  /gps/nav_pvt /gps/fix /gps/velocity /gps/mag
  /tf /tf_static /scan
)
echo "Recording -> $OUT"
echo ">>> keep the robot STILL for the first 5 s <<<   (Ctrl+C to stop)"
exec ros2 bag record -s mcap -o "$OUT" "${TOPICS[@]}" "$@"
