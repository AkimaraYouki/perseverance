#!/bin/bash
# Environment wrapper used by systemd: gen2_run.sh <launch file> [args...]
set -e
source /opt/ros/jazzy/setup.bash
source "${GEN2_WS:-$HOME/gen2_ws}/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Boot: wait for Wi-Fi / wired IPv4 (up to GEN2_NET_WAIT s) before choosing the DDS interfaces — otherwise a
# service started before Wi-Fi binds to lo for good and is invisible off-board (2026-10-13/14: gen2-balance on lo,
# START from the pad never arrived). No network after the wait -> local mode (robot alone).
ipv4_of() { ip -4 -o addr show dev "$1" 2>/dev/null | awk '{print $4}' | head -1; }
for _ in $(seq "${GEN2_NET_WAIT:-60}"); do
  [ -n "$(ipv4_of "${GEN2_DDS_IFACE:-wlP1p1s0}")$(ipv4_of "${GEN2_DDS_WIRED:-enP8p1s0}")" ] && break
  sleep 1
done
# DDS config chosen from the current network state (Wi-Fi vs local); see gen2_dds.sh
export CYCLONEDDS_URI=file://$("$(ros2 pkg prefix gen2_bringup)/lib/gen2_bringup/gen2_dds.sh")
# which mode this launch got (the NetworkManager hook restarts a launch that came up in local mode)
NET="$(ipv4_of "${GEN2_DDS_IFACE:-wlP1p1s0}")$(ipv4_of "${GEN2_DDS_WIRED:-enP8p1s0}")"
mkdir -p "$HOME/.ros" && { [ -n "$NET" ] && echo net || echo local; } > "$HOME/.ros/gen2_dds_mode.${1%.launch.py}"
exec ros2 launch gen2_bringup "$@"
