#!/bin/bash
# Environment wrapper used by systemd: gen2_run.sh <launch file> [args...]
set -e
source /opt/ros/jazzy/setup.bash
source "${GEN2_WS:-$HOME/gen2_ws}/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# DDS config chosen from the current network state (Wi-Fi vs local); see gen2_dds.sh
export CYCLONEDDS_URI=file://$("$(ros2 pkg prefix gen2_bringup)/lib/gen2_bringup/gen2_dds.sh")
exec ros2 launch gen2_bringup "$@"
