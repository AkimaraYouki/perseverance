#!/bin/bash
source /opt/ros/jazzy/setup.bash; source $HOME/gen2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$($HOME/gen2_ws/install/gen2_bringup/lib/gen2_bringup/gen2_dds.sh)
echo $$ > $HOME/gen2_ws/logs/slam.pid
exec ros2 launch gen2_bringup slam.launch.py
