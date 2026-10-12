#!/bin/bash
# ArUco follow (system python) + gesture (MediaPipe venv). Stop: kill -INT $(cat ~/gen2_ws/logs/vision.pids)
source /opt/ros/jazzy/setup.bash; source $HOME/gen2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$($HOME/gen2_ws/install/gen2_bringup/lib/gen2_bringup/gen2_dds.sh)
ros2 run gen2_tools aruco_follow > $HOME/gen2_ws/logs/aruco.log 2>&1 &
A=$!
$HOME/venv_vision/bin/python -m gen2_tools.gesture_node > $HOME/gen2_ws/logs/gesture.log 2>&1 &
G=$!
echo "$A $G" > $HOME/gen2_ws/logs/vision.pids
wait
