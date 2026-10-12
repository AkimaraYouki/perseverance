#!/bin/bash
# Person following stack: leg_detector + person_detector (GPU) + gesture (stop / restart) + person_follow
# + Nav2 planner/MPPI (nav_follow.launch: drives around obstacles; person_follow falls back to direct steering
# when it is not up). Needs gen2-balance (scan_gate, cmd_mux, odom TF) running.
# Start: setsid ~/gen2_ws/logs/run_follow.sh &      Stop: ~/gen2_ws/logs/stop_follow.sh
L=$HOME/gen2_ws/logs
if [ -f $L/follow.pgid ] && kill -0 -- -$(cat $L/follow.pgid) 2>/dev/null; then
  echo "follow stack already running (pgid $(cat $L/follow.pgid)): stop it first"; exit 1
fi
source /opt/ros/jazzy/setup.bash; source $HOME/gen2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONUNBUFFERED=1
export CYCLONEDDS_URI=file://$($HOME/gen2_ws/install/gen2_bringup/lib/gen2_bringup/gen2_dds.sh)
echo $(ps -o pgid= -p $$) > $L/follow.pgid
ros2 launch gen2_bringup nav_follow.launch.py > $L/nav_follow.log 2>&1 &
ros2 run gen2_tools leg_detector > $L/leg_detector.log 2>&1 &
$HOME/venv_vision/bin/python -m gen2_tools.person_detector > $L/person_detector.log 2>&1 &
$HOME/venv_vision/bin/python -m gen2_tools.gesture_node > $L/gesture.log 2>&1 &
ros2 run gen2_tools person_follow > $L/person_follow.log 2>&1 &
wait
