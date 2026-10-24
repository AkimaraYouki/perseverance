#!/bin/bash
# Camera-first following stack (2026-10-25, desktop): person_tracker (YOLO11n-pose + ByteTrack + OSNet ReID, GPU) +
# person_follow_cam (camera lock, LiDAR range in the person's box, 0.7 m/s) + Nav2 (only around obstacles).
# No leg_detector. Same services / topics as the old stack (follow/enable, follow/hold, follow/status, cmd_vel/auto).
# Install once on the Jetson: copy this file to ~/gen2_ws/logs/ and tools/robot/models/osnet_x0_25_msmt17.onnx to
# ~/gen2_ws/models/ (stop with the existing stop_follow.sh: same pgid file)
# First start builds the OSNet TensorRT engine from ~/gen2_ws/models/osnet_x0_25_msmt17.onnx (~1-3 min, then cached).
# CAMERA_MODEL=equidistant HFOV=160 for the 160 deg lens (default pinhole fx 530 = the current data-sheet values).
L=$HOME/gen2_ws/logs
if [ -f $L/follow.pgid ] && kill -0 -- -$(cat $L/follow.pgid) 2>/dev/null; then
  echo "follow stack already running (pgid $(cat $L/follow.pgid)): stop it first"; exit 1
fi
source /opt/ros/jazzy/setup.bash; source $HOME/gen2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONUNBUFFERED=1
export CYCLONEDDS_URI=file://$($HOME/gen2_ws/install/gen2_bringup/lib/gen2_bringup/gen2_dds.sh)
CM=${CAMERA_MODEL:-pinhole}; HF=${HFOV:-160}
echo $(ps -o pgid= -p $$) > $L/follow.pgid
ros2 launch gen2_bringup nav_follow.launch.py > $L/nav_follow.log 2>&1 &
$HOME/venv_vision/bin/python -m gen2_tools.person_tracker --ros-args -p camera_model:=$CM -p hfov_deg:=$HF.0 > $L/person_tracker.log 2>&1 &
[ "$GESTURE_ON_JETSON" = 1 ] && $HOME/venv_vision/bin/python -m gen2_tools.gesture_node > $L/gesture.log 2>&1 &
ros2 run gen2_tools person_follow_cam --ros-args -p camera_model:=$CM -p hfov_deg:=$HF.0 > $L/person_follow.log 2>&1 &
wait
