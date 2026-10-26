#!/bin/bash
# Camera-first following stack (2026-10-25, desktop): person_tracker (YOLO11n-pose + ByteTrack + OSNet ReID, GPU) +
# person_follow_cam (camera lock, LiDAR range in the person's box, 0.7 m/s) + Nav2 (only around obstacles).
# No leg_detector. Same services / topics as the old stack (follow/enable, follow/hold, follow/status, cmd_vel/auto).
# Install once on the Jetson: copy this file to ~/gen2_ws/logs/ and tools/robot/models/osnet_x0_25_msmt17.onnx to
# ~/gen2_ws/models/ (stop with the existing stop_follow.sh: same pgid file)
# First start builds the OSNet TensorRT engine from ~/gen2_ws/models/osnet_x0_25_msmt17.onnx (~1-3 min, then cached).
# Lens: camera_cal.yaml (opencv fisheye K, D from the 2026-10-26 checkerboard, 124 deg horizontal) — copy it to ~/gen2_ws/logs/.
L=$HOME/gen2_ws/logs
if [ -f $L/follow.pgid ] && kill -0 -- -$(cat $L/follow.pgid) 2>/dev/null; then
  echo "follow stack already running (pgid $(cat $L/follow.pgid)): stop it first"; exit 1
fi
source /opt/ros/jazzy/setup.bash; source $HOME/gen2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONUNBUFFERED=1
export CYCLONEDDS_URI=file://$($HOME/gen2_ws/install/gen2_bringup/lib/gen2_bringup/gen2_dds.sh)
# 카메라 모델: ~/gen2_ws/logs/camera_cal.yaml (체커보드 캘리브레이션, tools/robot/camera_cal.yaml) 이 있으면 그것.
# CAMERA_MODEL=... 을 주면 그 값이 덮어씀 (pinhole | equidistant HFOV=.. | opencv_fisheye)
CAL=(); [ -f $L/camera_cal.yaml ] && CAL=(--params-file $L/camera_cal.yaml)
OV=(); [ -n "$CAMERA_MODEL" ] && OV=(-p camera_model:=$CAMERA_MODEL -p hfov_deg:=${HFOV:-160}.0)
echo $(ps -o pgid= -p $$) > $L/follow.pgid
ros2 launch gen2_bringup nav_follow.launch.py > $L/nav_follow.log 2>&1 &
$HOME/venv_vision/bin/python -m gen2_tools.person_tracker --ros-args "${CAL[@]}" "${OV[@]}" > $L/person_tracker.log 2>&1 &
[ "$GESTURE_ON_JETSON" = 1 ] && $HOME/venv_vision/bin/python -m gen2_tools.gesture_node > $L/gesture.log 2>&1 &
ros2 run gen2_tools person_follow_cam --ros-args "${CAL[@]}" "${OV[@]}" > $L/person_follow.log 2>&1 &
wait
