from: robot
re: 사용자 요청 "카메라 영상 받는 거랑 SLAM 시도 — 제어랑 동시에"
status: 요청 (데스크톱에서 보기)

로봇에서 지금 돌고 있음 (제어와 동시):
- 카메라: /camera/image_raw/compressed 30 Hz (JPEG 640x480).
- **SLAM**: slam_toolbox online async (Jetson) → /map (5 cm, 지금 7.1 x 5.2 m), TF map → odom → base_link → laser / camera_link / imu_link.
  odom = balance_node (바퀴 절대 속도 + 자이로 yaw 적분, stand/balance 중에만 적분). LiDAR 는 scan_gate 를 거친 /scan_level (|pitch|,|roll| < 5° 인 스캔만 — 가속 때 바닥을 벽으로 그리지 않게).
  로봇 CPU: slam·scan_gate 합쳐 몇 % (scan_gate 는 IMU 를 shm 에서 읽음).

데스크톱에서:
```
git pull       # src/gen2_bringup/rviz/gen2.rviz 갱신됨 (Map, TF, Odometry, /scan_level, 카메라)
sudo apt install ros-humble-rviz2 ros-humble-image-transport-plugins ros-humble-rqt-image-view
rviz2 -d src/gen2_bringup/rviz/gen2.rviz          # Fixed Frame = map
ros2 run rqt_image_view rqt_image_view /camera/image_raw/compressed    # 카메라만 볼 때
```
지도 저장 (로봇): `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/parksudo/maps/room'}}"`.
알려 줄 것: RViz 에서 map 이 보이는지, 카메라 지연 체감. 주행하며 지도 그리면 odom 품질 (바퀴 미끄럼·백래시) 을 같이 보겠다.
