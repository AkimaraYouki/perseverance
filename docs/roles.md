# 젯슨 / 데스크톱 역할 분배 (2026-10-12)

원칙: **와이파이가 끊겨도 로봇이 안전하게 하던 일을 계속해야 하는 것 = 젯슨**, 무겁고 늦어도 되는 것·사람이 보는 것 = 데스크톱.
(젯슨은 Jazzy, 데스크톱은 Humble: 표준 메시지·서비스만 오간다. nav2_msgs 액션은 두 배포판 정의가 달라서 Nav2 는 젯슨에만 둔다.)

| 구분 | 젯슨 (로봇) | 데스크톱 |
|---|---|---|
| 제어 | balance_node 200 Hz, 모터 CAN, 홈/일어서기/점프, cmd_mux | — |
| 센서 | IMU, 라이다, 카메라, 전원(sensor_hub), scan_gate(기울기·자기몸 마스크) | — |
| 사람 인식 | person_detector (YOLO11n-pose TensorRT, GPU 3 ms), leg_detector (라이다 다리) | — |
| 추종 판단 | person_follow (대상 등록·가림 예측·재획득·직접 조향) | — |
| 장애물 회피 | Nav2 planner + RPP (가는 길이 막혔을 때만) | — |
| 손짓 | — | **gesture_node** (MediaPipe, 젯슨 CPU 30 % 먹던 것) |
| 지도 | — | **SLAM** (slam_toolbox, 매핑할 때만) |
| 조작 | — | joy_node + pad_teleop (heartbeat 20 Hz) |
| 보기·기록 | LCD 상태판 | RViz, 대시보드, rosbag 기록, 파라미터 튜닝 |
| 시뮬 | — | Isaac Sim (게인·제어 검증) |

와이파이가 끊기면: 패드 heartbeat 가 끊겨 로봇은 멈추고 앉는다 (지금 안전 규칙). 손짓(데스크톱)도 끊기지만 정지는 위 규칙이 보장.
나중에 대회에서 패드 없이 자율로만 달려야 하면 gesture_node 를 다시 젯슨으로 (`GESTURE_ON_JETSON=1`).

## 실행

젯슨 (부팅 후):
```bash
# gen2-balance 는 부팅 때 자동
setsid ~/gen2_ws/logs/run_follow.sh &        # 추종 스택 (사람 인식 + 추종 + Nav2)
~/gen2_ws/logs/stop_follow.sh                # 정지
```

데스크톱 (gen2_ws 에서 `git pull && colcon build --packages-select gen2_tools gen2_bringup` 후):
```bash
ros2 run joy joy_node
ros2 run gen2_tools pad_teleop
ros2 run gen2_tools gesture_node --ros-args -p process_hz:=15.0      # 손짓: 손바닥 = 정지, 엄지 = 재개
ros2 launch gen2_bringup slam.launch.py                            # 지도 만들 때만
rviz2 -d ~/gen2_ws/src/gen2_bringup/rviz/gen2.rviz
```
gesture_node 준비 (한 번): `pip install mediapipe` 그리고 모델
`wget -O ~/gen2_ws/models/gesture_recognizer.task https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task`
