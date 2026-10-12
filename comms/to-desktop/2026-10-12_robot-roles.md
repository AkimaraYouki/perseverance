# 로봇 → 데스크톱: 젯슨/데스크톱 역할 분배 (사용자 요청) — 2026-10-12 23:20

사용자 지시로 역할을 나눴다. 전체 표: `docs/roles.md`. 데스크톱이 새로 맡을 것:

1. **gesture_node (손짓 정지/재개)** — 젯슨에서 뺐다 (MediaPipe CPU 30 %). 데스크톱에서 실행해 주세요.
   - `git pull && colcon build --packages-select gen2_tools gen2_bringup`
   - `pip install mediapipe` (Humble python3.10), 모델:
     `wget -O ~/gen2_ws/models/gesture_recognizer.task https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task`
   - `ros2 run gen2_tools gesture_node --ros-args -p process_hz:=15.0`
   - 입력 `camera/image_raw/compressed` (30 fps JPEG, 와이파이), 출력은 서비스 `follow/hold` (std_srvs/SetBool) — Humble/Jazzy 공통 타입.
2. **SLAM** — 매핑할 때 데스크톱에서 `ros2 launch gen2_bringup slam.launch.py` (scan_level + odom TF 사용). 젯슨에선 안 띄운다.
3. **대시보드 수집기** `~/.cache/desk_dash_robot.py` (ssh 로 젯슨에서 실행 중) 가 젯슨 CPU 를 25~27 % 쓴다:
   `/controller/state` 100 Hz 를 파이썬으로 받는 탓. 가능하면 데스크톱에서 직접 구독하거나, 젯슨에 둘 거면
   1 Hz 출력에 맞게 샘플링 (예: 받는 콜백에서 바로 return, 타이머 1 Hz 에서 최신값만 읽기는 이미 그렇다면 QoS depth 1
   + 구독 자체를 10 Hz 토픽으로) 해 주세요. 수정은 데스크톱 쪽 파일이라 손대지 않았다.
4. Nav2 는 젯슨에만 (nav2_msgs 액션 정의가 Humble/Jazzy 에서 달라 섞으면 깨짐). 추종 쪽 변경 요약:
   대상은 odom 좌표 + 속도 예측 (가려져도 예측 위치로 감, 1 초 넘게 가려지면 옷 색 일치로만 재획득),
   직진 통로가 비면 직접 조향 / 막히면 Nav2 (컨트롤러 MPPI → Regulated Pure Pursuit: MPPI 는 정지에서 +0.02 m/s 씩만 올라 출발을 못 했음).
   패드 X = 추종 켜기/끄기 (pad_teleop 최신이어야 함).
