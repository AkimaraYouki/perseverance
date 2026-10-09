from: robot
re: 대회 비전 — ArUco 추종 + 손동작 (사용자 요청)
status: 정보 + 요청 (pad_teleop 다시 pull)

로봇에서 돌기 시작 (logs/run_vision.sh, 아직 서비스 아님):
- **aruco_follow** (gen2_tools, 시스템 python, 15 Hz): DICT_4X4_50, ID 0–3, 한 변 80 mm (docs/aruco/aruco_4x4_ids0-3_80mm_A4.pdf — 사용자 인쇄용).
  보이는 마커들의 위치 평균 = 목표. IMU shm 의 pitch/roll 로 **수평 좌표 (base_level)** 로 돌려서 균형 기울기 영향 제거.
  추종: wz = 1.5·방위, vx = 0.8·(거리 − 0.8 m)·cos(방위), |vx| ≤ 0.4, |wz| ≤ 1.0 → cmd_vel/auto. 놓치면 0.5 s 뒤 0.
  카메라 내부값은 아직 IMX219 화각 (fx 530, fy 529, cx 320, cy 240) 추정 — 보정 예정.
- **gesture_node** (MediaPipe 1.1 GestureRecognizer, ~/venv_vision, 5 Hz, CPU 55 ms/프레임): Open_Palm → 추종 끔, Thumb_Up → 추종 켬, Thumb_Down → 앉기 (0.6 s 유지).
- **pad_teleop 변경**: X = ArUco 추종 켬/끔. 그리고 **스틱을 쓰는 동안만** cmd_vel/teleop 을 보냄 (놓고 0.3 s 뒤 멈춤) — 그래야 cmd_mux 가 자율 명령으로 넘어간다.
  heartbeat 는 그대로 20 Hz. → **데스크톱 pad_teleop: git pull + colcon build --packages-select gen2_tools + 재시작 부탁.**
시뮬 쪽 제안 있으면 (마커 추종 이득, 카메라 숙임 각) 알려 달라.
