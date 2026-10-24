from: desktop
re: 2026-10-25_desktop-follow-fps-blur.md / 사용자 "초록 상자가 있는데도 lock 을 못 함 — 다리 판단으로 추적하는 것 같음, 다리 추적은 현실성 없음",
    "최고의 추종 스택 만들어 보자 — 오픈소스 찾아 가장 빠르고 잘 되는 것"
status: 요청 — 브랜치 desktop/camera-follow (c73ce5a) 시험. 기존 추종 스택은 그대로 두고 옆에 새로 (run_follow_cam.sh)

## 원인 (person_follow.py)
- 후보·등록을 라이다 스캔 (10 Hz) 때만 계산. 등록 = 카메라 상자 + 같은 방위 다리 트랙 (또는 확신 0.6 + 무릎/발목 + 발 거리) + 2.5 m 안.
  → 초록 상자 (0.4) 가 있어도 다리 트랙이 방위 ±10° 에 없으면 잠기지 않음. 160° 광각이면 pinhole 방위 (fx 530) 가 가장자리에서 크게 틀려 더 안 맞음.
- 데스크톱 측정 (10-25, 로봇 안): 검출 30 FPS, 12 ms/장, 카메라 30 Hz → 모델 속도 문제 아님. HUD 9–10 FPS 는 Wi-Fi 수신 (데스크톱도 최대 1.3 s 끊김).

## 새 스택 (오픈소스 조합)
- person_tracker: YOLO11n-pose TRT (지금 엔진) → **ByteTrack** (ECCV 2022, 흐림·반가림 저확신 검출로 id 유지) →
  지정 인물 = track id, 놓치거나 다른 사람과 겹친 뒤에만 **OSNet-x0.25 ReID** (torchreid MSMT17, TensorRT FP16, 512 차원) 로 재확인.
  같은 군복이면 외형으로 못 가르므로 '사라진 방위 근처' 게이트 (12° + 40°/s, 최대 70°) 를 같이 씀.
  people/camera (기존 형식 + id, target) · people/target (상태 NONE/LOCK/LOST/REACQUIRED, 방위, 상자 방위 폭, 발 행) 30 Hz.
- person_follow_cam (노드 이름 person_follow, 서비스·상태·Nav2 회피 그대로 상속): 켜면 정면에 가장 가까운 사람 바로 잠금 (다리 불필요),
  거리 = 상자 방위 폭 안의 라이다 앞쪽 무리 중앙값 (없으면 발 행 → 지면), 카메라 주기로 제어, 최고 0.7 m/s.
- 데스크톱 시험: 두 사람 교차 (다른 옷 / 같은 군복 sim 0.99) + 1 s 가림 → 오추적 0, 다시 나타나면 바로 재잠금.
  ROS 가짜 입력: 등록·FOLLOW·0.7 상한·OCCLUDED→FOLLOW, 명령 50 Hz.

## 할 일
1. `git fetch && git checkout desktop/camera-follow` (또는 이 커밋만), `colcon build --packages-select gen2_tools`.
2. `cp tools/robot/models/osnet_x0_25_msmt17.onnx ~/gen2_ws/models/`, `cp tools/robot/run_follow_cam.sh ~/gen2_ws/logs/`.
3. 해제 상태에서 `setsid ~/gen2_ws/logs/run_follow_cam.sh &` → 첫 실행은 OSNet 엔진 빌드 1–3 분 (person_tracker.log 'ReID ... ready').
   person_tracker.log 10 s 마다: frames/s, decode / yolo / track / reid ms, target 상태 → 30 FPS 유지되는지.
4. 160° 렌즈면 `CAMERA_MODEL=equidistant HFOV=160` 으로 실행 (렌즈 실제 화각 확인, 가능하면 cv2.fisheye 캘리브레이션 → K, D 파라미터).
5. 사용자 시험: 한 명 → 두 명 교차 → 잠깐 가림. 로그 + `ros2 bag record /people/target /follow/status /cmd_vel/auto /odom` comms 로.
6. 문제 없으면 GUI '추종 스택 켜기' 를 run_follow_cam.sh 로 바꾸는 건 사용자 확인 뒤.
