from: desktop
re: 2026-10-25_desktop-camera-follow.md (갱신) — 데스크톱 사용 한도가 거의 끝나서 남은 설치·시험을 로봇 쪽에 넘김
status: 요청 — desktop/camera-follow (38493da 이후 최신) 를 main 에 합치고 설치·시험

## 이미 확인된 것 (데스크톱이 로봇에서 /tmp 로 돌려 봄, 로봇 설치본은 안 건드림)
- 배터리 전원: OSNet 엔진 빌드 62 s, 재부팅 없음 (30 W USB 어댑터일 때 두 번 리셋 → 엔진 빌드는 배터리로).
  엔진 저장됨: ~/gen2_ws/models/osnet_x0_25_msmt17_fp16.engine (onnx 도 있음).
- person_tracker 30.0 FPS: decode 4.8 / yolo 12.7 (벡터화 전, 이후 커밋으로 ~5 ms 예상) / track 0.2 / reid 1.5 ms.
- 로봇 시계가 1970 년 (인터넷 없음) — chrony 가 데스크톱·노트북 NTP 를 받게 하면 로그 시각 정리됨.

## 할 일
1. PR: desktop/camera-follow → main (gen2_tools 새 노드 3 개 + person_follow.py 에 _declare_extra 훅만, tools/gcs·desktop GUI/HUD/cw, tools/robot).
2. colcon build --packages-select gen2_tools; `cp tools/robot/run_follow_cam.sh ~/gen2_ws/logs/`.
3. 160° 렌즈인지 확인 → 맞으면 run_follow_cam.sh 를 CAMERA_MODEL=equidistant HFOV=<실제 화각> 으로 (기본 pinhole fx 530).
4. 시험 (사용자): 스택 켜기 (GUI 기본 = 카메라 중심) → START → X = 정면 사람 LOCK → 추종. HUD 초록 'TARGET #id'.
   한 명 → 좌우 이동 → 화면 밖 갔다 오기 (REACQUIRED) → 다른 사람 교차.
   로그: ~/gen2_ws/logs/person_tracker.log (10 s 마다 FPS·단계 ms·target 상태), person_follow.log, 가능하면
   `ros2 bag record /people/target /people/camera /follow/status /cmd_vel/auto /odom` → comms.
5. 문제 생기면 GUI '추종 스택 종류 → 다리 + 카메라 (옛)' 로 바로 되돌림 (run_follow.sh 그대로 있음).

## 주의
- person_follow_cam 은 노드 이름 person_follow 그대로 (패드 X, 손짓, GUI 버튼 동일). 균형 아니면 X 거부는 기존과 같음.
- TargetManager 파라미터 (person_tracker): reid_sim_min 0.65, reid_sim_single 0.75, reid_margin 0.08, reacq_gate_deg 12 + 40 °/s
  — 같은 군복은 외형으로 못 가르니 방위 게이트가 핵심. 오추적 보이면 bag 과 함께 알려 주면 데스크톱이 다음 세션에 조정.
