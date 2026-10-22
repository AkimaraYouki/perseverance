from: gcs
re: comms/to-gcs/2026-10-23_0215_robot-step-mode-gui-request.md (요청 2 — HUD), 2026-10-20_desktop-step-mode-display.md (2 번)
status: 요청 — HUD 는 tools/desktop/hud.py (데스크톱 코드) 라 GCS 가 직접 안 고침

로봇이 HUD (카메라 화면) 우상단에 턱 감지 모드 배지를 원함 (사용자 요청). 노트북·데스크톱 둘 다 hud.py 를 쓰니 원본에 넣어 주세요.
- 구독: `/balance/step_mode_active` (std_msgs/Bool), `/balance/step_phase` (std_msgs/Int8). **둘 다 RELIABLE + TRANSIENT_LOCAL, depth 1** (바뀔 때만 발행).
  rclpy 주의: latched 값이 구독 직후 바로 오므로 콜백에서 값만 저장하고 그리기 루프에서 읽을 것 (GUI 에서 신호 연결 전에 와서 버려진 적 있음).
- 표시: true 일 때만 우상단 반전 배지 "STEP" + 단계 글자 (0 접근, 1 막힘 → 후진, 2 재주행 → 점프). 운전 중 보이게 크게, 화면은 덜 가리게.
- 관제 GUI (tools/gcs/cw_gui.py) 는 같은 규칙으로 반영 끝 — 상단 상태줄 배지, 기능 탭 켜기/끄기 버튼.
- 고쳐서 push 하면 노트북은 setup_gcs.sh 6 단계 (hud.py 복사) 로 받겠다.
