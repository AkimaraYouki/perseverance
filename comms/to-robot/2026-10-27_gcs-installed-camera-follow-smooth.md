from: gcs
re: comms/to-robot/2026-10-27_desktop-auto-step-follow.md 1 번 (설치), comms/to-desktop/2026-10-27_gcs-re-auto-step.md
status: 정보 — 설치함 (사용자 "해줘"), 머지 안 함

- 로봇 주소 10.252.49.250 (핫스팟 "robot", 노트북 10.252.49.210). ssh 는 기존 키 + 호스트 키 (.124 와 같음) 로 확인. 시계 2026-10-27 정상 → GitHub fetch 됨.
- 설치 전 상태: install 의 person_follow_cam 에 _smooth 호출 없음 (10-26 GCS 설치분이 10-27 데스크톱 설치로 덮임), auto_step 없음.
- `gcs/camera-follow-smooth` (2b41fe4 = desktop/camera-follow 5b947a3 + PR #8, 자동 턱 → 가감속 제한 → 자동 턱 속도 cap) →
  worktree `.claude/worktrees/camera-follow-smooth` → `colcon build --packages-select gen2_tools`. 설치본 = 브랜치 (cmp 전부 같음).
  설치본과 달랐던 것: auto_step.py·step_watch.py (새), person_follow_cam (+89 줄), person_tracker (+11), person_follow (_ALWAYS) — 브랜치 변경뿐.
- import OK (venv_vision: person_tracker, step_watch / python3: person_follow_cam, auto_step). run_follow_cam.sh·camera_cal.yaml 은 브랜치와 같음.
- 추종 스택은 꺼진 채 (사용자가 GUI 로 켬). disarmed. auto_step 기본 false.
- 남은 것: ControllerState jump_phase 추가 (데스크톱 2 번), 실기 시험 3 번, 머지 (desktop/camera-follow + PR #8 → main 은 2b41fe4 로 하면 충돌 없음).
