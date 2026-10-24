from: gcs
re: 2026-10-25_desktop-camera-follow-install.md (사용자 "gogo" → GCS 가 노트북에서 설치)
status: 정보 — 설치함 (main 합치기·PR 은 안 함, 로봇 몫으로 남김)

로봇 disarmed 에서 (로봇 시계 1970 이라 시각은 노트북 기준 2026-10-25):
- `git fetch origin desktop/camera-follow` (547ff70) → **worktree `~/gen2_ws/.claude/worktrees/camera-follow` (detached)**.
  로봇 작업 트리 (main 3b20ca3, golden_jump.csv 수정분 등) 는 그대로.
- `colcon build --base-paths <worktree>/src/gen2_tools --packages-select gen2_tools` (6.4 s) → install 에 person_tracker,
  person_follow_cam, tracking_core 생김. 브랜치의 gen2_tools 는 3b20ca3 에 새 파일 + person_follow.py `_declare_extra` 훅만 더한 것 (diff 확인).
- `tools/robot/run_follow_cam.sh` → `~/gen2_ws/logs/` (+x). osnet onnx·fp16 engine 은 이미 있었음.
- import 확인: venv_vision 에서 gen2_tools.person_tracker / tracking_core OK, person_follow_cam OK. 실제 구동·카메라·ReID 는 아직 안 돌림 (사용자 시험).
- 160° 렌즈 여부 (CAMERA_MODEL/HFOV) 는 확인 못 함 → 기본 pinhole fx 530 그대로.

**주의**: main 의 gen2_tools 로 다시 빌드하면 새 노드가 사라진다. PR desktop/camera-follow → main 합친 뒤 main 에서 빌드하고 worktree 는 지워 주세요.
