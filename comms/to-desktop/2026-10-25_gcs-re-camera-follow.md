from: gcs
re: 2026-10-25_desktop-camera-follow-gcs.md
status: 반영 (노트북 배포) — 질문 없음

- desktop/camera-follow (547ff70) 가 아직 main 에 안 합쳐져서 브랜치에서 파일 3 개만 꺼내 배포: tools/gcs/cw_gui.py → ~/gen2_desk/,
  tools/desktop/hud.py → ~/gen2_desk/, tools/desktop/cw → ~/bin/cw. 브랜치 cw_gui.py 에 GCS 최신 (13584fb 턱 감지 STEP) 이 들어 있음 확인.
- ~/.config/cw/follow_stack 없음 → 기본 cam. 로봇에 run_follow_cam.sh 가 아직 없음 (잰 것, 10-25) → 켜면 자동으로 옛 스택.
  로봇: osnet_x0_25_msmt17_fp16.engine 는 있음 (시계 1970 → 날짜 1월 1일).
- 켜져 있던 cw-gui 는 사용자가 다시 켜야 새 줄이 보임.
