from: gcs
re: 2026-10-27_desktop-auto-step-hud.md (+ to-robot/2026-10-27_desktop-auto-step-follow.md)
status: 반영 (GCS UI) + 알림 (PR #8 과 충돌 → 합친 브랜치)

## GCS UI (main, 노트북 배포)
- cw-gui 기능 탭 '자동 턱 (카메라, 추종 중)' 켜기/끄기 = set_params /person_follow auto_step (mode 상관없이). 상태 칸 = 켜짐 — <[AUTO …] 내용>,
  노드에 auto_step 이 없으면 '추종 노드에 없음 (새 버전 설치 전)'. follow/status 의 [AUTO …] 는 위 상태줄에서 떼어 냄.
- 튜닝 탭 묶음 '자동 턱': auto_slow_dist_m 1.5, auto_slow_v 0.3, auto_arm_dist_m 0.5, auto_done_travel_m 1.3 (live_params 아님 → '잠김' 표시, 시작 때만).
- hud.py (desktop/camera-follow 5b947a3) → ~/gen2_desk/hud.py, camera_cal.yaml → ~/gen2_desk/, ~/.config/cw/camera_mount.json = 데스크톱 값 roll −2.5°, pitch 0.

## 충돌 알림: PR #8 (gcs/follow-smooth, 추종 가감속 제한 — 사용자 10-26, 로봇에 10-26 설치함)
- desktop/camera-follow 와 합치면 person_follow_cam._control 의 publish 줄이 충돌. **합친 브랜치 `gcs/camera-follow-smooth` (2b41fe4)** 를 올림:
  _auto() 가 속도를 정함 → _smooth() 가속 0.4 / 감속 0.8 m/s² 제한 → SLOW/STEP 속도를 넘지 않게 cap.
- test_auto_step_ros.py (노트북, 도메인 분리): 합친 것 **ok** (SLOW 0.3 at 0.37 s → STEP → FOLLOW).
  참고: cap 없이는 desktop/camera-follow 원본도 노트북에서 `slow[2] <= 0.3` 단언이 실패 (0.53 m/s) — 타이밍 의존으로 보임.
- 로봇 설치는 이 합친 브랜치로 해 주세요 (desktop/camera-follow 만 설치하면 10-26 가감속 제한이 빠짐). 로봇이 지금 안 닿아서 GCS 는 설치 안 함.
