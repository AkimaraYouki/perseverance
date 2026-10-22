from: gcs
re: 2026-10-23_0215_robot-step-mode-gui-request.md, 2026-10-23_robot-step-mode-pad.md, 2026-10-23_robot-pad-start-height.md
status: 반영 (요청 1 · 3). 요청 2 (HUD) 는 tools/desktop/hud.py 가 데스크톱 코드라 데스크톱에 넘김

## 1. 관제 GUI
- 상단 상태줄: `balance/step_mode_active` true 면 반전 배지 **"STEP · 접근 / 막힘 → 후진 / 재주행 → 점프"** (`balance/step_phase` 0/1/2), false 면 숨김.
  둘 다 RELIABLE + TRANSIENT_LOCAL depth 1 로 구독. (latched 값이 GUI 신호 연결 전에 와서 버려지던 것 → 저장 후 0.5 s 마다 다시 표시.)
- 기능 탭 "턱 감지 모드 (평지 전용, 패드 십자키 ↑/↓)" 켜기·끄기 = `/balance/step_mode` SetBool, 응답 message 는 GUI 로그. /balance 가 없으면 회색 "balance_node 없음".
- /rosout: 새 형식 `jump: start, <operator|step-detect> (v …)`, `jump: refused, <who> (…)` 해석 (옛 형식도 받음), 점프 시작 때 주황 **JUMP** 배지 2 s.
  `step-detect: approach / blocked -> back off / run at the step (odo …)` → 로그에 "턱 감지: 접근 / 막힘 → 후진 / 재주행 → 점프".
- 시험 (잰 것): 가짜 balance 노드 (도메인 77) 로 배지·단계·로그 문구 확인. 실기 (disarmed, 02:05): step_mode_active false → 배지 숨김, 점프 줄 "켜짐 0.20–0.60 m/s".
  켜진 상태·단계 바뀜은 실기에서 아직 못 봄.

## 3. 패드
- main (3b20ca3) pull → 노트북 `colcon build gen2_msgs gen2_tools gen2_bringup` → pad_teleop 재시작 (disarmed 에서, 02:00).
  로그: "pad_teleop ready: … D-pad ↑/↓ = step mode". 축 7 ↑ = +1 은 노트북 측정값 그대로라 파라미터 안 바꿈. START 높이 초기화도 들어감.
