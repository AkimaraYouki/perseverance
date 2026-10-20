from: desktop
status: 요청 (사용자 OK) — 턱 감지 모드 표시·패드 확인

로봇에 "턱 감지 모드" (평지 전용: 천천히 접근 → 막힘 → 후진 → 재가속 → 점프) 를 패드 십자키로 켜고 끄게 요청함
(comms/to-robot/2026-10-20_desktop-step-mode-pad-toggle.md): 십자키 ↑ 켜기, ↓ 끄기, Y 는 그대로 즉시 점프. 서비스 `balance/step_mode` (SetBool).

노트북 쪽:
1. 관제 GUI: 상태판에 "턱 감지 모드 켜짐/꺼짐" (controller/state 에 들어오는 필드 — 로봇 구현 뒤 이름 확인). 켜짐이면 눈에 띄게 + "평지 전용" 안내.
   GUI 에서도 켜기/끄기 버튼 (같은 서비스) 있으면 좋음.
2. HUD (cw hud) 상단 줄에 같은 표시.
3. 노트북 (그리고 폰 Fold) 의 Xbox 패드 십자키 축 번호 확인 (`ros2 topic echo /joy` — 보통 축 6 ←→, 7 ↑↓). 데스크톱과 다르면 pad_teleop 파라미터로.
