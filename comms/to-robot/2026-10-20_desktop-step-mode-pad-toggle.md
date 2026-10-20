from: desktop
re: 2026-10-20_desktop-step-detect-mode.md (턱 감지 모드 사양·값)
status: 요청 (사용자 OK) — 턱 감지 모드를 패드로 켜고 끄기. 장애물마다 전략을 바꾸려는 것.

## 버튼 (사용자 확정)
| 버튼 | 동작 |
|---|---|
| 십자키 ↑ | 턱 감지 모드 **켜기** |
| 십자키 ↓ | 턱 감지 모드 **끄기** (일반 주행) |
| Y | 그대로: 즉시 점프 (모드 켜져 있어도 수동 점프 가능) |
LB 단독은 쓰지 않음 (LB+RB 비상 해제와 겹침). 토글 아닌 켜기/끄기 분리 = 상태 헷갈림 방지.
Xbox 십자키는 joy 에서 보통 축 6 (←→) / 7 (↑↓) — `ros2 topic echo /joy` 로 확인해 `*_axis` 파라미터로. 폰 (Fold) 어댑터도 axes 8 개라 같은 위치일 것 (노트북·폰에서 확인).

## 할 일
1. **wb_core**: 턱 감지 로직 포팅 (사양·최종값 = 2026-10-20_desktop-step-detect-mode.md 갱신 절, wbctrl desktop/jump-rl cfd0055) +
   `jump_retry` 를 **실행 중 켜고 끌 수 있게** (Params 값이 아니라 상태). 끌 때 rt_phase·rt_run·blk_t 초기화, 후진·재가속 중이면 즉시 멈춤 (vx 0).
   골든: GOLDEN_SPIN 처럼 desktop/jump-rl wbctrl 로 턱 감지 케이스 (막힘 → 후진 → 재가속 → 점프) 를 만들어 맞춤.
2. **balance_node**: 서비스 `balance/step_mode` (std_srvs/SetBool). **균형 중에도 받음** (use_nav 켜기/끄기와 같은 "on/off 스위치" 예외). 응답 메시지에 결과.
   상태는 controller/state 에 (필드 추가 또는 기존 상태 문자열) + /rosout 한 줄 ("step mode on/off"). 끄기는 언제나 받기, 켜기는 DRIVE 일 때만.
3. **pad_teleop**: 십자키 ↑/↓ → `balance/step_mode` true/false (누르는 순간 한 번, 길게 눌러도 반복 X). 상단 docstring 버튼표 갱신.
4. **표시**: 로봇 LCD 헤더에 "STEP" 표시. (관제 GUI·HUD 는 노트북에 따로 요청함)
5. 모드 켜짐일 때 동작 확인 (점프 끄고 먼저): 스틱 앞으로 → 0.156 m/s 접근 (미는 양 무관), 상자 벽에 막힘 → 0.4 m 후진 → 0.28 m/s 재가속, 스틱 놓으면 취소, 회전 중엔 막힘 판정·점프 안 함.

평지 전용 모드 — 험지·경사에서 켜면 오판 가능 (관제 화면 경고로 안내).
