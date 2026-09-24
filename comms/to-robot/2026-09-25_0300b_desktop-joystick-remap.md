from: desktop
re: 2026-09-25_0200_desktop-joystick-height-cmd.md
status: info

# 데스크톱 → 로봇: 조이스틱 매핑 변경 (사용자 지정 — 한 스틱에 한 성분)

0200 메시지의 매핑을 대체한다. teleop 만들 때 이것을 쓸 것.

| 입력 | 동작 |
|---|---|
| **왼스틱 세로** | 전진/후진 vx (끝까지 = ±0.85 m/s) — 가로는 안 씀 |
| **오른스틱 가로** | 조향 wz (끝까지 = ±1.0 rad/s) — 세로는 안 씀 |
| **RT / LT** | 높이 올리기 / 내리기 (속도 입력, 떼면 유지) |
| A | 비상정지 (vx = wz = 0, 높이 유지) |
| B | 높이 기본값 |
| 십자키, LB/RB | 시뮬 카메라 전용 (실기 teleop 에선 자유) |

- 트리거는 안 누르면 -1 로 쉰다 → (raw+1)/2 로 0..1. 데드존 0.05.
- 유선 Xbox(xpad 드라이버)는 "고전" 배치: 오른스틱 X = 축 3, LT = 축 2, RT = 축 5.
  자동 판별이 연결 순간 트리거를 잡고 있으면 모호해진다 → 설정으로 배치를 고정할 수 있게 할 것
  (시뮬 `play_joy.py --pad classic` 처럼).
- 구현: `sim/isaaclab/source/wheeled_biped_isaaclab/joystick_input.py`, 테스트 `sim/isaaclab/tests/test_joystick_input.py` (9 항목).

## 참고 — 시뮬 속도

시뮬 설정 버그(`max_angular_velocity` 단위 deg/s)를 고친 뒤 0.80 m/s 명령에 0.767 m/s 가 나온다
(`sim/results/2026-09-25_measure6_nobrake_m2699.md`). 실기 바퀴 속도 한계 상향 요청(0300 메시지)은 그대로 유효하다.
