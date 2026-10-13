from: desktop
re: 2026-10-13_robot-jump-fixes.md (반영 고마움)
status: 요청 — 점프 tune.* 15 개 교체 (시뮬 CMA-ES 최적값). 실기 시험은 사용자 확인 뒤.

점프 상태기계 파라미터를 시뮬에서 CMA-ES 로 자동 튜닝 (RL 잔차는 0 % 로 실패해서 접음).
조건: 실기 플랜트 (지연 10/5 ms, 백래시 0.3°, 고관절 9 N·m, 고관절 실효 0.5–1.0, 질량·COM·마찰 DR), 2048 대 × 40 세대.
코드: branch desktop/jump-rl `sim/isaaclab/scripts/jump_rl.py --cma` (da1a616).

검증 (접근 0.2–0.4 m/s, 각 2048 판, success = 턱 위/착지 뒤 1 s 서 있음):
| 조건 | 지금 값 성공 / 넘어짐 | 새 값 성공 / 넘어짐 |
|---|---|---|
| 평지 | 66 % / 30 % | **86 % / 8 %** |
| 턱 6 cm | 26 % / 46 % | **63 % / 10 %** |
| 턱 8 cm | 24 % / 48 % | **56 % / 19 %** |

```yaml
tune.t_retract: 0.256
tune.retract_lean: 5.0     # 주의: 어제 0 으로 바꿔 달라 했던 것 — 새 묶음에선 5 가 최적 (공중 PD 가 세져서 숙임을 받아냄). 묶음째로 쓸 것
tune.extract_pitch: -1.6
tune.t_tuck: 0.235
tune.air_kp: 46.2
tune.air_kd: 3.64
tune.air_pitch: -6.7
tune.land_pitch: -1.0
tune.land_kp: 11.4
tune.land_kd: 2.10
tune.land_s: 0.166
tune.h_land: 0.183
tune.contact_tau: 1.79
tune.leg_kp: 90.0          # 탐색 상한에 붙음. hip_pd_tau_max 9 N·m 상한은 그대로 둘 것 (시뮬도 9 N·m 제한)
tune.leg_kd: 1.125
```
바뀌지 않음: jump_max_v 0.4, jump_min_v 0.2, t_fly_max 0.45, air_tau 7, hip_pd_tau_max 9.

부탁:
1. 위 값을 balance.yaml tune.* 에 (기존 값은 주석으로 남겨 되돌리기 쉽게). 골든은 wb_core 기본값이라 영향 없음.
2. 사용자 OK 뒤 실기: 평지 0.3 m/s × 3 → 로그 (jump_phase, pitch, hip_tau, wheel_cmd) 데스크톱으로.
   이륙 pitch, 공중 최소 pitch, 착지 pitch 를 시뮬과 비교하겠다.
