from: desktop
re: 2026-10-14_0930_desktop-cma5-backflip-reproduced.md
status: 새 점프 묶음 CMA 7 (실험용, 사용자 OK 뒤) — **평지 0.3 m/s 만 먼저**. 턱은 아직 넘어짐이 높음.

파일: sim/model/robot_params_2026-10-14_kt083/balance_tune_jump_cma7.yaml (점프 tune 17 개, 게인 ×1.2 와 같이).
시뮬 (실기식 고관절 상한, 실효 0.5–0.7, 0.2–0.4 m/s, 2048 판):
| | 평지 | 6 cm | 8 cm | 10 cm |
|---|---|---|---|---|
| 성공 / 넘어짐 | 91 / 1 % | 62 / 9 % | 59 / 17 % | 54 / 26 % |
EXTRACT 최대 v 0.50 m/s (CMA 5: 1.0 — 실기 1.23 으로 달려나감), 이륙 gy 중간 −3.8 rad/s, 미끄럼 0.20 m/s, 바퀴 최고 17 cm, 다리 바닥 닿음 0 %.
공중 pitch PD 는 거의 끔 (air_kp 8, kd 0.5 — 어차피 75 ms 에 포화), spin_match 0.4 + air_w_max 15.3 이 착지 바퀴 속도를 맞춤.

실기 시험 때 보고 싶은 것 (step 로그): EXTRACT 중 v·바퀴 ω·바퀴 토크 부호 흔들림, 이륙 gy, 공중 바퀴 ω, DESCEND 에서 ω_abs·R vs vf, 접지 다리 h.
→ 시뮬 값과 다르면 그 차이를 시뮬에 넣고 다시. CMA 8 (탐색 범위 넓힘, 턱 6–10 cm 위주) 돌리는 중.
IMU USB 커넥터 고정 뒤에 해 주세요.
