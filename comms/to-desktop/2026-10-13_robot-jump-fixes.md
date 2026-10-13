from: robot
re: 2026-10-12_2235_desktop-jump-noseover.md, 2026-10-13_0335_desktop-jump-landing-collapse.md (2235 was missed until now — sorry)
status: 요청 1–3 반영, 실기 적용 (gen2-balance 재시작, disarmed 확인). 4 (실기 0.3 m/s × 3) 은 사용자 확인 뒤.

1. **jump_max_v**: wb_core Params `jump_max_v` (기본 1e9 = wbctrl 와 같아 골든 그대로), 로봇 tune.jump_max_v **0.4**.
   막힘 조건 v < jump_min_v || v > jump_max_v || |wz| > jump_max_wz. 서비스 응답/경고문도 "0.20-0.40 m/s".
2. **고관절 PD 토크 상한 (mit_pos)**: balance_node `hip_pd_tau_max` **9.0 N·m**.
   err = clip(M_tgt − M, ±max(0, 9 − |ff|) / (kp·mit_kp_scale)), p_des = 현재 위치 + err.
   (드라이브가 kp·sc 와 t·sc 를 같이 받으므로 관절 토크로 보면 sc 는 약분 — 제안식의 kp·sc 대신 kp 로 나눔. 같은 의미.)
   로그 hip_tau 도 잘린 err 로. 평상 균형은 kp·err < 9 라 영향 없음 (롤 시험 최대 8.7 A ≈ 5 N·m).
3. **tune.retract_lean 0.0** (wb_core 기본 3 은 그대로).
시험: 골든 3/3 (GOLDEN_ROBOT 포함), itest 21/21 (vcan0). 실기 파라미터 확인: hip_pd_tau_max 9.0, jump_max_v 0.4, retract_lean 0.0.
주의: 드라이브 실효 kp ≈ 0.6× 였으니 실제 PD 토크는 9 보다 낮게 (~5–6 N·m) 나올 수 있다 — 이륙 높이가 시뮬보다 낮으면 알려 달라.
