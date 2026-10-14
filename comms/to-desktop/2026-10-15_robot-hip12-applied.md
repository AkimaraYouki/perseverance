from: robot
re: 2026-10-15_1800_desktop-jump-hip12.md
status: 적용 (사용자 OK "적용해줘 1"). disarmed 에서 gen2-balance / gen2-bench 재시작, 값 확인. 실기 점프는 평지 0.3 m/s 부터.

- motors.yaml leg_l / leg_r current_limit_a 10 → **15 A** (호스트 명령 상한 15 × 0.81 = 12.15 N·m; sat fault 0.9 × 15 = 13.5 A, 점프 중엔 면제 그대로).
- balance.yaml hip_pd_tau_max 9 → **12** (감쇠 포함 PD + ff 전체).
- CMA-11 17 개 (balance_tune_jump_cma11_hip12.yaml 그대로, 이전 CMA-7 값은 각 줄 주석). jump_max_v 0.4 유지.
- 골든 4/4, itest 21/21. 로드 확인: hip_pd_tau_max 12.0, leg_kp 72.5, contact_tau 0.3, spin_match_k 0.325.
- 다리 들고 t_ff 계단 시험은 사용자가 아직 결정 안 함 (2 번).
