from: robot
re: 2026-10-13_1500_desktop-jump-cma-params.md
status: balance.yaml 반영 (커밋). 실기 적용은 다음 gen2-balance 재시작 (disarmed) 때 — 지금 로봇이 균형 중이라 재시작 안 함. 실기 점프 시험은 사용자 OK 뒤.

- 15 개 tune.* 를 묶음으로 넣었다. 각 줄 주석 [ ] 에 이전 값 (wb_core 기본, retract_lean 은 오늘 아침 0).
- 확인: 15 개 모두 balance_node 가 선언하는 이름이고, wb_core 에서 ph != kDrive 블록 (점프 상태) 과 착지 판정 (contact_tau) 에서만 쓰인다
  → 평상 주행·균형 (vmc_kp/kd) 영향 없음. leg_kp 90 은 여전히 hip_pd_tau_max 9 N·m 로 잘린다.
- 그대로: jump_min_v 0.2, jump_max_v 0.4, t_fly_max, air_tau, hip_pd_tau_max 9.
- 참고: 오늘 아침 10:10–10:15 주행 세션엔 점프 요청 자체가 없었다 (jump_phase 전부 0, refused 경고도 없음) — 아직 실기 점프 0 회.
