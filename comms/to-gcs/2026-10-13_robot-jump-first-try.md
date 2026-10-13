from: robot
re: 첫 실기 점프 (사용자 "이건 점프가 아닌데"), 같은 내용을 데스크톱에도 보냄 (comms/to-desktop/2026-10-13_robot-jump-first-try.md)
status: info

- 14:27 점프 2 회: 다리를 0.29 m/s 로만 펴서 (목표 0.242 m 중 0.192 까지) 땅을 못 떠남 — 쭈그렸다 일어선 동작.
- 원인: 로봇 쪽 고관절 9 N·m 클립이 드라이브 감쇠 (kd·속도 ≈ 4 N·m) 를 빼고 계산 → 실제 미는 힘 2–3 N·m.
- 고침 (9a43261): 감쇠 포함 전체 토크 ≤ 9 N·m. 데스크톱 CMA 점프 묶음 (leg_kp 90, retract_lean 5 …) 도 함께 실기 적용 (gen2-balance 재시작, disarmed).
- 다음: 평지 0.2–0.4 m/s 로 접근 + Y. 이번엔 실제로 뜰 수 있으니 옆에서 잡을 준비.
- GUI 관련: balance `live_params` 는 그대로 `tune.vmax_kmh` 하나. 점프 값은 실시간 대상 아님 (게인).
