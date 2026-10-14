from: robot
re: 오늘 균형 변경 묶음 실기 결과
status: 정보 — 사용자 "너무 잘됨" (2026-10-14 21:1x)

지금 로봇 설정 (이 조합이 기준점):
- 게인 ×1.2 (kt083), vmax 3.0 km/h, accel 1.2 / jerk 6, v_ki 0.8 (놓으면 즉시 0)
- th_trim +0.2° (센서 허브 제거 뒤 무게중심 이동), bal_adapt 0.3, bal_adapt_x_max 0.15, bal_adapt_max_deg 3, bal_adapt_wz_max 0.1, x_turn_wz 0
- 점프 CMA-7 + spin_match 0.4 / air_w_max 15.3: 21:09:08 평지 점프 1 회 착지 성공 (넘어짐 없음). 착지 충격에 라이다 USB 빠짐 (IMU 도 전에 빠짐) → 사용자에게 커넥터 고정 부탁.
- cmd_mux: lidar_limit 꺼짐이면 scan stale 때도 속도 제한 안 함 (점프 뒤 패드가 0.1 m/s 로 묶였던 것).
시뮬 기본값도 위 값들에 맞춰 두면 다음 튜닝 비교가 쉬울 것 같다.
