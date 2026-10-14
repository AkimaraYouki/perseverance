from: robot
re: 2026-10-14_2130_desktop-jump-height-log.md, 2026-10-15_1800_desktop-jump-hip12.md
status: 로그 첨부 + 답. CMA-11 / 고관절 12 N·m 는 사용자 OK 대기

## 2130 요청: 21:09:08 점프 ±2 s
- attachments/2026-10-14_210908_jump_pm2s.csv (step 로그 열 그대로, 800 행; 2026-10-15_jumps_cma7_all.csv 의 jump_id 4 와 같은 판)
- EXTRACT 165 ms, 다리 h 0.124 → 0.241 (평균 0.70 m/s), 고관절 보고 전류 평균 L 4.6 / R 4.9 A (명령 8.1 N·m 클립).
- 드라이브 온도 (health 1 Hz): 고관절 L 42 / R 44 °C, 바퀴 31 °C — 열 문제 없음. 전압: 드라이브 CAN 응답에 전압 없음 (서보 상태·MIT 응답에 필드 없음), 센서 허브 제거로 배터리 전압도 없음.
- IMU 비력은 step 로그에 없음 (/imu/data 500 Hz 는 bag 으로만). 다음 점프 때 rosbag (/imu/data) 같이 남기겠다.

## 1800 (고관절 12 N·m + CMA-11)
- 사용자에게 OK 묻는 중. 적용 땐: balance_node hip_pd_tau_max 12, hip 명령 상한 (current_limit 10 A × Kt 0.81 = 8.1) 은 motors.yaml leg current_limit_a 를
  15 A 로 올려야 12 N·m 가 나옴 → sat_frac fault (0.9 × current_limit) 도 같이 오름. 드라이브 자체 전류 한계 확인 필요.
- t_ff 시험 (다리 들고 t_ff 계단 → 보고 전류) 은 모터 명령이라 사용자 OK 뒤.

## 참고 (오늘 07:45 추종 중 넘어짐)
따라가던 중 (명령 0.25–0.53 m/s, 회전 −0.6) 바퀴가 0.1 s 에 +10 → −1 rad/s 로 멈춤 (충돌) → gy +4.8 rad/s 로 앞으로 넘어짐. 1 분 전 사용자가 GUI 로
corridor_margin_m 을 0 으로 내림 (Nav2 로 넘기는 통로 판단이 늦어짐). 넘어진 뒤 IMU USB 다시 끊김 (0.3 s 뒤 복구).
