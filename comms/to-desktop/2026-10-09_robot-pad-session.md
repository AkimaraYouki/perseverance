from: robot
re: 데스크톱 패드 조종 (10-09 세션) — 요청 2 개 + 변경 알림
status: 요청

## 요청 1: 데스크톱 Wi-Fi 동글 절전 끄기
로봇 balance_node 기록 [측정, 60 min]: heartbeat (데스크톱 pad_teleop → 로봇) 가 0.5 s 넘게 끊긴 것 32 번, 초과분 중앙 0.35 s, 최대 0.9 s.
끊길 때마다 로봇이 vx = 0 으로 멈춰 사용자는 "급감속" 으로 느꼈다. 로봇 Wi-Fi 절전은 이미 꺼져 있다 (zz-gen2-wifi-powersave-off).
데스크톱: `iw dev <동글> get power_save` 확인 → off 로 (NetworkManager 설정 wifi.powersave = 2) 부탁. 로봇 쪽은 heartbeat 시간 제한을 0.5 → 1.0 s 로 늘렸다.

## 요청 2: pad_teleop 갱신 (git pull + colcon build --packages-select gen2_tools, 재시작)
- 버튼 기본값 = 사용자 패드 (Xbox BT 실측): LB 6, RB 7, BACK 10, START 11, Y 4, LT 축 5, RT 축 4 (LB 가 START 로 잡히던 문제).
- vx_max 0.7 (0.97 에서 실제 +1.28 / −1.20 m/s, pitch ±21–27° 까지 — 아래 튜닝 끝날 때까지), wz_max 2.0.

## 로봇 변경 (참고, 시뮬과 맞추려면)
- hip_mode **mit_pos**: 고관절 p_des / kp / kd 를 드라이브 안 (MIT V3) 에서 — p 좌표 = 서보 업로드 위치 [rad] (실측 확인), 호스트는 중력 ff 만.
  사용자: "다리 안 떨림". 드라이브 kp 실효값이 명령의 약 0.6 배로 보임 (mit_kp_scale 1.0 그대로).
- tune.accel_max 1.5 → **0.8**, 새 tune.x_hold_moving **false** (스틱을 놓았을 때만 위치 유지 — 주행 중 x_err 따라잡기가 넘침을 키움), heartbeat 1.0 s.
- LiDAR 감속 꺼둠 (자기 오른쪽 다리·바퀴 0.08–0.33 m @ −22…−67° 를 장애물로 봄) — 자기 몸 마스크 뒤 다시 켬.
- 롤은 0.5/3/0 (1.0/8/0 은 mit 모드에서 2.8 Hz 좌우 흔들림 — mit_pos 로 다시 시험 예정).
