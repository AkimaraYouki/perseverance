from: robot
re: 사용자 "GUI 업데이트 요청" — 10-14 저녁 ~ 10-15 로봇 쪽 변경을 관제 GUI 에 반영
status: 요청 (로봇 쪽 준비 끝, 커밋됨)

## 1. 사람 추종 (person_follow)
- **카메라만으로도 추적** (라이다 다리가 없거나 라이다가 죽어도): 거리 = 발목 화면 높이 → 바닥면 (카메라 높이 = 다리 h + 0.081, 몸 pitch 반영).
- `follow/status` 끝에 **추적 출처** 붙음: `FOLLOW target 1.80 m @ +0 deg [camera]` / `[legs]` → GUI 에 출처 표시 (camera 면 "거리 추정" 으로 회색 등).
- `live_params` 새 항목 (disarmed 만): `cam_fx` (530), `cam_tilt_deg` (0), `cam_x_m` (0.108), `cam_h_off_m` (0.081), `cam_max_dist_m` (5.0).
  `camera_only` (true) 는 켜기/끄기라 **언제든** (use_nav 와 같음).
- 추종 vx_max 기본 0.83, Nav2 RPP `FollowPath.desired_linear_vel` 0.83 → 슬라이더 범위 0.83 이상으로.

## 2. 속도 (comms 2026-10-14_robot-walking-speed.md)
- pad_teleop vx_max 기본 **0.83 m/s (3 km/h)** — 슬라이더 범위 ≥ 1.0 까지. balance `tune.vmax_kmh` 3.0 (live, 0.5–3.5).

## 3. 상태 표시 (읽기 전용, GetParameters 로)
- 점프 켜짐/꺼짐: `/balance tune.jump_max_v` (0 = 꺼짐, 지금 0.4 = 켜짐, CMA-7). 점프 결과는 지금처럼 /rosout "jump:" 로.
- 균형 기준값 (사용자 "너무 잘됨" 10-14): accel_max 1.2, jerk_max 6, th_trim_deg 0.2, v_ki 0.8, bal_adapt_x_max 0.15, bal_adapt_max_deg 3, x_turn_wz 0.
- cmd_mux: lidar_limit 꺼짐이면 라이다가 죽어도 속도 제한 없음 (예전엔 0.1 m/s 로 묶임). → **라이다/IMU 끊김 경고**를 GUI 에 크게:
  /scan 이 0.5 s 넘게 없으면 "LiDAR 끊김 (USB 확인)", controller/state.imu_age_ms > 20 이면 "IMU 지연". 착지 충격에 둘 다 USB 가 빠진 적 있음.

## 4. 시험 표시 버튼 (새로)
- 로봇 health_logger 가 `gcs/marker` (std_msgs/String) 를 받아 events 로그에 `MARKER: …` 로 시각과 함께 남긴다.
- GUI 에 버튼 몇 개: "점프 평지", "점프 턱 5.6 cm", "자유 메모" → 시험 전에 누르면 로그 분석 때 판별 가능 (10-14 점프 20 회는 평지/턱 구분이 안 돼 분석을 못 나눔).

## 5. 배터리
- 모터 배터리 전압: 드라이브 CAN 응답엔 전압이 없음 (서보 상태·MIT 응답 = 위치·속도·전류·온도·오류). 지금처럼 "센서 없음" 유지.
  나중에 INA226 (I2C) 를 달거나 드라이브 설정으로 STATUS_5 를 받게 되면 알리겠다.
