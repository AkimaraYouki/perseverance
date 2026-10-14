from: gcs
re: 2026-10-15_robot-gui-update-request.md (+ walking-speed, jump-cma5-first, bias-walk, cma7-turn-creep 읽음)
status: 반영 (커밋). 질문 없음

1. 추종: `follow/status` 끝의 `[camera]` → "— 카메라 (거리 추정)" 회색, `[legs]` → "— 라이다 다리". 기능 탭에 "카메라만으로 추적" 켜기/끄기
   (`camera_only`, mode 상관없이 보냄). 튜닝 탭에 cam_fx · cam_tilt_deg · cam_x_m · cam_h_off_m · cam_max_dist_m (disarmed 만, 기본값 = person_follow.py).
2. 속도: pad_teleop · person_follow vx_max, FollowPath.desired_linear_vel 기본 0.83, 슬라이더 최대 1.2. balance vmax_kmh 기본 3.0.
   노트북 `colcon build gen2_msgs gen2_tools gen2_bringup` 함. pad_teleop 재시작은 사용자가 (지금 도는 건 옛 빌드 0.7).
3. 읽기 전용: 기능 탭 "점프 (패드 Y)" = tune.jump_max_v > 0 이면 켜짐 (+ jump_min_v 범위). 튜닝 탭 맨 위 "균형 기준값" =
   jump_min/max_v, accel_max, jerk_max, th_trim_deg, v_ki, bal_adapt_x_max, bal_adapt_max_deg, x_turn_wz (GetParameters, 노드가 보일 때마다).
   경고: 상태판 맨 위 빨강 — 로봇 (controller/state) 이 보이는데 /scan 이 0.5 s 넘게 없으면 "LiDAR 끊김 (USB 확인)",
   imu_age_ms > 20 이면 "IMU 지연 … ms". /scan 은 raw 구독 (도착 시각만, 역직렬화 안 함).
4. 시험 표시: 기능 탭 "점프 평지" / "점프 턱 5.6 cm" / "자유 메모…" → `gcs/marker` (std_msgs/String). 보낸 것은 GUI 로그에도.
5. 배터리: "센서 없음" 유지.

참고 (지금 잰 것, 10-15 오전): motor_monitor 진단 4 개 모두 "no feedback" (모터 전원 꺼짐으로 보임). GUI 는 응답 없는 드라이브 온도 (0) 를 안 씀.
