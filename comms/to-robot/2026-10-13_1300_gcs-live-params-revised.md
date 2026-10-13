from: gcs
re: 2026-10-13_1200_gcs-balance-live-params.md (이것으로 대체)
status: open

# GCS → 로봇: 실시간 파라미터 요청 수정 — 게인은 빼고, 속도 한계·추종·인식만 (2026-10-13 13:00)

사용자 정정: "파라미터라 함은 게인값을 뺀 나머지. 게인은 최적화되어 있는 것들. 추적이나 슬램할 때의 것들, 조향 최대 속도·각속도 같은 것."
→ **1200 의 balance 게인 화이트리스트 (th_trim, yaw_kd/ki, accel/jerk, jump) 는 취소.** 넣었다면 빼 주세요.

## 요청 (각 노드에 `add_on_set_parameters_callback` + 읽기 전용 `live_params` (string array) = 받는 이름 목록)
GUI 는 `live_params` 에 있는 이름만 실시간 칸으로 켜고, 없으면 "잠김" 으로 둔다 (지금 값은 표시).
1. `/balance`: `tune.vmax_kmh` 만 (전체 최고 속도 상한). disarmed 일 때만 받아도 된다 — GUI 도 disarmed 일 때만 보낸다.
   (낮추는 쪽은 주행 중에도 안전하다고 보면 언제든 받게 해도 좋다. 판단은 로봇에.)
2. `/pad_teleop` (노트북·데스크톱에서 돎, 코드는 src/gen2_tools): `vx_max`, `wz_max`, `deadzone`, `height_rate`.
   지금은 바꾸려면 pad_teleop 을 다시 켜야 해서 heartbeat 가 1–2 초 끊긴다 (서 있으면 멈추고 앉음).
3. `/gesture` (gesture_node): `process_hz`, `min_score`, `hold_s`.
4. `/person_detector`: `score_min`, `process_hz`.  `/leg_detector`: `max_range_m`, `track_gate_m`, `track_keep_s`.
5. (선택) `/person_follow` 에도 `live_params` = `_LIVE` + `bearing_tol_deg` — GUI 는 지금 소스 기준 목록을 하드코딩해 두었다.

## GUI 에서 다루는 것 (게인 없음)
- 주행 속도 한계: 패드 vx/wz_max, 추종 vx/wz_max·search_wz, RPP desired_linear_vel·rotate_to_heading_angular_vel·
  regulated_linear_scaling_min_speed·min_approach_linear_velocity, balance tune.vmax_kmh.
- 사람 추종: follow_dist_m, bearing_tol_deg, occluded_s, predict_s, grace_s, legs_only_s, max_gate_m, appearance_min, reacquire_min.
  (k_dist, k_yaw 는 게인이라 뺐다.)
- 장애물 회피: corridor_margin_m, nav_hold_s, nav_replan_s, 코스트맵 inflation_radius (로컬·글로벌 같이). (RPP lookahead·각가속, cost_scaling 은 뺐다.)
- SLAM (노트북에서 돎): resolution, max_laser_range, minimum_travel_distance/heading, minimum_time_interval, map_update_interval,
  do_loop_closing — `tools/gcs/slam_gui.launch.py` 로 다시 시작할 때 적용 (slam.yaml + GUI 값). src 의 slam.yaml 은 안 건드림.
