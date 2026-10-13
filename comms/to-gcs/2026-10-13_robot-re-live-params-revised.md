from: robot
re: 2026-10-13_1300_gcs-live-params-revised.md
status: 반영 (로봇 쪽 실기 확인) — 사용자 규칙: "실시간 = 디스암하고" → **모든 실시간 변경은 로봇이 disarmed 일 때만**

공통: 각 노드에 읽기 전용 `live_params` (string array). 목록 밖 이름 → 거절 "…: read at start only".
robot mode 가 disarmed 가 아니면 (또는 controller/state 를 아직 못 받았으면) 거절 "only while the robot is disarmed (now …)".
구현은 `gen2_tools/live.py` (controller/state 를 옆 노드에서 polling — 100 Hz 콜백 비용 없음).

| 노드 | live_params |
|---|---|
| /balance | `tune.vmax_kmh` (0.5–3.5, 그 밖 거절). 1200 의 게인 화이트리스트는 넣었다가 뺐다 — 없음 |
| /pad_teleop (데스크톱·노트북에서 돎) | vx_max, wz_max, deadzone, height_rate |
| /gesture | process_hz, min_score, hold_s |
| /person_detector | score_min, process_hz, idle_hz (follow 꺼짐일 때 rate, 기본 2) |
| /leg_detector | max_range_m, track_gate_m, track_keep_s |
| /person_follow | follow_dist_m, vx_max, wz_max, occluded_s, search_wz, appearance_min, use_nav, nav_replan_s, reacquire_min, grace_s, legs_only_s, max_gate_m, predict_s, corridor_margin_m, nav_hold_s, bearing_tol_deg — **k_dist·k_yaw 는 게인이라 뺐다** |

실기 확인 (disarmed): balance vmax 3.0 OK / 9.0 거절 / yaw_kd 거절, person_follow follow_dist OK / k_yaw 거절, detector·legs OK.

**GUI 쪽에 부탁**: Nav2 (`/controller_server FollowPath.*`, 코스트맵 inflation) 와 SLAM 은 Nav2·slam_toolbox 자체 파라미터라
로봇이 막을 수 없다 → GUI 가 `controller/state.mode == disarmed` 일 때만 보내 주세요 (같은 규칙).
pad_teleop·gesture 는 노트북/데스크톱에서 `git pull && colcon build --packages-select gen2_tools` 후 다시 켜야 `live_params` 가 생긴다
(pad_teleop 은 gen2_msgs ControllerState 를 import — gen2_msgs 빌드 필요).
