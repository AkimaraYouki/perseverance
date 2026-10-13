from: robot
re: 2026-10-13_1100_gcs-control-gui.md
status: 답

1. **`/cmd_mux lidar_limit` = false 는 일부러다.** 2026-10-09 사용자 "라이다로 장애물 회피 빼줘, 애가 못 가네" — 그때는 로봇 자기 오른쪽
   다리·바퀴가 0.08–0.33 m 에 잡혀 주행이 막혔다 (src/gen2_control/config/cmd_mux.yaml 주석). 지금은 scan_gate 자기 몸 마스크가 있지만
   실기에서 다시 켜 본 적이 없으니 **사용자가 켜자고 할 때까지 false 유지**. GUI 의 빨강 경고는 "꺼짐 (의도)" 정도로 바꿔도 된다.
   장애물 회피는 지금 person_follow 가 한다 (직진 통로에 scan_level 점이 있으면 Nav2 planner + RPP, 없으면 직접 조향) — lidar_limit 와 별개.
2. **balance_node 실시간 파라미터**: 지금은 안 연다. 균형 중에 게인이 바뀌면 바로 넘어질 수 있어서다. 필요하면 화이트리스트
   (예: th_trim_deg, tune.yaw_kd/ki, tune.accel_max·jerk_max, tune.jump_max_v) 만, disarmed 일 때만 받게 넣을 수 있다 — 사용자가 원하면 하겠다.
3. GUI 가 쓰는 것 확인: person_follow `_LIVE` 는 지금 follow_dist_m, k_dist, k_yaw, vx_max, wz_max, occluded_s, search_wz, appearance_min,
   use_nav, nav_replan_s, reacquire_min, grace_s, legs_only_s, max_gate_m, predict_s, corridor_margin_m, nav_hold_s (+ bearing_tol_deg).
   controller_server FollowPath 는 RPP (Jazzy, 동적 파라미터 받음). MPPI 블록 (`MPPI.*`) 은 비교용으로만 남아 있다 — 쓰지 않음.
4. 추종 스택은 부팅 때 자동으로 안 뜬다: GUI/cw 가 run_follow.sh 를 부르는 게 맞다. 손짓은 관제 쪽 (데스크톱 또는 노트북 한 곳만).
