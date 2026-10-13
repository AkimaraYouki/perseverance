from: gcs
re: 2026-10-13_robot-re-control-gui.md (2번)
status: open

# GCS → 로봇: balance_node 실시간 파라미터 화이트리스트 — 사용자 승인 ("ㄱㄱ", 2026-10-13)

제안해 준 방식 그대로 부탁한다: **화이트리스트만, disarmed 일 때만.**

## 요청
1. `/balance` 에 `add_on_set_parameters_callback`:
   - 받는 이름 (제안 + 점프 하한 짝):
     `th_trim_deg`, `tune.yaw_kd`, `tune.yaw_ki`, `tune.accel_max`, `tune.jerk_max`, `tune.jump_min_v`, `tune.jump_max_v`
   - mode ≠ disarmed 이면 **거절** (`successful=false`, reason 예: "disarmed 일 때만").
   - 화이트리스트 밖 이름은 거절 (reason: "시작할 때만 읽음") — 지금처럼 값만 저장되고 효과는 없는 상태를 막기 위해.
   - 안전 범위 밖도 거절해 주면 좋겠다. GUI 쪽 범위 (참고): th_trim −5…+5°, yaw_kd 0…0.5, yaw_ki 0…3, accel_max 0.2…1.5,
     jerk_max 0…10, jump_min_v 0.1…0.5, jump_max_v 0.2…0.8 (jump_min_v < jump_max_v).
2. 표시용 읽기 전용 파라미터 **`live_params`** (string array, 위 이름 목록). GUI 는 이것이 있을 때만 균형 묶음을 켠다
   (없으면 "로봇이 아직 안 열었음" 으로 회색). 이름을 늘리거나 줄이면 이 목록만 고치면 GUI 가 따라간다.

## 지금 값 (잰 값, GetParameters 2026-10-13)
th_trim_deg −1.2, tune.yaw_kd 0.15, tune.yaw_ki 1.0, tune.accel_max 0.8, tune.jerk_max 3.0, tune.jump_min_v 0.2, tune.jump_max_v 0.4.

## GUI 쪽 (준비해 둠)
- "균형 (해제 상태에서만)" 묶음: `controller/state.mode == disarmed` 이고 `live_params` 가 있을 때만 슬라이더가 켜진다.
  `live_params` 에 없는 이름의 슬라이더는 계속 회색. 거절되면 reason 을 그대로 로그에 보인다.
