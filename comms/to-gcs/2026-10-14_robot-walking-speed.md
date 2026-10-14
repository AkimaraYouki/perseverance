from: robot
re: 사용자 "걷는 속도 정도로 하고 싶은데 시속 3 km"
status: 반영 (커밋) — 패드·GUI 쪽은 pull + 빌드 + pad_teleop 재시작 필요

- 전진 최고 속도 0.7 → **0.83 m/s (3.0 km/h)** = balance tune.vmax_kmh 3.0 (kt083 추천과 같음):
  pad_teleop vx_max (config/pad_teleop.yaml + 코드 기본값), person_follow vx_max, Nav2 RPP desired_linear_vel.
- 데스크톱·GCS: `git pull && colcon build --packages-select gen2_tools gen2_bringup`, pad_teleop 다시 켜기.
  GUI 의 패드 vx_max 슬라이더 범위가 0.7 까지면 0.83 이상으로.
- 주의 (실기): 명령 0.7 에서도 외란 (턱·밀림) 뒤 실제 1.2–1.4 m/s 까지 넘친 적 있음 (10-14 01:55 넘어짐: 바퀴 17–18 rad/s 포화).
  0.83 m/s = 바퀴 11.9 rad/s (한계 18.85 의 63 %). 시뮬에서 vmax 3.0 의 외란 뒤 최대 속도·바퀴 포화 빈도 알려 주면 좋겠다.
