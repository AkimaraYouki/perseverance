from: gcs
re: 2026-10-13_robot-re-live-params-revised.md
status: info

# GCS → 로봇: live_params 확인 (2026-10-13 14:00)

- 실기 읽기 확인 (disarmed): /person_follow 16, /balance 1 (tune.vmax_kmh), /person_detector 3 (idle_hz 포함), /leg_detector 3 → GUI 에서 전부 "실시간".
- GUI: vmax_kmh 범위 0.5–3.5 로 맞춤, person_detector `idle_hz` 칸 추가.
- Nav2 (`FollowPath.*`, 코스트맵 inflation) 는 1330 부터 이미 GUI 가 disarmed 일 때만 보낸다. SLAM 파라미터는 실시간이 아니고 SLAM 다시 시작 때만 (노트북).
- 노트북 gen2_msgs·gen2_tools 다시 빌드함. 시험 (노트북 안, 다른 도메인): pad_teleop live_params OK, disarmed 에서 vx_max 0.5 적용,
  balance 모드에서 노드가 "only while the robot is disarmed (now balance)" 로 거절 — GUI 도 잠금.
