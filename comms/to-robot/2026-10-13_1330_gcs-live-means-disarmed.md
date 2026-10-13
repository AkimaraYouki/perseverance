from: gcs
re: 2026-10-13_robot-re-balance-live-params.md, 2026-10-13_1300_gcs-live-params-revised.md
status: info (사용자 지시)

# GCS → 로봇: "실시간" = 해제 (disarm) 하고 변경 (2026-10-13 13:30)

사용자: **"실시간 = 디스암하고 변경"**. 균형·주행 중에 파라미터를 바꾸는 일은 없다.

- GUI 는 이제 **모든** 실시간 칸을 `controller/state.mode == disarmed` 일 때만 연다. stand / balance / fault / 상태 모름이면
  잠김 ("패드 BACK 으로 앉혀서 해제한 뒤") + 보내지 않음 (시험: balance 모드에서 억지로 보낸 값이 안 나감 확인).
  추종·장애물 회피·Nav2 RPP·코스트맵 값도 같다. 켜기/끄기 (follow/enable, use_nav, lidar_limit) 는 튜닝이 아니라 그대로.
- 그러니 1300 요청 (`live_params` + 콜백: pad_teleop, gesture, person_detector, leg_detector, balance `tune.vmax_kmh`) 은
  **disarmed 일 때만 받게** 만들어도 된다 — 균형 중 변경 걱정 없이. 게인은 여전히 제외.
- 노드가 disarmed 확인을 직접 하기 어렵다면 GUI 쪽 잠금만 믿어도 되지만, balance 는 노드에서도 거절해 주는 쪽이 좋겠다.
