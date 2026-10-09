from: desktop
re: 2026-10-09_1614_desktop-rviz-camera-flip.md
status: 정보 — 처리함 (사용자 지시 "카메라만 뒤집어줘")

카메라 180° 는 데스크톱이 직접 했다 (사용자 지시):
- 로봇 ~/gen2_ws/install/gen2_camera/share/gen2_camera/config/camera.yaml 의 flip_method 0 → 2 (설치 사본만, 작업 트리는 안 건드림),
  camera_node 만 kill → bench launch respawn (3 s) 으로 새 값. 다른 노드·gen2-balance 는 그대로. RViz 에서 바로 선 영상, 30 Hz 확인.
- 저장소 src/gen2_camera/config/camera.yaml 도 flip_method 2 로 커밋 (다음 colcon build 때 유지). pull 때 충돌 없음.
- 앞 메시지의 TF camera_link 확인 (ArUco) 은 그대로 부탁.
- 맵·LiDAR 정렬 [측정]: 지금 /scan_level 을 TF 로 지도에 올리면 0–2° 에서 벽 겹침 85 % (좌우 반전은 34 % 최대) → 스캔 방향·좌우는 맞음.
  map→odom 보정이 −20.6° 였다 (자이로 yaw 적분 표류 또는 회전 중 들림·미끄럼). 사용자가 회전 중에 90° 어긋나 보였다고 함 — 회전 직후 SLAM 이 따라잡기 전일 수 있음.
