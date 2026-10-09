from: desktop
re: 2026-10-09_robot-slam-camera.md
status: 결과 + 요청 (카메라 180°)

## RViz (데스크톱 Humble) [확인, 화면 캡처]
- **지도 (/map 141x103, 갱신 포함)·/scan_level 점·TF·odom 모두 보임**, 카메라 30 Hz 수신.
- 처음엔 지도 한 장만 오고 갱신·스캔·카메라가 안 왔다 — 원인은 데스크톱 터미널이 env.sh 없이 **FastDDS** 로 붙은 것
  (~/.bashrc 는 ROS_DOMAIN_ID 42 만). "sequence size exceeds remaining buffer" = FastDDS↔Cyclone. `source ~/gen2_desk/env.sh` 뒤 정상.
- 카메라: gen2.rviz 의 Image 토픽이 /camera/image_raw (무압축, 발행 없음) → "No Image". **/camera/image_raw/compressed** 로 바꾸면 나옴
  (Humble RViz 가 토픽 끝 이름으로 compressed transport 를 고름, image-transport-plugins 설치함). gen2.rviz 도 그렇게 바꿔 주면 좋겠다
  (데스크톱은 지금 사본 ~/gen2_desk/gen2_desktop.rviz 로 씀).

## 요청: 카메라 180° 회전 (사용자: "카메라 180도 회전")
영상이 위아래·좌우 뒤집혀 보인다. src/gen2_camera/config/camera.yaml `flip_method: 0 → 2` (nvvidconv 180°) 로 바꾸고 카메라 노드 재시작 부탁.
TF camera_link 방향도 같이 확인 (ArUco 자세 계산에 영향).
