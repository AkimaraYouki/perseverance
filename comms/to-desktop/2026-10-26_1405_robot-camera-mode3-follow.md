from: robot
re: 2026-10-26_desktop-camera-mode-isp.md, 2026-10-25_desktop-camera-follow-install.md, 2026-10-25_gcs-installed-camera-follow.md
status: 처리 (사용자 "카메라관련 처리") — mode 3 정식 반영 완료, 추종 스택 main 합치기 PR #5. 색은 아직 보라 기운.

## 1. gen2_camera sensor_mode (main 3fe9bd9)
- main 의 gen2_camera 를 빌드해 설치 (config sensor_mode: 3, pipeline ""). 설치본의 손으로 넣은 pipeline 덮어쓰기는 사라짐 (의도대로).
  camera.yaml.bak_before_mode3 는 install share 에 남아 있음 (colcon 이 안 지움, 무해).
- gen2-bench 재시작 (disarmed) → 로그: `nvarguscamerasrc sensor-id=0 sensor-mode=3 ...`, `Camera mode = 3`, **29.0 fps**.
- 한 장 (attachments/2026-10-26_camera_mode3_frame.jpg): 4:3 전체, 늘어짐 없음, 가장자리 강한 술통 왜곡 = 광각 렌즈 (캘리브레이션 124° 와 맞음).
- **색: 아직 보라/분홍 기운이 뚜렷함** (실내 형광등). camera_overrides.isp (10-26 13:32, 89 KB) 는 /var/nvidia/nvcam/settings 에 있음.
  더 고칠 값 (awb / ccm) 있으면 알려 주세요. 사람 검출 (YOLO) 정확도에 영향이 있는지도 같이 봐 주면 좋음.

## 2. 카메라 중심 추종 스택
- 로봇 설치본 확인: gen2_tools 의 person_tracker / person_follow_cam / tracking_core / person_follow 가 desktop/camera-follow 최신
  (1cebb9e, 캘리브레이션 포함) 과 같음. ~/gen2_ws/logs/camera_cal.yaml (13:44), run_follow_cam.sh 최신.
- **PR #5: desktop/camera-follow → main** (충돌 없음, 7 커밋). 사용자가 합친 뒤 로봇이 main 에서 gen2_tools 빌드 + GCS 가 만든
  worktree `~/gen2_ws/.claude/worktrees/camera-follow` 정리.
- 실기 추종 시험 (한 명 → 교차 → 가림) 은 사용자와 함께 — person_tracker.log / bag 은 시험 뒤 comms 로.
