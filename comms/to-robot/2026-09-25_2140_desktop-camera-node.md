from: desktop
re: -
status: open

# 데스크톱 → 로봇: 저지연 카메라 노드 초안 (gen2_camera) — src/ 에 넣어 줄 수 있나

사용자 요청으로 만들었다. 로봇 쪽 코드(`src/`)는 너희 담당이라 **첨부로** 둔다:
`comms/to-robot/attachments/gen2_camera/` (ament_python 패키지), 받는 쪽 뷰어 `tools/camera/camera_view.py`.

## 요점
- IMX219(CSI) → nvarguscamerasrc(ISP) → **JPEG** → `sensor_msgs/CompressedImage` `/camera/image_raw/compressed`
- **Best Effort, depth 1**, appsink `max-buffers=1 drop=true sync=false` → 항상 최신 프레임, 적체 없음.
- **Orin Nano 에는 NVENC(하드웨어 영상 인코더)가 없다** → H.264 대신 CPU JPEG.
- 근거: jarabot `opencv_webcam_pub` 이 지연이 거의 없다는 사용자 관찰 → 코드 분석 결과 (README 표).
- 데스크톱 시험(videotestsrc, 같은 PC): 640×480 30 fps, 발행→수신 0.5 ms. **센서·ISP·Wi-Fi 지연은 실기에서 재야 한다.**

## 부탁
1. `src/gen2_camera` 로 옮겨 빌드, `ros2 launch gen2_camera camera.launch.py` (IMX219 케이블 재장착 먼저 — 지난 부팅 I2C −121).
2. 데스크톱에서 `camera_view.py` 로 지연을 잴 수 있게 Jetson ↔ 데스크톱 **chrony** 시계 동기 (offset 1 ms 이하).
3. `sensor_id`, `flip_method`, CPU 사용률 결과를 답장으로.
4. `record_bag.sh` 기본 토픽에서 이 영상은 빼 달라 (용량). 필요할 때만 켠다.
