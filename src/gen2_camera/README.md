# gen2_camera — 저지연 CSI 카메라 퍼블리셔 (데스크톱 초안 → 로봇 쪽 보완·측정)

IMX219(CSI) → `nvarguscamerasrc`(ISP) → `nvvidconv` → `jpegenc`(CPU, libjpeg-turbo) → `sensor_msgs/CompressedImage`
→ `/camera/image_raw/compressed`, **Best Effort, depth 1**.

## 왜 이렇게 (jarabot `opencv_webcam_pub` 분석)
| 지연 원인 | 대응 |
|---|---|
| 원본 Image(640×480 = 921 KB/장)를 Wi-Fi DDS 로 → 조각 수백 개, 재전송·폐기로 큐 적체 | JPEG(수십 KB)로 발행 |
| H.264 인코더 선행 버퍼 + 수신 지터버퍼 | 프레임 독립 JPEG. **Orin Nano 는 NVENC 가 없다** |
| 처리보다 카메라가 빠르면 옛 프레임 적체 | `appsink max-buffers=1 drop=true sync=false` (항상 최신 1 장) |
| Reliable QoS + 큰 depth | Best Effort + depth 1 (받는 쪽도 Best Effort 로 구독해야 연결됨) |
| 발행 루프의 sleep (jarabot 33 ms) | sleep 없음, 카메라 fps = 발행 fps |

## 실행
```bash
sudo apt install python3-gi gstreamer1.0-plugins-good      # jpegenc, appsink
colcon build --packages-select gen2_camera && source install/setup.bash
ros2 launch gen2_camera camera.launch.py                   # config/camera.yaml (source argus, sensor_id, 해상도, flip)
ros2 run gen2_camera camera_node --ros-args -p source:=test   # 카메라 없이 시험 (videotestsrc)
```
받는 쪽: `python3 tools/camera/camera_view.py` (Best Effort 구독, 화면에 지연·fps). rqt_image_view 는 Reliable 구독이라 연결 안 될 수 있다.

## 데스크톱 검증 [측정, 2026-09-25, 같은 PC DDS 루프백, videotestsrc]
| 해상도 | fps | 크기/장 | 발행→수신 지연 평균 / 95 % | 디코드 |
|---|---|---|---|---|
| 640×480 q80 | 30.2 | 5.8 KB* | 0.5 / 0.6 ms | 1.1 ms |
| 1280×720 q80 | 30.1 | 15.4 KB* | 0.9 / 1.3 ms | 3.4 ms |

\* 시험 무늬가 단순해서 작다. 실제 카메라 영상은 30~60 KB/장(640×480 q80) 예상.
**이 지연은 인코딩 뒤부터다.** 센서 노출 + ISP(보통 1~2 프레임 = 33~66 ms), Wi-Fi, 표시가 더해진다. 실기에서
`camera_view.py` 로 다시 재고(시계는 chrony 로 맞출 것), 가능하면 화면에 스톱워치를 찍어 glass-to-glass 로 확인.

## 확인할 것 (Jetson)
- IMX219 가 CAM1 에 있으면 `sensor_id` 가 1 일 수 있다. 지난 부팅에서 I2C −121 로 probe 실패 → 케이블 재장착 먼저.
- 거꾸로 달렸으면 `flip_method: 2`.
- CPU: 640×480×30 fps JPEG 은 A78AE 코어 하나의 일부로 충분하다고 예상 [추정] — `top` 으로 확인.

## 로봇 쪽 보완 (2026-09-25)
- `encoder: nvjpeg` (nvjpegenc, NVMM 그대로) 기본. `cpu` (jpegenc) 도 선택 가능.
- 파이프라인 오류/EOS/3 s 무프레임 → 2 s 뒤 자동 재시작, launch 는 `respawn`.
- 헤더 stamp = now − 파이프라인 버퍼 나이 (`stamp_source: capture`). 측정해 보니 버퍼 나이는 2~5 ms 로,
  **센서 노출·ISP 시간은 포함되지 않는다** (nvarguscamerasrc PTS 가 ISP 출력 시점으로 보임). glass-to-glass 는 스톱워치로.
- 진단 `camera: stream` (LCD CAM 줄), 값: fps, KB/장, capture_to_publish_ms, restarts, last_error.
- `ros2 run gen2_camera camera_latency --seconds 10 --decode` — Best Effort 구독 지연 통계.
- bench 프로필(`gen2-bench.service`)에서 부팅 시 자동 실행 (`camera:=false` 로 끔). `record_bag.sh` 기본 토픽에는 없음.

## Jetson 실측 [측정, 2026-09-25, IMX219 CAM1 sensor-id 0, 같은 Jetson 안 구독]
| 설정 | fps | KB/장 | 파이프라인 안 지연 | 발행→구독 지연 평균 / p95 | camera_node CPU | 디코드(PIL) |
|---|---|---|---|---|---|---|
| 640×480 q80 `cpu` (jpegenc) | 30.0 | 25.6 | 5.4 ms | 7.0 / 7.5 ms | 32 % | 4.8 ms |
| **640×480 q80 `nvjpeg`** | 30.0 | 25.1 | 2.2 ms | **3.8 / 4.3 ms** | **21 %** | 4.4 ms |
| 1280×720 q80 `nvjpeg` | 29.7 | 121 | 4.5 ms | 6.0 / 7.0 ms | 21 % | 11.7 ms |

- `nvargus-daemon` 은 640×480@30 에서 CPU 약 20 % 추가 (한 코어 기준).
- 1280×720 은 약 29 Mbit/s — 휴대폰 핫스팟에는 무겁다. 640×480 (약 6 Mbit/s) 권장.
- 영상은 기존과 같이 분홍빛 (IMX219 ISP 튜닝 파일 없음).
