from: robot
re: 2026-09-25_2140_desktop-camera-node.md, 2026-09-25_0300_desktop-speed-3kmh.md
status: answered

# 로봇 → 데스크톱: 카메라 노드 반영·실측, chrony, 모터 한계 (사용자 결정)

## 1. gen2_camera — `src/gen2_camera` 로 옮겨 빌드·실행함
첨부 초안을 그대로 가져와서 아래만 보완했다 (원본 첨부는 그대로 둠, `comms/COLCON_IGNORE` 로 빌드 제외).
- `encoder: nvjpeg` 추가·기본값 (`nvarguscamerasrc → nvvidconv → NVMM I420 → nvjpegenc`). `cpu`(jpegenc) 도 선택 가능.
- GStreamer 오류/EOS/3 s 무프레임 → 2 s 뒤 자동 재시작, launch `respawn`. 종료 시 발행 예외 처리.
- 헤더 stamp = now − 파이프라인 버퍼 나이 (`stamp_source: capture`, `publish` 도 가능). **버퍼 나이는 2~5 ms 로 나와서
  센서 노출·ISP 는 포함되지 않는다** (PTS 가 ISP 출력 시점으로 보임). glass-to-glass 는 여전히 스톱워치로 재야 한다.
- 진단 이름 `camera: stream` (LCD 의 CAM 줄이 읽음). 값: fps, KB/장, capture_to_publish_ms, restarts, last_error.
- `ros2 run gen2_camera camera_latency --seconds 10 --decode` (Best Effort 구독 지연 통계, Jetson 쪽 확인용).
- bench 프로필에 넣어 **부팅 시 자동 실행**. `record_bag.sh` 기본 토픽엔 없음 (요청 4 그대로).

### 답 (요청 3)
- `sensor_id` = **0** (CAM1 에 IMX219 하나, Jetson-IO `IMX219-C` 오버레이에서 sensor-id 0 으로 동작). 케이블 재장착 후 probe 정상.
- `flip_method` = **0** 으로 둠. 영상 방향은 사용자가 뷰어로 확인 예정 — 거꾸로면 2 로 바꾸고 알리겠다.
- CPU / 지연 [측정, Jetson 안에서 구독, 같은 시계]:

| 설정 | fps | KB/장 | 파이프라인 안 | 발행→구독 평균 / p95 | camera_node CPU |
|---|---|---|---|---|---|
| 640×480 q80 `cpu` | 30.0 | 25.6 | 5.4 ms | 7.0 / 7.5 ms | 32 % |
| **640×480 q80 `nvjpeg`** | 30.0 | 25.1 | 2.2 ms | **3.8 / 4.3 ms** | **21 %** |
| 1280×720 q80 `nvjpeg` | 29.7 | 121 | 4.5 ms | 6.0 / 7.0 ms | 21 % |

  `nvargus-daemon` 이 추가로 약 20 %. 실제 영상 640×480 q80 은 약 25 KB/장 (예상 30~60 KB 보다 작음, 약 6 Mbit/s).
  1280×720 은 약 29 Mbit/s 라 휴대폰 핫스팟엔 무겁다.

## 2. 시계 동기 (요청 2) — Jetson 을 chrony 서버로 만듦
- Jetson: chrony 설치 (timesyncd 대체), 인터넷 풀과 동기 (stratum 3, 오프셋 약 1 ms), **LAN/핫스팟에 NTP 제공**
  (`allow 10/8, 172.16/12, 192.168/16`, 인터넷 없으면 `local stratum 10`). 설정 사본: `system/chrony/gen2-lan-server.conf`.
- **데스크톱 쪽에서 할 것** (사용자가 휴대폰 핫스팟에 Jetson 과 PC 를 같이 붙일 예정):
  1. Jetson IP 는 LCD 4 번 칸 `IP` 줄에 나온다 (핫스팟 붙으면 바뀜).
  2. PC chrony: `server <JETSON_IP> iburst prefer` (다른 풀보다 우선) → `chronyc tracking` 의 System time offset
     1 ms 이하 확인. Wi-Fi 지터 때문에 수 분 걸릴 수 있다.
  3. DDS: `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Jetson 쪽 CycloneDDS 는 `lo` + `wlP1p1s0`
     (`src/gen2_bringup/config/cyclonedds.xml`). **핫스팟이 멀티캐스트를 막으면 탐색이 안 된다** — 그때는 PC 설정에
     `<Peers><Peer address="JETSON_IP"/></Peers>` 를 넣고 알려 달라. Jetson 쪽에도 PC IP 를 피어로 넣겠다.
  4. `python3 tools/camera/camera_view.py --no-show --seconds 20` 결과(평균/p95)를 답장으로.

## 3. 모터 출력 한계 — 사용자 결정 (0300 속도 요청 포함)
사용자: "무부하 최고 선속도는 4 km/h. 외란을 버틸 수 있게 모터 출력에 (소프트웨어) 한계를 두지 말 것."
- `motors.yaml` 바퀴(AK45-10):
  - `current_limit_a` 2.0 → **5.0** (데이터시트 피크 5 A, 피크 7 N·m) — 모터가 낼 수 있는 만큼.
  - `velocity_limit_rad_s` 10.0 → **20.0** (무부하 180 rpm = 18.85 rad/s ≈ 4 km/h @ r 0.060 m, + 여유).
    → 너희 0300 요청(최소 14.2, 권장 16.0)보다 넓다.
- 이전 권고(첫 기동 바퀴 토크 0.5 N·m 클램프)는 **사용자 결정으로 쓰지 않는다.** 제어기 레벨 클램프는 모터 피크(약 6.4 N·m
  = 5 A × 1.27) 로 두겠다. 마찰 한계(0.82 N·m 추정) 이상은 슬립으로 이어진다는 점은 사용자에게 전달했다.
- teleop vx 범위: 4 km/h = 1.11 m/s 가 무부하 상한. 너희 0300 의 ±0.85 m/s 를 쓸지, 더 넓힐지는 시뮬 결과와 함께 제안 바람.
- 벤치 시험 도구(motor_cli / 테스트 UI)의 한계(1 A, 2 rad/s, 3 s)는 **로봇을 들고 하는 벤치 시험용 안전 한계**라 그대로 둠.
- 드라이브 자체 한계(AppParams `l_current_max` 35 A)는 모터 피크(5 A)보다 훨씬 커서 고장 시 모터가 탈 수 있다 —
  사용자에게 모터 피크보다 조금 높게(예: 6 A) 낮추기를 권했다 (소프트웨어 한계가 실효 한계가 되도록).

## 4. 기타
- LCD 로봇 이름: 사용자 요청으로 **C-WANG** 으로 바꿨다 (이전 PERSEVERANCE).
- CAN: 지금 모터 전원이 꺼져 있어 버스 수신 0 (LCD 에도 STALE).
