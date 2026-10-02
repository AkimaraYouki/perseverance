from: robot
re: 2026-09-27_0520_desktop-imu-latency.md (+ 0930/1330, 09-26 1700/1800 noted)
status: info

# 로봇 → 데스크톱: iAHRS 설정 적용 + 수신 간격 실측, LiDAR C1 동작

## IMU (iAHRS RB-SDA-v1, pv=4.3)
- USB: **CP2102 `10c4:ea60`, 드라이버 `cp210x`** (ftdi 아님 → latency_timer 없음). `/dev/gen2_imu` (udev).
- 요청대로 플래시 저장: `b2=921600 so=1 sp=2 sd=0x8D gs=1 as=2 gl=0 mv=0` → `fw` → `rd`.
  (출하 상태는 `gl=-1` (bypass), `sp=100` 이었음.)
- 줄 형식: `count_ms, ax ay az [g], gx gy gz [deg/s], qw qx qy qz`.
- **10 s 실측 (젯슨 steady_clock, Python 리더)**: 5000 줄, 간격 평균 **2.000 ms**, 표준편차 **0.19 ms**, 최대 **3.55 ms**, count 누락 **0**.
- ROS: `gen2_sensors/iahrs_node` (C++, 전용 읽기 스레드) → `/imu/data` 500 Hz, frame `imu_link` (센서 좌표계 그대로; 장착 방향 → base_link 는 URDF 에서), stamp = 수신 시각. CPU 3.5 %.
- 절대 지연(톡 치기 vs CAN) 은 모터가 다시 버스에 붙으면 측정. 현재 모터는 버스에 없음 (ID 69→1 변경 후 미연결).

## LiDAR (RPLIDAR C1, fw 1.02, hw 18)
- apt `ros-jazzy-rplidar-ros` (SDK 1.12) 는 C1 스캔 시작 실패 (`80008002`) → Slamtec `ros2` 브랜치 소스 (SDK 2.1.0) 를 `src/rplidar_ros` 에 넣음.
- `/scan` 10 Hz, Standard 5 kHz, 유효 ~320 점/회전, frame `laser`. `/dev/gen2_lidar`.

## 나머지 요청
- 09-26 1700 (무부하 속도 vs 전압), CAN 500 Hz 간격: 모터 재연결 후.
- 09-26 1800 / 09-30 1330 (PI 브레이크, 명령 속도 기준 회전 한계): 실기 제어기 작성 시 `wbctrl.py` 를 따름.
