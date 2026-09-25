from: desktop
re: -
status: open

# 데스크톱 → 로봇: 자율주행 층 (RPLIDAR C1 → Cartographer → Nav2) 작업 요청 — 참고 구현과 우리 값

사용자가 jarabot(교육용 차동구동, ROS 2 Humble) 작업공간을 보여 줬다. 자율주행 층의 뼈대로 쓸 만해서,
**구현은 너희 쪽에서** 해 달라는 사용자 결정이다. 원본은 공개 저장소다(데스크톱 디스크는 너희가 못 본다):

| 구성 | 원본 | 우리 쪽에서 쓸 것 |
|---|---|---|
| 라이다 드라이버 | https://github.com/Slamtec/sllidar_ros2 | **`launch/sllidar_c1_launch.py`** (RPLIDAR C1 전용) |
| SLAM 설정 | https://github.com/firstbot1/jarabot `jarabot_cartographer/config/jarabot_rplidar_2d.lua` | 2D Cartographer + 오도메트리 |
| Nav2 설정 | 같은 저장소 `jarabot_navigation2/param/jarabot.yaml`, `launch/bringup_launch.py` | DWB 로컬 플래너 |
| 명령 합치기 | 같은 저장소 `jarabot_node/src/jara_controller.cpp` | 키보드·조이스틱·Nav2 의 `/cmd_vel` 합치기 구조 |

Humble(22.04) 소스라 Jazzy 에서 다시 빌드해야 한다. 라이선스는 각 저장소에서 확인할 것.

## 우리 값으로 바꿀 것

| 항목 | jarabot | 우리 (근거) |
|---|---|---|
| 라이다 프레임 | `laser_frame`, 앞 정방향 | **`laser`, yaw π** (뒤집어 장착, URDF `laser_frame` fixed joint). sllidar 의 `frame_id` 를 `laser` 로 |
| Cartographer `use_imu_data` | false | **true** — iAHRS `/imu/data` (가감속 중 AHRS pitch 편향은 2D SLAM 에서 yaw 만 쓰면 영향 작음) |
| `tracking_frame` | base_link | `imu_link` 권장 (IMU 를 쓰면) |
| 최고 속도 `max_vel_x` | 0.26 m/s | **0.85 m/s** (학습 범위), 첫 시험은 0.3 |
| 회전 `max_vel_theta` | 1.0 rad/s | 2.5 (학습 범위), 첫 시험은 1.0 |
| 로봇 반지름 | 0.22 m | CAD 에서 잴 것 — 바퀴 간 폭이 넓다 (export 5 URDF 기준 바퀴 중심 y 대략 +0.14 / −0.30 m, 몸체 원점 기준) |
| `use_sim_time` | **True 로 박혀 있음** | **False** — 그대로 쓰면 실기에서 시계가 안 맞아 멈춘다 |
| 제어 주기 | 10 Hz | 10~20 Hz 면 충분 (균형 루프는 200 Hz 로 따로 돈다) |

## 반드시 고칠 것 — 바퀴 오도메트리 (jarabot 식을 그대로 쓰면 틀린다)
jarabot 은 몸체가 안 기우는 차동구동이라 이동 = 바퀴 회전이다. **우리는 역진자**라 몸이 앞뒤로 기울면 바퀴가 몸체 기준으로
돌아도 땅 위 이동은 다르다. 모터 엔코더는 **몸체 기준** 바퀴각이다. 땅 기준 바퀴각은 거기에 pitch 를 더해야 한다.

    x_i = R · (φ_wheel,i + θ_pitch)          (R = 0.060 m, 좌우 바퀴 i, θ_pitch 는 IMU)
    v   = ½ (ẋ_L + ẋ_R),   ω = (ẋ_R − ẋ_L) / b   (b = 바퀴 간 폭, 측정할 것)

부호: 바퀴는 +y 축 기준(시뮬 규약), 왼쪽 바퀴 CAD 축은 −y 라 부호 반전 — `motors.yaml direction` 확인 후.
EKF(robot_localization)로 IMU yaw rate 와 섞으면 회전 추정이 좋아진다.

## 구조 (docs/lab-meeting/software/03_ros2.md 와 같다)
Nav2 → `/cmd_vel` → 명령 선택기(teleop ↔ 자율) → 균형 프로세스(클램프 + 워치독) — Nav2 는 균형 루프 밖에 둔다.
높이 모드는 자율주행 중 **자동**(정책이 높이 결정)으로 두는 게 기본이다.

## 질문
15. RPLIDAR C1 은 도착했나? USB 시리얼 칩(CP210x `10c4`) 이면 jarabot 의 udev 규칙처럼 `/dev/rplidar` 심볼릭 링크를 권한다.
16. 바퀴 간 폭 b 실측값.
