from: robot
re: LiDAR 자기 몸 가림 · RViz 로봇 모델 (사용자 요청)
status: 요청

# CAD 에서 뽑아 주세요: (1) 로봇 몸을 z 로 투영한 평면 박스·다각형, (2) LiDAR·카메라·IMU 장착 위치, (3) RViz 용 URDF 정리

## 왜
- LiDAR 가 로봇 자기 다리·바퀴·몸통을 0.1–0.4 m 로 본다 (앉은 자세에선 반 바퀴 넘게 가림). 장애물 감속 (cmd_mux) 이 이걸
  장애물로 오인 → **로봇 형상을 LiDAR 평면에서 지울 마스크** 가 필요. 사용자 지시: "CAD 형상을 z 로 투영해서 박스를 만들어".
- RViz 에 로봇 모델 + 센서 좌표를 띄워 ArUco 추종·장애물 회피를 디버깅하려 함.

## 1. z 투영 발자국 (footprint) — base_link 좌표 (x 앞, y 왼, z 위, 단위 m)
- 로봇 전체 (몸통 + 다리 + 바퀴) 를 z 방향으로 납작하게 투영한 **xy 외곽**: 
  (a) 축 정렬 박스 [x_min, x_max, y_min, y_max] 한 줄, (b) 가능하면 볼록 다각형 꼭짓점 목록 (10–20 점).
- 다리 자세에 따라 바뀌므로 **서기 IDLE (h 0.1825, M +22.4°)**, **앉기 (h 0.1255)**, **최고 (h 0.2425)** 세 자세 각각.
- 추가로 **LiDAR 스캔 높이 (LiDAR 광학 중심 z)** 에서 자른 단면이 따로 있으면 그것도 (투영보다 마스크가 작아져 좋다). 바퀴 반지름 0.07 m 포함.

## 2. 센서 장착 위치 (base_link 기준 x y z [m], roll pitch yaw [deg])
- LiDAR (RPLIDAR C1) 광학 중심 + 0° 방향 (로봇 실측: 0° ≈ 앞, ±10°)
- 카메라 (IMX219, CAM1) 광학 중심 + 광축 방향 (아래로 몇 도 숙였는지)
- IMU (iAHRS) — 오늘 재고정함. 장착 보정은 mount_rpy [−0.64, −0.46, −0.29]° (10-07), 균형 trim −1.2° 로 맞춰 쓰는 중
- base_link 원점 정의도 같이 (몸통 기준점? 고관절 축 중간?)

## 3. RViz 용 모델
- sim/model/cad_export7_robot.urdf 를 그대로 robot_state_publisher 에 써도 되는지 (mesh 경로·관절 이름). 로봇이 발행할 joint_states 이름을
  URDF 관절 이름에 맞추겠다: 고관절 L/R (M 라디안, + = 다리 펴짐), 바퀴 L/R (회전각). 4-bar 의 수동 관절이 있으면 그 계산식도.
- 가능하면 mesh 를 가볍게 (단순화 STL) — 데스크톱 RViz 에서 볼 거라 로봇 성능엔 영향 없음.

받으면 로봇 쪽: cmd_mux 에 자기 몸 마스크, TF (base_link→laser/camera/imu, odom→base_link), joint_states 발행, RViz 설정 (gen2.rviz) 을 만든다.
