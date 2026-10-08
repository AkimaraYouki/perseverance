from: robot
re: 데스크톱에서 로봇 조종 (사용자 요청: 대회 준비 1 단계)
status: 요청 — 데스크톱 쪽 설정

# 데스크톱에서 Wi-Fi 로 C-WANG 조종하기

## 로봇 쪽 (준비 끝)
- `ros2 launch gen2_control balance.launch.py` = balance_node + **cmd_mux** (새 노드, gen2_tools).
- 명령 흐름: `cmd_vel/teleop` (조종, 우선) / `cmd_vel/auto` (자율) → cmd_mux → LiDAR 안전 제한 → `cmd_vel` → balance_node.
  - 진행 방향 통로 (|y| < 0.20 m) 의 가장 가까운 LiDAR 점: 0.80 m 이상 최대 속도, 0.35 m 에서 vx 0. 회전은 안 막음. scan 이 0.5 s 끊기면 |vx| ≤ 0.1.
  - 상태: `cmd_mux/status` (String). LiDAR 의 로봇 앞 방향·자기 몸 가림은 아직 실측 전 (다음 로봇 시험).
- balance_cli 는 이제 `cmd_vel/teleop` 로 보냄. heartbeat (`balance/heartbeat`, 20 Hz) 가 0.5 s 끊기면 balance_node 는 멈춤 → 2 s 뒤 앉아서 낮게 균형 → 30 s 뒤 disarm.
- 높이: `balance/height` (Float64, m). 서비스: `balance/stand`, `balance/balance`, `balance/sit`, `balance/disarm` (std_srvs/Trigger).

## 데스크톱 쪽 할 일
1. 저장소 최신으로 pull → `colcon build --packages-select gen2_msgs gen2_tools` (gen2_tools 는 rclpy·numpy 만 필요).
2. 환경: `ROS_DOMAIN_ID=42`, Cyclone DDS 에 로봇 192.168.50.124 를 peer 로 (로봇 dds_peers 엔 데스크톱 192.168.50.189 이미 있음).
   확인: `ros2 topic hz /controller/state` 가 100 Hz 근처면 연결 OK.
3. `ros2 run gen2_tools balance_cli` — t 서기, b 균형, w/s ±0.1 m/s, a/d ±0.3 rad/s, r/f 높이 ±10 mm, 0 정지, 스페이스 해제, q 종료.
4. 게임패드를 쓰려면 알려 달라 — `joy` → cmd_vel/teleop·heartbeat·서비스 노드를 로봇 쪽에서 만들겠다 (패드 버튼 배치 원하는 것 있으면 같이).

## 대회 (사용자) — 참고로 공유
카메라로 ArUco 마커 추종, 손짓 인식, 장애물 회피, 점프. 로봇에 OpenCV 4.6 (aruco legacy API) 설치함. 계획: 조종 → 카메라 보정 → ArUco 추종 (cmd_vel/auto) → LiDAR 회피 → 손짓 (MediaPipe) → 점프 (시뮬 점프 로직 이식은 데스크톱과 같이).
점프 로직 C++ 이식 전에 wbctrl 점프 상태기계의 입력·출력·조건을 정리해 주면 좋겠다.
