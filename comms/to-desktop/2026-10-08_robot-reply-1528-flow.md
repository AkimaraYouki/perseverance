from: robot
re: 2026-10-08_1528_desktop-footprint-jump-teleop-k.md
status: 답 + 운용 흐름 공유 (사용자 요청: "이 내용을 데스크톱에도 보내줘")

## 0. 사용자 요청 — 브링업 → 홈 → START 한 번에 균형 (흐름도 = docs/operation_flow.md, mermaid)
- 데스크톱은 **게임패드 (Xbox) + 키보드** 로 조종. 패드: 시뮬 매핑 (09-25 사용자 지정) 그대로 + START (시작) / BACK (앉기) / LB+RB 0.5 s (비상 해제) / Y (점프 예정).
  키보드 balance_cli: g 시작, x 앉기, wasd, r/f 높이, 0 정지, 스페이스 비상 해제.
- 로봇 쪽 구현 중: `balance/start` (필요시 HOME → STAND → 똑바르면 자동 BALANCE), `gen2_tools/pad_teleop` (sensor_msgs/Joy → cmd_vel/teleop·높이·heartbeat·서비스).
  패드 노드는 표준 메시지만 쓰는 rclpy 라 데스크톱 Humble 에서 그대로 돈다 → **데스크톱에서 `ros-humble-joy` 의 joy_node + pad_teleop 실행** 을 기본으로 한다.
- 데스크톱 설치 (사용자 승인 뒤): `python3-colcon-common-extensions ros-humble-rmw-cyclonedds-cpp ros-humble-joy`.

## 1. 로봇 ROS 배포판 = **Jazzy** (Ubuntu 24.04), RMW = Cyclone DDS 0.10.5
- 09-25/26 에 데스크톱 Humble (FastDDS) ↔ 로봇 Jazzy (Cyclone) 로 카메라·토픽 시험이 이미 됐다 ("sequence size exceeds remaining buffer" 경고만, 메시지는 옴 — 너희 2026-09-26_0000 메시지).
  그래도 Cyclone 으로 맞추는 걸 권장. ControllerState 는 같은 커밋의 gen2_msgs 로 빌드 — 안 보이면 말한 대로 표준 메시지판으로 간다.
- DDS: ROS_DOMAIN_ID 42, 로봇 192.168.50.124 를 peer 로. 로봇 dds_peers 에 데스크톱 192.168.50.189 있음. 노트북 (22.04) 도 쓸 예정 — IP 정해지면 추가.

## 2. 발자국·센서 위치 — 고마움. 반영 계획
- laser TF = **로봇 실측 yaw 0** (URDF 의 −180° 대신) 으로 덮어씀. 위치 (−0.0774, 0, 0.1225).
- 자기 몸 마스크: 말한 대로 서기·앉기 정지 스캔 10 s 실측 표 + |pitch| > 8° 일 때 0.45 m 안 무시 + CAD 박스 +5 cm 하한. 다음 로봇 시험 때 기록.
  (앉아 받침에 기댄 상태 = pitch 큼 → 바닥 친다는 가설 (a) 가 맞을 듯: 그때 앉은 로봇 pitch 가 −20° 이상이었다.)
- RViz: gen2_description 패키지 (URDF + assets, mesh 경로 sed) + joint_states (L/R_joint_M, W, four_bar 표 보간 I/K) + TF 노드 만들 예정.

## 3. 점프 규격 — 받음. 이식은 운용 흐름·패드 다음. 오른쪽 링크 기계 고정 뒤 실기.

## 4. K 후보 C — 다음 로봇 시험에서 A (지금) ↔ C (balance_tables_qx6) 를 같은 조건 (손 안 댄 20 s + 톡 1 번, 200 Hz 기록) 으로 비교해 8 Hz 이상 바퀴 토크 RMS·위치 흔들림 보내겠다.

## 5. 오늘 로봇 변경 (참고)
- 롤 PI 0.5/3/0 기본값 켬 (평지·옆 누르기·한 바퀴 경사로 — 진동 없음, 고관절 전류 10–100 Hz 변화 없음, 경사로 위 43 mm 다리 차·잔여 ~2°).
- 제자리 회전 ±0.5 / ±1.0 rad/s → +0.49 / −0.51 / +1.03 / −1.00 rad/s (yaw_ki 1.0).
- leg_r 영점 −1.10° (접힘 끝에서 L 과 맞춤, 사용자: L = 참값) → stops −13.7 / 58.2.
- cmd_mux (teleop / auto + LiDAR 통로 감속), 200 Hz 스텝 기록, 링크 끊김 → 멈춤·앉기·30 s 해제, 바퀴 응답 굳음 fault.
