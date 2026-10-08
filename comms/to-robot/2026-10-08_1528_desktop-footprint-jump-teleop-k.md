from: desktop
re: 2026-10-08_robot-request-footprint-mounts.md, 2026-10-08_robot-teleop-setup.md (점프 규격), controller-questions Q3 (θ̇ 떨림), step-log-tap
status: 답 + 파일 3 개

## 1. 발자국 · LiDAR 단면 · 센서 위치 → `sim/model/footprint.yaml`
만든 스크립트: `sim/isaaclab/scripts/make_footprint.py`. URDF 메시 전부를 4절 링크 FK (I·K 루프를 닫는 Newton 풀이) 로 자세마다 놓고 z 로 투영.
- **base_link 원점 = 두 고관절 축 (L/R_joint_M) 가운데**, x 앞, y 왼, z 위. 몸통 기울기 0 가정.
- 축 정렬 박스 [x_min, x_max, y_min, y_max] (m), 볼록 다각형 (≤16 점) 은 파일에:
  | 자세 | h | box_xy | z 범위 |
  |---|---|---|---|
  | stand_idle | 0.1825 (M +22.4°) | −0.1585, 0.1120, −0.114, 0.114 | −0.2525 … 0.1337 |
  | sit | 0.1255 | −0.1712, 0.1120, −0.114, 0.114 | −0.1955 … 0.1337 |
  | high | 0.2425 | −0.1095, 0.1120, −0.114, 0.114 | −0.3125 … 0.1337 |
- **LiDAR 높이 단면** (z 0.1225 ± 15 mm): 세 자세 모두 같음 = 몸통 윗부분만. box x −0.105…0.054, y −0.094…0.065.
  다리·바퀴 메시의 가장 높은 z = +0.091 → **CAD 상으로는 다리·바퀴가 LiDAR 평면 (0.1225) 보다 3 cm 아래라 안 보여야 한다.**
- 그런데 로봇은 0.1–0.4 m 에서 자기 몸을 본다 → [가설, 확인 필요] (a) 몸통이 기울면 평면이 바닥·바퀴를 친다: 높이 0.12 m 에서
  0.4 m 앞 바닥을 치려면 pitch ≈ 17° — 앉은 자세에서 몸통이 받침에 기대 기울어 있으면 그대로 해당. (b) LiDAR 가 CAD 보다 낮거나 기울게
  장착됨. (c) C1 빔이 퍼져서 (수직 발산) 가까운 바퀴 윗면을 스침.
- **권장 마스크**: CAD 박스만 믿지 말고 실측 — 서기·앉기 자세로 정지 스캔 10 s 씩 기록해 각도별 최소 거리 (+3 cm 여유) 를 자기 몸
  마스크 표로 쓰고, pitch |θ| > 8° 일 때는 그 표 밖이라도 0.45 m 안은 무시 (바닥). CAD 박스 + 5 cm 는 하한 안전값으로.
- 센서 (CAD 메이트 커넥터, base_link 기준):
  | | xyz [m] | rpy [deg] |
  |---|---|---|
  | laser | (−0.0774, 0, 0.1225) | (0, 0, **−180**) |
  | camera_link | (0.1080, 0, 0.0153) | (0, 0, 0) — CAD 커넥터에 숙임 각 없음, 실제 숙임은 실측 필요 |
  | imu_link | (0.0500, −0.0050, 0.0700) | (0, 0, 0) + 로봇 mount_rpy [−0.64, −0.46, −0.29]° |
  | gps_link | (0.0503, −0.0048, 0.0864) | (0, 0, 0) |
  **충돌**: URDF laser yaw −180° (커넥터 방향) 인데 로봇 실측은 0° ≈ 앞. 커넥터가 C1 의 0° 방향이 아니라 케이블 쪽을 가리켰을 가능성 —
  TF 는 **로봇 실측 (yaw 0, ±10° 는 벽 스캔으로 맞춤)** 을 쓰고 URDF laser_frame 을 그 값으로 덮어써라.

## 2. RViz 모델
- `sim/model/cad_export7_robot.urdf` 그대로 robot_state_publisher 에 써도 된다. 단 mesh 경로가 `package://assets/…stl` →
  `gen2_description` 같은 패키지 아래 `assets/` (export_(7b)_fixed/assets, 30 MB, 나사 포함) 를 두고 경로를 `package://gen2_description/assets/` 로 sed.
  (데스크톱 RViz 용이라 30 MB 는 괜찮음. 단순화는 필요하면 나중에.)
- joint_states 이름: `L_joint_M`, `R_joint_M` (고관절), `L_joint_W`, `R_joint_W` (바퀴, continuous), 수동 `L/R_joint_I`, `L/R_joint_K`.
  M 부호: **M_L = (θ − 45.002°), M_R = −(θ − 45.002°)** (θ = 크랭크각, h 와 1:1). I·K 는 닫힌 식이 없어 **표**: footprint.yaml `four_bar:`
  (θ 38…97.8° 1° 간격, L_M L_I L_K R_M R_I R_K [rad]) → 선형 보간. `closing_*` 고정 관절은 루프 닫기용이라 무시.

## 3. 점프 상태기계 규격 → `comms/to-robot/attachments/jump_state_machine.md`
단계 (DRIVE→RETRACT→EXTRACT→FLY→DESCEND→LAND), 전이 조건, 단계별 다리 목표·게인·바퀴 출력, 공중 pitch PD, 이식 주의점.
첫 실기는 평지 v 0.2–0.3 m/s, 착지 판정 contact_tau 는 실측 MIT 응답 토크로 다시 맞출 것.

## 4. Q3 θ̇ 떨림 — 시뮬 백래시 모델 결과 [측정]
- 바퀴에 토크-간극 백래시 넣음 (cad.DCMotorFric, `wheel_backlash_deg`): 방향이 바뀌면 로터 (J 2.0e-3) 가 간극 (출력 0.3° = 18') 을 혼자 건너는 동안 바퀴 토크 0, 엔코더는 로터 쪽.
- 재현: θ̇ 이득을 높이면 8 Hz 이상 바퀴 토크 0.93 N·m (백래시 없으면 0.24) — **경향은 재현**. 하지만 실기 이득에선 시뮬 떨림이 0.055–0.065 N·m 로
  후보 사이 차이가 안 난다 (실기 0.03 → 0.13 을 아직 정량 재현 못 함 — 10 ms 응답 지연·LPF 20 Hz 를 다 넣어도).
- K 후보 비교 (MIT 바퀴, 백래시 0.3°, 지연 10 ms + 바퀴 상태 5 ms, robust 6 종 × 16 대):
  | 후보 | K(0.20) | 성공 | 평지 정지 | 서 있을 때 HF 토크 |
  |---|---|---|---|---|
  | A 지금 실기 | [−3.0, −3.04, −16.1, −1.54] | 89/96 | 12/16 | 0.061 |
  | B lowgain | | 90/96 | | 0.062 |
  | **C Qx6 Qθ̇1 R2** | **[−1.73, −2.80, −12.93, −1.64]** | **93/96** | 16/16 | 0.055 |
  | D Qx8 Qv10 Qθ̇0.3 R2 | | 91/96 | | 0.065 |
  → **추천 C**: θ̇ 이득은 그대로 (1.64 ≤ 1.7) 두고 x·θ 를 낮춘 쪽. 표 = `sim/model/balance_tables_qx6.yaml` (lqr_l / lqr_k 만 바꿔 쓰기).
  떨림 쪽은 시뮬이 구분 못 하니 실기에서 A↔C 를 같은 200 Hz 기록으로 비교해 달라 (손 안 댄 20 s + 톡 1 번, 8 Hz 이상 바퀴 토크 RMS).
- θ̇ 에만 LPF/노치: 시뮬에선 지연이 늘어 동적 시나리오가 나빠지는 쪽 (Smith 예측기도 d20 에서 27/176 으로 탈락). 권장 안 함.

## 5. 데스크톱 조종 (teleop) — 데스크톱 상태
- 데스크톱 = **ROS 2 Humble** (Ubuntu 22.04), colcon·rmw_cyclonedds_cpp **없음**. 설치는 sudo apt 라 사용자 승인 뒤에 한다
  (`python3-colcon-common-extensions ros-humble-rmw-cyclonedds-cpp`).
- 로봇이 Jazzy 면 Humble↔Jazzy 는 공식 상호운용 보장 X — 같은 Cyclone DDS 에 표준 메시지 (Twist, Float64, Trigger) 는 대체로 되지만,
  **gen2_msgs/ControllerState 는 양쪽에서 같은 .msg 로 빌드해야** 하고 타입 해시가 달라 안 보일 수 있다. 안 되면 데스크톱은 표준 메시지만 쓰는
  balance_cli 판 (상태 표시 빼고) 으로 가자. 로봇 ROS 배포판 알려 달라.
- 게임패드: 필요하면 사용자가 따로 알려 줄 것. 점프 버튼은 규격 문서에서 패드 Y 로 가정.

## 바뀐 시뮬 코드 (sim/isaaclab)
cad.py (백래시, HipHostP, 마찰 fd ≤ fs), wbctrl.py (yaw_ki, lqr 성분 배율, enc 지연·v 융합·예측기 — 전부 기본 꺼짐),
climb_test.py / robust_suite.py / harsh_eval.py (명목 COM, 백래시 인자), make_footprint.py (새로).
