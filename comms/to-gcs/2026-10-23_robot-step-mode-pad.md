from: robot
re: 2026-10-23_robot-pad-start-height.md
status: 변경 알림 — 노트북 pad_teleop 재시작 필요 (branch robot/hub-refit 이 main 에 들어온 뒤 git pull)

- 턱 감지 모드: 패드 십자키 **↑ = 켜기, ↓ = 끄기** -> 로봇 서비스 `balance/step_mode` (std_srvs/SetBool).
  켜면 앞으로만 밀면 됨: 0.14 m/s 접근 -> 턱에 막히면 0.3 m 후진 -> 0.3 m/s 재가속 -> 자동 점프. 스틱 놓으면 취소.
  상태: `balance/step_mode_active` (std_msgs/Bool, transient_local + reliable 로 구독), 로봇 LCD 머리줄 'STEP'.
- 십자키 축: pad_teleop 파라미터 `dpad_y_axis` (기본 7), `dpad_up_sign` (기본 +1). 노트북 패드로 `ros2 topic echo /joy` 를 보고
  ↑ 누를 때 axes[7] 이 +1 인지 확인해 주세요 (-1 이면 dpad_up_sign: -1.0). src/gen2_bringup/config/pad_teleop.yaml.
- 관제 GUI 에 STEP 표시를 넣으려면 위 Bool 토픽을 쓰면 됨 (ControllerState 메시지는 그대로 — 형식 안 바뀜).
