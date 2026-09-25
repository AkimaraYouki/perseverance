from: robot
re: -
status: info

# 로봇 → 데스크톱: `MotorState.configured` 필드 추가 (미등록 모터 자동 발견) — gen2_msgs 다시 빌드 필요

사용자 요청: "모터 4 개인데 디스플레이는 3 개", "CAN 에 인식되면 자동으로 추가되나?"
- `gen2_msgs/msg/MotorState.msg` 에 **`bool configured`** 추가 (name, can_id 다음).
  데스크톱에 gen2_msgs 를 이미 빌드했다면 **다시 빌드해야 한다** (필드가 늘어서 역직렬화가 안 맞는다).
- motor_monitor: `motors.yaml` 에 없는 CubeMars 드라이브가 0x29 상태를 보내면 `/motors/state` 에
  `id_<N>?` (configured false, 원시값만: position_rad = 드라이브 각도[rad] 스케일 없음, velocity/torque/kt = NaN) 로
  자동 추가, 진단 `motor: id_<N>?` WARN. 명령은 여전히 **등록된 모터에만** 보낼 수 있다.
  vcan + 가짜 드라이브(id 69 등록, id 70 미등록)로 확인.
- LCD 모터 칸: 한 줄에 한 모터, 항상 4 줄 (등록 → 미등록 `#N?` → `waiting`). 위치는 화면에서만 ±180° 로 감아 표시.
- gen2-bench: `KillMode=mixed` — 노드가 SIGINT 를 두 번 받아 정리 중 죽던 문제 해결 (정지 2 s, traceback 0).
