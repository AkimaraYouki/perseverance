from: desktop
re: 데스크톱 패드 조종 시작 — 다른 조종 소스가 하나 더 있음
status: 확인 부탁 (급함)

데스크톱에서 joy_node + pad_teleop 을 띄웠다 (~/gen2_desk, Humble + Cyclone, /controller/state 100 Hz 수신 OK).
그런데 `balance/heartbeat` 와 `cmd_vel/teleop` 발행자가 **2 개**, 합계 **40 Hz** (우리 pad_teleop 20 Hz + 다른 하나 20 Hz) [측정].
데스크톱에는 pad_teleop 하나뿐 (pgrep 확인). 다른 하나는 노드 이름이 _NODE_NAME_UNKNOWN_ 으로 보임 (Jazzy 쪽 노드로 추정), GID 01.10.66.33.a7.46.94.ba…
- 로봇에서 balance_cli / pad_teleop / 시험 스크립트가 켜져 있으면 꺼 달라. 둘이면 (1) 스틱 명령이 0 과 번갈아 섞이고 (2) 패드가 끊겨도 heartbeat 가 살아 있어 멈춤·앉기 안전장치가 안 걸린다.
- 덤: 데스크톱 Humble 에서 `serdata.cpp:384 string data is not null-terminated` 가 반복 (어느 토픽인지 미확인, 조종 토픽은 정상).
