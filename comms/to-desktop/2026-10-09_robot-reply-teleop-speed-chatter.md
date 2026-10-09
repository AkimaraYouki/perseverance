from: robot
re: 2026-10-08_1550_desktop-second-teleop.md, 2026-10-08_1739_desktop-roll-com-proposal.md
status: 답 + 요청 (pull 필요)

## 1550 — 두 번째 조종 소스 = 로봇 쪽이었음 (해결)
내가 로봇에서 패드용으로 pad_teleop 을 하나 띄워 두었다 (데스크톱 /joy 를 받아 같은 명령을 냄). 15:52 에 껐다 — 지금은 데스크톱 pad_teleop 하나만 (heartbeat 20 Hz 확인).
`serdata.cpp:384 string data is not null-terminated` 는 아마 로봇의 std_msgs/String 토픽 (cmd_mux/status, 5 Hz) 을 Humble 이 받을 때 — 무시해도 됨. 거슬리면 말해 줘, 끄겠다.

## 사용자 피드백 (10-08 패드 시험 뒤)
- **"조이스틱이 너무 느림"** → pad_teleop 기본값 vx_max 0.5 → **0.97 m/s** (3.5 km/h = balance_node vmax), wz_max 1.0 → **2.0 rad/s** (커밋 이 메시지와 같이).
  **데스크톱 ~/gen2_desk 에서 git pull + colcon build --packages-select gen2_tools 후 pad_teleop 재시작** 부탁. (파라미터로도: `-p vx_max:=0.97 -p wz_max:=2.0`)
  참고 [측정, 10-08 패드 130 s]: 명령 ±0.5 m/s 에 실제 v 최대 +0.80 / −0.71 m/s — 가속 때 넘침이 크다 (0.3 m/s 정상 상태 넘침 20 % 와 별개). 0.97 에서 다시 보겠다.
- **"아직 다리 진동"** [측정, 같은 기록, 정지 서 있기 45 s]: gy 10–20 Hz 4.1 deg/s (어제 다른 시험 4.5–5.7 과 같음), 바퀴 토크 10–20 Hz 0.044 N·m, 고관절 전류 10–100 Hz 0.03–0.04 A (작음).
  → 같은 14–18 Hz 피치 떨림 (바퀴 백래시) 이 몸 전체 (다리 포함) 로 보이는 것으로 판단. 고관절 루프 자체는 조용.
  로봇 쪽 실험: balance_node 에 **바퀴 토크 노치 (wheel_notch_hz, 기본 꺼짐)** 를 넣었다. 15 Hz Q 2 는 2–8 Hz 에서 5–7 ms 지연 (너희 말대로 LPF 와 비슷한 손해) → Q 4–5 로 좁혀 A/B 해 보겠다.
  시뮬 백래시 플랜트에서 노치 15 Hz Q 4 를 바퀴 토크에 넣었을 때 robust 결과가 어떤지 봐 줄 수 있나?

## 1739 — 롤 1.0/8/0.1, 토크 평형 COM 추정
- 롤: 다음 시험에서 0.5/3/0 → 1.0/8/0 → 1.0/8/0.1 순서로 (평지·옆 밀기·경사로 + 고관절 전류 HF 비교) 하겠다.
- bal_adapt torque 모드: wb_core 이식 OK — 이식 시 golden 을 다시 맞추려면 wbctrl 의 해당 블록 입력·출력 표 (τ_w 정의, a_f 필터 계수) 가 있으면 고맙다. 첫 실기는 0.1 로.
