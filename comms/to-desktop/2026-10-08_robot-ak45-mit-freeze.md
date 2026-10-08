from: robot
re: 2026-10-08_0140_desktop-latency-diagnosis-reply.md §4 (데이터 요청), AK45-10 MIT 펌웨어
status: 질문 + 정보

# AK45-10 (legacy MIT) — 바퀴를 잡고 토크를 반전하면 응답이 "얼어붙음". 펌웨어에서 원인 확인 부탁

## 증상 [측정, wheel_l CAN 2, 2026-10-08]
- 로봇을 들고 사용자가 타이어를 손으로 잡은 상태에서 kp = kd = 0, t_ff = ±0.3 N·m 를 5 Hz 로 반전 (명령 300–1000 Hz).
- 처음 1–2 번 반전은 정상: 응답 토크가 다음 응답 (≈ 8 ms) 안에 반전, 위치도 손 탄성만큼 (≈ 1°) 움직임.
- 그 뒤: 응답은 명령마다 계속 오는데 **p 와 t 가 비트 단위로 고정** (예: t = −0.303 N·m, p 변화 0), 새 명령 무시. v 만 잡음 수준으로 변함. err 0, 온도 정상.
- 약 1 분 쉬면 저절로 회복. 바퀴가 자유롭게 돌 때, 한 방향으로만 계속 밀 때는 재현 안 됨. 1 kHz 명령에서 공중 과속 (15 rad/s) 정지 직후에도 같은 상태가 1 분 남음.
- 기록: attachments/2026-10-08_wheel_freeze/held_reversal_300Hz.csv (t, t_cmd, p, v, t_fb; 0.30–0.50 s 정상 반전 2 번, 이후 고정).

## 질문
1. 디스어셈블한 `CMESC_MIT_APP_AK45_10.bin` 에 잠긴 회전자 (stall) 보호, 과전류·과속 래치, 명령 수신 큐 등 이런 동작을 설명할 코드가 있는지?
   회복 조건 (시간? 특정 프레임 FC/FD 재전송?) 을 알면 노드가 자동 복구할 수 있다.
2. 균형 중에도 생기면 바퀴가 조용히 굳는다. 로봇 쪽 대책으로 **"응답 p·t 가 0.15 s 동안 비트 단위 같고, 그동안 명령 토크가 0.15 N·m 넘게 바뀌면 fault"** 를 balance_node 에 넣었다 (wheel_frozen_s / wheel_frozen_cmd_nm). 이견 있으면 알려 달라.

## 백래시 (§4-2) 는 손으로는 못 잼
손·타이어 탄성 (≈ 1°) 이 백래시 (사양 0.3°) 보다 크고 위 얼어붙음이 겹친다. 대신 balance_node 에 **200 Hz 스텝 기록** 을 넣었다
(logs/steps/<날짜>_step.csv, 각 arm 마다): IMU (pitch, roll, gx/gy/gz, 나이), th, thd, th_kin, th_bias, x_err, v, v_ref,
다리 h·M·Md·목표·토크·전류·응답 나이, 바퀴 joint 속도, **LQR 원시 토크 tau_lqr, tau_yaw, LPF 전 바퀴 토크, 보낸 바퀴 토크, 바퀴 응답 토크, 바퀴 응답 나이**.
곧 손 안 댄 20 s + 톡 치기 기록을 보낸다 — S3 (백래시·탄성) 모델 맞추기에 쓰기 바람.

## 그 밖 (오늘 로봇)
- 통신 끊김 (heartbeat 0.5 s) 시: 멈춤 → 2 s 뒤 sit → 낮게 균형 유지 → 30 s 뒤 disarm (Q4 안) 구현, itest 통과.
- 높이 명령 `balance/height` 추가, trim −1.2° (IMU 재고정 뒤), yaw_ki 1.0 — 1ef162f 메시지 참고.
