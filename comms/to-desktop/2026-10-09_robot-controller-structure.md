from: robot
re: 사용자 요청 "제어기 구조 데스크톱에 보내"
status: 정보 — 시뮬 TUNE 대조용

실기 제어기 구조 전체 = **docs/controller_structure.md** (mermaid 흐름도 2 개 + 200 Hz 스텝 순서 + 현재 값 표 + 알려진 문제).
오늘 바뀐 것 중 시뮬에 넣어야 같아지는 것:
1. hip_mode **mit_pos** — 고관절 위치 루프가 드라이브 안 (p_des = 서보 위치 + (M_tgt − M), kp = vmc_kp·sc, kd = vmc_kd·sc, t_ff = 중력 ff). 호스트 200 Hz 지연이 kp 경로에서 빠짐.
2. 속도 기준 **S-curve** (tune.jerk_max 3 m/s³, accel_max 0.8): a_want = clip(sign(e)·√(2 J |e|), ±a_max), 가속도가 jerk 로 따라감, 목표 넘으면 고정.
3. **x_hold_moving false**: |vx 명령| > 0.02 면 x_err = 0 (주행 중 위치 따라잡기 없음, 스틱 놓으면 그 자리 유지).
4. heartbeat 1.0 s, 패드 vx_max 0.7, wz_max 2.0.
wbctrl 에 2·3 을 넣어 주면 golden 을 다시 맞추겠다 (지금 로봇 golden 은 jerk 0 / x_hold true 로 통과).
