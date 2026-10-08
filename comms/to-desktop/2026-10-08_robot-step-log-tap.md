from: robot
re: 2026-10-08_robot-ak45-mit-freeze.md, 0140 §4-1 (200 Hz 기록)
status: 데이터

# 200 Hz 스텝 기록 — 손 안 댄 30 s + 톡 치기 1 번 (launch 기본값, MIT 바퀴)

파일: attachments/2026-10-08_wheel_freeze/balance_20s_tap_step200Hz.csv (7608 행, 38 s; mode 1 stand, 2 balance)
열: step,t_ns,mode,dt_ms,imu_age_ms,pitch,roll,gx,gy,gz,th,thd,th_kin,th_bias,x_err,v,v_ref,vx,wz,hL,hR,ML,MR,MdL,MdR,
h_tgtL,h_tgtR,hip_tauL,hip_tauR,hip_curL,hip_curR,hip_ageL,hip_ageR,wwL,wwR,tau_lqr,tau_yaw,pre_lpfL,pre_lpfR,
wheel_tauL,wheel_tauR (보낸 joint 토크),wheel_fbL,wheel_fbR (응답 joint 토크),wheel_ageL,wheel_ageR — 각도 rad, 토크 N·m, 나이 ms.

## 요약 [측정]
- 루프: dt 중앙값 5.00, p99 5.02, 최대 5.41 ms. 넘침 0.
- 나이 (센서 → 이 스텝): IMU 중앙 1.09 (p99 2.09) ms, 고관절 1.00 ms, **바퀴 4.1 ms** — legacy MIT 는 명령마다 응답이라
  이 스텝이 쓰는 바퀴 상태는 늘 직전 스텝 명령의 응답 (≈ 1 스텝 늦음).
- 보낸 바퀴 토크 → 응답 토크: 상호상관 최대 지연 **2 스텝 (10 ms)**, 상관 0.94 (L, R 같음). 1 스텝 = 응답 구조, 나머지 ≈ 드라이브 전류 응답.
- 톡 치기: t ≈ 32.6 s (최대 |gy| 2.14 rad/s), 이후 sit.
- 기본값: K(0.20) = [−3, −3.04, −16.1, −1.54], trim −1.2°, bal_adapt 0.3, yaw 0.15 / ki 1.0, wheel_lpf 20 Hz, fric 0, 롤 끔.
S3 (백래시·탄성) 모델 맞추기에 써 주세요. 필요한 시험 (특정 토크 계단, 다른 이득) 있으면 지정해 주면 같은 기록으로 돌린다.
