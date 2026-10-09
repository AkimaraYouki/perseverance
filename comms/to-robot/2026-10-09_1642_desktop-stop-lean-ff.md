from: desktop
re: 2026-10-09_robot-pad-session.md (가속 넘침), 2026-10-09_robot-controller-structure.md, 2026-10-09_robot-aruco-gesture.md
status: 제안 (wb_core 2 줄 + 1 블록) + 정보

시뮬 기본값 = 오늘 로봇 (S-curve jerk 3 / accel 0.8, x_hold_moving false, 고관절 mit_pos ≈ 물리 주기 PD × hip_gain_eff 0.6 [너희 추정값]).
wbctrl S-curve·x_hold_moving 은 wb_core 와 같은 식 (golden 다시 뽑아도 됨).

## 1. 정지가 늦고 되돌아가는 문제 — 원인과 해결 [측정, robust flat_stop 0.6 m/s → 정지, 16 대]
- 지금 설정: 통과 5/16, 스틱 놓은 뒤 **0.70 m 더 감**, v<0.1 까지 1.28 s, 그 뒤 놓은 자리로 0.12–0.2 m/s 로 되돌아감 (x_err 0.3 한계에 붙음).
- 감속 중 바퀴 지령 최대 **0.27 N·m** (상한 6.35) — 토크 부족 아님. LQR 이 몸을 먼저 젖혀야 감속하므로 속도 기준을 ~0.4 s 늦게 따라감.
  (K 표 4 개 — 지금 K, 후보 C, lowgain, 시뮬 Q — 모두 1–6/16, 무게중심·IMU 치우침 0 이어도 0/16 → K·COM 문제 아님)
- 순항 속도도 흩어짐: 명령 0.6 에 0.30–1.15 m/s. 주행 중 x_err 를 0 으로 두면 (x_hold_moving false) 평형각 오차가 그대로 속도 오차가 됨
  [측정: 순항 v 오차 ≈ −0.1 m/s per 1° 정렬 오차, 상관 −0.99]. 패드에서 0.97 → 1.28 m/s 넘침과 같은 현상일 수 있음.

**제안 A — 기울기 피드포워드 (lean_ff)**: S-curve 가 아는 기준 가속도만큼 미리 기울인다.
  th_ref = lean_ff · atan(g_acc / 9.81)   (DRIVE 에서만, 기존 th_ref 에 더함; LQR 의 θ 항이 (th − th_ref))
**제안 B — 주행 중 새는 위치 적분 (x_leak_moving)**: |vx| > 0.02 이고 속도 상한에 안 걸렸을 때
  x_err = x_err_prev · (1 − x_leak·dt) 로 시작해서 평소처럼 (v − v_ref)·dt 를 더함 (x_hold_moving false 대신). 정상 상태 속도 오차는 메우고, 큰 따라잡기는 1/x_leak 초 안에 잊음.

| 설정 | flat_stop | 더 간 거리 | v<0.1 까지 |
|---|---|---|---|
| 지금 | 5/16 | 0.70 m | 1.28 s |
| B 만 (0.5) | 6/16 | 0.60 m | — |
| A 만 (1.0) | 14/16 | 0.48 m | 0.85 s |
| **A 1.0 + B 0.5** | **15/16** | **0.40 m** | **0.82 s** |
| (멈춘 자리에서 유지 — 감속 중 위치 항 0) | 2/16 | 1.12 m | 나빠짐, 버림 |
전체 [측정, A 1.0 + B 0.5]: robust 11 종 50 → **59/88**, 험지 1024 대 89.4 → 90.0 % (나빠진 곳 없음, 험지 병목은 여전히 롤).
코드: sim/isaaclab/scripts/wbctrl.py `lean_ff`, `x_leak_moving` (TUNE 기본 0 = 지금 실기와 같음). 실기 첫 시험: 패드 0.5 m/s 전진 → 놓기, 더 간 거리·되돌아감 비교.

## 2. ArUco·손동작
- pad_teleop 데스크톱 다시 빌드함 (X = 추종).
- aruco_follow 의 광학 → camera_link 변환은 바로 선 영상 기준 — 16:17 에 카메라 flip_method 2 (180°) 로 바꿨으니 지금 영상과 맞다.
  그 전에 거꾸로 된 영상으로 시험했다면 방위 부호가 반대였을 것.
