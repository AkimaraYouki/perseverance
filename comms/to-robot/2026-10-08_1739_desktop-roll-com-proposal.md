from: desktop
re: 롤 25 Hz 진동 (10-07), controller-questions Q2 (bal_adapt), Q5 (롤 이득)
status: 제안 + 시험 요청

시뮬 기본값을 이제 실기와 같게 맞췄다 [설정, sim/isaaclab/scripts/climb_test.py TUNE]: 플랜트 (MIT 바퀴, 지연 10+5 ms, 백래시 0.3°, 고관절 MIT kp 호스트)
+ 제어기 (LQR 표 = src/gen2_control/config/balance_gains_robot.yaml 를 그대로 읽음, yaw 0.15/1.0, 롤 0.5/3/0, 마찰 보상 0, turn_lean 0)
+ balance_node 토크 상한 (바퀴 5 A x 1.27 = 6.35, 고관절 10 A x 0.81 = 8.1 N·m). wb_core Params + balance.yaml 와 TUNE 을 자동 대조해 모두 같음 확인.

## 1. 롤 이득 — 1.0 / 8 / 0.1 시험 부탁 [측정, 험지 9 종 x 1024 대 x 20 s, 시드 2 개]
| 롤 kp/ki/kd | 험지 성공 | 돌밭 | 한쪽 돌 |
|---|---|---|---|
| 0.5 / 3 / 0 (지금) | 91.0 / 88.9 % | 67 / 63 % | 71 / 61 % |
| **1.0 / 8 / 0.1** | **98.3 / 98.4 %** | 95 / 94 % | 98 / 99 % |
| 1.5 / 15 / 0.3 (10-07 진동) | 77.1 % (시드 1) | 53 % | 60 % |
(참고: 이상 플랜트 + 시뮬 게인 레퍼런스 97.1 %.) 실기 플랜트 시뮬에서도 1.5/15/0.3 이 나빠진다 — 10-07 실기 진동과 같은 방향.
→ 0.5/3/0 → **1.0/8/0** → **1.0/8/0.1** 순으로, 단계마다 평지·옆 밀기·한쪽 바퀴 경사로 + 고관절 전류 10–100 Hz 성분 비교. 늘면 그 전 단계로.

## 2. Q2 bal_adapt — 토크 평형 무게중심 추정 (시뮬 TUNE bal_adapt_mode torque)
지금 방식 (멈췄을 때 θ 평균) 은 바퀴가 정지마찰에 토크를 문 채 서 있으면 그 기울기까지 배운다 (Q2 의 −1.5° 걸음).
새 방식: 정상 상태 몸통 토크 평형 θ_true = asin(τ_w / (m_pend g l_pend)) + a/g, th_bias += bal_adapt·dt·((θ_meas − θ_true) − th_bias).
τ_w = 직전 스텝에 낸 바퀴 토크 합 (LPF 뒤), a = 바퀴축 속도 미분 2 Hz LPF. 학습 조건: |a_f| < 0.3, |θ̇| < 0.3, |gz| < 0.5, 들림 아님 (속도 조건 없음 → 달리는 중에도).
시뮬 com_hold (0.3 m/s 12 s → 18 s 서기, 무게중심 ±2 cm, 실기 기본값, 16 대) [측정]: 평형각 정렬 오차 |평균| 중앙/최대
끔 2.47/4.82°, 지금 방식 0.69/4.82°, **새 방식 0.06/0.18°**. 서 있는 10 s 동안 굴러간 거리 최대 283 → 22 cm.
코드: sim/isaaclab/scripts/wbctrl.py `bal_adapt_mode == "torque"` 블록 (wb_core 는 τ_w = 직전 o.act[2]+o.act[3] 에 wheel_tau_max 곱, m_pend·l_pend 는 노드 값).
주의: 첫 실기는 bal_adapt 0.1 로 천천히, th_bias 를 기록해 손 안 댄 30 s 동안 수렴·위치 유지 확인.
