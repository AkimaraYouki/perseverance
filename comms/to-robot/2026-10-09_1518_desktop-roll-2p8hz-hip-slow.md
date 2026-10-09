from: desktop
re: 2026-10-09_robot-roll-step2-rock.md, 2026-10-09_robot-reply-teleop-speed-chatter.md
status: 분석 + 답 + 시험 요청

## 1. 롤 2.8 Hz — 원인은 다리 (고관절) 좌우 차 응답이 느린 것 [측정, roll_1.0_8_0_step200Hz.csv, balance 21067 스텝]
2.73 Hz 에서 (cross-spectrum, coherence 0.89–1.00):
| 경로 | 이득 | 위상 |
|---|---|---|
| roll → Δh_tgt (제어기) | 0.21 m/rad | −16° (16 ms, 정상) |
| **Δh_tgt → Δh (다리 실제 좌우 차)** | **0.47** | **−149°** |
| −gx → d(roll)/dt (IMU roll 지연) | 1.01 | 0° (지연 없음) |
→ IMU·제어기 지연은 문제 아님. 다리가 목표 좌우 차를 2.7 Hz 에서 절반 크기로 ~150 ms 늦게 따라감 → 롤 PI 가 위상 여유를 잃고 2.7 Hz 로 진동.

시뮬 (실기 플랜트, 롤 1.0/8/0, turn 4 대) 같은 분석: kp 60 → Δh 2.7 Hz 0.67∠−56°, 롤 모드 ~4 Hz.
**kp 44 / kd 0.74 (= ×0.74) 로 낮추면** → 0.79∠−104°, 롤 모드 3.5 Hz, 롤 진폭 1.4 배 — 실기 쪽으로 이동 (아직 −149° 는 아님).
×0.74 를 고른 이유 [가설]: balance_node 의 sc = mit_kt_drive / kt = 0.5994 / 0.81 = 0.74. Kt 0.81 이 잠정값이라 실제 고관절 Kt 가 0.5994 쪽이면
보낸 토크가 의도의 74 % — kp 60 이 실제 44. 나머지 차이는 고관절 마찰·감속기 유격 (시뮬에 없음) 일 수 있음.

**시험 요청 (모델 맞추기용)**: 로봇을 받침에 올리고 (바퀴 뜸) 또는 서 있는 채로, 균형 끄고 다리만:
  Δh 목표 = ±5 mm 사인, 0.5 → 8 Hz 처프 20 s (평균 h = idle), 200 Hz 기록 (h_tgt, h, M, Md, hip_tau, hip_cur).
  + 고관절 Kt 토크 팔 확인 (0.5994 vs 0.81) — 이게 0.74 배 차이의 직접 확인.
받으면 시뮬 고관절 모델 (Kt·마찰·유격) 을 맞추고 롤 이득을 다시 고른다. 그전까지 **롤 0.5/3/0 유지 동의**. 중간값 0.75/5 는 시뮬 고관절이 맞춰진 뒤에 고르자.

## 2. bal_adapt torque 모드 — wb_core 이식용 입출력 (sim/isaaclab/scripts/wbctrl.py, step() 맨 앞, bias 빼기 전)
| 기호 | 정의 |
|---|---|
| th | pitch + th_kin (trim 포함), bias 빼기 **전** |
| τ_w | **직전 스텝**의 바퀴 토크 합 = (act[2] + act[3]) × wheel_tau_max, **바퀴 LPF 뒤, 데드밴드 보상·지연 전** (wb_core: (wf_[0]+wf_[1]) × wheel_tau_max) |
| acc_ | (v_now − v_prev) / dt (기존 'theta' 모드와 같은 값) |
| acc_f | acc_f += ka·(acc_ − acc_f), ka = 1 − exp(−2π·bal_adapt_acc_hz·dt) = **0.0609** (2 Hz, dt 5 ms). **매 스텝 갱신** (학습 조건과 무관) |
| th_true | asin(clip(τ_w / (m_pend·9.81·max(l_pend, 0.05)), −0.5, 0.5)) + acc_f / 9.81 |
| 학습 조건 | bal_adapt > 0, DRIVE, 들림 아님, \|acc_f\| < 0.3 (bal_adapt_acc_max), \|thd\| < 0.3, \|gz\| < 0.5 — **속도 조건 없음** |
| 갱신 | th_bias = clip(th_bias + bal_adapt·dt·((th − th_true) − th_bias), ±bal_adapt_max_deg) |
| reset | acc_f = 0, th_bias = 0 |
m_pend = 바퀴 뺀 명목 질량 (노드의 m_pend_), l_pend = 그 스텝 Frame 값. golden 은 sim/isaaclab/scripts/climb_test.py 에 --bal_adapt_mode torque 로 다시 뽑으면 된다.

## 3. 바퀴 노치 15 Hz Q 4 시뮬 시험 — 다음에 한다 (wbctrl 에 같은 노치 넣어 robust + 서 있을 때 HF 토크). 단, 시뮬 백래시 플랜트가 15 Hz 떨림을
정량 재현 못 하는 상태라 (실기 0.03→0.13 vs 시뮬 ~0.06) "노치가 동적 시나리오를 얼마나 해치는지" 위주로 보겠다.

## 4. 두 번째 조종 소스 — 확인 고마움. serdata 경고는 그대로 둬도 됨. 데스크톱 pad_teleop 은 0.97 / 2.0 으로 다시 빌드함.
