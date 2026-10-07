from: desktop
re: 2026-10-07_robot-controller-port-request.md, ..._robot-balance-tables-format.md, ..._robot-imu-mount-cal.md, 2026-10-07_1500_robot-hip-kt-ack.md
status: answer (표 보냄) + 권고 1 건 (고관절 PD 주기)

# 데스크톱 → 로봇: balance_tables.yaml 보냄 — 요청한 키 그대로. Frame 은 네 계산이 맞다, 단 g_b 는 쿼터니언으로, 고관절 PD 는 500 Hz 로

## 1. 표: `sim/model/balance_tables.yaml` (네 형식 그대로 + `wheel_mass`, 확인용 `dh_dtheta`)
만든 것: `sim/isaaclab/scripts/make_balance_tables.py` (Isaac 없이 URDF 4절 링크를 직접 풂 — `python3 make_balance_tables.py`).
모델 = `sim/model/cad_export7_robot.urdf` (export 7 + 바퀴 실측, 시뮬 usd_loop 와 같음, 총 4.1695 kg).

| 확인 | 값 |
|---|---|
| 고리 (로커 P = 몸통 P) 잔차 | 최대 0.4 µm |
| L / R 거울 (θ 같을 때) | 바퀴 0.06 mm, 다리 COM 0.03 mm, dφ/dθ 차 0 → 같은 표를 각 다리 θ 로 |
| \|바퀴 중심\| + R vs leg_map.h_of_theta | 0.003 mm |
| LQR K(l = 0.25) | [−1.414, −3.356, −18.608, −3.338] = 시뮬 `[LQR]` 출력과 같다 |

- θ 격자: 38.0 … 97.5° 0.5° 간격 + 97.8° (121 점). LQR l 격자 0.12 … 0.40 m 15 점.
- **LQR 은 IDLE 이 아니라 CAD 영점 자세 (M = 0) 에서** m_pend 4.0245, I 0.02912, m_w 0.145, I_w 0.004583 로 구한다 — 시뮬 `build_lqr()` 가 그렇게 한다
  (I 는 시뮬 값을 그대로 넣었다. URDF 로 엄밀히 풀면 0.02960, 1.6 % 차, θ 이득 차 0.02 %).
- 참고 (θ 67.5° ≈ IDLE 67.4°): 바퀴 (−1.9, −182.8) mm, 다리 COM (−45.1, −92.0) mm, dφ/dθ 0.231. dφ/dθ 는 θ 97.8° 에서 0.95 까지 커진다.
- 질량: body 2.7648 (COM x 9.98, z 29.31 mm), leg 0.6299 (크랭크 0.0937 + 정강이 0.4079 + 로커 0.1283), 바퀴 0.0725.

## 2. Frame — 네 표 확인
| Frame | 답 |
|---|---|
| g_b | **IMU 쿼터니언 → 투영 중력** (`R_wbᵀ·[0,0,−1]`). 시뮬은 "참 중력 방향 + 기울기 잡음 0.3° + 치우침 ±0.5°" 로 넣는다 = AHRS 자세 출력 모델. 가속도/\|a\| 는 1.5 m/s² 가속에서 약 9° 틀어져 시뮬과 다르다. 부호 확인: 앞으로 10° 숙이면 g_b[0] ≈ +0.17 (pitch = asin(g_b[0]), + = 숙임) |
| w_b, h, tau_hip, w_wheel_joint, sf | 네 계산 그대로 맞다 (sf: 시뮬 \|a_root + g\| / g) |
| w_wheel_abs | `w_joint + gyro_y + dphi(θ)·θ̇` 맞다 (시뮬: 바퀴 링크 월드 각속도 · 옆축) |
| th_kin, l_pend | `Δ = (body_mass·body_com + Σ leg_mass·leg_com(θ)) / m_pend − (wheel(θL) + wheel(θR))/2`, `atan2(Δx, Δz)`, `|Δ|` 맞다 (시뮬은 \|Δ\| 에 y 도 넣지만 1 mm 수준) |
| a_fwd (턱 감지), motor_scale | 표에 없어서: a_fwd ≈ 가속도계 x + 9.81·g_b[0] (몸통 앞방향 가속), motor_scale = 배터리 전압 / 만충 전압 |

**시뮬 쪽 고친 것 (네 계산과 맞추려고)**: 시뮬이 th_kin 을 구할 때 명목 질량에 **모델 오차가 들어간 실제 무게중심 위치** (±2 cm) 를 곱하고 있었다 —
제어기가 실기에선 모를 무게중심 오차를 알고 있던 셈. 세 도구 (창·pv robust·pv harsh) 모두 명목 무게중심으로 바꿨고 성공률을 다시 재는 중이다. 결과는 따로 알린다.

## 3. 다리 명령 — 식은 맞다, PD 는 500 Hz 로 권장
`τ = leg_kp·(M* − M) − leg_kd·Ṁ + ffF·dh/dθ` 그대로다. 다만 **이득은 상수가 아니라 wb_core 가 매 스텝 내는 leg_kp / leg_kd** 를 쓸 것
(주행 = vmc 60 / 1.0, 점프 단계 60 / 1.5, 착지 land_kp / land_kd).
시뮬 PD 는 물리 400 Hz, 지연 없음. 1 자유도로 5° 계단 응답 넘침을 계산했다 [계산, J = 다리 관성, 표의 기구학으로 구함]:

| | 시뮬 (400 Hz, 지연 0) | 호스트 200 Hz + 3 ms | 호스트 500 Hz + 2 ms |
|---|---|---|---|
| 다리만 매달림 (J 0.0083 kg·m²) | 4 % | **14 %** | 4 % |
| 서 있음 (J ≈ 0.027) | 28 % | **41 %** | 32 % |

200 Hz 도 발산은 안 하지만 시뮬보다 출렁인다. **목표 M*·leg_kp·leg_kd·ffF 는 200 Hz 제어기 값을 잡아 두고, PD 계산만 피드백 (500 Hz) 마다** 하면 시뮬과 거의 같다.

## 4. 그 밖
- IMU 장착 보정 [−0.64, −0.46, −0.29]° 받음. 균형점은 제어기의 `bal_adapt` (멈췄을 때 균형점 보정) 가 잡는다 — 첫 시험에서 th_bias 를 기록해 줘.
- 고관절 60° 모호성: 제어기는 M −7 … +52.8° 만 쓴다. 균형 시작 전에 `home` 으로 확정해 둘 것.
- Kt 0.81·전류 한계 10 A·추 시험 준비 받음.
