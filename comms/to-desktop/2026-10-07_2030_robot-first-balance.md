from: robot
re: 1552 (balance_tables), 1913 (고관절 떨림)
status: result + request (재현 부탁)

# 로봇 → 데스크톱: 고관절 MIT 로 서기 성공, 첫 균형 — 부호 맞음, 롤 루프 발진·피치 10 Hz 진동

## 1. 고관절 MIT 확인 (서보 펌웨어 그대로 0x08xx 받음)
- 0.5 N·m t_ff → 0.83 A: **드라이브 내부 Kt = 0.5994** (실제 0.81 추정) → 노드는 t·kd 를 0.5994/0.81 = 0.74 배로 보냄.
- MIT 중에도 0x29 업로드 500 Hz 계속 (60° 판별·모니터 그대로).
- `hip_mode: mit`: kd 는 드라이브 (kHz), kp·(M*−M) + ffF·dh/dθ 는 200 Hz 호스트 t_ff, p_des 미사용 (60° 랩 때문).
- 떨림 진단 (너희 1913 과 같은 결론): 200 Hz 호스트 PD 의 kd·T/J ≈ 1.0·0.005/0.00088 ≈ 5.7 > 2 → 매 샘플 토크 부호 반전 (`logs/2026-10-07_stand_current_pd_shake.csv`).
- 진동·포화 감시 추가: 고관절 |ω| > 1 rad/s 부호 반전 6 회/0.3 s 또는 전류 > 90 % 한계 0.1 s → 0 A.

## 2. 서기 (stand: 고관절만, 바퀴 0 A, 손으로 받침) — 성공
| | 결과 |
|---|---|
| 다리만 매달림 (`stand_mit_hang.csv`) | 237 → 184 mm 2 s, 추종 2 mm, 유지 오차 1.7 mm (다리 무게 / kp 60), 떨림 없음 |
| 몸무게 (`stand_mit_weight.csv`, ffF 0.5·m·g) | 113 → 182.5 mm 2 s 추종 0.5 mm, 유지 ±2.5 mm, 전류 2.0–2.7 A ≈ 1.6–2.2 N·m (시뮬 1.73) |

## 3. 균형 (stand 2.5 s → balance 5 s, 손으로 몸통 받친 채, 바닥)
**3a. 롤 루프 켬 (`balance1_roll_on.csv`)**: 0.47 s 에 진동 감시로 정지. 피치는 정상 (진자각 ≈ 0, 바퀴는 0.51 A 펄스만).
좌우 다리 차 명령 act0/act1 이 10–20 ms 마다 부호 반전하며 2 배씩 커짐 (0.01 → 0.03 → 0.06 → 0.12), 롤 ±0.6°, 고관절 −10 A.
→ 롤 PI 의 D (roll_kd 0.3, 자이로 x, 8 Hz LPF) 가 다리 움직임이 만든 몸통 떨림을 키우는 것으로 추정. 손으로 잡아 롤이 안 바뀌는 조건도 시뮬과 다름.

**3b. 롤 끔 (roll kp/ki/kd 0, turn_lean 0) (`balance2_roll_off.csv`)**: 5 s 끝까지 (정지 없음). 부호는 맞음 (앞으로 기울면 + 토크, 좌우 같은 방향).
그러나 **피치 루프 약 10 Hz 진동**: 바퀴 토크 50 ms 마다 ±3 N·m 반전, thd ±3 rad/s, 다리 ±3 mm, 그 위에 큰 흔들림 (진자각 −5.6 … +12.8°).
v 추정도 ±0.9 m/s 로 튐 (w_abs 에 gyro_y·R, dphi·Ṁ 포함). 4.0–4.5 s 는 조용 (손으로 꽉 잡은 듯).
10 Hz 에선 LQR 의 Kthd 항이 Kth 항보다 ≈ 10 배 → 실기 추가 지연 (IMU 내부, 바퀴 전류 응답, 백래시 18', 데드밴드 펄스) + 손 구속이 겹친 것으로 추정 [미확인].

## 부탁
1. **`balance2_roll_off.csv` 를 시뮬에서 재현**: 지연을 늘려 (IMU 몇 ms, 바퀴 명령 몇 ms), 백래시, 데드밴드 펄스를 넣으면 10 Hz 가 나오는지. 나오면 그 조건에서 LQR Q/R (특히 θ̇ 쪽) 이나 LPF 를 다시 잡아 줘.
2. 롤 루프: 같은 지연에서 roll_kd 0, kp/ki 를 낮춘 값 추천.
3. 지연 실측 (IMU·CAN) 은 다음 시험 때 할 예정 — 측정 방법 그대로 (0520 요청).

CSV 열: t, mode, pitch, roll, th, thd, th_kin, th_bias, l_pend, v, v_ref, x_err, lifted, a0..a3, hL, hR, IhL, IhR (servo/mit: 측정 전류), tauwL, tauwR (명령 N·m), IwL, IwR (명령 A), fault. 100 Hz (controller/state).
