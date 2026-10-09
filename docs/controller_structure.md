# C-WANG 실기 제어기 구조 (2026-10-09 기준, 로봇 기본값)

코드: `src/gen2_control/src/balance_node.cpp` (노드, 200 Hz RT 루프), `src/gen2_control/src/wb_core.cpp` (wbctrl.py DRIVE 경로 C++ 이식, golden 시험),
`src/gen2_tools/gen2_tools/cmd_mux.py`, `pad_teleop.py`. 설정: `config/balance.yaml`, `balance_tables.yaml` (데스크톱 모델), `balance_gains_robot.yaml` (실기 LQR).
부팅 때 `gen2-balance.service` 가 balance_node (DISARMED) + cmd_mux 를 rtprio 95 로 띄운다.

## 1. 전체 신호 흐름

```mermaid
flowchart LR
  PAD[데스크톱 joy_node + pad_teleop<br/>Joy → Twist·높이·heartbeat 20 Hz] -- Wi-Fi DDS --> MUX
  KB[balance_cli 키보드] --> MUX
  AUTO[자율: cmd_vel/auto<br/>ArUco 등 예정] --> MUX
  MUX[cmd_mux<br/>teleop 우선 · LiDAR 통로 감속 (현재 꺼짐)] -- cmd_vel --> BN
  PAD -- balance/height, heartbeat, start/sit/disarm --> BN
  IMU[iAHRS 500 Hz<br/>shm seqlock, 나이 ~1 ms] --> BN
  CAN[(CAN 1 Mbit)] <--> BN
  BN[balance_node 200 Hz<br/>모드 · 가드 · 감지 · 출력] --> CORE[wb_core.step<br/>속도 기준 · LQR · yaw · 롤 · VMC 높이]
  CORE --> BN
  BN -- MIT V3 p/kp/kd/t_ff --> HIP[AK60-6 고관절 x2<br/>servo 펌웨어, MIT V3 프레임]
  BN -- legacy MIT t --> WHL[AK45-10 바퀴 x2<br/>legacy MIT, 명령마다 응답]
```

## 2. 200 Hz 한 스텝 (balance_node::step)

1. **감지**: IMU (shm, 없으면 DDS) → 몸통 투영 중력 g_b, 자이로 w_b, pitch/roll. 모터 4 개 피드백 (고관절 = 서보 업로드 500 Hz, 바퀴 = 직전 명령의 MIT 응답, 나이 ~4 ms).
   고관절 각 M (4절 표로 h, θ), 바퀴 joint 속도, 바퀴 절대 속도 = joint + 몸통 pitch rate + 정강이 회전. th_kin·l_pend (데스크톱 CAD 무게중심 표) + **th_trim −1.2°**.
2. **안전 검사** (stand/balance 중): 모터·IMU 신선 (50 ms), 드라이브 에러·온도, 고관절 영점, **바퀴 응답 굳음** (p·t 비트 동일 0.15 s + 명령 0.15 N·m 변화),
   고관절 진동 (|ω| > 1 rad/s 부호 6 번 / 0.3 s), 고관절 전류 포화 (> 0.9×10 A 0.1 s; 앉는 중 > 5 A), 기울기 > 45° → FAULT (0 A).
3. **조작자 링크**: heartbeat **1.0 s** 끊김 → (balance 중) vx, wz = 0 → 2 s 뒤 sit → 낮게 균형 유지 → 30 s 뒤 disarm. (stand 중이면 FAULT.)
4. **모드**: DISARMED / STAND / BALANCE / FAULT. **START** = (고관절 영점 애매하면 HOME: −0.4 N·m 접기, 멈춤 0.3 s → stop_min 으로 확정) → STAND (2 s 관절 램프) → |pitch|,|roll| < 5° 면 BALANCE.
   START 는 이미 서 있을 때 무시. SIT = 높이를 3 s 에 바닥 높이로 → disarm.
5. **wb_core.step** (아래 3 장) → 다리 목표 h_tgt, 바퀴 토크 L/R.
6. **출력**:
   - 고관절 `hip_mode: mit_pos` — MIT V3: **p_des = 서보 업로드 위치 + (M_tgt − M)** [드라이브 출력 rad, 실측 확인], **kp = vmc_kp·sc**, **kd = vmc_kd·sc**, t_ff = 중력 ff (0.5 m_pend g · dh/dθ)·dir·sc.
     sc = 0.5994 / 0.81 (드라이브 내부 Kt / 실제 Kt). 위치 루프가 드라이브 안 (kHz). [실측 힌트: 드라이브 kp 실효 ≈ 명령의 0.6 배, mit_kp_scale 1.0 그대로]
     (이전 `mit`: 호스트 kp 200 Hz + 드라이브 kd → 2.8 Hz 에서 다리 위상 −150°, 롤 1.0/8 에서 좌우 흔들림.)
   - 바퀴 — legacy MIT 토크 프레임 t = 바퀴 joint 토크 (전류 한계 5 A × 1.27). 선택: 바퀴 토크 노치 (wheel_notch_hz, 기본 꺼짐).
7. **기록**: 200 Hz 스텝 CSV (logs/steps), ControllerState 100 Hz.

## 3. wb_core.step (wbctrl.py DRIVE 경로 이식 + 로봇 추가분)

```mermaid
flowchart TD
  A[pitch·gyro·바퀴 속도] --> TH[θ = pitch + th_kin − th_bias<br/>bal_adapt 0.3: 멈춰 있을 때 θ 를 배움]
  A --> V[v = r·바퀴 절대 속도 평균, 10 Hz LPF]
  CMD[vx, wz 명령] --> LIM[속도 한계 v_lim<br/>vmax 3.5 km/h, 4 Hz 속도 브레이크 PI,<br/>turn_slow: vm − 0.094·|wz|]
  LIM --> REF[속도 기준 v_ref<br/>S-curve: jerk 3 m/s³, accel 0.8 m/s²<br/>바퀴 > 0.8·w_max 이면 v_ref 를 v 쪽으로 깎음]
  REF --> XE[x_err = ∫(v − v_ref)dt, ±0.3 m<br/>x_hold_moving false: |vx| > 0.02 면 0]
  TH --> LQR
  V --> LQR
  XE --> LQR
  REF --> LQR[LQR τ_w = −K(l)·(x_err, v − v_ref, θ, θ̇)<br/>K(0.20) = −3.0, −3.04, −16.1, −1.54]
  CMD --> YAW[yaw: τ_y = 0.15·(wz − gz) + ∫ 1.0·(wz − gz) (±0.5)]
  LQR --> MIX[바퀴 L/R = 0.5 τ_w ∓ τ_y<br/>마찰 보상 0, 20 Hz LPF, MIT 라 데드밴드 보상 꺼짐]
  YAW --> MIX
  A --> ROLL[롤 PI 0.5 / 3 / 0, leak 0.5<br/>→ 다리 높이 차 dlt (±0.10 m)]
  H[높이 명령 balance/height<br/>0.05 m/s 램프, 133–233 mm] --> VMC[다리 목표 h_tgt = h_mid ± dlt/2<br/>VMC kp 60, kd 1.0, ff 0.5 m g]
  ROLL --> VMC
  A --> LIFT[들림 감지: 고관절 토크 < 0.8 N·m 0.3 s → 바퀴 감쇠만]
```

| 항목 | 값 | 비고 |
|---|---|---|
| 루프 | 200 Hz SCHED_FIFO, kDt 0.005 | dt 중앙 5.00 ms, p99 5.02 |
| LQR 표 | balance_gains_robot.yaml: x −3.0 (사용자), v·θ = lowgain→기본 2/3, θ̇ lowgain | K(0.12) = −3, −3.14, −14.8, −1.32; K(0.40) = −3, −2.99, −18.3, −2.24 |
| trim | −1.2° (IMU 재고정 뒤) | bal_adapt 0.3 (θ 방식) |
| yaw | kd 0.15, ki 1.0 | 0.5 는 13 Hz 좌우 떨림 |
| 롤 | 0.5 / 3 / 0 | 1.0 / 8 / 0 = mit 모드에서 2.8 Hz 흔들림 (mit_pos 로 재시험 예정) |
| 바퀴 | 마찰 보상 0, LPF 20 Hz | 0.11 은 12–15 Hz 떨림 |
| 속도 | vmax 0.97 m/s, accel 0.8, jerk 3, 패드 0.7 | 0.97 패드에서 +1.28 m/s 넘침 → 조정 |
| 지연 (실측) | IMU 1.1 ms, 고관절 1.0 ms, 바퀴 응답 4.1 ms, 바퀴 명령→응답 토크 10 ms | |

## 4. 알려진 문제 (2026-10-09)
- 바퀴 백래시 (18′) → 14–18 Hz 피치 떨림 (gy 10–20 Hz 4–6 deg/s). θ̇ 이득을 못 올림.
- legacy MIT 바퀴 응답이 막힌 상태 + 토크 반전에서 굳음 (1 분 뒤 회복) → 감지 fault 만.
- LiDAR 감속: 자기 오른쪽 다리·바퀴를 0.08–0.33 m 로 봄 → 마스크 전까지 꺼둠.
- Wi-Fi heartbeat 끊김 32 회/h (0.55–1.4 s) → 1.0 s 로 늘림, 데스크톱 동글 절전 확인 요청.
