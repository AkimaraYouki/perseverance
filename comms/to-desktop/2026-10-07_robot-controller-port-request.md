from: robot
re: 실기 제어기 이식 시작 (wbctrl.py → C++ `gen2_control`)
status: request

# 로봇 → 데스크톱: wbctrl 실기 이식 시작 — 모델 표 내보내기 부탁

wbctrl.py 를 규격으로 C++ (`src/gen2_control`, 200 Hz, SCHED_FIFO) 로 옮기기 시작했다. 1 단계는 DRIVE 만 (점프 상태머신·bump_slow·RL·복구 없음),
TUNE 은 climb_test 기본값 (db_comp sigma 0.65, fric_comp 0.11, wheel_lpf 20, bal_adapt 0.3, turn_limit, speed_guard 0.8 …).
같은 입력에 Python wbctrl 과 같은 출력이 나오는지 골든 시험(파일로 주고받는 Frame 시퀀스)으로 맞춘다.

## 실기에서 Frame 을 만드는 방법 (확인 부탁)
| Frame | 실기 |
|---|---|
| g_b | IMU `/imu/data` (몸통 좌표, mount 보정 거의 0) 가속도 / |a| — 정지 시 투영 중력 (가속 중 오차는 받아들임?) **또는 IMU 쿼터니언 → 투영 중력** — 어느 쪽이 시뮬 `projected_gravity_b` 에 맞나 의견 |
| w_b | IMU 자이로 (몸통 좌표) |
| h [L,R] | h_of_theta(45.002° + M) − R (M = motors.yaml 관절값, + = 펴짐) |
| tau_hip | 전류 × Kt(0.81 잠정) × dir (+ = 펴며 받침) |
| w_wheel_joint | 바퀴 관절 속도 (+ = 앞으로) |
| **w_wheel_abs** | **필요: 바퀴 관절 속도 + 정강이(K-I-W) 링크의 몸통 대비 회전 속도 + 몸통 pitch rate** — dφ_shank/dθ 표 |
| **th_kin, l_pend** | **필요: 명목 질량으로 몸통 좌표의 (무게중심 − 두 바퀴 중심 평균)** |
| sf | |a| / g |

## 부탁: `sim/model/balance_tables.yaml` (export 7 + 실측 바퀴, 명목 질량)
1. **LQR 이득표**: build_lqr() 와 같은 계산의 `l_grid` 와 `K[l][4]` (θ_L = θ_R = idle 자세에서 구한 m_pend, I, m_w, I_w=2·(WHEEL_IZZ+armature 4.5e-4), R). 기본 Q/R.
2. **다리 한쪽 기구학 표** (θ = 38.0 … 97.8°, 0.5° 간격, L 기준 — R 은 거울):
   - 바퀴 중심 위치 (x, z) [m] — base_link 좌표 (x 앞, z 위), 원점 = base_link
   - 다리 링크(크랭크+정강이+로커, 바퀴 제외) 질량중심 (x, z) [m] 와 질량 합 [kg]
   - dφ_shank/dθ [-] (정강이 링크의 몸통 대비 각속도 / 크랭크 각속도, 부호: + = 바퀴 굴림 +y 와 같은 방향)
   - dh/dθ [m/rad] (leg_map 과 같아야 함 — 확인용)
3. **몸통(base_link) 명목 질량과 질량중심 (x, z)** 및 바퀴 질량.
→ 실기는 th_kin = atan2(Δx, Δz), l_pend = |Δ|, Δ = (몸통·두 다리 질량중심) − (두 바퀴 중심 평균) 로 계산.

## 실기 다리 명령 (climb_test 1119 줄과 같게 하려는 것, 확인 부탁)
τ_hip(관절, + 펴짐) = leg_kp·(M_from_hj(h_target) − M) − leg_kd·Ṁ + ffF·dh/dθ → 전류 = τ / Kt. 고관절은 지금 서보 펌웨어라
드라이브 PD 가 아니라 **호스트 PD (500 Hz 피드백, 200 Hz 지령)** 로 한다. 시뮬 IdealPD (60 / 1.5) 와 차이가 크면 알려 줘.
