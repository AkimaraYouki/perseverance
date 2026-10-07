# 시뮬레이션·제어기 담당 전달 — 20 ms 해석 재검토와 실기 모델 일치

- from: 사용자 요청에 따른 코드 검토
- to: desktop / Isaac Lab 시뮬레이션·제어기 담당
- status: 분석 결과 + 모델·비교 실험 제안 (시뮬 재실행·코드 수정 미실행)
- 분석 기준: `AkimaraYouki/perseverance`, commit `aa4b0b839465dd7936a67b3970503ec30ecb05b2`
- 권장 배치 경로: `comms/to-desktop/2026-10-07_latency-diagnosis.md`
- 짝 문서: `2026-10-07_to-robot_latency-diagnosis.md`

> 앞선 분석을 시뮬 담당이 독립적으로 읽고 검증할 수 있도록 정리했다. 현재 작업 트리와 기준 커밋의 차이를 확인한 뒤 적용한다. 아래 실험은 제안이며 실행 결과가 아니다. GitHub에 자동 게시하지 않았다.

## 1. 결론과 기존 해석의 수정

**delay_ms=15–20에서 실기와 비슷한 진동이 생겼다는 사실만으로, 실기에 같은 크기의 순수 시간 지연이 있다고 확정할 수 없다.** 현재는 “추가 지연으로 실기 진동을 재현할 수 있다”는 결과다. 미모델링된 고관절 제어, 감속기 전달, 접지 동특성 등의 위상·이득 오차가 인위적인 delay로 흡수될 수 있다.

낮은 이득에서 실기 진동이 감소했다는 결과도 지연에만 고유하지 않다. 기존 교신의 “조용해지면 지연 원인 확정”은 “지연 및 미모델링 동특성에 대한 안정 여유 문제를 지지”로 수정하는 것이 정확하다.

고정적인 미지의 10 ms 소프트웨어 대기는 소스에서 확인되지 않았다. 핵심 다음 단계는 실기와 시뮬의 제어 구조·측정 구간을 일치시키는 것이다.

## 2. 관련 구조

| 경로 | 역할 |
| --- | --- |
| `sim/isaaclab/scripts/wbctrl.py` | Python 제어기, 속도 추정·LQR·필터·보상·명령 delay 큐 |
| `sim/isaaclab/scripts/climb_test.py` | 실행 설정, stiffness/damping 설정, 행동 적용 |
| `sim/model/balance_tables*.yaml` | LQR 및 모델 테이블 |
| `src/gen2_control/src/wb_core.cpp` | 실기 C++ 코어, DRIVE 경로 |
| `src/gen2_control/src/balance_node.cpp` | 실기 센서 조립, 고관절 제어 및 CAN 명령 |
| `src/gen2_control/include/gen2_control/wb_core.hpp` | 실기 파라미터와 고정 kDt=0.005 |
| `src/gen2_hardware/config/motors.yaml` | 방향·감속비·Kt·데드밴드 관련 실기 설정 |
| `comms/`, `logs/` | 비교 기록·측정 결과 |

실기 정상 경로는 IMU 500 Hz→SHM→200 Hz 제어, CAN 500 Hz RX→최신 상태→제어→직접 CAN TX다. Wi-Fi/DDS를 균형 루프 전체 지연으로 넣는 것은 현재 SHM 경로와 맞지 않는다. DDS fallback과 정상 SHM은 별도 조건으로 취급한다.

## 3. 기존 실험 결과와 증거의 범위

다음은 레포 교신의 보고값이며 이번 검토에서 재실행한 결과가 아니다.

| 조건 | theta_dot 표준편차 | 바퀴 토크 표준편차 | 주성분 |
| --- | --- | --- | --- |
| 시뮬 delay 5 ms | 0.07 rad/s | 0.22 N·m | — |
| 시뮬 delay 10 ms | 0.10 | 0.24 | — |
| 시뮬 delay 15 ms | 1.23 | 1.16 | 10 Hz |
| 시뮬 delay 20 ms | 1.66 | 1.43 | 8 Hz |
| 통신 9 ms + 바퀴 엔코더만 15 ms | 0.26 | 0.39 | 10 Hz, 작은 진동 |
| 실기 기본 이득 | 약 1.74–1.75 | 합 토크 약 3.97–4.35 N·m라는 실기 보고 | 8.4–9.6 Hz |
| 실기 낮은 이득 | 0.15 | 합 토크 0.51 N·m | 약 1 Hz |

토크 통계는 원래 교신에서 실기 약 2 N·m와 합 토크 약 4 N·m가 함께 쓰인다. **한 바퀴/합 토크, 평균 제거 여부, RMS/표준편차, 분석 창을 통일한 후 비교**해야 한다. 주파수 하나와 진폭 하나만으로 모델을 식별하지 않는다.

전류 계단→보고 전류는 중앙 L 3.1 ms, R 3.5 ms이며 500 Hz 피드백 샘플링이 포함된다. IMU 내부 약 3 ms는 추정이다. SHM IMU age 중앙 0.48 ms는 수신 이후의 나이이지 내부 지연을 포함한 값이 아니다. 따라서 20−9=11 ms를 그대로 물리적 누락 구간으로 확정하지 않는다.

## 4. delay 모델의 의미부터 명확히 할 것

`wbctrl.py`는 필터·데드밴드 보상 이후 행동을 큐에 넣는다.

```python
self.q.append(act.copy())
dly = int(round(P.delay_ms / 5.0)) + jitter_step
act = 과거의_act
```

이 act는 바퀴 토크뿐 아니라 다리 목표도 포함한다. `leg_kp`, `leg_kd`, `ff_force`는 반환 경로가 별도이므로 모든 신호가 동일하게 늦춰지는 모델도 아니다. 9 ms 설정은 round에 의해 기본 2스텝, 즉 10 ms가 된다. 입력 숫자와 실제 적용 스텝 수를 함께 보고할 것.

권장 분리:

- IMU 자세·자이로의 샘플 나이 및 내부 필터.
- CAN 피드백 샘플 나이·주기·수신 지터.
- 호스트 계산 및 명령 송신 지연.
- 바퀴 명령→전류 동특성.
- 감속기 입력→출력 토크 전달의 비선형 동특성.
- 고관절 센서·호스트 P·드라이브 D의 각 경로.

추가 delay 큐, 샘플링, ZOH의 효과를 지연 예산에서 중복 계산하지 않는다. 구현상 한 스텝 적용 순서까지 표시한다.

## 5. 가장 중요한 구조 불일치: 고관절 PD

실기 MIT 분기는 다음 구조다.

```text
host @200 Hz:
    tau_ff = kp*(M_target - M_feedback) + gravity_feedforward
drive:
    tau = tau_ff + kd*(0 - drive_velocity)
```

실기 코드에서 `mc.t`와 `mc.kd`를 설정하며 이 경로에서 내부 `mc.kp`는 설정하지 않는다. 반면 `climb_test.py`는 시뮬 액추에이터의 stiffness/damping을 설정한다. 이것은 지연된 host feedback으로 계산한 P와 동등하지 않다.

먼저 시뮬에 실기와 동일한 200 Hz 호스트 P + 드라이브 D를 구현한 조건을 추가한다. 고관절 feedback 지연과 actuator 반응은 측정 가능한 별도 파라미터로 둔다. 변경 전후 같은 LQR·필터·접촉 조건으로 비교한다.

고관절 영향은 높이 흔들림에만 머물지 않는다.

```text
theta_pendulum = pitch_body + theta_kin(M_L, M_R)
wheel_abs_rate = wheel_joint_rate + body_pitch_rate + shank_rate_from_hip
```

따라서 다리 동특성이 피치 LQR 입력으로 다시 결합한다. 기존 기록의 ±수 mm 다리 진동과 바퀴 진동을 함께 봐야 한다.

실기 P를 내부로 옮기는 비교는 로봇 담당의 위치 좌표·방향·wrap·범위·Kt 검증 이후 진행한다. host P를 남긴 채 internal P를 더하는 중복 제어를 피한다.

## 6. 전류가 아니라 실제 출력 전달을 모델링할 것

모터의 전류-토크 관계 `tau_motor≈Kt*I`와 감속기 출력축 토크 응답은 구분해야 한다. 반전에서 유격을 통과하는 동안 전류가 이미 생겨도 출력 힘 전달이 달라질 수 있다. 따라서 엔코더 신호만 15 ms 늦추는 모델로 실제 백래시의 효과를 배제할 수 없다.

가능한 단계적 모델:

1. 측정 전류 응답에 맞춘 actuator lag. 임계값 3 ms를 바로 동일한 1차 시정수로 설정하지 말고 응답 파형으로 추정.
2. 회전자 관성과 출력측 관성을 구분한 전달 모델.
3. 유격 구간과 접촉 상태, 필요 시 탄성·감쇠 및 정지/운동 마찰.
4. 실기 한 방향/반전·명령 진폭별 데이터로 검증.

사양의 18 arcmin은 출력측 약 0.3°이고, 10:1 환산으로 회전자측 약 3°다. 그러나 이것만으로 고정 시간 지연을 정할 수는 없다. 통과 시간은 상대속도·가속도·초기 접촉 상태에 따라 달라진다. 현재 데이터만으로 고충실도 모델 파라미터가 확정된 것은 아니다.

톡 치기에서 자이로가 바퀴보다 먼저 반응한 결과는 기계적 전달 조사 근거다. 서로 다른 물리량의 문턱 통과 차이를 IMU 절대 지연이나 엔코더 순수 지연으로 해석하지 않는다. 로봇 담당에게 출력축/토크 측정을 요청한다.

## 7. 필터 위상: 계산 결과와 중복 방지

실기와 Python 제어기에 모두 있는 식:

```text
y[k] = (1-alpha)*y[k-1] + alpha*u[k]
alpha = 1 - exp(-2*pi*fc*Ts), Ts=0.005 s
H(z) = alpha / (1-(1-alpha)*z^-1)
phase-equivalent delay = -arg(H(exp(j*2*pi*f*Ts)))/(2*pi*f)
```

이산 구현 자체의 위상 등가 지연 계산값이다. group delay나 주파수와 무관한 dead time과는 다르며, ZOH를 추가 포함한 값도 아니다.

| 필터 cutoff | 8 Hz에서 | 10 Hz에서 |
| --- | --- | --- |
| 10 Hz, 속도 추정 | 11.05 ms | 10.13 ms |
| 20 Hz, 토크 명령 | 5.33 ms | 5.14 ms |
| 40 Hz, 토크 후보 | 1.94 ms | 1.91 ms |
| 60 Hz, 토크 후보 | 0.88 ms | 0.87 ms |

속도 필터는 v 피드백 가지에만 있으며 자이로 theta_dot 전체를 늦추지 않는다. 토크 필터는 합성 명령 뒤에 있다. 분기별 위상을 전체 순수 지연 하나로 단순 합산하지 않는다.

`wbctrl.py`에도 이 필터들이 있고 `climb_test.py`는 환경 행동 항의 추가 바퀴 필터를 끄는 코드가 있다. 실제 실행 설정을 확인해 이중 필터 여부를 검증하되, 이미 포함된 필터를 “빠진 10 ms”로 더하지 않는다.

마찰 보상→토크 LPF→시그마-델타→명령 delay 순서와 실기의 대응을 확인한다. 시그마-델타 펄스는 비선형 현상을 자극할 수 있으므로 필터 전후와 보상 후 파형을 별도 기록한다.

## 8. 비교 실험 매트릭스

각 비교는 같은 초기조건·난수 seed·부하·gain·분석 창으로 수행하고 한 번에 한 요인만 바꾼다.

| 실험 | 변경 요소 | 판별 목적 |
| --- | --- | --- |
| S0 | 기존 설정과 실제 delay 스텝 수 보존 | 기존 결과 재현, 설정 기준점 |
| S1 | 시뮬 고관절을 200 Hz host P + local D로 | PD 구조 불일치 효과 |
| S2 | 바퀴 명령→전류 동특성 추가 | 전기적/드라이브 응답 영향 |
| S3 | S2에 출력 전달 유격·탄성 후보 추가 | 엔코더 지연 대체 모델과 실제 전달 모델 비교 |
| S4 | 토크 LPF 20/40/60 Hz | 안정 여유와 노이즈·펄스 영향 |
| S5 | 기본/낮은 LQR gain | 구조 일치 모델에서의 강건성 |
| S6 | 실측 지터·샘플 age 분포 적용 | 평균 지연과 간헐적 지터 효과 구분 |

모든 모델을 먼저 복잡하게 쌓지 말고, 각 추가 요소가 실기와의 차이를 얼마나 줄이는지 보고한다. 아직 측정되지 않은 파라미터는 추정값 범위로 표시한다.

비교 출력:

- pitch, theta, theta_dot, v_raw/v_filtered, hip 높이·속도·theta_kin.
- LQR 원시 토크, LPF 출력, sigma 출력, 적용 전류 및 실제/모델 출력 토크.
- 동일 정의의 좌우별·합 토크 표준편차와 RMS, 주파수 및 신호 간 위상.
- 명령 진폭·반전 시점에 따른 응답 변화.
- 낮은 이득에서 정지 안정성과 주행·외란 복원 성능을 별도로 평가.

## 9. 실기 계측과 반드시 맞출 사항

로봇 담당에게 별도 문서로 요청한 변경:

1. 200 Hz 스텝 번호·시각을 입력/출력과 함께 일관된 스냅샷으로 저장.
2. 100 Hz ROS 발행 시각을 제어 시각과 분리하고 공유 상태 데이터 경쟁 제거.
3. CAN 사용자 recv 시각과 커널 수신 시각을 같은 시간 기준에서 비교.
4. IMU SHM을 ROS 발행 전에 갱신하고 SHM 성공 시 DDS mutex/복사를 건너뛰기.
5. 필터 전후·보상 후 토크와 출력축 응답 측정.

현재 CSV는 100 Hz 상태 발행을 바탕으로 하므로 5–10 ms 지연을 정밀 분리하는 자료로 쓰기 어렵다. 실행 로그의 일관성이 먼저다. `poll(...,20)`은 최대 무수신 대기이며 프레임마다 고정 20 ms가 아니다.

실기 코어는 `kDt=0.005` 고정이다. `rate_hz`만 500 Hz로 올려 비교하면 필터·적분·미분·시간 판정이 어긋난다. dt 일관성을 먼저 해결하고 변경된 코어와 Python 기준의 의미 있는 비교 검증을 수행한다. 제어 계산 최적화나 동적 할당 제거는 측정된 병목·지터에 근거해 진행한다.

## 10. 요청하는 회신

- 기존 결과를 만드는 정확한 실행 명령, commit, 설정·테이블과 실제 적용 지연 스텝 수.
- S0/S1을 우선 비교한 결과와 실기 고관절 구조를 구현한 위치.
- 토크 통계 정의를 통일한 기본/낮은 gain 비교.
- 아직 측정되지 않은 actuator·backlash 파라미터 목록과 로봇 담당에게 필요한 데이터.
- 원인 가설별 지지/반증/미판정 구분.

낮은 이득에서 조용해진 것만으로 기본 이득 복귀나 전체 주행 안정성을 판단하지 않는다. 원인을 분리한 뒤 필터·액추에이터 상태를 포함한 이산 모델로 LQR을 재설계하는 방향을 검토할 수 있다. 현재 단계에서 새로운 gain의 안정성을 보증할 근거는 없다.

## 11. 근거 링크

- [wbctrl.py](https://github.com/AkimaraYouki/perseverance/blob/aa4b0b839465dd7936a67b3970503ec30ecb05b2/sim/isaaclab/scripts/wbctrl.py)
- [climb_test.py](https://github.com/AkimaraYouki/perseverance/blob/aa4b0b839465dd7936a67b3970503ec30ecb05b2/sim/isaaclab/scripts/climb_test.py)
- [실기 balance_node.cpp](https://github.com/AkimaraYouki/perseverance/blob/aa4b0b839465dd7936a67b3970503ec30ecb05b2/src/gen2_control/src/balance_node.cpp)
- [실기 wb_core.cpp](https://github.com/AkimaraYouki/perseverance/blob/aa4b0b839465dd7936a67b3970503ec30ecb05b2/src/gen2_control/src/wb_core.cpp)
- [시뮬 진동 재현 기록](https://github.com/AkimaraYouki/perseverance/blob/aa4b0b839465dd7936a67b3970503ec30ecb05b2/comms/to-robot/2026-10-07_2032_desktop-oscillation-repro.md)
- [지연 측정](https://github.com/AkimaraYouki/perseverance/blob/aa4b0b839465dd7936a67b3970503ec30ecb05b2/logs/2026-10-07_latency.txt)
- [낮은 이득 실기 기록](https://github.com/AkimaraYouki/perseverance/blob/aa4b0b839465dd7936a67b3970503ec30ecb05b2/comms/to-desktop/2026-10-07_2245_robot-lowgain-quiet.md)
