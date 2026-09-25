# 하드웨어 ③ 액추에이터

> 설정 단일 출처: `src/gen2_hardware/config/motors.yaml` (근거·검증 플래그 포함)
> 매뉴얼: `docs/vendor/` — AK Series Module Driver User Manual V1.0.15.X (AK45-10), AK Series Product Manual V3.2.0 (AK60-6 V3.0)

## 0. 구성 — 모터 4 개

| 위치 | 모델 | 수 | 구동 | 이름 (URDF, L = +y) |
|---|---|---|---|---|
| 고관절 (4절링크 크랭크) | CubeMars **AK60-6 V3.0** KV80 | 2 | 다리 길이 | `leg_l`, `leg_r` (CAD `L/R_joint_M`) |
| 바퀴 | CubeMars **AK45-10** KV75 | 2 | 균형·주행 토크 | `wheel_l`, `wheel_r` (CAD `L/R_joint_W`) |

## 1. 사양 [데이터시트 / 매뉴얼]

| 항목 | AK60-6 V3.0 (고관절) | AK45-10 (바퀴) |
|---|---|---|
| 감속비 | 6 : 1 | 10 : 1 |
| 정격 / 피크 토크 | 3 Nm / 9 Nm | 2.5 Nm @ 2.1 A / 7 Nm @ 5 A |
| 무부하 속도 | URDF 한계 24.4 rad/s | 180 rpm = 18.85 rad/s (정격 150 rpm) |
| 토크 상수 (출력축) | 0.5994 N·m/A (V3.2.0 MIT 표) ⚠ | 1.27 N·m/A (= 0.127 로터측 × 10) |
| 극쌍 | 14 | 14 |
| 전압 | (매뉴얼 확인) | 24 V |
| 제어 모드 | 서보 + **MIT** (위치·속도·kp·kd·토크 한 프레임) | 서보 (전류/속도/위치) |
| 피드백 업로드 | 1 – 2000 Hz | 1 – 500 Hz (현재 50 Hz) |

⚠ AK60-6 토크 상수가 자료끼리 다르다. `asd.py` 설계 헤더는 데이터시트 값 **모터축 0.135 N·m/A × 6 = 출력 0.810 N·m/A** (정격 전류 3.8 A, 피크 10.3 A @24 V)를 쓴다. 로봇 쪽은 V3.2.0 매뉴얼 MIT 표의 0.5994 N·m/A 를 쓴다. **토크암으로 재서 확정**해야 한다(`verified.kt`). AK45-10 의 1.27 도 정격 2.5/2.1 = 1.19, 피크 7/5 = 1.4 와 대략 맞을 뿐 아직 미검증이다.

## 2. 고관절 — 토크 여유

평지에서 서 있을 때(속도 0, 냉각 불리) 고관절이 내는 토크는 가상일로 구한다.

```math
\tau_{\text{hold}}=\frac{Mg}{2}\cdot\frac{\partial h}{\partial\theta}\approx\frac{4.15\times9.81}{2}\times0.113\sim0.119=2.30\sim2.42\ \text{Nm}\quad[\text{계산, gains.yaml}]
```

→ **정격의 77 – 81 %**. 총중량 5.14 kg 에서 100 % 에 닿는다([CAD §3](01_cad.md#3-질량-export-5-urdf-cad)).
- 피크 9 Nm 는 **순간** 값이다. 다른 팀(QBMET 외골격 보고서)이 9 Nm 를 정격으로 읽고 30 분 연속으로 돌려 과열 셧다운된 사례가 있다.
- 동적 여유: 요철 흡수(0.85 m/s 로 3 cm 턱을 10 cm 안에 넘음)에 필요한 속도는 아래 정도로 한계(24.4 rad/s)보다 한참 작다.

```math
\dot h\approx0.25\ \text{m/s}\ \Rightarrow\ \dot\theta=\dot h/J\approx2.2\ \text{rad/s}
```

## 3. 바퀴 — 속도·토크 여유

```math
v_{\max}=\omega_0R=18.85\times0.06=1.13\ \text{m/s},\qquad
v_{3\,\text{km/h}}=0.833\ \text{m/s}\ \Rightarrow\ \omega=13.9\ \text{rad/s}\ (74\,\%)
```

DC 모터 토크-속도 직선에서 그 속도의 가용 토크는 다음과 같다.

```math
\tau_{\text{avail}}=\tau_s\Big(1-\frac{\omega}{\omega_0}\Big)=7\times0.26=1.8\ \text{Nm}
```

바퀴가 미끄러지지 않는 한계는 이보다 작다.

```math
\tau_{\text{fric}}=\mu\frac{Mg}{2}R=0.7\times\frac{4.0\times9.81}{2}\times0.06\approx0.82\ \text{Nm}\ \ (0.65\ \text{A})
```

→ AK45-10 은 **토크가 남는다**. 병목은 마찰이다. 그래서 RL 은 바퀴 토크를 1.5 Nm 로 제한하고, 실기 첫 기동은 0.5 Nm 로 한 번 더 자른다.
시뮬에서 균형 유지에 쓴 바퀴 토크 RMS 는 0.01 – 0.10 Nm 다 [측정, CAD m4650].

## 4. 시뮬 모델과 실기 대응

| | 시뮬 (Isaac Lab) | 실기 계획 |
|---|---|---|
| 고관절 | 명시적 PD @ 800 Hz, $k_p$ 60 Nm/rad, $k_d$ 1.5 Nm·s/rad, 9 Nm 제한, 회전자 관성은 링크에 포함 | **MIT 모드로 드라이브 안에서 PD** — 호스트 200 Hz PD 는 시뮬 800 Hz 와 달라진다. 목표 $M^*$ 는 leg_map 으로 계산 |
| 바퀴 | DC 모터: $\tau\in[-\tau_s(1+\omega/\omega_0),\ \tau_s(1-\omega/\omega_0)]\cap[\pm7]$, 점성 감쇠 0 | 전류 모드 $I=\tau/K_t$, **LPF 20 Hz** 는 호스트에서 |
| 좌우 부호 | CAD 바퀴축 L = −y → 정책 규약(+y)으로 왼쪽 부호 반전 | `motors.yaml` `direction` (검증 단계 5) |

잘못 넣었다가 고친 것: 바퀴 관절에 점성 감쇠 0.1 이 들어가 있었다. 이게 가짜 브레이크 역할을 해서 전진이 0.39 m/s 에서 막혔다. DC 모터 모델로 바꾸자 0.77 m/s 가 나왔다.

## 5. 버스·안전

- CAN 1 Mbit/s 한 버스에 4 개. 피드백 500 Hz 면 부하 56 % 로 상한이다([통신 §1.2](../software/02_communication.md#12-버스-부하--왜-500-hz-가-상한인가)).
- 현재 버스에 있는 건 AK45-10 id 69 하나. 나머지 3 개는 배송 뒤 ID 를 확인해 `motors.yaml` 에 넣는다.
- 드라이브 쪽 전류 한계를 35 A → 5 A 로 낮춘다(CubeMars 툴).
- 기동 한계 2.0 A, 10 rad/s, 80 °C. 지령이 1 s 끊기면 드라이브가 스스로 멈춘다.
- **하드웨어 비상정지 스위치가 아직 없다 — 균형 시험 전에 필수.**

## 6. 영점

AK60-6 영점은 CAD 영점 자세($M=0$, $\theta$ 45.0°, 고관절 높이 196.5 mm)에 맞춘다. 좌우 $M$ 은 부호가 반대다. 이 자세가 IDLE 이자 자동 높이 모드의 공칭 높이다.
