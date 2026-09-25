# 소프트웨어 ① 학습 — 강화학습 균형·주행 제어기

> 대상 코드: `sim/isaaclab/source/wheeled_biped_isaaclab/tasks/balance/`
> (`env_cfg.py` 환경, `commands.py` 명령, `actions.py`·`cad.py` 행동, `rewards.py` 보상, `terrain.py` 지형)
> 도구: Isaac Lab 2.3.2 / Isaac Sim 5.1, rsl_rl PPO, RTX 5080 한 대.
> 수치 옆 **[측정]** 은 시뮬에서 잰 값, **[설정]** 은 코드에 넣은 값, **[추정]** 은 계산·가정이다.

---

## 0. 한눈에

```math
\pi_\phi:\ \underbrace{o_t\in\mathbb{R}^{20}}_{\text{IMU + 관절 + 명령}}\ \longmapsto\ \underbrace{a_t\in[-1,1]^4}_{\text{다리 2 + 바퀴 2}},\qquad 200\ \text{Hz}
```

| 과제 ID | 모델 | 관측 | 지형 | 용도 |
|---|---|---|---|---|
| `Isaac-WheeledBiped-Balance-v0` | 단순화 (다리 = 직선 관절) | 19 | 평지 | 초기 학습 |
| `Isaac-WheeledBiped-CAD-v0` | **CAD 4절링크 폐루프** | 19 | 평지 | 6방향·고속·코너 |
| `Isaac-WheeledBiped-CAD-Rough-v0` | CAD 4절링크 폐루프 | **20** (+높이 모드) | **거친 지형 커리큘럼** | 짐벌(몸통 고정), 자동/수동 높이 |

세 과제의 정책 입출력은 같은 규약이다. 그래서 앞 과제의 정책을 다음 과제로 **이어서 학습**한다(§11.3).

---

## 1. 문제 정식화

부분관측 MDP $(\mathcal{S},\mathcal{O},\mathcal{A},P,r,\gamma)$. 선속도와 지형은 **관측하지 않는다**. 실기에서 직접 잴 수 없어서, 학습 때 쓰면 sim2real 에서 깨지기 때문이다.

```math
J(\phi)=\mathbb{E}_{\pi_\phi}\Big[\sum_{t=0}^{T-1}\gamma^t r_t\Big],\qquad
\Delta t_{\text{phys}}=\tfrac{1}{800}\,\text{s},\quad \Delta t=4\,\Delta t_{\text{phys}}=\tfrac{1}{200}\,\text{s},\quad T=\tfrac{20\,\text{s}}{\Delta t}=4000
```

물리 800 Hz, 정책 200 Hz(decimation 4). 실기 제어 루프(200 Hz 지령, 모터 피드백 500 Hz)와 같은 자리다.

### 좌표·부호 (REP-103)
- 몸체 좌표: $x$ 전진, $y$ 왼쪽, $z$ 위. 왼쪽 = L = $+y$.
- 투영 중력 $g_b = R_{wb}^\top[0,0,-1]^\top$. 앞으로 기울면 $g_{b,x}>0$, 왼쪽으로 기울면 $g_{b,y}=+\sin\phi$.
- 바퀴 각속도·토크는 $+y$ 축 기준이다. CAD 바퀴축은 왼쪽이 $-y$ 라서 왼쪽만 부호를 뒤집는다(`cad.WHEEL_SIGN = (-1, +1)`).

---

## 2. 다리 기구학 — 4절링크 ↔ "다리 길이"

정책은 다리 하나를 **길이 $h$** 하나로 본다. 실기와 CAD 는 모터각 $\theta$ 로 움직인다. 그 사이를 `leg_map.py` 가 잇는다.

링크 길이 [CAD 실측]: $L_1=|MI|=102.72$, $L_2=|MP|=100.72$, $L_3=|IK|=80.00$, $L_4=|IW|=170.98$, $L_5=|PK|=80.00$ mm, 바퀴 $R=60$ mm.

```math
I(\theta)=L_1\begin{bmatrix}\cos\theta\\ \sin\theta\end{bmatrix},\qquad
K=\text{원-원 교점}\big(\,|K-P|=L_5,\ |K-I|=L_3\,\big)\ (\text{조립 분기 } -1),\qquad
W=I+L_4\,\frac{I-K}{|I-K|}
```

```math
h(\theta)=|W-M|\quad(\text{고관절 모터축}\to\text{바퀴 중심}),\qquad
h_{\text{hip}}(\theta)=h(\theta)+R\quad(\text{모터축의 지면 높이})
```

| $\theta$ [deg] | 38.0 (최소) | **45.0 (CAD 영점, IDLE)** | 52.5 | 70 | 90 | 97.8 (최대) |
|---|---|---|---|---|---|---|
| $h_{\text{hip}}$ [mm] | 182.5 | **196.5** | 212.0 | 247.8 | 287.1 | 302.5 |
| $h$ (정책 값) [mm] | 122.5 | **136.5** | 152.0 | 187.8 | 227.1 | 242.5 |
| $J=\partial h/\partial\theta$ [mm/rad] | 113.2 | 117.0 | 118.4 | 115.6 | 110.5 | 118.8 |

- 모터각과의 관계: $M_i=\sigma_i(\theta_i-\theta_0)$, $\theta_0=45.002^\circ$, $\sigma_L=+1,\ \sigma_R=-1$.
- 가상일: 다리 수직력 $F$ ↔ 고관절 토크 $\tau=J(\theta)\,F$. 정지 하중 $F=\tfrac{1}{2}Mg$ 이면 $\tau\approx 2.3\sim2.4$ Nm [추정, `sim/control/gains.yaml`]. AK60-6 정격 3 Nm 의 약 80 %.
- 검증 [측정]: Isaac 폐루프 잔차 0.00 mm. 다리 길이는 leg_map 과 최대 0.04 mm, 관절각은 설계 스크립트 `asd.py` 와 0.11–0.18° 차이.

---

## 3. 명령 벡터 $c$

```math
c=\big[\,v_x^*,\ \omega_z^*,\ h_{\text{ref}},\ m\,\big]^\top,\qquad
v_x^*\in[-0.85,0.85]\ \text{m/s},\ \ \omega_z^*\in[-2.5,2.5]\ \text{rad/s},\ \ h_{\text{ref}}\in[0.130,0.235]\ \text{m},\ \ m\in\{0,1\}
```

- 0.85 m/s = 3 km/h(사용자 목표)이고, 바퀴로는 $\omega=v/R=14.2$ rad/s(무부하 18.85 의 75 %)다.
- **높이 모드 $m$** (Rough 과제만, 평지 과제에는 없어서 명령이 3 개):
  - $m=0$ **수동 (평지 전용)**: 평균 높이 = 사용자가 준 $h_{\text{ref}}$. 학습에서도 로봇이 **평지 타일 위에 있을 때만** 수동이 추첨된다.
  - $m=1$ **자동 (모든 지형)**: 양쪽 다리 높이를 정책이 외란·지형에 맞춰 정한다. $h_{\text{ref}}$ 는 공칭값 $h_0=136.5$ mm(CAD 기본자세 $M=0$, IDLE)로 고정되고, 보상에서 느슨한 선호로만 쓴다(§6.2).

**재추첨** (3–6 s 마다, `commands.py`). 균등 추첨 뒤 아래 순서로 덮어쓴다. 오리 로봇 프로젝트에서 가져온 구조다.

```math
\begin{aligned}
&(v_x^*,\omega_z^*,h_{\text{tgt}})\sim\mathcal{U}(\text{box})\\
&\text{w.p. }0.35:\ \text{단일축만 남김 — } \{v_x\text{ 만},\ \omega_z\text{ 만},\ \text{높이만}\}\sim\text{Cat}(0.50,\,0.35,\,0.15)\\
&\text{w.p. }0.20:\ \text{고속 코너 — } |v_x^*|\sim\mathcal{U}(0.50,0.85),\ |\omega_z^*|\sim\mathcal{U}(0.6,2.0),\ \text{부호 무작위}\\
&\text{w.p. }0.10:\ v_x^*=\omega_z^*=0\\
&m=\begin{cases}\text{Bernoulli}(0.5) & \text{평지 타일 위}\\ 1 & \text{그 밖}\end{cases},\qquad m=1\Rightarrow h_{\text{tgt}}=h_0
\end{aligned}
```

단일축 명령을 섞는 이유: 축마다 독립 균등으로만 뽑으면 "한 방향만" 명령이 거의 안 나온다. 오리 로봇에서는 이 때문에 25 개 버전 모두 좌우 오차가 전진 오차의 2–4 배였다.

**높이 기준의 속도 제한**: 목표는 튀어도, 기준은 0.10 m/s 로만 따라간다. 정책이 보는 값도, 다리 행동의 기준도 $h_{\text{ref}}$ 다.

```math
h_{\text{ref},k+1}=h_{\text{ref},k}+\text{clip}\big(h_{\text{tgt}}-h_{\text{ref},k},\ -0.10\,\Delta t,\ +0.10\,\Delta t\big)
```

---

## 4. 관측 벡터 $o$ (20 차원) ⟵ 실기 센서

| 인덱스 | 기호 | 정의 | 단위 | 실기 출처 | 학습 잡음 $\mathcal{U}(\pm)$ |
|---|---|---|---|---|---|
| 0–2 | $\omega_b$ | 몸체 각속도 $(\omega_x,\omega_y,\omega_z)$ | rad/s | iAHRS 자이로 | 0.2 |
| 3–5 | $g_b$ | 투영 중력 $R_{wb}^\top[0,0,-1]^\top$ | – | iAHRS 쿼터니언 | 0.05 |
| 6 | $v_x^*$ | 전진 속도 명령 | m/s | 조이스틱 / `/cmd_vel` | – |
| 7 | $\omega_z^*$ | 회전 명령 | rad/s | 조이스틱 / `/cmd_vel` | – |
| 8 | $h_{\text{ref}}$ | 높이 기준 (속도 제한 후) | m | 조이스틱 트리거 | – |
| 9 | $m$ | 높이 모드 (0 수동, 1 자동) | – | 조이스틱 버튼 | – |
| 10–11 | $h_i-\bar h$ | 다리 길이 − 0.1825 (L, R) | m | AK60-6 엔코더 → leg_map | 0.002 |
| 12–13 | $\dot h_i$ | $J(\theta_i)\,\dot\theta_i$ (L, R) | m/s | AK60-6 속도 | 0.5 |
| 14–15 | $\omega_{w,i}$ | 바퀴 각속도, $+y$ 규약 (L, R) | rad/s | AK45-10 속도 | 0.5 |
| 16–19 | $a_{t-1}$ | 직전 행동 (다리 L, R, 바퀴 L, R) | – | 제어기 내부 | – |

- 평지 과제는 9 번($m$)이 없는 19 차원이다. 그 뒤 인덱스가 하나씩 당겨진다.
- **넣지 않은 것**: 몸체 선속도(측정 불가), 바퀴 회전각(계속 커져서 학습 때 못 본 값이 된다), 지형 높이(블라인드 정책).
- 정규화: 망 안에서 누적 통계로 한다. 내보낸 `policy.pt`/`.onnx` 에 정규화기가 포함된다.

```math
\tilde o=\frac{o-\mu}{\sigma+0.01}
```

- ⚠ 알려진 문제: 12–13 번 $\dot h$ 에 붙인 잡음 ±0.5 m/s 는 단순화 모델(직선관절 속도)에서 물려받은 값이다. 실제 $\dot h$(대개 0.1 m/s 이하)보다 커서, 정책이 이 두 칸을 사실상 못 쓴다. 다음 개정에서 약 0.05 로 줄일 후보다.

---

## 5. 행동 벡터 $a$ (4 차원) ⟶ 모터 지령

```math
a=[\,a_{h,L},\ a_{h,R},\ a_{w,L},\ a_{w,R}\,]^\top,\qquad \bar a=\text{clip}(a,-1,1)
```

자르기는 래퍼가 아니라 **행동 항 안에서** 한다. 래퍼에서 자르면 ±1 밖으로 밀려난 정책 평균을 벌할 수 없어 뱅뱅 제어가 된다(실제로 원시 출력이 최대 194 까지 나왔다). 밖으로 나간 양은 §6 에서 벌한다.

### 5.1 다리 (위치 지령 → 고관절 PD)

```math
h_i^*=\text{clip}\big(h_{\text{ref}}+s_h\,\bar a_{h,i},\ 0.1225,\ 0.2425\big),\qquad
M_i^*=\sigma_i\big(\theta(h_i^*)-\theta_0\big)
```

```math
\tau_{\text{hip},i}=\text{clip}\big(k_p(M_i^*-M_i)-k_d\dot M_i,\ \pm 9\ \text{Nm}\big),\qquad k_p=60\ \tfrac{\text{Nm}}{\text{rad}},\ k_d=1.5\ \tfrac{\text{Nm·s}}{\text{rad}}
```

- $s_h=0.12$ m (Rough): 어떤 $h_{\text{ref}}$ 에서든 최소·최대 **양끝까지** 닿는다(사용자 요청). 평지 과제는 $s_h=0.03$ 이다.
- PD 는 800 Hz 명시적(explicit)으로 돈다. PhysX 암시적 드라이브는 폐루프(Exclude-from-articulation) 관절의 구속력을 못 봐서 90 Nm 를 줘도 20° 뒤처졌다.

### 5.2 바퀴 (토크 지령 → 1차 LPF → DC 모터 한계)

```math
u_i=1.5\,\bar a_{w,i}\ \text{[Nm]},\qquad
\tau_{w,i}[k]=\tau_{w,i}[k-1]+\alpha\big(u_i[k]-\tau_{w,i}[k-1]\big),\qquad
\alpha=1-e^{-2\pi f_c\Delta t}=0.466\ \ (f_c=20\ \text{Hz})
```

```math
\tau_{w,i}\in\Big[\max\big(-7,\ -\tau_s(1+\tfrac{\omega_{w,i}}{\omega_0})\big),\ \min\big(7,\ \tau_s(1-\tfrac{\omega_{w,i}}{\omega_0})\big)\Big],\qquad \tau_s=7\ \text{Nm},\ \omega_0=18.85\ \text{rad/s}
```

- 1.5 Nm 스케일의 근거: 바퀴가 미끄러지지 않는 한계는 다음과 같다.

```math
\tau_{\text{fric}}=\mu\,\tfrac{Mg}{2}\,R=0.7\times\tfrac{4.0\times9.81}{2}\times0.06\approx0.82\ \text{Nm}
```

  2.0 Nm 로 두면 행동 범위의 60 % 가 쓸모없이 미끄러지는 구간이고, 탐색 잡음만 키웠다.
- **LPF 20 Hz 는 실기 제어기에도 같은 주기로 넣어야 한다.** 정책은 이 필터를 전제로 학습됐다.

---

## 6. 보상

Isaac Lab 은 모든 항에 $\Delta t$ 를 곱한다: $r_t=\sum_i w_i\,f_i(s_t,a_t)\,\Delta t$. 그래서 아래 $w$ 는 **초당** 가치다.

### 6.1 추종

```math
f_{v}=\exp\!\Big(-\frac{(v_x^*-v_x)^2+v_y^2}{0.25^2}\Big),\qquad
f_{\omega}=\exp\!\Big(-\frac{(\omega_z^*-\omega_z)^2}{0.5^2}\Big)
```

$v$ 는 몸체 **COM** 속도다. CAD 몸체 원점이 고관절 중점에서 8 cm 떨어져 있어서, 원점 속도를 쓰면 회전 중 속도가 왜곡된다.

### 6.2 높이 — 평지: 다리 길이 / 거친 지형: **짐벌**

평지 과제:

```math
f_h=\exp\!\Big(-\tfrac{1}{2}\sum_i\frac{(h_i-h_{\text{ref}})^2}{0.02^2}\Big)
```

거친 지형 과제("몸통을 짐벌처럼 제자리에"): 기준을 바퀴 밑이 아니라 **몸통 아래 넓은 지면의 평균** $\bar T$ 로 잡는다. $\bar T$ 는 고관절 중점 중심 60 × 40 cm, 35 점 레이캐스트이고 보상 계산에만 쓴다.

몸통 높이를 기하로 쓰면 다음과 같다($\bar z_w$ = 두 바퀴 중심 높이 평균, $\delta$ = 고관절-몸체 기준점 사이 상수).

```math
z_b\approx\bar z_w+\bar h+\delta
```

이상적인 짐벌은 $z_b-\bar T=R+h_{\text{ref}}+\delta$ 를 지킨다. 여기서 오차를 빼면 $\delta$ 가 소거된다.

```math
e=(z_b-\bar T)-(R+h_{\text{ref}}+\delta)=\underbrace{(\bar z_w-R-\bar T)}_{\text{바퀴가 밟은 요철}}+\underbrace{(\bar h-h_{\text{ref}})}_{\text{다리 보상량}},\qquad
f_{\text{gimbal}}=\exp\!\Big(-\frac{e^2}{\sigma_m^2}\Big),\ \ \sigma_0=0.02,\ \sigma_1=0.05\ \text{m}
```

- 바퀴가 +2 cm 턱에 오르면 다리를 2 cm 줄여야 $e=0$ 이다. 몸통은 그 자리에 있다. 파인 곳이면 다리를 편다.
- 평지에서는 첫 항이 0 이라 $f_h$ 와 같아진다.
- 패치 크기(60 cm)가 **공간 저역통과 폭**이다. 그보다 짧은 요철은 다리가 흡수하고, 긴 경사·파도는 몸이 따라간다.

**승차감 (짐벌 본체)**: 몸통 COM 수직가속도.

```math
f_{\text{ride}}=\exp\!\Big(-\frac{a_{z}^2}{1^2}\Big)\quad[\text{m/s}^2]
```

첫 판은 σ = 3 이었다. 기준선의 $a_z$ RMS 0.2–3 m/s² 가 전부 0.6 이상 보상을 받아서 기울기가 거의 없었다.

**행정 여유 (자동 모드만)**: 다리가 양끝 10 mm 안에 붙지 않게 한다. 하한에 붙으면 더 줄일 수 없어 요철을 흡수하지 못한다.

```math
f_{\text{stroke}}=m\sum_i\Big[\max\Big(0,\ 1-\frac{\min(h_i-h_{\min},\ h_{\max}-h_i)}{0.010}\Big)\Big]^2
```

**다리 부드러움 (실제 이동량 기준)**: 권한을 $s_h$ 0.03 → 0.12 로 넓히면, 같은 행동 변화 벌점이 실제 다리 이동으로는 $1/16$ 이 된다. 예전 물리량 기준으로 되돌린다.

```math
f_{\text{legrate}}=\Big(\tfrac{0.12}{0.03}\Big)^2\sum_{i\in\{L,R\}}\big(a_{h,i,t}-a_{h,i,t-1}\big)^2
```

속도가 아니라 가속도를 쓴다. 그래서 경사를 일정하게 오르거나 높이를 천천히 바꾸는 건 벌하지 않고, 튀는 것만 잡는다. 수직속도 L2 벌점은 경사 등반($v\cdot\tan\alpha$)까지 벌해서 뺐다.

### 6.3 자세 — 앞뒤는 벌, 좌우는 **코너 기울기 추종**

두 바퀴 역진자라 가감속 중 앞뒤 기울기는 피할 수 없다. 앞뒤는 약하게 벌한다.

```math
f_{\text{pitch}}=g_{b,x}^2
```

좌우는 원심력과 중력의 합력 방향에 몸을 맞춘다(바이크의 협조 선회).

```math
\tan\phi^*=\frac{a_c}{g}=\frac{v_x\,\omega_z}{g},\qquad
f_{\text{roll}}=\exp\!\Big(-\frac{\big(g_{b,y}-\sin\phi^*\big)^2}{0.03^2}\Big)
```

직진이나 제자리 회전이면 $v_x\omega_z=0$ 이라 목표는 수평이다.
[측정, CAD m4650]: 0.8 m/s × 2 rad/s 코너에서 좌 +6.9°(목표 +6.2°), 우 −9.8°(목표 −8.7°).

### 6.4 제자리 회전 (정지 + 회전 명령일 때만)

두 바퀴 평균 각속도 $\bar\omega_w=\tfrac12(\omega_{w,L}+\omega_{w,R})$ 는 차축 중점의 직진 성분이다. 0 이 아니면 한쪽 바퀴를 축으로 원을 그린다. smoothstep $S(x;a,b)=t^2(3-2t),\ t=\text{clip}\big(\tfrac{x-a}{b-a},0,1\big)$ 로 문을 만든다.

```math
G=\big(1-S(|v_x^*|;0.05,0.15)\big)\,S(|\omega_z^*|;0.1,0.3),\qquad
f_{\text{spin}}=G\,\exp\!\Big(-\frac{\bar\omega_w^2}{0.5^2}\Big)
```

처음에 정지 명령이면 항상 L2 벌점을 줬더니, 외란을 받아낼 때 필요한 양쪽 같은 방향 회전까지 벌했다. 정책이 넘어지는 쪽을 택해서 완주율이 0.005 로 붕괴했다. 그래서 문을 좁히고, 상한이 있는 보상(0–1)으로 바꿨다.

### 6.5 전체 표 (Rough 과제 가중치 / 평지 과제 차이)

| 항 | 식 | $w$ [/s] | 비고 |
|---|---|---|---|
| track_lin_vel | $f_v$ | +2.0 | |
| track_ang_vel | $f_\omega$ | +1.5 | |
| gimbal_height | $f_{\text{gimbal}}$ | +1.5 | 평지: `track_height` $f_h$ +1.0 |
| ride | $f_{\text{ride}}$ | +1.0 | Rough 만 |
| stroke_margin | $f_{\text{stroke}}$ | −1.0 | Rough, 자동 모드만 |
| leg_rate | $f_{\text{legrate}}$ | −0.1 | Rough 만 |
| roll_track | $f_{\text{roll}}$ | +1.5 | |
| spin_in_place | $f_{\text{spin}}$ | +1.0 | |
| alive | $1$ | +2.0 | 20 s 완주 = +40 |
| upright (pitch) | $g_{b,x}^2$ | −2.0 | |
| ang_vel_xy | $\omega_x^2+\omega_y^2$ | −0.1 | |
| action_rate | $\lVert a_t-a_{t-1}\rVert^2$ | −0.1 | |
| wheel_effort | $\sum_i\tau_{w,i}^2$ | −0.1 | 바퀴 진동 억제 |
| action_range | $\sum_j\max(\lvert a_j\rvert-1,0)^2$ | −0.5 | |
| leg_vel | $\sum_i\dot M_i^2$ | −0.002 | |
| joint_limits | 소프트 한계 밖 $M_i$ | −1.0 | |
| terminated | $\mathbb{1}[\text{넘어짐}]$ | −200 | 실효 −200·Δt = **−1.0** 한 번 |
| (평지만) lin_vel_z | $v_{z,b}^2$ | −1.0 | Rough 에서는 끔 |

"죽는 비용"은 −1.0 한 번에, 남은 시간 동안 못 받는 alive(2/s)와 추종 보상이 더해진 값이다. 초기 버전은 −50(실효 −0.25)이었다. 그때는 넘어지는 쪽이 흔들림 벌점 누적(−0.45)보다 쌌다.

---

## 7. 종료 조건

```math
\text{done}=\underbrace{t\ge 20\,\text{s}}_{\text{time-out (부트스트랩)}}\ \lor\ \underbrace{\arccos(-g_{b,z})>0.8\ \text{rad}}_{\text{기울기 }46^\circ}\ \lor\ \underbrace{z_{\text{base}}-\bar T<0.12\ \text{m}}_{\text{지면 기준 높이}}
```

---

## 8. 도메인 랜덤화·외란

| 항목 | 분포 | 근거 |
|---|---|---|
| 바퀴-바닥 마찰 | $\mu\sim\mathcal{U}(0.5,0.8)$, 지면 1.0 곱 결합 | 사용자 지정 (대리석·실내) |
| 몸체 질량 | $+\mathcal{U}(-0.3,+0.5)$ kg | 실측 총질량 ≈ CAD 4.03 + 0.25 kg [추정] |
| 몸체 COM | $x\pm2$, $y\in[-3,+1]$, $z\pm1$ cm | 편심 15 mm 실측 |
| 밀기 | 4–8 s 마다 $\Delta v_{xy}\sim\mathcal{U}(\pm0.4)$ m/s | |
| 초기 자세 | pitch ±0.15, roll ±0.08 rad, 각속도 ±0.3 rad/s | |
| 관측 잡음 | §4 표 | |

---

## 9. 거친 지형 커리큘럼 (`terrain.py`)

8 × 8 m 타일 10 행(난이도) × 16 열(종류). 행 $r$ 의 난이도는 $d\in[r/10,(r+1)/10)$ 이다. 학습용 seed 42, 측정용 seed 7(보지 못한 지형).

| 종류 | 비율 | 형상 (난이도 $d$) |
|---|---|---|
| flat | 0.20 | 평지 (평지 성능 유지, 수동 모드는 여기서만) |
| rough | 0.22 | 25 cm 격자 무작위 높이 $\pm(0.5+3.0d)$ cm, 스플라인 보간 |
| wave_long | 0.14 | $z=\tfrac{A}{2}\big(\cos\tfrac{2\pi y}{\lambda}+\sin\tfrac{2\pi x}{\lambda}\big)$, $\lambda=2$ m, $A=0.10d$ |
| wave_short | 0.14 | 같은 식, $\lambda=1$ m, $A=0.06d$ |
| slope_up / down | 0.10 / 0.10 | 피라미드 경사 $\tan\alpha=0.30d$ (최대 16.7°) |
| bumps | 0.18 | 턱·파인 곳 $1+3d$ cm, 폭 0.3–1.0 m, 10 cm 에 걸친 경사로 |

**턱을 경사로로 만든 이유**: 반지름 $R$ 바퀴가 수직 턱 $h$ 를 마찰만으로 넘으려면 다음 조건이 필요하다.

```math
\frac{F_x}{W}=\frac{\sqrt{2Rh-h^2}}{R-h}\le\mu
```

$\mu=0.65,\ R=60$ mm 이면 $h\approx1$ cm 가 한계다. 100 mm 계단은 이 방식으로는 오를 수 없어서 별도 과제로 다룬다.

**레벨 갱신** (IsaacLab 기본은 이동거리 기준인데, 정지·제자리 회전 명령이 섞여 있어서 생존 기준으로 바꿨다):

```math
\ell\leftarrow\ell+\mathbb{1}[\text{20 s 완주}]-\mathbb{1}[\text{넘어짐}],\qquad \ell_0\le2
```

---

## 10. PPO

```math
L^{\text{clip}}(\phi)=\mathbb{E}_t\Big[\min\big(\rho_t\hat A_t,\ \text{clip}(\rho_t,1-\epsilon,1+\epsilon)\hat A_t\big)\Big],\quad
\rho_t=\frac{\pi_\phi(a_t|o_t)}{\pi_{\phi_{\text{old}}}(a_t|o_t)},\quad \epsilon=0.2
```

```math
\delta_t=r_t+\gamma V(o_{t+1})-V(o_t),\qquad \hat A_t=\sum_{l\ge0}(\gamma\lambda)^l\delta_{t+l}
```

```math
L=-L^{\text{clip}}+c_v\,L^{V}_{\text{clip}}-c_e\,\mathcal{H}[\pi_\phi],\qquad c_v=1.0,\ c_e=0.006
```

### 10.1 시간 규모 환산 — 핵심 수정

IsaacLab 기본 하이퍼파라미터는 50 Hz 기준이다. 200 Hz 에 그대로 쓰면 할인 시야가 1/4 로 줄어든다. 실제로 세 번의 학습이 모두 iter 100 근처에서 정점을 찍고 무너졌다. 시간 상수 $T$ 를 유지하도록 환산했다.

```math
\gamma=1-\frac{\Delta t}{T_\gamma}=1-\frac{0.005}{2.0}=0.9975,\qquad
T_{\text{GAE}}=\frac{\Delta t}{1-\gamma\lambda}=\frac{0.005}{1-0.9975\times0.9875}=0.33\ \text{s},\qquad
N_{\text{roll}}=96\ (0.48\ \text{s})
```

| 항목 | 값 |
|---|---|
| 병렬 env | 4096 → 배치 $4096\times96=393{,}216$ 샘플/iter |
| 미니배치 / 에폭 | 4 / 5 |
| 학습률 | $10^{-3}$, KL 적응형 (목표 0.01) |
| 망 | actor·critic MLP [128, 128, 128], ELU, 가우시안 정책(상태 무관 std) |
| 엔트로피 $c_e$ | 평지 0.006 (0.005: std 붕괴 0.23 → 탐색 사망, 0.02: std 1.87 로 폭주) / **거친 지형 0.002** (§10.2) |
| 속도 [측정] | CAD 폐루프 약 8.7만 step/s (단순화 모델 39만의 1/4.4), 반복당 4–5 s |

### 10.2 이어서 학습할 때의 가중치 변환 (`scripts/add_mode_input.py`)

평지 정책(19 차원, $s_h=0.03$) → 모드 과제(20 차원, $s_h=0.12$). **처음 동작이 정확히 같도록** 망을 수술한다.

1. 첫 층에 모드 입력 열을 0 으로 끼운다. 모드가 처음에는 출력에 영향이 없다.
2. 다리 출력 $a_{\text{new}}=a_{\text{old}}\cdot\frac{0.03}{0.12}$: 출력층 다리 행과 std 에 $\tfrac14$ 를 곱한다.
3. 관측 속 직전 행동 두 칸도 $\tfrac14$ 이 되므로, 정규화 값이 같아지도록 통계를 고친다.

```math
\mu'=\tfrac{\mu}{4},\qquad \sigma'+\epsilon=\tfrac{\sigma+\epsilon}{4}
```

검증 [측정]: 변환 전후로 같은 지형을 재면 수동 모드 결과가 같다. 격리율 1.00 / 1.00, 낙상 3 / 2 (84 중). 차이는 다리 클램프 범위가 넓어진 탓이다.

4. **Adam 모멘트도 같은 수술로 옮긴다.** 매개변수가 $x\to Kx$ 이면 기울기는 $1/K$ 배이므로 $m\to m/K,\ v\to v/K^2$ 이다.

첫 판에서는 Adam 모멘트를 비우고 $c_e=0.006$ 으로 시작했다. 새 지형·보상에 크리틱이 맞춰지기 전에는 추종 기울기가 잡음이다. 그동안 방향이 일정한 엔트로피 기울기 $\partial(c_e\log\sigma)/\partial\sigma=c_e/\sigma>0$ 만 Adam 에 누적됐다.

결과 [측정]: 150 iter 만에 std 가 다리 0.11 → 0.61(실제 다리 잡음 1.3 → 7 cm), 바퀴 0.54 → 1.2 로 커졌고, 완주율은 64 % → 39 % 로 떨어졌다. 그래서 중단하고 모멘트 보존 + $c_e=0.002$ 로 다시 시작했다.

---

## 11. 학습 이력 — 무엇이 실패했고 무엇을 고쳤나

| 증상 | 원인 | 수정 |
|---|---|---|
| 20 s 완주 0 회, iter 100 뒤 퇴행 | 200 Hz 에 50 Hz 용 $\gamma,\lambda$ | §10.1 환산 |
| 바퀴 10 Hz 뱅뱅, 다리가 하한에 99.5 % 붙음 | 행동 무제한(원시 최대 194) | 행동 항 안 클램프 + 범위 벌점 + LPF 20 Hz |
| 보상이 엉뚱한 관절을 셈 | `SceneEntityCfg` 기본 인자는 매니저가 해석 안 함 | 반드시 `params` 로 전달 |
| 바퀴 속도가 1.7 rad/s 에서 막힘 | `max_angular_velocity` 단위가 deg/s | 3600 deg/s |
| 전진이 0.39 m/s 에서 포화 | 바퀴 관절 점성 감쇠 0.1 (가짜 브레이크) | DC 모터 모델, 감쇠 0 → 0.77 m/s |
| 정지 회전 보상 넣자 붕괴 | L2 벌점이 외란 회복까지 벌함 | 문 달린 상한 보상 (§6.4) |
| CAD 다리가 중력에 20° 처짐 | implicit 드라이브가 폐루프 구속력을 못 봄 | explicit PD |
| 거친 지형 이어 학습에서 행동 std 폭주 (평균 0.31 → 1.00) | Adam 모멘트 초기화 + 새 도메인에서 엔트로피만 일관된 기울기 | 모멘트 변환 보존, $c_e$ 0.002 (§10.2) |

---

## 12. 결과

### 12.1 평지, CAD 모델 (model_4650) — 21 조건 × 8 env [측정]

| 지표 | 값 |
|---|---|
| 낙상 | 2 / 168 (고속 좌코너 2) |
| 회전 추종 오차 (±2 rad/s) | 3 % |
| 전진 추종 오차 (0.3 m/s) | 평균 0.069 m/s |
| 고속 전진 (0.8 명령) | 0.735 m/s |
| 높이 추종 오차 | 평균 5.6 mm, 전환 90 % 도달 0.85–0.90 s |
| 코너 안쪽 기울기 | 목표 대비 평균 0.50° |

상세: `sim/results/2026-09-25_measure6_cad_m4650.md`

### 12.2 거친 지형 — 학습 전 기준선 (평지 정책을 지형에 올림) [측정]

`scripts/rough_probe.py`, 보지 못한 지형(seed 7). 7 종류 × 4 난이도 × 3 env, 전진 0.5 m/s 구간에서 잰다.
격리율 $\rho=\sigma(z_b-\bar T)/\sigma(\bar z_w-R-\bar T)$ 이다. 1 = 몸통이 바퀴를 그대로 따라감, 0 = 완벽한 짐벌.

| 모드 | 낙상 | 격리율 $\rho$ | 수직가속 RMS | roll RMS | 정지 중 밀림 |
|---|---|---|---|---|---|
| 수동 (h 0.1825) | 2 / 84 | **1.00** | 0.61 m/s² | 2.33° (최대 6.7°) | 3.1 cm (경사 18 cm) |
| 자동 (공칭 = CAD 기본자세) | 4 / 84 | 1.04 | 0.69 m/s² | 1.93° | **20.4 cm** |

해석: 평지 정책은 서스펜션 동작이 전혀 없다. 몸통이 강체처럼 바퀴를 따라 오르내린다.

### 12.3 거친 지형 + 높이 모드 — 중간 점검 (model_5100) [측정]

| 모드 | 낙상 | 격리율 | 수직가속 RMS | roll RMS | 정지 중 밀림 | 주행 중 다리 $h$ |
|---|---|---|---|---|---|---|
| 수동 | 2 → **0** / 84 | 1.00 → 1.05 | 0.61 → **4.18** m/s² | 2.33 → 1.77° | 3.1 → 6.3 cm | 181 → 172 mm |
| 자동 | 4 → 2 / 84 | 1.04 → 1.00 | 0.69 → 0.38 m/s² | 1.93 → 1.93° | 20.4 → 11.8 cm | 130 → **120** mm |

좋아진 것: 낙상, 경사에서의 롤과 밀림.

드러난 문제 세 가지와 대응:
1. **수동 모드 다리 떨림** (평지에서도 수직가속 1–3 m/s²). 권한 확대로 다리 부드러움 벌점이 1/16 이 된 탓이다 → `leg_rate`.
2. **자동 모드가 항상 최저 자세**(하한에 붙음) → 요철 흡수 여유가 없다 → `stroke_margin`.
3. 격리율이 여전히 약 1 → 짐벌 동작 미학습 → 승차감 σ 3 → 1.

같은 측정으로 "평지에서 바퀴 접지 − 지면 평균 = 0.2 mm" 를 확인했다. 짐벌 오차 식의 기준에 오프셋은 없다.
사용자 정의에 따라 수동 모드는 평지 전용으로 바꿨다(학습·측정 모두).

### 12.4 수정 후 이어 학습 (model_5100 → 5600) — 체크포인트 비교 [측정]

보지 못한 지형(seed 7), 전진 0.5 m/s. 수동 모드는 평지에서만 쟀다(사용자 정의).

| | 기준선 | 5100 | 5200 | 5400 | **5600** |
|---|---|---|---|---|---|
| 자동: 낙상 | 3/84 | 0/84 | 4/84 | 1/84 | **0/84** |
| 자동: roll RMS | 1.77° | 1.77° | 1.88° | 1.18° | **1.34°** |
| 자동: 정지 중 밀림 | 20.1 cm | 12.0 | 8.7 | 11.0 | **11.2** |
| 자동: 다리 평균 | 130 mm | 120 (하한) | 120 | 129 | **131** |
| 자동: 수직가속 RMS | 0.64 m/s² | 0.35 | 0.41 | 1.12 | **1.32** |
| **자동: 격리율** | 1.01 | 1.01 | 1.02 | 1.00 | **0.99** |
| 수동(평지): 수직가속 RMS | 0.43 m/s² | 2.81 | 3.75 | 0.78 | **0.77** |
| 수동(평지): 정지 중 밀림 | 1.0 cm | 2.9 | 9.9 | 18.8 | **9.6** |

학습 로그: 5400 이후 지형 단계가 5.2 → 4.0 으로 내려오고, 20 s 완주 env 비율이 79 % → 68 % 로 떨어져서 5600 에서 멈췄다.

**판단**:
- 좋아진 것: 낙상, 좌우 수평, 경사 밀림. `stroke_margin` 으로 하한 탈출(120 → 131 mm), `leg_rate` 로 수동 떨림 감소(3.75 → 0.77 m/s²).
- **핵심 목표인 짐벌(격리율)은 어느 체크포인트에서도 배우지 못했다(≈ 1.0).** 수동 평지 정지 밀림은 기준선보다 나빠졌다.

**원인 (관측 부족)**: 정책이 수직 충격을 느낄 신호가 없다.
- 관측에 가속도계가 없다.
- 다리 속도 $\dot h$ 에는 신호(< 0.1 m/s)보다 큰 잡음(±0.5 m/s)이 붙어 있다(§4 ⚠).
- 다리 부하(모터 토크)도 보지 않는다.

실기에서는 iAHRS 가속도와 AK60-6 전류로 모두 측정할 수 있다.

**다음 개정안**:
1. 관측에 몸체 가속도(IMU), 다리 모터 토크(전류 × $K_t$)를 넣고, $\dot h$ 잡음을 현실적 값으로 줄인다.
2. **비대칭 액터-크리틱**: 크리틱만 지형 높이 스캔을 본다. 정책은 여전히 블라인드라 실기 이식성은 그대로다. 크리틱의 가치 추정이 좋아지면 새 지형에서 std 가 폭주하던 문제도 줄어든다.
3. 수동 모드 평지 성능(정지 밀림 1 cm)을 지키는 회귀 검사를 학습 중 체크포인트 선택에 넣는다.

---

## 13. 실기 이식 체크리스트

1. 관측 20 칸의 **순서·단위·부호**를 §4 표대로 맞춘다. 특히 $g_b$ 부호와 바퀴 $+y$ 규약.
2. 다리: 정책 $h^*$ → `leg_map.theta_of_h` → $M^*$. 모터 한계는 $M\le52.5^\circ$ (θ 97.5°)에서 여유 없이 끊는다. 그 너머는 전달각 155° 로 사점에 가깝다.
3. 바퀴: $u=1.5\bar a$ → **LPF 20 Hz @ 200 Hz ($\alpha=0.466$)** → 토크 지령. 첫 기동에서는 추가 클램프 0.5 Nm.
4. IMU 가감속 편향: 가속 $a$ 가 있으면 AHRS pitch 가 다음만큼 편향된다(1 m/s² 에 5.8°).

```math
\Delta\theta\approx\arctan\frac{a}{g}
```

   $\omega_y$ 는 자이로를 직접 쓰고, 편향이 보이면 상보필터를 쓴다.
5. 200 Hz 루프, 모터 피드백 500 Hz 업로드. CAN 부하 56 % 로 4 모터 한도다([통신](02_communication.md)).

---

## 부록 A. 고전 제어 기준선 — VMC + LQR (Liu & Wang, 2024)

실기 첫 기동은 이 제어기로 한다(`sim/control/vmc_lqr_design.py` → `gains.yaml`).

```math
x=[\theta,\ \dot\theta,\ x-x_d,\ \dot x-v_d]^\top,\qquad u=T_\theta=-K(h)\,x,\qquad
K=\arg\min\int_0^\infty\!\big(x^\top Qx+u^\top Ru\big)\,dt,\ \ Q=\text{diag}(6000,1,1000,100),\ R=100
```

```math
K_{ij}(h)=p_0+p_1h+p_2h^2\ \ (\text{10 mm 간격으로 풀고 맞춤}),\qquad
T_{L}=\tfrac12T_\theta+\tfrac12T_\delta,\ \ T_{R}=\tfrac12T_\theta-\tfrac12T_\delta
```

```math
F=\frac{Mg}{\cos\theta}+\text{PID}(h_d-h),\qquad \tau_{\text{hip}}=J(\theta_m)^{\!\top}F=\frac{\partial h}{\partial\theta_m}F
```

| $h_{\text{hip}}$ [mm] | $K_\theta$ | $K_{\dot\theta}$ | $K_x$ | $K_{\dot x}$ | $\tau_{\text{hip,ff}}$ [Nm] |
|---|---|---|---|---|---|
| 182.5 | −11.85 | −1.14 | −3.16 | −3.42 | 2.30 |
| 212.5 | −12.32 | −1.28 | −3.16 | −3.37 | 2.41 |
| 242.5 | −12.76 | −1.43 | −3.16 | −3.33 | 2.37 |
| 302.5 | −13.58 | −1.73 | −3.16 | −3.31 | 2.42 |

RL 과 VMC+LQR 은 **같은 인터페이스**(명령 $v_x^*,\omega_z^*,h_{\text{ref}}$, 다리 $h\leftrightarrow\theta$ 는 leg_map)를 쓴다. 그래서 실기에서 두 제어기를 바꿔 끼울 수 있다.
