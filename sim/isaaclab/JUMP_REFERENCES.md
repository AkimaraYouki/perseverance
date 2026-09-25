# 점프 — 선행 연구 정리와 학습 방향 (학습 에이전트용 지침)

2026-09-26 작성. 사용자 요청: "다른 논문은 점프를 어떻게 했나 — RL로 학습? 점프 정책만 잠깐? 동역학 분석으로 입력?"
이 문서는 **참고용 판단 근거**다. 지금 r6 설계를 버리라는 뜻이 아니다. 새 실험을 고를 때 여기 순서를 따른다.
논문 요약은 초록·배경지식 기준이라 원문으로 확인하지 않았다 (특히 Ollie 점프 방식은 추정). 수치를 인용할 때는 원문을 연다.

## 1. 선행 연구는 세 갈래

| 방식 | 대표 논문 | 요점 |
|---|---|---|
| **A. 동역학·궤적 기반** (상태머신 + 피드포워드 + 균형제어) | [Ascento, ICRA 2019](https://arxiv.org/abs/2005.11435) (ETH, 두바퀴 점프) | 웅크림 → 다리 급신장 → 공중 수축 → 착지. 점프 다리 궤적은 미리 정한 피드포워드, 균형은 LQR 이 계속. **우리 VMC+LQR 과 구조가 가장 가깝다** |
| | [Ollie, ICRA 2021](https://dl.acm.org/doi/abs/10.1109/ICRA48506.2021.9561579) (Tencent) | 모델 기반 균형 + 점프·착지 궤적 계획 (추정) |
| | [Underactuated Motion Planning and Control for Jumping With Wheeled-Bipedal Robots](https://www.researchgate.net/publication/348059579_Underactuated_Motion_Planning_and_Control_for_Jumping_With_Wheeled-Bipedal_Robots) | 궤적최적화(TO)로 점프 궤적 → 추종 |
| | [MIT Cheetah 3 점프, ICRA 2019](https://arxiv.org/abs/1902.00513) | 오프라인 TO + PD 추종 (다리형) |
| **B. 점프 전용 RL 정책을 잠깐 켬** | [Cassie 점프, RSS 2023](https://arxiv.org/abs/2302.09450) | 목표 높이·거리를 입력받는 점프 전용 정책 |
| | [Robot Parkour Learning, CoRL 2023](https://arxiv.org/abs/2309.05665), [ANYmal Parkour, Sci. Robot. 2024](https://arxiv.org/abs/2306.14874) | 스킬별 정책(점프 등) + 상위 선택기 (또는 나중에 한 정책으로 증류) |
| **C. 한 RL 정책에 점프 포함** | [Extreme Parkour, ICRA 2024](https://arxiv.org/abs/2309.14341), [Walk These Ways, CoRL 2022](https://arxiv.org/abs/2212.03238) | 명령값·지형 입력에 따라 한 정책이 점프까지 |
| | [바퀴-다리 MoE 다중모드 정책](https://pmc.ncbi.nlm.nih.gov/articles/PMC12975443/) | 전문가 여러 개를 한 네트워크로 묶음 |
| 혼합 | [Learning to Jump from Pixels, CoRL 2021](https://arxiv.org/abs/2110.15344) | RL 은 상위 결정만, 하위 제어는 모델 기반 |

**경향:** 바퀴-다리 로봇 실기 점프는 A 가 주류. RL 점프는 B(전용 정책)로 시작해서 C 로 합치는 순서가 흔하다.
C 를 처음부터 하는 건 보상 설계·커리큘럼 부담이 가장 크다.

## 2. 우리 저장소는 이미 A 와 C 를 둘 다 갖고 있다

- **C = r6** (`Isaac-WheeledBiped-CAD-Jump-v0`): 명령 `[vx, wz, h_ref, m, j]`, 버튼 j 후 `jump_window_s` 동안
  `jump_clearance` 보상, 점프 중 높이·승차감·정지·다리 부드러움 보상 끔 (`env_cfg.py` `WheeledBipedCADJumpEnvCfg`).
- **A (시뮬 검증용) = `scripts/climb_test.py --mode jump`**: 균형은 학습된 정책, 다리만 스크립트
  (웅크림 → 최대 신전 → 공중 수축 → 착지). Ascento 와 같은 구조.

## 3. 학습 에이전트가 할 일 (순서대로)

1. **물리적으로 되는지 먼저.** `climb_test.py --mode jump` 로 8 cm 턱 기준 바퀴 최대 높이·고관절 토크/속도 포화를 잰다
   (DC 고관절 모델, 9 N·m, 무부하 33.5 rad/s — 역기전력 계산값, 실측 전).
   스크립트 점프로도 목표 clearance(10 cm)가 안 나오면 **RL 보상을 만져도 안 된다**. 그 경우 목표를 낮추거나 사용자에게 보고.
2. **r6(C)가 점프를 못 배우면 B 로 바꾼다.** 균형·주행 정책은 그대로 두고, 버튼 구간(이륙~착지)만 맡는 점프 전용 정책을 따로 학습한다.
   - 초기 상태를 "주행 정책이 굴러가던 상태"에서 뽑는다 (전환 시점 분포 일치).
   - 착지 후 기존 정책으로 넘겨도 안 넘어지는지(전환 성공률)를 평가 지표에 넣는다.
3. **보상에 A 의 단계 구조를 빌려 쓴다** (C·B 공통). 논문들이 공통으로 쓰는 요령:
   - 이륙 순간 수직 속도(또는 몸통 높이 최대값) 보상 — clearance 만으로는 희소하다.
   - 공중 다리 수축, 착지 순간 다리 여유(행정 끝에 붙지 않기)·pitch 보상.
   - 필요하면 `climb_test.py` 의 스크립트 궤적을 참조 궤적으로 두고 추종 보상(모방)을 초기에만 준다.
4. **실기 첫 점프는 A 로 한다.** 로봇 쪽에 VMC 다리 힘 피드포워드 펄스 + 공중 중 바퀴 토크 0 + 착지 임피던스 하강을
   상태머신으로 요청한다 (`comms/to-robot/` 메시지로). RL 점프는 시뮬에서 A 보다 낫다는 게 확인된 뒤 넘긴다.

## 4. 보고

실험마다 `sim/results/` 에 방식(A/B/C), 체크포인트, 바퀴 최대 높이, 성공률, 고관절 포화 비율을 남긴다.
