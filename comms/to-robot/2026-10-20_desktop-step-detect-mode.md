from: desktop
re: 사용자 요청 "턱 감지 주행 모드 (평지 전용): 천천히 이동 → 앞으로 안 감 = 턱 → 적절한 거리 후진 → 가속 → 점프"
status: 시뮬 완료 → wb_core 포팅 요청 (골든 포함). 실기는 재조립 + 사용자 OK 뒤. 고관절 상한 15 N·m 묶음 전제.

## 로직 (wbctrl desktop/jump-rl, 기존 jump_retry 확장 — wb_core 에는 jump_retry 자체가 아직 없음)
조종자가 앞으로만 밀면 (vx > 0.05, retry_approach_v > 0 일 때):
1. 접근: vx = retry_approach_v (천천히)
2. 막힘 판정: DRIVE, 안 들림, |v| < 0.05 m/s 가 retry_block_s 이어짐 → 그 자리 오도메트리 x_b 기록
3. 후진: vx = −retry_back_v 로 x_b − retry_back_m 까지
4. 재가속: vx = retry_v
5. 발사: 오도메트리 ≥ x_b + retry_contact − trigger 에서 점프 요청 (바퀴 중심이 모서리 앞 trigger). 그 뒤 점프 상태기계 그대로.
조종자가 앞 명령을 놓으면 언제든 취소. 짧게 착지 (턱 앞) 하면 다시 막힘 → 자동 재시도.
odo = ∫ v_now dt (DRIVE 중 추정 속도). retry_approach_v / retry_v = 0 이면 예전처럼 스틱 속도.

## 최적값 (CMA-ES, 보정 플랜트: 고관절 실효 0.3–0.4, w0 29.3, 명령 상한 15 N·m, 턱 6–12 cm)
```yaml
jump_retry: true
retry_approach_v: 0.156   # m/s
retry_block_s: 0.43       # s
retry_back_m: 0.40        # m
retry_back_v: 0.3
retry_v: 0.326            # m/s (재가속 = 점프 접근 속도)
retry_contact: 0.07       # 바퀴 반지름
trigger: 0.311            # m
```
시뮬 (각 2048 판): 턱 6 / 8 / 10 / 12 cm 성공 58 / 63 / 61 / 63 %, 넘어짐 19 / 13 / 15 / 13 %, 턱 앞 착지 24 % (실기에선 자동 재시도).
같은 15 N·m 묶음을 무작위 거리에서 점프하면: 10 cm 58 % / 넘어짐 27 %, 12 cm 47 % / 37 % → 넘어짐 절반.
15 N·m 최대 높이 (평지): 몸통 상승 중간 8.8 cm (최대 11.3), 바퀴 바닥 최고 13.9 cm (최대 16.9).

## 실기 순서 제안
평지 막힘 (상자 벽) 으로 1–4 단계만 (점프 끄고) → 후진 거리·재가속 속도 확인 → 6 cm → 8 cm → 10 cm.
