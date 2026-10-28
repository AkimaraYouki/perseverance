from: desktop
re: 2026-10-28_desktop-jump-friction-sets.md (GUI 점프 세트 요청) — 합판 세트 완성
status: 요청 — balance_tune_plywood.yaml 을 로봇에 (bundle) 설치 + GUI '점프 세트 (턱 재질)' 에 '합판 (미끄러움)' = gen2_tune.sh plywood

## 결과 (시뮬, 실기 보정 플랜트, 대회 2 단 칸 1 m, 턱 감지 모드, 2048 판)
| 턱 마찰 | 2 단 6 cm 기본 → 합판 | 2 단 8 cm 기본 → 합판 |
|---|---|---|
| 0.5 | 0.3 → 11.6 % | 0.5 → 17.3 % |
| 0.7 (합판 ≈ 1 일차) | 33.3 → 51.6 % | 35.2 → 58.4 % |
| 1.0 (잘 붙음) | 76.5 → 62.2 % | 73.5 → 58.4 % |
착지 pitch −15 → −9° (더 수직), 착지 뒤 후진 없음. 잘 붙는 판에서는 기본이 나음 → 세트를 고르는 이유.
주의: 착지 바퀴 하중 ~410 → ~580 N — 연습 뒤 바퀴 허브 확인.

## 설치
- main 의 src/gen2_control/config/balance_tune_plywood.yaml → 로봇 ~/gen2_ws/install/gen2_control/share/gen2_control/config/ 에도 (launch 가 읽는 곳),
  gen2_tune.sh plywood 로 확인 (launch 로그 'tune overlay: PLYWOOD', 13 키 값 확인). 기본은 default 그대로.
- 내일 연습: 해제 상태에서 세트 바꾸고 6 cm 한 단으로 기본 vs 합판 비교 → 나은 쪽으로 8 cm. 배터리 22 V 충전된 것.
