from: desktop
re: 사용자 2026-10-28 (대회 1 일차: 합판 턱 미끄러워 6 cm 1 단만 성공, 2 단째 앞으로 고꾸라짐 / 내일 8 cm) "GUI 에 점프 마찰별 세트 선택할 수 있게 요청해"
status: 요청 — cw-gui 에 점프 세트 (턱 재질 마찰별) 선택. 세트 파일은 데스크톱이 오늘 밤 최적화해서 comms 로 보냄

## 근거 (시뮬, 실기 보정 플랜트, 2 단 칸 1 m, 2048 판, 지금 기본 묶음)
| 턱 마찰 (바퀴 x) | 2 단 6 cm | 2 단 8 cm |
|---|---|---|
| 1.0 (이전 시뮬) | 76.5 % | 73.5 % |
| 0.7 (합판 ≈ 경기 결과) | 33.3 % | 35.2 % |
| 0.5 | 0.3 % | 0.5 % |
넘어지는 순간 = 주로 첫 단 착지 직후 (착지 pitch −15° 로 젖혀진 채 미끄러져 뒤로, 또는 되잡다 앞으로). → 마찰 0.7 에서 '수직 착지' CMA 진행 중.

## GUI 요청
1. 기능 탭에 줄 '점프 세트 (턱 재질)': 버튼 또는 드롭다운
   - `기본 (마찰 높음)` → `gen2_tune.sh default`
   - `합판 (미끄러움)` → `gen2_tune.sh plywood` (balance_tune_plywood.yaml, 데스크톱이 곧 보냄)
   - 나중에 더 생기면 같은 방식 (예 `plywood_slick` 마찰 0.5).
   상태 칸 = 지금 켜진 세트 (~/.ros/gen2_tune 내용 또는 launch 로그 'tune overlay: ...').
2. gen2_tune.sh 는 gen2-balance 를 재시작하고 서 있으면 거부 → 버튼은 **해제 (disarmed) 일 때만** 활성, 아니면 '앉힌 뒤 바꾸기' 안내.
3. 세트 파일이 로봇에 없으면 (`config/balance_tune_<이름>.yaml`) 버튼 비활성 + '설치 전'.
4. 롤1 (balance_tune_roll1.yaml) 과 동시에 못 씀 (gen2_tune 은 하나만) — 오늘은 롤1 안 씀이라 문제 없음. 둘 다 필요해지면 gen2_tune 을
   여러 이름 (`gen2_tune.sh plywood roll1`) 받게 바꾸는 건 로봇 쪽.

## 순서
데스크톱 CMA (~2.5 h) → 검증 (마찰 0.5 / 0.7 / 1.0 x 6 / 8 cm, 기본과 비교) → 좋으면 balance_tune_plywood.yaml 을 main 에 + comms →
GCS 가 bundle 로 로봇 설치 + GUI 세트 선택. 내일 연습 때 6 cm 로 기본 vs 합판 비교 뒤 8 cm.
