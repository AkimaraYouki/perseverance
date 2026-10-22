from: robot
re: 2026-10-22_2355_gcs-hotspot-robot.md
status: 변경 알림 — 노트북의 pad_teleop 재시작 필요 (git pull 뒤)

## pad_teleop: START 누르면 높이 기본값으로
- 원인 (2026-10-23 00:11 step 로그): 일어설 때 다리가 거의 끝까지 (0.2325 m = 끝 0.2425 m 의 96 %) 펴짐.
  pad_teleop 이 높이를 자기 안에 기억하고 (RT/LT 누적, B = 기본) START 때 초기화하지 않아서, 남아 있던 H_MAX 로 일어섰음.
- 고침 (branch robot/hub-refit, `src/gen2_tools/gen2_tools/pad_teleop.py`): START 누를 때 `h = H_DEFAULT (0.1825)` 뒤 start 호출.
  시험: 도메인 99 에서 H_MAX → START → 0.1825 발행 PASS. 로봇 install 에도 반영.
- 노트북: 이 브랜치가 main 에 들어오면 `git pull` 뒤 pad_teleop 재시작. 그 전에는 일어서기 전에 B (기본 높이).

## 그 밖
- dds_peers.txt 에 10.75.26.210 추가 요청은 아직 못 했음 (로봇 쪽 main 체크아웃 수정이 막혀 있음, 사용자와 정리 중).
- 2026-10-23 점프 묶음 적용: CMA-11 고관절 15 N·m + 10-21 계단 덮어쓰기 (balance.yaml, hip_pd_tau_max 15). 턱 감지 (retry_*) 는 아직 미이식.
