from: desktop
re: 2026-10-28_gcs-lidar-cluster.md (로봇 설치본 = gcs/lidar-cluster, auto_step true, explore true)
status: 긴급 (본경기 당일) — 버그 수정 브랜치 desktop/fix-explore-step (gcs/lidar-cluster + 1 커밋). 설치 전까지는 GUI 로 둘 중 하나 끄기

## 버그
person_follow_cam._control: 사람을 occluded_s (5 s) 넘게 놓치면 EXPLORE 로 가는데, EXPLORE 는 _auto() 보다 먼저 return.
→ 자동 턱이 STEP (balance/step_mode ON) 인 동안 사람을 5 s 넘게 놓치면 (2 단 계단 ~20 s, 점프로 화면 흔들림) 턱 감지 모드가 켜진 채
  0.4 m/s 로 탐색 지점까지 회전·주행 → 아무 데서나 막히면 후진·재주행·점프할 수 있음.
## 고침 (1 줄 조건)
STEP 중에는 EXPLORE 로 안 감 (계단 먼저 마침: 기존 코드대로 회전 0, 전진 step_v). 시험: test_explore_ros ok, test_auto_step_ros ok,
새 확인 (STEP + 7 s 놓침 → SEARCH 분기, wz 0, vx 0.3, step mode 유지) ok.
## 요청
- 설치 가능하면 desktop/fix-explore-step 로 gen2_tools 빌드·설치 (추종 IDLE 일 때 재시작).
- 설치 못 하면 경기 동안 GUI 에서 **자동 턱 (카메라) 끄기** 권장 (실기 미시험 + 이 충돌). 쓸 거면 가려짐 탐색을 끄기.
