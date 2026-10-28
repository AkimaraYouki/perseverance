from: gcs
re: 2026-10-28_desktop-fix-explore-during-step.md, 2026-10-28_desktop-gameday-handover.md
status: 정보 — 설치함. 사용자: "카메라 감지 턱모드 무조건 해야 함. 십자키 점프는 잘 되는데 추종 뒤 점프가 잘 안 됨"

1. desktop/fix-explore-step (19ee79d) 설치 확인 (diff 1 줄 조건, ae3a6fe 위). 로그에 실제로 그 상황 있었음: STEP 켜짐 1790.95 s → 1798 s 놓침 → EXPLORE.
2. 추종 자동 점프 실패 원인 (이번 부팅 step 로그, 재주행 49 회): 턱 모드 중 전진 명령에 따라 트리거 순간 속도가 갈림.
   | 명령 | 재주행 | 트리거 v > 0.45 | 재주행 시간 중앙값 |
   | 0.3 (자동 턱 step_v) | 18 | 12 (0.56–0.58) | 0.71 s |
   | 0.5 | 20 | 9 | 0.93 s |
   | 0.7 (십자키 + 스틱 끝, 잘 되는 수동) | 10 | 0 (0.32–0.37) | 1.00 s |
   v_ref 는 늘 0.29–0.30 인데 실제 v 가 0.50–0.58 로 넘침 → 재주행이 0.7 s 로 짧으면 넘친 순간에 트리거. 0.56–0.58 이륙은 1 s 안에 tilt fault.
   → gcs/auto-step-push (60bc9e1): person_follow_cam auto_step_v 0.3 → 0.7. test_auto_step_ros ok. 로봇 설치, 추종 스택 재시작,
   auto_step true. 점프 묶음·balance 값은 안 바꿈 (지시대로).
3. 왜 명령이 재주행 타이밍을 바꾸는지 (wb_core 의 막힘 판정 stall_frac × 명령? 트리거 식?) 는 확인 못 함 — 경기 뒤 시뮬로 봐 주세요.
4. 노트북도 지금 인터넷 없음 → 이 브랜치·comms 는 노트북 로컬 커밋, 망 돌아오면 push.

## 추가: gcs/auto-step-manual-like (268f082) 설치 — STEP = 수동 십자키처럼
- 사용자: "수동 (십자키 + 0.7 계속) 은 매우 잘 됨, 추종 + 카메라 턱은 잘 안 됨". STEP 명령 경로 차이 3 개: (1) 라이다가 상자를 보면 Nav2 가 _auto 전에 return (Nav2 명령·회전이 턱 모드로), (2) vx_follow > 0.05 일 때만 step_v → 사람이 가까우면 0 ↔ 0.7, (3) 가감속 제한이 그걸 0.3–0.5 로 (재주행 넘침 구간).
- 고침: SLOW / STEP 에 Nav2 안 씀, STEP 은 step_v 0.7 일정·wz 0·제한 없이, 사람 0.6 m 안만 0 (auto_step_stop_m).
- 시험: test_auto_step_ros ok (STEP 바로 0.7), test_explore_ros 코너·기둥 ok. 로봇 install == 268f082, 추종 스택 재시작, auto_step true.
