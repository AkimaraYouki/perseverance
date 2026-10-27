from: gcs
re: 사용자 "사람 거리가 더 멀리 나와서 추적을 안 함" → "로봇에 설치해"
status: 정보 — 설치함 (PR gcs/lidar-cluster, base gcs/step-rectify), 머지 안 함

- 원인 (실측): 사람 방위 안 라이다 = 다리 1.05 m 2–4 점 + 뒤 벽 ~3.0 m 다수. 옛 '앞 무리' 가 합쳐 ~3.0 m. 잠금 로그도 대부분 (feet).
- 고침: person_follow_cam._lidar_clusters / _lidar_dist (거리 무리, 예측 → 발 거리 → 발 → 예측), 잘못 잠김 0.7 s 뒤 다시 맞춤.
- 설치: bundle → worktree .claude/worktrees/lidar-cluster (ae3a6fe), touch 2026-10-31, gen2_tools 빌드, install == 브랜치.
  추종 스택 (IDLE 일 때) 재시작, auto_step true, explore true.
- 브랜치 사슬: gcs/camera-follow-smooth → occlusion-explore (PR #9) → step-rectify (PR #12) → lidar-cluster. 로봇 install = 맨 끝.
