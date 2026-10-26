from: gcs
re: PR #8 (gcs/follow-smooth) — 사용자 "로봇에 설치까지"
status: 정보 — 설치함 (머지 안 함)

- 로봇 시계 1970 → GitHub TLS 실패라 노트북에서 git bundle 로 origin/gcs/follow-smooth (f20d014) 를 넘김 → worktree
  `~/gen2_ws/.claude/worktrees/follow-smooth`. 설치본과 다른 파일은 person_follow.py / person_follow_cam.py 둘 (이 PR 변경만) 확인 뒤 빌드.
- **주의 (시계 1970)**: 새 파일 mtime 이 1970 이라 colcon/setuptools 가 기존 build·install (2026 mtime) 을 그대로 둔다 — 빌드는 "성공" 인데
  안 바뀜. `rm -rf build/gen2_tools/build` + 소스 `touch -d 2026-10-27` 뒤 빌드해서 설치본 = 브랜치 확인 (cmp). chrony 로 시계를 맞추기 전엔 같은 함정.
- 추종 스택 (카메라 중심) 을 disarmed 에서 stop_follow.sh → run_follow_cam.sh 로 재시작. `/person_follow follow_accel` = 0.4 확인, status IDLE.
- 로봇 main 체크아웃은 그대로 (behind 34, run_follow.sh·golden_jump.csv 수정분). PR #8 머지 뒤 main 으로 빌드하면 worktree 는 지워도 됨.
