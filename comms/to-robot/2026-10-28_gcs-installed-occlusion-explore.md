from: gcs
re: PR #9 (gcs/occlusion-explore, base gcs/camera-follow-smooth) — 사용자 "설치해줘, 접속해서 ㄱㄱ"
status: 정보 — 설치함 (머지 안 함)

- 로봇 192.168.50.124 ("ROBOT"), disarmed. **시계 다시 1970** → git bundle 로 0d68943 전달, worktree `.claude/worktrees/occlusion-explore`.
- 설치 전 install 의 person_follow.py / person_follow_cam.py = 2b41fe4 (이전 GCS 설치) 확인 → 이 두 파일만 바뀜.
  소스 `touch -d 2026-10-29` 뒤 `colcon build --packages-select gen2_tools` → install == 0d68943 (cmp 전부 같음).
- 추종 스택 (카메라 중심) stop_follow.sh → run_follow_cam.sh 재시작. /person_follow: explore true, explore_s 12, dist_gate_m 0.35,
  follow_accel 0.4, auto_step false. status IDLE. person_tracker 대기 2.7 fps (추종 꺼짐), 오류 없음.
- 실기 시험 전. 권장: 상자 모서리를 돌아 사라지기 → 기둥 뒤 지나가기. follow/status 에 'EXPLORE GO n/3 …' / 'EXPLORE SCAN n/3'.
- 노트북: GUI 만 갱신 (24272b5, 기능 탭 '가려지면 탐색', 튜닝 '가려짐 탐색'). 노트북은 추종 노드를 안 돌려 gen2_tools 재빌드 불필요.
