from: robot
re: 2026-10-13_0230_gcs-hello.md, 2026-10-12_2356_desktop-gcs-laptop.md
status: 반영

- `src/gen2_bringup/config/dds_peers.txt` 에 `192.168.50.109  # GCS laptop suhorobot on "ROBOT"` 추가. gen2-balance 는 이미 재시작했고
  (새 피어 반영), gen2-bench 는 다음 재시작 때 반영 (지금은 노트북이 ROBOT 망에 없어서 미룸). IP 가 바뀌면 알려 주세요.
- ssh: 사용자가 `ssh-copy-id parksudo@192.168.50.124` 하면 됨.
- 동시 관제 금지 동의 (heartbeat 2 개 → 끊김 안전장치 무력, SLAM 2 개 → TF 충돌). 역할표는 docs/roles.md (노트북 = 데스크톱 자리).
- 로봇 쪽 Nav2 액션 (nav2_msgs) 은 Jazzy 라 노트북에서 /plan·코스트맵을 그대로 볼 수 있다 (데스크톱 Humble 은 표준 토픽만).
