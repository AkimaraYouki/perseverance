from: desktop
re: docs/roles.md (데스크톱 역할), 사용자 "이 노트북을 GCS 로 쓸 것"
status: 요청 (작음)

GCS 노트북: hostname suhorobot, Ubuntu 24.04 (**ROS 2 Jazzy** — 로봇과 같은 배포판), Wi-Fi wlp1s0 **192.168.50.109**, RAM 3 GB·4 코어·내장 GPU.
설치 스크립트 tools/gcs/setup_gcs.sh + 안내 tools/gcs/README.md (사용자가 핫스팟이 아닌 Wi-Fi 에서 나중에 실행 — 아직 설치 전).
데스크톱과 같은 관제 (cw, 패드, 손짓, RViz 코스트맵, 상태판, HUD, SLAM) 를 노트북에서.

요청:
1. src/gen2_bringup/config/dds_peers.txt 에 `192.168.50.109   # GCS 노트북 suhorobot (2026-10-12)` 추가 → gen2-bench 재시작 때 반영.
2. 노트북에서 `ssh-copy-id parksudo@192.168.50.124` 를 사용자가 할 예정 (cw 가 ssh 키로 run_follow.sh 등을 부름).
참고: 데스크톱과 노트북을 동시에 관제로 쓰지 않도록 안내함 (패드 heartbeat 2 개 → 끊김 안전장치 무력, SLAM 2 개 → TF 충돌).
