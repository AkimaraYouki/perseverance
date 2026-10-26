from: desktop
re: 사용자 2026-10-26 — 화면 보라색 + "크롭인지 축소인지" + 체커보드 캘리브레이션 준비
status: 알림 + 요청 — 데스크톱이 로봇에서 직접 바꾼 것 (사용자 지시), 소스 반영 (이 커밋), 다음 빌드 때 설치본 정리

## 바꾼 것 (로봇, 10.75.26.250)
1. ISP 보정 (Waveshare camera_overrides.isp, 텍스트 설정): /var/nvidia/nvcam/settings/camera_overrides.isp (root 664),
   nvargus-daemon 재시작 (camera_node 한 번 죽고 respawn 됨). em.* 속성 일부는 이 JetPack 에서 "Invalid isp config attribute" (무시됨), 색은 좋아짐.
2. 크롭 원인: Argus 자동 선택이 센서 모드 2 (1920x1080, 가운데 크롭, 가로 화각 ~59 %) → 640x480 으로 세로 늘림.
   설치본 camera.yaml 에 pipeline 직접 지정으로 모드 3 (1640x1232 전체 4:3) 고정 (백업 camera.yaml.bak_before_mode3).
   소스는 이 커밋에서 정식 파라미터 sensor_mode (기본 -1 자동, config 3) 로 — colcon build gen2_camera 뒤 설치본의 pipeline 덮어쓰기는 지워질 것 (의도대로).
## 요청
- gen2_camera 빌드·재시작해 Camera mode = 3 확인. 화각이 넓어져 person 방위·거리 (cam_fx 530 기준) 가 바뀜 → 캘리브레이션 (K, D) 결과 나오면 person_tracker / person_follow 에 넣을 것.
