from: desktop
re: 사용자 2026-10-25 "추종 노드를 켜면 9–10 FPS, 블러돼서 잘 안 됨. 다리 추적 (LiDAR) 은 현실성 없음. 추종·명령 최고 0.7 m/s 로 Wave 통과해야 함, 160° 광각에 희망"
status: 요청 — 1) 어디서 느린지 측정 2) 블러 줄이기 3) 다리 추적 빼고 카메라 중심 추종으로 단순화 (제안). 측정 결과 보고 같이 정함

## 1. 측정 (추종 켬 / 끔 각각, 같은 장소·조명)
- 카메라 자체 fps (LCD CAM 줄 / diagnostics "camera: stream"), people/camera 발행 Hz (`ros2 topic hz`), follow/status·cmd_vel/auto Hz.
- person_detector 단계별 ms: JPEG 해독 / 전처리 / TRT / 후처리 (HSV·NMS) / 발행 — 로그로 1 s 평균.
- `top -H` (또는 tegrastats) 로 CPU 코어별·프로세스별 점유: person_detector, person_follow, leg_detector, Nav2 controller_server (MPPI batch 1000 x 40 스텝 20 Hz 가 의심), SLAM, balance_node.
- 데스크톱 추정: 검출 한 장 ~10–15 ms (해독 반해상도 + TRT 3 ms + 파이썬 후처리) 라 혼자면 30 Hz 가 나와야 함 → 9–10 FPS 는 CPU 경쟁 (MPPI 등) 이나 카메라 노출 쪽일 가능성.

## 2. 블러
- 지금 nvarguscamerasrc 기본 자동노출 → 실내에선 노출이 프레임 길이 (33 ms) 까지 늘어 흔들리면 번짐.
- 제안: 노출 상한 8 ms 안팎 + 게인으로 보충 (`exposuretimerange="100000 8000000"`, `gainrange="1 16"`, 필요하면 `ispdigitalgainrange`), 사진 몇 장으로 노이즈 vs 번짐 비교.
  가능하면 1280x720@60 센서 모드를 640x480 으로 줄여 받기 (프레임 16 ms).

## 3. 추종 구조 단순화 (제안, 사용자 "다리 추적은 현실성 없음")
- 사람 찾기·지정 인물 = 카메라만 (YOLO + 옷 색 / 이후 필요하면 가벼운 ReID). LiDAR 는 '카메라가 준 방위의 거리 재기 + 장애물' 만:
  사람 상자 좌우 방위 구간 안의 scan 점 중앙값 = 거리 (없으면 발목 행으로 지면 거리, 지금 카메라 단독 경로).
- leg_detector 노드는 끄고 (CPU 절약), 기존 퓨전 경로는 옵션으로 남김.
- person_follow vx_max 기본 0.83 → 0.7 (사용자: 추종·명령 최고 0.7 m/s, Nav2 는 이미 0.7).
- Wave (좌우 12/14°) 위에서 몸이 흔들려도 160° 광각이라 사람은 화면 안에 남을 것 — 대신 상자 좌표가 출렁이니 방위에 짧은 저역 (예: 5 Hz) + 몸 roll/pitch 로 방위 보정은 검토.

## 데스크톱 쪽
- 시뮬에 대회 Wave (좌우 12/14° 번갈아) + 사람 앞 추종 명령 (0–0.7 m/s, 멈춤, 작은 방향 변화) 을 만들어 균형·롤 게인 확인 중 (롤1 이 가장 효과 클 지형).
