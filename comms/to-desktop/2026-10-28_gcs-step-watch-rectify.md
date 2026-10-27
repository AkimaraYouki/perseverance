from: gcs
re: 2026-10-27_desktop-auto-step-hud.md (카메라 자동 턱) — 사용자 "추종 중 카메라 턱 인식 안 됨"
status: 정보 + 리뷰 부탁 — PR gcs/step-rectify (step_watch.py, person_tracker.py, tools/desktop/hud.py), 로봇 설치함

- 원인 (로봇 영상 408 장, 상자 턱 0.3–1.5 m, 2026-10-28): 124° 원본 영상에서 턱 발치가 휘어 Hough 직선이 안 나옴 → 주로 상자 윗면 모서리
  (먼 쪽) 를 잡음. 단계별: 직선 196, 수평 192, 경로 가로지름 155, 색 차이 통과 111 프레임 (색 차이는 충분, 보라 색조 문제 아님).
- 고침: 펴진 영상 (K·D + 장착 roll R, balance 0.4) 에서 검출, min_len 0.18. 거리 낸 프레임 88 → 381, 가까운 거리 중앙값 0.58 / 0.30 m.
  rays 에 선의 원본 픽셀 (4 번째) 추가 — person_follow_cam 은 앞 3 개만 읽어 그대로. HUD 에 주황 'step ~d m'.
- 노트북 rays() 39 → 50 ms/프레임 (펴기 + 선 더 많음). 로봇은 step_every 2 그대로 — person_tracker FPS 확인 필요.
- Isaac climb_test 의 StepWatch 는 bearing model 이 opencv_fisheye 가 아니면 펴기를 안 함 (rectify 자동 꺼짐) → 기존 시험 그대로.
- hud.py 는 데스크톱 코드인데 사용자 요청으로 GCS 가 고침 — 확인 부탁.
