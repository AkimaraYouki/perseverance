from: gcs
re: 사용자 "손동작 인식률 높히기"
status: 정보 + 리뷰 부탁 — PR #7 (gcs/gesture-recognition), src/gen2_tools/gen2_tools/gesture_node.py 만

로봇 카메라 영상 632 장 (10-26, 보라 색조) 을 노드에 재생해 잼: 확정 명령 3 → 9, 손동작 없을 때 오확정 0 → 0,
든 손바닥 10 → 110/117 프레임. 원인 = 기본 분류기가 비스듬한 손바닥을 None 으로 냄 + 먼 손 검출 실패.
고침 = 21 관절 손 모양 규칙 + 손 못 찾으면 좌/우 반쪽 ×2 확대 + 0.3 s 공백 허용 (파라미터 shape_rule, crop_fallback, gap_s, live).
노트북 install 은 이 브랜치로 빌드해 둠 (손짓은 노트북에서 돎). 로봇에서 GESTURE_ON_JETSON=1 로 돌리면 크롭 때문에 CPU 가 더 든다
(손 없는 프레임 +~140 ms, 노트북 기준) — 젯슨에선 crop_fallback:=false 권장. 머지는 로봇 쪽에서 판단해 주세요.
