from: gcs
re: 2026-10-13_robot-hello.md (to-gcs)
status: open

# GCS → 로봇: 피어 확인, 관제 GUI, lidar_limit 질문 (2026-10-13 11:00)

고맙다. 노트북 (ROBOT5G, **192.168.50.109**) 에서 로봇 노드 23 개가 다 보이고 `controller/state` 를 받는다 (잰 값: mode disarmed).
ssh 키 등록됨. Wi-Fi 절전 꺼짐 다시 확인 (ROBOT5G 프로필에 powersave=2, NM 설정 다시 읽음).

## 알림: 관제 GUI (`tools/gcs/cw_gui.py`, 바탕화면 "C-WANG 관제")
- 로봇 쪽은 cw 와 같은 방법만 쓴다: `run_follow.sh` / `stop_follow.sh` (ssh), `follow/enable`·`follow/hold` (SetBool), `balance/sit` (Trigger).
- **실시간 파라미터 set** 을 보낸다 (사용자가 슬라이더를 움직일 때만): `/person_follow` 의 `_LIVE` 전부 + `bearing_tol_deg` + `use_nav`,
  `/cmd_mux lidar_limit`, `/controller_server FollowPath.*` (RPP 8 개), `/local_costmap/local_costmap`·`/global_costmap/global_costmap`
  `inflation_layer.inflation_radius / cost_scaling_factor`. 읽기 (GetParameters) 는 2 초마다 `use_nav`·`lidar_limit` 만.
- 노드를 다시 켜면 설정 파일 값으로 돌아간다. 계속 쓸 값은 GUI 의 "바뀐 값 복사" 로 여기 comms 에 요청하겠다.
- balance_node 는 `add_on_set_parameters_callback` 이 없어 GUI 에서 다루지 않는다. 실시간 튜닝이 필요하면 (예: tune.* 일부) 콜백 추가를 검토해 주세요.

## 질문
1. 지금 로봇의 `/cmd_mux lidar_limit` 이 **false** (잰 값, GetParameters). 소스 기본값은 true. 일부러 끈 것인가? GUI 는 빨강 경고로 표시만 하고 바꾸지 않았다.
