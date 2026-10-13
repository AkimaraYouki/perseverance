# C-WANG GCS 노트북 (suhorobot, Ubuntu 24.04, 192.168.50.109)

데스크톱과 같은 관제 환경을 노트북에 — 패드 조종, 손짓, RViz (지도·코스트맵), 상태판, HUD, SLAM.
**경기 중 판단·주행은 전부 젯슨** (Wi-Fi 가 끊겨도 로봇은 안전하게 멈춤). 노트북은 조종·감시·손짓·지도만.
노트북은 Ubuntu 24.04 → 로봇과 같은 **ROS 2 Jazzy** (데스크톱 Humble 과 달리 메시지 버전 차이 없음).

## 1. 설치 (한 번, 일반 Wi-Fi 에서 — 데이터 약 1.5–2 GB)
```bash
bash ~/setup_gcs.sh          # sudo 비밀번호를 물음. 10–20 분
```
하는 일: 기본 도구 → ROS 2 Jazzy 저장소 → ROS 패키지 (ros-base, rviz2, cyclonedds, joy, 이미지 압축, slam_toolbox, colcon)
→ perseverance 받기·빌드 (gen2_msgs, gen2_tools, gen2_bringup) → Cyclone DDS 설정 (도메인 42, 로봇 peer)
→ 관제 도구 (cw, wifi, 상태판, HUD, RViz 설정, 관제 GUI) → 손짓용 MediaPipe 가상환경 + 모델.
다시 실행해도 안전 (끝난 단계는 넘어감).

## 2. ROBOT 망에서 한 번
```bash
ssh-keygen -t ed25519            # Enter 세 번
ssh-copy-id parksudo@192.168.50.124   # 로봇 비밀번호
```
→ 새 터미널 (PATH 에 ~/bin) → `cw check`.

## 3. 매번 (데스크톱과 같은 명령)
```bash
cw up        # 로봇 확인 → 사람 추종 (로봇) → 패드 → 손짓 → RViz → 상태판 → HUD
cw check     # 상태
cw down      # 노트북 쪽 끄기
```
하나씩: `cw slam start|stop|reset|save`, `cw follow start|stop`, `cw gesture`, `cw pad`, `cw rviz`, `cw dash`, `cw hud`, `cw ssh`,
`wifi` (신호 %), `wifi -r` (로봇 상태판). 패드 버튼은 바탕화면 브링업 안내 (`tools/desktop/bringup.txt`) 와 같음.

## 관제 GUI — 바탕화면 아이콘 **C-WANG 관제** (또는 `cw-gui`)
터미널 없이 창 하나에서 켜고 끄고 튜닝. `cw` 와 같은 프로세스를 다루므로 섞어 써도 된다.
- **기능 켜기/끄기**: 로봇 — 사람 추종 스택, 추종 활성화 (= 패드 X), 일시 정지 (= 손바닥), 장애물 우회 (Nav2, `use_nav`),
  장애물 앞 정지·감속 (`cmd_mux lidar_limit`). 노트북 — 패드, 손짓, SLAM (지우기·저장), RViz, HUD, 상태판. 위쪽 **추종 정지 / 로봇 앉기** 버튼.
  노트북 쪽은 뒤에서 돌고 로그는 `~/gen2_desk/logs/*.log` ('로그' 탭).
- **파라미터 튜닝** (게인은 없음 — 최적화된 값): 주행 속도 한계 (패드·추종·Nav2 우회·균형 노드 상한), 사람 추종, 사람 인식,
  장애물 회피, SLAM, 패드·손짓. 칸마다 적용 시점 표시:
  **실시간** = 슬라이더를 놓으면 0.3 초 뒤 반영 · **재시작 때** = 노트북에서 다시 켤 때 (패드·손짓·SLAM — SLAM 은 지도가 지워짐) ·
  **잠김** = 노드가 실시간 변경을 아직 안 받음 (노드가 `live_params` 를 내놓으면 자동으로 실시간이 된다).
  처음엔 지금 값을 읽어 온다. 주황 = 아직 반영 안 됨. 로봇 노드를 다시 켜면 설정 파일 값으로 돌아가니, 계속 쓸 값은
  **바뀐 값 복사** → `comms/to-robot/`. 프리셋 = `~/gen2_desk/gui_presets/`. SLAM 은 `slam_gui.launch.py` + `~/gen2_desk/slam_gui.yaml`.
- 다른 컴퓨터 (데스크톱) 에서 패드·SLAM 이 돌고 있으면 켜지 않는다.
- **오른쪽 상태판** (큰 글씨): 모드 · 모터 배터리 6S · 속도 · 가속도 (바퀴 축 속도 변화) · 기울기 · 다리 높이 · 모터 최고 온도와 이상.
  배터리는 2026-10-14 센서 허브를 빼서 **'센서 없음'** (직접 확인, 6S 셀당 3.5 V) — 허브를 다시 달아 `/power/*` 가 오면 전압이 보인다
  (셀당 3.50 V 아래 주황, 3.30 V 아래 빨강). 모터는 motor_monitor `/diagnostics` (1 Hz). 값이 안 들어오면 '—'.
- **아래 자세 줄** (늘 보임, 탭과의 경계를 끌어 크기 조절): 자세계 (인공 수평선) · 위에서 (요) · 옆에서 (피치) · 뒤에서 (롤) · 10 초 기록.
  **크게 보기** = 따로 뜨는 큰 창. 부호: 피치 + = 앞 숙임, 롤 + = 오른쪽 내려감, 요 + = 왼쪽으로 돎. 요 기준은 임의라 **요 0 맞추기**.
  롤·피치 = `/controller/state` (중력 방향), 요 = `/odom` (balance_node 가 서 있을 때만 적분). `/imu/data` 500 Hz 는 파이썬으로 받으면
  한 코어의 45–75 % 라 쓰지 않는다 (같은 이유로 `/motors/state` 100 Hz 대신 진단 1 Hz). GUI 전체 ≈ 한 코어의 40 % (잰 값, 2026-10-14).

## 주의
- 노트북은 RAM 3 GB·4 코어·내장 그래픽: **데스크톱과 노트북에서 같은 것을 동시에 켜지 말 것** (패드 2 개 = heartbeat 2 개 → 끊김 안전장치가 무력해짐,
  SLAM 2 개 = 지도·TF 충돌). 한 대만 관제.
- 손짓 (MediaPipe) 은 노트북 CPU 를 많이 쓴다 — 느리면 `cw gesture` 대신 로봇 쪽에서 (`GESTURE_ON_JETSON=1`, docs/roles.md).
- 로봇이 노트북을 찾으려면 로봇 dds_peers 에 192.168.50.109 가 있어야 한다 (데스크톱 에이전트가 요청함). IP 가 바뀌면 다시.
- 같은 도구의 원본은 `tools/desktop/` — 고치면 `git pull` 후 `bash ~/setup_gcs.sh` 다시 (6 단계가 복사).
