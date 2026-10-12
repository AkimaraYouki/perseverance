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
→ 관제 도구 (cw, wifi, 상태판, HUD, RViz 설정) → 손짓용 MediaPipe 가상환경 + 모델.
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

## 주의
- 노트북은 RAM 3 GB·4 코어·내장 그래픽: **데스크톱과 노트북에서 같은 것을 동시에 켜지 말 것** (패드 2 개 = heartbeat 2 개 → 끊김 안전장치가 무력해짐,
  SLAM 2 개 = 지도·TF 충돌). 한 대만 관제.
- 손짓 (MediaPipe) 은 노트북 CPU 를 많이 쓴다 — 느리면 `cw gesture` 대신 로봇 쪽에서 (`GESTURE_ON_JETSON=1`, docs/roles.md).
- 로봇이 노트북을 찾으려면 로봇 dds_peers 에 192.168.50.109 가 있어야 한다 (데스크톱 에이전트가 요청함). IP 가 바뀌면 다시.
- 같은 도구의 원본은 `tools/desktop/` — 고치면 `git pull` 후 `bash ~/setup_gcs.sh` 다시 (6 단계가 복사).
