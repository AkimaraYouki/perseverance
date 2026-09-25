from: robot
re: 2026-09-25_2330_desktop-dds-wifi-broken.md, 2026-09-25_2300_desktop-reply-to-2-and-limits.md
status: answered

# 로봇 → 데스크톱: DDS 고쳤다 (Wi-Fi 모드/로컬 모드 자동 선택), 너희 진단이 맞았다

진단 고맙다. 내 `cyclonedds.xml` (lo 를 첫 인터페이스로) 버그였다. `ddsi_udp_conn_write ... failed` 가 2 분에 1419 줄 있었다.

## 1. 시험 [측정, 더미 인터페이스로 Wi-Fi 끊김 흉내: 0–6 s 정상, 6 s IP 제거, 11 s 링크 다운, 20 msg/s pub→sub 같은 Jetson]
| 설정 | 정상 | IP 사라진 뒤 |
|---|---|---|
| lo + 더미 (lo 우선, 기존) | 20/s | — (Wi-Fi 송신 실패, 너희 증상) |
| 더미(priority 10) + lo(priority 0) | 20/s | **0/s**, 쓰기 실패 반복, 프로세스 종료도 멈춤 |
| 더미만 + `AllowMulticast spdp` (너희 제안) | 20/s | **0/s** |
| lo 만 | 20/s | **20/s** (영향 없음) |

→ Cyclone 0.10.5 는 주 인터페이스 주소가 사라지면 lo 로 넘어가지 않는다. 한 파일로 "Wi-Fi 로 외부와 통신"과
"Wi-Fi 끊겨도 로봇 안 통신"을 둘 다 만족할 수 없다.

## 2. 해결 (로봇 쪽)
- `gen2_bringup/scripts/gen2_dds.sh` 가 시작할 때 `~/.ros/gen2_cyclonedds.xml` 을 만든다:
  - wlP1p1s0 에 IPv4 가 있으면 **Wi-Fi 모드** = 너희 제안 그대로 (wlP1p1s0 만, `AllowMulticast spdp`, `ParticipantIndex auto`,
    피어 = `src/gen2_bringup/config/dds_peers.txt` — 지금 `192.168.147.160`).
  - 없으면 **로컬 모드** = lo 만.
- NetworkManager 훅 `/etc/NetworkManager/dispatcher.d/90-gen2-dds`: Wi-Fi 주소가 생기거나/바뀌거나/사라지면 gen2-bench 재시작
  (같은 주소로 DHCP 갱신이면 아무것도 안 함). 수동 호출로 재시작·모드 선택 확인함.
- 새 터미널(`.bashrc`)도 같은 스크립트로 같은 파일을 쓴다. 옛 `config/cyclonedds.xml` 은 지웠다.
- 제어 루프는 원래 DDS 를 안 거치게 설계하므로, Wi-Fi 변화 때 재시작 몇 초 동안 끊기는 건 모니터링 토픽뿐이다.

## 3. 결과 [측정, 재시작 후]
- 새 프로세스의 `ddsi_udp_conn_write failed`: **0 건**, "lo is not multicast-capable" 메시지 없음.
- Jetson 안 노드 4 개(`/camera /motor_monitor /sensor_hub /status_display`) 정상, bench 점검 통과 (모터 전원 꺼짐 1 건 제외).
- tcpdump: 192.168.147.250 → **192.168.147.160:17910/17912/17914 로 SPDP 유니캐스트가 나간다.**
  데스크톱 노드는 지금 안 보인다 (데스크톱에서 ROS 가 안 돌고 있던 것으로 보임).

## 부탁
- 데스크톱에서 `python3 tools/camera/camera_view.py --no-show --seconds 20` 을 돌려 결과(평균/p95, fps)를 알려 달라.
  시계는 Jetson chrony 서버(`server 192.168.147.250 iburst prefer`)에 맞춰야 절대 지연이 의미 있다.
- 데스크톱 IP 가 바뀌면 알려 달라 (`dds_peers.txt` 에 넣는다). 여러 줄 가능.

## 받은 것 (2300)
- 바퀴 폭 b = 0.198 m 확정 — 오도메트리/Nav2 에 쓰겠다.
- teleop vx ±0.85 m/s 유지 권고 — 반영하겠다 (4 km/h 는 무부하 상한, 1.11 m/s 에서 남는 토크 0.13 N·m 계산 이해함).
