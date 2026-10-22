from: gcs
re: -
status: 요청 (작음) + 정보

사용자가 로봇·노트북을 새 와이파이 **핫스팟 "robot" (10.75.26.0/24, 게이트웨이 10.75.26.19)** 에 붙였다 (2026-10-22 23:45).
- 잰 값: 젯슨 wlP1p1s0 = **10.75.26.250**, GCS 노트북 = **10.75.26.210** (둘 다 DHCP). 이 망은 SPDP 멀티캐스트가 통해서 지금도 서로 보인다
  (노트북에서 /balance 파라미터 읽기 OK, 로봇에서 /balance/heartbeat 20 Hz 수신 확인).
- "패드 해제가 안 됨" 의 원인은 노트북 쪽이었다: pad_teleop·joy_node 가 10-15 옛 망에서 켜진 채라 UDP 송신이 전부 실패 → 재시작으로 해결.
  (로봇 balance 로그: 23:38 부팅 뒤 start/sit/disarm 수신 0 건, 계속 disarmed.)

## 요청
1. `src/gen2_bringup/config/dds_peers.txt` 에 `10.75.26.210   # GCS laptop suhorobot on hotspot "robot" (2026-10-22)` 추가
   (멀티캐스트가 막히는 날 대비. DHCP 라 바뀌면 다시 알림). gen2-bench 재시작은 disarmed 일 때.

## 노트북 쪽에서 한 것 (참고)
- `~/.ssh/config`: 노트북이 10.75.26.x 에 있으면 192.168.50.124 로의 ssh 를 10.75.26.250 으로 돌림 (HostKeyAlias .124 — 같은 젯슨 키로 검증).
  cw · wifi -r · slam_reset · 관제 GUI 가 코드 그대로 새 망에서 동작. setup_gcs.sh 도 같은 설정을 만든다.
- 노트북 cyclonedds.xml 피어 = 192.168.50.124 + 10.75.26.250.
- 젯슨 IP 가 바뀌면 (예: .10 계획) 알려 주면 노트북 쪽 두 곳을 고친다.
