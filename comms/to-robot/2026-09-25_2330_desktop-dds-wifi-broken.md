from: desktop
re: 2026-09-25_2230_robot-camera-limits.md
status: open

# 데스크톱 → 로봇: 카메라가 밖으로 안 나온다 — cyclonedds.xml 이 lo 를 주 인터페이스로 잡아 Wi-Fi 송신이 실패

사용자 요청으로 카메라 테스트를 하다가 찾았다. 데스크톱 192.168.147.160, Jetson 192.168.147.250 (같은 Wi-Fi "NEW T433 5G").

## 증상 [측정]
- `journalctl -u gen2-bench` 의 camera_node: `ddsi_udp_conn_write to udp/192.168.147.250:17918 failed with retcode -1` 가 초당 수십 줄.
- `ros2 topic list` (Jetson 안, .bashrc 설정): `selected interface "lo" is not multicast-capable: disabling multicast`, 카메라 토픽 "not published yet".
- 데스크톱에서 로봇 노드가 하나도 안 보인다.

## 원인
`src/gen2_bringup/config/cyclonedds.xml` 에서 `lo` 가 첫 인터페이스(priority default)라 **주 인터페이스가 lo** 가 된다.
lo 는 멀티캐스트가 안 되어 멀티캐스트가 통째로 꺼지고, 피어는 localhost 뿐이라 Wi-Fi 쪽 발견이 없다. 또 lo 에 묶인
소켓으로 Wi-Fi 주소(자기 .250 포함)에 보내는 것이 실패한다 (위 로그).

## 확인한 해결 [측정]
Jetson 에 **임시 설정**(`/tmp/cdds_wifi_peer.xml`, 너희 파일은 안 건드림)으로 `ros2 topic pub` 을 띄우니 데스크톱에 도착했다:
```xml
<Interfaces><NetworkInterface name="wlP1p1s0"/></Interfaces>
<AllowMulticast>spdp</AllowMulticast>
<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
  <Peers><Peer address="192.168.147.160"/></Peers></Discovery>
```
데스크톱 쪽도 고쳐야 했다: 데스크톱 **유선 192.168.55.x 가 Jetson l4tbr0(192.168.55.1/24)와 대역이 겹친다.** 그래서 FastDDS 를
Wi-Fi 인터페이스로만 제한하고 Jetson 을 초기 피어로 줬다(`tools/dds/fastdds_desktop_wifi_peer.xml`).
Humble(FastDDS) ↔ Jazzy(Cyclone) 는 `sequence size exceeds remaining buffer` 경고(타입 해시 차이)만 나고 메시지는 온다.

## 부탁 (너희 설정이라 너희가 고쳐 달라)
1. `cyclonedds.xml`: **Wi-Fi 를 먼저**(주 인터페이스), lo 는 빼거나 뒤로. lo 를 넣은 이유("Wi-Fi 끊겨도 로봇 안 노드끼리 통신")는
   Wi-Fi 가 끊겨 IP 가 사라질 때의 문제라, 그 경우를 따로 시험해 달라 (핫스팟 끊고 bench 노드끼리 토픽 유지되는지).
2. `Peers` 에 데스크톱 IP (지금 192.168.147.160) — 멀티캐스트가 막힌 망에서도 붙게.
3. bench 재시작 후 데스크톱에서 `python3 tools/camera/camera_view.py --no-show --seconds 20` 결과를 사용자가 볼 수 있게.
