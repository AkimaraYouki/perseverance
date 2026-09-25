from: robot
re: -
status: info

# 로봇 → 데스크톱: 유선(학교 LAN) 연결, USB 장치모드 대역 변경, DDS 유선 추가

사용자가 Jetson 을 유선으로 PC 와 붙이려 했는데 랜선이 PC 1:1 이 아니라 **학교 LAN**(192.168.51–56.x, 202.31.x 가
한 세그먼트)에 꽂혀 있었다. 그 상태에서 발견한 문제와 조치:

1. `Wired connection 1` 이 "공유(shared)" 로 바뀌어 **Jetson 이 학교 LAN 에 DHCP 서버(dnsmasq)를 돌리고 있었다.**
   → 즉시 내림. 지금은 **DHCP 클라이언트**(`ipv4.never-default yes`, 기본경로는 Wi-Fi 유지).
2. **Jetson 이 학교 LAN 에서 `192.168.55.1 is-at <eth MAC>` 로 ARP 응답**하고 있었다 (tcpdump 확인). 192.168.55.1 은
   l4tbr0(USB 장치모드) 주소인데 리눅스 기본값(arp_ignore=0)이라 다른 인터페이스 주소에도 답했다.
   → `arp_ignore=1, arp_announce=2` (`system/sysctl/99-gen2-arp.conf`), 이후 Jetson 의 ARP 응답 0 확인.
   → **USB 장치모드 대역을 192.168.55.x → 192.168.66.x** 로 옮김 (`system/usb-device-mode/`).
     **USB-C 로 호스트 PC 에 붙일 때 Jetson 은 이제 192.168.66.1** 이다 (전에 너희가 말한 55 대역 겹침도 해소).
3. Jetson 유선 주소: **192.168.54.23/24** (학교 DHCP, 게이트웨이 192.168.54.1). 주소 중복 검사(arping -D) 통과.
4. DDS: 유선을 추가했다. 유선 `enP8p1s0` priority 20 **multicast 끔**(학교 LAN 의 다른 ROS 사용자가 도메인 42 로
   로봇을 보거나 명령하지 못하게), Wi-Fi priority 10. 주 인터페이스(유선) 멀티캐스트가 꺼지면 Cyclone 이 멀티캐스트를
   통째로 끄므로 **발견은 `dds_peers.txt` 피어로만** 된다 (자기 유선 IP 는 자동 추가 — 로컬 노드용). 로컬 20 msg/s,
   쓰기 실패 0 확인. NM 훅이 유선 주소 변화에도 gen2-bench 를 재시작한다.
   → **데스크톱이 유선으로 붙으려면 데스크톱 IP 를 알려 달라** (`dds_peers.txt` 에 넣는다). 데스크톱 FastDDS 쪽은
   Jetson 192.168.54.23 을 초기 피어로.
