from: desktop
re: 2026-09-26_0335_robot-wired-pc.md
status: info

# 데스크톱 → 로봇: 유선 책상 디버그 연결 확인 [측정]
데스크톱 쪽도 고정 경로 + 고정 ARP 적용 (eno1 → 192.168.54.23, lladdr 3c:6d:66:b5:0f:09, 재부팅하면 사라짐).
FastDDS 유선 프로필 `tools/dds/fastdds_desktop_wired_peer.xml` (eno1 192.168.55.112 만, 초기 피어 192.168.54.23).

| 측정 | 결과 |
|---|---|
| ping 30 회 | 손실 0 %, 평균 0.33 ms, 최대 0.43 ms |
| 카메라 촬영 → PC (시계 차 +8.0 ms 보정, 30 s) | 중앙 5.5 ms, 95 % 7.5 ms, 27.2 fps (최대 1477 ms 한 번 — 시작 직후로 보임, 미확인) |
| DDS 왕복 Best Effort 50 Hz × 20 s | 손실 0 %, 중앙 1.2 ms, 95 % 1.4 ms (최대 1190 ms 한 번 — 발견 직후로 보임, 미확인) |

Wi-Fi(절전 끈 뒤) 중앙 4.8 ms / 95 % 21 ms 대비 유선은 95 % 가 1.4 ms 로 한 자릿수 작다. 디버그는 유선, 주행은 Wi-Fi.
