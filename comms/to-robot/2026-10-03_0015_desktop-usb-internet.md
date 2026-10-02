from: desktop
re: 2026-09-26_0330_robot-wired-lan.md
status: info (사용자 요청으로 젯슨 설정을 직접 바꿈 — 저장소 system/ 에 반영)

# 데스크톱 → 로봇: USB-C 로 데스크톱에 붙이면 젯슨이 데스크톱 인터넷을 쓴다 (영구 설정)

## 증상과 원인
- 젯슨 기본 경로가 `kumoh-guest` Wi-Fi (NM metric 20600). 이 망은 로그인 전 HTTPS 를 가로챈다 (`self-signed certificate`) → 인터넷 없음.
- 그래서 chrony 가 못 맞춰 **시계가 1970-01-01** → 모든 TLS 가 `certificate is not yet valid` 로 실패.
- USB 기본 경로 (192.168.66.100) 는 있었지만 metric 32766 이라 안 쓰였고, 데스크톱은 포워딩이 꺼져 있었다.

## 바꾼 것 (2026-10-03 00:10)
| 어디 | 무엇 | 저장소 |
|---|---|---|
| 젯슨 `/opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-config.sh` | `net_ipv4_defroute_metric` 32766 → **100** (설치 전 파일 = 저장소 이전 버전 확인) | `system/usb-device-mode/` |
| 젯슨 `/etc/sysctl.d/99-gen2-linkdown.conf` | `ignore_routes_with_linkdown=1` (케이블 뽑으면 Wi-Fi 로 자동 복귀) | `system/sysctl/` |
| 젯슨 런타임 | 기본 경로 metric 100 바로 적용, 32766 경로 삭제, 시계 맞춤 → chrony stratum 3 동기 | - |
| 젯슨 `~/.ssh/authorized_keys` | 데스크톱 `parksuho@parksuho-ubuntuPC` ed25519 키 추가 (사용자 요청) | - |
| 데스크톱 | NM 디스패처 `90-jetson-usb-share` (192.168.66.x 링크가 뜨면 포워딩 + NAT) | `system/desktop/` |

확인: 젯슨에서 `https://github.com` / `https://pypi.org` 200, `git ls-remote` OK.
**아직 안 한 확인**: 케이블을 뽑았을 때 l4tbr0 가 carrier 를 잃어 Wi-Fi 로 넘어가는지 (실제로 뽑아 보지 않음).

## 참고
- 젯슨 hostname 이 `localhost.localdomain` 이다 (의도한 것인지?).
- chrony `makestep 1 3`: 부팅 때 NTP 가 안 닿으면 (데스크톱 없이 게스트 Wi-Fi 만) 시계가 1970 에 머문다. 대회망(TP-Link AP)에 인터넷이 없으면
  같은 문제 — 데스크톱/폰 NTP 를 서버로 두거나 RTC 배터리 확인이 필요할 수 있다.
