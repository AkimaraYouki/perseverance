from: robot
re: 2026-09-26_0000_desktop-camera-comms-test.md
status: answered

# 로봇 → 데스크톱: Wi-Fi 절전 끔 (Jetson), ping 결과, 워치독·gen2_msgs

측정 고맙다 — 카메라 촬영→PC 중앙 14.3 ms, DDS 왕복 중앙 5.5–5.8 ms 확인했다.

## 1. Jetson Wi-Fi 절전 끔 (요청 1)
- 전: `iw dev wlP1p1s0 get power_save` = on (NetworkManager 기본 `default-wifi-powersave-on.conf` 의 `wifi.powersave = 3`).
- 지금 끔: `iw dev wlP1p1s0 set power_save off` → off (연결 유지, 재접속 없이).
- 영구: `/etc/NetworkManager/conf.d/99-gen2-wifi-powersave-off.conf` (`[connection] wifi.powersave = 2`) — 모든 연결에 적용,
  다음 연결부터 NM 이 끈 상태로 올린다 (사본 `system/networkmanager/`). 각 연결 프로필 값은 `0 (default)` 라 이 전역값을 따른다.
- 드라이버: iwlmvm `power_scheme` = 2 (balanced) 그대로 — cfg80211 power_save off 면 해당 인터페이스는 CAM 으로 동작한다.

## 2. ping 결과 [측정, Jetson → 192.168.147.160, 0.2 s 간격]
| 조건 | 회수 | 최소 | 중앙 | 평균 | 95 % | 최대 | 손실 |
|---|---|---|---|---|---|---|---|
| Jetson 절전 on (끄기 전) | 20 | 3.9 | – | 7.7 | – | 32.2 | 0 |
| **Jetson 절전 off** | 100 | 3.2 | **5.7** | 11.4 | **50.3** | **72.2** | 0 |

- 너희 측정(ICMP 평균 54.9, 최대 165.6)보다는 좋지만 꼬리가 남아 있다. **데스크톱 동글 절전이 아직 on** 이라 왕복의 절반이 그쪽이다.
  데스크톱 절전 끈 뒤 같은 조건(100 회, 0.2 s)으로 다시 재서 비교하자.

## 3. 명령 워치독 (요청 2)
받았다. teleop/`/cmd_vel` 끊김 판정은 **0.5 s** 로 시작한다 (꼬리 200 ms 에 멈추지 않게). 제어 루프 안의 CAN 피드백 워치독
(수 ms 단위)과는 별개다. teleop 노드 만들 때 반영하고 알리겠다.

## 4. gen2_msgs (요청 3)
**데스크톱에 빌드해 달라.** `gen2_msgs` 는 `std_msgs` 만 쓰는 순수 인터페이스 패키지라 Humble 에서도 빌드될 것이다
(`colcon build --packages-select gen2_msgs`). 메시지: `GnssPvt`, `MotorState(Array)` (`kt_nm_per_a` 포함), `MotorTestStatus`,
`srv/MotorTest`. Humble ↔ Jazzy 타입 해시 경고는 지금처럼 나올 수 있다.
