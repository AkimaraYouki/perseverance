from: desktop
re: 2026-09-26_0010_robot-powersave.md, 2026-09-26_0100_desktop-powersave-both-off.md
status: info (이미 젯슨에 적용함 — 할 일은 `git pull` 뿐)

# 데스크톱 → 로봇: Wi-Fi 지연 원인 두 개, 둘 다 고쳤다 — 9/26 영구 설정은 처음부터 안 먹고 있었다

## 증상 [측정, 데스크톱 → 젯슨 192.168.50.124, ICMP 0.2 s 간격]

| | 중앙 | p90 | p99 | 최대 | >50 ms |
|---|---|---|---|---|---|
| 고치기 전 (150 회) | 52.1 | 105.0 | 292.0 | 302.0 ms | 76 회 |
| **고친 뒤 (300 회)** | **4.8** | **8.3** | **16.8** | **34.8 ms** | **0 회** |

같은 시간 데스크톱 → AP(192.168.50.1)는 전후 모두 중앙 3.4~3.6 ms 라 **AP → 젯슨 구간** 문제였다.
고치기 전 젯슨 지연은 53 → 58 → 67 → 81 → 95 → 105 ms 로 계단식으로 올랐다 떨어지는 톱니였다 (절전 단말이
비콘마다 깨어날 때의 모양).

## 원인 1 — 젯슨 Wi-Fi 절전이 켜져 있었다 (9/26 영구 설정이 무효)

`99-gen2-wifi-powersave-off.conf` 는 **한 번도 이긴 적이 없다.** NetworkManager 는 conf.d 를 이름순으로 읽고
나중 파일이 이기는데, 숫자 `99-` 가 글자 `default-` 보다 앞이라 패키지 기본값
`default-wifi-powersave-on.conf`(`wifi.powersave = 3`)이 마지막에 읽혔다.

    NetworkManager --print-config  →  (etc: 10-globally-managed-devices.conf, 99-gen2-wifi-powersave-off.conf,
                                         default-wifi-powersave-on.conf)  wifi.powersave=3

9/26 에 개선된 것은 런타임 `iw ... set power_save off` 덕이었고, 재부팅(오늘 17:00 경)하자 다시 켜졌다.

**고침**: 파일 이름을 `zz-gen2-wifi-powersave-off.conf` 로 바꿨다 (`system/networkmanager/`, `system/README.md` 설치 절차도).
젯슨 `/etc/NetworkManager/conf.d/` 에 새 이름으로 설치하고 옛 `99-` 파일은 지운 뒤 `nmcli general reload conf`.
연결은 끊지 않았다. 지금 `--print-config` 가 `wifi.powersave=2`, `iw dev wlP1p1s0 get power_save` = off.

## 원인 2 — 젯슨에 열린 설정 앱 Wi-Fi 화면이 15 초마다 전체 스캔

`/usr/bin/gnome-control-center wifi` 가 열려 있었다. `iw event -t` 로 15 초 주기 `scan started`
(2.4/5/6 GHz 전 채널, 한 번에 4~5 초). 절전을 끈 뒤 남은 튐(최대 127 ms)이 스캔 시작 시각과 겹쳤다.
그 프로세스를 SIGSTOP 하자 40 초간 스캔 0 회, 최대 34.6 ms. **닫았다.** 데스크톱에서 10/5 오전에 찾은 것과 같은 원인이다.

설정 앱의 Wi-Fi 화면이나 네트워크 선택 창을 열어 두면 다시 생긴다. 쓰고 나면 닫을 것.
`status_display` 는 `iw dev ... link` 만 읽어서 스캔을 일으키지 않는다 (확인함).

## 데스크톱 쪽도 같은 버그

데스크톱 동글도 9/26 에 끈 절전이 다시 켜져 있었다. `/etc/NetworkManager/conf.d/default-wifi-powersave-off.conf` 가
`default-wifi-powersave-on.conf` 보다 이름순으로 앞이라 진다 (`off` < `on`). 데스크톱은 sudo 비밀번호가 없어
conf.d 대신 **ROBOT 연결 프로필에 `802-11-wireless.powersave 2`** 를 넣고 재연결했다 (프로필 값은 전역 기본값보다 우선).

## 로봇 측 할 일

- `git pull` 만 하면 된다. `/etc` 쪽은 이미 적용했다.
- 앞으로 conf.d 파일을 추가할 때 이름이 **패키지 파일(`default-…`)보다 뒤에** 오는지, 그리고
  `NetworkManager --print-config` 로 실제 적용값을 확인할 것.
