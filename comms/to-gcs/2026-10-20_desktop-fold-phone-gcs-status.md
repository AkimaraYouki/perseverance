from: desktop
status: 정보 — 휴대폰 (갤럭시 폴드) 지상국 개발 현황 (2026-10-20)
출처: 별도 에이전트 (Codex) 작업 보고 `~/Documents/Codex/2026-10-19/…/outputs/fold-ros`, `fold-ros-gpu`, `fold-integration` (데스크톱) + 데스크톱 오프라인 점검

## 완료 (폰에서 실제 확인)
| 항목 | 내용 |
|---|---|
| 기반 | 루팅 없이 Termux + Termux:X11 + Ubuntu 24.04 (proot) + ROS 2 Jazzy + Cyclone DDS. perseverance `6ff86a2` 관제 코드 (gen2_msgs·gen2_tools·gen2_bringup) 폰에서 빌드 |
| 실행 | Termux 에서 `fold-gcs` (관제 GUI), `fold-gcs rviz`, `fold-gcs terminal`, `fold-stop`, `fold-network check / prepare <젯슨IP> / use robot / use offline` |
| RViz | Adreno 840 GPU (VirGL → ANGLE → Vulkan), 10 FPS. 시험 지도·라이다·TF·영상 표시, Global Status OK |
| 패드 | Xbox 패드 블루투스 실제 입력 → `/joy` → 원본 `pad_teleop`. 스틱·LT/RT·A/B/X/Y·LB/RB 매핑 확인, 약 20 Hz |
| 안전 | X11 화면 이탈 → `/joy` 중단 → 약 0.49 s 뒤 heartbeat 중단 (실물 확인). 재개는 중립 (스틱·트리거·버튼 모두 놓음) 확인 뒤 |
| Wi-Fi | `wifi`, `wifi -r` 폰용. ROBOT5G, 폰 192.168.50.117 |
| 데스크톱 점검 | 단위 테스트 4/4 (HMAC·재전송·만료, 형식, 중립 재확인, 버튼 눌림 재개 차단). 입력 손실·포커스 이탈·패드 2 대 이상·0.3 s 넘은 입력이면 발행 중단, 원본 pad_teleop 0.5 s 타임아웃 그대로 (코드 확인) |

## 준비만 됨 (젯슨 꺼짐 → 미검증)
- 로봇 프로필: domain 42, Wi-Fi 유니캐스트 (젯슨 192.168.50.124), `/root/gen2_desk/cyclonedds-wifi.xml`. 실제 토픽 수신 미확인.
- SSH 상태판 (`wifi -r` 로봇 CPU·온도) 과 `cw` 로봇 명령: 폰에서 `parksudo@젯슨` 으로 접속할 SSH 키를 따로 등록해야 함 (PC 키는 복사하지 않음).
- 실기 구동, SLAM 지도 작성, Nav2·사람 추종, HUD: 미검증.

## 남은 일
1. 로봇 재조립 뒤 실기 검증 (순서): `fold-stop` → `fold-network prepare <젯슨IP>` → `fold-network use robot` → `fold-gcs terminal` 에서 `/controller/state`·`/diagnostics`·`/odom`·`/scan_level`·TF·카메라 수신 (패드 끈 채) → SSH 상태판 → RViz/SLAM/HUD 하나씩 → 패드 구동 (사용자 허용 범위) → 블루투스·Wi-Fi 끊김·앱 종료 시 `/joy`·heartbeat 중단.
   수신이 안 되면: 젯슨 IP, ROS_DOMAIN_ID 42, RMW, Cyclone 인터페이스/피어, 공유기 기기 간 차단, UDP 방화벽 순으로. 같은 SSID 라는 것만으로 성공 판단 X. 폰 IP 가 바뀌면 앱 재시작.
2. 손짓 인식용 MediaPipe 환경을 폰에 구성.
3. 블루투스 실물 단절 시험: 끊김 순간 마지막 입력이 최대 0.3 s (20 Hz) 더 나갈 수 있고 pad_teleop 0.5 s 까지 최악 ~0.8 s 뒤 정지 — 실측 확인.
4. (참고) 같은 joy_node 를 띄운 채 X11 앱을 33 번 넘게 재시작하면 입력이 계속 거부됨 (안전 쪽 정지) → joy_node 재시작으로 해결.

## 주의
- Termux:X11 은 서명을 바꾼 개인 시험판 (공개 테스트 키). 공식 업데이트를 깔면 패드·Wi-Fi 중계가 사라짐. 원본 APK·소스·재빌드 스크립트는 `수정소스_및_복구자료.zip`, 폰 백업 `/root/fold-backup-android-bridge`.
- 중계 HMAC 토큰은 출력·공유 금지. 다른 기기엔 새로 생성.
- **관제는 데스크톱·노트북·폰 중 한 곳만** (패드 heartbeat·SLAM 중복 금지).
- 지금 폰은 로봇 없는 시험 프로필 (domain 142, loopback).
