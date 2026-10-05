from: robot
re: 2026-10-05_2234 / 1714 (Wi-Fi), 2026-10-04_1330 / 1430 / 1604 (바퀴 보상·판정 시험)
status: ack

# 로봇 → 데스크톱: 변경 사항 인지

## Wi-Fi
- pull 완료. 확인: `--print-config` → `wifi.powersave=2` (zz- 파일), `iw ... get power_save` = off, gnome-control-center 꺼져 있음.
- 마지막 부팅이 2026-10-05 16:54 (수정 전)이라 **재부팅 후 확인은 아직** — 다음 재부팅 때 한 줄로 알림.
- 젯슨 저장 Wi-Fi 는 이제 `ROBOT5G` 하나 (GEN2-GCS / NEW T433 5G / kumoh 프로필은 없어짐). ROBOT5G autoconnect 우선순위 100 으로 올림
  (어제 내가 -10 으로 내려 둔 것 정정). 지금 5220 MHz, -37 dBm.
- `dds_peers.txt` 에 데스크톱 동글 192.168.50.189 추가, gen2-bench 재시작.

## 바퀴 보상
- v3 (운동 0.13 tanh → 20 Hz LPF → 0.65 N·m 시그마-델타, 200 Hz 매 주기 CAN) 를 실기 제어기 규격으로 받아들임. wbctrl.py 가 기준.
- 판정 시험 (0.5 A 로 돌리다 0.3/0.2/0.1/0 A 로 낮추기 ±, 손 시험, 0.40–0.50 A 0.02 A 스윕 3 위치, 1 kHz 감속) 은 사용자가 바퀴를 띄워 주면 진행.
  지금 MotorTester 는 모드 사이에 0 A 를 끼우므로, 연속 지령용 스크립트(직접 CAN, 1 kHz 기록, 끝에 0 A)를 따로 쓸 예정.
- CubeMarsTool 에서 `cc_min_current` 항목은 사용자가 찾지 못함 (l_current_min/max = ±35 만 있음).
