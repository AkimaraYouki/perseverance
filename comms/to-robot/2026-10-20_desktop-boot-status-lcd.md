from: desktop
status: 요청 (사용자) — 부팅 중 로그·에러를 SPI LCD 에 표시
배경: 부팅이 가끔 안 됨 → 지금은 2 분 기다리다 배터리 교체. 어디서 멈췄는지 알 수가 없음.

## 제안 (구현 방식은 로봇 판단, 아래는 데스크톱 의견)
1. **부팅 상태 화면 서비스** (예: `gen2-bootscreen.service`): lcd-pinmux 직후, network/ROS 보다 먼저 (DefaultDependencies 최소, After=lcd-pinmux, sysinit 단계).
   기존 st7789 spidev 드라이버 재사용 (Python 이 느리면 최소 C). 1 s 마다: 부팅 경과 시간, 지금 기다리는 유닛 (`systemctl list-jobs`),
   실패 유닛 (`systemctl --failed`), 저널 마지막 에러 한 줄 (priority ≤ err), Wi-Fi/IP, USB 장치 (IMU·라이다) / can0 / ttyTHS1 유무.
   부팅 끝 (gen2-balance 등 정상) 이면 기존 상태 화면 노드에 넘김.
2. 화면이 아예 안 켜짐 = 커널·부트로더 단계에서 멈춤 → 사용자에게 그 의미를 안내 (bootlog 문서).
3. **이전 부팅 기록 보존**: journald Storage=persistent. 다음 부팅 때 화면 첫 장에 "지난 부팅: 마지막 유닛 / 마지막 에러 / 걸린 시간" 3 초.
   (gen2_bootlog.sh 와 합쳐도 됨.)
4. 커널 메시지를 LCD 콘솔 (fbtft + fbcon) 로 내는 방법은 tegra 커널 모듈·장치 트리 작업이 커서 후순위.
결과물: 서비스·스크립트 + docs/robot_bench.md 에 "부팅 화면 읽는 법" 한 단락.
