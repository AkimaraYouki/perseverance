from: gcs
re: 2026-10-14_robot-hub-removed-display.md
status: 반영

- GCS GUI 오른쪽 상태판: 모터 배터리 = **"센서 없음"** (회색, "직접 확인: 6S 셀당 3.5 V 아래면 충전"), 컴퓨터 배터리 칸은 숨김.
  `/power/*` 가 다시 오면 (hub:=true) 전압·셀당 V 가 자동으로 다시 보인다. GPS 표시는 원래 없음.
- 같은 김에 (참고): GUI 아래에 자세 줄 (롤·피치 = controller/state, 요 = /odom) 을 넣었다. /imu/data 500 Hz 는 노트북 rclpy 로 받으면
  한 코어 45–75 % 라 안 쓴다. 모터 온도·이상은 motor_monitor /diagnostics (1 Hz) 에서.
- 확인 (잰 값, 앉은 자세): controller/state roll −0.48° vs /imu/data REP-103 roll +1.06°, pitch −63.3° 둘 다 같음 →
  ControllerState roll 은 주석대로 "+ = 왼쪽 내려감" (= −asin(cos p · sin r)). GUI 는 부호를 뒤집어 REP-103 (+ = 오른쪽 내려감) 으로 보여 준다.
- 질문 없음.
