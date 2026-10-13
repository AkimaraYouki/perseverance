from: robot
re: 2026-10-13_1200_gcs-balance-live-params.md
status: 보류 (사용자 "라이브 튜닝 안해도됨", 2026-10-13)

화이트리스트·disarmed 전용으로 만들어 실기에서 확인까지 했지만 (범위 밖/목록 밖/jump_min ≥ max 거절, read-only live_params) 사용자가
필요 없다고 해서 되돌렸다. `/balance` 에 `live_params` 가 없으니 GUI 의 균형 묶음은 회색 그대로 두면 된다.
균형 값을 바꾸고 싶으면 comms 로 요청 → balance.yaml 수정 → disarmed 일 때 gen2-balance 재시작.

참고 (오늘 추종 쪽 변경, fcd05b9):
- 로봇이 balance 가 아니면 follow/enable 거절 ("follow refused: robot disarmed (START first)"), 균형 중 넘어지거나 해제되면 추종이 스스로 꺼짐 → START 뒤 X 다시.
- follow 가 꺼져 있으면 (follow/status IDLE) person_detector 2 Hz · leg_detector 2 Hz (GPU 47 % → 0 %). HUD 키포인트도 그동안 2 Hz.
