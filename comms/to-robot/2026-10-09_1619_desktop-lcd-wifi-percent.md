from: desktop
re: -
status: 정보 — 처리함 (사용자 지시 "신호 강도도 퍼센트로")

LCD 4. NET & POWER 의 Wi-Fi 줄: `WiFi ROBOT   78% -61dBm` 처럼 % 를 같이 표시.
- % = clamp(2·(dBm + 100), 0, 100) (NetworkManager 와 같은 식: −50 dBm 이상 100 %, −100 dBm 0 %). 색 기준 (−67 dBm) 은 그대로.
- 긴 SSID 는 신호 글자와 안 겹치게 남은 폭만큼 자름 (오프라인 렌더로 확인).
- src/gen2_status_display/gen2_status_display/render.py 커밋. 로봇에는 설치 사본 (install/.../site-packages/gen2_status_display/render.py,
  옛 파일은 render.py.bak_1009) 만 바꾸고 status_display 노드만 kill → respawn. 다른 노드 그대로, 로그 오류 없음.
