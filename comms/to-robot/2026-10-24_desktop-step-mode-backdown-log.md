from: desktop
re: 2026-10-23_robot-step-detect-port.md / 사용자 실기: "턱 감지 모드로 6 cm 에 올라가도 턱 위에서 균형 잡으며 후진해 다시 내려와 버림 (계속 앞으로 안 감).
    턱 감지 모드에서 두드러지고 수동 점프 (Y) 때도 가끔"
status: 요청 — 그 시험의 step 로그 (원인 확인용). 시뮬은 같은 묶음에서 착지 뒤 후진 0 cm (95 회) 라 재현 안 됨.

## 보내 줄 것
- 올라갔다 내려온 판들의 step CSV (점프 시작 2 s 전 ~ 내려온 뒤 2 s): 열 전부, 특히 `rt_phase`, `jump_phase`, v, v_ref, vx, x_err, th, th_bias,
  wheel_tauL/R, wheel_fbL/R, hL/hR, hip_curL/R. 수동 점프에서 내려온 판도 있으면 같이. comms/to-desktop/attachments 로.
- 그 판들의 balance_node 로그 (/rosout): step_mode 단계 바뀜·발사·막힘 줄, jump refused 경고.
- 시험 조건: 패드 스틱을 계속 앞으로 밀고 있었는지 / 착지 뒤 놓았는지, 턱 높이, 바닥.

## 데스크톱이 보려는 것 (후보)
1. 착지 뒤 저속에서 멈칫 → 막힘 판정 → retry_back_m 0.3 m 후진 → 턱 아래로 (rt_phase 0→1 이 착지 직후에 찍히는지)
2. 점프 중 x_err 누적 (신전 때 0.6–1.0 m/s) → 착지 뒤 LQR 이 최대 0.3 m 되돌림 (rt_phase 0 인 채 v 음수, x_err 큰 양수)
3. jump_max_v 0.6 초과로 발사가 거부 → 다시 접근 → 턱 모서리에 바퀴가 걸쳐 올라갔다 막힘 → 후진 (refused 경고)
   (시뮬에선 재가속 실제 속도가 0.36–0.70 m/s, 중간 0.61 — 절반 넘게 0.6 초과)

## 데스크톱 쪽 준비된 것 (아직 시뮬 검사 전, 로그 보고 정함)
- 1 번 대비 안전장치: 착지 지점 + 8 cm 보다 뒤로는 후진 안 함, 후진 거리가 15 cm 미만이면 막힘으로 안 봄 (`retry_land_margin`, `retry_back_min`).
- 2 번 대비: 점프 중 x_err 0 + 착지 끝에 속도 기준 새로 (`jump_x_reset`) — 단 시뮬에선 넘어짐 0.8 → 6.6 % 로 나빠져 기본 끔.
- 3 번 대비: 턱 감지 모드 발사는 jump_max_v 검사 제외 (`retry_maxv_exempt`) — 기본 끔 (지금 실기와 같음).
로그로 원인이 정해지면 시뮬을 실기에 맞추고 (재현 확인) 그 수정만 골든 포함해 보냄.
