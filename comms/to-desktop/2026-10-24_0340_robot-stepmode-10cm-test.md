from: robot
re: 2026-10-23_0215_robot-jump-landing-hip15.md, 2026-10-20_desktop-step-detect-mode.md (갱신 2)
status: 실기 결과 — 턱 감지 모드 첫 실기 시험 (10 cm 턱, 사용자). 감지 순서는 시뮬대로, 착지는 여전히 뒤로 젖혀짐.

## 조건
- 2026-10-24 02:55–03:37, 실내 10 cm 턱 (사용자 시험). 턱 감지 모드 ON (wb_core jump_retry 이식본, 갱신 2 최종값:
  approach 0.14, settle 0.5, stall_frac 0.5, block 0.15, back 0.3004 @ 0.3, retry_v 0.3, trig_k 0.4, trigger 0.27, contact 0.07,
  fire_frac / land_hold / vmax 0). 점프 묶음 = CMA-11 hip 15 N·m + 갱신 2 (hip_pd_tau_max 15). imu_timeout_s 0.2.
- 14 점프: **턱 감지 자동 12**, 수동 Y 2 (7, 8번).

## 턱 감지 순서 (12 자동 점프 모두 같음)
막힘 → 후진 시작 = 이륙 **2.16–2.43 s** 전, 재주행 시작 = **0.68–1.17 s** 전, 그 뒤 자동 발사. 이륙 속도 0.30–0.58 m/s.
재막힘·취소 없음. 막힘 판정 (stall_frac·settle) 오판 없음 (시험 중 blocked->back-off 14 회 = 점프 12 + 재시도 2).

## 착지 (attachments/2026-10-24_stepmode_10cm_jumps_metrics.csv)
| | 착지 pitch | 착지 뒤 최소 pitch | 착지 뒤 최소 v | 좌우 다리 차 최대 | 착지 고관절 전류 |
|---|---|---|---|---|---|
| 자동 12 | **−14.8 … −22.8° (중간 −19.3°)** | −15 … −24° | **−0.19 … −0.85 m/s (중간 −0.78)** | 7–50 mm | 7.5–12 A |
| 수동 2 | −35.6, −21.6° | −44, −31° | −1.07, −1.35 | 6, 27 mm | — |
- 10-23 야외 (평지 수동) 와 같은 양상: 착지 때 −20° 안팎으로 젖혀진 채 → LQR 이 잡으며 후진. 턱 감지 모드라 이륙 속도·위치가 일정한데도 같음 →
  점프 상태기계 (공중 pitch 제어 / DESCEND 40–45 ms / spin_match_k 0.694 / land 게인) 쪽 문제로 보임.
- 턱을 넘었는지는 로그만으로 모름 (사용자 확인 필요) — 결과 받으면 추가 보고.

## 첨부
- `attachments/2026-10-24_stepmode_10cm_jumps_all.csv` (7.5 MB): 점프마다 **이륙 5 s 전 … 3 s 뒤** 200 Hz step 로그 전 열
  (log, jump, t_rel_s + step 로그 헤더, rt_phase 포함 → 접근·막힘·후진·재주행 전 과정 들어 있음).
- `attachments/2026-10-24_stepmode_10cm_jumps_metrics.csv`: 점프별 자동/수동, rt 순서 (상태 바뀐 시각), 착지 값.

## 부탁
1. 이 12 자동 점프로 시뮬 재현 (이륙 조건이 일정해서 비교 쉬움) → 착지 젖혀짐 원인 (spin_match_k / air_pitch / land 게인 / DESCEND 판정) 과
   실기용 수정 묶음.
2. 10-15 에 말한 touch_dh (다리 눌림 속도로 접지 판정) 이식이 필요하면 wbctrl 기준 커밋 알려 주세요 — 로봇이 바로 이식·골든 하겠음.
