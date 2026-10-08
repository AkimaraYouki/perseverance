# 점프 상태기계 — wbctrl.py 규격 (C++ 이식용, 2026-10-08)

출처: `sim/isaaclab/scripts/wbctrl.py` `WBController.step()` (`# 점프 상태머신`, `# 점프 단계의 다리 목표와 공중 바퀴 pitch PD`).
값은 `climb_test.py` TUNE 기본. 시뮬 검증: 8 cm 턱 점프, robust `jump` 시나리오 (실측 모델에서 15/16).

## 단계와 전이 (매 스텝 시작 때 단계로 판정, 한 스텝에 한 번만 전이)
| 단계 | 들어가는 조건 | 나가는 조건 → 다음 |
|---|---|---|
| DRIVE | — | 점프 요청 (패드 Y) 이고 **v ≥ jump_min_v (0.2 m/s), \|gz\| ≤ jump_max_wz (1.0 rad/s), 들림 아님** → RETRACT (아니면 `jump_blocked` 이유 기록, 무시) |
| RETRACT (웅크림) | 요청 | t ≥ t_retract (0.25 s) → EXTRACT |
| EXTRACT (신전 = 이륙) | | 두 다리 h ≥ H_MAX − 8 mm **또는** t ≥ 0.25 s → FLY (이 순간 = 이륙 시각 t_takeoff) |
| FLY (접기) | | t ≥ t_tuck (0.12 s) → DESCEND, `soft` 켬 |
| DESCEND (착지 준비) | | (t > 0.04 s 이고 \|고관절 토크\| 최대 > contact_tau 2.0 N·m) **또는** t − t_takeoff ≥ t_fly_max (0.45 s) → LAND |
| LAND (착지) | | t ≥ land_s (0.30 s) → DRIVE, `soft` 끔 |
t = 그 단계에 들어온 뒤 시간. 자동 발동 (모서리 x 앞 trigger 0.30 m) 은 시험용이라 이식 안 해도 된다.

## 단계별 출력
| 단계 | 다리 목표 h (관절값, m) | 다리 kp / kd | 자중 FF | 바퀴 |
|---|---|---|---|---|
| DRIVE | VMC (idle_h ± 롤) | vmc 60 / 1.0 | 0.5·m·g | LQR + yaw |
| RETRACT | h0 → H_MIN (0.1225) 을 0.75·t_retract 동안 선형, 그 뒤 H_MIN 유지 (h0 = 발동 때 두 다리 평균) | leg 60 / 1.5 | 0 | LQR + yaw, **θ_ref = retract_lean 3°** (앞으로 숙이며 웅크림) |
| EXTRACT | H_MAX (0.2425) | 60 / 1.5 | 0 | **pitch PD** (아래) — pd_from = extract |
| FLY | H_MIN | 60 / 1.5 | 0 | pitch PD |
| DESCEND | h_land (0.1825) | soft: land 20 / 1.0 | 0 | pitch PD (목표 land_pitch) |
| LAND | h_land | soft: land 20 / 1.0 | 0.5·m·g | LQR + yaw |
롤 PI 는 RETRACT·EXTRACT·FLY·DESCEND 동안 정지 (적분 상태 = 현재 hL − hR 로 리셋).

## 공중 바퀴 pitch PD (EXTRACT·FLY·DESCEND)
```
ref = extract_pitch (0°)  if EXTRACT
      land_pitch (0°)     if DESCEND
      air_pitch (−5°)     if FLY            # 음수 = 뒤로 젖힘 → 바퀴가 몸 아래로 앞으로
u   = air_kp·(pitch − ref) + air_kd·gy      # air_kp 30 N·m/rad, air_kd 5 N·m·s/rad, 바퀴 하나당
τ_L = τ_R = clip(u, ±air_tau 7 N·m)        # 두 바퀴 같은 값 (+y 규약 = 앞으로 굴림), yaw 분배 없음
```
바퀴를 반작용 휠로 써서 몸통 pitch 를 잡는다 (공중에서 바퀴 토크 0 이면 뒤로 −190°/s 로 넘어감).
MIT 바퀴는 데드밴드가 없어서 그대로 된다 (서보 0.5 A 데드밴드면 작은 u 가 버려짐 — 2026-10-04 비교).

## 이식 시 주의
- 이륙 직전 바퀴 속도: RETRACT 를 0.25 s 보다 빨리 접으면 다리 링크 회전이 바퀴 관절 속도에 더해져 바퀴 한계 (18.85 rad/s) 에 붙는다.
- 착지 판정 contact_tau 2.0 N·m 는 시뮬 고관절 토크. 실기는 MIT 응답 토크 (Kt 확정 전 0.81 잠정) 로 — 실측 착지 충격으로 다시 맞출 것.
- t_fly_max 0.45 s 안전장치 필수 (접지 판정 실패 시 강제 착지 게인).
- `jump_blocked` 는 화면용. 점프 기록 (이륙·착지 시각, 착지 pitch) 을 남기면 시뮬과 비교하기 좋다.
- 첫 실기 시험은 평지에서 제자리 근처 (v 0.2–0.3 m/s) — 8 cm 턱은 그 뒤.
