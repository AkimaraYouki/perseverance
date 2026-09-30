from: desktop
re: 2026-09-26_1800_desktop-brake-pi-turn-limit.md
status: info (규격 `sim/isaaclab/scripts/wbctrl.py` 변경 — 실기 이식은 이 파일을 따른다)

# 데스크톱 → 로봇: 달릴 때 회전 상한은 **명령 속도** 기준으로, 기본으로 켬

## 바뀐 것 (`wbctrl.py`)

```python
if turn_limit:
    v_cmd = min(abs(vx_stick), vm)                  # 실제 속도가 아니라 명령(스틱) 속도
    wz_lim = max(0.5, (wheel_margin * w_max * R - v_cmd) / 0.094)
    wz = clip(wz, -wz_lim, wz_lim)
```

- TUNE: `turn_limit=True`, `wheel_margin=0.85` (예전 기본: 끔, 0.7, 실제 속도 기준)
- 3.5 km/h 스틱 끝이면 회전 최대 약 1.6 rad/s, 스틱 절반이면 `wz_max` (2.0) 그대로. 제자리 회전은 그대로.

## 왜

3.5 km/h 로 달리다 스틱 끝까지 틀면 바깥 바퀴가 (v + 0.094 wz) / R = 16.1 rad/s 로 모터 한계 근처 → 토크가 없어
회전도 균형도 잃고 앞으로 고꾸라진다 (창 기록 2026-09-30 12:43, 모터 x0.89: 한계의 96 %).

`pv robust` (로봇마다 다른 질량·무게중심·모터·센서 잡음·지연):

| 설정 | 평지 최고 속도 급회전 (fast_turn) | 자갈길 급회전 (stones_turn) | 전체 11 종 x 8 대 |
|---|---|---|---|
| 끔 (예전) | 14/16, 5/8 | 16/16 | 85/88 |
| 실제 속도 기준, 0.85 | 16/16 | **14/16** (추정 속도가 튀어 상한이 출렁임) | - |
| **명령 속도 기준, 0.85** | **16/16** | **16/16** | **88/88** |

## 같이 바뀐 것

- 창 시뮬(`climb_test.py`, `pv demo` / `pv jump`)도 이제 `wbctrl.WBController` 를 그대로 부른다. 창·`pv robust`·실기 규격이 한 코드.
- `step()` 에 선택 인자 `jump` (조종자 점프 요청, 속도·회전 조건 미달이면 막힘) 와 `h_mid` (다리 높이 가운데, 수동 높이) 추가.
  안 주면 예전과 같다.
