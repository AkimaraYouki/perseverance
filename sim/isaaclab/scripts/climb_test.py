"""대회 장애물 넘기 시험 — Ascento (arXiv 2005.11435, IV-B) 식 점프 상태머신. 점프는 강화학습이 아니다.

균형·주행은 학습된 균형 정책(관측 25, r3~r5)이 한다. 점프 구간만 상태머신이 다리(와 공중 바퀴)를 덮어쓴다:
  retract  다리 접기(웅크림) — 궤적(램프)으로. 한 번에 접으면 바퀴가 들린다. 균형은 정책이 계속 (Ascento: 균형 제어 켜 둔 채 접는다)
  extract  양다리 최대 신장(이륙). 다 펴지면 fly
  fly      지면 균형 제어 끔. 다리는 원래 게인으로 최대 접기 (바퀴를 모서리 위로) t_tuck 동안.
           바퀴는 반작용 휠로 몸통 pitch 만 잡는다 (Ascento 에 없는 우리 추가: 이륙 때 생긴 pitch 회전을
           안 잡으면 공중에서 -190 deg/s 로 뒤로 넘어간다, 2026-09-26 시험). air_ctrl=False 면 바퀴 토크 0
  descend  바퀴는 fly 와 같음. 다리는 부드러운 스프링-댐퍼(land_kp/kd)로 착지 길이까지 내려 충격 행정을 확보.
           고관절 토크가 contact_tau 를 넘으면 접지 (Ascento 와 같은 감지 — 실기에서도 전류로 된다)
  land     정책이 바퀴 재개, 다리는 부드러운 게인 그대로 land_s 동안, 그 뒤 원래 게인
발동: 바퀴 중심이 다음 모서리 앞 trigger 에 오면 (조종자 버튼 대신). 2 단 계단은 모서리마다 한 번씩 = 연속 점프.

    obstacle plateau   평대 (높이 step_h, 윗면 length)
    obstacle stairs2   ㅗ 모양 2 단 (한 단 step_h, 칸 길이 tread: 아래 단 | 윗단 | 아래 단)
    --mode stance        한 바퀴 서기 위험 시험 (평지),  --mode none  점프 없이 달리기

튜닝 값은 아래 TUNE 에 모여 있다.
    pv jump [--obstacle stairs2 ...]      창으로 보기     pv jump --record    mp4 로 (헤드리스)
"""
import argparse
import json
import math
import os
import itertools
import sys

from isaaclab.app import AppLauncher

# ================================ 튜닝 — 여기 값만 바꾸고 실행 ================================
# `pv jump` 로 창을 띄워 본다. 한 번만 바꿔 보려면 명령줄 `--이름 값` (예: pv jump --trigger 0.35)
TUNE = dict(
    # --- 제어기: policy (강화학습 균형 정책) | lqr (바퀴 LQR + 다리 VMC, lqr_vmc.py — Ascento 식, 학습 없음) ---
    ctrl="lqr",
    lqr_qx=2.0, lqr_qv=5.0, lqr_qth=100.0, lqr_qthd=5.0,   # LQR 상태 가중 [진행거리 m, 속도 m/s, 진자각 rad, 각속도 rad/s]
    lqr_r=1.0,           # LQR 입력 가중 (두 바퀴 토크 합 N·m)
    wheel_tau_max=7.0,   # 바퀴 토크 한계 [N·m] (AK45-10 피크)
    wz_max=2.5,          # 제자리 회전 최대 [rad/s] (143 deg/s). 5 는 사용자가 너무 빠르다고 함 (4 / 6 rad/s 도 추종·안정은 됨)
    turn_limit=False,    # 달릴 때 회전 한계 (wheel_margin) 켜기. 사용자: 조향은 사람이 조심 -> 끔 (자갈길 3 km/h 급회전은 넘어질 수 있음)
    rl_on=True,          # 잔차 RL 보정 섞기 (패드 B 로 켜기/끄기). 드라이브 중에만, 들림·점프 중엔 끔
    rl_policy="logs/rsl_rl/wheeled_biped_residual/2026-09-26_19-59-38_pv_rl2/model_2200.pt",   # 잔차 정책 (pv rl2 2200: 대회형 시험 세트 72/72)
    wheel_margin=0.7,    # (0.85 -> 0.7: 자갈길 최고 속도 급회전 6/8 -> 8/8. 3 km/h 에서는 회전 0.5 rad/s 로 제한됨) 달리며 돌 때 바깥 바퀴 속도 한계 = 모터 한계 x 이 비율 -> 속도가 빠를수록 회전 한계를 줄인다
    vmax_kmh=3.0,        # 최고 속도 [km/h] (사용자 2026-09-26). 스틱 끝 = 이 속도. 모터 한계는 4.1 km/h (18.85 rad/s x 0.06)
    vmax_motor_frac=0.75,  # 최고 속도 <= 이 비율 x (배터리 전압으로 추정한 바퀴 모터 한계 x R). 모터가 약하면 자동으로 낮춘다
                         #   (만충 18.85 rad/s: 0.85 m/s = 3.05 km/h, 모터 -13 %: 2.66 km/h). 약한 모터가 경사로·삼각형길에서 넘어짐 (pv robust)
    bal_adapt=0.3,       # 균형점 자동 보정 [1/s]: 서 있거나 일정 속도일 때 추정 진자각 평균을 천천히 학습해 뺀다
                         #   (무게중심 오차 2 cm = 약 4.6 deg, IMU 바이어스도 같이). 0 = 끔
    bal_adapt_max_deg=8.0, # 보정 한계 [deg]
    brake_kp=1.0,        # 최고 속도 초과 브레이크 P: 한계 = 최고 - (kp x 초과 + 적분). 켜고 끄지 않고 부드럽게 조인다
    brake_ki=5.5,        # 브레이크 I [1/s]: 내리막에서 필요한 제동량을 찾아가고, 속도가 내려가면 서서히 풀린다
    speed_lpf_hz=4.0,    # 브레이크가 보는 속도 필터 [Hz]
    accel_max=1.5,       # 목표 속도 변화율 한계 [m/s^2] (스틱·브레이크 모두)
    speed_guard=0.8,     # 바퀴 관절 속도가 모터 한계의 80 % 를 넘으면 목표 속도를 낮춰 뒤로 젖히며 감속 (푸시백). 1 = 끔.
                         #   자갈길 3 km/h 급회전에서 기울며 가속 -> 모터 한계 -> 고꾸라짐 3/3 -> 0/3 (2026-09-26).
                         #   최고 속도 3 km/h (13.9 rad/s) 가 기준 (15.1) 아래라 평지 직진에서는 걸리지 않음
                         #   한계에 붙으면 앞으로 기울 때 바퀴를 더 못 돌려 고꾸라진다 (내리막 0.9 m/s, 2026-09-26). 1 = 끔
    yaw_kd=0.5,          # 회전: 좌우 바퀴 토크 차 = yaw_kd x (명령 - 실제 yaw rate) [N·m·s/rad]
    vmc_kp=60.0,         # 다리 가상 스프링 [N·m/rad, 관절] (바퀴에서 약 4.5 kN/m. 30 은 좌우 수평이 못 따라가 넘어짐). 자중은 피드포워드로 따로 받친다
    vmc_kd=1.0,          # 다리 가상 댐퍼 [N·m·s/rad]
    turn_lean=1.0,       # 회전 중 안쪽 기울기 비율: roll 목표 = atan(turn_lean x v x wz / g) (Ascento lean 모드). 0 = 수평 유지
                         #   (수평으로 붙잡으면 원심력에 바깥으로 넘어짐: 0.6 m/s + 1 rad/s 평지 33 % 넘어짐, 2026-09-26)
    roll_kp=1.5,         # (3/30/0.6 은 센서 잡음 + 지연에서 다리가 흔들려 회전 중 바퀴가 뜸 -> 1.5/15/0.3, pv robust 52/56, 2026-09-26)         # roll 수평 P: 좌우 다리 길이 차 += roll_kp x 0.198 x sin(roll)
    roll_ki=15.0,        # roll 수평 I [1/s]
    roll_rate_lpf_hz=8.0,  # roll D 항에 넣는 roll 각속도 저역통과 [Hz]. 필터 없이 D 0.6 + 지연 5 ms 면 다리가 발진 (3 mm/스텝 떨림 -> 넘어짐)
    roll_kd=0.3,         # roll 각속도 D [s]: 좌우 다리 길이 차 += roll_kd x 0.198 x roll rate (흔들림 감쇠)
    land_sf_min=0.4,     # 접지 판정에 IMU 비력 > 이 값 [g] 도 요구 (0 = 안 봄). 자유낙하 중 다리 토크 오판 방지용
    roll_leak=0.5,       # roll 적분 누설 [1/s] (약 2 s 에 걸쳐 0 쪽으로 — 한계에 붙어 있지 않게)
    lift_detect_s=0.3,   # (0.05 면 돌 위에서 잠깐 뜨는 것도 들림으로 봐 균형이 꺼짐) 두 다리 모두 무하중(고관절 토크 < contact_tau_min)이 이만큼 [s] -> 들림: 균형 끄고 바퀴만 멈춤, 적분 비움
    land_detect_s=0.02,  # 들린 뒤 한쪽 다리라도 하중이 이만큼 [s] -> 내려놓음: 균형 즉시 재개 (Ascento 처럼 접지 즉시.
                         #   두 다리 0.1 s 기다리면 한 바퀴부터 닿는 동안 균형 없이 뒤로 넘어짐, 손 들기 시험)
    lift_wheel_kd=0.05,  # 들린 동안 바퀴 속도 감쇠 [N·m·s/rad] (헛돌지 않게)
    contact_tau_min=0.8, # 접지 판정: 고관절 토크가 다리를 펴는 쪽(+, 몸을 받침)으로 이 이상 [N·m] (실기: 전류). 뜨면 0 이나 반대 부호
    roll_freeze_deg=20.0,  # |roll| 이 이보다 크면 roll 적분 멈춤 (다리로 못 잡는 기울기)
    # --- 다리 높이: 항상 자동 모드 (정책이 외란·지형에 맞춰 정한다). idle_h 는 자동 모드 공칭 = IDLE ---
    level_rate=10.0,     # 좌우 수평 유지 [1/s]: 좌우 다리 길이 차를 roll 이 0 이 되게 적분 (d(hL-hR)/dt = rate x 0.198 x sin(roll)).
                         #   정책은 평균 높이·앞뒤 균형만, 좌우 차는 이 루프가 맡는다 (IMU + 다리 각도만 쓰니 실기 그대로). 0 = 끔 (정책이 좌우도)
    level_max=0.10,      # 좌우 다리 길이 차 한계 [m]
    leg_kp=60.0,         # 주행 중 고관절 P [N·m/rad] (학습값 60 = 바퀴에서 4.5 kN/m, 정하중 처짐 4 mm = 딱딱).
    leg_kd=1.5,          # 주행 중 고관절 D [N·m·s/rad]. 낮추면 서스펜션처럼 먼저 받아 준다 (실기 MIT kp/kd 그대로)
    idle_h=0.1825,       # [m, 다리 관절값]. 행정 가운데 = 좌우 ±60 mm. 0.20 은 신장 여유 42.5 mm 라 8 cm 엇갈린 삼각형길에서 다리가 끝에 닿음 (2026-09-26 LQR 시험). 충격 흡수 계산 (2026-09-26 계산, leg_map + URDF 4.03 kg):
                         #   정하중 토크는 행정 전체 2.1~2.3 N·m (정격 3 아래) 라 제약이 아니다.
                         #   9 N·m 로 바닥까지 눌리며 흡수 가능한 에너지 = 약 1.15 J / 압축 10 mm.
                         #   착지 2.9 J (1.2 m/s), 8 cm 낙하 3.2 J -> 0.1365(CAD 자세) 는 1.6 J 라 바닥을 친다.
                         #   0.1825 = 6.8 J (2.2 배 여유) + 신장 60 mm (구덩이). ※ r3 정책은 자동 공칭 0.1365 로 학습됐다
    # --- 접근·발동 ---
    heading_kp=2.0,      # 자동 시험 방향 유지: wz = -heading_kp x yaw [1/s] (정책이 yaw 로 흘러서 직선 주행이 안 된다). 0 = 끔. 패드는 사람이 조향
    v=0.4,               # 접근 속도 [m/s] (자동 시험용. 패드는 스틱). 바퀴 한계 18.85 rad/s x R 0.06 = 1.13 m/s, 웅크림·숙임 여유 두고 0.5~0.6
    jump_max_wz=1.0,     # 패드 점프는 회전 속도가 이보다 작을 때만 [rad/s] (돌면서 뛰면 공중에서 5.6 rad/s 로 돌며 착지 실패)
    jump_min_v=0.2,      # 패드 점프는 앞으로 이 속도 [m/s] 이상일 때만 (0.72 km/h). 제자리·후진 중 점프 막기
    trigger=0.34,        # 바퀴 중심이 모서리 앞 이 거리에 오면 발동 [m] (자동 시험). LQR v 0.4: 이륙 -185 mm, 착지 +9 mm (2026-09-26)
    # --- 1 retract: 웅크림 (정책이 균형) ---
    t_retract=0.30,      # [s]. 앞 75 % 동안 램프로 접고 나머지는 유지. 한 번에 접으면 바퀴가 들린다.
                         #   빨리 접으면 다리 링크 회전이 바퀴 관절 속도에 더해져 모터 한계 18.85 rad/s 에 붙는다
                         #   (0.12 s, v 0.8: 14.2 -> 18.7 rad/s, 토크 0 = 바퀴 제어 불능). HUD WHEEL rad/s 로 확인
    # --- 2 extract: 신전 = 이륙 (바퀴는 pitch PD) ---
    retract_lean=3.0,    # (LQR 3 deg. 8 이면 숙이며 가속해 바퀴 속도 한계 -> 공중 자세 제어 불능) 웅크리는 동안 앞으로 숙일 pitch [deg]. 0 = 균형 정책 그대로. >0 = 바퀴 pitch PD 로 숙인다
                         #   (펴는 동안 고관절 반작용으로 몸이 뒤로 ~17 deg 젖혀지며 전진 속도를 잃는다 -> 미리 숙여 상쇄. 숙이면 가속도 된다)
    extract_wheels="pd",  # 신전 중 바퀴: pd (pitch PD. 숙임 8 deg 와 같이 쓰면 이륙 0.62 m/s, 숙임 없으면 0.18) | policy (균형 정책 그대로) | free (토크 0)
    extract_pitch=0.0,   # 신전 중 pitch 목표 [deg, + = 앞으로 숙임]. 뒤로 젖힌 채 밀면 전진 속도를 잃는다 (-7 deg 에서 0.4 m/s)
    # --- 3 fly / 4 descend: 공중 (바퀴 = 반작용 휠) ---
    t_tuck=0.12,         # 이륙 뒤 다리를 최대로 접어 두는 시간 [s]. 그 뒤 착지 길이로 부드럽게 편다
    air_ctrl=True,       # False = Ascento 원형 (공중 바퀴 토크 0) -> 뒤로 -190 deg/s 로 넘어갔다
    pd_from="extract",   # 바퀴 pitch PD 를 켜는 단계: extract | fly
    air_pitch=-10.0,     # 공중 pitch 목표 [deg]. 음수 = 뒤로 젖힘 -> 몸 아래 바퀴가 앞으로 나간다
    air_kp=30.0,         # [N·m/rad, 바퀴 하나]
    air_kd=3.0,          # [N·m·s/rad, 바퀴 하나]
    air_tau=7.0,         # PD 구간 바퀴 토크 한계 [N·m] (AK45-10 피크. 평소 정책은 1.5)
    # --- 5 land: 착지 ---
    h_land=0.1825,       # 착지 다리 길이 [m] (행정 0.1225 ~ 0.2425). idle_h 와 같게 = 압축 행정 최대
    land_kp=20.0,        # 착지 스프링 [N·m/rad] (평소 60)
    land_kd=1.0,         # 착지 댐퍼 [N·m·s/rad] (평소 1.5)
    land_s=0.30,         # 부드러운 게인 유지 시간 [s]
    contact_tau=2.0,     # 접지 판정 고관절 토크 [N·m] (descend 0.04 s 뒤부터)
    t_fly_max=0.45,      # 이륙 뒤 이 시간 안에 접지를 못 느끼면 강제로 land [s]
    # --- 장애물 ---
    obstacle="plateau",  # plateau (평대) | stairs2 (ㅗ 모양 2 단) | ridges (ㅅㅅㅅ 삼각형길) | cad (아래 cad_file)
                         #   | gen (지형 생성기 요철: gen_type) | env (Isaac 기본 환경: env_name)
    gen_type="stones",   # obstacle="gen": stones (둥근 돌 자갈길) | gravel (10 cm 격자 블록, 수직 턱) | bumps (흩어진 턱) | waves
                         #   | oneside_stones / oneside (왼쪽 바퀴 차선만 -> 두 바퀴가 늘 다른 높이)
    gen_h=0.06,          # 요철 최대 높이 [m] (5~8 cm 시험)
    gen_len=8.0,         # 요철 길이 [m] (출발 앞 edge 만큼 평지)
    gen_seed=0,
    env_name="rough_plane",  # obstacle="env": rough_plane | slope | stairs | warehouse | warehouse_full | warehouse_shelves |
                         #   hospital | office | grid | rivermark (야외) | twin_warehouse  (Isaac 클라우드 에셋, 처음엔 내려받음)
    cad_file="~/perseverance/sim/obstacles/obstacle.stl",   # obstacle="cad": STL/OBJ/FBX. 좌표 규약 (CAD 에서 그대로):
                         #   원점 = 출발할 때 두 바퀴 축 가운데 바로 아래 바닥, +X = 달리는 방향, +Z = 위, 바닥면 Z = 0.
                         #   바퀴는 Y = +-99 mm 를 지난다. 파일을 고쳐 저장하면 다음 실행 때 다시 변환한다
    cad_unit=0.001,      # CAD 단위 -> m (mm 로 뽑았으면 0.001, m 면 1.0)
    step_h=0.08,         # 턱(한 단) 높이 [m]
    length=0.35,         # 평대 윗면 길이 [m]
    tread=1.0,           # stairs2 한 칸 길이 [m] (대회: 1 m | 1 m | 1 m)
    edge=1.0,            # 첫 모서리 x [m] (ridges: 평지 도움닫기 길이)
    ridge_h=0.05,        # ridges 삼각 턱 높이 [m] (대회 약 5 cm)
    ridge_base=0.35,     # ridges 밑변 [m] (길게)
    ridge_period=0.60,   # ridges 반복 주기 [m]
    ridge_lane=0.20,     # ridges 차선 폭 [m]. 차선마다 반 주기 엇갈림 -> 좌우 바퀴(간격 198 mm)가 번갈아 탄다
    ridge_len=5.0,       # ridges 길이 [m]
    # --- 현실화 (시뮬을 실기 조건으로) ---
    est="sensors",       # truth: 시뮬 정답 상태 | sensors: 실기처럼 IMU(잡음·바이어스) + 관절·바퀴 엔코더 + 명목 질량 모델로 추정
    imu_tilt_noise_deg=0.3,   # 기울기 추정 잡음 σ [deg]
    imu_tilt_bias_deg=0.5,    # 기울기 바이어스 (시작 때 ± 무작위) [deg]
    imu_gyro_noise=0.01,      # 자이로 잡음 σ [rad/s]
    enc_vel_noise=0.05,       # 바퀴 엔코더 속도 잡음 σ [rad/s]
    v_fuse_hz=0.0,       # 속도 추정: IMU 가속도 적분 + 바퀴 오도메트리 상보 필터 교차 주파수 [Hz] (0 = 오도메트리 저역통과만). 꺼 둠:
                         #   몸통 가속도 != 바퀴축 가속도 (pitch 흔들림) -> 기구학 변환 없이 섞으면 평지도 0.37 m/s 틀림. 변환 넣은 뒤 다시
                         #   턱에서 바퀴가 튀고 미끄러지면 오도메트리만으로는 0.6~0.9 m/s 틀림 (삼각형길, 2026-09-26)
    imu_acc_noise=0.05,  # 가속도계 잡음 σ [m/s^2]
    v_lpf_hz=10.0,       # 속도 추정 저역통과 [Hz]: 바퀴 엔코더 속도는 바퀴 자체의 빠른 요동을 담는다 -> 걸러야 지연과 겹쳐 발진 안 함
                         #   (필터 없이 센서 + 지연 5 ms 면 바퀴 속도 +-18 rad/s 발진 -> 넘어짐, 2026-09-26)
    delay_ms=5.0,        # 제어 지연: 센서 -> 명령 적용 [ms] (200 Hz 한 주기 = 5 ms)
    jitter_ms=2.0,       # 지연 흔들림: 가끔 한 주기 더 늦음 (확률 = jitter/5)
    dr_mass=0.15,        # 몸통 질량 ± 비율 (제어기는 명목값을 믿음)
    dr_com_cm=2.0,       # 몸통 무게중심 ± [cm] (x, z)
    dr_motor=0.15,       # 바퀴 모터 토크·속도 한계 - 0~비율 (배터리 처짐)
    dr_seed=0,           # 0 = 실행마다 무작위, 그 외 = 고정 시드
    # --- 화면 ---
    render_hz=50.0,      # 창 렌더 주기 [Hz]. 낮출수록 창이 빨라진다 (재시작)
    physics_hz=400.0,    # 물리 주기 [Hz] (제어 200 Hz 의 배수, 재시작). 400 에서도 점프 착지 +17 mm, 삼각형길 통과 (800 과 같음)
    color_body="#1F3A5F",    # 몸통 색 (재시작)
    color_legs="#F2A541",    # 다리 링크 색
    color_wheels="#202020",  # 바퀴 색
    # --- 기타 ---
    spawn_z=0.10,        # 출발할 때 바퀴 바닥 높이 [m] — 공중에서 떨어뜨려 시작 (사용자 2026-09-26). 0 = 바닥에 닿게
    hip="dc",            # dc (토크-속도 모델) | ideal
    hip_w0=None,         # 고관절 무부하 속도 [rad/s] (24 V 33.5, 6S 처짐 21 V 29.3). None = 33.5
    seconds=8.0,
)
POLICY = "logs/rsl_rl/wheeled_biped_balance/2026-09-25_18-24-20_rough_v3/model_7099.pt"   # 관측 25 균형 정책 (r3)
# ==========================================================================================

CHOICES = dict(est=("truth", "sensors"), ctrl=("policy", "lqr"), obstacle=("flat", "plateau", "stairs2", "ridges", "cad", "gen", "env"),
               gen_type=("stones", "gravel", "bumps", "waves", "oneside", "oneside_stones"), extract_wheels=("pd", "policy", "free"), pd_from=("extract", "fly"), hip=("dc", "ideal"))
ap = argparse.ArgumentParser()
ap.add_argument("--policy", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", POLICY))
ap.add_argument("--mode", choices=("jump", "stance", "none"), default="jump")
for k, v in TUNE.items():
    if isinstance(v, bool):
        ap.add_argument(f"--{k}", type=lambda x: x.lower() in ("1", "true", "on", "yes"), default=v)
    elif isinstance(v, str):
        ap.add_argument(f"--{k}", choices=CHOICES.get(k), default=v)
    else:
        ap.add_argument(f"--{k}", type=float, default=v)
ap.add_argument("--joystick", nargs="?", const="/dev/input/js0", default=None, metavar="DEV",
                help="패드로 조종: 왼스틱 세로 전후, 오른스틱 가로 회전, Y/LT/RT 점프, A 정지, START 처음으로, 십자키/LB/RB/BACK 카메라")
ap.add_argument("--pad", choices=("auto", "classic", "modern"), default="classic")
ap.add_argument("--wz", type=float, default=None, help="자동 시험: 회전 명령 [rad/s] 고정 (방향 유지 대신)")
ap.add_argument("--hand", type=float, nargs=2, default=None, metavar=("T_GRAB", "T_RELEASE"),
                help="손 들기 시험: 이 시각에 가상 손(몸통 스프링)으로 들었다가 놓는다 [s]")
ap.add_argument("--hand_lift", type=float, default=0.15, help="손으로 드는 높이 [m]")
ap.add_argument("--hand_roll", type=float, default=10.0, help="들고 있는 동안 옆으로 기울이는 각 [deg] (한쪽 바퀴부터 닿게)")
ap.add_argument("--stop_at", type=float, default=None, help="자동 시험: 이 시각 [s] 에 속도 명령 0 (달리다 멈추기)")
ap.add_argument("--record", default=None, metavar="DIR")
ap.add_argument("--out", default=None)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
if args.record:
    args.enable_cameras = True
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import wheeled_biped_isaaclab.tasks  # noqa: F401,E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.terrains import TerrainImporterCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from wheeled_biped_isaaclab.tasks.balance import cad  # noqa: E402

TASK = "Isaac-WheeledBiped-CAD-Rough-Play-v0"
R = cad.R_WHEEL
H_MIN, H_MAX = 0.1225, 0.2425
H_CRUISE = 0.18

cfg = parse_env_cfg(TASK, device=args.device, num_envs=1, use_fabric=True)
cfg.scene.terrain = TerrainImporterCfg(
    prim_path="/World/ground", terrain_type="plane", collision_group=-1,
    physics_material=sim_utils.RigidBodyMaterialCfg(friction_combine_mode="multiply", restitution_combine_mode="multiply",
                                                    static_friction=1.0, dynamic_friction=1.0))
if args.hip == "dc":
    cfg.scene.robot = cad.CAD_ROBOT_CFG_DCHIP.replace(prim_path="{ENV_REGEX_NS}/Robot")
    if args.hip_w0:
        cfg.scene.robot.actuators["legs"].velocity_limit = args.hip_w0
cfg.events.reset_base.params["pose_range"] = {}
cfg.events.reset_base.params["velocity_range"] = {}
cfg.events.base_mass = None
cfg.events.base_com = None
# 시험에는 학습용 계산이 필요 없다 — 매 스텝(200 Hz) 부가 계산을 덜어 창 속도를 올린다
if hasattr(cfg.events, "push"):
    cfg.events.push = None
for _grp in (cfg.rewards, cfg.curriculum):
    for _n in list(vars(_grp)):
        if not _n.startswith("_"):
            setattr(_grp, _n, None)
if hasattr(cfg.observations, "critic"):
    cfg.observations.critic = None                                     # 크리틱 관측 (지형 스캔 117 레이)
if hasattr(cfg.scene, "critic_scanner"):
    cfg.scene.critic_scanner = None
cfg.terminations.time_out = None
if args.joystick:                                                     # 패드: 부딪히거나 기울어도 리셋하지 않고 계속 균형 (START 로만 처음으로)
    for _n in list(vars(cfg.terminations)):
        if not _n.startswith("_"):
            setattr(cfg.terminations, _n, None)


def box(name, x0, x1, h):
    """정적 상자 (rigid body 없음 = 고정 충돌체). 폭 3 m 라 옆으로 비껴갈 일은 없다."""
    return AssetBaseCfg(
        prim_path=f"/World/{name}",
        spawn=sim_utils.CuboidCfg(size=(x1 - x0, 3.0, h), collision_props=sim_utils.CollisionPropertiesCfg(),
                                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.45, 0.35))),
        init_state=AssetBaseCfg.InitialStateCfg(pos=((x0 + x1) / 2, -0.08, h / 2)))


edges, top = [], 0.0
if args.mode in ("jump", "none"):
    if args.obstacle == "plateau":
        cfg.scene.obstacle = box("obstacle", args.edge, args.edge + args.length, args.step_h)
        edges, top = [args.edge], args.step_h
    elif args.obstacle == "stairs2":
        # ㅗ 모양 (대회): 아래 단 tread | 윗단 tread | 아래 단 tread. 오르는 모서리 둘만 점프 대상
        cfg.scene.obstacle = box("tier1", args.edge, args.edge + 3 * args.tread, args.step_h)
        cfg.scene.obstacle2 = box("tier2", args.edge + args.tread, args.edge + 2 * args.tread, 2 * args.step_h)
        edges, top = [args.edge, args.edge + args.tread], 2 * args.step_h
if args.obstacle == "gen" and args.mode in ("jump", "none"):
    # 지형 생성기 요철 (terrain_gen.py). 타일 가운데 = 출발점, 앞 edge 까지 평지, 그 뒤 gen_len 동안 요철
    from isaaclab.terrains import TerrainGeneratorCfg
    from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg
    from isaaclab.terrains.height_field.utils import height_field_to_mesh
    import terrain_gen

    GX, GY = 2 * (args.edge + args.gen_len), 6.0

    @height_field_to_mesh
    def gen_field(difficulty, c):
        nx, ny = int(c.size[0] / c.horizontal_scale), int(c.size[1] / c.horizontal_scale)
        z = terrain_gen.make_heights(args.gen_type, nx, ny, c.horizontal_scale, args.gen_h,
                                     int((GX / 2 + args.edge) / c.horizontal_scale), seed=int(args.gen_seed))
        return np.rint(z / c.vertical_scale).astype(np.int16)

    @configclass
    class GenCfg(HfTerrainBaseCfg):
        function = gen_field

    cfg.scene.terrain = TerrainImporterCfg(
        prim_path="/World/ground", terrain_type="generator", collision_group=-1, max_init_terrain_level=None,
        terrain_generator=TerrainGeneratorCfg(
            size=(GX, GY), num_rows=1, num_cols=1, horizontal_scale=0.02, vertical_scale=0.001, slope_threshold=None,
            curriculum=False, use_cache=False, sub_terrains={"gen": GenCfg(proportion=1.0, border_width=0.0)}),
        physics_material=sim_utils.RigidBodyMaterialCfg(friction_combine_mode="multiply", restitution_combine_mode="multiply",
                                                        static_friction=1.0, dynamic_friction=1.0))
    cfg.scene.floor = AssetBaseCfg(
        prim_path="/World/floor",
        spawn=sim_utils.CuboidCfg(size=(60.0, 60.0, 0.1), collision_props=sim_utils.CollisionPropertiesCfg(),
                                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.25, 0.25, 0.27))),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.0505)))
    _p = cfg.scene.robot.init_state.pos
    cfg.scene.robot.init_state.pos = (_p[0], _p[1] + 0.08, _p[2])     # 바퀴 가운데 = 타일 가운데 (oneside 차선 경계)
if args.obstacle == "env":
    # Isaac 기본 환경 (클라우드 USD). 레이 스캐너는 메시 하나만 받아서 끈다 (종료 판정도 같이)
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
    ENVS = dict(rough_plane="Terrains/rough_plane.usd", slope="Terrains/slope.usd", stairs="Terrains/stairs.usd",
                warehouse="Simple_Warehouse/warehouse.usd", warehouse_full="Simple_Warehouse/full_warehouse.usd",
                warehouse_shelves="Simple_Warehouse/warehouse_multiple_shelves.usd", hospital="Hospital/hospital.usd",
                office="Office/office.usd", grid="Grid/default_environment.usd", rivermark="Outdoor/Rivermark/rivermark.usd",
                twin_warehouse="Digital_Twin_Warehouse/small_warehouse_digital_twin.usd")
    if args.env_name not in ENVS:
        raise SystemExit(f"env_name 은 {list(ENVS)} 중 하나")
    cfg.scene.terrain = TerrainImporterCfg(
        prim_path="/World/ground", terrain_type="usd", usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/{ENVS[args.env_name]}",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(friction_combine_mode="multiply", restitution_combine_mode="multiply",
                                                        static_friction=1.0, dynamic_friction=1.0))
    if hasattr(cfg.scene, "height_scanner"):
        cfg.scene.height_scanner = None
    if hasattr(cfg.terminations, "too_low"):
        cfg.terminations.too_low = None
    _p = cfg.scene.robot.init_state.pos
    cfg.scene.robot.init_state.pos = (_p[0], _p[1], _p[2] + 0.3)     # 지면 높이를 몰라 조금 더 위에서
if args.obstacle == "ridges" and args.mode in ("jump", "none"):
    # 학습 지형과 같은 함수 (terrain.staggered_ridges_terrain) 에 평지 도움닫기만 붙인다. 타일 가운데 = 로봇 출발점
    from isaaclab.terrains import TerrainGeneratorCfg
    from isaaclab.terrains.height_field.utils import height_field_to_mesh
    from wheeled_biped_isaaclab.tasks.balance import terrain as T

    SX, SY = 2 * (args.edge + args.ridge_len), 4.0                   # 출발점 x = SX/2, 차선 경계 y = SY/2 = 2.0 (0.2 의 배수)

    @height_field_to_mesh
    def ridges_runup(difficulty, c):
        z = T.staggered_ridges_terrain.__wrapped__(difficulty, c)
        i0 = int((SX / 2 + args.edge) / c.horizontal_scale)          # 출발점 앞 edge 까지 평지.
        for j in range(z.shape[1]):                                    # 차선마다 삼각형이 0 인 곳부터 시작 (한 x 로 자르면
            i = i0                                                     #  엇갈린 차선에 수직 턱이 생긴다 — 2.1 cm, 2026-09-26)
            while i < z.shape[0] and z[i, j] != 0:
                i += 1
            z[:i, j] = 0
        return z

    @configclass
    class RidgesRunupCfg(T.StaggeredRidgesTerrainCfg):
        function = ridges_runup

    cfg.scene.terrain = TerrainImporterCfg(
        prim_path="/World/ground", terrain_type="generator", collision_group=-1, max_init_terrain_level=None,
        terrain_generator=TerrainGeneratorCfg(
            size=(SX, SY), num_rows=1, num_cols=1, horizontal_scale=0.02, vertical_scale=0.001, slope_threshold=None,
            curriculum=False, use_cache=False,
            sub_terrains={"ridges": RidgesRunupCfg(proportion=1.0, height_range=(args.ridge_h, args.ridge_h), base=args.ridge_base,
                                                   period=args.ridge_period, lane_w=args.ridge_lane, border_width=0.0)}),
        physics_material=sim_utils.RigidBodyMaterialCfg(friction_combine_mode="multiply", restitution_combine_mode="multiply",
                                                        static_friction=1.0, dynamic_friction=1.0))
    cfg.scene.floor = AssetBaseCfg(                                    # 타일 밖으로 나가도 허공에 떨어지지 않게 바닥판 (윗면 z = 0)
        prim_path="/World/floor",
        spawn=sim_utils.CuboidCfg(size=(60.0, 60.0, 0.1), collision_props=sim_utils.CollisionPropertiesCfg(),
                                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.25, 0.25, 0.27))),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.0505)))
    # 두 바퀴 가운데 = 루트 y - 0.08 (스캐너 오프셋과 같음) -> 루트를 +0.08 에 두면 바퀴 가운데가 차선 경계에 온다
    _p = cfg.scene.robot.init_state.pos
    cfg.scene.robot.init_state.pos = (_p[0], _p[1] + 0.08, _p[2])
if args.obstacle == "cad" and args.mode in ("jump", "none"):
    # CAD 메시 -> USD (정적 충돌체, 삼각형 메시 그대로). 결과는 ~/pv_out/cad_obstacles/ 에 캐시 (파일이 바뀌면 다시 변환)
    from isaaclab.sim.converters import MeshConverter, MeshConverterCfg
    from isaaclab.sim.schemas import schemas_cfg
    _src = os.path.expanduser(args.cad_file)
    if not os.path.isfile(_src):
        raise SystemExit(f"CAD 파일이 없다: {_src}  (TUNE 의 cad_file 또는 --cad_file)")
    _conv = MeshConverter(MeshConverterCfg(
        asset_path=_src, usd_dir=os.path.expanduser("~/pv_out/cad_obstacles"),
        usd_file_name=os.path.splitext(os.path.basename(_src))[0] + ".usd", make_instanceable=False,
        collision_props=schemas_cfg.CollisionPropertiesCfg(collision_enabled=True),
        mesh_collision_props=schemas_cfg.TriangleMeshPropertiesCfg(),          # 삼각형 메시 그대로 (정적 충돌체라 가능)
        scale=(args.cad_unit,) * 3))
    print(f"[CAD] {_src} -> {_conv.usd_path}", flush=True)
    cfg.scene.cad_obstacle = AssetBaseCfg(prim_path="/World/cad_obstacle", spawn=sim_utils.UsdFileCfg(usd_path=_conv.usd_path),
                                          init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)))
    _p = cfg.scene.robot.init_state.pos                                  # 바퀴 가운데 = 루트 y - 0.08 -> CAD Y 0 에 맞춘다
    cfg.scene.robot.init_state.pos = (_p[0], _p[1] + 0.08, _p[2])
_p = cfg.scene.robot.init_state.pos                                   # 공중 스폰: 바퀴 바닥이 spawn_z 에 오게
cfg.scene.robot.init_state.pos = (_p[0], _p[1], _p[2] + args.spawn_z)
cfg.decimation = max(1, round(args.physics_hz / 200.0))              # 제어는 200 Hz 그대로
cfg.sim.dt = 1.0 / (200.0 * cfg.decimation)
if not args.headless and not args.record:
    cfg.sim.render_interval = cfg.decimation * max(1, round(200.0 / args.render_hz))   # 200 Hz 마다 그리면 실시간의 0.17 배
if True:                                                              # 옆에서 로봇을 따라가는 카메라 (창·녹화 공통)
    cfg.viewer.origin_type = "asset_root"; cfg.viewer.asset_name = "robot"; cfg.viewer.env_index = 0
    cfg.viewer.eye = (0.2, -1.4, 0.35); cfg.viewer.lookat = (0.0, 0.0, 0.1); cfg.viewer.resolution = (1280, 720)
def paint_robot(env, env_ids):
    """몸통·다리·바퀴에 색. 시뮬 시작 전(prestartup)에 링크 프림에 PreviewSurface 를 묶는다
    (시작 뒤에 묶으면 물리 뷰가 무효화돼 죽는다)."""
    import omni.usd
    rgb = lambda h: tuple(int(h.lstrip("#")[i:i + 2], 16) / 255.0 for i in (0, 2, 4))  # noqa: E731
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath("/World/envs/env_0/Robot")
    groups = {"body": [], "legs": [], "wheels": []}
    for c in root.GetChildren():
        n = c.GetName()
        if "wheel" in n:
            groups["wheels"].append(n)
        elif n == "base_link":
            groups["body"].append(n)
        elif any(k in n for k in ("crank", "shank", "rocker", "thigh", "leg")):
            groups["legs"].append(n)
    for grp, col in (("body", args.color_body), ("legs", args.color_legs), ("wheels", args.color_wheels)):
        mat = f"/World/Looks/pv_{grp}"
        cm = sim_utils.PreviewSurfaceCfg(diffuse_color=rgb(col), roughness=0.45, metallic=0.1)
        cm.func(mat, cm)
        for n in groups[grp]:
            sim_utils.bind_visual_material(f"/World/envs/env_0/Robot/{n}", mat, stronger_than_descendants=True)
    print(f"[색] 몸통 {args.color_body} {groups['body']}, 다리 {args.color_legs} {groups['legs']}, 바퀴 {args.color_wheels} {groups['wheels']}", flush=True)


from isaaclab.managers import EventTermCfg as _EventTerm  # noqa: E402
cfg.events.paint = _EventTerm(func=paint_robot, mode="prestartup")
cfg.scene.replicate_physics = False                                   # prestartup 이벤트 조건 (로봇 1 대라 비용 없음)
env = gym.make(TASK, cfg=cfg, render_mode="rgb_array" if args.record else None).unwrapped
dev = env.device
robot = env.scene["robot"]
if args.obstacle == "cad" and args.mode in ("jump", "none"):                # 마찰 1.0 (다른 장애물과 같게)
    sim_utils.spawn_rigid_body_material("/World/Materials/cadMat", sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0))
    sim_utils.bind_physics_material("/World/cad_obstacle", "/World/Materials/cadMat")
cmd = env.command_manager.get_term("base_velocity")
cmd.pin(True)


def load_policy(path):
    try:
        return torch.jit.load(path, map_location=dev).eval()
    except RuntimeError:
        pass
    sd = torch.load(path, map_location=dev, weights_only=False)["actor_state_dict"]
    layers = []
    for i in range(4):
        w = sd[f"mlp.{2*i}.weight"]
        lin = torch.nn.Linear(w.shape[1], w.shape[0]).to(dev)
        lin.weight.data[:] = w; lin.bias.data[:] = sd[f"mlp.{2*i}.bias"]
        layers += [lin] + ([torch.nn.ELU()] if i < 3 else [])
    mlp = torch.nn.Sequential(*layers).eval()
    mean, std = sd["obs_normalizer._mean"], sd["obs_normalizer._std"]
    return lambda x: mlp((x - mean) / (std + 1e-2))


policy = load_policy(args.policy)
leg_ids = robot.find_joints(cad.LEG_JOINTS, preserve_order=True)[0]
wheel_bodies = robot.find_bodies(["l_wheel", "r_wheel"], preserve_order=True)[0]
hip_sign = torch.tensor([cad.M_SIGN["L"], cad.M_SIGN["R"]], device=dev)
wheel_ids = robot.find_joints(cad.WHEEL_JOINTS, preserve_order=True)[0]
wheel_term = env.action_manager.get_term("wheels")
scale0 = wheel_term.cfg.torque_scale
wsign = torch.tensor(cad.WHEEL_SIGN, device=dev, dtype=torch.float32)   # 관절축 -> +y 부호 (기록용). 행동은 이미 +y 규약
legs_act = robot.actuators["legs"]



# --- LQR/VMC 모델 값: 시뮬 물체에서 직접 (실기는 URDF 같은 값) -----------------------------------------------
import numpy as _np  # noqa: E402
import lqr_vmc  # noqa: E402
_nonwheel = [i for i in range(robot.num_bodies) if i not in wheel_bodies]
_mass = robot.root_physx_view.get_masses()[0].clone().to(dev)    # 명목 질량 (복사! 모델 오차가 바꾸지 않게)
_m_pend = float(_mass[_nonwheel].sum()); _m_w = float(_mass[wheel_bodies].sum())
_I_w = 2 * 1.755e-3                                                   # URDF l_wheel izz (회전자 반사관성 포함) x 2
lqr = None


def build_lqr():
    global lqr
    d = robot.data
    c = (d.body_com_pos_w[0, _nonwheel] * _mass[_nonwheel, None]).sum(0) / _m_pend
    iyy = robot.root_physx_view.get_inertias()[0][:, 4].to(dev)
    rel = d.body_com_pos_w[0, _nonwheel] - c
    I = float((iyy[_nonwheel] + _mass[_nonwheel] * (rel[:, 0] ** 2 + rel[:, 2] ** 2)).sum())
    lqr = lqr_vmc.WheelLQR(_m_pend, I, _m_w, _I_w, R, q=(args.lqr_qx, args.lqr_qv, args.lqr_qth, args.lqr_qthd), r=args.lqr_r)
    return I


SENSE = dict(pitch=0.0, roll=0.0, gx=0.0, gy=0.0, gz=0.0, wl=0.0, wr=0.0)
VF = dict(v=0.0)                                                      # 속도 추정 필터 상태
RF = dict(r=0.0)                                                      # roll 각속도 필터 상태 (roll D 항용)
_rng = np.random.default_rng(int(args.dr_seed) if args.dr_seed else None)
_bias = _rng.uniform(-1, 1, 2) * math.radians(args.imu_tilt_bias_deg)      # (pitch, roll) 바이어스


def sense():
    """실기 센서: IMU 기울기(잡음+바이어스), 자이로(잡음), 바퀴 엔코더 속도(+y 규약, 잡음). truth 면 잡음 없이."""
    d = robot.data
    g = d.projected_gravity_b[0]
    p_, r_ = math.asin(max(-1.0, min(1.0, float(g[0])))), math.asin(max(-1.0, min(1.0, float(g[1]))))
    w = d.root_ang_vel_b[0]
    wv = (d.joint_vel[0, wheel_ids] * wsign).tolist()
    if args.est == "sensors":
        n = _rng.normal
        tn, gn, en = math.radians(args.imu_tilt_noise_deg), args.imu_gyro_noise, args.enc_vel_noise
        SENSE.update(pitch=p_ + _bias[0] + n(0, tn), roll=r_ + _bias[1] + n(0, tn), gx=float(w[0]) + n(0, gn),
                     gy=float(w[1]) + n(0, gn), gz=float(w[2]) + n(0, gn), wl=wv[0] + n(0, en), wr=wv[1] + n(0, en))
    else:
        SENSE.update(pitch=p_, roll=r_, gx=float(w[0]), gy=float(w[1]), gz=float(w[2]), wl=wv[0], wr=wv[1])


def lqr_state():
    """(진자각 θ, 각속도, 진자 길이 l, 바퀴 진행 속도 v, yaw rate). 실기: IMU + 다리 기구학 + 바퀴 엔코더."""
    d = robot.data
    if args.est == "sensors":
        # 무게중심의 몸체 좌표 위치 = 관절각으로 정해지는 기구학 (실기: URDF 순기구학) + 명목 질량. 기울기는 IMU.
        from isaaclab.utils.math import quat_apply_inverse
        c = (d.body_com_pos_w[0, _nonwheel] * _mass[_nonwheel, None]).sum(0) / _m_pend
        ax = d.body_pos_w[0, wheel_bodies].mean(0)
        rb = quat_apply_inverse(d.root_quat_w[0:1], (c - ax)[None])[0].tolist()
        th = SENSE["pitch"] + math.atan2(rb[0], rb[2])
        thd = SENSE["gy"]
        # 땅을 짚은 바퀴만 속도 추정에 쓴다 (뜬 바퀴는 헛돌아 엔코더가 의미 없다 — 삼각형길에서 한 바퀴가 떠
        # -16 rad/s 로 돌자 평균이 '후진 중' 으로 나와 LQR 이 앞으로 가속, 폭주·넘어짐, 2026-09-26). 접지 = 고관절 토크(실기: 전류)
        th_ = (d.applied_torque[0, leg_ids] * hip_sign).tolist()    # 부호 있음 (+ = 몸을 받침). 절댓값이면 뜬 다리의 반대 토크를 접지로 오판
        # 바퀴 절대 회전 = 엔코더(정강이 기준) + 정강이 링크 회전(몸체 pitch + 4절 링크가 다리 길이에 따라 도는 몫).
        # 실기: 링크 회전은 고관절 각속도 x 4절 링크 기구학으로 계산 (몸체 pitch 만 더하면 다리가 움직일 때 0.5 m/s 넘게 틀림).
        # 시뮬: 바퀴 절대 회전(= 그 계산의 결과)에 엔코더 잡음을 얹는다.
        psi_ = yaw_of(d.root_quat_w[0]); yh = torch.tensor([-math.sin(psi_), math.cos(psi_), 0.0], device=dev)
        wabs = (d.body_ang_vel_w[0, wheel_bodies] @ yh).tolist()
        if args.enc_vel_noise > 0:
            wabs = [w_ + _rng.normal(0, args.enc_vel_noise) for w_ in wabs]
        ws_ = [w_ for w_, t_ in zip(wabs, th_) if t_ >= args.contact_tau_min]
        if args.v_fuse_hz > 0:                                      # 상보 필터: 가속도 적분(빠른 성분) + 오도메트리(느린 성분)
            acc = d.body_lin_acc_w[0, 0].tolist()                   # 실기: 가속도계 비력을 자세로 돌려 중력 뺀 값
            a_h = acc[0] * math.cos(psi_) + acc[1] * math.sin(psi_) + _rng.normal(0, args.imu_acc_noise)
            VF["v"] += a_h * dt
            if ws_:
                VF["v"] += (1.0 - math.exp(-2 * math.pi * args.v_fuse_hz * dt)) * (R * sum(ws_) / len(ws_) - VF["v"])
            return th, thd, math.sqrt(sum(x * x for x in rb)), VF["v"], SENSE["gz"]
        if not ws_:                                                 # 둘 다 뜸 -> 직전 추정 유지
            return th, thd, math.sqrt(sum(x * x for x in rb)), VF["v"], SENSE["gz"]
        v_raw = R * sum(ws_) / len(ws_)
        if args.v_lpf_hz > 0:
            VF["v"] += (1.0 - math.exp(-2 * math.pi * args.v_lpf_hz * dt)) * (v_raw - VF["v"])
        else:
            VF["v"] = v_raw
        return th, thd, math.sqrt(sum(x * x for x in rb)), VF["v"], SENSE["gz"]
    c = (d.body_com_pos_w[0, _nonwheel] * _mass[_nonwheel, None]).sum(0) / _m_pend
    ax = d.body_pos_w[0, wheel_bodies].mean(0)
    psi = yaw_of(d.root_quat_w[0]); f = (math.cos(psi), math.sin(psi))
    r = (c - ax).tolist()
    th = math.atan2(r[0] * f[0] + r[1] * f[1], r[2])
    w = d.root_ang_vel_w[0].tolist()
    thd = -w[0] * f[1] + w[1] * f[0]
    vw = d.body_lin_vel_w[0, wheel_bodies].mean(0).tolist()
    return th, thd, math.sqrt(sum(x * x for x in r)), vw[0] * f[0] + vw[1] * f[1], w[2]


roll_pi = lqr_vmc.RollPI()


def apply_dr():
    """실제 로봇이 명목 모델과 다름: 몸통 질량·무게중심, 바퀴 모터 한계. 제어기(_mass, 18.85)는 명목값 그대로."""
    msg = []
    view = robot.root_physx_view
    idx = torch.arange(1, device="cpu")
    if args.dr_mass > 0:
        m = view.get_masses().clone(); k = 1.0 + _rng.uniform(-1, 1) * args.dr_mass
        m[0, 0] *= k; view.set_masses(m, idx); msg.append(f"몸통 질량 x{k:.2f}")
    if args.dr_com_cm > 0:
        c = view.get_coms().clone(); dx, dz = _rng.uniform(-1, 1, 2) * args.dr_com_cm / 100.0
        c[0, 0, 0] += dx; c[0, 0, 2] += dz; view.set_coms(c, idx); msg.append(f"무게중심 x{dx*100:+.1f} z{dz*100:+.1f} cm")
    if args.dr_motor > 0:
        wa = robot.actuators["wheels"]; k = 1.0 - _rng.uniform(0, 1) * args.dr_motor
        wa.velocity_limit = wa.velocity_limit * k if torch.is_tensor(wa.velocity_limit) else wa.velocity_limit * k
        if hasattr(wa, "_saturation_effort"):
            wa._saturation_effort = wa._saturation_effort * k
        msg.append(f"바퀴 모터 한계 x{k:.2f}")
    print("[모델 오차] " + (", ".join(msg) if msg else "없음") + f" | 추정 {args.est}, 지연 {args.delay_ms:.0f}+{args.jitter_ms:.0f} ms", flush=True)
    return msg


DR_MSG = apply_dr()

kp0, kd0 = legs_act.stiffness.clone(), legs_act.damping.clone()
kp0[:], kd0[:] = args.leg_kp, args.leg_kd


def yaw_of(q):
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def soft_legs(on):
    legs_act.stiffness[:] = args.land_kp if on else kp0
    legs_act.damping[:] = args.land_kd if on else kd0


pad = None
BTN_Y, BTN_BACK = 3, 6
import time  # noqa: E402
if args.joystick:
    from wheeled_biped_isaaclab import joystick_input as J
    pad = J.Gamepad(args.joystick)
    pad.poll()
    if args.pad != "auto":
        pad.axis_right_x = J.AXIS_RIGHT_X_CLASSIC if args.pad == "classic" else J.AXIS_RIGHT_X_MODERN
        pad.layout, pad._layout_done = args.pad, True
    print(f"[패드] {args.joystick} ({pad.layout}) — 왼스틱 세로 전후 / 오른스틱 가로 회전 / 높이 자동 / "
          f"Y 점프 / A 정지 / START 처음으로 / 십자키·LB·RB·BACK 카메라.  모서리: " + ", ".join(f"x {e:.2f} m" for e in edges), flush=True)
rng = cmd.cfg.ranges


# ── 창 모드 HUD + 카메라 (play_joy.py 에서 옮김. omni.ui 는 한글이 깨져서 영어) ─────────────────────
BTN_LB, BTN_RB, BTN_START, DPAD_X, DPAD_Y = 4, 5, 7, 6, 7
_c = (0.2, -1.4, 0.35)
cam0 = (math.atan2(_c[1], _c[0]), math.hypot(_c[0], _c[1]), _c[2])
cam = list(cam0)                                                      # [방위각 rad, 수평거리 m, 높이 m]
vcc = getattr(env, "viewport_camera_controller", None)


def apply_cam():
    if vcc is not None:
        yaw, dd, hz = cam
        vcc.update_view_location(eye=(dd * math.cos(yaw), dd * math.sin(yaw), hz), lookat=(0.0, 0.0, 0.12))


HIST = 200
hist = {k: [0.0] * HIST for k in ("pitch", "roll", "vx")}
hud, plots = None, {}
stats = dict(falls=0, jumps=0, last="-", rtf=float("nan"), mark=(0.0, None))
if not args.headless:
    try:
        import omni.kit.viewport.utility as vp_utils
        import omni.ui as ui
        _font = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
        _fst = {"font_size": 15, "color": 0xFFFFFFFF, "font": _font}
        with vp_utils.get_active_viewport_window().get_frame("wb_jump_hud"):
            with ui.HStack():
                ui.Spacer()
                with ui.VStack(width=ui.Pixel(380)):
                    ui.Spacer(height=8)
                    with ui.ZStack():
                        ui.Rectangle(style={"background_color": 0xB0000000, "border_radius": 6})
                        with ui.VStack(spacing=2):
                            ui.Spacer(height=8)
                            hud = ui.Label("", style=_fst, alignment=ui.Alignment.LEFT_TOP)
                            ui.Spacer(height=6)
                            for key, lab, col in (("pitch", "pitch", 0xFF44FFFF), ("roll", "roll", 0xFFFF8844), ("vx", "speed", 0xFF44AAFF)):
                                with ui.HStack(height=ui.Pixel(26)):
                                    ui.Spacer(width=8)
                                    ui.Label(lab, width=ui.Pixel(80), style=_fst)
                                    plots[key] = ui.Plot(ui.Type.LINE, -1.0, 1.0, *([0.0] * HIST), width=ui.Pixel(220),
                                                         height=ui.Pixel(24), style={"color": col, "background_color": 0x30FFFFFF})
                            ui.Spacer(height=8)
                    ui.Spacer()
                ui.Spacer(width=12)
        print("[HUD] 활성", flush=True)
    except Exception as e:
        print(f"[HUD] 비활성: {type(e).__name__} {e}", flush=True)


def push(key, val):
    h = hist[key]
    h.append(max(-1.0, min(1.0, val)))
    del h[0]


def pad_camera():
    """십자키 둘러보기, LB 멀리 / RB 가까이, BACK 카메라 초기화 (play_joy 와 같음)."""
    moved = False
    dx, dy = pad.axis(DPAD_X), pad.axis(DPAD_Y)
    if dx:
        cam[0] += dx * 1.2 * dt; moved = True
    if dy:
        cam[2] = min(3.0, max(0.05, cam[2] - dy * 0.6 * dt)); moved = True
    if pad.button(BTN_LB) or pad.button(BTN_RB):
        cam[1] = min(4.0, max(0.3, cam[1] * (1.0 + (0.8 if pad.button(BTN_LB) else -0.8) * dt))); moved = True
    if pad.button(BTN_BACK):
        cam[:] = list(cam0); moved = True
    if moved:
        apply_cam()


def hud_update(t, phase, vx, wz, h_cmd, wheel_pos, wx, tau, next_edge):
    """꼭 필요한 것만: 단계, 속도, 기울기, 다리, 바퀴 속도 여유, 직전 점프, 넘어짐, 창 속도."""
    d = robot.data
    g = d.projected_gravity_b[0]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(g[0])))))
    roll = math.degrees(math.asin(max(-1.0, min(1.0, float(g[1])))))
    v_now = float(d.root_com_lin_vel_b[0, 0])
    hl, hr = (float(x) * 1000 for x in cad.leg_state(robot)[0][0])
    ws = 100 * float(d.joint_vel[0, wheel_ids].abs().max()) / 18.85
    push("pitch", pitch / 30.0); push("roll", roll / 30.0); push("vx", v_now / 1.0)
    m_sim, m_wall = stats["mark"]
    now = time.time()
    if m_wall is None or now - m_wall > 1.0:
        if m_wall is not None and now > m_wall and t > m_sim:
            stats["rtf"] = (t - m_sim) / (now - m_wall)
        stats["mark"] = (t, now)
    ahead = [e for e in edges if e > wx - 0.03]
    wb = [(float(z) - R) * 1000 for z in wheel_pos[:, 2]]
    wt = [float(x) for x in d.applied_torque[0, wheel_ids] * wsign]
    wv = [float(x) for x in d.joint_vel[0, wheel_ids] * wsign]
    yr = float(d.root_ang_vel_w[0, 2])
    rtf = f"{stats['rtf']:.2f}x" if stats["rtf"] == stats["rtf"] else "-"
    lines = [
        f"PHASE  {('LIFTED' if stats.get('lift') else phase.upper()):8s}  CTRL {args.ctrl.upper()}" + (f" + RL {'ON' if args.rl_on else 'OFF'} (B)" if RL['pol'] is not None else ""),
        f"SPEED  {abs(v_now)*3.6:3.1f} km/h ({v_now:+.2f} m/s) / cmd {vx*3.6:+.1f} / max {args.vmax_kmh:.1f}" + ("  BRAKE" if stats.get("brake") else ""),
        f"YAW    {yr:+.2f} / cmd {wz:+.2f} rad/s",
        f"TILT   P {pitch:+5.1f}  R {roll:+5.1f} deg",
        f"LEGS   L {hl:3.0f}  R {hr:3.0f}  idle {args.idle_h*1000:3.0f} mm",
        f"WHL z  L {wb[0]:3.0f}  R {wb[1]:3.0f} mm",
        f"HIP    L {float(tau[0]):+5.2f}  R {float(tau[1]):+5.2f} Nm",
        f"WHL    L {wt[0]:+5.2f}  R {wt[1]:+5.2f} Nm",
        f"WHL w  L {wv[0]:+5.1f}  R {wv[1]:+5.1f} rad/s {ws:3.0f}%" + (" SLOW" if ws > 100 * args.speed_guard else ""),
        f"EDGE   {(ahead[0] - wx) * 1000:4.0f} mm" if ahead else "EDGE   -",
        f"JUMP   {stats['last']}",
        f"COUNT  jumps {stats['jumps']}  falls {stats['falls']}  sim {rtf}",
        "",
        (f"LQR    qth {args.lqr_qth:.0f} qv {args.lqr_qv:.0f}  VMC {args.vmc_kp:.0f}/{args.vmc_kd:.1f}  roll {args.roll_kp:.0f}/{args.roll_ki:.0f}"
         if args.ctrl == "lqr" else f"LEG    kp {args.leg_kp:.0f} kd {args.leg_kd:.1f}"),
        f"JMP    trig {args.trigger:.2f} lean {args.retract_lean:.0f} air {args.air_pitch:+.0f} land {args.land_kp:.0f}/{args.h_land*1000:.0f}",
    ]
    hud.text = "\n".join(lines)
    for kk, pl in plots.items():
        pl.set_data(*hist[kk])

RESTART = ("rl_policy", "dr_mass", "dr_com_cm", "dr_motor", "dr_seed", "imu_tilt_bias_deg", "render_hz", "physics_hz", "color_body", "color_legs", "color_wheels", "spawn_z", "obstacle", "step_h", "length", "tread", "edge", "hip", "hip_w0", "ridge_h", "ridge_base", "ridge_period",
           "ridge_lane", "ridge_len", "cad_file", "cad_unit", "gen_type", "gen_h", "gen_len", "gen_seed", "env_name")   # 장면을 다시 만들어야 해서 재시작 필요
CLI_KEYS = {k for k in TUNE if f"--{k}" in sys.argv}                        # 명령줄로 준 값은 파일보다 우선


def reload_tune():
    """이 파일의 TUNE 블록을 다시 읽어 args 에 반영한다 (창을 띄운 채 파일만 저장하면 다음 시도부터 적용)."""
    try:
        src = open(os.path.abspath(__file__), encoding="utf-8").read()
        i = src.index("TUNE = dict(")
        ns = {}
        exec(src[i:src.index("\n)\n", i) + 3], ns)
    except Exception as e:                                             # 저장 도중이거나 문법 오류면 이전 값 유지
        print(f"[튜닝] TUNE 읽기 실패, 이전 값 유지: {e}", flush=True)
        return
    for k, v in ns["TUNE"].items():
        if k in CLI_KEYS or k in ("rl_on",) or getattr(args, k, None) == v:   # rl_on 은 패드 B 로만 (파일값은 시작할 때만)
            continue
        if k in RESTART:
            print(f"[튜닝] {k} = {v} 는 재시작해야 적용된다 (지금 {getattr(args, k)})", flush=True)
            continue
        print(f"[튜닝] {k}: {getattr(args, k)} -> {v}", flush=True)
        setattr(args, k, v)
        if k.startswith("lqr_") and args.ctrl == "lqr":
            build_lqr()
    cmd.cfg.auto_height = cmd.cfg.default_height = args.idle_h
    kp0[:], kd0[:] = args.leg_kp, args.leg_kd


dt = env.step_dt
LOOP = not args.headless and not args.record and pad is None          # 창으로 볼 때는 계속 반복
rec = None
if args.record:
    import datetime
    import imageio.v2 as imageio
    os.makedirs(args.record, exist_ok=True)
    rec_path = os.path.join(args.record, f"ascento_{args.mode}_{args.obstacle}_{int(args.step_h*1000)}mm_{datetime.datetime.now():%H%M%S}.mp4")
    rec = imageio.get_writer(rec_path, fps=50, codec="libx264", quality=8, macro_block_size=8)
    rec_every = max(1, round(1.0 / (dt * 50)))


gov = dict(vf=0.0, i=0.0, ref=0.0)
lift = dict(on=False, t_un=0.0, t_ld=0.0)                              # 들림 (손으로 들기, 공중 스폰) 상태
_m_tot = float(robot.root_physx_view.get_masses()[0].sum())
from isaaclab.utils.math import quat_apply  # noqa: E402


def hand_wrench(t, grab):
    """가상 손: 몸통을 스프링-댐퍼로 잡아 들어 올리고 옆으로 기울인다 (월드 힘 + 몸체 토크)."""
    d = robot.data
    p, v = d.root_pos_w[0], d.root_lin_vel_w[0]
    s_ = min(1.0, (t - grab["t0"]) / 0.5)                           # 0.5 s 동안 들어 올림
    tgt = torch.tensor([grab["x"], grab["y"], grab["z"] + s_ * args.hand_lift], device=dev)
    F = 400.0 * (tgt - p) - 40.0 * v
    F[2] += _m_tot * 9.81
    g = d.projected_gravity_b[0]
    phi, th = -math.asin(max(-1.0, min(1.0, float(g[1])))), math.asin(max(-1.0, min(1.0, float(g[0]))))
    w = d.root_ang_vel_b[0]
    tb = torch.tensor([20.0 * (math.radians(args.hand_roll) * s_ - phi) - 2.0 * float(w[0]),
                       20.0 * (0.0 - th) - 2.0 * float(w[1]), -1.0 * float(w[2])], device=dev)
    tw = quat_apply(d.root_quat_w[0:1], tb[None])[0]
    robot.set_external_force_and_torque(F[None, None], tw[None, None], body_ids=[0], is_global=True)
import collections  # noqa: E402
RING = collections.deque(maxlen=2000)                                 # 패드 모드: 최근 10 s 기록 (다리 진동 원인 찾기)
OSC = dict(last=-99.0, x_prev=False)


def dump_ring(reason):
    import datetime
    path = os.path.expanduser(f"~/pv_out/climb/pad_{reason}_{datetime.datetime.now():%H%M%S}.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    json.dump(dict(reason=reason, tune={k: getattr(args, k) for k in TUNE}, log=list(RING)), open(path, "w"), ensure_ascii=False)
    print(f"[기록 저장] {reason}: {path}", flush=True)


def check_leg_osc(t):
    """최근 1 s 에 좌우 다리 차가 8 mm 넘게 3 번 이상 번갈아 뒤집히면 저장 (5 s 에 한 번)."""
    if t - OSC["last"] < 5.0 or len(RING) < 200:
        return
    d = [r["hl"] - r["hr"] for r in list(RING)[-200:]]
    m = sum(d) / len(d)
    flips, sgn = 0, 0
    for v in d:
        z = v - m
        if abs(z) > 0.008:
            s_ = 1 if z > 0 else -1
            if sgn and s_ != sgn:
                flips += 1
            sgn = s_
    if flips >= 3:
        OSC["last"] = t
        print(f"[다리 진동 감지] 최근 1 s 에 좌우 {flips} 번 뒤집힘", flush=True)
        dump_ring("leg_osc")


ACTQ = collections.deque(maxlen=16)
from wheeled_biped_isaaclab.tasks.balance import rewards as _crew  # noqa: E402
RL = dict(pol=None, prev=torch.zeros(4, device=dev), b_prev=False)
if args.ctrl == "lqr" and args.rl_policy:
    _rp = args.rl_policy if os.path.isabs(args.rl_policy) else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", args.rl_policy)
    if os.path.isfile(_rp):
        from policy_io import load_actor
        RL["pol"] = load_actor(_rp, dev)
        print(f"[RL] 잔차 정책 {os.path.basename(os.path.dirname(_rp))}/{os.path.basename(_rp)} ({'켬' if args.rl_on else '끔'}, 패드 B 로 전환)", flush=True)
    else:
        print(f"[RL] 정책 파일 없음: {_rp}", flush=True)
EST = dict(th=0.0, thd=0.0, v=0.0)


def true_v():
    d = robot.data
    psi = yaw_of(d.root_quat_w[0]); vw = d.body_lin_vel_w[0, wheel_bodies].mean(0).tolist()
    return vw[0] * math.cos(psi) + vw[1] * math.sin(psi)


def true_th():
    """실제 질량으로 구한 진자각 (모델 오차 포함 실제값)."""
    d = robot.data
    mt = robot.root_physx_view.get_masses()[0].to(dev)
    c = (d.body_com_pos_w[0, _nonwheel] * mt[_nonwheel, None]).sum(0) / mt[_nonwheel].sum()
    r = (c - d.body_pos_w[0, wheel_bodies].mean(0)).tolist(); psi = yaw_of(d.root_quat_w[0])
    return math.atan2(r[0] * math.cos(psi) + r[1] * math.sin(psi), r[2])
PROF = dict(ctrl=0.0, step=0.0, hud=0.0, sleep=0.0, n=0)


def episode():
    soft_legs(False)
    wheel_term.cfg.torque_scale = scale0
    obs, _ = env.reset()
    global wheel_y0
    wheel_y0 = [round(float(v), 3) for v in robot.data.body_pos_w[0, wheel_bodies, 1]]
    cmd.cfg.auto_height = cmd.cfg.default_height = args.idle_h
    kp0[:], kd0[:] = args.leg_kp, args.leg_kd
    phase, t_phase, next_edge, h0 = "drive", 0.0, 0, args.idle_h
    lvl = 0.0                                                  # 수평 유지 루프의 좌우 다리 길이 차 명령 hL - hR [m]
    x_err = 0.0                                                # LQR 진행거리 오차 (명령 속도 적분 대비)
    lift.update(on=False, t_un=0.0, t_ld=0.0)
    gov.update(vf=0.0, i=0.0, ref=0.0)                         # 속도 제한·브레이크 상태
    roll_pi.reset()
    if args.ctrl == "lqr":
        _I = build_lqr()
        if not getattr(build_lqr, "_shown", False):
            build_lqr._shown = True
            print(f"[LQR] m {_m_pend:.2f} kg, I {_I:.4f} kg·m², 바퀴 {_m_w:.3f} kg / {_I_w:.4f} kg·m², "
                  f"K(l=0.25) {_np.round(lqr.gain(0.25), 2).tolist()}", flush=True)
    tipped = False
    log, jumps = [], []
    fell, fell_t = False, None
    y_prev, back_prev, h_cmd, wall0 = True, True, args.idle_h, None
    for k in (itertools.count() if pad is not None else range(int(args.seconds / dt))):
        if not app.is_running():
            return None
        LEG_TARGET = {"retract": H_MIN, "extract": H_MAX, "fly": H_MIN, "descend": args.h_land, "land": args.h_land}
        t = k * dt
        _t0 = time.perf_counter()
        d = robot.data
        wheel_pos = d.body_pos_w[0, wheel_bodies]                 # (2, 3)
        wx = float(wheel_pos[:, 0].mean())
        h_now = cad.leg_state(robot)[0][0]                         # (2,) 다리 길이
        tau = d.applied_torque[0, leg_ids] * hip_sign
        # --- 상태머신 --------------------------------------------------------------------------------
        vx, wz, y_edge = (0.0 if args.stop_at is not None and t >= args.stop_at else args.v), 0.0, False
        if args.wz is not None:
            wz = args.wz
        elif args.heading_kp > 0:
            wz = max(-1.0, min(1.0, -args.heading_kp * yaw_of(robot.data.root_quat_w[0])))
        if pad is not None:                                        # 패드: 속도·회전·높이는 사람이, 점프는 Y
            pad.poll()
            _vm = args.vmax_kmh / 3.6
            vx, wz, dh, estop, reset_h = J.command_from_gamepad(pad, (-_vm, _vm), (-args.wz_max, args.wz_max))
            trig = max(J.trigger(pad, J.AXIS_RT), J.trigger(pad, J.lt_axis(pad)))
            y, back = pad.button(BTN_Y) or trig > 0.5, pad.button(BTN_START)   # 점프: Y, RT, LT 어느 것이든
            bb = pad.button(1)
            if bb and not RL["b_prev"]:                            # B = 잔차 RL 켜기/끄기
                args.rl_on = not args.rl_on
                RL["prev"] = torch.zeros(4, device=dev)
                print(f"[RL] {'켬' if args.rl_on else '끔'}", flush=True)
            RL["b_prev"] = bb
            y_edge, y_prev = y and not y_prev, y
            if k % int(2.0 / dt) == 0 and k > 0 and phase == "drive":
                reload_tune()                                      # 패드: 2 s 마다 파일 TUNE 반영 (점프 중엔 안 함)
                soft_legs(False)
            if back and not back_prev:
                print("[처음으로]", flush=True)
                return "restart"
            back_prev = back
            pad_camera()
            if wall0 is None:
                wall0 = time.time() - t
            _sl = max(0.0, wall0 + t - time.time())
            time.sleep(_sl)                                        # 실제 시간에 맞춘다
            PROF["sleep"] += _sl
        if args.mode == "jump":
            tp = t - t_phase
            auto_go = pad is None and next_edge < len(edges) and wx >= edges[next_edge] - args.trigger
            v_fwd = float(robot.data.root_com_lin_vel_b[0, 0])
            w_now = float(robot.data.root_ang_vel_w[0, 2])
            if phase == "drive" and y_edge and (v_fwd < args.jump_min_v or abs(w_now) > args.jump_max_wz):  # 제자리·후진·회전 중 점프 막기
                y_edge = False
                why = f"{v_fwd*3.6:+.1f} km/h" if v_fwd < args.jump_min_v else f"turning {w_now:+.1f} rad/s"
                stats["last"] = f"blocked: {why}"
                print(f"[점프 막음] {why} — 앞으로 {args.jump_min_v*3.6:.1f} km/h 이상, 회전 {args.jump_max_wz:.1f} rad/s 이하에서만", flush=True)
            if phase == "drive" and (auto_go or y_edge):
                if pad is not None:
                    reload_tune()                                  # 점프마다 파일의 TUNE 을 다시 읽는다
                    next_edge = next((i for i, e in enumerate(edges) if e > wx - 0.03), len(edges))
                phase, t_phase, h0 = "retract", t, float(h_now.mean())
                jumps.append(dict(edge=next_edge, t_trigger=round(t, 3), x_trigger=round(wx, 3)))
            elif phase == "retract" and tp >= args.t_retract:
                phase, t_phase = "extract", t
                wheel_term.cfg.torque_scale = scale0
                if args.air_ctrl and args.pd_from == "extract" and args.extract_wheels == "pd":
                    wheel_term.cfg.torque_scale = args.air_tau
            elif phase == "extract" and (float(h_now.min()) >= H_MAX - 0.008 or tp >= 0.25):
                phase, t_phase = "fly", t
                jumps[-1]["t_takeoff"], jumps[-1]["x_takeoff"] = round(t, 3), round(wx, 3)
                if args.air_ctrl:
                    wheel_term.cfg.torque_scale = args.air_tau
            elif phase == "fly" and tp >= args.t_tuck:
                phase, t_phase = "descend", t
                soft_legs(True)
            elif phase == "descend" and ((tp > 0.04 and float(tau.abs().max()) > args.contact_tau)
                                         or t - jumps[-1]["t_takeoff"] >= args.t_fly_max):
                jumps[-1].update(t_land=round(t, 3), x_land=round(wx, 3), contact="torque" if tp > 0.04 and
                                 float(tau.abs().max()) > args.contact_tau else "timeout",
                                 wheel_bottom_mm=round((float(wheel_pos[:, 2].min()) - R) * 1000, 1))
                phase, t_phase = "land", t
                wheel_term.cfg.torque_scale = scale0
                stats["jumps"] += 1
                j, e = jumps[-1], (edges[next_edge] if next_edge < len(edges) else None)
                rel = (lambda x: f"{1000*(x-e):+.0f} mm") if e is not None else (lambda x: f"x {x:.3f} m")
                stats["last"] = f"#{len(jumps)} takeoff {rel(j['x_takeoff'])}  land {rel(j['x_land'])}" if e is not None else f"#{len(jumps)} land bottom {j['wheel_bottom_mm']:.0f} mm"
                if pad is not None:
                    print(f"[창 속도] 실시간 대비 {stats['rtf']:.2f} 배", flush=True)
                    print(f"[점프 {len(jumps)}] 모서리 {next_edge+1 if e is not None else '-'}  이륙 {rel(j['x_takeoff'])}  "
                          f"착지 {rel(j['x_land'])} ({j['contact']})  착지 때 바퀴 바닥 {j['wheel_bottom_mm']} mm", flush=True)
            elif phase == "land" and tp >= args.land_s:
                soft_legs(False)
                phase, t_phase = "drive", t
                next_edge += 1
            if pad is None and next_edge >= len(edges) and wx >= edges[-1] + 0.25:
                vx = 0.0                                           # 마지막 모서리 0.25 m 지나 멈춘다 (모서리에 서면 굴러 내려옴)
        elif args.mode == "stance":
            vx = 0.0
            phase = "stand" if t < 1.0 else ("lean" if t < 2.0 else "lift")
        cmd.set(torch.tensor([vx], device=dev), torch.tensor([wz], device=dev), torch.tensor([h_cmd], device=dev),
                mode=torch.tensor([1.0], device=dev))                  # 항상 자동 (h 는 무시되고 idle_h)
        act = policy(obs["policy"]).clone()
        h_ref = float(cmd.command[0, 2])
        if args.ctrl == "lqr":                                     # 정책 대신 LQR (바퀴) + VMC (다리)
            act = torch.zeros_like(act)
            sense()
            th, thd, l_p, v_now, wz_now = lqr_state()
            RF["r"] += (1.0 - math.exp(-2 * math.pi * args.roll_rate_lpf_hz * dt)) * (-SENSE["gx"] - RF["r"]) if args.roll_rate_lpf_hz > 0 else (-SENSE["gx"] - RF["r"])
            EST.update(th=th, thd=thd, v=v_now)
            ffF = 0.0
            if phase in ("drive", "retract", "extract", "land"):
                th_ref = math.radians(args.retract_lean) if phase == "retract" else 0.0
                vm = args.vmax_kmh / 3.6
                gov["vf"] += (1.0 - math.exp(-2 * math.pi * args.speed_lpf_hz * dt)) * (v_now - gov["vf"])
                e = abs(gov["vf"]) - vm                            # 최고 속도 초과분 (필터한 속도)
                gov["i"] = max(0.0, min(vm, gov["i"] + args.brake_ki * e * dt))
                v_lim = max(0.0, vm - (args.brake_kp * max(e, 0.0) + gov["i"]))
                stats["brake"] = v_lim < vm - 0.02
                tgt = max(-v_lim, min(v_lim, vx))
                gov["ref"] += max(-args.accel_max * dt, min(args.accel_max * dt, tgt - gov["ref"]))
                v_ref = gov["ref"]
                if abs(vx) > v_lim + 1e-3:
                    x_err = 0.0                                    # 제한 중에는 뒤처진 거리를 쌓지 않는다 (풀릴 때 튀지 않게)
                ww = float(robot.data.joint_vel[0, wheel_ids].abs().max()) / 18.85
                if ww > args.speed_guard and args.speed_guard < 1.0:     # 푸시백: 지금 속도보다 낮은 목표 -> LQR 이 뒤로 젖혀 감속
                    cut = min(1.0, (ww - args.speed_guard) / (1.0 - args.speed_guard))
                    v_ref = v_now * (1.0 - 0.6 * cut) if v_now * vx >= 0 else vx
                    x_err = 0.0
                x_err = max(-0.3, min(0.3, x_err + (v_now - v_ref) * dt))
                tau_w = lqr.torque(l_p, x_err, v_now - v_ref, th - th_ref, thd)
                # 바깥 바퀴 = (|v| + 0.094 |wz|) / R <= 한계 x wheel_margin -> 회전 한계
                if args.turn_limit:
                    wz_lim = max(0.5, (args.wheel_margin * 18.85 * R - abs(v_now)) / 0.094)
                    wz = max(-wz_lim, min(wz_lim, wz))
                tau_y = args.yaw_kd * (wz - wz_now)
                wheel_term.cfg.torque_scale = args.wheel_tau_max
                act[0, 2] = max(-1.0, min(1.0, (0.5 * tau_w - tau_y) / args.wheel_tau_max))
                act[0, 3] = max(-1.0, min(1.0, (0.5 * tau_w + tau_y) / args.wheel_tau_max))
            unl = float(tau.max()) < args.contact_tau_min            # 두 다리 모두 무하중 (펴는 쪽 토크가 없음)
            sf = float(torch.norm(d.body_lin_acc_w[0, 0] + torch.tensor([0.0, 0.0, 9.81], device=dev))) / 9.81
            ldd = float(tau.max()) >= args.contact_tau_min and sf > args.land_sf_min   # 한쪽이라도 펴는 쪽으로 받침 (+ 자유낙하 아님)
            #   (공중에서 다리가 움직이는 토크를 접지로 오판하지 않게 — 스폰 낙하 0.14 s 에 오판했음)
            if phase == "drive" and not lift["on"]:
                lift["t_un"] = lift["t_un"] + dt if unl else 0.0
                if lift["t_un"] >= args.lift_detect_s:
                    lift.update(on=True, t_ld=0.0)
                    if pad is not None or args.hand:
                        print(f"[들림] t {t:.2f}", flush=True)
            elif phase == "drive" and lift["on"]:
                lift["t_ld"] = lift["t_ld"] + dt if ldd else 0.0
                if lift["t_ld"] >= args.land_detect_s:
                    lift.update(on=False, t_un=0.0)
                    x_err = 0.0; gov.update(i=0.0, ref=v_now, vf=v_now); roll_pi.reset(0.0)
                    if pad is not None or args.hand:
                        print(f"[내려놓음] t {t:.2f} — 균형 재개", flush=True)
            stats["lift"] = lift["on"]
            if phase == "drive" and lift["on"]:                      # 들린 동안: 균형 끔, 바퀴만 멈춤, 다리 IDLE, 적분 비움
                wv = d.joint_vel[0, wheel_ids] * wsign
                act[0, 2:] = (-args.lift_wheel_kd * wv / args.wheel_tau_max).clamp(-1.0, 1.0)
                act[0, 0] = act[0, 1] = (args.idle_h - h_ref) / 0.12
                legs_act.stiffness[:] = args.vmc_kp; legs_act.damping[:] = args.vmc_kd
                roll_pi.reset(0.0); x_err = 0.0; gov.update(i=0.0, ref=0.0, vf=0.0)
            elif phase == "drive":
                # 명령값으로 (측정 회전 속도는 흔들려서 기울기 목표까지 흔든다)
                roll_ref = max(-math.radians(20), min(math.radians(20), math.atan(args.turn_lean * gov["ref"] * wz / 9.81)))
                rl = SENSE["roll"] - roll_ref                   # 회전 중 안쪽으로 기울이기
                airborne = float(tau.min()) < args.contact_tau_min      # 부호 있음: + = 다리를 펴며 몸을 받침 (뜨면 0 이나 반대 부호)
                dlt = roll_pi(rl, dt, args.roll_kp, args.roll_ki, args.level_max,
                              freeze=airborne or abs(math.degrees(rl)) > args.roll_freeze_deg, leak=args.roll_leak,
                              rate=RF["r"], kd=args.roll_kd)
                tl = min(H_MAX, max(H_MIN, args.idle_h + 0.5 * dlt)); tr = min(H_MAX, max(H_MIN, args.idle_h - 0.5 * dlt))
                act[0, 0], act[0, 1] = (tl - h_ref) / 0.12, (tr - h_ref) / 0.12
                legs_act.stiffness[:] = args.vmc_kp; legs_act.damping[:] = args.vmc_kd
                ffF = 0.5 * _m_pend * 9.81
                if RL["pol"] is not None and args.rl_on:                  # 잔차 RL: 학습 환경과 같은 관측 37 -> 보정 4
                    tau_b = act[0, 2:4] * args.wheel_tau_max
                    h_b = torch.tensor([tl, tr], device=dev)
                    ctrl_o = torch.cat([
                        torch.tensor([th * 5.0, v_now, v_ref, x_err * 5.0, dlt * 10.0, 0.0, 0.0, 0.0], device=dev),
                        tau_b / args.wheel_tau_max, (h_b - args.idle_h) * 10.0, (h_now - args.idle_h) * 10.0,
                        tau / 5.0, RL["prev"]])
                    o = torch.cat([d.projected_gravity_b[0], d.root_ang_vel_b[0], cmd.command[0], cad.leg_vel(env)[0],
                                   cad.wheel_vel(env)[0], _crew.imu_specific_force_g(env)[0], ctrl_o]).float()[None]
                    res = RL["pol"](o)[0].clamp(-1.0, 1.0)
                    RL["prev"] = res.clone()
                    act[0, 2:4] = (act[0, 2:4] + res[:2] * 1.5 / args.wheel_tau_max).clamp(-1.0, 1.0)
                    hh = (h_b + res[2:] * 0.02).clamp(H_MIN, H_MAX)
                    act[0, :2] = (hh - h_ref) / 0.12
            elif phase == "land":
                ffF = 0.5 * _m_pend * 9.81
            else:
                roll_pi.reset(float(h_now[0] - h_now[1]))
            M = robot.data.joint_pos[:, leg_ids]
            robot.set_joint_effort_target(hip_sign * ffF * cad.dh_from_M(M).to(torch.float32), joint_ids=leg_ids)
        if args.mode == "jump" and phase in LEG_TARGET:
            tgt = LEG_TARGET[phase]
            if phase == "retract":
                tgt = h0 + (H_MIN - h0) * min(1.0, (t - t_phase) / (0.75 * args.t_retract))
            act[0, :2] = (tgt - h_ref) / 0.12
            pd_phases = ("fly", "descend") if args.pd_from == "fly" or args.extract_wheels != "pd" else ("extract", "fly", "descend")
            if phase == "extract" and args.extract_wheels == "free":
                act[0, 2:] = 0.0
            if phase == "retract" and args.retract_lean > 0 and args.ctrl == "policy":
                wheel_term.cfg.torque_scale = args.air_tau
                pitch = math.asin(max(-1.0, min(1.0, float(d.projected_gravity_b[0, 0]))))
                u = args.air_kp * (pitch - math.radians(args.retract_lean)) + args.air_kd * float(d.root_ang_vel_b[0, 1])
                act[0, 2:] = max(-1.0, min(1.0, u / args.air_tau))
            if phase in pd_phases:                                 # 지면 균형 제어 끔
                if not args.air_ctrl:
                    act[0, 2:] = 0.0
                else:                                              # 바퀴 +y 토크 u -> 몸통에 -u. 뒤로 젖혀지면(pitch<0) 바퀴를 감속
                    if args.ctrl == "lqr":                         # 센서 추정값 (sense() 는 LQR 블록에서 이번 스텝에 불림)
                        pitch, gy = SENSE["pitch"], SENSE["gy"]
                    else:
                        pitch, gy = math.asin(max(-1.0, min(1.0, float(d.projected_gravity_b[0, 0])))), float(d.root_ang_vel_b[0, 1])
                    ref = args.extract_pitch if phase == "extract" else args.air_pitch
                    u = args.air_kp * (pitch - math.radians(ref)) + args.air_kd * gy
                    act[0, 2:] = max(-1.0, min(1.0, u / args.air_tau))   # 두 바퀴 같은 값 (+y 규약, 좌우 부호는 액션 항이 처리)
        if args.level_rate > 0 and phase == "drive" and args.mode != "stance" and args.ctrl == "policy":
            # 로그의 roll = asin(g_y) > 0 이면 왼쪽(+y)이 낮다 -> 왼다리를 늘린다
            rl = math.asin(max(-1.0, min(1.0, float(d.projected_gravity_b[0, 1]))))
            lvl = max(-args.level_max, min(args.level_max, lvl + dt * args.level_rate * 0.198 * math.sin(rl)))
            mean = h_ref + 0.12 * 0.5 * (float(act[0, 0].clamp(-1, 1)) + float(act[0, 1].clamp(-1, 1)))
            tl = min(H_MAX, max(H_MIN, mean + 0.5 * lvl)); tr = min(H_MAX, max(H_MIN, mean - 0.5 * lvl))
            act[0, 0], act[0, 1] = (tl - h_ref) / 0.12, (tr - h_ref) / 0.12
        elif phase != "drive":
            lvl = float(h_now[0] - h_now[1])                       # 점프 중에는 실제 차를 따라가 두었다가 이어받는다
        if args.mode == "stance" and phase != "stand":
            s = min(1.0, (t - 1.0) / 1.0)
            hl, hr = H_CRUISE + 0.045 * s, H_CRUISE - 0.045 * s     # 좌우 길이차 9 cm -> 약 24 deg 기울기
            if phase == "lift":
                hl = H_MIN                                         # 왼쪽 다리를 최대로 접어 바퀴를 든다
            act[0, 0], act[0, 1] = (hl - h_ref) / 0.12, (hr - h_ref) / 0.12
        if args.hand:                                              # 손 들기 시험
            if args.hand[0] <= t < args.hand[1]:
                if "grab" not in stats:
                    p0 = robot.data.root_pos_w[0]
                    stats["grab"] = dict(t0=t, x=float(p0[0]), y=float(p0[1]), z=float(p0[2]))
                hand_wrench(t, stats["grab"])
            elif t >= args.hand[1] and stats.get("grab") is not None:
                robot.set_external_force_and_torque(torch.zeros(1, 1, 3, device=dev), torch.zeros(1, 1, 3, device=dev), body_ids=[0])
                stats["grab"] = None
        if args.delay_ms > 0 or args.jitter_ms > 0:                 # 제어 지연 (+ 가끔 한 주기 더)
            ACTQ.append(act.clone())
            dly = int(round(args.delay_ms / 5.0)) + (1 if _rng.random() < args.jitter_ms / 5.0 else 0)
            act = ACTQ[max(0, len(ACTQ) - 1 - dly)]
        _t1 = time.perf_counter()
        obs, _, term, _, _ = env.step(act)
        _t2 = time.perf_counter()
        PROF["ctrl"] += _t1 - _t0; PROF["step"] += _t2 - _t1; PROF["n"] += 1
        if bool(term[0]) and not fell:
            fell, fell_t = True, t
            stats["falls"] += 1
            if pad is not None:
                print(f"[넘어짐] 단계 {phase} — 처음으로", flush=True)
                return "restart"
            if LOOP:
                break                                              # 창 모드: 넘어지면 바로 다음 시도
        if k % int(10.0 / dt) == 0 and k > 0 and not args.headless:
            n_ = max(PROF["n"], 1)
            print(f"[창 속도] 실시간 대비 {stats['rtf']:.2f} 배 (시뮬 {t:.0f} s) | 스텝당 ms: env.step {1e3*PROF['step']/n_:.2f}"
                  f" (물리 {cfg.decimation} x + 렌더), 제어·상태머신 {1e3*PROF['ctrl']/n_:.2f}, HUD {1e3*PROF['hud']/n_:.2f},"
                  f" 대기 {1e3*PROF['sleep']/n_:.2f}  (실시간 = {1e3*dt:.1f} ms)", flush=True)
            for _k in PROF:
                PROF[_k] = 0
        if pad is not None:                                        # 리셋 없이 넘어짐만 센다 (60 deg 넘게 기울면 1 회)
            gg = robot.data.projected_gravity_b[0]
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(gg[2])))))
            if tilt > 60 and not tipped:
                tipped = True; stats["falls"] += 1
                print(f"[넘어짐] 단계 {phase} (리셋 안 함, START = 처음으로)", flush=True)
                dump_ring("fall")                                  # 넘어지기 직전 10 s 자동 저장 (원인 분석용)
            elif tilt < 20:
                tipped = False
        if pad is not None:                                        # 최근 기록 + 진동 감지 + X = 수동 저장
            RING.append(dict(t=round(t, 3), phase=phase, wx=round(wx, 3), vx=round(vx, 3), wz=round(wz, 3),
                             v=round(float(d.root_com_lin_vel_b[0, 0]), 3), wzr=round(float(d.root_ang_vel_w[0, 2]), 3),
                             roll=round(math.degrees(math.asin(max(-1.0, min(1.0, float(d.projected_gravity_b[0, 1]))))), 2),
                             pitch=round(math.degrees(math.asin(max(-1.0, min(1.0, float(d.projected_gravity_b[0, 0]))))), 2),
                             hl=round(float(h_now[0]), 4), hr=round(float(h_now[1]), 4),
                             zl=round(float(wheel_pos[0, 2]) - R, 4), zr=round(float(wheel_pos[1, 2]) - R, 4),
                             lvl=round(roll_pi.i, 4), tl=round(float(tau[0]), 2), tr=round(float(tau[1]), 2)))
            check_leg_osc(t)
            xb = pad.button(2)
            if xb and not OSC["x_prev"]:
                dump_ring("manual")
            OSC["x_prev"] = xb
        if hud is not None and k % 5 == 0:
            _th = time.perf_counter()
            hud_update(t, phase, vx, wz, h_cmd, wheel_pos, wx, tau, next_edge)
            PROF["hud"] += time.perf_counter() - _th
        # --- 기록 ------------------------------------------------------------------------------------
        if pad is not None:
            continue
        g = d.projected_gravity_b[0]
        mvel = d.joint_vel[0, leg_ids] * hip_sign
        log.append(dict(t=t, phase=phase, wx=wx, wz_l=float(wheel_pos[0, 2]), wz_r=float(wheel_pos[1, 2]),
                        bz=float(d.root_com_pos_w[0, 2]), bx=float(d.root_com_pos_w[0, 0]),
                        yaw=math.degrees(yaw_of(d.root_quat_w[0])), wzr=float(d.root_ang_vel_w[0, 2]), pitch=math.degrees(math.asin(max(-1, min(1, float(g[0]))))),
                        roll=math.degrees(math.asin(max(-1, min(1, float(g[1]))))),
                        hl=float(h_now[0]), hr=float(h_now[1]), tau_l=float(tau[0]), tau_r=float(tau[1]),
                        w_l=float(mvel[0]), w_r=float(mvel[1]),
                        tw_l=float(d.applied_torque[0, wheel_ids[0]] * wsign[0]), vw_l=float(d.joint_vel[0, wheel_ids[0]] * wsign[0]),
                        th_est=EST["th"], thd_est=EST["thd"], v_est=EST["v"], v_true=true_v(), th_true=true_th()))
        if rec is not None and k % rec_every == 0:
            rec.append_data(np.asarray(env.render())[..., :3])
    return judge(log, jumps, fell, fell_t)


def judge(L, jumps, fell, fell_t):
    tau_max = max(max(abs(r["tau_l"]), abs(r["tau_r"])) for r in L)
    w_max = max(max(abs(r["w_l"]), abs(r["w_r"])) for r in L)
    res = dict(mode=args.mode, tune={k: getattr(args, k) for k in TUNE}, wsign=wsign.tolist(), wheel_y=wheel_y0,
               fell=fell, fell_t=fell_t, tau_hip_max_nm=tau_max, hip_speed_max_rad_s=w_max,
               current_max_a_kt060=tau_max / 0.5994, current_max_a_kt081=tau_max / 0.81, jumps=jumps)
    if args.obstacle == "ridges" and args.mode in ("jump", "none"):
        res["success"] = not fell
        res["roll_range_deg"] = [float(min(r["roll"] for r in L)), float(max(r["roll"] for r in L))]
        res["pitch_range_deg"] = [float(min(r["pitch"] for r in L)), float(max(r["pitch"] for r in L))]
        res["leg_h_range_mm"] = [float(min(min(r["hl"], r["hr"]) for r in L) * 1000), float(max(max(r["hl"], r["hr"]) for r in L) * 1000)]
        res["body_z_range_mm"] = [float(min(r["bz"] for r in L) * 1000), float(max(r["bz"] for r in L) * 1000)]
        res["wheel_y"] = wheel_y0
        res["final_x"] = L[-1]["wx"]
    elif args.mode in ("jump", "none"):
        last = L[-int(1.0 / dt):]
        res["wheel_bottom_max_mm"] = float(max(min(r["wz_l"], r["wz_r"]) - R for r in L) * 1000)
        res["final_wheel_bottom_mm"] = float(np.mean([min(r["wz_l"], r["wz_r"]) - R for r in last]) * 1000)
        res["success"] = (not fell) and res["final_wheel_bottom_mm"] > (top - 0.01) * 1000
        air = [r for r in L if r["phase"] in ("extract", "fly", "descend", "land")]
        res["pitch_range_in_jump_deg"] = [float(min(r["pitch"] for r in air)), float(max(r["pitch"] for r in air))] if air else None
        res["final_x"] = L[-1]["wx"]
        res["yaw_max_deg"] = float(max(abs(r["yaw"]) for r in L))
    else:
        lift = [r for r in L if r["phase"] == "lift"]
        ok = [r for r in lift if abs(r["roll"]) < 40]
        res["left_wheel_lift_mm"] = float(max(r["wz_l"] - R for r in lift) * 1000) if lift else None
        res["stance_time_s"] = (fell_t - 2.0) if fell else (len(ok) * dt)
    return res, L


def summary(n, res):
    """창 모드용 한 줄: 성공, 넘어짐, 점프마다 이륙·착지 x (모서리 기준 mm), 착지 순간 바퀴 바닥 높이, 공중 pitch 범위."""
    js = "  ".join(f"[{j['edge']+1}] 이륙 {1000*(j.get('x_takeoff', float('nan'))-edges[j['edge']]):+.0f} 착지 "
                   f"{1000*(j.get('x_land', float('nan'))-edges[j['edge']]):+.0f} mm ({j.get('contact', '-')}, 바닥 {j.get('wheel_bottom_mm', '-')} mm)"
                   for j in res["jumps"])
    pr = res.get("pitch_range_in_jump_deg")
    print(f"#{n} {'성공' if res.get('success') else '실패'}"
          + (f"  넘어짐 {res['fell_t']:.2f} s" if res["fell"] else "")
          + (f"  pitch {pr[0]:.0f}~{pr[1]:.0f} deg" if pr else "") + f"  yaw 최대 {res.get('yaw_max_deg', 0):.0f} deg  {js}", flush=True)


n = 0
with torch.inference_mode():
    while True:
        n += 1
        if LOOP and n > 1:
            reload_tune()
        r = episode()
        if r is None:
            break
        if r == "restart":
            continue
        res, L = r
        if LOOP:
            summary(n, res)
            continue
        break

if not LOOP and r is not None:
    if rec is not None:
        rec.close()
    print("\n" + json.dumps(res, ensure_ascii=False, indent=1), flush=True)
    summary(1, res)
    out = args.out or os.path.expanduser(f"~/pv_out/climb/ascento_{args.mode}_{args.obstacle}_{int(args.step_h*1000)}mm.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(dict(result=res, log=L[:: max(1, len(L) // 2000)]), open(out, "w"), ensure_ascii=False)
    print(f"저장: {out}" + (f"\n영상: {rec_path}" if rec is not None else ""), flush=True)
env.close()
app.close()
