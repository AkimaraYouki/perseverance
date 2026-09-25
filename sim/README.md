# sim — 데스크톱 쪽 설계·시뮬 자산

데스크톱(Ubuntu, RTX 5080, Isaac Sim 5.1 / Isaac Lab 2.3.2)에서 만든 것들이다.
**URDF 는 아직 확정 전이다.** 아래 값은 전부 URDF/CAD 에서 계산되므로 URDF 가 바뀌면 다시 돌린다.

| 경로 | 내용 |
|---|---|
| `model/leg_map.py` | 다리 모터각 θ ↔ 다리 높이 h (4절링크). CAD 실측 치수, MuJoCo 닫힌고리와 0.3 mm 이내 일치. **실기에서 그대로 쓸 것** |
| `model/fix_urdf.py` | onshape-to-robot export 후처리: 바퀴 continuous, 모터 한계, 회전자 관성, 에셋 이름, 루트 → `base_link`, 관절 한계 |
| `model/joint_limits_from_urdf.py` | export URDF 의 폐루프를 직접 풀어 관절 한계 → `joint_limits.json`. **CAD 를 다시 뽑으면 반드시 다시 돌린다** |
| `model/make_simple_urdf.py` | 후처리된 CAD URDF → 단순화 URDF. 원점 = 두 고관절 중점, 센서 프레임 4 개 포함 |
| `model/robot_simple.urdf` | 단순화 모델. 4절링크를 직선관절(고관절~바퀴중심)로 치환. 전진 +x. **현재 CAD = export (5)** |
| `control/vmc_lqr_design.py` | **VMC + LQR 게인 설계** (Liu & Wang 2024 구조). 다리 높이별 LQR 게인 스케줄과 VMC 고관절 토크 |
| `control/gains.yaml` | 위 스크립트 출력. 손으로 고치지 말 것 |
| `isaaclab/` | Isaac Lab RL 프로젝트 소스 (체크포인트 제외) |
| `isaaclab/scripts/pd_balance_check.py` | PD 게인 스윕 — 모델이 세워지는지 판정 |
| `isaaclab/scripts/lqr_balance_check.py` | `gains.yaml` 을 Isaac 에서 검증 |
| `isaaclab/JUMP_REFERENCES.md` | 점프 선행 연구 정리와 학습 방향 (학습 에이전트용 지침) |

## CAD → 모델 파이프라인

CAD export(zip, 메시 포함 약 10 MB)는 저장소에 넣지 않는다. 데스크톱에서:
```
python3 fix_urdf.py "export (N).zip"                       # -> export_(N)_fixed/robot.urdf
python3 model/joint_limits_from_urdf.py export_(N)_fixed/robot.urdf   # 관절 한계 재계산
python3 fix_urdf.py export_(N)_fixed/robot.urdf              # 새 한계 적용 (멱등)
python3 model/make_simple_urdf.py --src export_(N)_fixed/robot.urdf
python3 sim/control/vmc_lqr_design.py                        # 게인 재계산
```

## 부호 규약 (전 파일 공통)

전진 +x, 가로 +y(**왼쪽**, ROS REP-103), 위 +z. 이름 `l_*`/`L_*` = +y 쪽. 바퀴 회전축 +y. 넘어지는 축 = pitch(y).
θ = +y 축 회전 = **윗부분이 앞(+x)으로 기울면 +**. θ ≈ projected_gravity_b.x, θ̇ = ang_vel_b.y.
양(+)의 바퀴 토크 → 로봇 전진, 몸체는 뒤로 반작용.
