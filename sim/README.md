# sim — 데스크톱 쪽 설계·시뮬 자산

데스크톱(Ubuntu, RTX 5080, Isaac Sim 5.1 / Isaac Lab 2.3.2)에서 만든 것들이다.
**URDF 는 아직 확정 전이다.** 아래 값은 전부 URDF/CAD 에서 계산되므로 URDF 가 바뀌면 다시 돌린다.

| 경로 | 내용 |
|---|---|
| `model/leg_map.py` | 다리 모터각 θ ↔ 다리 높이 h (4절링크). CAD 실측 치수, MuJoCo 닫힌고리와 0.3 mm 이내 일치. **실기에서 그대로 쓸 것** |
| `model/make_simple_urdf.py` | CAD URDF(onshape-to-robot export) → 학습용 단순화 URDF. 입력 CAD export 는 저장소에 없다 |
| `model/robot_simple.urdf` | 단순화 모델. 4절링크를 직선관절(고관절~바퀴중심)로 치환. 전진 +x |
| `control/vmc_lqr_design.py` | **VMC + LQR 게인 설계** (Liu & Wang 2024 구조). 다리 높이별 LQR 게인 스케줄과 VMC 고관절 토크 |
| `control/gains.yaml` | 위 스크립트 출력. 손으로 고치지 말 것 |
| `isaaclab/` | Isaac Lab RL 프로젝트 소스 (체크포인트 제외) |
| `isaaclab/scripts/pd_balance_check.py` | PD 게인 스윕 — 모델이 세워지는지 판정 |
| `isaaclab/scripts/lqr_balance_check.py` | `gains.yaml` 을 Isaac 에서 검증 |

## 부호 규약 (전 파일 공통)

전진 +x, 가로 +y, 위 +z. 바퀴 회전축 +y. 넘어지는 축 = pitch(y).
θ = +y 축 회전 = **윗부분이 앞(+x)으로 기울면 +**. θ ≈ projected_gravity_b.x, θ̇ = ang_vel_b.y.
양(+)의 바퀴 토크 → 로봇 전진, 몸체는 뒤로 반작용.
