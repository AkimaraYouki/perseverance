# CAD 4절링크 폐루프를 Isaac Sim 으로 — 2026-09-25

Isaac Sim 5.1/6.0 `rig_closed_loop_structures` 방식 (폐루프 관절을 Exclude From Articulation).

## 파이프라인 (Onshape export 부터 자동)
```
python3 sim/model/fix_urdf.py "export (N).zip"
python3 sim/model/joint_limits_from_urdf.py export_(N)_fixed/robot.urdf ; python3 sim/model/fix_urdf.py export_(N)_fixed/robot.urdf
python3 sim/model/make_loop_urdf.py export_(N)_fixed/robot.urdf --out export_(N)_fixed/robot_loop.urdf
isaaclab.sh -p sim/isaaclab/scripts/convert_urdf.py --urdf export_(N)_fixed/robot_loop.urdf --out usd_loop
isaaclab.sh -p sim/model/add_loop_joints.py usd_loop/robot_simple.usd export_(N)_fixed/loop_closure.json
```
- `make_loop_urdf.py`: closing_* 프레임에서 폐루프 점 P 와 축을 뽑고(조립 잔차 0.000 mm), closing 링크 제거,
  충돌체 단순화(CAD 는 보이는 메시 93 개 전부가 충돌체라 관절부에서 겹친다 → 바퀴 원통 + 몸체 상자).
- `add_loop_joints.py`: 몸체-로커 사이 구면 관절, excludeFromArticulation = True.

## 검증 (`sim/isaaclab/scripts/loop_check.py`, 몸체 고정, 다리 모터 M 스윕)
- 폐루프 잔차 0.00 mm
- 다리 길이 vs leg_map: 최대 0.04 mm
- 관절각 vs `asd.py` calculate_kinematics (CAD 치수, 조립분기 -1): I 0.11 deg, K 0.18 deg, 다리 0.11 mm
  (`sim/model/verify_loop_with_asd.py`)
- 중력 아래 모터 추종: **explicit PD 에서 0.1~0.2 deg (9 Nm 이내)**, 다리 122.8 ~ 242.2 mm.

## 밟은 것
1. 수동 관절(I, K) 에 관절 한계를 걸면 폐루프와 겹쳐 기구가 잠긴다 → 폐루프 모델에선 한계를 모터 M 에만.
2. **PhysX implicit 드라이브는 Exclude 관절의 구속력을 못 본다** → 90 Nm 를 줘도 20 deg 뒤처짐.
   IdealPD(explicit) 로 바꾸면 해결. 솔버 반복 16->64 로는 안 바뀐다. 회전/구면 관절 차이도 없다.
3. 설계 끝(theta 97.5 deg)을 넘어 한계 여유(+2.9 deg)까지 가면 전달각 155 deg 로 사점에 가까워져
   다리가 펴진 채 안 돌아온다 → **실기 다리 모터 한계는 여유 없이 M 52.5 deg (theta 97.5 deg)** 에서 끊을 것.

## 아직
- 몸체 풀고 바퀴 접지해서 서기, 학습 속도 (단순화 모델 대비). RL 은 당분간 단순화(직선관절) 모델로.
