# 시뮬 장애물 (CAD)

`pv jump --joystick --obstacle cad` 가 `obstacle.stl` 을 불러온다 (다른 파일: `--cad_file 경로`, 또는 climb_test.py 의 TUNE `cad_file`).

좌표 규약 — CAD 에서 이대로 그리면 변환 없이 맞는다:

- 원점 = 출발할 때 두 바퀴 축 가운데 **바로 아래 바닥**
- +X = 달리는 방향, +Z = 위, 바닥면 Z = 0 (장애물 밑면을 Z = 0 에)
- 바퀴는 Y = ±99 mm 선을 지난다 (바퀴 간 198 mm)
- 단위 mm (`cad_unit` 0.001). m 로 뽑았으면 `--cad_unit 1`
- 형식 STL / OBJ / FBX. 충돌은 삼각형 메시 그대로 (정적), 마찰 1.0
- 출발점에서 1 m 정도는 비워 두면 속도를 붙일 수 있다

파일을 고쳐 저장하면 다음 실행 때 다시 변환한다 (결과 캐시 `~/pv_out/cad_obstacles/`). 장면이라 창을 다시 띄워야 한다 (`pv stop` → `pv jump ...`).

## 대회 코스 (2026-09-26, Onshape Part Studio 2)

- `obs1_ridges.stl` ㅅㅅㅅ: 높이 80 mm, 밑변·주기 500 mm, 폭 1 m 를 좌우 반씩 반 주기(250 mm) 엇갈림, 길이 5.25 m
- `obs2_pyramid.stl` ㅗ: 1 m | 1 m | 1 m, 한 단 90 mm (합 180)
- Onshape 파트는 원점 중심 + 달리는 방향 +Y 라서 `make_course.py` 가 돌려서 잇는다 -> `course.stl`
  (출발 -> 1 m -> 삼각형길 X 1.0~6.25 m -> 1.5 m -> ㅗ X 7.75~10.75 m)

      pv jump --joystick --obstacle cad --cad_file ~/perseverance/sim/obstacles/course.stl --idle_h 0.1365

r3 균형 정책 0.5 m/s 직진: 삼각형길 30 cm 들어가서 넘어짐 (8 cm 는 학습 범위 밖 — 학습 지형 ridges 는 1~6 cm).
