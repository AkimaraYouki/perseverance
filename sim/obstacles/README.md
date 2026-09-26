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
