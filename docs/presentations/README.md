# 발표 자료 (2026년 11월 학회)

2026년 11월 25~28일 추계 학술대회 두 곳에서 발표한 포스터와 영상입니다. 같은 Gen2 두 바퀴 휠-레그 로봇으로 주제를 나눴습니다.

```text
docs/presentations
├── 2026-11_KSNVE_poster_Park       한국소음진동공학회 추계 (여수 EXPO), 포스터, 발표 박수호
│   ├── KSNVE_poster_Park.pdf       Control Performance Evaluation of a Wheeled Bipedal Robot
│   └── KSNVE_poster_Park.pptx      편집용 원본
├── 2026-11_KSMPE_capstone_Mun      한국기계가공학회 추계 (제주), 캡스톤 디자인, 발표 문인주
│   ├── KSMPE_poster_Mun.pdf        Locomotion Control of a Two Wheel-Legged Robot
│   ├── KSMPE_poster_Mun.pptx       편집용 원본
│   ├── KSMPE_capstone_slides_Mun.pptx  캡스톤 발표 슬라이드 (19장, 5분, 영상 3개 자동 재생, 노트에 대본)
│   └── videos                      슬라이드에 넣은 영상 (따로 재생용)
│       ├── slide10_curb_grass_wave_road.mp4     연석·잔디 주행, 대회 웨이브 길 (15초)
│       ├── slide13_auto_step_jump_phases.mp4    연구실 10cm 자동 턱 (점프 6단계 표시, 느리게) + 대회 8cm 2단 (1.8배속) (23초)
│       └── slide15_maze_person_following.mp4   대회 미로에서 지정한 한 사람 추종 (20초)
└── videos                          두 포스터 공용
    ├── poster_highlight_40s.mp4    포스터 옆 태블릿 반복 재생용 (1920x1080, 소리 없음, 40초)
    └── fourbar_leg_stroke.mp4      4절 링크 다리 왕복 애니메이션 (현재 링크 길이, 12초)
```

하이라이트 영상은 주행, 험지(실기 웨이브 길 | Isaac 험지), 10cm 턱 점프 시뮬레이션과 실기 비교(단계 시각을 맞춰 나란히, 느리게), 대회 2단 자동 턱, 미로 사람 추종, 4절 링크 왕복 순서입니다.

## 받는 법

PDF, PPTX, MP4는 Git LFS로 올렸습니다. 젯슨 clone이 무거워지지 않도록 `.lfsconfig`에서 이 폴더를 기본 다운로드에서 빼 두었기 때문에, `git pull`만 하면 실제 파일 대신 작은 포인터 파일만 받아집니다. 실제 파일이 필요하면 아래처럼 받으면 됩니다.

```bash
git lfs pull -I "docs/presentations/**"
```

GitHub 웹에서 파일을 눌러 바로 내려받아도 됩니다.
