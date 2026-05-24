# drive_sim — project3 sim-to-real 시뮬레이터

`project3.py` / `project3.ino` 를 **그대로** 돌려보는 2D 시뮬레이터.
의사결정 코드를 다시 구현하지 않고, **실제 `project3.py` 를 import 해서
`choose_cmd`(DWA) · `lidar_points_to_xy` · 모든 상수를 그대로 호출**한다.
→ 시뮬레이터에서 잘 도는 코드는 라즈베리파이에서 동일하게 돈다(센서/모터 모델 한도 내에서).

## 실행

```bash
cd project3/drive_sim
pip install pygame numpy        # 최초 1회
python3 sim.py
```

## 조작

| 동작 | 방법 |
|---|---|
| 도구 선택 | 우측 상단 버튼 1줄: Erase / Obstacle / Robot, 2줄: Red / Yellow / Blue |
| 장애물 배치 | Obstacle 선택 후 클릭=중심, 누른 채 드래그=회전각, 떼면 배치 (발자국 = `obstacle w` / `obstacle depth`, 수직 높이 = `obstacle z`) |
| 색종이 배치 | Red/Yellow/Blue 선택 후 클릭 (크기 = `patch size` 슬라이더) |
| 로봇 배치 | Robot 선택 후 클릭=위치, 그대로 드래그=초기 방향 |
| 삭제 | Erase 선택 후 객체 근처 클릭 |
| 주행 시작/일시정지 | `Start` 버튼 또는 `Space` |
| 리셋(시작 자세로) | `Reset` 또는 `R` |
| 전부 지우기 | `Clear` |
| 배치 저장/불러오기 | `Save` / `Load` (`layout.json`) |
| 타깃 색 수동 변경 | `1`=RED `2`=YELLOW `3`=BLUE |
| 아레나 확대/축소 | 왼쪽 시뮬레이션 영역 위에서 마우스 휠 |
| 아레나 화면 이동 | 왼쪽 시뮬레이션 영역에서 마우스 휠 버튼(가운데 버튼) 드래그 |
| 파라미터 조정 | 우측 슬라이더 (패널 위에서 마우스 휠로 스크롤) |
| 파라미터 설명 | 슬라이더 이름 위에 마우스를 올리면 짧은 설명 표시 |

`project3.py` 나 `project3.ino` 를 저장하면 **자동으로 다시 로드**된다("RELOADED code").

`Save`로 생성되는 `layout.json`은 단순 배치값뿐 아니라 전체 4m x 4m 맵 크기,
중앙 2m x 2m 시험 영역, 로봇 자세/높이, 회전 직사각형 장애물의 중심/발자국/수직 높이/각도/꼭짓점,
색상 목표 위치, 카메라 유효 거리 구간, 센서 스냅샷, 현재 판정 상태, 그리고 실패 가능 원인을 함께 담는다.
따라서 Codex/Claude Code 같은 코드 에이전트가 파일 하나만 봐도 “어떤 상황에서 왜 통과하지
못했는지”를 추적할 수 있다.

## 화면 표시

- **반투명 빨강 링** = `COLLISION_DIST`, **반투명 노랑 링** = `SLOW_DIST` (슬라이더로 즉시 변함)
- **반투명 하늘색 직사각형** = 로봇 위쪽 카메라가 내려다볼 수 있는 바닥 영역.
  전방 `cam_min`부터 `cam_max` 사이의 직사각형 영역만 보며, 기본 로봇/카메라 높이는 0.18m, 기본 `cam_max`는 1.0m다.
- 어두운 초록 사각형 = 장애물/색상 영역이 배치되는 **2m x 2m 시험 영역**.
  전체 표시 맵은 4m x 4m이며 외곽 테두리는 벽으로 취급하지 않는다.
- 주황 점 = 실제 `lidar_points_to_xy` 가 만든 전방 장애물 점
- 하늘색 곡선 = DWA가 선택한 (v,w) 예측 궤적 `predict_xy`
- 자홍 선 = 카메라가 본 타깃 색 방위(β)
- 하늘색 외곽선 = 카메라가 보는 직사각형 바닥 영역
- 흰 점 2개 = 좌/우 바퀴 바닥 접점("양 바퀴 영역 진입" 판정용)

좌상단 텔레메트리: 상태 · 타깃/순서 진행 · `see` · `gb`(목표방위) · 명령 `v,w` ·
`clr`(여유거리) · `BLOCKED` · **collisions** · FINISHED.

## sim-to-real: 무엇이 "실제 코드"인가

| 요소 | 처리 |
|---|---|
| DWA 결정 `choose_cmd` | **실제 project3.py 함수 그대로 호출** |
| 라이다 점 변환/필터 `lidar_points_to_xy` | **실제 함수** (합성 원시 스캔을 입력) |
| 궤적 예측 `predict_xy` | **실제 함수** |
| 모든 DWA/로봇/카메라 상수 | **실제 project3.py 값** (수정 시 reload) |
| 명령 직렬화 `send_vw` | **실제 함수** ("V..\n" 포맷 그대로) |
| `project3.ino` 상수 | 정규식 파싱(WHEEL_R/BASE, 한계, **CMD_TIMEOUT**, deadband …) |
| (v,w)→바퀴 목표, 클램프, 정지, 타임아웃 | **.ino 로직 그대로 포팅** (`arduino_model.py`) |

## 의도적으로 추상화/근사한 부분 (한계)

1. **모터+PID 과도응답** → 1차 지연(`wheel tau`) + 가속 한계로 근사.
   실제 PID 게인까지 재현하려면 검증된 모터 전기 파라미터가 필요하다.
   `wheel tau` 를 실제 로봇의 속도 상승시간에 맞추면 거의 일치한다.
2. **카메라**는 이미지 렌더링 대신 **기하 기반**으로 `(visible, β, area, cy)` 생성.
   단, `HFOV_DEG` · `BEARING_SIGN` · `MIN_AREA` 등 **실제 상수를 사용**하므로
   부호 규약과 추적 방향은 그대로 검증된다. (HSV 색 임계값 자체는 합성색이라
   항상 통과 → 실차 조명/색 튜닝은 시뮬레이터로 검증 불가.)
   로봇 상단 카메라가 바닥을 내려다본다고 보고 전방 `cam_min`~`cam_max` 직사각형 구간만 보이며,
   장애물 수직 높이(`obstacle z`, 기본 0.23m)가 카메라-바닥 시야선보다 높으면 가림으로 처리한다.
   `area`/`cy` 는 비례 근사값으로, 정밀 거리(미터)는 Step 8 기하 캘리브 이후 갱신.
3. **라이다**는 합성 스캔이라 캘리브 상수(`ANGLE_OFFSET_DEG` 등)의 *효과*(필터·전방
   선별·서브샘플)는 검증되지만, 상수의 *실차 정확도*는 검증 대상이 아니다.
4. **main() 의 제어 루프 글루**는 `sim.py` 의 `brain_tick()` 이 미러링한다.
   (Step 11에서 main 에 상태기계가 들어가면 이 부분도 함께 갱신 필요.)

## 파일 구성

- `code_bridge.py` — 실제 project3.py 로드(serial 스텁) + .ino 상수 파싱 + hot-reload
- `arduino_model.py` — project3.ino 포팅(명령 파싱·바퀴 목표·한계·타임아웃 + 휠 지연 모델)
- `world.py` — 차동구동 물리 + 충돌/영역 심판(양 바퀴 1초 정지 = 통과)
- `sensors.py` — 라이다 레이캐스팅(→실제 변환) + 카메라 기하 인식
- `sim.py` — pygame GUI + 파라미터 패널 + main() 글루 미러
- `layout.json` — Save/Load 배치(자동 생성)

## 셀프테스트(헤드리스)

```bash
SDL_VIDEODRIVER=dummy python3 sim.py --selftest   # 30프레임 렌더 후 종료
```
