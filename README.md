# Introduction_to_Robot_Engineering_Design

로봇공학설계입문 수업에서 진행한 자율주행 로봇 프로젝트 모음입니다.
라즈베리파이(상위 제어) + 아두이노(모터 제어)로 구성된 차동 구동 로봇을 사용하며,
두 보드는 UART 시리얼로 통신합니다.

---

## Project 1 — 거리 기반 제어

라즈베리파이에서 이동 거리(m)를 한 번 입력하면, 로봇이 그 거리만큼 정확히 직진한 뒤 정지하는 프로젝트

[![YouTube](https://img.shields.io/badge/YouTube-Project1-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/D7GIgPEmSmw?si=WfYOUOvX33R6n5a1)

### 동작 방식

- **라즈베리파이** (`project1/rp-arduino_uart.py`)
  사용자가 입력한 거리 값을 `D <거리>` 형식의 명령으로 아두이노에 전송
- **아두이노** (`project1/project1.ino`)
  엔코더 카운트를 거리로 환산해 목표 카운트에 도달할 때까지 주행

### 구현 방법

- **PID 제어**
  - 위치 PID: 목표 카운트까지 남은 오차를 줄여 정확한 거리에서 정지
  - 좌우 동기화 PID: 두 바퀴의 카운트 차이를 보정해 직진성 유지
- **가감속 램프**: 출발 시 서서히 가속하고, 목표 지점 근처에서 감속해 오버슈트를 줄임
- **거리별 오차 보정**: 이동 거리에 따른 좌우 편차(`encoderdiff`)와 보정 계수를 모델링하여
  실측 오차를 줄임

---

## Project 2 — LiDAR 기반 장애물 회피

RPLiDAR C1으로 주변 장애물을 인식해, 목표지점까지 충돌 없이 자율 주행하는 프로젝트

[![YouTube](https://img.shields.io/badge/YouTube-Project2-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/il0o1bk1G74?si=mJ4EAW9nrT4-ygAv)

### 동작 방식

- **라즈베리파이** (`project2/fgm_drive_final_*.py`)
  RPLidar 스캔을 받아 진행 방향을 계산하고, 선속도·각속도 명령
  (`V <v>,<w>`)을 아두이노로 전송
- **아두이노** (`project2/project2.ino`)
  받은 (v, w)를 좌우 바퀴 목표 각속도로 변환하고, 바퀴별 PID + 피드포워드로 추종

### 구현 방법

**Follow the Gap Method (FGM)** 사용

- LiDAR 스캔을 각도별 거리 배열로 변환하고 평활화
- 가장 가까운 장애물 주변을 **안전 버블**로 부풀려 막음
- 통과 가능한 연속 구간(**Gap**)을 찾고, 너무 좁은 Gap은 제거
- 정면 선호 / 목표 방향 / 직전 방향 연속성 / 개방 폭 / 누적 회전량 페널티 등을
  점수화해 최적의 목표 방향을 선택
- 목표 각도와 전방·좌우 거리에 따라 속도를 가변 제어

### 추가 로직

- **Recovery 모드**: 안전한 Gap이 없거나 막혔을 때 제자리 회전으로 새 경로를 탐색하고, 전방이 트이면 다시 FGM 주행으로 복귀
- **좌우 벽 보정**: 좁은 통로에서 조향각·속도 상한을 낮춰 벽 충돌 방지