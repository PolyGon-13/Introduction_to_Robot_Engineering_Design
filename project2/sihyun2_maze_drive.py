#!/usr/bin/env python3
# 위 shebang은 리눅스에서 이 파일을 직접 실행할 때 Python 3로 실행하라는 의미입니다.

# =========================
# 1. 필요한 라이브러리 불러오기
# =========================
import serial          # 라이다와 아두이노를 시리얼 통신으로 연결하기 위한 라이브러리
import time            # sleep, 현재 시간 측정 등에 사용
import math            # sin, cos, atan2, radians 등 수학 함수 사용
import threading       # 라이다 데이터를 메인 루프와 별도 스레드에서 계속 읽기 위해 사용
import numpy as np     # 라이다 점 좌표 계산과 후보 경로 평가를 빠르게 처리하기 위해 사용


# =========================
# 2. 시리얼 포트 설정
# =========================
LIDAR_PORT = "/dev/ttyUSB0"   # RPLidar가 연결된 리눅스 시리얼 포트
LIDAR_BAUD = 460800           # RPLidar C1 통신 속도
ARDU_PORT  = "/dev/ttyS0"     # 아두이노가 연결된 시리얼 포트
ARDU_BAUD  = 9600             # 아두이노 통신 속도


# =========================
# 3. 라이다 캘리브레이션 값
# =========================
# 라이다 장착 방향이나 실제 측정 오차를 보정하기 위한 값입니다.
ANGLE_OFFSET_DEG = +1.54      # 라이다 각도 보정값. 측정 각도에 +1.54도를 더함
DIST_OFFSET_MM   = 0.0        # 거리 보정값. 측정 거리 mm에 더함
LIDAR_ANGLE_SIGN = -1.0       # 라이다 각도 방향을 반대로 뒤집기 위한 부호

# 이 코드에서 사용하는 로봇 기준 좌표계:
# x축: 로봇의 정면 방향이 +x
# y축: 로봇의 왼쪽 방향이 +y
# 각도: +각도는 좌회전 방향, -각도는 우회전 방향
# 즉 +각도/+y = 좌측, -각도/-y = 우측입니다.


# =========================
# 4. 라이다 전처리 파라미터
# =========================
MIN_LIDAR_DIST_M = 0.05       # 5cm보다 가까운 라이다 점은 노이즈 가능성이 높아 무시
MAX_LIDAR_DIST_M = 2.5        # 2.5m보다 먼 점은 현재 회피 판단에 필요 없으므로 무시
MIN_QUALITY = 1               # 라이다 품질값이 1보다 낮으면 무시
MIN_X_FOR_PLANNING = 0.10     # x가 0.10m 이상인 점만 회피 계획에 사용. 즉 로봇 바로 앞 10cm 이내 점은 제거
MAX_EVAL_POINTS = 720         # 평가에 사용할 최대 라이다 점 개수. 너무 많으면 계산량이 커짐
SCAN_HOLD_S = 0.30            # 라이다 스캔이 잠깐 끊겨도 0.30초 이내면 마지막 명령 유지
LOOP_DT_S = 0.05              # 메인 제어 루프 주기. 0.05초 = 20Hz 정도


# =========================
# 5. 로컬 플래너 기본 파라미터
# =========================
BASE_V = 0.18                 # 기본 전진 속도 m/s. 이 코드는 v는 거의 고정하고 w만 선택하는 구조

# 후보 각속도 목록입니다.
# w < 0: 우회전, w = 0: 직진, w > 0: 좌회전
# 이 후보들 중 가장 점수가 높은 w를 선택합니다.
W_CANDIDATES = [-0.90, -0.70, -0.50, -0.35, -0.20, -0.10, 0.0,
                0.10, 0.20, 0.35, 0.50, 0.70, 0.90]

PREDICT_TIME = 1.20           # 후보 명령을 넣었을 때 1.2초 뒤까지의 경로를 예측
PREDICT_DT = 0.10             # 예측 경로 계산 간격. 0.1초마다 위치를 계산
ROBOT_RADIUS = 0.16           # 로봇을 원으로 근사했을 때 반지름 16cm
SAFETY_MARGIN = 0.00          # 충돌 판정에 추가할 안전 여유. 현재는 0
COLLISION_DIST = ROBOT_RADIUS + SAFETY_MARGIN  # 장애물과 이 거리보다 가까우면 충돌 위험으로 판단
CLEARANCE_CAP = 1.0           # clearance 점수 계산 시 최대 1m까지만 점수에 반영
FRONT_CORRIDOR_HALF = COLLISION_DIST    # 정면 통로 반폭. |y|가 이 값보다 작으면 정면 영역으로 봄

FRONT_SECTOR_HALF_DEG = 30.0
FRONT_SECTOR_HALF_RAD = math.radians(FRONT_SECTOR_HALF_DEG)

ACTIVE_FRONT_DIST = 0.60      # 정면 장애물 위험을 판단할 최대 거리. x가 0.6m 이내이면 적극 회피 대상
SIDE_NEAR_DIST = COLLISION_DIST + 0.20         # 측면이 이 거리보다 가까우면 감점
W_CMD_RATE_LIMIT = 0.30       # 일반 상황에서 한 루프당 w 변화량 제한. 급격한 좌우 흔들림 방지
W_CMD_RATE_LIMIT_URGENT = 0.40 # 위급 상황에서 한 루프당 w 변화량 제한. 일반보다 조금 더 빠르게 틀 수 있음
URGENT_FRONT_DIST = 0.30      # 정면 장애물이 0.3m보다 가까우면 위급 모드로 판단


# =========================
# 6. 좌우 비대칭 보상 파라미터
# =========================
# 정면이 막혔을 때 좌측/우측 중 어느 방향이 더 열려 있는지 판단하기 위한 섹터입니다.
# 예: 좌측 30~60도 평균거리가 우측보다 크면 좌측이 더 비어 있다고 판단합니다.
SIDE_SECTOR_MIN_DEG = 45.0    # 좌우 섹터 시작 각도. 정면 0도를 기준으로 45도부터 봄
SIDE_SECTOR_MAX_DEG = 90.0    # 좌우 섹터 끝 각도. 90도까지 봄
SIDE_CAP_M = 1.5              # 평균 거리 계산 시 너무 먼 점은 1.5m로 제한하여 평균 왜곡 방지
ASYMMETRY_GATE = 0.3          # front_factor가 이 값 이상일 때만 좌우 비대칭 보상 활성화


# =========================
# 7. 목표 지점 관련 파라미터
# =========================
# 이 코드는 장애물 회피뿐 아니라 목표점 x=3.0, y=0.0 쪽으로 가려는 항도 포함합니다.
GOAL_X_M = 3.0                # 목표 x좌표. 시작점 기준 앞으로 3m
GOAL_Y_M = 0.0                # 목표 y좌표. 중앙선 y=0 유지
GOAL_TOL_M = 0.15             # 목표점까지 15cm 이내로 들어오면 도착으로 판단
GOAL_HEADING_WEIGHT = 1.0     # 목표 방향과 헤딩 차이에 대한 감점 가중치
GOAL_LATERAL_WEIGHT = 1.2     # 목표 y축에서 벗어난 정도에 대한 감점 가중치
GOAL_DISTANCE_WEIGHT = 0.4    # 목표까지 가까워지는 정도에 대한 보상 가중치
TURN_SOFT_LIMIT_RAD = math.radians(70.0) # 로봇 방향이 70도 이상 틀어지면 추가 감점 시작
TURN_HARD_LIMIT_RAD = math.radians(88.0) # 88도 이상 틀어지면 큰 감점. 거의 옆을 보는 상태 방지
TURN_LIMIT_WEIGHT = 28.0      # soft limit 초과에 대한 감점 가중치
TURN_GROWTH_WEIGHT = 12.0     # 이미 많이 틀어진 상태에서 더 틀어지는 것에 대한 감점


# =========================
# 8. 후보 경로 점수 계산 가중치
# =========================
# 아래 값들은 evaluate_candidate()에서 후보 w의 점수를 계산할 때 사용됩니다.
clearance_weight = 3.0        # 정면 clearance가 클수록 주는 보상
collision_weight = 100.0      # 충돌 위험일 때 큰 감점
side_clearance_weight = 0.6   # 측면 clearance 보상
side_near_weight = 8.0        # 측면 근접 시 감점
side_collision_weight = 18.0  # 측면 충돌 위험 감점
forward_weight = 1.0          # 직진에 가까운 w를 선호하는 기본 보상
far_forward_weight = 2.2      # 전방이 안전할 때 직진 선호를 더 키우는 보상
turn_weight = 0.25            # 회전을 싫어하는 기본 감점
far_turn_weight = 0.55        # 전방이 안전할 때 회전을 더 싫어하도록 추가 감점
smooth_weight = 0.5           # 직전 w와 많이 달라지는 명령 감점. 좌우 요동 방지
front_approach_weight = 5.0   # 정면 장애물에 가까워지는 경로 감점
asymmetry_weight = 3.0        # 좌우 비대칭 보상 강도. 열린 방향으로 돌도록 유도


# =========================
# 9. 로봇의 현재 추정 위치
# =========================
# 엔코더 기반 실제 위치가 아니라, 명령한 v,w를 적분해서 대략 추정하는 위치입니다.
robot_x = 0.0                 # 현재 로봇 x 위치 추정값
robot_y = 0.0                 # 현재 로봇 y 위치 추정값
robot_theta = 0.0             # 현재 로봇 방향 추정값. rad 단위


# ============================================================
# 10. RPLidar C1 시리얼 통신 드라이버 클래스
# ============================================================
class RPLidarC1:
    def __init__(self, port, baud):
        # 라이다 시리얼 포트를 엽니다.
        # timeout=0.1은 read()가 데이터를 못 받으면 최대 0.1초까지만 기다린다는 뜻입니다.
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # RPLidar 프로토콜 명령:
        # 0xA5 0x40 = RESET 명령
        # 라이다를 초기화한 뒤 2초 기다립니다.
        self.ser.write(bytes([0xA5, 0x40])); time.sleep(2.0)

        # RESET 과정에서 들어온 불완전한 데이터나 노이즈를 버립니다.
        self.ser.reset_input_buffer()

        # 0xA5 0x20 = SCAN 명령
        # 라이다에게 일반 스캔 모드로 측정을 시작하라고 명령합니다.
        self.ser.write(bytes([0xA5, 0x20]))

        # 라이다는 SCAN 명령에 대해 7바이트 응답 헤더를 보냅니다.
        header = self.ser.read(7)

        # 정상 헤더는 0xA5, 0x5A로 시작해야 합니다.
        # 헤더가 이상하면 포트를 닫고 에러를 발생시킵니다.
        if len(header) != 7 or header[0] != 0xA5 or header[1] != 0x5A:
            self.ser.close()
            raise RuntimeError("[LIDAR] Response Header Error")

        # latest_scan은 라이다 읽기 스레드와 메인 제어 루프가 같이 접근합니다.
        # 따라서 lock으로 동시에 접근하는 문제를 막습니다.
        self.lock = threading.Lock()
        self.latest_scan = None       # 가장 최근 한 바퀴 스캔 데이터가 저장될 변수
        self.running = True           # 스레드 실행 여부 플래그

        # 라이다 데이터를 계속 읽는 별도 스레드를 시작합니다.
        # daemon=True라서 메인 프로그램이 끝나면 이 스레드도 같이 종료됩니다.
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        # 한 바퀴 스캔 데이터를 임시 저장할 버퍼입니다.
        # buf_a: 각도 배열, buf_d: 거리 배열, buf_q: 품질 배열
        buf_a, buf_d, buf_q = [], [], []

        while self.running:
            try:
                # RPLidar 일반 스캔 데이터 포인트 하나는 5바이트입니다.
                data = self.ser.read(5)

                # 5바이트를 못 읽으면 이번 루프는 무시합니다.
                if len(data) != 5:
                    continue

                # 첫 바이트 bit0은 새 스캔 시작 플래그입니다.
                s_flag = data[0] & 0x01

                # 첫 바이트 bit1은 시작 플래그의 반전값입니다.
                s_inv = (data[0] & 0x02) >> 1

                # RPLidar 데이터 규칙상 s_flag와 s_inv는 서로 반대여야 합니다.
                # 이 조건이 안 맞으면 깨진 데이터로 보고 버립니다.
                if s_inv != (1 - s_flag):
                    continue

                # 두 번째 바이트의 bit0은 항상 1이어야 합니다.
                # 아니면 데이터 프레임이 깨진 것으로 판단합니다.
                if (data[1] & 0x01) != 1:
                    continue

                # quality: 측정 신호 품질입니다. 값이 높을수록 신뢰도가 높습니다.
                quality = data[0] >> 2

                # angle: 라이다가 측정한 각도입니다.
                # RPLidar 포맷에 맞게 두 바이트에서 angle 값을 조합하고 64로 나눠 degree로 변환합니다.
                angle = ((data[1] >> 1) | (data[2] << 7)) / 64.0

                # dist: 측정 거리입니다.
                # 두 바이트를 조합하고 4로 나눠 mm 단위 거리로 변환합니다.
                dist = (data[3] | (data[4] << 8)) / 4.0

                # s_flag == 1이면 새 회전이 시작되었다는 뜻입니다.
                # 즉 이전까지 모은 buf_a, buf_d, buf_q가 한 바퀴 스캔 데이터가 됩니다.
                # 점이 너무 적으면 잘못된 스캔일 수 있으므로 50개 초과일 때만 저장합니다.
                if s_flag == 1 and len(buf_a) > 50:
                    with self.lock:
                        self.latest_scan = (np.array(buf_a, dtype=np.float32),
                                            np.array(buf_d, dtype=np.float32),
                                            np.array(buf_q, dtype=np.float32))

                    # 다음 한 바퀴 스캔을 위해 버퍼를 비웁니다.
                    buf_a, buf_d, buf_q = [], [], []

                # 거리가 0보다 크고 품질도 0보다 큰 점만 유효 측정으로 저장합니다.
                if dist > 0 and quality > 0:
                    buf_a.append(angle)
                    buf_d.append(dist)
                    buf_q.append(quality)

            except (serial.SerialException, OSError) as e:
                # 시리얼 통신 문제가 발생하면 프로그램을 바로 죽이지 않고 1초 뒤 재시도합니다.
                print(f"[LIDAR] Serial Error: {e}, retrying in 1 second...")
                time.sleep(1.0)
                try:
                    self.ser.reset_input_buffer()
                except:
                    pass

    def get_scan(self):
        # 메인 루프에서 가장 최근 스캔 데이터를 가져올 때 사용합니다.
        # lock을 걸어 스레드가 latest_scan을 쓰는 순간과 충돌하지 않게 합니다.
        with self.lock:
            return self.latest_scan

    def close(self):
        # 라이다 스레드 종료 플래그를 내립니다.
        self.running = False

        # RPLidar STOP 명령: 0xA5 0x25
        # 실패해도 종료 과정이므로 무시합니다.
        try:
            self.ser.write(bytes([0xA5, 0x25]))
        except:
            pass

        # 약간 기다린 후 시리얼 포트를 닫습니다.
        time.sleep(0.1)
        self.ser.close()


# ============================================================
# 11. 각도 정규화 함수들
# ============================================================
def normalize_angle_deg(angle):
    # degree 각도를 -180도 ~ +180도 범위로 정규화합니다.
    # 예: 190도 -> -170도, -190도 -> +170도
    angle = (angle + 180.0) % 360.0 - 180.0
    return angle


def normalize_angle_rad(angle):
    # rad 각도를 -π ~ +π 범위로 정규화합니다.
    # atan2(sin, cos)를 쓰면 각도가 여러 바퀴 돌아도 안정적으로 정규화됩니다.
    return math.atan2(math.sin(angle), math.cos(angle))


# ============================================================
# 12. 로봇 위치 추정 관련 함수
# ============================================================
def update_pose(v, w, dt):
    # 현재 명령 속도 v,w가 dt초 동안 유지되었다고 가정하고 로봇 위치를 업데이트합니다.
    # 실제 엔코더 피드백이 아니라 명령값 기반 dead reckoning입니다.
    global robot_x, robot_y, robot_theta

    # 현재 heading 방향으로 v*dt만큼 전진했다고 보고 x,y를 업데이트합니다.
    robot_x += v * math.cos(robot_theta) * dt
    robot_y += v * math.sin(robot_theta) * dt

    # 각속도 w가 dt초 동안 적용되었으므로 heading 변화량은 w*dt입니다.
    robot_theta += w * dt

    # heading 값이 계속 커지거나 작아지는 것을 막기 위해 -π~π로 정규화합니다.
    robot_theta = normalize_angle_rad(robot_theta)


def goal_distance():
    # 현재 추정 위치에서 목표점까지의 직선거리를 계산합니다.
    dx = GOAL_X_M - robot_x
    dy = GOAL_Y_M - robot_y

    # math.hypot(dx, dy)는 sqrt(dx^2 + dy^2)와 같습니다.
    return math.hypot(dx, dy)


def goal_heading_error_from_pose(x, y, theta):
    # 특정 위치 x,y,theta에서 목표점을 바라보기 위해 필요한 heading 오차를 계산합니다.
    dx = GOAL_X_M - x
    dy = GOAL_Y_M - y

    # 목표점 방향 각도입니다.
    target_heading = math.atan2(dy, dx)

    # 목표 방향과 현재 방향의 차이입니다.
    # 결과는 -π~π로 정규화됩니다.
    return normalize_angle_rad(target_heading - theta)


def transform_local_to_global(local_x, local_y):
    # 로봇 기준 좌표(local)를 전역 좌표(global)로 변환합니다.
    # local_x, local_y는 "현재 로봇 위치와 방향을 기준으로 본 좌표"입니다.
    # robot_x, robot_y, robot_theta를 이용해 실제 전역 좌표로 바꿉니다.
    gx = robot_x + (local_x * math.cos(robot_theta) - local_y * math.sin(robot_theta))
    gy = robot_y + (local_x * math.sin(robot_theta) + local_y * math.cos(robot_theta))
    return gx, gy


# ============================================================
# 13. 라이다 스캔 데이터를 로봇 기준 x,y 점들로 변환
# ============================================================
def lidar_points_to_xy(scan):
    # scan이 없으면 빈 점 배열을 반환합니다.
    if scan is None:
        return np.empty((0, 2), dtype=np.float32)

    # scan은 RPLidarC1._loop()에서 만든 (angles, dists, qualities) 튜플입니다.
    angles, dists, qualities = scan

    # 거리 단위 변환: mm -> m
    # DIST_OFFSET_MM를 더해 거리 오차를 보정할 수 있습니다.
    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0

    # 라이다 각도 보정값을 더하고 -180~180도로 정규화합니다.
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG)

    # 라이다 장착 방향 때문에 각도 방향이 반대라면 부호를 뒤집습니다.
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    # 사용할 수 있는 라이다 점만 필터링합니다.
    # 너무 가까움, 너무 멂, 품질 낮음은 제거합니다.
    mask = ((dist_m >= MIN_LIDAR_DIST_M) &
            (dist_m <= MAX_LIDAR_DIST_M) &
            (qualities >= MIN_QUALITY))

    # 조건을 만족하는 점이 하나도 없으면 빈 배열 반환
    if not mask.any():
        return np.empty((0, 2), dtype=np.float32)

    # 필터링된 거리와 각도만 남깁니다.
    dist_m = dist_m[mask]
    angle_rad = np.deg2rad(angle_deg[mask])

    # 극좌표 r, theta를 직교좌표 x,y로 변환합니다.
    # x = r*cos(theta), y = r*sin(theta)
    # x>0은 로봇 앞쪽, y>0은 로봇 왼쪽입니다.
    x = dist_m * np.cos(angle_rad)
    y = dist_m * np.sin(angle_rad)

    # 회피 계획에는 x가 MIN_X_FOR_PLANNING 이상인 점만 사용합니다.
    # 즉 로봇 뒤쪽 또는 너무 가까운 앞쪽 점은 제거됩니다.
    # 주의: 이 값이 0.10이면 front_distance()도 최소 x=0.10 이상 점만 보게 됩니다.
    mask_xy = x >= MIN_X_FOR_PLANNING

    if not mask_xy.any():
        return np.empty((0, 2), dtype=np.float32)

    # x,y를 N행 2열 배열로 묶습니다.
    # points[:, 0]은 모든 x좌표, points[:, 1]은 모든 y좌표입니다.
    points = np.column_stack((x[mask_xy], y[mask_xy])).astype(np.float32)

    # 점이 너무 많으면 계산량을 줄이기 위해 가까운 점부터 MAX_EVAL_POINTS개만 남깁니다.
    if len(points) > MAX_EVAL_POINTS:
        # 원점에서 가까운 순서대로 정렬하기 위한 인덱스입니다.
        order = np.argsort(points[:, 0] ** 2 + points[:, 1] ** 2)
        points = points[order[:MAX_EVAL_POINTS]]

    return points


# ============================================================
# 14. 후보 속도 v,w로 미래 경로 예측
# ============================================================
def predict_trajectory(v, w):
    # PREDICT_TIME 동안 PREDICT_DT 간격으로 몇 개의 점을 예측할지 계산합니다.
    n_steps = int(PREDICT_TIME / PREDICT_DT)

    # 각 예측 시점입니다.
    # 예: 0.1, 0.2, ..., 1.2초
    ts = (np.arange(n_steps) + 1) * PREDICT_DT

    # 시작 heading을 0으로 봤을 때 각 시점의 상대 heading 변화량입니다.
    thetas = w * ts

    if abs(w) > 1e-6:
        # w가 0이 아니면 로봇은 원호를 그리며 움직입니다.
        # 차동구동 로봇의 회전 반지름 R = v / w 입니다.
        xs = (v / w) * np.sin(thetas)
        ys = (v / w) * (1.0 - np.cos(thetas))
    else:
        # w가 0이면 직진입니다.
        xs = v * ts
        ys = np.zeros(n_steps)

    # 각 step마다 [x, y, theta] 형태로 반환합니다.
    # 이 좌표는 현재 로봇 기준 local 좌표입니다.
    return np.column_stack((xs, ys, thetas)).astype(np.float32)


# ============================================================
# 15. 정면 장애물 거리 계산
# ============================================================
def front_distance(points):
    # 라이다 점이 없으면 주변에 장애물이 없다고 보고 최대 거리 반환
    if len(points) == 0:
        return MAX_LIDAR_DIST_M

    x = points[:, 0]
    y = points[:, 1]

    # 각 점의 로봇 중심 기준 거리
    dist = np.sqrt(x * x + y * y)

    # 각 점의 전방 기준 각도
    # +각도: 왼쪽, -각도: 오른쪽
    angle = np.arctan2(y, x)

    # 부채꼴 전방 영역 조건
    # 1) x > 0: 로봇 앞쪽
    # 2) dist < ACTIVE_FRONT_DIST: 반지름 거리 이내
    # 3) abs(angle) < FRONT_SECTOR_HALF_RAD: 전방 각도 범위 이내
    mask = ((x > 0.0) &
            (dist < ACTIVE_FRONT_DIST) &
            (np.abs(angle) < FRONT_SECTOR_HALF_RAD))

    if not mask.any():
        return MAX_LIDAR_DIST_M

    # 부채꼴 안에 있는 장애물 중 가장 가까운 실제 거리 반환
    return float(np.min(dist[mask]))


# ============================================================
# 16. 좌/우 섹터 평균 거리 계산
# ============================================================
def compute_side_averages(points):
    # 점이 없으면 좌우 모두 충분히 열려 있다고 보고 SIDE_CAP_M 반환
    if len(points) == 0:
        return SIDE_CAP_M, SIDE_CAP_M

    # 각 점의 원점 기준 거리 계산
    dist = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)

    # 각 점의 각도 계산
    # arctan2(y, x)이므로 +각도는 좌측, -각도는 우측입니다.
    angle_deg = np.degrees(np.arctan2(points[:, 1], points[:, 0]))

    # 너무 먼 점이 평균값을 과하게 키우지 않도록 SIDE_CAP_M으로 제한합니다.
    dist_capped = np.minimum(dist, SIDE_CAP_M)

    # 좌측 섹터: +30도 ~ +60도
    left_mask = (angle_deg >= SIDE_SECTOR_MIN_DEG) & (angle_deg <= SIDE_SECTOR_MAX_DEG)

    # 우측 섹터: -60도 ~ -30도
    right_mask = (angle_deg <= -SIDE_SECTOR_MIN_DEG) & (angle_deg >= -SIDE_SECTOR_MAX_DEG)

    # 해당 섹터에 점이 있으면 평균 거리, 없으면 SIDE_CAP_M을 사용합니다.
    left_avg = float(np.mean(dist_capped[left_mask])) if left_mask.any() else SIDE_CAP_M
    right_avg = float(np.mean(dist_capped[right_mask])) if right_mask.any() else SIDE_CAP_M

    return left_avg, right_avg


# ============================================================
# 17. 후보 경로와 장애물 사이의 거리 계산
# ============================================================
def trajectory_clearances(traj, points):
    # 라이다 점이 없으면 정면/측면/전체 모두 안전하다고 판단
    if len(points) == 0:
        return MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M

    # traj[:, :2]는 예측 경로의 각 step 위치 x,y입니다.
    traj_xy = traj[:, :2]

    # traj[:, 2]는 예측 경로의 각 step heading입니다.
    theta = traj[:, 2]

    # 모든 예측 step과 모든 라이다 점 사이의 좌표 차이를 한 번에 계산합니다.
    # points[None, :, :]는 shape이 (1, N, 2)
    # traj_xy[:, None, :]는 shape이 (steps, 1, 2)
    # 두 배열을 빼면 shape이 (steps, N, 2)가 됩니다.
    diff = points[None, :, :] - traj_xy[:, None, :]

    # 각 step 기준으로 장애물까지의 x,y 차이입니다.
    dx = diff[:, :, 0]
    dy = diff[:, :, 1]

    # 모든 step-장애물 쌍의 거리입니다.
    dist = np.sqrt(dx ** 2 + dy ** 2)

    # 각 step의 heading에 맞춰 장애물 좌표를 그 step의 로봇 기준 좌표로 회전 변환합니다.
    c = np.cos(theta)[:, None]
    s = np.sin(theta)[:, None]

    # rel_x: 해당 step에서 봤을 때 장애물이 앞/뒤 어디에 있는지
    # rel_y: 해당 step에서 봤을 때 장애물이 좌/우 어디에 있는지
    rel_x = c * dx + s * dy
    rel_y = -s * dx + c * dy

    # 정면 위험 영역에 들어오는 장애물을 찾습니다.
    rel_dist = np.sqrt(rel_x * rel_x + rel_y * rel_y)
    rel_angle = np.arctan2(rel_y, rel_x)

    front_mask = ((rel_x > 0.0) &
                  (rel_dist < ACTIVE_FRONT_DIST) &
                  (np.abs(rel_angle) < FRONT_SECTOR_HALF_RAD))

    if front_mask.any():
        # 정면 영역 안에 있는 장애물 중 가장 가까운 거리
        front_clear = float(np.min(dist[front_mask]))
    else:
        # 정면 위험 영역에 장애물이 없으면 최대 거리로 간주
        front_clear = MAX_LIDAR_DIST_M

    # front_mask가 아닌 나머지 점들을 측면/기타 점으로 봅니다.
    side_mask = ~front_mask
    if side_mask.any():
        side_clear = float(np.min(dist[side_mask]))
    else:
        side_clear = MAX_LIDAR_DIST_M

    # body_clear는 정면/측면 구분 없이 전체 장애물 중 가장 가까운 거리입니다.
    body_clear = float(np.min(dist))

    return front_clear, side_clear, body_clear


# ============================================================
# 18. 후보 명령 v,w 평가 함수 - 최적화 버전
# ============================================================
def evaluate_candidate(v, w, points, prev_w, front_dist, left_avg, right_avg):
    # 후보 v,w로 미래 경로 예측
    traj = predict_trajectory(v, w)

    # 예측 경로와 장애물 사이의 정면/측면/전체 최소 거리 계산
    front_clearance, side_clearance, body_clearance = trajectory_clearances(traj, points)

    # ------------------------------------------------------------
    # max_abs_w 캐싱
    # W_CANDIDATES는 보통 실행 중 바뀌지 않으므로 매번 max 계산하지 않음
    # 만약 W_CANDIDATES가 바뀌면 자동으로 다시 계산됨
    # ------------------------------------------------------------
    wc_tuple = tuple(W_CANDIDATES)
    cache = getattr(evaluate_candidate, "_w_cache", None)

    if cache is None or cache[0] != wc_tuple:
        max_abs_w = max(abs(wc) for wc in wc_tuple)
        evaluate_candidate._w_cache = (wc_tuple, max_abs_w)
    else:
        max_abs_w = cache[1]

    # 자주 쓰는 값 미리 계산
    abs_w = abs(w)
    abs_robot_theta = abs(robot_theta)

    # ------------------------------------------------------------
    # front_factor 계산
    # np.clip() 대신 if문 사용 → 함수 호출/넘파이 연산 감소
    # front_factor = 0이면 전방 안전
    # front_factor = 1이면 전방 위험
    # ------------------------------------------------------------
    denom = ACTIVE_FRONT_DIST - COLLISION_DIST
    if denom < 1e-6:
        denom = 1e-6

    front_factor = (ACTIVE_FRONT_DIST - front_dist) / denom

    if front_factor < 0.0:
        front_factor = 0.0
    elif front_factor > 1.0:
        front_factor = 1.0

    # 전방이 안전할수록 safe_factor가 커짐
    safe_factor = 1.0 - front_factor

    # 전방이 안전할수록 직진 선호 증가, 회전 감점 증가
    forward_w = forward_weight + safe_factor * far_forward_weight
    turn_w = turn_weight + safe_factor * far_turn_weight

    score = 0.0

    # ------------------------------------------------------------
    # clearance 보상
    # min() 대신 조건식 사용
    # ------------------------------------------------------------
    if front_clearance < CLEARANCE_CAP:
        score += clearance_weight * front_clearance
    else:
        score += clearance_weight * CLEARANCE_CAP

    if side_clearance < CLEARANCE_CAP:
        score += side_clearance_weight * side_clearance
    else:
        score += side_clearance_weight * CLEARANCE_CAP

    # ------------------------------------------------------------
    # 충돌 / 근접 감점
    # ------------------------------------------------------------
    if front_clearance < COLLISION_DIST:
        score -= collision_weight * (COLLISION_DIST - front_clearance + 1.0)

    if side_clearance < COLLISION_DIST:
        score -= side_collision_weight * (COLLISION_DIST - side_clearance + 1.0)
    elif side_clearance < SIDE_NEAR_DIST:
        score -= side_near_weight * (SIDE_NEAR_DIST - side_clearance)

    # ------------------------------------------------------------
    # 직진 선호, 회전 감점, 부드러운 회전 감점
    # ------------------------------------------------------------
    score += forward_w * (1.0 - abs_w / max_abs_w)
    score -= turn_w * abs_w
    score -= smooth_weight * abs(w - prev_w)

    # 정면 장애물 접근 감점
    if front_clearance < ACTIVE_FRONT_DIST:
        score -= front_approach_weight * (ACTIVE_FRONT_DIST - front_clearance)

    # ------------------------------------------------------------
    # 좌/우 비대칭 보상
    # ------------------------------------------------------------
    if front_factor >= ASYMMETRY_GATE and abs_w > 1e-6:
        asym = (left_avg - right_avg) / (left_avg + right_avg + 1e-6)

        turn_ratio = abs_w / max_abs_w
        if turn_ratio > 1.0:
            turn_ratio = 1.0

        # w > 0: 좌회전
        # w < 0: 우회전
        if w > 0.0:
            score += front_factor * asymmetry_weight * asym * turn_ratio
        else:
            score += front_factor * asymmetry_weight * (-asym) * turn_ratio

    # ------------------------------------------------------------
    # 목표점 방향 유지 점수
    # ------------------------------------------------------------
    local_x = float(traj[-1, 0])
    local_y = float(traj[-1, 1])
    local_theta = float(traj[-1, 2])

    # transform_local_to_global() 함수 호출 대신 직접 계산
    cos_t = math.cos(robot_theta)
    sin_t = math.sin(robot_theta)

    candidate_x = robot_x + (local_x * cos_t - local_y * sin_t)
    candidate_y = robot_y + (local_x * sin_t + local_y * cos_t)

    # 후보 경로 마지막에서의 전역 heading
    candidate_theta = normalize_angle_rad(robot_theta + local_theta)

    # goal_heading_error_from_pose() 함수 호출 대신 직접 계산
    dx_goal = GOAL_X_M - candidate_x
    dy_goal = GOAL_Y_M - candidate_y
    target_heading = math.atan2(dy_goal, dx_goal)
    heading_err = normalize_angle_rad(target_heading - candidate_theta)

    # goal_distance() 함수 호출 대신 직접 계산
    cur_dx = GOAL_X_M - robot_x
    cur_dy = GOAL_Y_M - robot_y
    current_goal_dist = math.hypot(cur_dx, cur_dy)

    candidate_goal_dist = math.hypot(dx_goal, dy_goal)
    goal_progress = current_goal_dist - candidate_goal_dist

    # y축 중심선 오차
    lateral_err = candidate_y - GOAL_Y_M

    # heading 제한 관련 계산
    theta_abs = abs(candidate_theta)

    theta_excess = theta_abs - TURN_SOFT_LIMIT_RAD
    if theta_excess < 0.0:
        theta_excess = 0.0

    theta_growth = theta_abs - abs_robot_theta
    if theta_growth < 0.0:
        theta_growth = 0.0

    goal_factor = safe_factor

    # 목표점 관련 보상/감점
    score += goal_factor * GOAL_DISTANCE_WEIGHT * goal_progress
    score -= goal_factor * GOAL_HEADING_WEIGHT * abs(heading_err)
    score -= goal_factor * GOAL_LATERAL_WEIGHT * abs(lateral_err)

    # 과도한 회전 각도 감점
    score -= TURN_LIMIT_WEIGHT * theta_excess * theta_excess

    # 이미 많이 돌아간 상태에서 더 돌아가려 하면 감점
    if abs_robot_theta > TURN_SOFT_LIMIT_RAD:
        score -= TURN_GROWTH_WEIGHT * theta_growth

    # 거의 90도 가까이 틀어지는 후보는 큰 감점
    if theta_abs > TURN_HARD_LIMIT_RAD:
        score -= collision_weight * (theta_abs - TURN_HARD_LIMIT_RAD + 1.0)

    return score, front_clearance, side_clearance, body_clearance, candidate_theta


# ============================================================
# 19. w 명령 변화량 제한 함수
# ============================================================
def rate_limit_w(prev_w, target_w, urgent=False):
    # 일반 상황과 위급 상황에서 사용할 변화량 제한값을 다르게 선택합니다.
    limit = W_CMD_RATE_LIMIT_URGENT if urgent else W_CMD_RATE_LIMIT

    # target_w - prev_w가 너무 크면 -limit~+limit 안으로 자릅니다.
    delta = float(np.clip(target_w - prev_w, -limit, limit))

    # 제한된 변화량만 적용한 새 w를 반환합니다.
    return prev_w + delta


# ============================================================
# 20. 최적 v,w 선택 함수
# ============================================================
def choose_best_cmd(scan, prev_w, cmd_v):
    # 라이다 스캔을 x,y 점 배열로 변환합니다.
    points = lidar_points_to_xy(scan)

    # 사용할 라이다 점이 없으면 장애물이 없다고 보고 직진 + w를 0으로 천천히 복귀시킵니다.
    if len(points) == 0:
        return cmd_v, rate_limit_w(prev_w, 0.0), {
            "score": 0.0,
            "clear": MAX_LIDAR_DIST_M,
            "side": MAX_LIDAR_DIST_M,
            "body": MAX_LIDAR_DIST_M,
            "front": MAX_LIDAR_DIST_M,
            "points": 0,
            "collision": False,
            "raw_w": 0.0,
            "cth": robot_theta,
            "left": 1.0,
            "right": 1.0,
            "l_avg": SIDE_CAP_M,
            "r_avg": SIDE_CAP_M,
        }

    # 현재 정면 장애물 거리 계산
    fdist = front_distance(points)

    # 좌/우 30~60도 섹터 평균 거리 계산
    left_avg, right_avg = compute_side_averages(points)

    # 좌우 평균거리 차이를 정규화한 값입니다.
    # +이면 왼쪽이 더 열림, -이면 오른쪽이 더 열림입니다.
    asym = (left_avg - right_avg) / (left_avg + right_avg + 1e-6)

    # info_left, info_right는 로봇 바로 옆 근접 장애물 판단용 값입니다.
    # 초기값 1.0은 근접 장애물이 없다는 의미로 사용됩니다.
    info_left = 1.0
    info_right = 1.0

    # 로봇 중심 근처의 좌우 장애물을 찾기 위한 마스크입니다.
    # |x| < 0.15, |y| < 0.30 영역의 점만 봅니다.
    sb = (np.abs(points[:, 0]) < 0.15) & (np.abs(points[:, 1]) < 0.30)

    if sb.any():
        ys_sb = points[sb, 1]

        # y가 +0.05보다 크면 왼쪽 가까운 점
        ly = ys_sb[ys_sb > 0.05]

        # y가 -0.05보다 작으면 오른쪽 가까운 점
        ry = ys_sb[ys_sb < -0.05]

        # 왼쪽 점이 있으면 가장 작은 y값, 즉 로봇에 가장 가까운 왼쪽 거리 사용
        if len(ly) > 0:
            info_left = float(np.min(ly))

        # 오른쪽 점이 있으면 y는 음수이므로 -np.max(ry)를 통해 양수 거리로 변환
        if len(ry) > 0:
            info_right = float(-np.max(ry))

    # near_thresh보다 좌우 측면 거리가 가까우면 해당 방향 회전에 감점을 줍니다.
    near_thresh = 0.14

    # fallback clear_score에서 회전 크기를 정규화하기 위한 최대 w입니다.
    max_abs_w_local = max(abs(wc) for wc in W_CANDIDATES)

    # 최종 점수 기준으로 선택할 후보 정보 초기화
    best_w = 0.0
    best_score = -float("inf")
    best_clearance = -float("inf")
    best_side_clearance = MAX_LIDAR_DIST_M
    best_body_clearance = MAX_LIDAR_DIST_M

    # 모든 후보가 충돌 위험인지 확인하기 위한 플래그입니다.
    all_collision = True

    # 모든 후보가 충돌일 때 사용할 fallback 후보 정보입니다.
    best_clear_w = 0.0
    best_clear_score = -float("inf")

    # 선택된 후보의 예측 최종 heading입니다.
    best_theta = 0.0

    # 모든 후보 w를 하나씩 평가합니다.
    for w in W_CANDIDATES:
        # 후보 경로 점수와 clearance 계산
        score, clearance, side_clearance, body_clearance, candidate_theta = (
            evaluate_candidate(cmd_v, w, points, prev_w, fdist, left_avg, right_avg)
        )

        # 정면 clearance가 충돌거리보다 작으면 충돌 후보로 판단
        collision = clearance < COLLISION_DIST

        # 충돌이 아닌 후보가 하나라도 있으면 all_collision은 False
        if not collision:
            all_collision = False

        # fallback용 clear_score 계산
        # 모든 후보가 충돌 위험인 상황에서도 그나마 clearance가 큰 방향으로 돌리기 위함입니다.
        theta_abs = abs(candidate_theta)
        theta_excess = max(0.0, theta_abs - TURN_SOFT_LIMIT_RAD)
        theta_growth = max(0.0, theta_abs - abs(robot_theta))

        # clear_score는 기본적으로 clearance가 큰 후보를 선호합니다.
        clear_score = clearance + 0.18 * side_clearance + 0.03 * abs(w) - 0.02 * abs(w - prev_w)

        # 지나치게 큰 heading으로 틀어지는 후보는 감점
        clear_score -= 0.20 * theta_excess

        # 이미 많이 틀어진 상태에서 더 틀어지면 감점
        if abs(robot_theta) > TURN_SOFT_LIMIT_RAD:
            clear_score -= 0.12 * theta_growth

        # 우회전하려는데 오른쪽이 너무 가까우면 감점
        if w < 0 and info_right < near_thresh:
            closeness = (near_thresh - info_right) / near_thresh
            clear_score -= 0.5 * closeness * abs(w)

        # 좌회전하려는데 왼쪽이 너무 가까우면 감점
        if w > 0 and info_left < near_thresh:
            closeness = (near_thresh - info_left) / near_thresh
            clear_score -= 0.7 * closeness * abs(w)

        # 직진인데 좌우 중 하나가 너무 가까우면 감점
        if w == 0.0:
            nearest = min(info_left, info_right)
            if nearest < near_thresh:
                closeness = (near_thresh - nearest) / near_thresh
                clear_score -= 0.7 * closeness

        # fallback clear_score에도 좌우 비대칭 정보를 반영합니다.
        # 모든 후보가 충돌일 때 열린 방향으로 회전하도록 돕습니다.
        if w > 1e-6:
            # 좌회전 후보: 왼쪽이 열려 있으면 보상
            clear_score += 0.5 * asymmetry_weight * asym * min(abs(w) / max_abs_w_local, 1.0)
        elif w < -1e-6:
            # 우회전 후보: 오른쪽이 열려 있으면 보상
            clear_score += 0.5 * asymmetry_weight * (-asym) * min(abs(w) / max_abs_w_local, 1.0)

        # fallback 후보 중 clear_score가 가장 높은 w 저장
        if clear_score > best_clear_score:
            best_clear_score = clear_score
            best_clear_w = w

        # 일반 점수 score가 가장 높은 후보를 최종 후보로 저장
        if score > best_score:
            best_score = score
            best_w = w
            best_clearance = clearance
            best_side_clearance = side_clearance
            best_body_clearance = body_clearance
            best_theta = candidate_theta

    # 모든 후보가 충돌 위험이면 일반 점수 대신 fallback 후보를 선택합니다.
    # 즉, 완전히 막혔을 때는 충돌이 덜한 방향으로 회전하도록 합니다.
    if all_collision:
        best_w = best_clear_w
        best_score, best_clearance, best_side_clearance, best_body_clearance, best_theta = (
            evaluate_candidate(cmd_v, best_w, points, prev_w, fdist, left_avg, right_avg)
        )

    # rate limit 적용 전의 원래 선택 w입니다. 로그에서 raw로 출력됩니다.
    raw_best_w = best_w

    # 정면이 위급하거나 모든 후보가 충돌이면 urgent=True로 더 빠르게 w 변화 허용
    best_w = rate_limit_w(prev_w, best_w, fdist < URGENT_FRONT_DIST or all_collision)

    # cmd_v는 그대로 유지하고, 선택된 w와 디버깅 정보를 반환합니다.
    return cmd_v, best_w, {
        "score": best_score,
        "clear": best_clearance,
        "side": best_side_clearance,
        "body": best_body_clearance,
        "front": fdist,
        "points": len(points),
        "collision": best_clearance < COLLISION_DIST,
        "raw_w": raw_best_w,
        "cth": best_theta,
        "left": info_left,
        "right": info_right,
        "l_avg": left_avg,
        "r_avg": right_avg,
    }


# ============================================================
# 21. 메인 실행 함수
# ============================================================
def main():
    global robot_x, robot_y, robot_theta

    # 라이다 객체 생성. 생성자에서 라이다 초기화와 스캔 스레드 시작까지 수행합니다.
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)

    # 아두이노 시리얼 포트를 엽니다.
    ardu  = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)

    # 라이다/아두이노 초기 안정화를 위해 2초 대기합니다.
    print("[INFO] Warming up for 2 seconds..."); time.sleep(2.0)

    def send_vw(v, w):
        # 아두이노로 속도 명령을 보냅니다.
        # 형식: V전진속도,각속도\n
        # 예: V0.180,0.350
        # 아두이노 쪽 코드가 이 문자열을 파싱해서 모터를 제어해야 합니다.
        ardu.write(f"V{v:.3f},{w:.3f}\n".encode())

    def stop():
        # 아두이노로 정지 명령을 보냅니다.
        # 아두이노 쪽에서 S를 받으면 모터 정지하도록 되어 있어야 합니다.
        ardu.write(b"S\n")

    # 시작 전 일단 정지 명령 전송
    stop()
    print("[INFO] Initialization Complete. Press Enter to start!")

    # 사용자가 Enter를 누르면 주행 시작
    # 표준입력이 없는 환경에서는 EOFError가 발생할 수 있어 바로 시작하게 처리합니다.
    try:
        input()
    except EOFError:
        print("[WARN] Could not read standard input. Starting immediately.")

    print("[INFO] Go!!")

    # 주행 시작 시 로봇 위치 추정값 초기화
    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0

    # 마지막으로 라이다 스캔을 정상 수신한 시간
    last_scan_ok = 0.0

    # 마지막으로 보낸 속도 명령
    last_v, last_w = BASE_V, 0.0

    # 로그 출력 주기 조절용 시간
    last_log = 0.0

    # pose 업데이트용 이전 시간
    last_pose_time = time.time()

    try:
        while True:
            # 현재 시간과 직전 루프 이후 경과 시간 계산
            now = time.time()
            dt = max(0.0, min(0.20, now - last_pose_time))
            last_pose_time = now

            # 목표점에 도착했으면 정지하고 루프 종료
            if goal_distance() <= GOAL_TOL_M:
                stop()
                print("[INFO] Goal reached. Stopping.")
                break

            # 가장 최근 라이다 스캔 가져오기
            scan = lidar.get_scan()

            # 아직 스캔이 없거나 라이다 데이터가 안 들어오는 경우 처리
            if scan is None:
                # 최근 0.30초 이내에 정상 스캔이 있었다면 마지막 명령을 유지합니다.
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    send_vw(last_v, last_w)
                    update_pose(last_v, last_w, dt)
                else:
                    # 라이다가 오래 끊겼으면 안전하게 정지 명령을 보냅니다.
                    send_vw(0.0, 0.0)
                    update_pose(0.0, 0.0, dt)

                # 정지/유지 중에도 목표 도착 여부 확인
                if goal_distance() <= GOAL_TOL_M:
                    stop()
                    print("[INFO] Goal reached. Stopping.")
                    break

                # 루프 주기만큼 대기 후 다음 루프로 넘어갑니다.
                time.sleep(LOOP_DT_S)
                continue

            # 스캔이 정상적으로 있으면 마지막 정상 수신 시간을 갱신합니다.
            last_scan_ok = time.time()

            # 현재 스캔과 직전 w를 바탕으로 최적 v,w 명령을 선택합니다.
            v, w, info = choose_best_cmd(scan, last_w, BASE_V)

            # 선택된 속도 명령을 아두이노로 전송합니다.
            send_vw(v, w)

            # 명령값 기준으로 로봇 위치 추정값을 업데이트합니다.
            update_pose(v, w, dt)

            # 다음 루프에서 사용할 마지막 명령값 저장
            last_v, last_w = v, w

            # 목표까지 거리와 heading 오차 계산
            gd = goal_distance()
            he = goal_heading_error_from_pose(robot_x, robot_y, robot_theta)

            # 목표 도착 확인
            if gd <= GOAL_TOL_M:
                stop()
                print("[INFO] Goal reached. Stopping.")
                break

            # 0.25초마다 디버깅 로그 출력
            if time.time() - last_log > 0.25:
                print(f"[RUN2] x={robot_x:.2f} y={robot_y:.2f} "
                    f"th={robot_theta:.2f} gd={gd:.2f} he={he:.2f} "
                    f"v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                    f"front={info['front']:.2f} clear={info['clear']:.2f} "
                    f"side={info['side']:.2f} body={info['body']:.2f} "
                    f"score={info['score']:.2f} pts={info['points']} "
                    f"coll={int(info['collision'])} cth={info['cth']:.2f} "
                    f"L={info.get('left',-1):.2f} R={info.get('right',-1):.2f} "
                    f"lAvg={info.get('l_avg',-1):.2f} rAvg={info.get('r_avg',-1):.2f}")
                last_log = time.time()

            # 메인 루프 주기 유지
            time.sleep(LOOP_DT_S)

    except KeyboardInterrupt:
        # Ctrl+C를 누르면 예외가 발생합니다.
        # 여기서는 별도 메시지 없이 finally에서 안전 종료합니다.
        pass

    finally:
        # 어떤 이유로 종료되든 모터 정지, 시리얼 포트 닫기, 라이다 종료를 수행합니다.
        stop()
        time.sleep(0.2)
        ardu.close()
        lidar.close()
        print("[INFO] Shutdown complete.")


# 이 파일을 직접 실행했을 때만 main()을 실행합니다.
# 다른 파일에서 import할 경우 자동 실행되지 않게 하는 표준 Python 패턴입니다.
if __name__ == "__main__":
    main()
