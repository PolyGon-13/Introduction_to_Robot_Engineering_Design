#!/usr/bin/env python3

import serial
import time
import math
import threading
import numpy as np

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
ARDU_PORT  = "/dev/ttyS0"
ARDU_BAUD  = 9600

# 라이다 캘리브레이션 (확정값)
ANGLE_OFFSET_DEG = +1.54
DIST_OFFSET_MM   = 0.0
LIDAR_ANGLE_SIGN = -1.0

# 좌표계: +각도/+y = 좌측, -각도/-y = 우측

# 라이다 전처리 파라미터
MIN_LIDAR_DIST_M = 0.05
MAX_LIDAR_DIST_M = 2.5
MIN_QUALITY = 1
MIN_X_FOR_PLANNING = 0.10
MAX_EVAL_POINTS = 720
SCAN_HOLD_S = 0.30
LOOP_DT_S = 0.05

# 로컬 플래너 파라미터
BASE_V = 0.18
W_CANDIDATES = [-0.70, -0.50, -0.35, -0.20, -0.10, 0.0,
                0.10, 0.20, 0.35, 0.50, 0.70]

PREDICT_TIME = 1.50
PREDICT_DT = 0.10
ROBOT_RADIUS = 0.16 # 로봇 반경
COLLISION_DIST = ROBOT_RADIUS + 0.05 # 충돌 판정 거리 (이 이상 장애물과 가까워지는 것 막음)
CLEARANCE_CAP = 0.6
FRONT_CORRIDOR_HALF = COLLISION_DIST - 0.09 # 정면으로 간주할 y축 거리
ACTIVE_FRONT_DIST = 0.30 # 정면 위험 판정 거리
FRONT_DANGER_DIST = 0.19

W_CMD_RATE_LIMIT = 0.20
W_CMD_RATE_LIMIT_URGENT = 0.30
URGENT_FRONT_DIST = 0.30 # 위급 모드 진입 거리

# 좌/우 비대칭 보상 파라미터 (정면 30°~60° 섹터의 평균 거리 차이로 회전 방향 유도)
SIDE_SECTOR_MIN_DEG = 45.0 # 섹터 시작 각도 (정면 30° 제외)
SIDE_SECTOR_MAX_DEG = 90.0 # 섹터 끝 각도
SIDE_CAP_M = 1.0 # 먼 점이 평균을 왜곡하는 것 방지 (캡)
ASYMMETRY_GATE = 0.0 # front_factor가 이 값 이상일 때만 비대칭 보상 활성화

ASYM_DEADZONE = 0.10
COUNT_PENALTY_MIN_POINTS = 5 # 점 개수 페널티 활성화 최소 총 점 개수
count_penalty_weight = 5.0 # 좌/우 점 개수 기반 페널티 강도

GOAL_X_M = 3.0
GOAL_Y_M = 0.0
GOAL_TOL_M = 0.15
GOAL_HEADING_WEIGHT = 1.0
GOAL_LATERAL_WEIGHT = 1.2
GOAL_DISTANCE_WEIGHT = 0.4
TURN_SOFT_LIMIT_RAD = math.radians(50.0)
TURN_HARD_LIMIT_RAD = math.radians(80.0)
TURN_LIMIT_WEIGHT = 28.0
TURN_GROWTH_WEIGHT = 12.0

clearance_weight = 3.0
side_clearance_weight = 0.6

collision_weight = 100.0 # 정면 충돌 페널티1 (충돌 직전)
front_approach_weight = 5.0 # 정면 충돌 페널티2

side_collision_weight = 18.0 # 측면 충돌 페널티1 (충돌 직전)
side_near_weight = 8.0 # 측면 충돌 페널티2

forward_weight = 1.0
far_forward_weight = 2.2
turn_weight = 0.25
far_turn_weight = 0.55
smooth_weight = 0.5
asymmetry_weight = 3.0 # 좌/우 비대칭 보상의 강도

robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0


# RPLidar C1 시리얼 통신 드라이버
class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1) # 시리얼 포트 열기

        self.ser.write(bytes([0xA5, 0x40])); time.sleep(2.0) # RESET
        self.ser.reset_input_buffer() # 리셋 동안 들어온 노이즈 제거

        self.ser.write(bytes([0xA5, 0x20])) # SCAN
        header = self.ser.read(7) # 응답 헤더
        if len(header) != 7 or header[0] != 0xA5 or header[1] != 0x5A:
            self.ser.close()
            raise RuntimeError("[LIDAR] Response Header Error")

        self.lock = threading.Lock()
        self.latest_scan = None # 최근 스캔 데이터
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buf_a, buf_d, buf_q = [], [], [] # 각도, 거리, 품질 버퍼
        while self.running:
            try:
                data = self.ser.read(5) # 5바이트 읽기
                if len(data) != 5: continue

                s_flag = data[0] & 0x01 # 시작 플래그
                s_inv = (data[0] & 0x02) >> 1 # 시작 플래그 반전값
                if s_inv != (1 - s_flag): continue # 시작 플래그 검증 (s_flag와 s_inv는 항상 반대)

                if (data[1] & 0x01) != 1: continue # 두 번째 바이트의 bit0은 항상 1

                quality = data[0] >> 2 # 신호 품질 (첫 바이트의 상위 6비트)
                angle = ((data[1] >> 1) | (data[2] << 7)) / 64.0 # 각도 (degree)
                dist = (data[3] | (data[4] << 8)) / 4.0 # 거리 (mm)

                # 이전 바퀴가 끝나면, 버퍼 저장 후 초기화
                if s_flag == 1 and len(buf_a) > 50:
                    with self.lock:
                        self.latest_scan = (np.array(buf_a, dtype=np.float32),
                                            np.array(buf_d, dtype=np.float32),
                                            np.array(buf_q, dtype=np.float32))
                    buf_a, buf_d, buf_q = [], [], []

                # 유효한 측정만 버퍼에 추가
                if dist > 0 and quality > 0:
                    buf_a.append(angle); buf_d.append(dist); buf_q.append(quality)

            except (serial.SerialException, OSError) as e:
                print(f"[LIDAR] Serial Error: {e}, retrying in 1 second...")
                time.sleep(1.0)
                try:
                    self.ser.reset_input_buffer()
                except:
                    pass

    # 가장 최근 스캔 반환
    def get_scan(self):
        with self.lock:
            return self.latest_scan

    # 종료
    def close(self):
        self.running = False
        try: self.ser.write(bytes([0xA5, 0x25])) # STOP
        except: pass
        time.sleep(0.1); self.ser.close()


# 각도(degree) -180° ~ +180° 범위 정규화
def normalize_angle_deg(angle):
    angle = (angle + 180.0) % 360.0 - 180.0
    return angle

# 각도(rad) -π ~ +π 범위 정규화
def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# 일정시간 동안 로봇이 이동한 거리/각도 계산
def update_pose(v, w, dt):
    global robot_x, robot_y, robot_theta
    robot_x += v * math.cos(robot_theta) * dt # x축으로 간 거리
    robot_y += v * math.sin(robot_theta) * dt # y축으로 간 거리
    robot_theta += w * dt # 각도 변화량
    robot_theta = normalize_angle_rad(robot_theta)


# 목표 지점까지의 직선 거리 계산
def goal_distance():
    dx = GOAL_X_M - robot_x
    dy = GOAL_Y_M - robot_y
    return math.hypot(dx, dy) # 유클리드 거리 (sqrt(dx^2 + dy^2)) - 빗변 계산

# 목표 방향과 현재 헤딩의 차이 계산
def goal_heading_error_from_pose(x, y, theta):
    dx = GOAL_X_M - x
    dy = GOAL_Y_M - y
    target_heading = math.atan2(dy, dx)
    return normalize_angle_rad(target_heading - theta)


# 로봇 기준 좌표(local) -> 전역 좌표(global)
# 로봇의 전역좌표에 원하는 지점의 좌표를 전역좌표로 변경하여 더해주는 과정
def transform_local_to_global(local_x, local_y):
    gx = robot_x + (local_x * math.cos(robot_theta) - local_y * math.sin(robot_theta))
    gy = robot_y + (local_x * math.sin(robot_theta) + local_y * math.cos(robot_theta))
    return gx, gy


def lidar_points_to_xy(scan):
    if scan is None: # 스캔 없으면 빈 배열
        return np.empty((0, 2), dtype=np.float32)

    angles, dists, qualities = scan
    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0 # 거리 변환 mm -> m, DIST_OFFSET_MM만큼 보정
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG) # 각도 변환
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg # 각도 부호 변환

    # 유효 측정 필터
    mask = ((dist_m >= MIN_LIDAR_DIST_M) & # 일정 거리 미만은 무시 (5cm)
            (dist_m <= MAX_LIDAR_DIST_M) & # 일정 거리 이상은 무시 (2.5m)
            (qualities >= MIN_QUALITY)) # 낮은 신호 품질 측정은 무시
    if not mask.any():
        return np.empty((0, 2), dtype=np.float32)

    # 극좌표(r,θ) -> 직교좌표(x,y) 변환
    dist_m = dist_m[mask]
    angle_rad = np.deg2rad(angle_deg[mask])
    x = dist_m * np.cos(angle_rad)
    y = dist_m * np.sin(angle_rad)

    # 전방 필터링
    mask_xy = x >= MIN_X_FOR_PLANNING
    if not mask_xy.any():
        return np.empty((0, 2), dtype=np.float32)

    # 필터링된 x,y 값들을 점 배열로 제작
    points = np.column_stack((x[mask_xy], y[mask_xy])).astype(np.float32)

    if len(points) > MAX_EVAL_POINTS: # 점 개수가 720개 이상인 경우 (RPLidar C1이 한 회전에 400~600개 점이라 거의 실행될 일 없음)
        order = np.argsort(points[:, 0] ** 2 + points[:, 1] ** 2)
        points = points[order[:MAX_EVAL_POINTS]] # 가장 가까운 720개 점
    return points
    # points[:, 0] : 모든 x좌표, points[:, 1] : 모든 y좌표
    # argsort : 정렬했을 때 원래 인덱스가 어디로 가는지 반환


# 후보 v,w 명령으로 로봇이 PREDICT_TIME초 동안 그릴 미래 경로 계산
def predict_trajectory(v, w):
    n_steps = int(PREDICT_TIME / PREDICT_DT) # step 수 미리 계산
    ts = (np.arange(n_steps) + 1) * PREDICT_DT # 각 step의 누적 시간을 한 줄로 제작
    thetas = w * ts # 모든 스텝의 헤딩

    if abs(w) > 1e-6: # 곡선 주행
        # v/w : 회전 반지름 R
        xs = (v / w) * np.sin(thetas)
        ys = (v / w) * (1.0 - np.cos(thetas))
        # 회전중심이 (0,R), 로봇의 시작점은 (0,-R)
        # 절대 좌표는 (0+Rsin(theta),R+(-Rcos(theta))
    else: # 직진
        xs = v * ts
        ys = np.zeros(n_steps)

    return np.column_stack((xs, ys, thetas)).astype(np.float32)


# 로봇의 정면 범위 안에서 가장 가까운 장애물까지의 거리
def front_distance(points):
    if len(points) == 0: # 점이 하나도 없으면 안전으로 간주하고 MAX 거리 반환
        return MAX_LIDAR_DIST_M
    
    mask = ((points[:, 0] > 0.0) & # x가 양수 -> 로봇 앞쪽
            (points[:, 0] < ACTIVE_FRONT_DIST) & # x가 특정값 이내 -> 너무 멀지 않음
            (np.abs(points[:, 1]) < FRONT_CORRIDOR_HALF)) # |y|가 특정값 이내 -> 정면 범위 안
    if not mask.any():
        return MAX_LIDAR_DIST_M
    
    # 가장 가까운 정면 장애물과의 거리
    return float(np.min(points[mask, 0])) 


# 좌/우 섹터의 평균 거리 계산 (정면 30°~60° 영역)
# 좌측이 더 열려있으면 left_avg > right_avg, 비대칭(asym)이 양수 → 좌회전(w>0) 보상
def compute_side_averages(points):
    if len(points) == 0:
        return SIDE_CAP_M, SIDE_CAP_M # 점이 없으면 양쪽 모두 캡값으로 간주

    # 각 점의 극좌표 표현 (거리와 각도)
    dist = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)
    angle_deg = np.degrees(np.arctan2(points[:, 1], points[:, 0])) # +y(좌측) → +각도

    # 각 점에 대해 거리를 캡으로 클리핑 (먼 점 하나가 평균 왜곡 방지)
    dist_capped = np.minimum(dist, SIDE_CAP_M)

    # 좌측 섹터: +30° ~ +60°
    left_mask = (angle_deg >= SIDE_SECTOR_MIN_DEG) & (angle_deg <= SIDE_SECTOR_MAX_DEG)
    # 우측 섹터: -60° ~ -30°
    right_mask = (angle_deg <= -SIDE_SECTOR_MIN_DEG) & (angle_deg >= -SIDE_SECTOR_MAX_DEG)

    left_avg = float(np.mean(dist_capped[left_mask])) if left_mask.any() else SIDE_CAP_M
    right_avg = float(np.mean(dist_capped[right_mask])) if right_mask.any() else SIDE_CAP_M

    return left_avg, right_avg


# 전방 반원(x>0)의 좌/우 점 개수 계산
# 더 많은 점이 찍힌 쪽은 장애물이 더 많다는 의미
def compute_side_counts(points):
    if len(points) == 0:
        return 0, 0
    ys = points[:, 1]
    left_count = int(np.sum(ys > 0)) # 좌측(y>0) 점 개수
    right_count = int(np.sum(ys < 0)) # 우측(y<0) 점 개수
    return left_count, right_count


# 경로를 따라가면 미래 스텝 동안 장애물에 얼마나 가까이 지나가는지 계산
# points : (N,2) -> 라이다가 측정한 장애물 위치
# traj : (steps,3) -> 미래 경로의 가상 위치들
def trajectory_clearances(traj, points):
    if len(points) == 0:
        return MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M

    traj_xy = traj[:, :2] # 각 step의 (x,y) : (steps,2)
    theta = traj[:, 2] # 각 step의 헤딩 : (steps, 1)

    # 차원을 맞추어 브로드캐스팅을 이용하기 위해 None 추가
    # (1,N,2) - (steps,1,2) = (steps,N,2) : 모든 step과 모든 점의 모든 쌍에 대해 좌표 차이 한 번에 계산
    diff = points[None, :, :] - traj_xy[:, None, :] # 각 step에서 실제 장애물까지의 거리
    dx = diff[:, :, 0] # Δx
    dy = diff[:, :, 1] # Δy
    dist = np.sqrt(dx ** 2 + dy ** 2) # 모든 점 쌍의 유클리드 거리

    c = np.cos(theta)[:, None] # 각 step의 cos값
    s = np.sin(theta)[:, None] # 각 step의 sin값
    rel_x = c * dx + s * dy
    rel_y = -s * dx + c * dy
    # 2D 회전행렬

    # 정면 위험도 검사
    front_mask = ((rel_x > 0.0) &
                  (rel_x < ACTIVE_FRONT_DIST) &
                  (np.abs(rel_y) < FRONT_CORRIDOR_HALF))
    if front_mask.any():
        front_clear = float(np.min(dist[front_mask]))
        # 범위 안에 점이 하나라도 있으면 그 점들의 거리 중 최솟값
    else:
        front_clear = MAX_LIDAR_DIST_M

    # 측면 위험도 검사
    side_mask = ~front_mask # 범위 안에 들어오지 않은 모든 점
    if side_mask.any():
        side_clear = float(np.min(dist[side_mask]))
    else:
        side_clear = MAX_LIDAR_DIST_M

    # 전체 위험도 검사
    body_clear = float(np.min(dist))

    return front_clear, side_clear, body_clear


# (v,w)에 대해 cost항으로 점수 계산
def evaluate_candidate(v, w, points, prev_w, front_dist, left_avg, right_avg, left_count, right_count):
    traj = predict_trajectory(v, w) # 후보 경로
    front_clearance, side_clearance, body_clearance = trajectory_clearances(traj, points) # 정면/측면/전체 최단 거리

    max_abs_w = max(abs(wc) for wc in W_CANDIDATES)
    # 전방이 얼마나 위험한지를 0~1로 표현 (안전할수록 1에 가까움)
    # ACTIVE_FRONT_DIST보다 크거나 같으면 0.0, FRONT_DANGER_DIST보다 작으면 1.0
    front_factor = float(np.clip((ACTIVE_FRONT_DIST - front_dist) / max(1e-6, ACTIVE_FRONT_DIST - FRONT_DANGER_DIST), 0.0, 1.0))

    # 직진 보상 가중치
    # 전방이 안전할수록(front_factor이 0에 가까워짐) 커짐
    forward_w = forward_weight + (1.0 - front_factor) * far_forward_weight # 1.0 ~ 3.2

    # 회전 보상 가중치
    # 전방이 안전할수록(front_factor이 0에 가까워짐) 작아짐
    turn_w = turn_weight + (1.0 - front_factor) * far_turn_weight # 0.25 ~ 0.80


    # 보상 함수 부분
    score = 0.0

    # 예측 경로가 정면 장애물과 얼마나 멀리 떨어져 지나가는지에 대한 보상
    # 정면 장애물 거리가 CLEARANCE_CAP에 가까워질수록 보상 증가, 넘으면 최댓값 유지
    # front_clearance : 후보 v,w로 앞으로 PREDICT_TIME 동안 움직인다고 예측했을 때 경로상 정면 장애물과의 최소 거리
    score += clearance_weight * min(front_clearance, CLEARANCE_CAP) # 0 ~ 1.8

    # 예측 경로가 측면 장애물과의 여유거리가 클수록 보상
    # 측면 장애물 거리가 CLEARANCE_CAP에 가까워질수록 보상 증가, 넘으면 최댓값 유지
    # side_clearance : 정면 통로에 들어오지 않은 나머지
    score += side_clearance_weight * min(side_clearance, CLEARANCE_CAP) # 0 ~ 0.36

    # 정면 충돌 페널티
    # 정면 장애물이 COLLISION_DIST 안으로 들어오면 "강한" 페널티
    if front_clearance < COLLISION_DIST:
        score -= collision_weight * (COLLISION_DIST - front_clearance + 1.0) # -121 ~ 1.8
    # 정면 장애물이 ACTIVE_FRONT_DIST 안으로 들어오면 페널티
    elif front_clearance < ACTIVE_FRONT_DIST:
        score -= front_approach_weight * (ACTIVE_FRONT_DIST - front_clearance) # -3 ~ 0

    # 측면 충돌 페널티
    # 측면 장애물이 COLLISION_DIST 안으로 들어오면 페널티
    if side_clearance < COLLISION_DIST:
        score -= side_collision_weight * (COLLISION_DIST - side_clearance + 1.0) # -21.78 ~ -18.0
    # 측면 장애물이 COLLISION_DIST는 아니지만, 너무 가까울 때 주는 "약한" 페널티
    elif side_clearance < (COLLISION_DIST + 0.1):
        score -= side_near_weight * ((COLLISION_DIST + 0.1) - side_clearance) # -0.8 ~ 0
    
    # 직진에 가까운 후보일수록 보상
    # max_abs_w는 W_CANDIDATES 리스트의 최댓값
    score += forward_w * (1.0 - abs(w) / max_abs_w) # 0.0 ~ 3.2

    # 회전량이 클수록 페널티
    score -= turn_w * abs(w) # -0.72 ~ 0

    # 회전 명령에서 너무 갑자기 회전하는 후보 페널티
    score -= smooth_weight * abs(w - prev_w) # -0.9 ~ 0

    # 전방이 막혔을 때, 좌우 중 더 열린 방향으로 회전하도록 보상/페널티
    asym = (left_avg - right_avg) / (left_avg + right_avg + 1e-6)
    side_avg_valid = (left_avg < SIDE_CAP_M) and (right_avg < SIDE_CAP_M)
    if side_avg_valid and front_factor >= ASYMMETRY_GATE:
        # 왼쪽이 더 멀리 비어 있으면 양수, 오른쪽이 더 멀리 비어있으면 음수
        if w > 1e-6: # 왼쪽이 더 열려있는 경우
            score += front_factor * asymmetry_weight * asym * min(abs(w) / max_abs_w, 1.0) # -3 ~ 3
        elif w < -1e-6:
            score += front_factor * asymmetry_weight * (-asym) * min(abs(w) / max_abs_w, 1.0) # -3 ~ 3

    # 좌/우 평균 거리가 비슷할 때 (애매한 상황)
    total_count = left_count + right_count # 좌우 점 개수 총합
    # 좌우 평균 거리 차이가 작고, 좌우 점이 최소 5개 이상 있을 때만
    if abs(asym) < ASYM_DEADZONE and total_count >= COUNT_PENALTY_MIN_POINTS:
        count_asym = (left_count - right_count) / total_count # 좌우 점 개수 비대칭(왼쪽 점이 더 많으면 양수, 오른쪽 점이 더 많으면 음수) : -1.0 ~ 1.0
        if w > 1e-6 and count_asym > 0: # 왼쪽 점이 더 많은데 좌회전하려는 후보
            score -= count_penalty_weight * count_asym * min(abs(w) / max_abs_w, 1.0) # -5 ~ 0
        elif w < -1e-6 and count_asym < 0: # 오른쪽 점이 더 많은데 우회전하려는 후보
            score -= count_penalty_weight * (-count_asym) * min(abs(w) / max_abs_w, 1.0) # -5 ~ 0


    # w로 움직였을 때 최종 위치/방향이 목표점 기준으로 얼마나 좋은지 평가하기 위한 값 계산
    # predict_trajectory로 만든 예측 경로 (PREDICT_TIME초 움직인 뒤의 예상 위치/각도)
    local_x = float(traj[-1, 0])
    local_y = float(traj[-1, 1])
    local_theta = float(traj[-1, 2])

    # 로봇 기준 좌표를 전역 좌표계 위치로 변환
    candidate_x, candidate_y = transform_local_to_global(local_x, local_y)

    # 후보 경로의 최종 방향을 전역 방향으로 변환
    candidate_theta = normalize_angle_rad(robot_theta + local_theta)

    # 후보 실행 후 위치/방향에서 봤을 때, 목표점을 향하려면 얼마나 방향 오차가 있는지 계산
    heading_err = goal_heading_error_from_pose(candidate_x, candidate_y, candidate_theta)

    # 후보 실행 후 y위치가 목표 경로 y에서 얼마나 벗어났는지 계산
    lateral_err = candidate_y - GOAL_Y_M

    # 현재 로봇 위치에서 목표점까지의 거리
    current_goal_dist = goal_distance()

    # 후보 실행 후 예상 위치에서 목표점까지의 거리
    candidate_goal_dist = math.hypot(GOAL_X_M - candidate_x, GOAL_Y_M - candidate_y)

    # 목표점에 얼마나 가까워지는지 계산
    goal_progress = current_goal_dist - candidate_goal_dist

    # 목표점 추종을 얼마나 강하게 반영할지 결정
    # 전방이 안전하면 강하게 추종, 위험하면 거의 무시
    goal_factor = 1.0 - front_factor

    # 후보 실행 후 로봇 방향이 기준 방향에서 얼마나 틀어졌는지 절댓값으로 확인
    theta_abs = abs(candidate_theta)

    # 부드러운 회전 제한을 얼마나 초과했는지 계산 (TURN_SOFT_LIMIT_RAD만큼은 OK)
    theta_excess = max(0.0, theta_abs - TURN_SOFT_LIMIT_RAD)

    # 현재 방향보다 후보 실행 후 방향이 얼마나 더 켜졌는지 계산
    # 현재 A도, 후보 실행 후 B도면 A-B, 만약 0보다 작다면(기준선에서 멀어지는 방향의 회전이 아니라면), 0으로 설정
    theta_growth = max(0.0, theta_abs - abs(robot_theta))

    # 후보 경로가 목표점에 가까워지면 보상, 멀어지면 페널티
    score += goal_factor * GOAL_DISTANCE_WEIGHT * goal_progress # -0.086 ~ 0.086

    # 후보 경로의 최종방향이 목표점을 잘 바라보지 못하면 페널티
    score -= goal_factor * GOAL_HEADING_WEIGHT * abs(heading_err) # -3.14 ~ 0

    # 후보 경로의 최종 위치가 목표 중심선에서 벗어나면 페널티
    score -= goal_factor * GOAL_LATERAL_WEIGHT * abs(lateral_err) # -1.2 ~ 0

    # 후보 실행 후 로봇의 방향이 너무 많이 틀어졌을 때 페널티
    score -= TURN_LIMIT_WEIGHT * theta_excess * theta_excess # -103.2 ~ 0

    # 로봇 방향이 너무 틀어지는 것 방지 페널티
    # TURN_SOFT_LIMIT_RAD 초과 틀어지면 페널티
    if abs(robot_theta) > TURN_SOFT_LIMIT_RAD:
        score -= TURN_GROWTH_WEIGHT * theta_growth # -23 ~ 0
    # TURN_HARD_LIMIT_RAD 초과 틀어지면 "강한" 페널티
    if theta_abs > TURN_HARD_LIMIT_RAD:
        score -= collision_weight * (theta_abs - TURN_HARD_LIMIT_RAD + 1.0) # -260.6 ~ -100

    return score, front_clearance, side_clearance, body_clearance, candidate_theta


# 전방 장애물 거리 fdist가 URGENT_FRONT_DIST보다 작으면 urgent=True
# 모든 w 후보가 충돌 위험이라고 판단되어 all_collision=True가 되면 urgent=True
def rate_limit_w(prev_w, target_w, urgent=False):
    limit = W_CMD_RATE_LIMIT_URGENT if urgent else W_CMD_RATE_LIMIT
    delta = float(np.clip(target_w - prev_w, -limit, limit))
    return prev_w + delta


def choose_best_cmd(scan, prev_w, cmd_v):
    points = lidar_points_to_xy(scan)
    if len(points) == 0:
        return cmd_v, rate_limit_w(prev_w, 0.0), {
            "score": 0.0,
            "clear": MAX_LIDAR_DIST_M,
            "side": MAX_LIDAR_DIST_M,
            "body": MAX_LIDAR_DIST_M,
            "front": MAX_LIDAR_DIST_M,
            "front_factor": 0.0,
            "points": 0,
            "collision": False,
            "raw_w": 0.0,
            "cth": robot_theta,
            "left": 1.0,
            "right": 1.0,
            "l_avg": SIDE_CAP_M,
            "r_avg": SIDE_CAP_M,
            "l_cnt": 0,
            "r_cnt": 0,
        }

    fdist = front_distance(points)
    front_factor = float(np.clip((ACTIVE_FRONT_DIST - fdist) / max(1e-6, ACTIVE_FRONT_DIST - FRONT_DANGER_DIST), 0.0, 1.0))

    # 좌/우 섹터 평균 거리 (모든 후보 평가에 공통으로 사용되므로 한 번만 계산)
    left_avg, right_avg = compute_side_averages(points)
    # 좌/우 전방 점 개수 (평균 거리가 비슷할 때 페널티에 사용)
    left_count, right_count = compute_side_counts(points)
    # fallback에서 사용할 비대칭 값 (전방이 막혔을 때 좌/우 선택에 사용)
    asym = (left_avg - right_avg) / (left_avg + right_avg + 1e-6)
    # fallback에서 사용할 점 개수 비대칭 값
    total_count = left_count + right_count
    count_asym = (left_count - right_count) / total_count if total_count > 0 else 0.0

    info_left = 1.0
    info_right = 1.0
    sb = (np.abs(points[:, 0]) < 0.15) & (np.abs(points[:, 1]) < 0.30)
    if sb.any():
        ys_sb = points[sb, 1]
        ly = ys_sb[ys_sb > 0.05]
        ry = ys_sb[ys_sb < -0.05]
        if len(ly) > 0:
            info_left = float(np.min(ly))
        if len(ry) > 0:
            info_right = float(-np.max(ry))

    near_thresh = 0.14
    max_abs_w_local = max(abs(wc) for wc in W_CANDIDATES)

    best_w = 0.0
    best_score = -float("inf")
    best_clearance = -float("inf")
    best_side_clearance = MAX_LIDAR_DIST_M
    best_body_clearance = MAX_LIDAR_DIST_M
    all_collision = True
    best_clear_w = 0.0
    best_clear_score = -float("inf")
    best_theta = 0.0

    for w in W_CANDIDATES:
        score, clearance, side_clearance, body_clearance, candidate_theta = (
            evaluate_candidate(cmd_v, w, points, prev_w, fdist, left_avg, right_avg, left_count, right_count)
        )
        collision = clearance < COLLISION_DIST
        if not collision:
            all_collision = False
        theta_abs = abs(candidate_theta)
        theta_excess = max(0.0, theta_abs - TURN_SOFT_LIMIT_RAD)
        theta_growth = max(0.0, theta_abs - abs(robot_theta))
        clear_score = clearance + 0.18 * side_clearance + 0.03 * abs(w) - 0.02 * abs(w - prev_w)
        clear_score -= 0.20 * theta_excess
        if abs(robot_theta) > TURN_SOFT_LIMIT_RAD:
            clear_score -= 0.12 * theta_growth
        if w < 0 and info_right < near_thresh:
            closeness = (near_thresh - info_right) / near_thresh
            clear_score -= 0.5 * closeness * abs(w)
        if w > 0 and info_left < near_thresh:
            closeness = (near_thresh - info_left) / near_thresh
            clear_score -= 0.7 * closeness * abs(w)
        if w == 0.0:
            nearest = min(info_left, info_right)
            if nearest < near_thresh:
                closeness = (near_thresh - nearest) / near_thresh
                clear_score -= 0.7 * closeness
        # fallback의 clear_score에도 비대칭 정보 반영
        # all_collision 상황에서 좌/우 평균 거리 차이로 회전 방향 유도
        side_avg_valid = (left_avg < SIDE_CAP_M) and (right_avg < SIDE_CAP_M)
        if side_avg_valid and w > 1e-6:
            clear_score += 0.5 * asymmetry_weight * asym * min(abs(w) / max_abs_w_local, 1.0)
        elif side_avg_valid and w < -1e-6:
            clear_score += 0.5 * asymmetry_weight * (-asym) * min(abs(w) / max_abs_w_local, 1.0)
        # fallback에서도 평균 거리가 비슷할 때 점 개수 페널티 적용
        if abs(asym) < ASYM_DEADZONE and total_count >= COUNT_PENALTY_MIN_POINTS:
            if w > 1e-6 and count_asym > 0:
                clear_score -= 0.5 * count_penalty_weight * count_asym * min(abs(w) / max_abs_w_local, 1.0)
            elif w < -1e-6 and count_asym < 0:
                clear_score -= 0.5 * count_penalty_weight * (-count_asym) * min(abs(w) / max_abs_w_local, 1.0)
        if clear_score > best_clear_score:
            best_clear_score = clear_score
            best_clear_w = w
        if score > best_score:
            best_score = score
            best_w = w
            best_clearance = clearance
            best_side_clearance = side_clearance
            best_body_clearance = body_clearance
            best_theta = candidate_theta

    if all_collision:
        best_w = best_clear_w
        best_score, best_clearance, best_side_clearance, best_body_clearance, best_theta = (
            evaluate_candidate(cmd_v, best_w, points, prev_w, fdist, left_avg, right_avg, left_count, right_count)
        )

    raw_best_w = best_w
    best_w = rate_limit_w(prev_w, best_w, fdist < URGENT_FRONT_DIST or all_collision)
    return cmd_v, best_w, {
        "score": best_score,
        "clear": best_clearance,
        "side": best_side_clearance,
        "body": best_body_clearance,
        "front": fdist,
        "front_factor": front_factor,
        "points": len(points),
        "collision": best_clearance < COLLISION_DIST,
        "raw_w": raw_best_w,
        "cth": best_theta,
        "left": info_left,
        "right": info_right,
        "l_avg": left_avg,
        "r_avg": right_avg,
        "l_cnt": left_count,
        "r_cnt": right_count,
    }


def main():
    global robot_x, robot_y, robot_theta
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    ardu  = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    print("[INFO] Warming up for 2 seconds..."); time.sleep(2.0)

    def send_vw(v, w):
        ardu.write(f"V{v:.3f},{w:.3f}\n".encode())

    def stop():
        ardu.write(b"S\n")

    stop()
    print("[INFO] Initialization Complete. Press Enter to start!")
    try:
        input()
    except EOFError:
        print("[WARN] Could not read standard input. Starting immediately.")
    print("[INFO] Go!!")

    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0
    last_scan_ok = 0.0
    last_v, last_w = BASE_V, 0.0
    last_log = 0.0
    last_pose_time = time.time()

    try:
        while True:
            now = time.time()
            dt = max(0.0, min(0.20, now - last_pose_time))
            last_pose_time = now

            if goal_distance() <= GOAL_TOL_M:
                stop()
                print("[INFO] Goal reached. Stopping.")
                break

            scan = lidar.get_scan()
            if scan is None:
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    send_vw(last_v, last_w)
                    update_pose(last_v, last_w, dt)
                else:
                    send_vw(0.0, 0.0)
                    update_pose(0.0, 0.0, dt)
                if goal_distance() <= GOAL_TOL_M:
                    stop()
                    print("[INFO] Goal reached. Stopping.")
                    break
                time.sleep(LOOP_DT_S); continue

            last_scan_ok = time.time()
            v, w, info = choose_best_cmd(scan, last_w, BASE_V)
            send_vw(v, w)
            update_pose(v, w, dt)
            last_v, last_w = v, w

            gd = goal_distance()
            he = goal_heading_error_from_pose(robot_x, robot_y, robot_theta)
            if gd <= GOAL_TOL_M:
                stop()
                print("[INFO] Goal reached. Stopping.")
                break

            if time.time() - last_log > 0.25:
                print(f"[RUN2] x={robot_x:.2f} y={robot_y:.2f} "
                    f"th={robot_theta:.2f} gd={gd:.2f} he={he:.2f} "
                    f"v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                    f"front={info['front']:.2f} ff={info['front_factor']:.2f} clear={info['clear']:.2f} "
                    f"side={info['side']:.2f} body={info['body']:.2f} "
                    f"score={info['score']:.2f} pts={info['points']} "
                    f"coll={int(info['collision'])} cth={info['cth']:.2f} "
                    f"L={info.get('left',-1):.2f} R={info.get('right',-1):.2f} "
                    f"lAvg={info.get('l_avg',-1):.2f} rAvg={info.get('r_avg',-1):.2f} "
                    f"lCnt={info.get('l_cnt',0)} rCnt={info.get('r_cnt',0)}")
                last_log = time.time()

            time.sleep(LOOP_DT_S)

    except KeyboardInterrupt:
        pass
    finally:
        stop(); time.sleep(0.2)
        ardu.close(); lidar.close()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()