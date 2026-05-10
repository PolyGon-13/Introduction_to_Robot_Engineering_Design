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
MIN_X_FOR_PLANNING = -0.10
MAX_EVAL_POINTS = 720
SCAN_HOLD_S = 0.30
LOOP_DT_S = 0.05

# 로컬 플래너 파라미터
BASE_V = 0.18

W_CANDIDATES = [-0.90, -0.70, -0.50, -0.35, -0.20, -0.10, 0.0,
                0.10, 0.20, 0.35, 0.50, 0.70, 0.90]

PREDICT_TIME_NORMAL = 1.50
PREDICT_TIME_SQUEEZE = 0.80
PREDICT_DT = 0.10
ROBOT_RADIUS = 0.16 # 로봇 반경
SAFETY_MARGIN = 0.16 # 안전 여유
COLLISION_DIST = ROBOT_RADIUS + SAFETY_MARGIN # 충돌 판정 거리
CLEARANCE_CAP = 1.0
FRONT_CORRIDOR_HALF = COLLISION_DIST + 0.30 # 정면 통로 반폭
ACTIVE_FRONT_DIST = 0.30 # 정면 위험 판정 거리
SIDE_NEAR_DIST = COLLISION_DIST + 0.20 # 측면 근접 경고 거리
W_CMD_RATE_LIMIT = 0.20
W_CMD_RATE_LIMIT_URGENT = 0.40
URGENT_FRONT_DIST = 0.30 # 위급 모드 진입 거리

GOAL_X_M = 3.0
TURN_SOFT_LIMIT_RAD = math.radians(60.0)
TURN_HARD_LIMIT_RAD = math.radians(95.0)
TURN_LIMIT_WEIGHT = 28.0
TURN_GROWTH_WEIGHT = 12.0
X_PROGRESS_WEIGHT = 0.8
CENTERING_WEIGHT = 0.8

SQUEEZE_ENTER_THRESH = 0.20
SQUEEZE_EXIT_THRESH = 0.25
HEADING_OFF_THRESH = math.radians(15.0)
SQUEEZE_BOOST_WEIGHT = 8.0
SQUEEZE_W_MIN = 0.50
SQUEEZE_SIDE_COLLISION_WEIGHT = 50.0

clearance_weight = 3.0
collision_weight = 80.0
side_clearance_weight = 0.6
side_near_weight = 8.0
side_collision_weight = 18.0
forward_weight = 1.0
far_forward_weight = 2.2
turn_weight = 0.25
far_turn_weight = 0.55
smooth_weight = 0.5

robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

prev_in_squeeze = False


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
def predict_trajectory(v, w, predict_time):
    n_steps = int(predict_time / PREDICT_DT) # step 수 미리 계산
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
def evaluate_candidate(v, w, points, prev_w, front_dist, in_squeeze, recovery_sign, predict_time):
    traj = predict_trajectory(v, w, predict_time) # 후보 경로
    front_clearance, side_clearance, body_clearance = trajectory_clearances(traj, points) # 정면/측면/전체 최단 거리

    max_abs_w = max(abs(wc) for wc in W_CANDIDATES)
    front_factor = float(np.clip((ACTIVE_FRONT_DIST - front_dist) / max(1e-6, ACTIVE_FRONT_DIST - COLLISION_DIST), 0.0, 1.0)) # 전방이 얼마나 위험한지를 0~1로 표현 (안전할수록 1에 가까움)
    forward_w = forward_weight + (1.0 - front_factor) * far_forward_weight
    turn_w = turn_weight + (1.0 - front_factor) * far_turn_weight

    side_w = SQUEEZE_SIDE_COLLISION_WEIGHT if in_squeeze else side_collision_weight

    score = 0.0
    score += clearance_weight * min(front_clearance, CLEARANCE_CAP)
    score += side_clearance_weight * min(side_clearance, CLEARANCE_CAP)
    if front_clearance < COLLISION_DIST:
        score -= collision_weight * (COLLISION_DIST - front_clearance + 1.0)
    if side_clearance < COLLISION_DIST:
        score -= side_w * (COLLISION_DIST - side_clearance + 1.0)
    elif side_clearance < SIDE_NEAR_DIST:
        score -= side_near_weight * (SIDE_NEAR_DIST - side_clearance)

    score += forward_w * (1.0 - abs(w) / max_abs_w)
    score -= turn_w * abs(w)
    score -= smooth_weight * abs(w - prev_w)

    local_x = float(traj[-1, 0])
    local_y = float(traj[-1, 1])
    local_theta = float(traj[-1, 2])
    candidate_x, candidate_y = transform_local_to_global(local_x, local_y)
    candidate_theta = normalize_angle_rad(robot_theta + local_theta)
    x_progress = candidate_x - robot_x
    theta_abs = abs(candidate_theta)
    theta_excess = max(0.0, theta_abs - TURN_SOFT_LIMIT_RAD)
    theta_growth = max(0.0, theta_abs - abs(robot_theta))
    centering_progress = abs(robot_theta) - abs(candidate_theta)
    centering_factor = min(abs(robot_theta) / TURN_SOFT_LIMIT_RAD, 1.0)

    score += X_PROGRESS_WEIGHT * x_progress
    score += CENTERING_WEIGHT * centering_factor * centering_progress
    score -= TURN_LIMIT_WEIGHT * theta_excess * theta_excess

    if abs(robot_theta) > TURN_SOFT_LIMIT_RAD:
        score -= TURN_GROWTH_WEIGHT * theta_growth

    if theta_abs > TURN_HARD_LIMIT_RAD:
        score -= collision_weight * (theta_abs - TURN_HARD_LIMIT_RAD + 1.0)

    if in_squeeze and robot_theta * w > 0.0:
        same_turn_factor = min(abs(robot_theta) / TURN_SOFT_LIMIT_RAD, 1.0)
        score -= SQUEEZE_SIDE_COLLISION_WEIGHT * same_turn_factor * abs(w)

    if in_squeeze and abs(w) >= SQUEEZE_W_MIN and math.copysign(1.0, w) == recovery_sign:
        score += SQUEEZE_BOOST_WEIGHT * abs(w)

    return score, front_clearance, side_clearance, body_clearance, candidate_theta


def rate_limit_w(prev_w, target_w, urgent=False):
    limit = W_CMD_RATE_LIMIT_URGENT if urgent else W_CMD_RATE_LIMIT
    delta = float(np.clip(target_w - prev_w, -limit, limit))
    return prev_w + delta


def choose_best_cmd(scan, prev_w, cmd_v):
    global prev_in_squeeze
    points = lidar_points_to_xy(scan)
    if len(points) == 0:
        prev_in_squeeze = False
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
            "squeeze": 0,
        }

    fdist = front_distance(points)

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

    heading_off = abs(robot_theta) > HEADING_OFF_THRESH
    if prev_in_squeeze:
        both_sides_blocked = (info_left < SQUEEZE_EXIT_THRESH and info_right < SQUEEZE_EXIT_THRESH)
    else:
        both_sides_blocked = (info_left < SQUEEZE_ENTER_THRESH and info_right < SQUEEZE_ENTER_THRESH)
    in_squeeze = both_sides_blocked and heading_off
    prev_in_squeeze = in_squeeze
    if in_squeeze:
        recovery_sign = -math.copysign(1.0, robot_theta)
        predict_time = PREDICT_TIME_SQUEEZE
    else:
        recovery_sign = 0.0
        predict_time = PREDICT_TIME_NORMAL

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
        if in_squeeze and robot_theta * w > 0.0:
            continue
        if in_squeeze and abs(w) < SQUEEZE_W_MIN:
            continue

        score, clearance, side_clearance, body_clearance, candidate_theta = (
            evaluate_candidate(cmd_v, w, points, prev_w, fdist, in_squeeze, recovery_sign, predict_time)
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
            evaluate_candidate(cmd_v, best_w, points, prev_w, fdist, in_squeeze, recovery_sign, predict_time)
        )

    raw_best_w = best_w
    best_w = rate_limit_w(prev_w, best_w, fdist < URGENT_FRONT_DIST or all_collision)

    if in_squeeze and robot_theta * best_w > 0.0:
        best_w = 0.0

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
        "squeeze": int(in_squeeze),
    }


def main():
    global robot_x, robot_y, robot_theta, prev_in_squeeze
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
    prev_in_squeeze = False
    last_scan_ok = 0.0
    last_v, last_w = BASE_V, 0.0
    last_log = 0.0
    last_pose_time = time.time()

    try:
        while True:
            now = time.time()
            dt = max(0.0, min(0.20, now - last_pose_time))
            last_pose_time = now

            if robot_x >= GOAL_X_M:
                stop()
                print("[INFO] Goal line crossed. Stopping.")
                break

            scan = lidar.get_scan()
            if scan is None:
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    send_vw(last_v, last_w)
                    update_pose(last_v, last_w, dt)
                else:
                    send_vw(0.0, 0.0)
                    update_pose(0.0, 0.0, dt)
                if robot_x >= GOAL_X_M:
                    stop()
                    print("[INFO] Goal line crossed. Stopping.")
                    break
                time.sleep(LOOP_DT_S); continue

            last_scan_ok = time.time()
            v, w, info = choose_best_cmd(scan, last_w, BASE_V)
            send_vw(v, w)
            update_pose(v, w, dt)
            last_v, last_w = v, w

            if robot_x >= GOAL_X_M:
                stop()
                print("[INFO] Goal line crossed. Stopping.")
                break

            if time.time() - last_log > 0.25:
                print(f"[RUN2] x={robot_x:.2f} y={robot_y:.2f} "
                    f"th={robot_theta:.2f} "
                    f"v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                    f"front={info['front']:.2f} clear={info['clear']:.2f} "
                    f"side={info['side']:.2f} body={info['body']:.2f} "
                    f"score={info['score']:.2f} pts={info['points']} "
                    f"coll={int(info['collision'])} cth={info['cth']:.2f} "
                    f"L={info.get('left',-1):.2f} R={info.get('right',-1):.2f} "
                    f"sq={info.get('squeeze',0)}")
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
