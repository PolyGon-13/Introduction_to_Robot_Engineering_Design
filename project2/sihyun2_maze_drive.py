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
# 전진 속도는 기존 그대로 유지
BASE_V = 0.18

W_CANDIDATES = [-0.90, -0.70, -0.50, -0.35, -0.20, -0.10, 0.0,
                0.10, 0.20, 0.35, 0.50, 0.70, 0.90]

PREDICT_TIME = 1.50
PREDICT_DT = 0.10

ROBOT_RADIUS = 0.16        # 로봇 반경
SAFETY_MARGIN = 0.14       # 안전 여유
COLLISION_DIST = ROBOT_RADIUS + SAFETY_MARGIN  # 충돌 판정 거리

CLEARANCE_CAP = 1.0

# 기존: COLLISION_DIST + 0.30
# 너무 넓으면 좌우 장애물도 정면 위험으로 과대판단함
FRONT_CORRIDOR_HALF = COLLISION_DIST + 0.12

# 기존 0.30은 COLLISION_DIST와 거의 같아서 반응이 늦을 수 있음
ACTIVE_FRONT_DIST = 0.45

SIDE_NEAR_DIST = COLLISION_DIST + 0.20

# 일반 상황에서는 부드럽게, 위급 상황에서는 반대 방향으로 빠르게 전환
W_CMD_RATE_LIMIT = 0.30
W_CMD_RATE_LIMIT_URGENT = 1.0

URGENT_FRONT_DIST = 0.35

# 좌우 장애물 조기 감지용 파라미터
# 기존에는 |x| < 0.15만 봐서 앞쪽 좌우 장애물을 늦게 잡았음
SIDE_LOOK_X_BACK = -0.05
SIDE_LOOK_X_FRONT = 0.45
SIDE_LOOK_Y = 0.40

# 좌우 장애물이 이 거리 안에 있으면 회피 방향 판단에 강하게 반영
SIDE_AVOID_DIST = 0.22

# 이 거리보다 가까우면 해당 장애물 반대 방향 회전을 강제적으로 선호
SIDE_HARD_DIST = 0.16

# 좌우 장애물 쪽으로 도는 후보 감점 강도
SIDE_TURN_INTO_PENALTY = 12.0
SIDE_STRAIGHT_NEAR_PENALTY = 2.5
SIDE_TURN_AWAY_BONUS = 1.4

GOAL_X_M = 3.0
GOAL_Y_M = 0.0
GOAL_TOL_M = 0.15
GOAL_HEADING_WEIGHT = 1.0
GOAL_LATERAL_WEIGHT = 1.2
GOAL_DISTANCE_WEIGHT = 0.4

TURN_SOFT_LIMIT_RAD = math.radians(70.0)
TURN_HARD_LIMIT_RAD = math.radians(88.0)
TURN_LIMIT_WEIGHT = 28.0
TURN_GROWTH_WEIGHT = 12.0

clearance_weight = 3.0
collision_weight = 100.0
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


# RPLidar C1 시리얼 통신 드라이버
class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.ser.write(bytes([0xA5, 0x40]))
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        self.ser.write(bytes([0xA5, 0x20]))
        header = self.ser.read(7)

        if len(header) != 7 or header[0] != 0xA5 or header[1] != 0x5A:
            self.ser.close()
            raise RuntimeError("[LIDAR] Response Header Error")

        self.lock = threading.Lock()
        self.latest_scan = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buf_a, buf_d, buf_q = [], [], []

        while self.running:
            try:
                data = self.ser.read(5)
                if len(data) != 5:
                    continue

                s_flag = data[0] & 0x01
                s_inv = (data[0] & 0x02) >> 1

                if s_inv != (1 - s_flag):
                    continue

                if (data[1] & 0x01) != 1:
                    continue

                quality = data[0] >> 2
                angle = ((data[1] >> 1) | (data[2] << 7)) / 64.0
                dist = (data[3] | (data[4] << 8)) / 4.0

                if s_flag == 1 and len(buf_a) > 50:
                    with self.lock:
                        self.latest_scan = (
                            np.array(buf_a, dtype=np.float32),
                            np.array(buf_d, dtype=np.float32),
                            np.array(buf_q, dtype=np.float32)
                        )
                    buf_a, buf_d, buf_q = [], [], []

                if dist > 0 and quality > 0:
                    buf_a.append(angle)
                    buf_d.append(dist)
                    buf_q.append(quality)

            except (serial.SerialException, OSError) as e:
                print(f"[LIDAR] Serial Error: {e}, retrying in 1 second...")
                time.sleep(1.0)
                try:
                    self.ser.reset_input_buffer()
                except:
                    pass

    def get_scan(self):
        with self.lock:
            return self.latest_scan

    def close(self):
        self.running = False
        try:
            self.ser.write(bytes([0xA5, 0x25]))
        except:
            pass
        time.sleep(0.1)
        self.ser.close()


def normalize_angle_deg(angle):
    angle = (angle + 180.0) % 360.0 - 180.0
    return angle


def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def update_pose(v, w, dt):
    global robot_x, robot_y, robot_theta

    robot_x += v * math.cos(robot_theta) * dt
    robot_y += v * math.sin(robot_theta) * dt
    robot_theta += w * dt
    robot_theta = normalize_angle_rad(robot_theta)


def goal_distance():
    dx = GOAL_X_M - robot_x
    dy = GOAL_Y_M - robot_y
    return math.hypot(dx, dy)


def goal_heading_error_from_pose(x, y, theta):
    dx = GOAL_X_M - x
    dy = GOAL_Y_M - y
    target_heading = math.atan2(dy, dx)
    return normalize_angle_rad(target_heading - theta)


def transform_local_to_global(local_x, local_y):
    gx = robot_x + (local_x * math.cos(robot_theta) - local_y * math.sin(robot_theta))
    gy = robot_y + (local_x * math.sin(robot_theta) + local_y * math.cos(robot_theta))
    return gx, gy


def lidar_points_to_xy(scan):
    if scan is None:
        return np.empty((0, 2), dtype=np.float32)

    angles, dists, qualities = scan

    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG)
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    mask = (
        (dist_m >= MIN_LIDAR_DIST_M) &
        (dist_m <= MAX_LIDAR_DIST_M) &
        (qualities >= MIN_QUALITY)
    )

    if not mask.any():
        return np.empty((0, 2), dtype=np.float32)

    dist_m = dist_m[mask]
    angle_rad = np.deg2rad(angle_deg[mask])

    x = dist_m * np.cos(angle_rad)
    y = dist_m * np.sin(angle_rad)

    mask_xy = x >= MIN_X_FOR_PLANNING

    if not mask_xy.any():
        return np.empty((0, 2), dtype=np.float32)

    points = np.column_stack((x[mask_xy], y[mask_xy])).astype(np.float32)

    if len(points) > MAX_EVAL_POINTS:
        order = np.argsort(points[:, 0] ** 2 + points[:, 1] ** 2)
        points = points[order[:MAX_EVAL_POINTS]]

    return points


def predict_trajectory(v, w):
    n_steps = int(PREDICT_TIME / PREDICT_DT)
    ts = (np.arange(n_steps) + 1) * PREDICT_DT
    thetas = w * ts

    if abs(w) > 1e-6:
        xs = (v / w) * np.sin(thetas)
        ys = (v / w) * (1.0 - np.cos(thetas))
    else:
        xs = v * ts
        ys = np.zeros(n_steps)

    return np.column_stack((xs, ys, thetas)).astype(np.float32)


def front_distance(points):
    if len(points) == 0:
        return MAX_LIDAR_DIST_M

    mask = (
        (points[:, 0] > 0.0) &
        (points[:, 0] < ACTIVE_FRONT_DIST) &
        (np.abs(points[:, 1]) < FRONT_CORRIDOR_HALF)
    )

    if not mask.any():
        return MAX_LIDAR_DIST_M

    return float(np.min(points[mask, 0]))


def side_distances(points):
    """
    좌우 가까운 장애물 거리 계산.
    기존 코드는 |x| < 0.15만 봐서 로봇 바로 옆 장애물만 감지했음.
    수정 후에는 로봇 앞쪽 좌우 장애물까지 미리 감지함.
    """

    info_left = 1.0
    info_right = 1.0

    if len(points) == 0:
        return info_left, info_right

    side_mask = (
        (points[:, 0] > SIDE_LOOK_X_BACK) &
        (points[:, 0] < SIDE_LOOK_X_FRONT) &
        (np.abs(points[:, 1]) < SIDE_LOOK_Y)
    )

    if not side_mask.any():
        return info_left, info_right

    side_points = points[side_mask]

    left_points = side_points[side_points[:, 1] > 0.04]
    right_points = side_points[side_points[:, 1] < -0.04]

    if len(left_points) > 0:
        info_left = float(np.min(left_points[:, 1]))

    if len(right_points) > 0:
        info_right = float(-np.max(right_points[:, 1]))

    return info_left, info_right


def trajectory_clearances(traj, points):
    if len(points) == 0:
        return MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M

    traj_xy = traj[:, :2]
    theta = traj[:, 2]

    diff = points[None, :, :] - traj_xy[:, None, :]

    dx = diff[:, :, 0]
    dy = diff[:, :, 1]

    dist = np.sqrt(dx ** 2 + dy ** 2)

    c = np.cos(theta)[:, None]
    s = np.sin(theta)[:, None]

    rel_x = c * dx + s * dy
    rel_y = -s * dx + c * dy

    front_mask = (
        (rel_x > 0.0) &
        (rel_x < ACTIVE_FRONT_DIST) &
        (np.abs(rel_y) < FRONT_CORRIDOR_HALF)
    )

    if front_mask.any():
        front_clear = float(np.min(dist[front_mask]))
    else:
        front_clear = MAX_LIDAR_DIST_M

    side_mask = ~front_mask

    if side_mask.any():
        side_clear = float(np.min(dist[side_mask]))
    else:
        side_clear = MAX_LIDAR_DIST_M

    body_clear = float(np.min(dist))

    return front_clear, side_clear, body_clear


def evaluate_candidate(v, w, points, prev_w, front_dist):
    traj = predict_trajectory(v, w)
    front_clearance, side_clearance, body_clearance = trajectory_clearances(traj, points)

    max_abs_w = max(abs(wc) for wc in W_CANDIDATES)

    # 전방 위험도 계산
    denom = max(1e-6, ACTIVE_FRONT_DIST - COLLISION_DIST)
    front_factor = float(np.clip((ACTIVE_FRONT_DIST - front_dist) / denom, 0.0, 1.0))

    forward_w = forward_weight + (1.0 - front_factor) * far_forward_weight
    turn_w = turn_weight + (1.0 - front_factor) * far_turn_weight

    score = 0.0

    score += clearance_weight * min(front_clearance, CLEARANCE_CAP)
    score += side_clearance_weight * min(side_clearance, CLEARANCE_CAP)

    if front_clearance < COLLISION_DIST:
        score -= collision_weight * (COLLISION_DIST - front_clearance + 1.0)

    if side_clearance < COLLISION_DIST:
        score -= side_collision_weight * (COLLISION_DIST - side_clearance + 1.0)
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

    heading_err = goal_heading_error_from_pose(candidate_x, candidate_y, candidate_theta)
    lateral_err = candidate_y - GOAL_Y_M

    current_goal_dist = goal_distance()
    candidate_goal_dist = math.hypot(GOAL_X_M - candidate_x, GOAL_Y_M - candidate_y)

    goal_progress = current_goal_dist - candidate_goal_dist
    goal_factor = 1.0 - front_factor

    theta_abs = abs(candidate_theta)
    theta_excess = max(0.0, theta_abs - TURN_SOFT_LIMIT_RAD)
    theta_growth = max(0.0, theta_abs - abs(robot_theta))

    score += goal_factor * GOAL_DISTANCE_WEIGHT * goal_progress
    score -= goal_factor * GOAL_HEADING_WEIGHT * abs(heading_err)
    score -= goal_factor * GOAL_LATERAL_WEIGHT * abs(lateral_err)

    score -= TURN_LIMIT_WEIGHT * theta_excess * theta_excess

    if abs(robot_theta) > TURN_SOFT_LIMIT_RAD:
        score -= TURN_GROWTH_WEIGHT * theta_growth

    if theta_abs > TURN_HARD_LIMIT_RAD:
        score -= collision_weight * (theta_abs - TURN_HARD_LIMIT_RAD + 1.0)

    return score, front_clearance, side_clearance, body_clearance, candidate_theta


def apply_side_avoidance_score(score, clear_score, w, info_left, info_right):
    """
    좌우 장애물이 가까울 때 점수 보정.
    - 왼쪽 장애물이 가까운데 왼쪽으로 돌면 강한 감점
    - 오른쪽 장애물이 가까운데 오른쪽으로 돌면 강한 감점
    - 가까운 장애물의 반대 방향으로 돌면 약간 가점
    """

    if info_left < SIDE_AVOID_DIST:
        closeness = (SIDE_AVOID_DIST - info_left) / SIDE_AVOID_DIST
        closeness = float(np.clip(closeness, 0.0, 1.0))

        if w > 0:
            # 왼쪽에 장애물이 있는데 왼쪽 회전하면 위험
            score -= SIDE_TURN_INTO_PENALTY * closeness * (1.0 + abs(w))
            clear_score -= 1.2 * closeness * (1.0 + abs(w))
        elif w < 0:
            # 왼쪽 장애물을 피하려면 오른쪽 회전이 유리
            score += SIDE_TURN_AWAY_BONUS * closeness * abs(w)
            clear_score += 0.4 * closeness * abs(w)
        else:
            # 가까운데 직진도 약간 위험
            score -= SIDE_STRAIGHT_NEAR_PENALTY * closeness
            clear_score -= 0.5 * closeness

    if info_right < SIDE_AVOID_DIST:
        closeness = (SIDE_AVOID_DIST - info_right) / SIDE_AVOID_DIST
        closeness = float(np.clip(closeness, 0.0, 1.0))

        if w < 0:
            # 오른쪽에 장애물이 있는데 오른쪽 회전하면 위험
            score -= SIDE_TURN_INTO_PENALTY * closeness * (1.0 + abs(w))
            clear_score -= 1.2 * closeness * (1.0 + abs(w))
        elif w > 0:
            # 오른쪽 장애물을 피하려면 왼쪽 회전이 유리
            score += SIDE_TURN_AWAY_BONUS * closeness * abs(w)
            clear_score += 0.4 * closeness * abs(w)
        else:
            # 가까운데 직진도 약간 위험
            score -= SIDE_STRAIGHT_NEAR_PENALTY * closeness
            clear_score -= 0.5 * closeness

    return score, clear_score


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
            "points": 0,
            "collision": False,
            "raw_w": 0.0,
            "cth": robot_theta,
            "left": 1.0,
            "right": 1.0,
            "urgent": False,
        }

    fdist = front_distance(points)
    info_left, info_right = side_distances(points)

    best_w = 0.0
    best_score = -float("inf")
    best_clearance = -float("inf")
    best_side_clearance = MAX_LIDAR_DIST_M
    best_body_clearance = MAX_LIDAR_DIST_M
    best_theta = 0.0

    all_collision = True

    best_clear_w = 0.0
    best_clear_score = -float("inf")

    for w in W_CANDIDATES:
        score, clearance, side_clearance, body_clearance, candidate_theta = (
            evaluate_candidate(cmd_v, w, points, prev_w, fdist)
        )

        collision = clearance < COLLISION_DIST

        if not collision:
            all_collision = False

        theta_abs = abs(candidate_theta)
        theta_excess = max(0.0, theta_abs - TURN_SOFT_LIMIT_RAD)
        theta_growth = max(0.0, theta_abs - abs(robot_theta))

        clear_score = (
            clearance
            + 0.18 * side_clearance
            + 0.03 * abs(w)
            - 0.02 * abs(w - prev_w)
        )

        clear_score -= 0.20 * theta_excess

        if abs(robot_theta) > TURN_SOFT_LIMIT_RAD:
            clear_score -= 0.12 * theta_growth

        # 좌우 장애물 회피 점수 보정
        score, clear_score = apply_side_avoidance_score(
            score, clear_score, w, info_left, info_right
        )

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

    # 모든 후보가 충돌 위험이면 그나마 가장 여유 있는 방향 선택
    if all_collision:
        best_w = best_clear_w
        best_score, best_clearance, best_side_clearance, best_body_clearance, best_theta = (
            evaluate_candidate(cmd_v, best_w, points, prev_w, fdist)
        )

    # 좌우 장애물이 매우 가까우면 가까운 쪽 반대 방향을 강하게 선택
    # 전진 속도는 건드리지 않고 w만 수정
    if min(info_left, info_right) < SIDE_HARD_DIST:
        if info_left < info_right:
            # 왼쪽이 더 가까우면 오른쪽 회전
            best_w = -0.70
        elif info_right < info_left:
            # 오른쪽이 더 가까우면 왼쪽 회전
            best_w = 0.70
        else:
            # 양쪽이 비슷하면 현재 회전 방향의 반대쪽으로 틀기
            if prev_w >= 0:
                best_w = -0.70
            else:
                best_w = 0.70

        best_score, best_clearance, best_side_clearance, best_body_clearance, best_theta = (
            evaluate_candidate(cmd_v, best_w, points, prev_w, fdist)
        )

    raw_best_w = best_w

    side_urgent = (info_left < SIDE_AVOID_DIST) or (info_right < SIDE_AVOID_DIST)
    urgent = (fdist < URGENT_FRONT_DIST) or all_collision or side_urgent

    # 우측 피한 직후 좌측 장애물처럼 반대 방향 급전환이 필요한 경우 빠르게 w 변경
    best_w = rate_limit_w(prev_w, best_w, urgent)

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
        "urgent": urgent,
    }


def main():
    global robot_x, robot_y, robot_theta

    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    ardu = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)

    print("[INFO] Warming up for 2 seconds...")
    time.sleep(2.0)

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

                time.sleep(LOOP_DT_S)
                continue

            last_scan_ok = time.time()

            # 전진 속도 BASE_V는 그대로 유지
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
                print(
                    f"[RUN2] x={robot_x:.2f} y={robot_y:.2f} "
                    f"th={robot_theta:.2f} gd={gd:.2f} he={he:.2f} "
                    f"v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                    f"front={info['front']:.2f} clear={info['clear']:.2f} "
                    f"side={info['side']:.2f} body={info['body']:.2f} "
                    f"score={info['score']:.2f} pts={info['points']} "
                    f"coll={int(info['collision'])} cth={info['cth']:.2f} "
                    f"L={info.get('left', -1):.2f} R={info.get('right', -1):.2f} "
                    f"urgent={int(info.get('urgent', False))}"
                )
                last_log = time.time()

            time.sleep(LOOP_DT_S)

    except KeyboardInterrupt:
        pass

    finally:
        stop()
        time.sleep(0.2)
        ardu.close()
        lidar.close()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
