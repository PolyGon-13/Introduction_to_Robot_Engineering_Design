#!/usr/bin/env python3
# maze2.py
# RPLiDAR C1 + 후보 각속도 평가 기반 로컬 플래너
# LiDAR Occupancy Map Visualization Version
# 화면 기준: 로봇 정면 = 화면 위쪽

import serial
import time
import math
import threading
import numpy as np

try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ModuleNotFoundError:
    plt = None
    MATPLOTLIB_AVAILABLE = False


# ─────────────── 사용자 튜닝 파라미터 ───────────────
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
ARDU_PORT  = "/dev/ttyS0"
ARDU_BAUD  = 9600

# 라이다 시각화 옵션
VISUALIZE_LIDAR = True       # True: 시각화 켜기 / False: 시각화 끄기
LIDAR_VIS_RANGE_M = 2.0      # 시각화 범위 2m
LIDAR_VIS_UPDATE_S = 0.10    # 시각화 갱신 주기

# 맵핑 시각화 옵션
MAP_RESOLUTION_M = 0.04      # 격자 한 칸 크기 4cm
WALL_INFLATE_CELL = 1        # 벽 두께 보정. 1이면 주변 1칸까지 벽처럼 표시

# 시각화 정면 보정값
# 화면에서 정면이 약간 틀어져 보이면 이 값만 조정
# 예: 정면 벽이 화면에서 오른쪽으로 기울면 -값, 왼쪽으로 기울면 +값을 조금씩 조정
VIS_FRONT_ROT_DEG = 0.0

# 라이다 캘리브레이션 (확정값)
ANGLE_OFFSET_DEG = +1.54
DIST_OFFSET_MM   = 0.0
LIDAR_ANGLE_SIGN = -1.0

# 좌표계: +각도/+y = 좌측, -각도/-y = 우측

# 라이다 전처리 파라미터
MIN_LIDAR_DIST_M = 0.05
MAX_LIDAR_DIST_M = 2.5
MIN_QUALITY = 1
MAX_EVAL_POINTS = 720
SCAN_HOLD_S = 0.30
LOOP_DT_S = 0.05

# 로컬 플래너 파라미터
BASE_V = 0.18
W_CANDIDATES = [-0.45, -0.30, -0.18, 0.0,
                0.18, 0.30, 0.45]
PREDICT_TIME = 1.20
PREDICT_DT = 0.10
ROBOT_RADIUS = 0.14
SAFETY_MARGIN = 0.14
COLLISION_DIST = ROBOT_RADIUS + SAFETY_MARGIN
CLEARANCE_CAP = 1.0
FRONT_CORRIDOR_HALF = COLLISION_DIST
ACTIVE_FRONT_DIST = 0.50
SIDE_NEAR_DIST = COLLISION_DIST + 0.12
W_CMD_RATE_LIMIT = 0.35
W_CMD_RATE_LIMIT_URGENT = 0.70
URGENT_FRONT_DIST = 0.55

# 전방 장애물 강제 제자리 회전 파라미터
# True  : 전방이 막혔고 좌우 중 한쪽이 가까울 때 v=0, w만 제어
# False : 해당 특수 w제어를 끄고 기존 후보 평가 방식으로만 주행
USE_FRONT_BLOCKED_W_CONTROL = False

FRONT_TURN_TRIGGER_DIST = 0.50
FRONT_TURN_W = 0.75
FRONT_TURN_SIDE_TRIGGER_DIST = 0.20

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
USE_GOAL_SLOWDOWN = False
GOAL_FINAL_SLOW_DIST = 0.30
GOAL_MIN_V = 0.08

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


# ─────────────── 라이다 드라이버 ───────────────
class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.ser.write(bytes([0xA5, 0x40]))
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0xA5, 0x20]))
        self.ser.read(7)
        self.lock = threading.Lock()
        self.latest_scan = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buf_a, buf_d, buf_q = [], [], []
        while self.running:
            data = self.ser.read(5)
            if len(data) != 5:
                continue

            s_flag = data[0] & 0x01
            s_inv  = (data[0] & 0x02) >> 1

            if s_inv != (1 - s_flag):
                continue
            if (data[1] & 0x01) != 1:
                continue

            quality = data[0] >> 2
            angle  = ((data[1] >> 1) | (data[2] << 7)) / 64.0
            dist   = (data[3] | (data[4] << 8)) / 4.0

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
    gx = robot_x + local_x * math.cos(robot_theta) - local_y * math.sin(robot_theta)
    gy = robot_y + local_x * math.sin(robot_theta) + local_y * math.cos(robot_theta)
    return gx, gy


def get_cmd_v():
    if USE_GOAL_SLOWDOWN:
        gd = goal_distance()
        if gd < GOAL_FINAL_SLOW_DIST:
            return max(GOAL_MIN_V, BASE_V * gd / GOAL_FINAL_SLOW_DIST)
    return BASE_V


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

    points = np.column_stack((x, y)).astype(np.float32)

    if len(points) > MAX_EVAL_POINTS:
        order = np.argsort(points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1])
        points = points[order[:MAX_EVAL_POINTS]]

    return points


def convert_points_for_map_view(points):
    """
    주행 좌표계:
        x = 로봇 정면
        y = 로봇 왼쪽

    화면 좌표계:
        map_x = 화면 오른쪽
        map_y = 화면 위쪽

    따라서:
        정면 x → 화면 위쪽 map_y
        왼쪽 y → 화면 왼쪽 -map_x
    """
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float32)

    x_front = points[:, 0]
    y_left = points[:, 1]

    # 시각화용 회전 보정
    rot = math.radians(VIS_FRONT_ROT_DEG)
    cos_r = math.cos(rot)
    sin_r = math.sin(rot)

    x_rot = x_front * cos_r - y_left * sin_r
    y_rot = x_front * sin_r + y_left * cos_r

    # 화면에서는 정면이 위쪽으로 보이게 변환
    map_x = -y_rot
    map_y = x_rot

    return np.column_stack((map_x, map_y)).astype(np.float32)


# ─────────────── 라이다 맵핑 시각화 ───────────────
class LidarMapVisualizer:
    def __init__(self, range_m=2.0, update_s=0.10, resolution=0.04):
        self.range_m = range_m
        self.update_s = update_s
        self.resolution = resolution
        self.last_update = 0.0

        self.grid_size = int((2.0 * self.range_m) / self.resolution) + 1
        self.center = self.grid_size // 2

        plt.ion()
        self.fig, self.ax = plt.subplots()

        initial_grid = np.full((self.grid_size, self.grid_size), 0.5, dtype=np.float32)

        self.img = self.ax.imshow(
            initial_grid,
            origin="lower",
            extent=[-self.range_m, self.range_m, -self.range_m, self.range_m],
            vmin=0.0,
            vmax=1.0,
            cmap="gray_r"
        )

        # 로봇 위치 표시: 로봇은 항상 지도 중앙, 화면 위쪽이 정면
        self.ax.plot(0.0, 0.0, marker="^", markersize=10)
        self.ax.arrow(
            0.0, 0.0,
            0.0, 0.25,
            head_width=0.07,
            head_length=0.08,
            length_includes_head=True
        )

        # 2m 범위 원 표시
        circle = plt.Circle((0, 0), self.range_m, fill=False, linestyle="--")
        self.ax.add_patch(circle)

        self.ax.set_aspect("equal", "box")
        self.ax.set_xlim(-self.range_m, self.range_m)
        self.ax.set_ylim(-self.range_m, self.range_m)
        self.ax.set_xlabel("Left / Right distance [m]")
        self.ax.set_ylabel("Front / Back distance [m]")
        self.ax.set_title("LiDAR Occupancy Map - Front is Up")
        self.ax.grid(True)

    def xy_to_cell(self, x, y):
        col = int(round((x + self.range_m) / self.resolution))
        row = int(round((y + self.range_m) / self.resolution))

        if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
            return row, col

        return None

    def bresenham(self, row0, col0, row1, col1):
        cells = []

        dx = abs(col1 - col0)
        dy = -abs(row1 - row0)

        sx = 1 if col0 < col1 else -1
        sy = 1 if row0 < row1 else -1

        err = dx + dy

        row = row0
        col = col0

        while True:
            cells.append((row, col))

            if row == row1 and col == col1:
                break

            e2 = 2 * err

            if e2 >= dy:
                err += dy
                col += sx

            if e2 <= dx:
                err += dx
                row += sy

        return cells

    def build_occupancy_grid(self, points):
        # 0.0 = 빈 공간, 0.5 = 미확인 공간, 1.0 = 벽/장애물
        grid = np.full((self.grid_size, self.grid_size), 0.5, dtype=np.float32)

        robot_cell = self.xy_to_cell(0.0, 0.0)
        if robot_cell is None:
            return grid

        robot_row, robot_col = robot_cell

        if len(points) == 0:
            return grid

        # 주행 좌표계를 화면 좌표계로 변환
        map_points = convert_points_for_map_view(points)

        dist = np.sqrt(map_points[:, 0] ** 2 + map_points[:, 1] ** 2)
        vis_points = map_points[dist <= self.range_m]

        for p in vis_points:
            x = float(p[0])
            y = float(p[1])

            cell = self.xy_to_cell(x, y)
            if cell is None:
                continue

            hit_row, hit_col = cell

            ray_cells = self.bresenham(robot_row, robot_col, hit_row, hit_col)

            # 라이다 광선이 지나간 공간은 빈 공간으로 표시
            for row, col in ray_cells[:-1]:
                if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                    grid[row, col] = 0.0

            # 마지막 감지점은 벽/장애물로 표시
            for dr in range(-WALL_INFLATE_CELL, WALL_INFLATE_CELL + 1):
                for dc in range(-WALL_INFLATE_CELL, WALL_INFLATE_CELL + 1):
                    rr = hit_row + dr
                    cc = hit_col + dc

                    if 0 <= rr < self.grid_size and 0 <= cc < self.grid_size:
                        grid[rr, cc] = 1.0

        # 로봇 주변은 빈 공간으로 처리
        robot_clear_cells = int(ROBOT_RADIUS / self.resolution) + 1

        for dr in range(-robot_clear_cells, robot_clear_cells + 1):
            for dc in range(-robot_clear_cells, robot_clear_cells + 1):
                rr = robot_row + dr
                cc = robot_col + dc

                if 0 <= rr < self.grid_size and 0 <= cc < self.grid_size:
                    if dr * dr + dc * dc <= robot_clear_cells * robot_clear_cells:
                        grid[rr, cc] = 0.0

        return grid

    def update(self, points, info=None):
        now = time.time()
        if now - self.last_update < self.update_s:
            return

        self.last_update = now

        grid = self.build_occupancy_grid(points)
        self.img.set_data(grid)

        if info is not None:
            self.ax.set_title(
                f"LiDAR Occupancy Map - Front is Up | "
                f"front={info['front']:.2f}m clear={info['clear']:.2f}m "
                f"pts={info['points']} rot={VIS_FRONT_ROT_DEG:.1f}deg"
            )

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def close(self):
        plt.ioff()
        plt.close(self.fig)


def predict_trajectory(v, w):
    traj = []
    x = 0.0
    y = 0.0
    theta = 0.0
    t = 0.0

    while t < PREDICT_TIME:
        x += v * math.cos(theta) * PREDICT_DT
        y += v * math.sin(theta) * PREDICT_DT
        theta += w * PREDICT_DT
        traj.append((x, y, theta))
        t += PREDICT_DT

    return np.array(traj, dtype=np.float32)


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


def sector_min_distance(points, angle_min_deg, angle_max_deg):
    if len(points) == 0:
        return MAX_LIDAR_DIST_M

    angles = np.rad2deg(np.arctan2(points[:, 1], points[:, 0]))
    dists = np.sqrt(points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1])

    mask = (
        (angles >= angle_min_deg) &
        (angles <= angle_max_deg)
    )

    if not mask.any():
        return MAX_LIDAR_DIST_M

    return float(np.min(dists[mask]))


def is_dead_end(points):
    front_blocked = sector_min_distance(points, -20.0, 20.0) <= 0.50
    left45_blocked = sector_min_distance(points, 25.0, 75.0) <= 0.15
    right45_blocked = sector_min_distance(points, -75.0, -25.0) <= 0.15

    return front_blocked and left45_blocked and right45_blocked


def front20_blocked(points):
    if len(points) == 0:
        return False

    angles = np.rad2deg(np.arctan2(points[:, 1], points[:, 0]))
    dists = np.sqrt(points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1])

    front_mask = (
        (angles >= -15.0) &
        (angles <= 15.0)
    )

    if not front_mask.any():
        return False

    return np.all(dists[front_mask] <= FRONT_TURN_TRIGGER_DIST)


def front_turn_side_close(points):
    left_dist = sector_min_distance(points, 25.0, 90.0)
    right_dist = sector_min_distance(points, -90.0, -25.0)

    return (left_dist < FRONT_TURN_SIDE_TRIGGER_DIST) or (right_dist < FRONT_TURN_SIDE_TRIGGER_DIST)


def choose_front20_turn(points):
    left_dist = sector_min_distance(points, 25.0, 90.0)
    right_dist = sector_min_distance(points, -90.0, -25.0)

    # 좌측 장애물이 더 멀면 좌회전(w +), 우측 장애물이 더 멀면 우회전(w -)
    if left_dist >= right_dist:
        return FRONT_TURN_W, left_dist, right_dist
    else:
        return -FRONT_TURN_W, left_dist, right_dist


def trajectory_clearances(traj, points):
    if len(points) == 0:
        return MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M

    traj_xy = traj[:, :2]
    theta = traj[:, 2]

    diff = points[None, :, :] - traj_xy[:, None, :]
    dx = diff[:, :, 0]
    dy = diff[:, :, 1]
    dist = np.sqrt(dx * dx + dy * dy)

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

    front_factor = float(
        np.clip(
            (ACTIVE_FRONT_DIST - front_dist) /
            max(1e-6, ACTIVE_FRONT_DIST - COLLISION_DIST),
            0.0,
            1.0
        )
    )

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
        }

    fdist = front_distance(points)

    # 전방 막힘 시 w만 제어하는 특수 로직
    # USE_FRONT_BLOCKED_W_CONTROL = True일 때만 작동
    # False로 바꾸면 이 조건은 무시되고 아래 기존 후보 평가 방식으로 주행
    if USE_FRONT_BLOCKED_W_CONTROL and front20_blocked(points) and front_turn_side_close(points):
        turn_w, left_dist, right_dist = choose_front20_turn(points)

        dists = np.sqrt(points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1])
        body_clearance = float(np.min(dists))

        return 0.0, turn_w, {
            "score": 0.0,
            "clear": fdist,
            "side": min(left_dist, right_dist),
            "body": body_clearance,
            "front": fdist,
            "points": len(points),
            "collision": fdist < COLLISION_DIST,
            "raw_w": turn_w,
            "cth": robot_theta,
        }

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
            evaluate_candidate(cmd_v, w, points, prev_w, fdist)
        )

        collision = clearance < COLLISION_DIST

        if not collision:
            all_collision = False

        theta_abs = abs(candidate_theta)
        theta_excess = max(0.0, theta_abs - TURN_SOFT_LIMIT_RAD)
        theta_growth = max(0.0, theta_abs - abs(robot_theta))

        clear_score = (
            clearance +
            0.18 * side_clearance +
            0.03 * abs(w) -
            0.02 * abs(w - prev_w)
        )

        clear_score -= 0.20 * theta_excess

        if abs(robot_theta) > TURN_SOFT_LIMIT_RAD:
            clear_score -= 0.12 * theta_growth

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
            evaluate_candidate(cmd_v, best_w, points, prev_w, fdist)
        )

    raw_best_w = best_w
    best_w = rate_limit_w(prev_w, best_w, fdist < URGENT_FRONT_DIST or all_collision)

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
    }


# ─────────────── 메인 ───────────────
def main():
    global robot_x, robot_y, robot_theta

    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    ardu  = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)

    visualizer = None

    if VISUALIZE_LIDAR:
        if MATPLOTLIB_AVAILABLE:
            try:
                visualizer = LidarMapVisualizer(
                    LIDAR_VIS_RANGE_M,
                    LIDAR_VIS_UPDATE_S,
                    MAP_RESOLUTION_M
                )
            except Exception as e:
                print("[WARN] 라이다 시각화 창을 열 수 없습니다.")
                print(f"[WARN] 원인: {e}")
                print("[WARN] 시각화 없이 주행합니다.")
                visualizer = None
        else:
            print("[WARN] matplotlib가 설치되어 있지 않아 라이다 시각화를 끕니다.")
            print("[WARN] 설치 명령어: sudo apt install -y python3-matplotlib")

    print("[INFO] 워밍업 2초...")
    time.sleep(2.0)

    def send_vw(v, w):
        ardu.write(f"V{v:.3f},{w:.3f}\n".encode())

    def stop():
        ardu.write(b"S\n")

    stop()

    print("[READY] 초기화 완료. Enter를 누르면 후보 평가형 로컬 플래너 주행 시작.")

    try:
        input()
    except EOFError:
        print("[WARN] 표준입력을 읽을 수 없어 바로 주행 시작")

    print("[GO]")

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
                print("[INFO] 목표 지점 도달 정지")
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
                    print("[INFO] 목표 지점 도달 정지")
                    break

                time.sleep(LOOP_DT_S)
                continue

            last_scan_ok = time.time()

            cmd_v = get_cmd_v()
            v, w, info = choose_best_cmd(scan, last_w, cmd_v)

            if visualizer is not None:
                vis_points = lidar_points_to_xy(scan)
                visualizer.update(vis_points, info)

            send_vw(v, w)
            update_pose(v, w, dt)

            last_v, last_w = v, w

            gd = goal_distance()
            he = goal_heading_error_from_pose(robot_x, robot_y, robot_theta)

            if gd <= GOAL_TOL_M:
                stop()
                print("[INFO] 목표 지점 도달 정지")
                break

            if time.time() - last_log > 0.25:
                print(
                    f"[RUN2] x={robot_x:.2f} y={robot_y:.2f} "
                    f"th={robot_theta:.2f} gd={gd:.2f} he={he:.2f} "
                    f"v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                    f"front={info['front']:.2f} clear={info['clear']:.2f} "
                    f"side={info['side']:.2f} body={info['body']:.2f} "
                    f"score={info['score']:.2f} pts={info['points']} "
                    f"coll={int(info['collision'])} cth={info['cth']:.2f}"
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

        if visualizer is not None:
            visualizer.close()

        print("[INFO] 종료")


if __name__ == "__main__":
    main()
