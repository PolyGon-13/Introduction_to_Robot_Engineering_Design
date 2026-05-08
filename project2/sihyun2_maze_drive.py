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
VISUALIZE_LIDAR = True
LIDAR_VIS_RANGE_M = 2.0
LIDAR_VIS_UPDATE_S = 0.10

# 맵핑 시각화 옵션
MAP_RESOLUTION_M = 0.04
WALL_INFLATE_CELL = 1

# 시각화 정면 보정값
VIS_FRONT_ROT_DEG = 0.0

# 라이다 캘리브레이션
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

# 너무 큰 회전 후보를 줄여서 좌우로 흔들리는 현상 완화
W_CANDIDATES = [-0.50, -0.35, -0.20, -0.10, 0.0,
                0.10, 0.20, 0.35, 0.50]

# 너무 멀리 예측하면 통로까지 위험하다고 판단하므로 약간 줄임
PREDICT_TIME = 1.30
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
USE_FRONT_BLOCKED_W_CONTROL = False

FRONT_TURN_TRIGGER_DIST = 0.50
FRONT_TURN_W = 0.75
FRONT_TURN_SIDE_TRIGGER_DIST = 0.20


# ─────────────── 막다른길 / 좁은 포켓 회피 파라미터 ───────────────
USE_DEAD_END_AVOIDANCE = True

# 너무 길게 보면 장애물 사이 통로도 막다른길로 오해하므로 줄임
DEAD_END_LOOKAHEAD_TIME = 1.60
DEAD_END_LOOKAHEAD_DT = 0.10

# 감점 강도를 낮춰서 회전만 하는 현상 완화
DEAD_END_CLEARANCE_MARGIN = 0.03
DEAD_END_PENALTY_WEIGHT = 35.0

# 진짜 막힌 경우만 막다른길로 판단하도록 기준 완화
DEAD_END_FRONT_DIST = 0.38
DEAD_END_SIDE_DIST = COLLISION_DIST - 0.03
DEAD_END_ESCAPE_W = 0.50

# 벽과 장애물 사이 좁은 포켓 감지용
POCKET_SIDE_WALL_DIST = COLLISION_DIST + 0.12
POCKET_DIAGONAL_OBS_DIST = 0.48
POCKET_FRONT_IGNORE_DIST = 0.30
POCKET_MIN_W = 0.12
POCKET_PENALTY_WEIGHT = 25.0
POCKET_NEAR_WALL_WEIGHT = 3.5
POCKET_TURN_AWAY_BONUS = 2.5


# ─────────────── 장애물 사이 통과 모드 파라미터 ───────────────
# True : 좌우에 장애물이 있어도 가운데 폭이 충분하면 회전만 하지 않고 전진
USE_PASSAGE_MODE = True

# 전방이 이 거리 이상 비어 있으면 통로로 판단 가능
PASSAGE_FRONT_FREE_DIST = 0.35

# 통로 중앙 정렬 제어
PASSAGE_CENTER_KP = 0.85

# 통로 통과 시 최대 회전값
PASSAGE_MAX_W = 0.18

# 통로 통과 시 전진 속도
PASSAGE_V = BASE_V * 0.90


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
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float32)

    x_front = points[:, 0]
    y_left = points[:, 1]

    rot = math.radians(VIS_FRONT_ROT_DEG)
    cos_r = math.cos(rot)
    sin_r = math.sin(rot)

    x_rot = x_front * cos_r - y_left * sin_r
    y_rot = x_front * sin_r + y_left * cos_r

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

        self.ax.plot(0.0, 0.0, marker="^", markersize=10)
        self.ax.arrow(
            0.0, 0.0,
            0.0, 0.25,
            head_width=0.07,
            head_length=0.08,
            length_includes_head=True
        )

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
        grid = np.full((self.grid_size, self.grid_size), 0.5, dtype=np.float32)

        robot_cell = self.xy_to_cell(0.0, 0.0)
        if robot_cell is None:
            return grid

        robot_row, robot_col = robot_cell

        if len(points) == 0:
            return grid

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

            for row, col in ray_cells[:-1]:
                if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                    grid[row, col] = 0.0

            for dr in range(-WALL_INFLATE_CELL, WALL_INFLATE_CELL + 1):
                for dc in range(-WALL_INFLATE_CELL, WALL_INFLATE_CELL + 1):
                    rr = hit_row + dr
                    cc = hit_col + dc

                    if 0 <= rr < self.grid_size and 0 <= cc < self.grid_size:
                        grid[rr, cc] = 1.0

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


def predict_trajectory(v, w, predict_time=None, predict_dt=None):
    if predict_time is None:
        predict_time = PREDICT_TIME

    if predict_dt is None:
        predict_dt = PREDICT_DT

    traj = []
    x = 0.0
    y = 0.0
    theta = 0.0
    t = 0.0

    while t < predict_time:
        x += v * math.cos(theta) * predict_dt
        y += v * math.sin(theta) * predict_dt
        theta += w * predict_dt
        traj.append((x, y, theta))
        t += predict_dt

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
    """
    진짜 막다른길일 때만 True.
    일반 장애물 사이 통로에서는 True가 되면 안 됨.
    """
    front_blocked = sector_min_distance(points, -25.0, 25.0) <= DEAD_END_FRONT_DIST
    left_blocked = sector_min_distance(points, 45.0, 110.0) <= DEAD_END_SIDE_DIST
    right_blocked = sector_min_distance(points, -110.0, -45.0) <= DEAD_END_SIDE_DIST

    return front_blocked and left_blocked and right_blocked


def side_pocket_status(points):
    """
    벽 + 대각선 장애물 사이의 좁은 막다른 포켓 감지.
    단, 실제 감점은 evaluate_candidate 안에서
    전방이 진짜 막혔을 때만 적용한다.
    """
    left_wall = sector_min_distance(points, 75.0, 115.0)
    right_wall = sector_min_distance(points, -115.0, -75.0)

    left_diag = sector_min_distance(points, 20.0, 70.0)
    right_diag = sector_min_distance(points, -70.0, -20.0)

    front = sector_min_distance(points, -15.0, 15.0)

    left_pocket = (
        left_wall < POCKET_SIDE_WALL_DIST and
        left_diag < POCKET_DIAGONAL_OBS_DIST and
        front > POCKET_FRONT_IGNORE_DIST
    )

    right_pocket = (
        right_wall < POCKET_SIDE_WALL_DIST and
        right_diag < POCKET_DIAGONAL_OBS_DIST and
        front > POCKET_FRONT_IGNORE_DIST
    )

    return left_pocket, right_pocket, left_wall, right_wall, left_diag, right_diag


def choose_dead_end_escape(points):
    """
    실제 막다른길에 들어간 경우,
    좌우 중 더 여유 있는 방향으로 회전한다.
    """
    left_free = sector_min_distance(points, 35.0, 150.0)
    right_free = sector_min_distance(points, -150.0, -35.0)

    if left_free >= right_free:
        return DEAD_END_ESCAPE_W, left_free, right_free
    else:
        return -DEAD_END_ESCAPE_W, left_free, right_free


def passage_width_status(points):
    """
    로봇 전방 구간에서 좌우 장애물 사이 폭을 계산한다.
    좌우에 장애물이 있어도 폭이 충분하고 전방이 열려 있으면 통로로 판단한다.
    """
    if len(points) == 0:
        return False, MAX_LIDAR_DIST_M, 0.0, 0.0, 0.0

    # 로봇 앞 15cm ~ 75cm 구간 확인
    x_min = 0.15
    x_max = 0.75

    band_mask = (
        (points[:, 0] > x_min) &
        (points[:, 0] < x_max)
    )

    if not band_mask.any():
        return False, MAX_LIDAR_DIST_M, 0.0, 0.0, 0.0

    band = points[band_mask]

    left_points = band[band[:, 1] > 0.0]
    right_points = band[band[:, 1] < 0.0]

    if len(left_points) == 0 or len(right_points) == 0:
        return False, MAX_LIDAR_DIST_M, 0.0, 0.0, 0.0

    # 왼쪽에서 가장 가까운 벽/장애물 y값
    left_y = float(np.min(left_points[:, 1]))

    # 오른쪽에서 가장 가까운 벽/장애물 y값
    right_y = float(np.max(right_points[:, 1]))

    width = left_y - right_y

    # 통로 판단은 COLLISION_DIST가 아니라 실제 로봇 지름 기준으로 계산
    # 기존 COLLISION_DIST를 쓰면 너무 보수적이라 장애물 사이에 못 들어감
    required_width = 0.10

    fdist = front_distance(points)

    passable = (
        fdist > PASSAGE_FRONT_FREE_DIST and
        width > required_width
    )

    center_offset = (left_y + right_y) / 2.0

    return passable, width, left_y, right_y, center_offset


def rate_limit_w(prev_w, target_w, urgent=False):
    limit = W_CMD_RATE_LIMIT_URGENT if urgent else W_CMD_RATE_LIMIT
    delta = float(np.clip(target_w - prev_w, -limit, limit))
    return prev_w + delta


def choose_passage_cmd(points, prev_w):
    """
    장애물 사이 통과 모드.
    좌우 폭이 충분하면 회전만 하지 않고 가운데로 정렬하면서 전진한다.
    """
    passable, width, left_y, right_y, center_offset = passage_width_status(points)

    if not passable:
        return None

    # center_offset > 0 이면 통로 중심이 왼쪽 → 좌회전
    # center_offset < 0 이면 통로 중심이 오른쪽 → 우회전
    target_w = PASSAGE_CENTER_KP * center_offset
    target_w = float(np.clip(target_w, -PASSAGE_MAX_W, PASSAGE_MAX_W))

    w = rate_limit_w(prev_w, target_w, urgent=False)

    fdist = front_distance(points)
    dists = np.sqrt(points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1])
    body_clearance = float(np.min(dists))

    return PASSAGE_V, w, {
        "score": 0.0,
        "clear": fdist,
        "side": width,
        "body": body_clearance,
        "front": fdist,
        "points": len(points),
        "collision": False,
        "raw_w": target_w,
        "cth": robot_theta,
    }


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

    # ─────────────── 막다른길 / 좁은 포켓 회피 감점 ───────────────
    if USE_DEAD_END_AVOIDANCE:
        long_traj = predict_trajectory(
            v,
            w,
            predict_time=DEAD_END_LOOKAHEAD_TIME,
            predict_dt=DEAD_END_LOOKAHEAD_DT
        )

        long_front_clearance, long_side_clearance, long_body_clearance = (
            trajectory_clearances(long_traj, points)
        )

        dead_clear = COLLISION_DIST + DEAD_END_CLEARANCE_MARGIN

        if long_body_clearance < dead_clear:
            score -= DEAD_END_PENALTY_WEIGHT * (dead_clear - long_body_clearance + 1.0)

        if long_front_clearance < dead_clear:
            score -= (DEAD_END_PENALTY_WEIGHT * 0.65) * (dead_clear - long_front_clearance + 1.0)

        left_pocket, right_pocket, left_wall, right_wall, left_diag, right_diag = (
            side_pocket_status(points)
        )

        # 핵심 수정:
        # 전방이 실제로 막혀 있을 때만 포켓 감점 적용.
        # 전방이 열려 있으면 장애물 사이 통로일 수 있으므로 감점하지 않음.
        front_really_blocked = front_dist < DEAD_END_FRONT_DIST

        if front_really_blocked:
            if left_pocket and w > POCKET_MIN_W:
                score -= POCKET_PENALTY_WEIGHT * (1.0 + abs(w))

            if right_pocket and w < -POCKET_MIN_W:
                score -= POCKET_PENALTY_WEIGHT * (1.0 + abs(w))

            if left_pocket and w < -POCKET_MIN_W:
                score += POCKET_TURN_AWAY_BONUS * abs(w)

            if right_pocket and w > POCKET_MIN_W:
                score += POCKET_TURN_AWAY_BONUS * abs(w)

            if left_wall < POCKET_SIDE_WALL_DIST and w > POCKET_MIN_W:
                score -= POCKET_NEAR_WALL_WEIGHT * (POCKET_SIDE_WALL_DIST - left_wall) * (1.0 + abs(w))

            if right_wall < POCKET_SIDE_WALL_DIST and w < -POCKET_MIN_W:
                score -= POCKET_NEAR_WALL_WEIGHT * (POCKET_SIDE_WALL_DIST - right_wall) * (1.0 + abs(w))

    return score, front_clearance, side_clearance, body_clearance, candidate_theta


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

    # 1순위: 장애물 사이 통과 가능하면 막다른길 회피보다 먼저 전진 통과
    if USE_PASSAGE_MODE:
        passage_cmd = choose_passage_cmd(points, prev_w)
        if passage_cmd is not None:
            return passage_cmd

    # 2순위: 진짜 막다른길이면 천천히 움직이며 탈출 회전
    if USE_DEAD_END_AVOIDANCE and is_dead_end(points):
        turn_w, left_free, right_free = choose_dead_end_escape(points)

        dists = np.sqrt(points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1])
        body_clearance = float(np.min(dists))

        return 0.04, turn_w, {
            "score": 0.0,
            "clear": fdist,
            "side": max(left_free, right_free),
            "body": body_clearance,
            "front": fdist,
            "points": len(points),
            "collision": True,
            "raw_w": turn_w,
            "cth": robot_theta,
        }

    # 3순위: 전방 막힘 시 w만 제어하는 기존 특수 로직
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
