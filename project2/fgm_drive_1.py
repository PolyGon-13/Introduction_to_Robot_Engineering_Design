#!/usr/bin/env python3

import serial
import time
import math
import threading
import numpy as np

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600


# LiDAR
ANGLE_OFFSET_DEG = 1.54 # 라이다 각도 보정값
DIST_OFFSET_MM = 0.0 # 라이다 거리 보정값
LIDAR_ANGLE_SIGN = -1.0 # 라이다 각도 방향 반전 여부

MIN_LIDAR_DIST_M = 0.05 # 라이다 유효 최소 거리
MAX_LIDAR_DIST_M = 2.5 # 라이다 유효 최대 거리
MIN_QUALITY = 1 # 라이다 품질 최소값
MIN_X_FOR_PLANNING = 0.10 # FGM에 넣을 최소 전방 거리
MAX_EVAL_POINTS = 720 # 계산에 사용할 라이다 포인트 최대 개수
SCAN_HOLD_S = 0.30 # 최근 라이다 스캔을 유효하다고 판단할 시간 (마지막 정상 스캔이 N초 이내면 그냥 사용)

# 속도/거리 관련
BASE_V = 0.18 # FGM 기본 직진속도
MIN_V = 0.15 # FGM 장애물 회피속도
MAX_ABS_W = 0.70 # FGM 최대 회전속도

RECOVERY_MAX_W = 1.00 # Recovery 회전속도

WALL_FOLLOW_V = 0.15 # Wall follow 기본속도
WALL_SLOW_V = 0.10 # # Wall follow 최소속도
WALL_MAX_W = 0.65 # Wall follow 최대 회전속도

CLEARANCE_CAP = 0.6 # FGM - 해당 거리 이상 뚫려 있으면 동일 취급
W_CMD_RATE_LIMIT = 0.30 # 루프당 최대 회전속도 변화량
W_CMD_RATE_LIMIT_URGENT = 0.40 # 긴급 시 허용 변화량
URGENT_FRONT_DIST = 0.30 # 정면이 이 안이면 긴급모드


# 로봇 크기 & 충돌 거리
ROBOT_RADIUS = 0.16 # 로봇 반지름
COLLISION_DIST = ROBOT_RADIUS + 0.05 # 충돌위험 거리
ACTIVE_FRONT_DIST = 0.30 # 정면 장애물 반응 시작 거리
FRONT_DANGER_DIST = 0.19 # 정면 위험 거리
FRONT_CORRIDOR_HALF = COLLISION_DIST + 0.30 # FGM 장애물로 인식할 Y축 범위의 절반

INITIAL_HEADING_RAD = 0.0 # 출발 기준 방향
RECOVERY_INITIAL_DEADBAND_RAD = math.radians(2.0) # 출발 방향 기준 N도 안이면 방향 판단 보류

# 회전 limit
TURN_HARD_LIMIT_RAD = math.radians(80.0) # FGM 최대 목표 각도
CUM_TURN_SOFT_LIMIT_RAD = math.radians(70.0) # 누적 회전량 페널티 기준1
CUM_TURN_HARD_LIMIT_RAD = math.radians(88.0) # 누적 회전량 페널티 기준2
CUM_TURN_SOFT_PENALTY_WEIGHT = 8.0 # 90도 초과 시 페널티 강도
CUM_TURN_HARD_PENALTY_WEIGHT = 50.0 # 100도 초과 시 페널티 강도


# FGM 알고리즘
FGM_MIN_ANGLE_DEG = -90.0 # 우측 라이다 스캔 각도
FGM_MAX_ANGLE_DEG = 90.0 # 좌측 라이다 스캔 각도
FGM_ANGLE_STEP_DEG = 1.0 # 격자 생성 각도
FGM_BUBBLE_RADIUS = ROBOT_RADIUS + 0.08 # 장애물 부풀리는 반경
FGM_FREE_DIST = COLLISION_DIST + 0.04 # 이 이상 뚫려 있어야 지나갈 수 있는 칸으로 인식
FGM_MIN_GAP_WIDTH_DEG = 8.0 # 인식한 Gap의 최소 허용각도

FGM_SMOOTH_WINDOW = 5 # 이동평균 윈도우 크기 (5칸 = +-2도) : 튀는 값 방지 휘해 2도 간격으로 값들의 평균값으로 값을 대체

FGM_TURN_GAIN = 1.05 # 목표 각도->회전명령 가중치
FGM_PREV_TARGET_WEIGHT = 0.45 # 직전 목표 방향 연속성 가중치
FGM_GOAL_WEIGHT_SAFE = 1.00 # 안전할 때 목표 방향 가중치
FGM_GOAL_WEIGHT_DANGER = 0.30 # 위험할 때 목표 방향 가중치
FGM_STRAIGHT_WEIGHT = 0.70 # 정면 선호 가중치
FGM_CLEARANCE_WEIGHT = 1.80 # 넓게 뚫린 방향 선호 가중치
FGM_EDGE_WEIGHT = 0.85 # Gap 중앙 선호 가중치

SIDE_GAP_WARN_DIST = 0.175 # 옆 경고 시작 거리
SIDE_GAP_BLOCK_DIST = 0.16 # 강하게 거부할 옆 거리
SIDE_GAP_BIAS_MAX_DEG = 10.0 # 조향 보정 최대각도
SIDE_GAP_BIAS_GAIN_DEG_PER_M = 125.0 # 좌우 거리차
SIDE_TIGHT_TURN_LIMIT_DEG = 20.0 # 옆이 매우 좁을 때 회전한계
SIDE_NARROW_TURN_LIMIT_DEG = 35.0 # 옆이 좁을 때 회전한계
SIDE_NARROW_V = 0.12 # 옆이 좁을 때 FGM 속도 상한
SIDE_TIGHT_V = 0.08 # 옆이 매우 좁을 때 FGM 속도 상한


# Recovery Mode
RECOVERY_TURN_W = 1.00 # Recovery 제자리 회전 속도
RECOVERY_MAX_TURN_RAD = math.radians(200.0) # Recovery 최대 회전각도
RECOVERY_TURN_TIMEOUT_S = 15.0 # Recovery 해당 초 지나면 강제로 벽 따라가기

RECOVERY_TURN_ENABLE = True # Recovery 기능 on/off 스위치
RECOVERY_OPEN_JUMP_DIST = 0.07
RECOVERY_OPEN_PREV_MAX_DIST = 0.45


# Wall Follow
WALL_TARGET_DIST = 0.22 # 벽과 유지하려는 목표거리
WALL_KP = 6.0 # 벽 거리 오차
WALL_MIN_TURN_W = 0.16 # 오차가 있는데 명령이 너무 작으면 이만큼 보장
WALL_FRONT_SLOW_DIST = 0.14 # 정면이 이 안에 들어오면 감속
WALL_FRONT_HARD_STOP_DIST = 0.08 # 정면이 이 안에 들어오면 강제 회전
WALL_LOST_DIST = 0.30 # 벽 거리가 이 이상이면 벽 놓침
WALL_JUMP_DIST = 0.10 # 벽 거리가 한 번이 이만큼 늘어나면 벽 종료
WALL_OPEN_COUNT_N = 3 # N번 연속 트여있어야 FGM 복귀

WALL_FOLLOW_ENABLE = True # Wall follow on/off 스위치
WALL_SEARCH_W = 0.28 # 벽을 놓쳤을 때 벽 쪽으로 탐색하는 회전속도
WALL_MIN_TURN_ERR = 0.025 # 이 오차 이하이면 WALL_MIN_TURN_W 보장X
WALL_FRONT_CHECK_DIST = 0.30 # Wall follow 정면 장애물 반응 거리
WALL_ANGLE_HALF_WIDTH_DEG = 1.0 # Wall을 90도 하나가 아니라 +-1도 간격으로 더 확인
WALL_FRONT_Y_HALF = 0.16 # Wall follow 장애물로 인식할 Y 거리의 절반
WALL_MIN_FOLLOW_TIME_S = 0.80 # 진입 후 최소 N초는 종료 판정X
LEFT_WALL_ANGLE_CENTER_DEG = 90.0 # 왼쪽 벽 감지 기준 각도
RIGHT_WALL_ANGLE_CENTER_DEG = -90.0 # 오른쪽 벽 감지 기준 각도
WALL_MIN_POINTS = 2 # 벽으로 인정하는 최소 라이다 포인트 수


LOOP_DT_S = 0.05 # 메인 제어 루프 주기


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_error_rad(a, b):
    return normalize_angle_rad(a - b)

def choose_initial_based_recovery_dir(theta, prev_w=0.0):
    heading_from_initial = normalize_angle_rad(theta - INITIAL_HEADING_RAD)

    if heading_from_initial > RECOVERY_INITIAL_DEADBAND_RAD:
        return -1.0

    if heading_from_initial < -RECOVERY_INITIAL_DEADBAND_RAD:
        return +1.0

    if prev_w > 0.0:
        return -1.0

    if prev_w < 0.0:
        return +1.0

    return -1.0


class RobotPose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, v, w, dt):
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = normalize_angle_rad(self.theta + w * dt)

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
        self.scan_seq = 0
        self.scan_time = 0.0
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
                            np.array(buf_q, dtype=np.float32),
                        )
                        self.scan_seq += 1
                        self.scan_time = time.time()
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
                except Exception:
                    pass

    def get_scan(self):
        with self.lock:
            return self.latest_scan, self.scan_seq, self.scan_time

    def close(self):
        self.running = False
        try:
            self.ser.write(bytes([0xA5, 0x25]))
        except Exception:
            pass
        time.sleep(0.1)
        self.ser.close()


def lidar_points_to_xy(scan):
    if scan is None:
        return np.empty((0, 2), dtype=np.float32)

    angles, dists, qualities = scan
    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG)
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    mask = (
        (dist_m >= MIN_LIDAR_DIST_M)
        & (dist_m <= MAX_LIDAR_DIST_M)
        & (qualities >= MIN_QUALITY)
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

def lidar_points_to_xy_all(scan):
    if scan is None:
        return np.empty((0, 2), dtype=np.float32)

    angles, dists, qualities = scan
    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG)
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    mask = (
        (dist_m >= MIN_LIDAR_DIST_M)
        & (dist_m <= MAX_LIDAR_DIST_M)
        & (qualities >= MIN_QUALITY)
    )

    if not mask.any():
        return np.empty((0, 2), dtype=np.float32)

    dist_m = dist_m[mask]
    angle_rad = np.deg2rad(angle_deg[mask])

    x = dist_m * np.cos(angle_rad)
    y = dist_m * np.sin(angle_rad)

    return np.column_stack((x, y)).astype(np.float32)


def side_wall_distance_from_scan(scan, follow_side):
    if scan is None:
        return MAX_LIDAR_DIST_M

    angles, dists, qualities = scan

    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG)
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    if follow_side > 0.0:
        center = LEFT_WALL_ANGLE_CENTER_DEG
    else:
        center = RIGHT_WALL_ANGLE_CENTER_DEG

    min_angle = center - WALL_ANGLE_HALF_WIDTH_DEG
    max_angle = center + WALL_ANGLE_HALF_WIDTH_DEG

    mask = (
        (dist_m >= MIN_LIDAR_DIST_M)
        & (dist_m <= MAX_LIDAR_DIST_M)
        & (qualities >= MIN_QUALITY)
        & (angle_deg >= min_angle)
        & (angle_deg <= max_angle)
    )

    if np.count_nonzero(mask) < WALL_MIN_POINTS:
        return MAX_LIDAR_DIST_M

    return float(np.percentile(dist_m[mask], 20))


def side_wall_average_distance_from_scan(scan, follow_side):
    if scan is None:
        return MAX_LIDAR_DIST_M

    angles, dists, qualities = scan

    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG)
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    if follow_side > 0.0:
        min_angle = 30.0
        max_angle = 90.0
    else:
        min_angle = -90.0
        max_angle = -30.0

    mask = (
        (dist_m >= MIN_LIDAR_DIST_M)
        & (dist_m <= MAX_LIDAR_DIST_M)
        & (qualities >= MIN_QUALITY)
        & (angle_deg >= min_angle)
        & (angle_deg <= max_angle)
    )

    if np.count_nonzero(mask) < WALL_MIN_POINTS:
        return MAX_LIDAR_DIST_M

    angle_rad = np.deg2rad(angle_deg[mask])
    side_y = dist_m[mask] * np.sin(angle_rad)

    if follow_side > 0.0:
        side_dist = side_y
    else:
        side_dist = -side_y

    valid_side_dist = side_dist[side_dist > 0.0]
    if len(valid_side_dist) < WALL_MIN_POINTS:
        return MAX_LIDAR_DIST_M

    return float(np.mean(valid_side_dist))


def wall_follow_front_distance(points_all):
    if len(points_all) == 0:
        return MAX_LIDAR_DIST_M

    mask = (
        (points_all[:, 0] > 0.03)
        & (points_all[:, 0] < WALL_FRONT_CHECK_DIST)
        & (np.abs(points_all[:, 1]) < WALL_FRONT_Y_HALF)
    )

    if not mask.any():
        return MAX_LIDAR_DIST_M

    return float(np.min(points_all[mask, 0]))


def front_distance(points):
    if len(points) == 0:
        return MAX_LIDAR_DIST_M

    mask = (
        (points[:, 0] > 0.0)
        & (points[:, 0] < ACTIVE_FRONT_DIST)
        & (np.abs(points[:, 1]) < FRONT_CORRIDOR_HALF)
    )
    if not mask.any():
        return MAX_LIDAR_DIST_M
    return float(np.min(points[mask, 0]))


def compute_front_factor(front_dist):
    return float(
        np.clip(
            (ACTIVE_FRONT_DIST - front_dist)
            / max(1e-6, ACTIVE_FRONT_DIST - FRONT_DANGER_DIST),
            0.0,
            1.0,
        )
    )


def compute_side_info(points):
    info_left = 1.0
    info_right = 1.0
    if len(points) == 0:
        return info_left, info_right

    side_band = (np.abs(points[:, 0]) < 0.15) & (np.abs(points[:, 1]) < 0.30)
    if side_band.any():
        ys = points[side_band, 1]
        left = ys[ys > 0.05]
        right = ys[ys < -0.05]
        if len(left) > 0:
            info_left = float(np.min(left))
        if len(right) > 0:
            info_right = float(-np.max(right))
    return info_left, info_right


def scan_to_angle_ranges(scan):
    angles_grid = np.arange(
        FGM_MIN_ANGLE_DEG,
        FGM_MAX_ANGLE_DEG + 0.5 * FGM_ANGLE_STEP_DEG,
        FGM_ANGLE_STEP_DEG,
        dtype=np.float32,
    )
    ranges = np.full(len(angles_grid), MAX_LIDAR_DIST_M, dtype=np.float32)
    counts = np.zeros(len(angles_grid), dtype=np.int16)

    if scan is None:
        return angles_grid, ranges, counts

    angles, dists, qualities = scan
    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG)
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    valid = (
        (dist_m >= MIN_LIDAR_DIST_M)
        & (dist_m <= MAX_LIDAR_DIST_M)
        & (qualities >= MIN_QUALITY)
    )
    if not valid.any():
        return angles_grid, ranges, counts

    dist_m = dist_m[valid]
    angle_deg = angle_deg[valid]
    bins = np.rint((angle_deg - FGM_MIN_ANGLE_DEG) / FGM_ANGLE_STEP_DEG).astype(np.int32)
    in_sector = (bins >= 0) & (bins < len(angles_grid))

    for bin_idx, dist in zip(bins[in_sector], dist_m[in_sector]):
        if dist < ranges[bin_idx]:
            ranges[bin_idx] = dist
        counts[bin_idx] += 1

    return angles_grid, ranges, counts


def smooth_ranges_conservative(ranges):
    if FGM_SMOOTH_WINDOW <= 1:
        return ranges.copy()

    window = int(FGM_SMOOTH_WINDOW)
    if window % 2 == 0:
        window += 1
    pad = window // 2
    kernel = np.ones(window, dtype=np.float32) / float(window)
    padded = np.pad(ranges, (pad, pad), mode="edge")
    averaged = np.convolve(padded, kernel, mode="valid").astype(np.float32)
    return np.minimum(ranges, averaged)


def apply_safety_bubble(ranges, counts):
    working = ranges.copy()
    valid = counts > 0
    valid_obstacles = valid & (ranges < MAX_LIDAR_DIST_M)
    if not valid_obstacles.any():
        return working, -1, MAX_LIDAR_DIST_M, 0

    obstacle_ranges = np.where(valid_obstacles, ranges, np.inf)
    closest_idx = int(np.argmin(obstacle_ranges))
    closest_dist = float(ranges[closest_idx])
    half_angle_rad = math.atan2(FGM_BUBBLE_RADIUS, max(closest_dist, MIN_LIDAR_DIST_M))
    half_bins = int(math.ceil(math.degrees(half_angle_rad) / FGM_ANGLE_STEP_DEG))

    start = max(0, closest_idx - half_bins)
    end = min(len(working), closest_idx + half_bins + 1)
    working[start:end] = 0.0
    return working, closest_idx, closest_dist, half_bins


def find_free_gaps(free_mask):
    gaps = []
    start = None
    for idx, free in enumerate(free_mask):
        if free and start is None:
            start = idx
        elif not free and start is not None:
            gaps.append((start, idx))
            start = None
    if start is not None:
        gaps.append((start, len(free_mask)))
    return gaps


def filter_gaps_by_width(gaps):
    min_bins = max(1, int(math.ceil(FGM_MIN_GAP_WIDTH_DEG / FGM_ANGLE_STEP_DEG)))
    return [(start, end) for start, end in gaps if end - start >= min_bins]


def side_gap_steering_bias(left_dist, right_dist):
    if min(left_dist, right_dist) >= SIDE_GAP_WARN_DIST:
        return 0.0

    max_bias = math.radians(SIDE_GAP_BIAS_MAX_DEG)
    bias = math.radians(SIDE_GAP_BIAS_GAIN_DEG_PER_M) * (left_dist - right_dist)
    return float(np.clip(bias, -max_bias, max_bias))


def side_narrow_turn_limit(left_dist, right_dist):
    side_min = min(left_dist, right_dist)

    if side_min <= SIDE_GAP_BLOCK_DIST:
        return math.radians(SIDE_TIGHT_TURN_LIMIT_DEG)

    if side_min < SIDE_GAP_WARN_DIST:
        return math.radians(SIDE_NARROW_TURN_LIMIT_DEG)

    return TURN_HARD_LIMIT_RAD


def side_gap_speed_limit(left_dist, right_dist):
    side_min = min(left_dist, right_dist)

    if side_min <= SIDE_GAP_BLOCK_DIST:
        return SIDE_TIGHT_V

    if side_min < SIDE_GAP_WARN_DIST:
        return SIDE_NARROW_V

    return BASE_V


def cumulative_turn_penalty(angle_rad, accumulated_turn_rad):
    projected_turn = accumulated_turn_rad + angle_rad
    abs_accumulated = abs(accumulated_turn_rad)
    abs_projected = np.abs(projected_turn)
    increasing_turn = abs_projected > (abs_accumulated + math.radians(1.0))

    soft_excess = np.maximum(0.0, abs_projected - CUM_TURN_SOFT_LIMIT_RAD)
    hard_excess = np.maximum(0.0, abs_projected - CUM_TURN_HARD_LIMIT_RAD)
    soft_span = max(1e-6, CUM_TURN_HARD_LIMIT_RAD - CUM_TURN_SOFT_LIMIT_RAD)

    soft_penalty = CUM_TURN_SOFT_PENALTY_WEIGHT * (soft_excess / soft_span) ** 2
    hard_penalty = CUM_TURN_HARD_PENALTY_WEIGHT * (
        hard_excess / math.radians(10.0) + (hard_excess > 0.0)
    )
    penalty = np.where(increasing_turn, soft_penalty + hard_penalty, 0.0)

    return penalty.astype(np.float32), projected_turn


def choose_target_from_gaps(angles_deg, ranges, gaps, pose, prev_target_angle, front_factor, accumulated_turn_rad):
    if not gaps:
        return -1, (0, 0), -float("inf")

    goal_angle = normalize_angle_rad(INITIAL_HEADING_RAD - pose.theta)
    goal_angle = float(np.clip(goal_angle, -TURN_HARD_LIMIT_RAD, TURN_HARD_LIMIT_RAD))
    prev_angle = float(np.clip(prev_target_angle, -TURN_HARD_LIMIT_RAD, TURN_HARD_LIMIT_RAD))

    best_idx = -1
    best_gap = (0, 0)
    best_score = -float("inf")
    max_target_angle = TURN_HARD_LIMIT_RAD
    goal_weight = (
        FGM_GOAL_WEIGHT_SAFE * (1.0 - front_factor)
        + FGM_GOAL_WEIGHT_DANGER * front_factor
    )

    for start, end in gaps:
        idxs = np.arange(start, end)
        angle_rad = np.deg2rad(angles_deg[idxs])
        targetable = np.abs(angle_rad) <= max_target_angle
        if not targetable.any():
            continue

        idxs = idxs[targetable]
        angle_rad = angle_rad[targetable]
        local_width = max(1, end - start)

        edge_steps = np.minimum(idxs - start + 1, end - idxs)
        edge_score = np.clip(edge_steps / max(1.0, local_width * 0.5), 0.0, 1.0)
        clearance_score = np.clip(ranges[idxs] / CLEARANCE_CAP, 0.0, 1.0)
        straight_score = 1.0 - np.clip(np.abs(angle_rad) / max_target_angle, 0.0, 1.0)
        goal_score = 1.0 - np.clip(
            np.abs([angle_error_rad(a, goal_angle) for a in angle_rad]) / max_target_angle,
            0.0,
            1.0,
        )
        prev_score = 1.0 - np.clip(
            np.abs([angle_error_rad(a, prev_angle) for a in angle_rad]) / max_target_angle,
            0.0,
            1.0,
        )
        width_score = min(1.0, local_width * FGM_ANGLE_STEP_DEG / 60.0)
        turn_penalty, _ = cumulative_turn_penalty(angle_rad, accumulated_turn_rad)

        scores = (
            FGM_CLEARANCE_WEIGHT * clearance_score
            + FGM_EDGE_WEIGHT * edge_score
            + FGM_STRAIGHT_WEIGHT * straight_score
            + goal_weight * goal_score
            + FGM_PREV_TARGET_WEIGHT * prev_score
            + 0.25 * width_score
            - turn_penalty
        )
        if not np.isfinite(scores).any():
            continue

        local_best = int(np.argmax(scores))
        score = float(scores[local_best])
        if score > best_score:
            best_score = score
            best_idx = int(idxs[local_best])
            best_gap = (start, end)

    return best_idx, best_gap, best_score


def choose_fallback_target(angles_deg, ranges, left_dist=MAX_LIDAR_DIST_M, right_dist=MAX_LIDAR_DIST_M):
    usable = ranges > MIN_LIDAR_DIST_M
    if not usable.any():
        return len(ranges) // 2

    usable_ranges = np.where(usable, ranges, -np.inf)
    return int(np.argmax(usable_ranges))


def rate_limit_w(prev_w, target_w, urgent=False):
    limit = W_CMD_RATE_LIMIT_URGENT if urgent else W_CMD_RATE_LIMIT
    delta = float(np.clip(target_w - prev_w, -limit, limit))
    return prev_w + delta


def choose_speed(target_dist, target_angle, has_safe_gap):
    if not has_safe_gap:
        return 0.0

    turn_ratio = min(1.0, abs(target_angle) / TURN_HARD_LIMIT_RAD)
    v = BASE_V * (1.0 - 0.45 * turn_ratio)

    if target_dist < ACTIVE_FRONT_DIST:
        slow_ratio = np.clip(
            (target_dist - FRONT_DANGER_DIST)
            / max(1e-6, ACTIVE_FRONT_DIST - FRONT_DANGER_DIST),
            0.0,
            1.0,
        )
        v = min(v, MIN_V + (BASE_V - MIN_V) * float(slow_ratio))

    if target_dist < COLLISION_DIST:
        v = 0.0

    return float(np.clip(v, 0.0, BASE_V))


def choose_fgm_cmd(
    scan,
    prev_w,
    prev_target_angle,
    pose,
    accumulated_turn_rad=0.0,
):
    points = lidar_points_to_xy(scan)
    points_all = lidar_points_to_xy_all(scan)
    front_dist = front_distance(points)
    front_factor = compute_front_factor(front_dist)
    info_left, info_right = compute_side_info(points_all)

    angles_deg, raw_ranges, counts = scan_to_angle_ranges(scan)
    smooth_ranges = smooth_ranges_conservative(raw_ranges)
    bubble_ranges, closest_idx, closest_dist, bubble_bins = apply_safety_bubble(
        smooth_ranges, counts
    )

    free_mask = bubble_ranges >= FGM_FREE_DIST
    gaps = filter_gaps_by_width(find_free_gaps(free_mask))
    has_safe_gap = len(gaps) > 0

    target_idx, best_gap, best_score = choose_target_from_gaps(
        angles_deg, bubble_ranges, gaps, pose, prev_target_angle, front_factor, accumulated_turn_rad
    )

    if target_idx < 0:
        target_idx = choose_fallback_target(angles_deg, smooth_ranges, info_left, info_right)
        best_gap = (target_idx, target_idx + 1)
        best_score = 0.0

    gap_width = (best_gap[1] - best_gap[0]) * FGM_ANGLE_STEP_DEG
    gap_left = float(angles_deg[best_gap[1] - 1]) if best_gap[1] > best_gap[0] else float(angles_deg[target_idx])
    gap_right = float(angles_deg[best_gap[0]]) if best_gap[1] > best_gap[0] else float(angles_deg[target_idx])
    gap_left_rad = math.radians(gap_left)
    gap_right_rad = math.radians(gap_right)

    fgm_target_angle = math.radians(float(angles_deg[target_idx]))
    fgm_target_angle = float(np.clip(fgm_target_angle, -TURN_HARD_LIMIT_RAD, TURN_HARD_LIMIT_RAD))
    side_bias = side_gap_steering_bias(info_left, info_right)
    side_turn_limit = side_narrow_turn_limit(info_left, info_right)
    target_angle = float(np.clip(fgm_target_angle + side_bias, -side_turn_limit, side_turn_limit))
    if has_safe_gap and best_gap[1] > best_gap[0]:
        target_angle = float(np.clip(target_angle, gap_right_rad, gap_left_rad))
        if fgm_target_angle > 0.0:
            target_angle = max(0.0, target_angle)
        elif fgm_target_angle < 0.0:
            target_angle = min(0.0, target_angle)
    target_idx = int(np.argmin(np.abs(angles_deg - math.degrees(target_angle))))
    target_dist = float(smooth_ranges[target_idx])
    raw_w = float(np.clip(FGM_TURN_GAIN * target_angle, -MAX_ABS_W, MAX_ABS_W))
    selected_turn_penalty, selected_projected_turn = cumulative_turn_penalty(
        np.array([target_angle], dtype=np.float32),
        accumulated_turn_rad,
    )
    urgent = front_dist < URGENT_FRONT_DIST or not has_safe_gap
    w = rate_limit_w(prev_w, raw_w, urgent=urgent)
    v = choose_speed(target_dist, target_angle, has_safe_gap)
    v = min(v, side_gap_speed_limit(info_left, info_right))

    closest_angle = float(angles_deg[closest_idx]) if closest_idx >= 0 else 0.0

    return v, w, target_angle, {
        "score": best_score,
        "target_deg": math.degrees(target_angle),
        "target_dist": target_dist,
        "front": front_dist,
        "front_factor": front_factor,
        "left": info_left,
        "right": info_right,
        "points": len(points),
        "gaps": len(gaps),
        "gap_width": gap_width,
        "gap_right": gap_right,
        "gap_left": gap_left,
        "closest": closest_dist,
        "closest_angle": closest_angle,
        "bubble_bins": bubble_bins,
        "collision": target_dist < COLLISION_DIST or closest_dist < COLLISION_DIST,
        "raw_w": raw_w,
        "has_safe_gap": has_safe_gap,
        "side_bias_deg": math.degrees(side_bias),
        "side_turn_limit_deg": math.degrees(side_turn_limit),
        "turn_penalty": float(selected_turn_penalty[0]),
        "projected_turn_deg": math.degrees(float(selected_projected_turn[0])),
    }


def choose_wall_follow_cmd(scan, prev_w, follow_side):
    points_all = lidar_points_to_xy_all(scan)

    wall_dist = side_wall_average_distance_from_scan(scan, follow_side)
    front_dist = wall_follow_front_distance(points_all)

    wall_valid = wall_dist < WALL_LOST_DIST

    if wall_valid:
        error = wall_dist - WALL_TARGET_DIST
        target_w = follow_side * WALL_KP * error

        if abs(error) > WALL_MIN_TURN_ERR and abs(target_w) < WALL_MIN_TURN_W:
            if error > 0.0:
                target_w = follow_side * WALL_MIN_TURN_W
            else:
                target_w = -follow_side * WALL_MIN_TURN_W

    else:
        target_w = follow_side * WALL_SEARCH_W

    if wall_valid and wall_dist < WALL_TARGET_DIST:
        target_v = WALL_SLOW_V
    
        close_error = WALL_TARGET_DIST - wall_dist
        target_w = -follow_side * WALL_KP * close_error
    
        if abs(target_w) < WALL_MIN_TURN_W:
            target_w = -follow_side * WALL_MIN_TURN_W
    
    elif front_dist < WALL_FRONT_HARD_STOP_DIST:
        target_v = WALL_SLOW_V
    
        front_error = WALL_FRONT_HARD_STOP_DIST - front_dist
        target_w = -follow_side * WALL_KP * front_error
    
        if abs(target_w) < WALL_MIN_TURN_W:
            target_w = -follow_side * WALL_MIN_TURN_W
    
    elif front_dist < WALL_FRONT_SLOW_DIST:
        target_v = WALL_SLOW_V
    
        front_error = WALL_FRONT_SLOW_DIST - front_dist
        target_w = -follow_side * WALL_KP * front_error
    
        if abs(target_w) < WALL_MIN_TURN_W:
            target_w = -follow_side * WALL_MIN_TURN_W
    
    else:
        target_v = WALL_FOLLOW_V

    target_w = float(np.clip(target_w, -WALL_MAX_W, WALL_MAX_W))
    w = rate_limit_w(prev_w, target_w, urgent=True)

    return float(target_v), float(w), float(wall_dist), bool(wall_valid), float(front_dist)


def is_wall_open(wall_dist, prev_wall_dist, follow_start_time):
    if time.time() - follow_start_time < WALL_MIN_FOLLOW_TIME_S:
        return False

    open_by_lost = wall_dist >= WALL_LOST_DIST

    open_by_jump = (
        prev_wall_dist is not None
        and prev_wall_dist < WALL_LOST_DIST
        and (wall_dist - prev_wall_dist) >= WALL_JUMP_DIST
    )

    return bool(open_by_lost or open_by_jump)


def main():
    pose = RobotPose()
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

    pose.reset()
    last_scan_ok = 0.0
    last_processed_scan_seq = -1
    stale_scan_warned = False
    last_v, last_w = BASE_V, 0.0
    last_target_angle = 0.0
    last_log = 0.0
    last_pose_time = time.time()
    accumulated_turn_rad = 0.0

    recovery_turn_active = False
    recovery_turn_dir = 0.0
    recovery_follow_side = +1.0
    recovery_start_time = 0.0
    recovery_wall_seen_count = 0
    recovery_accum_turn = 0.0
    recovery_prev_open_dist = None

    recovery_turned_deg_log = 0.0
    recovery_wall_dist_log = MAX_LIDAR_DIST_M
    recovery_front_start_log = MAX_LIDAR_DIST_M

    wall_follow_active = False
    wall_follow_side = +1.0
    wall_follow_start_time = 0.0
    wall_prev_dist = None
    wall_open_count = 0

    wall_dist_log = MAX_LIDAR_DIST_M
    wall_front_log = MAX_LIDAR_DIST_M
    wall_valid_log = False

    try:
        while True:
            now = time.time()
            dt = max(0.0, min(0.20, now - last_pose_time))
            last_pose_time = now

            if recovery_turn_active:
                recovery_accum_turn += abs(last_w) * dt

            scan, scan_seq, scan_time = lidar.get_scan()
            if scan is None:
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    send_vw(last_v, last_w)
                    pose.update(last_v, last_w, dt)
                    accumulated_turn_rad += last_w * dt
                else:
                    send_vw(0.0, 0.0)
                    pose.update(0.0, 0.0, dt)
                time.sleep(LOOP_DT_S)
                continue

            scan_age = now - scan_time
            scan_is_new = scan_seq != last_processed_scan_seq
            scan_is_stale = scan_age > SCAN_HOLD_S

            if scan_is_new:
                stale_scan_warned = False
            elif not scan_is_stale:
                send_vw(last_v, last_w)
                pose.update(last_v, last_w, dt)
                accumulated_turn_rad += last_w * dt
                time.sleep(LOOP_DT_S)
                continue

            if scan_is_stale and not stale_scan_warned:
                print(f"[LIDAR] Stale scan age={scan_age:.2f}s. Reusing latest scan.")
                stale_scan_warned = True

            last_processed_scan_seq = scan_seq
            last_scan_ok = scan_time

            v = 0.0
            w = 0.0
            target_angle = 0.0
            info = None
            recovery_mode_name = "FGM"
            returned_from_wall_this_loop = False

            wall_dist_log = MAX_LIDAR_DIST_M
            wall_front_log = MAX_LIDAR_DIST_M
            wall_valid_log = False
            recovery_turned_deg_log = math.degrees(recovery_accum_turn)
            recovery_wall_dist_log = MAX_LIDAR_DIST_M
            recovery_front_start_log = MAX_LIDAR_DIST_M

            if WALL_FOLLOW_ENABLE and wall_follow_active:
                wall_v, wall_w, wall_dist, wall_valid, wall_front = choose_wall_follow_cmd(
                    scan,
                    last_w,
                    wall_follow_side,
                )

                wall_dist_log = wall_dist
                wall_front_log = wall_front
                wall_valid_log = wall_valid

                if is_wall_open(wall_dist, wall_prev_dist, wall_follow_start_time):
                    wall_open_count += 1
                else:
                    wall_open_count = 0

                if wall_open_count >= WALL_OPEN_COUNT_N:
                    wall_follow_active = False
                    wall_prev_dist = None
                    wall_open_count = 0
                    recovery_mode_name = "FGM_RETURN_FROM_WALL"
                    returned_from_wall_this_loop = True

                    print(
                        "[WALL] wall distance opened suddenly. "
                        "Return to FGM driving."
                    )

                    v, w, target_angle, info = choose_fgm_cmd(
                    scan,
                    last_w,
                    last_target_angle,
                    pose,
                    accumulated_turn_rad,
                )
                else:
                    v = wall_v
                    w = wall_w
                    target_angle = 0.0

                    if wall_follow_side > 0.0:
                        recovery_mode_name = "LEFT_WALL_FOLLOW"
                    else:
                        recovery_mode_name = "RIGHT_WALL_FOLLOW"

                    wall_prev_dist = wall_dist

            if (not recovery_turn_active) and (not wall_follow_active) and (not returned_from_wall_this_loop):
                v, w, target_angle, info = choose_fgm_cmd(
                    scan,
                    last_w,
                    last_target_angle,
                    pose,
                    accumulated_turn_rad,
                )

                blocked_now = info["collision"] and v <= 0.01
                no_safe_gap_now = not info["has_safe_gap"]
                narrow_wall_trigger = (blocked_now or no_safe_gap_now)

                if (
                    RECOVERY_TURN_ENABLE
                    and (not recovery_turn_active)
                    and narrow_wall_trigger
                ):
                    recovery_turn_dir = choose_initial_based_recovery_dir(
                        pose.theta,
                        last_w,
                    )
                    recovery_follow_side = -recovery_turn_dir
                    recovery_start_time = time.time()
                    recovery_wall_seen_count = 0
                    recovery_accum_turn = 0.0
                    recovery_prev_open_dist = None
                    recovery_turn_active = True

                    if blocked_now:
                        trigger_name = "blocked/narrow path"
                    else:
                        trigger_name = "no safe path"

                    if recovery_turn_dir > 0.0:
                        print(
                            f"[NARROW] {trigger_name}. "
                            "Start LEFT recovery turn. Follow RIGHT wall."
                        )
                    else:
                        print(
                            f"[NARROW] {trigger_name}. "
                            "Start RIGHT recovery turn. Follow LEFT wall."
                        )

            if (
                RECOVERY_TURN_ENABLE
                and recovery_turn_active
                and (not wall_follow_active)
            ):
                recovery_timeout = (
                    time.time() - recovery_start_time
                ) > RECOVERY_TURN_TIMEOUT_S

                recovery_max_turn_reached = recovery_accum_turn >= RECOVERY_MAX_TURN_RAD

                recovery_wall_dist = side_wall_distance_from_scan(
                    scan,
                    recovery_follow_side,
                )

                recovery_points_all = lidar_points_to_xy_all(scan)
                recovery_front_dist = wall_follow_front_distance(recovery_points_all)
                recovery_open_dist = recovery_wall_dist

                recovery_turned_deg_log = math.degrees(recovery_accum_turn)
                recovery_wall_dist_log = recovery_wall_dist
                recovery_front_start_log = recovery_front_dist

                recovery_open_detected = (
                    recovery_prev_open_dist is not None
                    and recovery_prev_open_dist <= RECOVERY_OPEN_PREV_MAX_DIST
                    and (recovery_open_dist - recovery_prev_open_dist) >= RECOVERY_OPEN_JUMP_DIST
                )

                recovery_wall_seen_count = 1 if recovery_open_detected else 0

                recovery_ready_to_wall_follow = recovery_open_detected

                recovery_prev_open_dist = recovery_open_dist

                if (
                    recovery_ready_to_wall_follow
                    or recovery_max_turn_reached
                    or recovery_timeout
                ):
                    recovery_turn_active = False
                    wall_follow_active = True
                    wall_follow_side = recovery_follow_side

                    wall_follow_start_time = time.time()
                    wall_prev_dist = None
                    wall_open_count = 0
                    recovery_wall_seen_count = 0
                    recovery_prev_open_dist = None

                    v = 0.0
                    w = 0.0
                    target_angle = 0.0

                    if wall_follow_side > 0.0:
                        recovery_mode_name = "RECOVERY_TO_LEFT_WALL"
                    else:
                        recovery_mode_name = "RECOVERY_TO_RIGHT_WALL"

                    if recovery_ready_to_wall_follow:
                        print(
                            f"[RECOVERY] wall found after {recovery_turned_deg_log:.1f}deg. "
                            f"wall90={recovery_wall_dist:.2f} "
                            f"front={recovery_front_dist:.2f}. "
                            "Start wall following."
                        )
                    elif recovery_max_turn_reached:
                        print(
                            f"[RECOVERY] max turn reached. "
                            f"turned={recovery_turned_deg_log:.1f}deg. "
                            "Start wall following."
                        )
                    else:
                        print(
                            "[RECOVERY] recovery turn timeout. "
                            "Start wall following anyway."
                        )

                else:
                    v = 0.0
                    target_w = recovery_turn_dir * RECOVERY_TURN_W
                    target_w = float(
                        np.clip(
                            target_w,
                            -RECOVERY_MAX_W,
                            RECOVERY_MAX_W,
                        )
                    )
                    w = rate_limit_w(last_w, target_w, urgent=True)
                    target_angle = 0.0

                    if recovery_turn_dir > 0.0:
                        recovery_mode_name = "RECOVERY_LEFT_TURN"
                    else:
                        recovery_mode_name = "RECOVERY_RIGHT_TURN"


            send_vw(v, w)
            pose.update(v, w, dt)
            accumulated_turn_rad += w * dt
            last_v, last_w = v, w

            if (not recovery_turn_active) and (not wall_follow_active):
                last_target_angle = target_angle

            accumulated_turn_deg_log = math.degrees(accumulated_turn_rad)

            if time.time() - last_log > 0.5:
                if recovery_turn_active:
                    if info is not None:
                        front_log = info["front"]
                        gaps_log = info["gaps"]
                        safe_log = int(info["has_safe_gap"])
                        close_log = info["closest"]
                        close_angle_log = info["closest_angle"]
                        left_log = info["left"]
                        right_log = info["right"]
                    else:
                        front_log = MAX_LIDAR_DIST_M
                        gaps_log = 0
                        safe_log = 0
                        close_log = MAX_LIDAR_DIST_M
                        close_angle_log = 0.0
                        left_log = 1.0
                        right_log = 1.0

                    print(
                        f"[{recovery_mode_name}] x={pose.x:.2f} y={pose.y:.2f} "
                        f"th={math.degrees(pose.theta):.1f}deg "
                        f"v={v:.2f} w={w:.2f} "
                        f"ct={accumulated_turn_deg_log:.1f}deg "
                        f"turned={recovery_turned_deg_log:.1f}deg "
                        f"wall90={recovery_wall_dist_log:.2f} "
                        f"wall_front={recovery_front_start_log:.2f} "
                        f"seen_cnt={recovery_wall_seen_count} "
                        f"front={front_log:.2f} "
                        f"gaps={gaps_log} safe={safe_log} "
                        f"close={close_log:.2f}@{close_angle_log:.0f} "                        
                        f"L={left_log:.2f} R={right_log:.2f}"
                    )

                elif wall_follow_active:
                    if wall_follow_side > 0.0:
                        wall_name = "left_wall_30_90_avg"
                    else:
                        wall_name = "right_wall_90_30_avg"

                    print(
                        f"[{recovery_mode_name}] x={pose.x:.2f} y={pose.y:.2f} "
                        f"th={math.degrees(pose.theta):.1f}deg "
                        f"v={v:.2f} w={w:.2f} "
                        f"ct={accumulated_turn_deg_log:.1f}deg "
                        f"{wall_name}={wall_dist_log:.2f} "
                        f"wall_front={wall_front_log:.2f} "
                        f"valid={int(wall_valid_log)} "
                        f"open_cnt={wall_open_count} "
                        f"FGM=OFF"
                    )

                elif info is not None:
                    print(
                        f"[FGM] x={pose.x:.2f} y={pose.y:.2f} "
                        f"th={pose.theta:.2f} "
                        f"v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                        f"ct={accumulated_turn_deg_log:.1f}deg "
                        f"pt={info['projected_turn_deg']:.1f}deg "
                        f"tp={info['turn_penalty']:.2f} "
                        f"tgt={info['target_deg']:.1f} td={info['target_dist']:.2f} "
                        f"front={info['front']:.2f} ff={info['front_factor']:.2f} "
                        f"gap={info['gap_width']:.0f} "
                        f"gr={info['gap_right']:.0f} gl={info['gap_left']:.0f} "
                        f"gaps={info['gaps']} safe={int(info['has_safe_gap'])} "
                        f"close={info['closest']:.2f}@{info['closest_angle']:.0f} "
                        f"bb={info['bubble_bins']} score={info['score']:.2f} "
                        f"pts={info['points']} coll={int(info['collision'])} "
                        f"sbias={info['side_bias_deg']:.1f}deg "
                        f"slim={info['side_turn_limit_deg']:.0f}deg "
                        f"L={info['left']:.2f} R={info['right']:.2f}"
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
