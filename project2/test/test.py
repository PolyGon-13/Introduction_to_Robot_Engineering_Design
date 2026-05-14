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

ANGLE_OFFSET_DEG = 1.54
DIST_OFFSET_MM = 0.0
LIDAR_ANGLE_SIGN = -1.0

MIN_LIDAR_DIST_M = 0.05
MAX_LIDAR_DIST_M = 2.5
MIN_QUALITY = 1
MIN_X_FOR_PLANNING = 0.10
MAX_EVAL_POINTS = 720
SCAN_HOLD_S = 0.30
LOOP_DT_S = 0.05

BASE_V = 0.18
MIN_V = 0.15
MAX_ABS_W = 0.90

ROBOT_RADIUS = 0.16
COLLISION_DIST = ROBOT_RADIUS + 0.05
CLEARANCE_CAP = 0.6
FRONT_CORRIDOR_HALF = COLLISION_DIST + 0.30
ACTIVE_FRONT_DIST = 0.30
FRONT_DANGER_DIST = 0.19

W_CMD_RATE_LIMIT = 0.30
W_CMD_RATE_LIMIT_URGENT = 0.40
URGENT_FRONT_DIST = 0.30

GOAL_X_M = 5.0
GOAL_Y_M = 0.0
GOAL_TOL_M = 0.15
TURN_SOFT_LIMIT_RAD = math.radians(70.0)
TURN_HARD_LIMIT_RAD = math.radians(88.0)
CUM_TURN_SOFT_LIMIT_RAD = math.radians(70.0)
CUM_TURN_HARD_LIMIT_RAD = math.radians(88.0)
CUM_TURN_SOFT_PENALTY_WEIGHT = 8.0
CUM_TURN_HARD_PENALTY_WEIGHT = 50.0

FGM_MIN_ANGLE_DEG = -100.0
FGM_MAX_ANGLE_DEG = 100.0
FGM_ANGLE_STEP_DEG = 1.0
FGM_SMOOTH_WINDOW = 5
FGM_SAFETY_MARGIN = 0.08
FGM_BUBBLE_RADIUS = ROBOT_RADIUS + FGM_SAFETY_MARGIN
FGM_FREE_DIST = COLLISION_DIST + 0.04
FGM_MIN_GAP_WIDTH_DEG = 8.0
FGM_TURN_GAIN = 1.05
FGM_PREV_TARGET_WEIGHT = 0.45
FGM_GOAL_WEIGHT_SAFE = 1.00
FGM_GOAL_WEIGHT_DANGER = 0.30
FGM_STRAIGHT_WEIGHT = 0.70
FGM_CLEARANCE_WEIGHT = 1.80
FGM_EDGE_WEIGHT = 0.85
SIDE_GAP_WARN_DIST = COLLISION_DIST + 0.06
SIDE_GAP_BLOCK_DIST = 0.14
SIDE_GAP_PENALTY_WEIGHT = 4.0
SIDE_GAP_BLOCK_ANGLE_DEG = 3.0
SIDE_STRAIGHT_PENALTY_WEIGHT = 1.0
NEAR_COLLISION_STRAIGHT_BLOCK_DIST = COLLISION_DIST
NEAR_COLLISION_STRAIGHT_BLOCK_ANGLE_DEG = 12.0
NEAR_COLLISION_STRAIGHT_WARN_ANGLE_DEG = 25.0
NEAR_COLLISION_STRAIGHT_PENALTY_WEIGHT = 4.0

NARROW_MIN_ANGLE_DEG = -90.0
NARROW_MAX_ANGLE_DEG = 90.0
NARROW_ANGLE_STEP_DEG = 1.0
NARROW_FREE_DIST = COLLISION_DIST + 0.04
NARROW_MIN_GAP_WIDTH_DEG = 8.0

INITIAL_HEADING_RAD = 0.0

RECOVERY_TURN_ENABLE = True
RECOVERY_TURN_W = 1.00
RECOVERY_MAX_W = 1.00

RECOVERY_MIN_TURN_RAD = math.radians(120.0)
RECOVERY_MAX_TURN_RAD = math.radians(200.0)
RECOVERY_TURN_TIMEOUT_S = 15.0
RECOVERY_INITIAL_DEADBAND_RAD = math.radians(2.0)

RECOVERY_WALL_START_DIST = 0.30
RECOVERY_WALL_START_COUNT_N = 2
RECOVERY_FRONT_START_DIST = 0.05

WALL_FOLLOW_ENABLE = True

WALL_TARGET_DIST = 0.22
WALL_FOLLOW_V = 0.15
WALL_SLOW_V = 0.10

WALL_KP = 2.80
WALL_SEARCH_W = 0.28
WALL_MAX_W = 0.65

WALL_MIN_TURN_ERR = 0.025
WALL_MIN_TURN_W = 0.16

LEFT_WALL_ANGLE_CENTER_DEG = 90.0
RIGHT_WALL_ANGLE_CENTER_DEG = -90.0
WALL_ANGLE_HALF_WIDTH_DEG = 1.0
WALL_MIN_POINTS = 2

WALL_FRONT_Y_HALF = 0.16
WALL_FRONT_CHECK_DIST = 0.30
WALL_FRONT_SLOW_DIST = 0.14
WALL_FRONT_HARD_STOP_DIST = 0.08
WALL_FRONT_KEEP_TURN_W = 0.18

WALL_MIN_FOLLOW_TIME_S = 0.80
WALL_JUMP_DIST = 0.10
WALL_LOST_DIST = 0.30
WALL_OPEN_COUNT_N = 3


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_error_rad(a, b):
    return normalize_angle_rad(a - b)


def rate_limit_w(prev_w, target_w, urgent=False):
    limit = W_CMD_RATE_LIMIT_URGENT if urgent else W_CMD_RATE_LIMIT
    delta = float(np.clip(target_w - prev_w, -limit, limit))
    return prev_w + delta


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

    def goal_distance(self):
        dx = GOAL_X_M - self.x
        dy = GOAL_Y_M - self.y
        return math.hypot(dx, dy)


def goal_heading_error(x, y, theta):
    dx = GOAL_X_M - x
    dy = GOAL_Y_M - y
    target_heading = math.atan2(dy, dx)
    return normalize_angle_rad(target_heading - theta)


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
                            np.array(buf_q, dtype=np.float32),
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
                except Exception:
                    pass

    def get_scan(self):
        with self.lock:
            return self.latest_scan

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


def scan_to_angle_ranges_narrow(scan):
    angles_grid = np.arange(
        NARROW_MIN_ANGLE_DEG,
        NARROW_MAX_ANGLE_DEG + 0.5 * NARROW_ANGLE_STEP_DEG,
        NARROW_ANGLE_STEP_DEG,
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

    bins = np.rint(
        (angle_deg - NARROW_MIN_ANGLE_DEG) / NARROW_ANGLE_STEP_DEG
    ).astype(np.int32)

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


def filter_narrow_gaps_by_width(gaps):
    min_bins = max(
        1,
        int(math.ceil(NARROW_MIN_GAP_WIDTH_DEG / NARROW_ANGLE_STEP_DEG)),
    )

    return [
        (start, end)
        for start, end in gaps
        if end - start >= min_bins
    ]


def detect_narrow_or_blocked(scan):
    points_all = lidar_points_to_xy_all(scan)
    front_dist = front_distance(points_all)

    angles_deg, ranges, counts = scan_to_angle_ranges_narrow(scan)

    valid_obstacle = counts > 0

    if valid_obstacle.any():
        closest_dist = float(np.min(ranges[valid_obstacle]))
        closest_idx = int(np.argmin(np.where(valid_obstacle, ranges, np.inf)))
        closest_angle = float(angles_deg[closest_idx])
    else:
        closest_dist = MAX_LIDAR_DIST_M
        closest_angle = 0.0

    free_mask = ranges >= NARROW_FREE_DIST
    gaps = filter_narrow_gaps_by_width(find_free_gaps(free_mask))

    has_safe_gap = len(gaps) > 0

    blocked_now = front_dist < COLLISION_DIST
    no_safe_gap_now = not has_safe_gap

    trigger = blocked_now or no_safe_gap_now

    info = {
        "front": front_dist,
        "closest": closest_dist,
        "closest_angle": closest_angle,
        "gaps": len(gaps),
        "has_safe_gap": has_safe_gap,
        "blocked_now": blocked_now,
        "no_safe_gap_now": no_safe_gap_now,
    }

    return trigger, blocked_now, no_safe_gap_now, info


def side_gap_penalty(angle_rad, left_dist, right_dist):
    penalty = np.zeros_like(angle_rad, dtype=np.float32)
    hard_block = np.zeros_like(angle_rad, dtype=bool)
    block_angle_rad = math.radians(SIDE_GAP_BLOCK_ANGLE_DEG)
    straight_mask = np.abs(angle_rad) <= block_angle_rad

    if left_dist < SIDE_GAP_WARN_DIST:
        pressure = float(
            np.clip(
                (SIDE_GAP_WARN_DIST - left_dist)
                / max(1e-6, SIDE_GAP_WARN_DIST - SIDE_GAP_BLOCK_DIST),
                0.0,
                1.0,
            )
        )
        left_side = angle_rad > block_angle_rad
        penalty += np.where(
            left_side,
            SIDE_GAP_PENALTY_WEIGHT * pressure,
            0.0,
        )
        penalty += np.where(
            straight_mask,
            SIDE_STRAIGHT_PENALTY_WEIGHT * pressure,
            0.0,
        )
        if left_dist <= SIDE_GAP_BLOCK_DIST:
            hard_block |= left_side

    if right_dist < SIDE_GAP_WARN_DIST:
        pressure = float(
            np.clip(
                (SIDE_GAP_WARN_DIST - right_dist)
                / max(1e-6, SIDE_GAP_WARN_DIST - SIDE_GAP_BLOCK_DIST),
                0.0,
                1.0,
            )
        )
        right_side = angle_rad < -block_angle_rad
        penalty += np.where(
            right_side,
            SIDE_GAP_PENALTY_WEIGHT * pressure,
            0.0,
        )
        penalty += np.where(
            straight_mask,
            SIDE_STRAIGHT_PENALTY_WEIGHT * pressure,
            0.0,
        )
        if right_dist <= SIDE_GAP_BLOCK_DIST:
            hard_block |= right_side

    return penalty, hard_block


def near_collision_straight_penalty(angle_rad, closest_dist):
    penalty = np.zeros_like(angle_rad, dtype=np.float32)
    hard_block = np.zeros_like(angle_rad, dtype=bool)

    if closest_dist >= NEAR_COLLISION_STRAIGHT_BLOCK_DIST:
        return penalty, hard_block

    abs_angle = np.abs(angle_rad)
    block_angle = math.radians(NEAR_COLLISION_STRAIGHT_BLOCK_ANGLE_DEG)
    warn_angle = math.radians(NEAR_COLLISION_STRAIGHT_WARN_ANGLE_DEG)
    hard_block |= abs_angle <= block_angle

    warn_mask = abs_angle <= warn_angle
    angle_pressure = np.clip(
        (warn_angle - abs_angle) / max(1e-6, warn_angle),
        0.0,
        1.0,
    )
    dist_pressure = np.clip(
        (NEAR_COLLISION_STRAIGHT_BLOCK_DIST - closest_dist)
        / max(1e-6, NEAR_COLLISION_STRAIGHT_BLOCK_DIST),
        0.0,
        1.0,
    )
    penalty += np.where(
        warn_mask,
        NEAR_COLLISION_STRAIGHT_PENALTY_WEIGHT
        * angle_pressure
        * dist_pressure,
        0.0,
    )

    return penalty, hard_block


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


def choose_target_from_gaps(
    angles_deg,
    ranges,
    gaps,
    pose,
    prev_target_angle,
    front_factor,
    left_dist,
    right_dist,
    closest_dist,
    accumulated_turn_rad,
):
    if not gaps:
        return -1, (0, 0), -float("inf")

    goal_angle = goal_heading_error(pose.x, pose.y, pose.theta)
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
        side_penalty, side_hard_block = side_gap_penalty(
            angle_rad,
            left_dist,
            right_dist,
        )
        near_penalty, near_hard_block = near_collision_straight_penalty(
            angle_rad,
            closest_dist,
        )
        turn_penalty, _ = cumulative_turn_penalty(angle_rad, accumulated_turn_rad)

        scores = (
            FGM_CLEARANCE_WEIGHT * clearance_score
            + FGM_EDGE_WEIGHT * edge_score
            + FGM_STRAIGHT_WEIGHT * straight_score
            + goal_weight * goal_score
            + FGM_PREV_TARGET_WEIGHT * prev_score
            + 0.25 * width_score
            - side_penalty
            - near_penalty
            - turn_penalty
        )
        scores = np.where(side_hard_block, -np.inf, scores)
        scores = np.where(near_hard_block, -np.inf, scores)
        if not np.isfinite(scores).any():
            continue

        local_best = int(np.argmax(scores))
        score = float(scores[local_best])
        if score > best_score:
            best_score = score
            best_idx = int(idxs[local_best])
            best_gap = (start, end)

    return best_idx, best_gap, best_score


def choose_fallback_target(
    angles_deg,
    ranges,
    pose,
    left_dist,
    right_dist,
    closest_dist,
    accumulated_turn_rad,
):
    usable = ranges > MIN_LIDAR_DIST_M
    angle_rad = np.deg2rad(angles_deg)
    _, side_hard_block = side_gap_penalty(angle_rad, left_dist, right_dist)
    _, near_hard_block = near_collision_straight_penalty(angle_rad, closest_dist)
    turn_penalty, _ = cumulative_turn_penalty(angle_rad, accumulated_turn_rad)
    usable = usable & ~side_hard_block
    usable = usable & ~near_hard_block
    if not usable.any():
        return len(ranges) // 2
    usable_ranges = np.where(usable, ranges - turn_penalty, -np.inf)
    return int(np.argmax(usable_ranges))


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
    front_dist = front_distance(points)
    front_factor = compute_front_factor(front_dist)
    info_left, info_right = compute_side_info(points)

    angles_deg, raw_ranges, counts = scan_to_angle_ranges(scan)
    smooth_ranges = smooth_ranges_conservative(raw_ranges)
    bubble_ranges, closest_idx, closest_dist, bubble_bins = apply_safety_bubble(
        smooth_ranges, counts
    )

    free_mask = bubble_ranges >= FGM_FREE_DIST
    gaps = filter_gaps_by_width(find_free_gaps(free_mask))
    has_safe_gap = len(gaps) > 0

    target_idx, best_gap, best_score = choose_target_from_gaps(
        angles_deg,
        bubble_ranges,
        gaps,
        pose,
        prev_target_angle,
        front_factor,
        info_left,
        info_right,
        closest_dist,
        accumulated_turn_rad,
    )

    if target_idx < 0:
        has_safe_gap = False
        target_idx = choose_fallback_target(
            angles_deg,
            smooth_ranges,
            pose,
            info_left,
            info_right,
            closest_dist,
            accumulated_turn_rad,
        )
        best_gap = (target_idx, target_idx + 1)
        best_score = 0.0

    target_angle = math.radians(float(angles_deg[target_idx]))
    target_angle = float(np.clip(target_angle, -TURN_HARD_LIMIT_RAD, TURN_HARD_LIMIT_RAD))
    selected_side_penalty, selected_side_block = side_gap_penalty(
        np.array([target_angle], dtype=np.float32),
        info_left,
        info_right,
    )
    selected_near_penalty, selected_near_block = near_collision_straight_penalty(
        np.array([target_angle], dtype=np.float32),
        closest_dist,
    )
    selected_turn_penalty, selected_projected_turn = cumulative_turn_penalty(
        np.array([target_angle], dtype=np.float32),
        accumulated_turn_rad,
    )
    target_dist = float(smooth_ranges[target_idx])
    raw_w = float(np.clip(FGM_TURN_GAIN * target_angle, -MAX_ABS_W, MAX_ABS_W))
    urgent = front_dist < URGENT_FRONT_DIST or not has_safe_gap
    w = rate_limit_w(prev_w, raw_w, urgent=urgent)
    v = choose_speed(target_dist, target_angle, has_safe_gap)

    gap_width = (best_gap[1] - best_gap[0]) * FGM_ANGLE_STEP_DEG
    gap_left = float(angles_deg[best_gap[1] - 1]) if best_gap[1] > best_gap[0] else 0.0
    gap_right = float(angles_deg[best_gap[0]]) if best_gap[1] > best_gap[0] else 0.0
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
        "side_penalty": float(selected_side_penalty[0]),
        "side_block": bool(selected_side_block[0]),
        "near_penalty": float(selected_near_penalty[0]),
        "near_block": bool(selected_near_block[0]),
        "turn_penalty": float(selected_turn_penalty[0]),
        "projected_turn_deg": math.degrees(float(selected_projected_turn[0])),
    }


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


def choose_wall_follow_cmd(scan, prev_w, follow_side):
    points_all = lidar_points_to_xy_all(scan)

    wall_dist = side_wall_distance_from_scan(scan, follow_side)
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

    if front_dist < WALL_FRONT_HARD_STOP_DIST:
        target_v = WALL_SLOW_V

        if follow_side > 0.0:
            target_w = max(target_w, WALL_FRONT_KEEP_TURN_W)
        else:
            target_w = min(target_w, -WALL_FRONT_KEEP_TURN_W)

    elif front_dist < WALL_FRONT_SLOW_DIST:
        target_v = WALL_SLOW_V

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

    wall_follow_active = False
    wall_follow_side = +1.0
    wall_follow_start_time = 0.0
    wall_prev_dist = None
    wall_open_count = 0

    mode_name = "FGM"

    try:
        while True:
            now = time.time()
            dt = max(0.0, min(0.20, now - last_pose_time))
            last_pose_time = now

            if recovery_turn_active:
                recovery_accum_turn += abs(last_w) * dt

            if pose.goal_distance() <= GOAL_TOL_M:
                stop()
                print("[INFO] Goal reached. Stopping.")
                break

            scan = lidar.get_scan()
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

            last_scan_ok = time.time()

            v = 0.0
            w = 0.0
            target_angle = last_target_angle
            fgm_info = None
            narrow_info = None

            wall_dist_log = MAX_LIDAR_DIST_M
            wall_front_log = MAX_LIDAR_DIST_M
            wall_valid_log = False

            recovery_wall_dist_log = MAX_LIDAR_DIST_M
            recovery_front_log = MAX_LIDAR_DIST_M
            recovery_turned_deg_log = math.degrees(recovery_accum_turn)

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

                    v = BASE_V
                    w = rate_limit_w(last_w, 0.0, urgent=False)
                    mode_name = "RETURN_FORWARD"

                    print("[WALL] wall opened. Return to forward driving.")

                else:
                    v = wall_v
                    w = wall_w

                    if wall_follow_side > 0.0:
                        mode_name = "LEFT_WALL_FOLLOW"
                    else:
                        mode_name = "RIGHT_WALL_FOLLOW"

                    wall_prev_dist = wall_dist

            if (not wall_follow_active) and (not recovery_turn_active):
                trigger, blocked_now, no_safe_gap_now, narrow_info = detect_narrow_or_blocked(scan)

                if RECOVERY_TURN_ENABLE and trigger:
                    recovery_turn_dir = choose_initial_based_recovery_dir(
                        pose.theta,
                        last_w,
                    )

                    recovery_follow_side = -recovery_turn_dir

                    recovery_start_time = time.time()
                    recovery_wall_seen_count = 0
                    recovery_accum_turn = 0.0
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

                else:
                    v, w, target_angle, fgm_info = choose_fgm_cmd(
                        scan,
                        last_w,
                        last_target_angle,
                        pose,
                        accumulated_turn_rad,
                    )
                    mode_name = "FGM"

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

                recovery_wall_dist_log = recovery_wall_dist
                recovery_front_log = recovery_front_dist
                recovery_turned_deg_log = math.degrees(recovery_accum_turn)

                wall_ready = recovery_wall_dist <= RECOVERY_WALL_START_DIST
                front_ok = recovery_front_dist >= RECOVERY_FRONT_START_DIST
                min_turn_ok = recovery_accum_turn >= RECOVERY_MIN_TURN_RAD

                if min_turn_ok and wall_ready and front_ok:
                    recovery_wall_seen_count += 1
                else:
                    recovery_wall_seen_count = 0

                recovery_ready_to_wall_follow = (
                    recovery_wall_seen_count >= RECOVERY_WALL_START_COUNT_N
                )

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

                    v = 0.0
                    w = 0.0

                    if wall_follow_side > 0.0:
                        mode_name = "RECOVERY_TO_LEFT_WALL"
                    else:
                        mode_name = "RECOVERY_TO_RIGHT_WALL"

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

                    if recovery_turn_dir > 0.0:
                        mode_name = "RECOVERY_LEFT_TURN"
                    else:
                        mode_name = "RECOVERY_RIGHT_TURN"

            send_vw(v, w)
            pose.update(v, w, dt)
            accumulated_turn_rad += w * dt

            last_v, last_w = v, w
            if fgm_info is not None:
                last_target_angle = target_angle

            gd = pose.goal_distance()
            he = goal_heading_error(pose.x, pose.y, pose.theta)
            accumulated_turn_deg_log = math.degrees(accumulated_turn_rad)

            if gd <= GOAL_TOL_M:
                stop()
                print("[INFO] Goal reached. Stopping.")
                break

            if time.time() - last_log > 0.25:
                if recovery_turn_active:
                    print(
                        f"[{mode_name}] "
                        f"x={pose.x:.2f} y={pose.y:.2f} "
                        f"th={math.degrees(pose.theta):.1f}deg "
                        f"v={v:.2f} w={w:.2f} "
                        f"turned={recovery_turned_deg_log:.1f}deg "
                        f"wall90={recovery_wall_dist_log:.2f} "
                        f"front={recovery_front_log:.2f} "
                        f"seen_cnt={recovery_wall_seen_count}"
                    )

                elif wall_follow_active:
                    if wall_follow_side > 0.0:
                        wall_name = "left_wall_90"
                    else:
                        wall_name = "right_wall_90"

                    print(
                        f"[{mode_name}] "
                        f"x={pose.x:.2f} y={pose.y:.2f} "
                        f"th={math.degrees(pose.theta):.1f}deg "
                        f"v={v:.2f} w={w:.2f} "
                        f"{wall_name}={wall_dist_log:.2f} "
                        f"wall_front={wall_front_log:.2f} "
                        f"valid={int(wall_valid_log)} "
                        f"open_cnt={wall_open_count}"
                    )

                elif fgm_info is not None:
                    print(
                        f"[FGM] x={pose.x:.2f} y={pose.y:.2f} "
                        f"th={pose.theta:.2f} gd={gd:.2f} he={he:.2f} "
                        f"v={v:.2f} w={w:.2f} raw={fgm_info['raw_w']:.2f} "
                        f"ct={accumulated_turn_deg_log:.1f}deg "
                        f"pt={fgm_info['projected_turn_deg']:.1f}deg "
                        f"tp={fgm_info['turn_penalty']:.2f} "
                        f"tgt={fgm_info['target_deg']:.1f} td={fgm_info['target_dist']:.2f} "
                        f"front={fgm_info['front']:.2f} ff={fgm_info['front_factor']:.2f} "
                        f"gap={fgm_info['gap_width']:.0f} "
                        f"gr={fgm_info['gap_right']:.0f} gl={fgm_info['gap_left']:.0f} "
                        f"gaps={fgm_info['gaps']} safe={int(fgm_info['has_safe_gap'])} "
                        f"close={fgm_info['closest']:.2f}@{fgm_info['closest_angle']:.0f} "
                        f"bb={fgm_info['bubble_bins']} score={fgm_info['score']:.2f} "
                        f"pts={fgm_info['points']} coll={int(fgm_info['collision'])} "
                        f"sp={fgm_info['side_penalty']:.2f} sb={int(fgm_info['side_block'])} "
                        f"np={fgm_info['near_penalty']:.2f} nb={int(fgm_info['near_block'])} "
                        f"L={fgm_info['left']:.2f} R={fgm_info['right']:.2f}"
                    )

                else:
                    if narrow_info is not None:
                        print(
                            f"[{mode_name}] "
                            f"x={pose.x:.2f} y={pose.y:.2f} "
                            f"th={math.degrees(pose.theta):.1f}deg "
                            f"v={v:.2f} w={w:.2f} "
                            f"front={narrow_info['front']:.2f} "
                            f"closest={narrow_info['closest']:.2f}@{narrow_info['closest_angle']:.0f} "
                            f"gaps={narrow_info['gaps']} "
                            f"safe={int(narrow_info['has_safe_gap'])} "
                            f"blocked={int(narrow_info['blocked_now'])} "
                            f"no_gap={int(narrow_info['no_safe_gap_now'])}"
                        )
                    else:
                        print(
                            f"[{mode_name}] "
                            f"x={pose.x:.2f} y={pose.y:.2f} "
                            f"th={math.degrees(pose.theta):.1f}deg "
                            f"v={v:.2f} w={w:.2f}"
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
