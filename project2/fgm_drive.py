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
MAX_ABS_W = 0.70

ROBOT_RADIUS = 0.16
COLLISION_DIST = ROBOT_RADIUS + 0.05
CLEARANCE_CAP = 0.6
FRONT_CORRIDOR_HALF = COLLISION_DIST + 0.30
ACTIVE_FRONT_DIST = 0.30
FRONT_DANGER_DIST = 0.19

W_CMD_RATE_LIMIT = 0.30
W_CMD_RATE_LIMIT_URGENT = 0.40
URGENT_FRONT_DIST = 0.30

GOAL_X_M = 3.0
GOAL_Y_M = 0.0
GOAL_TOL_M = 0.15
TURN_SOFT_LIMIT_RAD = math.radians(70.0)
TURN_HARD_LIMIT_RAD = math.radians(80.0)

FGM_MIN_ANGLE_DEG = -90.0
FGM_MAX_ANGLE_DEG = 90.0
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
GAP_WIDTH_MARGIN = 0.04
GAP_WIDTH_MIN_M = 2.0 * ROBOT_RADIUS + GAP_WIDTH_MARGIN
GAP_WIDTH_PENALTY_WEIGHT = 5.0
CANDIDATE_CORRIDOR_LOOKAHEAD = 0.45
CANDIDATE_CORRIDOR_HALF_WIDTH = ROBOT_RADIUS + 0.03
CANDIDATE_CORRIDOR_HARD_DIST = COLLISION_DIST + 0.10
CANDIDATE_CORRIDOR_PENALTY_WEIGHT = 6.0


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_error_rad(a, b):
    return normalize_angle_rad(a - b)


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


def gap_physical_width(angles_deg, ranges, start, end):
    if end <= start:
        return 0.0

    right_idx = start
    left_idx = end - 1
    right_angle = math.radians(float(angles_deg[right_idx]))
    left_angle = math.radians(float(angles_deg[left_idx]))
    right_dist = float(ranges[right_idx])
    left_dist = float(ranges[left_idx])

    right_x = right_dist * math.cos(right_angle)
    right_y = right_dist * math.sin(right_angle)
    left_x = left_dist * math.cos(left_angle)
    left_y = left_dist * math.sin(left_angle)

    return math.hypot(left_x - right_x, left_y - right_y)


def gap_width_penalty(width_m):
    if width_m >= GAP_WIDTH_MIN_M:
        return 0.0, False

    pressure = np.clip(
        (GAP_WIDTH_MIN_M - width_m) / max(1e-6, GAP_WIDTH_MIN_M),
        0.0,
        1.0,
    )
    return float(GAP_WIDTH_PENALTY_WEIGHT * pressure), width_m < ROBOT_RADIUS


def candidate_corridor_penalty(angle_rad, points):
    penalty = np.zeros_like(angle_rad, dtype=np.float32)
    hard_block = np.zeros_like(angle_rad, dtype=bool)

    if len(points) == 0:
        return penalty, hard_block

    px = points[:, 0]
    py = points[:, 1]

    for i, angle in enumerate(angle_rad):
        c = math.cos(float(angle))
        s = math.sin(float(angle))
        forward = px * c + py * s
        lateral = -px * s + py * c
        in_corridor = (
            (forward > 0.03)
            & (forward < CANDIDATE_CORRIDOR_LOOKAHEAD)
            & (np.abs(lateral) < CANDIDATE_CORRIDOR_HALF_WIDTH)
        )
        if not in_corridor.any():
            continue

        nearest = float(np.min(forward[in_corridor]))
        pressure = np.clip(
            (CANDIDATE_CORRIDOR_LOOKAHEAD - nearest)
            / max(1e-6, CANDIDATE_CORRIDOR_LOOKAHEAD),
            0.0,
            1.0,
        )
        penalty[i] = CANDIDATE_CORRIDOR_PENALTY_WEIGHT * pressure
        if nearest <= CANDIDATE_CORRIDOR_HARD_DIST:
            hard_block[i] = True

    return penalty, hard_block


def choose_target_from_gaps(
    angles_deg,
    ranges,
    gaps,
    points,
    pose,
    prev_target_angle,
    front_factor,
    left_dist,
    right_dist,
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
        physical_width = gap_physical_width(angles_deg, ranges, start, end)
        physical_width_penalty, physical_width_block = gap_width_penalty(
            physical_width
        )

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
        corridor_penalty, corridor_hard_block = candidate_corridor_penalty(
            angle_rad,
            points,
        )

        scores = (
            FGM_CLEARANCE_WEIGHT * clearance_score
            + FGM_EDGE_WEIGHT * edge_score
            + FGM_STRAIGHT_WEIGHT * straight_score
            + goal_weight * goal_score
            + FGM_PREV_TARGET_WEIGHT * prev_score
            + 0.25 * width_score
            - side_penalty
            - physical_width_penalty
            - corridor_penalty
        )
        if physical_width_block:
            scores[:] = -np.inf
        scores = np.where(side_hard_block, -np.inf, scores)
        scores = np.where(corridor_hard_block, -np.inf, scores)
        if not np.isfinite(scores).any():
            continue

        local_best = int(np.argmax(scores))
        score = float(scores[local_best])
        if score > best_score:
            best_score = score
            best_idx = int(idxs[local_best])
            best_gap = (start, end)

    return best_idx, best_gap, best_score


def choose_fallback_target(angles_deg, ranges, points, left_dist, right_dist):
    usable = ranges > MIN_LIDAR_DIST_M
    angle_rad = np.deg2rad(angles_deg)
    _, side_hard_block = side_gap_penalty(angle_rad, left_dist, right_dist)
    _, corridor_hard_block = candidate_corridor_penalty(angle_rad, points)
    usable = usable & ~side_hard_block
    usable = usable & ~corridor_hard_block
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


def choose_fgm_cmd(scan, prev_w, prev_target_angle, pose):
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
        points,
        pose,
        prev_target_angle,
        front_factor,
        info_left,
        info_right,
    )

    if target_idx < 0:
        has_safe_gap = False
        target_idx = choose_fallback_target(
            angles_deg,
            smooth_ranges,
            points,
            info_left,
            info_right,
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
    selected_corridor_penalty, selected_corridor_block = candidate_corridor_penalty(
        np.array([target_angle], dtype=np.float32),
        points,
    )
    target_dist = float(smooth_ranges[target_idx])
    raw_w = float(np.clip(FGM_TURN_GAIN * target_angle, -MAX_ABS_W, MAX_ABS_W))
    urgent = front_dist < URGENT_FRONT_DIST or not has_safe_gap
    w = rate_limit_w(prev_w, raw_w, urgent=urgent)
    v = choose_speed(target_dist, target_angle, has_safe_gap)

    gap_width = (best_gap[1] - best_gap[0]) * FGM_ANGLE_STEP_DEG
    gap_left = float(angles_deg[best_gap[1] - 1]) if best_gap[1] > best_gap[0] else 0.0
    gap_right = float(angles_deg[best_gap[0]]) if best_gap[1] > best_gap[0] else 0.0
    gap_width_m = gap_physical_width(
        angles_deg,
        bubble_ranges,
        best_gap[0],
        best_gap[1],
    )
    selected_width_penalty, selected_width_block = gap_width_penalty(gap_width_m)
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
        "gap_width_m": gap_width_m,
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
        "width_penalty": selected_width_penalty,
        "width_block": selected_width_block,
        "corridor_penalty": float(selected_corridor_penalty[0]),
        "corridor_block": bool(selected_corridor_block[0]),
    }


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

    try:
        while True:
            now = time.time()
            dt = max(0.0, min(0.20, now - last_pose_time))
            last_pose_time = now

            if pose.goal_distance() <= GOAL_TOL_M:
                stop()
                print("[INFO] Goal reached. Stopping.")
                break

            scan = lidar.get_scan()
            if scan is None:
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    send_vw(last_v, last_w)
                    pose.update(last_v, last_w, dt)
                else:
                    send_vw(0.0, 0.0)
                    pose.update(0.0, 0.0, dt)
                time.sleep(LOOP_DT_S)
                continue

            last_scan_ok = time.time()
            v, w, target_angle, info = choose_fgm_cmd(scan, last_w, last_target_angle, pose)
            send_vw(v, w)
            pose.update(v, w, dt)
            last_v, last_w = v, w
            last_target_angle = target_angle

            gd = pose.goal_distance()
            he = goal_heading_error(pose.x, pose.y, pose.theta)
            if gd <= GOAL_TOL_M:
                stop()
                print("[INFO] Goal reached. Stopping.")
                break

            if time.time() - last_log > 0.25:
                print(
                    f"[FGM] x={pose.x:.2f} y={pose.y:.2f} "
                    f"th={pose.theta:.2f} gd={gd:.2f} he={he:.2f} "
                    f"v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                    f"tgt={info['target_deg']:.1f} td={info['target_dist']:.2f} "
                    f"front={info['front']:.2f} ff={info['front_factor']:.2f} "
                    f"gap={info['gap_width']:.0f} "
                    f"gw={info['gap_width_m']:.2f} "
                    f"gr={info['gap_right']:.0f} gl={info['gap_left']:.0f} "
                    f"gaps={info['gaps']} safe={int(info['has_safe_gap'])} "
                    f"close={info['closest']:.2f}@{info['closest_angle']:.0f} "
                    f"bb={info['bubble_bins']} score={info['score']:.2f} "
                    f"pts={info['points']} coll={int(info['collision'])} "
                    f"sp={info['side_penalty']:.2f} sb={int(info['side_block'])} "
                    f"wp={info['width_penalty']:.2f} wb={int(info['width_block'])} "
                    f"cp={info['corridor_penalty']:.2f} cb={int(info['corridor_block'])} "
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
