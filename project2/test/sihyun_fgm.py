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

ANGLE_OFFSET_DEG = +1.54
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

# ============================================================
# 안전한 gap이 없을 때 처음 방향 기준 180도 회전 Recovery 설정
# ============================================================
INITIAL_HEADING_RAD = 0.0             # 시작 방향 기준. pose.reset() 후 theta=0
RECOVERY_TURN_ENABLE = True
RECOVERY_TURN_W = 0.45                # 180도 제자리 회전 속도. 너무 느리면 0.55 정도까지 가능
RECOVERY_TURN_TOL_RAD = math.radians(8.0)   # 목표 180도에서 이 각도 안에 들어오면 회전 완료
RECOVERY_TURN_TIMEOUT_S = 8.0         # 회전이 너무 오래 걸릴 때 강제 종료
RECOVERY_INITIAL_DEADBAND_RAD = math.radians(2.0)
# ============================================================

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


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_error_rad(a, b):
    return normalize_angle_rad(a - b)


def choose_initial_based_recovery_dir(theta, prev_w=0.0):
    """
    처음 방향 INITIAL_HEADING_RAD 기준으로 현재 로봇이 어느 쪽으로 휘었는지 판단한다.

    theta > 0  : 처음 방향 기준 왼쪽으로 휘어 있음 -> 왼쪽 180도 회전
    theta < 0  : 처음 방향 기준 오른쪽으로 휘어 있음 -> 오른쪽 180도 회전

    반환값:
    +1.0 : 왼쪽 회전
    -1.0 : 오른쪽 회전
    """
    heading_from_initial = normalize_angle_rad(theta - INITIAL_HEADING_RAD)

    if heading_from_initial > RECOVERY_INITIAL_DEADBAND_RAD:
        return +1.0
    if heading_from_initial < -RECOVERY_INITIAL_DEADBAND_RAD:
        return -1.0

    # 거의 정면이면 직전 회전 방향을 따라감
    if prev_w > 0.0:
        return +1.0
    if prev_w < 0.0:
        return -1.0

    # 완전히 애매하면 기본 왼쪽
    return +1.0


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


def choose_target_from_gaps(angles_deg, ranges, gaps, pose, prev_target_angle, front_factor):
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

        scores = (
            FGM_CLEARANCE_WEIGHT * clearance_score
            + FGM_EDGE_WEIGHT * edge_score
            + FGM_STRAIGHT_WEIGHT * straight_score
            + goal_weight * goal_score
            + FGM_PREV_TARGET_WEIGHT * prev_score
            + 0.25 * width_score
        )

        local_best = int(np.argmax(scores))
        score = float(scores[local_best])
        if score > best_score:
            best_score = score
            best_idx = int(idxs[local_best])
            best_gap = (start, end)

    return best_idx, best_gap, best_score


def choose_fallback_target(angles_deg, ranges):
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
        angles_deg, bubble_ranges, gaps, pose, prev_target_angle, front_factor
    )

    if target_idx < 0:
        target_idx = choose_fallback_target(angles_deg, smooth_ranges)
        best_gap = (target_idx, target_idx + 1)
        best_score = 0.0

    target_angle = math.radians(float(angles_deg[target_idx]))
    target_angle = float(np.clip(target_angle, -TURN_HARD_LIMIT_RAD, TURN_HARD_LIMIT_RAD))
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

    # ============================================================
    # Recovery turn 상태 변수
    # ============================================================
    recovery_turn_active = False
    recovery_turn_dir = 0.0
    recovery_target_theta = 0.0
    recovery_start_time = 0.0
    # ============================================================

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

            # 기본 FGM 명령 계산
            v, w, target_angle, info = choose_fgm_cmd(scan, last_w, last_target_angle, pose)

            # ============================================================
            # 안전한 gap이 없으면 처음 방향 기준으로 180도 제자리 회전
            # ============================================================
            recovery_remaining_deg = 0.0
            recovery_mode_name = "FGM"

            if RECOVERY_TURN_ENABLE:
                # 아직 recovery 중이 아니고, 안전한 gap이 없으면 recovery 시작
                if (not recovery_turn_active) and (not info["has_safe_gap"]):
                    recovery_turn_dir = choose_initial_based_recovery_dir(pose.theta, last_w)
                    recovery_target_theta = normalize_angle_rad(
                        pose.theta + recovery_turn_dir * math.pi
                    )
                    recovery_start_time = time.time()
                    recovery_turn_active = True

                    if recovery_turn_dir > 0.0:
                        print(
                            "[RECOVERY] No safe gap. "
                            "Initial 기준 왼쪽으로 휘어 있음 -> LEFT 180 turn start."
                        )
                    else:
                        print(
                            "[RECOVERY] No safe gap. "
                            "Initial 기준 오른쪽으로 휘어 있음 -> RIGHT 180 turn start."
                        )

                # recovery 중이면 FGM 명령을 무시하고 제자리 회전 명령으로 덮어쓰기
                if recovery_turn_active:
                    err_to_target = angle_error_rad(recovery_target_theta, pose.theta)
                    recovery_remaining_deg = abs(math.degrees(err_to_target))
                    recovery_timeout = (time.time() - recovery_start_time) > RECOVERY_TURN_TIMEOUT_S

                    if recovery_remaining_deg <= math.degrees(RECOVERY_TURN_TOL_RAD) or recovery_timeout:
                        # 180도 회전 완료
                        recovery_turn_active = False
                        v = 0.0
                        w = 0.0
                        target_angle = 0.0
                        recovery_mode_name = "RECOVERY_DONE"

                        if recovery_timeout:
                            print("[RECOVERY] 180 turn timeout. Stop recovery and return to FGM.")
                        else:
                            print("[RECOVERY] 180 turn complete. Return to FGM.")

                    else:
                        # 제자리 180도 회전
                        v = 0.0
                        target_w = recovery_turn_dir * RECOVERY_TURN_W
                        target_w = float(np.clip(target_w, -MAX_ABS_W, MAX_ABS_W))
                        w = rate_limit_w(last_w, target_w, urgent=True)
                        target_angle = 0.0

                        if recovery_turn_dir > 0.0:
                            recovery_mode_name = "RECOVERY_LEFT_180"
                        else:
                            recovery_mode_name = "RECOVERY_RIGHT_180"
            # ============================================================

            send_vw(v, w)
            pose.update(v, w, dt)
            last_v, last_w = v, w

            # recovery 중에는 FGM target angle을 기억하지 않음
            if not recovery_turn_active:
                last_target_angle = target_angle

            gd = pose.goal_distance()
            he = goal_heading_error(pose.x, pose.y, pose.theta)
            if gd <= GOAL_TOL_M:
                stop()
                print("[INFO] Goal reached. Stopping.")
                break

            if time.time() - last_log > 0.25:
                if recovery_turn_active:
                    print(
                        f"[{recovery_mode_name}] x={pose.x:.2f} y={pose.y:.2f} "
                        f"th={math.degrees(pose.theta):.1f}deg "
                        f"v={v:.2f} w={w:.2f} "
                        f"remain={recovery_remaining_deg:.1f}deg "
                        f"target_th={math.degrees(recovery_target_theta):.1f}deg "
                        f"front={info['front']:.2f} "
                        f"gaps={info['gaps']} safe={int(info['has_safe_gap'])} "
                        f"close={info['closest']:.2f}@{info['closest_angle']:.0f} "
                        f"L={info['left']:.2f} R={info['right']:.2f}"
                    )
                else:
                    print(
                        f"[FGM] x={pose.x:.2f} y={pose.y:.2f} "
                        f"th={pose.theta:.2f} gd={gd:.2f} he={he:.2f} "
                        f"v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                        f"tgt={info['target_deg']:.1f} td={info['target_dist']:.2f} "
                        f"front={info['front']:.2f} ff={info['front_factor']:.2f} "
                        f"gap={info['gap_width']:.0f} "
                        f"gr={info['gap_right']:.0f} gl={info['gap_left']:.0f} "
                        f"gaps={info['gaps']} safe={int(info['has_safe_gap'])} "
                        f"close={info['closest']:.2f}@{info['closest_angle']:.0f} "
                        f"bb={info['bubble_bins']} score={info['score']:.2f} "
                        f"pts={info['points']} coll={int(info['collision'])} "
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
