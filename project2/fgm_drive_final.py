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

CLEARANCE_CAP = 0.6 # FGM - 해당 거리 이상 뚫려 있으면 동일 취급
W_CMD_RATE_LIMIT = 0.30 # 루프당 최대 회전속도 변화량
W_CMD_RATE_LIMIT_URGENT = 0.40 # 긴급 시 허용 변화량
URGENT_FRONT_DIST = 0.30 # 정면이 이 안이면 긴급모드


# 로봇 크기 & 충돌 거리
ROBOT_RADIUS = 0.13 # 로봇 반지름
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
CUM_TURN_SOFT_PENALTY_WEIGHT = 8.0 # 각속도 누적 페널티 강도1
CUM_TURN_HARD_PENALTY_WEIGHT = 50.0 # 각속도 누적 페널티 강도


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
FGM_RANGE_DISCONTINUITY_WEIGHT = 0.70
FGM_RANGE_DISCONTINUITY_CAP = 0.50
FGM_RANGE_DISCONTINUITY_EDGE_WINDOW_DEG = 3.0

SIDE_GAP_WARN_DIST = 0.175 # 옆 경고 시작 거리
SIDE_GAP_BLOCK_DIST = 0.16 # 강하게 거부할 옆 거리
SIDE_GAP_BIAS_MAX_DEG = 10.0 # 조향 보정 최대각도
SIDE_GAP_BIAS_GAIN_DEG_PER_M = 125.0 # 좌우 거리차
SIDE_TIGHT_TURN_LIMIT_DEG = 35.0 # 옆이 매우 좁을 때 회전한계
SIDE_NARROW_TURN_LIMIT_DEG = 45.0 # 옆이 좁을 때 회전한계
SIDE_NARROW_V = 0.12 # 옆이 좁을 때 FGM 속도 상한
SIDE_TIGHT_V = 0.08 # 옆이 매우 좁을 때 FGM 속도 상한


# Recovery Mode
RECOVERY_TURN_W = 1.00 # Recovery 제자리 회전 속도
RECOVERY_MAX_TURN_RAD = math.radians(200.0) # Recovery 최대 회전각도
RECOVERY_TURN_TIMEOUT_S = 15.0

RECOVERY_TURN_ENABLE = True # Recovery 기능 on/off 스위치
RECOVERY_FRONT_OPEN_JUMP_DIST = 0.09
RECOVERY_FRONT_OPEN_PREV_MAX_DIST = 0.45
RECOVERY_FRONT_CHECK_DIST = 0.30
RECOVERY_FRONT_Y_HALF = 0.16


LOOP_DT_S = 0.05 # 메인 제어 루프 주기


# 각도를 -180 ~ +180 범위로 정규화
def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0

# 각도를 -pi ~ +pi 범위로 정규화
def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

# 두 각도의 차이를 -pi ~ +pi 범위로 계산
def angle_error_rad(a, b):
    return normalize_angle_rad(a - b)


# Recovery mode에 들어갈 때 제자리 회전 방향 결정
# theta : 현재 로봇 heading, accumulated_turn_rad : 누적 회전량, prev_w : 직전 회전속도
def choose_initial_based_recovery_dir(theta, accumulated_turn_rad=0.0, prev_w=0.0):
    heading_from_initial = normalize_angle_rad(theta - INITIAL_HEADING_RAD)

    # 왼쪽으로 돌아있는 경우 우회전
    if heading_from_initial > RECOVERY_INITIAL_DEADBAND_RAD:
        return -1.0
    # 오른쪽으로 돌아있는 경우 좌회전
    elif heading_from_initial < -RECOVERY_INITIAL_DEADBAND_RAD:
        return +1.0
    
    # 누적 회전량이 왼쪽으로 쌓인 경우 우회전
    if accumulated_turn_rad > RECOVERY_INITIAL_DEADBAND_RAD:
        return -1.0
    # 누적 회전량이 오른쪽으로 쌓인 경우 좌회전
    elif accumulated_turn_rad < -RECOVERY_INITIAL_DEADBAND_RAD:
        return +1.0

    # 직전 회전속도가 왼쪽이면 우회전
    if prev_w > 0.0:
        return -1.0
    # 직전 회전속도가 오른쪽이면 좌회전
    elif prev_w < 0.0:
        return +1.0

    # 위 조건 모두 만족하지 않으면 우회전 (사실상 발동할 일 없음)
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


# RPLidar 초기화
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


# LiDAR 스캔 데이터를 (x,y) 포인트 배열로 변환
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


def recovery_front_distance(points_all):
    if len(points_all) == 0:
        return MAX_LIDAR_DIST_M

    mask = (
        (points_all[:, 0] > 0.03)
        & (points_all[:, 0] < RECOVERY_FRONT_CHECK_DIST)
        & (np.abs(points_all[:, 1]) < RECOVERY_FRONT_Y_HALF)
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


# 가장 가까운 장애물 주변 각도 범위를 강제로 막는 함수 (안전 버블)
# ranges : 각도별 거리 배열, counts : 각도별 포인트 수
def apply_safety_bubble(ranges, counts):
    working = ranges.copy()
    valid = counts > 0 # 각도별로 실제 측정값이 있는지 확인
    valid_obstacles = valid & (ranges < MAX_LIDAR_DIST_M) # 실제 측정값이 있고, MAX_LIDAR_DIST_M보다 작은 칸만 선택
    if not valid_obstacles.any():
        return working, -1, MAX_LIDAR_DIST_M, 0

    obstacle_ranges = np.where(valid_obstacles, ranges, np.inf) # 장애물 후보인 칸은 실제 거리값을 유지하고, 후보가 아닌 칸은 inf(무한대)로 바꿈
    closest_idx = int(np.argmin(obstacle_ranges)) # 가장 가까운 장애물의 인덱스
    closest_dist = float(ranges[closest_idx]) # 가장 가까운 장애물까지의 거리
    half_angle_rad = math.atan2(FGM_BUBBLE_RADIUS, max(closest_dist, MIN_LIDAR_DIST_M)) # 가장 가까운 장애물 주변을 안전 반경만큼 부풀렸을 때, 그 장애물이 LiDAR 기준으로 중심방향에서 좌우 몇 라디안까지 차지하는지 계산
    half_bins = int(math.ceil(math.degrees(half_angle_rad) / FGM_ANGLE_STEP_DEG)) # half_angle_rad를 각도 격자 칸 수로 변환

    start = max(0, closest_idx - half_bins) # 막을 시작 인덱스
    end = min(len(working), closest_idx + half_bins + 1) # 막을 끝 인덱스
    working[start:end] = 0.0 # 버블 적용

    return working, closest_idx, closest_dist, half_bins


# 연속된 True 구간(=Gap) 찾는 함수
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


# 너무 좁은 Gap 제거
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


# 각속도 누적 페널티 계산 함수
# angle_rad : 후보 목표각, accumulated_turn_rad : 누적된 회전량
def cumulative_turn_penalty(angle_rad, accumulated_turn_rad):
    projected_turn = accumulated_turn_rad + angle_rad # 해당 후보를 선택했을 때의 예상 누적 회전량
    abs_accumulated = abs(accumulated_turn_rad) # 누적 회전량 절댓값
    abs_projected = np.abs(projected_turn) # 예상 누적 회전량 절댓값
    increasing_turn = abs_projected > (abs_accumulated + math.radians(1.0)) # 해당 후보가 누적 회전량을 더 키우는지 확인

    soft_excess = np.maximum(0.0, abs_projected - CUM_TURN_SOFT_LIMIT_RAD) # soft limit 초과량
    hard_excess = np.maximum(0.0, abs_projected - CUM_TURN_HARD_LIMIT_RAD) # hard limit 초과량
    soft_span = max(1e-6, CUM_TURN_HARD_LIMIT_RAD - CUM_TURN_SOFT_LIMIT_RAD) # soft limit 구간

    # 페널티 계산
    soft_penalty = CUM_TURN_SOFT_PENALTY_WEIGHT * (soft_excess / soft_span) ** 2
    hard_penalty = CUM_TURN_HARD_PENALTY_WEIGHT * (hard_excess / math.radians(10.0) + (hard_excess > 0.0))
    penalty = np.where(increasing_turn, soft_penalty + hard_penalty, 0.0) # 누적 회전량을 증가시키는 후보에만 페널티 적용

    return penalty.astype(np.float32), projected_turn


def valid_range_value(value):
    return (
        np.isfinite(value)
        and value >= MIN_LIDAR_DIST_M
        and value <= MAX_LIDAR_DIST_M
    )


def edge_jump_score(ranges, left_idx, right_idx):
    if left_idx < 0 or right_idx >= len(ranges):
        return 0.0

    left_value = float(ranges[left_idx])
    right_value = float(ranges[right_idx])
    if not valid_range_value(left_value) or not valid_range_value(right_value):
        return 0.0

    return float(np.clip(abs(right_value - left_value) / FGM_RANGE_DISCONTINUITY_CAP, 0.0, 1.0))


def gap_edge_discontinuity_score(ranges, start, end):
    edge_window = max(1, int(round(FGM_RANGE_DISCONTINUITY_EDGE_WINDOW_DEG / FGM_ANGLE_STEP_DEG)))
    scores = []

    for offset in range(edge_window):
        scores.append(edge_jump_score(ranges, start - 1 - offset, start + offset))
        scores.append(edge_jump_score(ranges, end - 1 - offset, end + offset))

    return max(scores) if scores else 0.0


# 여러 Gap 중에서 최적의 목표각을 선택하는 함수
def choose_target_from_gaps(angles_deg, ranges, discontinuity_ranges, gaps, pose, prev_target_angle, front_factor, accumulated_turn_rad):
    if not gaps:
        return -1, (0, 0), -float("inf"), []

    goal_angle = normalize_angle_rad(INITIAL_HEADING_RAD - pose.theta) # 출발 방향 기준으로 현재 로봇이 어느 방향에 있는지 계산
    goal_angle = float(np.clip(goal_angle, -TURN_HARD_LIMIT_RAD, TURN_HARD_LIMIT_RAD)) # 최대 목표각 범위 안으로 자르기
    prev_angle = float(np.clip(prev_target_angle, -TURN_HARD_LIMIT_RAD, TURN_HARD_LIMIT_RAD)) # 직전 목표각 범위 안으로 자르기

    best_idx = -1
    best_gap = (0, 0)
    best_score = -float("inf")
    candidate_summaries = []
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

        discontinuity_score = gap_edge_discontinuity_score(discontinuity_ranges, start, end)
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
            + FGM_RANGE_DISCONTINUITY_WEIGHT * discontinuity_score
            - turn_penalty
        )
        if not np.isfinite(scores).any():
            continue

        local_best = int(np.argmax(scores))
        score = float(scores[local_best])
        local_best_idx = int(idxs[local_best])
        candidate_summaries.append(
            f"{angles_deg[start]:.0f}:{angles_deg[end - 1]:.0f}@"
            f"{angles_deg[local_best_idx]:.0f}/{score:.2f}"
        )
        if score > best_score:
            best_score = score
            best_idx = local_best_idx
            best_gap = (start, end)

    return best_idx, best_gap, best_score, candidate_summaries


# 정상적인 gap 선택 실패 시, 임시 목표 방향 선택
# angles_deg : 각도 배열, ranges : 거리 배열, left_dist : 좌측 벽과의 거리, right_dist : 우측 벽과의 거리
def choose_fallback_target(angles_deg, ranges, left_dist=MAX_LIDAR_DIST_M, right_dist=MAX_LIDAR_DIST_M):
    usable = ranges > MIN_LIDAR_DIST_M # 각도별 거리값이 최소 유효 거리보다 큰지 검사
    if not usable.any():
        return len(ranges) // 2 # 배열의 가운데 인덱스 반환 (정면)

    usable_ranges = np.where(usable, ranges, -np.inf) # 사용 가능한 거리값은 그대로 두고, 사용할 수 없는 값은 무한대로 변환

    return int(np.argmax(usable_ranges)) # 가장 큰 거리값을 가진 인덱스 반환


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


# scan : 현재 LiDAR 스캔 데이터, prev_w : 직전 회전속도, prev_target_angle : 직전 목표 각도, pose : 현재 로봇 위치와 방향, accumulated_turn_rad : 누적 회전량
def choose_fgm_cmd(scan, prev_w, prev_target_angle, pose, accumulated_turn_rad=0.0):
    points = lidar_points_to_xy(scan) # LiDAR 스캔을 (x,y) 포인트 배열로 변환
    points_all = lidar_points_to_xy_all(scan) # 모든 LiDAR 스캔을 (x,y) 포인트 배열로 변환
    front_dist = front_distance(points) # 전방 장애물 거리 계산
    front_factor = compute_front_factor(front_dist) # 전방 위험도 계산
    info_left, info_right = compute_side_info(points_all) # 좌우 가까운 벽 거리 계산

    # LiDAR 스캔을 -90~+90 범위의 1도 간격 배열로 변환
    # angles_deg : 각도 배열, raw_ranges : 각도별 최소 거리 배열, counts : 각도별 포인트 수
    angles_deg, raw_ranges, counts = scan_to_angle_ranges(scan)
    smooth_ranges = smooth_ranges_conservative(raw_ranges) # 거리 배열 평활화

    # bubble_ranges : 안전 버블 적용된 거리 배열, closest_idx : 가장 가까운 장애물의 각도 인덱스, closest_dist : 가장 가까운 장애물의 거리, bubble_bins : 그 장애물 주변 몇 칸을 막았는지
    bubble_ranges, closest_idx, closest_dist, bubble_bins = apply_safety_bubble(smooth_ranges, counts) # 안전 버블 적용

    free_mask = bubble_ranges >= FGM_FREE_DIST # 지나갈 수 있는 칸인지 확인
    gaps = filter_gaps_by_width(find_free_gaps(free_mask)) # 연속된 안전한 구간(=Gap) 찾고, 너무 좁은 Gap 제거
    has_safe_gap = len(gaps) > 0 # 안전한 Gap이 있는지 여부

    # 최종적으로 어느 gap의 어느 각도를 목표로 할지 선택
    # target_idx : 목표 각도 인덱스, best_gap : 그 각도가 속한 Gap의 시작과 끝 인덱스, best_score : 그 후보의 점수, gap_candidates : 후보 요약 문자열 리스트
    target_idx, best_gap, best_score, gap_candidates = choose_target_from_gaps(angles_deg, bubble_ranges, smooth_ranges, gaps, pose, prev_target_angle, front_factor, accumulated_turn_rad)

    # safe gap이 없거나, 그 안에서 선택 가능한 후보가 없는 경우
    if target_idx < 0:
        target_idx = choose_fallback_target(angles_deg, smooth_ranges, info_left, info_right)
        # len(angles_deg) // 2
        
        best_gap = (target_idx, target_idx + 1)
        best_score = 0.0
        gap_candidates = []

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
    selected_turn_penalty, _ = cumulative_turn_penalty(
        np.array([target_angle], dtype=np.float32),
        accumulated_turn_rad,
    )
    urgent = front_dist < URGENT_FRONT_DIST or not has_safe_gap
    w = rate_limit_w(prev_w, raw_w, urgent=urgent)
    v = choose_speed(target_dist, target_angle, has_safe_gap)
    v = min(v, side_gap_speed_limit(info_left, info_right))

    closest_angle = float(angles_deg[closest_idx]) if closest_idx >= 0 else 0.0
    if closest_idx >= 0:
        bubble_half_angle_deg = math.degrees(
            math.atan2(FGM_BUBBLE_RADIUS, max(closest_dist, MIN_LIDAR_DIST_M))
        )
        bubble_start_idx = max(0, closest_idx - bubble_bins)
        bubble_end_idx = min(len(angles_deg), closest_idx + bubble_bins + 1)
        bubble_right = float(angles_deg[bubble_start_idx])
        bubble_left = float(angles_deg[bubble_end_idx - 1])
    else:
        bubble_half_angle_deg = 0.0
        bubble_right = 0.0
        bubble_left = 0.0

    return v, w, target_angle, {
        "score": best_score,
        "target_deg": math.degrees(target_angle),
        "target_dist": target_dist,
        "front": front_dist,
        "left": info_left,
        "right": info_right,
        "gaps": len(gaps),
        "gap_candidates": ",".join(gap_candidates) if gap_candidates else "none",
        "gap_width": gap_width,
        "gap_right": gap_right,
        "gap_left": gap_left,
        "closest": closest_dist,
        "closest_angle": closest_angle,
        "bubble_bins": bubble_bins,
        "bubble_half_angle_deg": bubble_half_angle_deg,
        "bubble_right": bubble_right,
        "bubble_left": bubble_left,
        "collision": target_dist < COLLISION_DIST or closest_dist < COLLISION_DIST,
        "raw_w": raw_w,
        "has_safe_gap": has_safe_gap,
        "side_bias_deg": math.degrees(side_bias),
        "side_turn_limit_deg": math.degrees(side_turn_limit),
        "turn_penalty": float(selected_turn_penalty[0]),
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
    last_processed_scan_seq = -1
    stale_scan_warned = False
    last_v, last_w = BASE_V, 0.0
    last_target_angle = 0.0
    last_log = 0.0
    last_pose_time = time.time()
    accumulated_turn_rad = 0.0

    recovery_turn_active = False
    recovery_turn_dir = 0.0
    recovery_start_time = 0.0
    recovery_accum_turn = 0.0
    recovery_prev_front_dist = None

    recovery_turned_deg_log = 0.0
    recovery_front_dist_log = MAX_LIDAR_DIST_M

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

            recovery_turned_deg_log = math.degrees(recovery_accum_turn)
            recovery_front_dist_log = MAX_LIDAR_DIST_M

            if not recovery_turn_active:
                v, w, target_angle, info = choose_fgm_cmd(
                    scan,
                    last_w,
                    last_target_angle,
                    pose,
                    accumulated_turn_rad,
                )

                blocked_now = info["collision"] and v <= 0.01 # 충돌 위험을 감지했고, 속도가 거의 0인 경우
                no_safe_gap_now = not info["has_safe_gap"] # safe gap이 없는 경우
                recovery_trigger = (blocked_now or no_safe_gap_now) # recovery 켤지 결정

                if (RECOVERY_TURN_ENABLE and (not recovery_turn_active) and recovery_trigger):
                    recovery_turn_dir = choose_initial_based_recovery_dir(pose.theta, accumulated_turn_rad, last_w) # 제자리 회전 방향 결정
                    recovery_start_time = time.time()
                    recovery_accum_turn = 0.0
                    recovery_prev_front_dist = None
                    recovery_turn_active = True

                    if blocked_now:
                        trigger_name = "blocked/narrow path"
                    else:
                        trigger_name = "no safe path"

                    if recovery_turn_dir > 0.0:
                        print(
                            f"[NARROW] {trigger_name}. "
                            "Start LEFT recovery turn."
                        )
                    else:
                        print(
                            f"[NARROW] {trigger_name}. "
                            "Start RIGHT recovery turn."
                        )

            if (
                RECOVERY_TURN_ENABLE
                and recovery_turn_active
            ):
                recovery_timeout = (
                    time.time() - recovery_start_time
                ) > RECOVERY_TURN_TIMEOUT_S

                recovery_max_turn_reached = recovery_accum_turn >= RECOVERY_MAX_TURN_RAD

                recovery_points_all = lidar_points_to_xy_all(scan)
                recovery_front_dist = recovery_front_distance(recovery_points_all)

                recovery_turned_deg_log = math.degrees(recovery_accum_turn)
                recovery_front_dist_log = recovery_front_dist

                recovery_open_detected = (
                    recovery_prev_front_dist is not None
                    and recovery_prev_front_dist <= RECOVERY_FRONT_OPEN_PREV_MAX_DIST
                    and (recovery_front_dist - recovery_prev_front_dist) >= RECOVERY_FRONT_OPEN_JUMP_DIST
                )

                recovery_ready_to_fgm = recovery_open_detected

                recovery_prev_front_dist = recovery_front_dist

                if recovery_ready_to_fgm:
                    recovery_turn_active = False
                    recovery_prev_front_dist = None

                    recovery_mode_name = "FGM_RETURN_FROM_RECOVERY"

                    print(
                        f"[RECOVERY] front opened after {recovery_turned_deg_log:.1f}deg. "
                        f"front={recovery_front_dist:.2f}. "
                        "Return to FGM driving."
                    )

                    v, w, target_angle, info = choose_fgm_cmd(
                        scan,
                        last_w,
                        last_target_angle,
                        pose,
                        accumulated_turn_rad,
                    )

                elif recovery_max_turn_reached or recovery_timeout:
                    recovery_turn_active = False
                    recovery_prev_front_dist = None

                    recovery_mode_name = "FGM_RETURN_FROM_RECOVERY"

                    if recovery_max_turn_reached:
                        print(
                            f"[RECOVERY] max turn reached. "
                            f"turned={recovery_turned_deg_log:.1f}deg. "
                            "Return to FGM driving."
                        )
                    else:
                        print(
                            "[RECOVERY] recovery turn timeout. "
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

            if not recovery_turn_active:
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
                        f"rfront={recovery_front_dist_log:.2f} "
                        f"front={front_log:.2f} "
                        f"gaps={gaps_log} safe={safe_log} "
                        f"close={close_log:.2f}@{close_angle_log:.0f} "                        
                        f"L={left_log:.2f} R={right_log:.2f}"
                    )

                elif info is not None:
                    print(
                        f"[{recovery_mode_name}] x={pose.x:.2f} y={pose.y:.2f} "
                        f"th={pose.theta:.2f} "
                        f"v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                        f"ct={accumulated_turn_deg_log:.1f}deg "
                        f"tp={info['turn_penalty']:.2f} "
                        f"tgt={info['target_deg']:.1f} td={info['target_dist']:.2f} "
                        f"front={info['front']:.2f} "
                        f"gap={info['gap_width']:.0f} "
                        f"gr={info['gap_right']:.0f} gl={info['gap_left']:.0f} "
                        f"gaps={info['gaps']} safe={int(info['has_safe_gap'])} "
                        f"cands={info['gap_candidates']} "
                        f"close={info['closest']:.2f}@{info['closest_angle']:.0f} "
                        f"bub={info['bubble_half_angle_deg']:.1f}deg/{info['bubble_bins']} "
                        f"br={info['bubble_right']:.0f} bl={info['bubble_left']:.0f} "
                        f"score={info['score']:.2f} "
                        f"coll={int(info['collision'])} "
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
