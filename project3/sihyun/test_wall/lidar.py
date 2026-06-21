#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import numpy as np
import serial



LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

RESET = b"\xA5\x40"
SCAN = b"\xA5\x20"
LIDAR_STOP = b"\xA5\x25"

ANGLE_OFFSET = 1.54
DIST_OFFSET = 0.0
ANGLE_SIGN = 1.0
MIN_Q = 1
MIN_D = 0.01
MAX_D = 2.5

ANG_MIN = -100.0
ANG_MAX = 100.0
ANG_STEP = 1.0
FREE_D = 0.30
MIN_GAP_DEG = 8.0
OBSTACLE_FRONT_DEG = 100.0
FRONT_HALF_DEG = 15.0
BRAKE_HALF_DEG = 45.0  # 전진 감속에 쓰는 전방 콘 반각(deg). 옆구리(90°) 평행벽은 제외

AVOID_BASE_V = 0.18
AVOID_MAX_W = 1.0
AVOID_TURN_GAIN = 1.5
AVOID_TURN_SIGN = -1.0
SIDE_CLEAR_D = 0.20
SIDE_CORRECT_MAX_DEG = 30.0
COLOR_TO_LIDAR_DEG = 45.0
OPPOSITE_WALL_GAIN = 1.5  # 색 추적 회피 시 색 반대쪽 벽에서 멀어지는 반발 가중치
MIN_OBSTACLE_BINS = 3  # 노이즈 제거: 가까운 측정이 이 개수 이상 모일 때만 장애물로 인정(단일 헛값 무시)

GRID = np.arange(ANG_MIN, ANG_MAX + 0.5 * ANG_STEP, ANG_STEP, dtype=np.float32)
LEFT_ZONE = (GRID >= ANG_MIN) & (GRID < -FRONT_HALF_DEG)
FRONT_LOG_ZONE = (GRID >= -FRONT_HALF_DEG) & (GRID <= FRONT_HALF_DEG)
FRONT_BRAKE_ZONE = (GRID >= -BRAKE_HALF_DEG) & (GRID <= BRAKE_HALF_DEG)
RIGHT_ZONE = (GRID > FRONT_HALF_DEG) & (GRID <= ANG_MAX)
OBSTACLE_ZONE = (GRID >= -OBSTACLE_FRONT_DEG) & (GRID <= OBSTACLE_FRONT_DEG)
MIN_GAP_BINS = max(1, int(np.ceil(MIN_GAP_DEG / ANG_STEP)))


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def norm_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


class RPLidarC1:
    def __init__(self):
        self.ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
        self.lock = threading.Lock()
        self.running = True
        self.scan = None
        self.scan_time = 0.0
        self.scan_seq = 0

        self.ser.write(RESET)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(SCAN)

        header = self.ser.read(7)
        if len(header) != 7 or header[:2] != b"\xA5\x5A":
            self.ser.close()
            raise RuntimeError("[LIDAR] response header error")

        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        angles, dists, qualities = [], [], []

        while self.running:
            try:
                packet = self.ser.read(5)
                if len(packet) != 5:
                    continue

                start = packet[0] & 1
                if ((packet[0] & 2) >> 1) != (1 - start) or (packet[1] & 1) != 1:
                    continue

                quality = packet[0] >> 2
                angle = ((packet[1] >> 1) | (packet[2] << 7)) / 64.0
                dist = (packet[3] | (packet[4] << 8)) / 4.0

                if start and len(angles) > 50:
                    scan = (
                        np.array(angles, dtype=np.float32),
                        np.array(dists, dtype=np.float32),
                        np.array(qualities, dtype=np.float32),
                    )

                    with self.lock:
                        self.scan = scan
                        self.scan_time = time.time()
                        self.scan_seq += 1

                    angles, dists, qualities = [], [], []

                if dist > 0 and quality >= MIN_Q:
                    angles.append(angle)
                    dists.append(dist)
                    qualities.append(quality)

            except (serial.SerialException, OSError):
                time.sleep(1.0)

    def get(self):
        with self.lock:
            return self.scan, self.scan_time, self.scan_seq

    def close(self):
        self.running = False

        try:
            self.ser.write(LIDAR_STOP)
        except Exception:
            pass

        self.thread.join(timeout=0.5)

        if self.ser.is_open:
            self.ser.close()


def front_ranges(scan):
    ranges = np.full(len(GRID), MAX_D, dtype=np.float32)

    if scan is None:
        return ranges

    angles, dists, qualities = scan
    dist_m = (dists + DIST_OFFSET) / 1000.0
    angle_deg = norm_deg(angles + ANGLE_OFFSET) * ANGLE_SIGN

    valid = (dist_m >= MIN_D) & (dist_m <= MAX_D) & (qualities >= MIN_Q)
    bins = np.rint((angle_deg[valid] - ANG_MIN) / ANG_STEP).astype(np.int32)
    dists_valid = dist_m[valid]
    in_grid = (bins >= 0) & (bins < len(ranges))

    np.minimum.at(ranges, bins[in_grid], dists_valid[in_grid])
    return ranges


def find_gaps(free):
    gaps = []
    start = None

    for idx, ok in enumerate(free):
        if ok and start is None:
            start = idx
        elif not ok and start is not None:
            gaps.append((start, idx))
            start = None

    if start is not None:
        gaps.append((start, len(free)))

    return [(start, end) for start, end in gaps if end - start >= MIN_GAP_BINS]


def obstacle_detected(ranges):
    if ranges is None:
        return False

    return zone_min_distance(ranges, OBSTACLE_ZONE) < FREE_D


def lidar_zone_distances(ranges):
    if ranges is None:
        return None

    def zone_distance(zone):
        values = ranges[zone]
        measured = values[values < MAX_D]
        if len(measured) == 0:
            return MAX_D, 0
        return float(np.min(measured)), len(measured)

    return (
        zone_distance(LEFT_ZONE),
        zone_distance(FRONT_LOG_ZONE),
        zone_distance(RIGHT_ZONE),
    )


def zone_min_distance(ranges, zone):
    """존 내 최단 거리. 단, 단일/이중 헛값에 안 흔들리도록 MIN_OBSTACLE_BINS번째로
    가까운 측정값을 사용한다(가까운 측정이 그만큼 안 모이면 장애물 없음=MAX_D)."""
    values = ranges[zone]
    measured = np.sort(values[values < MAX_D])
    if len(measured) < MIN_OBSTACLE_BINS:
        return MAX_D
    return float(measured[MIN_OBSTACLE_BINS - 1])


def avoid_cmd(ranges, color_deg=None, base_v=AVOID_BASE_V, color_follow=False):
    if ranges is None:
        return 0.0, 0.0, 0.0, 0

    safe_gaps = find_gaps(ranges >= FREE_D)

    if not safe_gaps:
        return 0.0, 0.0, 0.0, 0

    def gap_width(gap):
        start, end = gap
        return end - start

    def gap_center(gap):
        start, end = gap
        return 0.5 * (GRID[start] + GRID[end - 1])

    front_blocked = zone_min_distance(ranges, FRONT_LOG_ZONE) < FREE_D

    if color_deg is not None and len(safe_gaps) >= 2 and not front_blocked:
        start, end = min(safe_gaps, key=lambda gap: abs(norm_deg(gap_center(gap) - color_deg)))
    else:
        start, end = max(safe_gaps, key=lambda gap: (gap_width(gap), -abs(gap_center(gap))))

    target_deg = float(0.5 * (GRID[start] + GRID[end - 1]))
    left_d = zone_min_distance(ranges, LEFT_ZONE)
    right_d = zone_min_distance(ranges, RIGHT_ZONE)
    left_risk = max(0.0, SIDE_CLEAR_D - left_d)
    right_risk = max(0.0, SIDE_CLEAR_D - right_d)
    if color_follow and color_deg is not None:
        # 색 추적 회피: 색 반대쪽 벽에서만 멀어지도록 비대칭 반발(색 쪽은 보정 안 함).
        # target_deg는 양수=오른쪽. 색이 오른쪽(>=0)이면 반대쪽 왼쪽 벽 → 오른쪽(+)으로 밀기.
        if color_deg >= 0.0:
            side_correct_deg = OPPOSITE_WALL_GAIN * left_risk / SIDE_CLEAR_D * SIDE_CORRECT_MAX_DEG
        else:
            side_correct_deg = -OPPOSITE_WALL_GAIN * right_risk / SIDE_CLEAR_D * SIDE_CORRECT_MAX_DEG
    else:
        # 일반 회피: 양쪽 벽 거리 차이에 따른 대칭 보정
        side_correct_deg = (left_risk - right_risk) / SIDE_CLEAR_D * SIDE_CORRECT_MAX_DEG
    side_correct_deg = clamp(side_correct_deg, -SIDE_CORRECT_MAX_DEG, SIDE_CORRECT_MAX_DEG)
    target_deg = clamp(target_deg + side_correct_deg, ANG_MIN, ANG_MAX)

    w = clamp(AVOID_TURN_SIGN * AVOID_TURN_GAIN * np.deg2rad(target_deg), -AVOID_MAX_W, AVOID_MAX_W)
    return base_v, w, target_deg, len(safe_gaps)
