#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time
from collections import deque

import numpy as np
import serial

from motor_control import open_arduino, rate_limit, send_vw, stop


LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
LIDAR_TIMEOUT = 0.1
MOTOR_STOP_CMD = bytes([0xA5, 0x25])
MOTOR_RESET_CMD = bytes([0xA5, 0x40])
MOTOR_SCAN_CMD = bytes([0xA5, 0x20])

ANGLE_OFFSET_DEG = 1.54
DIST_OFFSET_MM = 0.0
LIDAR_ANGLE_SIGN = -1.0

MIN_DIST_M = 0.05
MAX_DIST_M = 0.75
MIN_QUALITY = 1
SCAN_HOLD_S = 0.30
SCAN_QUEUE_SIZE = 5

FRONT_X_MIN = 0.05
FRONT_X_MAX = 0.35
FRONT_Y_HALF = 0.18
SIDE_X_MIN = -0.03
SIDE_X_MAX = 0.25
SIDE_Y_MIN = 0.05
SIDE_Y_MAX = 0.35

GAP_MIN_ANGLE_DEG = -90.0
GAP_MAX_ANGLE_DEG = 90.0
GAP_STEP_DEG = 2.0
GAP_FREE_DIST = 0.20
GAP_MIN_WIDTH_DEG = 10.0

BASE_V = 0.16
SLOW_V = 0.09
MAX_W = 0.70
KP_TURN = 1.0
V_STEP = 0.04
W_STEP = 0.15
LOOP_DT = 0.05
ANGLE_GRID = np.arange(GAP_MIN_ANGLE_DEG, GAP_MAX_ANGLE_DEG + GAP_STEP_DEG, GAP_STEP_DEG)


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


class RPLidarC1:
    def __init__(self, port=LIDAR_PORT, baud=LIDAR_BAUD):
        self.ser = serial.Serial(port, baud, timeout=LIDAR_TIMEOUT)
        self.lock = threading.Lock()
        self.scan_queue = deque(maxlen=SCAN_QUEUE_SIZE)
        self.running = True

        self.ser.write(MOTOR_RESET_CMD)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(MOTOR_SCAN_CMD)

        header = self.ser.read(7)
        if len(header) != 7 or header[:2] != bytes([0xA5, 0x5A]):
            self.ser.close()
            raise RuntimeError("[LIDAR] Response header error")

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(f"[INFO] LiDAR connected: {port}, {baud}")

    def _loop(self):
        angles, dists, qualities = [], [], []
        while self.running:
            try:
                packet = self.ser.read(5)
                if len(packet) != 5:
                    continue

                start = packet[0] & 0x01
                if ((packet[0] & 0x02) >> 1) != (1 - start) or (packet[1] & 0x01) != 1:
                    continue

                quality = packet[0] >> 2
                angle = ((packet[1] >> 1) | (packet[2] << 7)) / 64.0
                dist = (packet[3] | (packet[4] << 8)) / 4.0

                if start and len(angles) >= 50:
                    self._save_scan(angles, dists, qualities)
                    angles, dists, qualities = [], [], []

                if dist > 0 and quality >= MIN_QUALITY:
                    angles.append(angle)
                    dists.append(dist)
                    qualities.append(quality)

            except (serial.SerialException, OSError) as e:
                print(f"[LIDAR] Serial error: {e}")
                time.sleep(1.0)

    def _save_scan(self, angles, dists, qualities):
        scan = (
            np.array(angles, dtype=np.float32),
            np.array(dists, dtype=np.float32),
            np.array(qualities, dtype=np.float32),
        )
        with self.lock:
            self.scan_queue.append((scan, time.time()))

    def get_scan(self):
        with self.lock:
            if not self.scan_queue:
                return None, 0.0
            return self.scan_queue.pop()

    def close(self):
        self.running = False
        try:
            self.ser.write(MOTOR_STOP_CMD)
        except Exception:
            pass
        self.thread.join(timeout=0.5)
        time.sleep(0.1)
        if self.ser and self.ser.is_open:
            self.ser.close()


def scan_to_xy(scan):
    dist_m, angle_deg = valid_polar(scan)
    if len(dist_m) == 0:
        return np.empty((0, 2), dtype=np.float32)

    angle_rad = np.deg2rad(angle_deg)
    return np.column_stack((dist_m * np.cos(angle_rad), dist_m * np.sin(angle_rad))).astype(np.float32)


def valid_polar(scan):
    if scan is None:
        return np.empty(0, dtype=np.float32), np.empty(0, dtype=np.float32)

    angles, dists, qualities = scan
    dist_m = (dists + DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles + ANGLE_OFFSET_DEG) * LIDAR_ANGLE_SIGN
    valid = (dist_m >= MIN_DIST_M) & (dist_m <= MAX_DIST_M) & (qualities >= MIN_QUALITY)
    return dist_m[valid], angle_deg[valid]


def min_dist(points, mask):
    if not mask.any():
        return MAX_DIST_M
    return float(np.min(np.hypot(points[mask, 0], points[mask, 1])))


def detect_obstacles(points):
    if len(points) == 0:
        return MAX_DIST_M, MAX_DIST_M, MAX_DIST_M

    x = points[:, 0]
    y = points[:, 1]
    abs_y = np.abs(y)
    front = (
        (x > FRONT_X_MIN)
        & (x < FRONT_X_MAX)
        & (abs_y < FRONT_Y_HALF)
    )
    side = (
        (x > SIDE_X_MIN)
        & (x < SIDE_X_MAX)
        & (abs_y > SIDE_Y_MIN)
        & (abs_y < SIDE_Y_MAX)
    )
    left = side & (y > 0)
    right = side & (y < 0)
    return min_dist(points, front), min_dist(points, left), min_dist(points, right)


def scan_to_ranges(scan):
    ranges = np.full(len(ANGLE_GRID), MAX_DIST_M, dtype=np.float32)
    dist_m, angle_deg = valid_polar(scan)
    bins = np.rint((angle_deg - GAP_MIN_ANGLE_DEG) / GAP_STEP_DEG).astype(np.int32)

    for idx, dist in zip(bins, dist_m):
        if 0 <= idx < len(ranges) and dist < ranges[idx]:
            ranges[idx] = dist
    return ANGLE_GRID, ranges


def find_gaps(free):
    gaps, start = [], None
    for i, ok in enumerate(free):
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            gaps.append((start, i))
            start = None
    if start is not None:
        gaps.append((start, len(free)))

    min_bins = max(1, int(math.ceil(GAP_MIN_WIDTH_DEG / GAP_STEP_DEG)))
    return [(s, e) for s, e in gaps if e - s >= min_bins]


def choose_gap_cmd(scan):
    angles, ranges = scan_to_ranges(scan)
    gaps = find_gaps(ranges >= GAP_FREE_DIST)
    if not gaps:
        return 0.0, MAX_W, 0.0, 0

    centers = [0.5 * (angles[start] + angles[end - 1]) for start, end in gaps]
    best_idx = int(np.argmin(np.abs(centers)))
    best = gaps[best_idx]
    target_deg = float(centers[best_idx])
    target_rad = math.radians(target_deg)
    target_dist = float(np.max(ranges[best[0]:best[1]]))
    v = SLOW_V if target_dist < GAP_FREE_DIST + 0.10 else BASE_V
    w = float(np.clip(KP_TURN * target_rad, -MAX_W, MAX_W))
    return v, w, target_deg, len(gaps)


def read_lidar_cmd(lidar):
    scan, scan_time = lidar.get_scan()
    if scan is None or time.time() - scan_time > SCAN_HOLD_S:
        print("[LIDAR] waiting for scan...")
        return 0.0, 0.0

    points = scan_to_xy(scan)
    front, left, right = detect_obstacles(points)
    v, w, target_deg, gap_count = choose_gap_cmd(scan)
    print(
        f"points={len(points)} "
        f"front={front:.2f}m left={left:.2f}m right={right:.2f}m "
        f"gap={gap_count} target={target_deg:.0f}deg v={v:.2f} w={w:.2f}"
    )
    return v, w


def main():
    lidar = RPLidarC1()
    motor = open_arduino()
    last_v = last_w = 0.0
    try:
        stop(motor)
        while True:
            v, w = read_lidar_cmd(lidar)
            last_v = rate_limit(last_v, v, V_STEP)
            last_w = rate_limit(last_w, w, W_STEP)
            send_vw(motor, last_v, last_w)
            time.sleep(LOOP_DT)
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt")
    finally:
        stop(motor)
        motor.close()
        lidar.close()
        print("[INFO] LiDAR closed")


if __name__ == "__main__":
    main()
