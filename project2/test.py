#!/usr/bin/env python3

import math
import time
import threading

import numpy as np
import serial

# =========================
# User settings
# =========================
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

# 라이다 캘리브레이션
ANGLE_OFFSET_DEG = +1.54
DIST_OFFSET_MM = 0.0
LIDAR_ANGLE_SIGN = -1.0

# 라이다 전처리
MIN_LIDAR_DIST_M = 0.05
MAX_LIDAR_DIST_M = 2.5
MIN_QUALITY = 1
MIN_X_FOR_PLANNING = -0.10
MAX_EVAL_POINTS = 720
LOOP_DT_S = 0.05

# 로봇 크기 기준
ROBOT_RADIUS = 0.16
SAFETY_MARGIN = 0.16
COLLISION_DIST = ROBOT_RADIUS + SAFETY_MARGIN

# 뾰족한 공간/막힌 포켓 판별 파라미터
POCKET_ANGLE_MIN_DEG = 18.0
POCKET_ANGLE_MAX_DEG = 88.0
POCKET_LOOKAHEAD_R = 0.75
POCKET_SIDE_MIN_Y = 0.10
POCKET_SIDE_MAX_Y = 0.75
POCKET_BINS = 8
POCKET_OCC_RATIO_TH = 0.30
POCKET_NEAR_RATIO_TH = 0.25
POCKET_SCORE_TH = 0.12

# 로그 설정
DETECT_LOG_INTERVAL_S = 0.30
DEBUG_LOG_INTERVAL_S = 1.00


class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.ser.write(bytes([0xA5, 0x40]))  # RESET
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        self.ser.write(bytes([0xA5, 0x20]))  # SCAN
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
            self.ser.write(bytes([0xA5, 0x25]))  # STOP
        except Exception:
            pass
        time.sleep(0.1)
        self.ser.close()


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


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


def analyze_pocket_side(points, side_sign):
    """
    side_sign = +1: 왼쪽(+y) 검사
    side_sign = -1: 오른쪽(-y) 검사
    """
    if len(points) == 0:
        return {
            "detected": False,
            "score": 0.0,
            "occ_ratio": 0.0,
            "near_ratio": 0.0,
            "nearest": MAX_LIDAR_DIST_M,
            "occupied_bins": 0,
            "points": 0,
        }

    x = points[:, 0]
    y = side_sign * points[:, 1]
    r = np.sqrt(x * x + y * y)
    ang = np.rad2deg(np.arctan2(y, x))

    mask = (
        (x > 0.02)
        & (y > POCKET_SIDE_MIN_Y)
        & (y < POCKET_SIDE_MAX_Y)
        & (r < POCKET_LOOKAHEAD_R)
        & (ang > POCKET_ANGLE_MIN_DEG)
        & (ang < POCKET_ANGLE_MAX_DEG)
    )

    if not mask.any():
        return {
            "detected": False,
            "score": 0.0,
            "occ_ratio": 0.0,
            "near_ratio": 0.0,
            "nearest": MAX_LIDAR_DIST_M,
            "occupied_bins": 0,
            "points": 0,
        }

    pocket_angles = ang[mask]
    pocket_ranges = r[mask]
    bin_edges = np.linspace(POCKET_ANGLE_MIN_DEG, POCKET_ANGLE_MAX_DEG, POCKET_BINS + 1)

    occupied_bins = 0
    near_sum = 0.0
    nearest_all = float(np.min(pocket_ranges))

    for b0, b1 in zip(bin_edges[:-1], bin_edges[1:]):
        bm = (pocket_angles >= b0) & (pocket_angles < b1)
        if not bm.any():
            continue

        nearest = float(np.min(pocket_ranges[bm]))
        occupied_bins += 1
        near_sum += float(
            np.clip(
                (POCKET_LOOKAHEAD_R - nearest)
                / max(1e-6, POCKET_LOOKAHEAD_R - COLLISION_DIST),
                0.0,
                1.0,
            )
        )

    occ_ratio = occupied_bins / float(POCKET_BINS)
    near_ratio = near_sum / max(1, occupied_bins)
    score = occ_ratio * near_ratio

    detected = (
        occ_ratio >= POCKET_OCC_RATIO_TH
        and near_ratio >= POCKET_NEAR_RATIO_TH
        and score >= POCKET_SCORE_TH
    )

    return {
        "detected": detected,
        "score": score,
        "occ_ratio": occ_ratio,
        "near_ratio": near_ratio,
        "nearest": nearest_all,
        "occupied_bins": occupied_bins,
        "points": int(np.count_nonzero(mask)),
    }


def main():
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    print("[INFO] Pocket detector started.")
    print("[INFO] Move the robot by hand. Press Ctrl+C to stop.")
    print("[INFO] Coordinate: left=+y, right=-y")

    last_detect_log = 0.0
    last_debug_log = 0.0

    try:
        while True:
            scan = lidar.get_scan()
            points = lidar_points_to_xy(scan)

            left = analyze_pocket_side(points, side_sign=+1)
            right = analyze_pocket_side(points, side_sign=-1)

            now = time.time()

            if (left["detected"] or right["detected"]) and now - last_detect_log > DETECT_LOG_INTERVAL_S:
                if left["detected"]:
                    print(
                        "[POCKET] LEFT  "
                        f"score={left['score']:.2f} "
                        f"occ={left['occ_ratio']:.2f} "
                        f"near={left['near_ratio']:.2f} "
                        f"nearest={left['nearest']:.2f}m "
                        f"bins={left['occupied_bins']}/{POCKET_BINS} "
                        f"pts={left['points']}"
                    )
                if right["detected"]:
                    print(
                        "[POCKET] RIGHT "
                        f"score={right['score']:.2f} "
                        f"occ={right['occ_ratio']:.2f} "
                        f"near={right['near_ratio']:.2f} "
                        f"nearest={right['nearest']:.2f}m "
                        f"bins={right['occupied_bins']}/{POCKET_BINS} "
                        f"pts={right['points']}"
                    )
                last_detect_log = now

            if now - last_debug_log > DEBUG_LOG_INTERVAL_S:
                print(
                    "[SCAN] "
                    f"pts={len(points)} | "
                    f"L(score={left['score']:.2f}, occ={left['occ_ratio']:.2f}, near={left['near_ratio']:.2f}) | "
                    f"R(score={right['score']:.2f}, occ={right['occ_ratio']:.2f}, near={right['near_ratio']:.2f})"
                )
                last_debug_log = now

            time.sleep(LOOP_DT_S)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping pocket detector...")
    finally:
        lidar.close()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
