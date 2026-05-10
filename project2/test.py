#!/usr/bin/env python3

import serial
import time
import math
import threading
import numpy as np


# =========================
# LiDAR 설정
# =========================
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

ANGLE_OFFSET_DEG = +1.54
DIST_OFFSET_MM = 0.0
LIDAR_ANGLE_SIGN = -1.0

MIN_LIDAR_DIST_M = 0.05
MAX_LIDAR_DIST_M = 2.5
MIN_QUALITY = 1
MIN_X_FOR_PLANNING = -0.10
MAX_EVAL_POINTS = 720


# =========================
# 포켓 감지 파라미터
# =========================
# 좌표계:
# +x = 로봇 전방
# +y = 로봇 왼쪽
# -y = 로봇 오른쪽

POCKET_ANGLE_MIN_DEG = 18.0
POCKET_ANGLE_MAX_DEG = 88.0

POCKET_LOOKAHEAD_R = 0.75

# 너무 로봇 중심 근처의 점은 일부 제외
# 단, 현재 좋은 배치에서 nearest=0.18m가 나오므로 너무 크게 잡지 않음
POCKET_MIN_R = 0.15
POCKET_MIN_X = 0.08

# 회전하려는 방향의 측면 영역만 검사
POCKET_SIDE_MIN_Y = 0.12
POCKET_SIDE_MAX_Y = 0.75

POCKET_BINS = 8

# 최종 감지 기준
POCKET_SCORE_TH = 0.80
POCKET_OCC_RATIO_TH = 0.75
POCKET_NEAR_RATIO_TH = 0.75
POCKET_MIN_OCC_BINS = 6
POCKET_MIN_POINTS = 20

# 양쪽이 동시에 감지될 때, 한쪽이 이 정도 이상 강해야 진짜 방향으로 인정
POCKET_SIDE_DOMINANCE_TH = 0.30

# near 계산 기준
# 장애물이 이 거리보다 안쪽에 많이 있으면 near가 커짐
COLLISION_DIST_REF = 0.32

SCAN_LOG_INTERVAL = 1.0
DETECT_LOG_INTERVAL = 0.20


# =========================
# RPLidar C1 드라이버
# =========================
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


# =========================
# 좌표 변환
# =========================
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

    mask_xy = x >= MIN_X_FOR_PLANNING

    if not mask_xy.any():
        return np.empty((0, 2), dtype=np.float32)

    points = np.column_stack((x[mask_xy], y[mask_xy])).astype(np.float32)

    if len(points) > MAX_EVAL_POINTS:
        order = np.argsort(points[:, 0] ** 2 + points[:, 1] ** 2)
        points = points[order[:MAX_EVAL_POINTS]]

    return points


# =========================
# 포켓 분석
# =========================
def analyze_pocket_side(points, side_name, side_sign):
    """
    side_sign:
        +1.0 -> LEFT 검사
        -1.0 -> RIGHT 검사

    오른쪽을 검사할 때는 y에 -1을 곱해서,
    검사 방향을 항상 +y 방향처럼 통일한다.
    """
    result = {
        "side": side_name,
        "score": 0.0,
        "occ_ratio": 0.0,
        "near_ratio": 0.0,
        "nearest": float("inf"),
        "occupied_bins": 0,
        "points": 0,
        "hit": False,
    }

    if len(points) == 0:
        return result

    x = points[:, 0]
    y = side_sign * points[:, 1]

    r = np.sqrt(x * x + y * y)
    ang = np.rad2deg(np.arctan2(y, x))

    mask = (
        (x > POCKET_MIN_X) &
        (r > POCKET_MIN_R) &
        (y > POCKET_SIDE_MIN_Y) &
        (y < POCKET_SIDE_MAX_Y) &
        (r < POCKET_LOOKAHEAD_R) &
        (ang > POCKET_ANGLE_MIN_DEG) &
        (ang < POCKET_ANGLE_MAX_DEG)
    )

    if not mask.any():
        return result

    a = ang[mask]
    rr = r[mask]

    result["points"] = int(len(rr))
    result["nearest"] = float(np.min(rr))

    bin_edges = np.linspace(
        POCKET_ANGLE_MIN_DEG,
        POCKET_ANGLE_MAX_DEG,
        POCKET_BINS + 1
    )

    occupied_bins = 0
    near_sum = 0.0

    for b0, b1 in zip(bin_edges[:-1], bin_edges[1:]):
        bm = (a >= b0) & (a < b1)

        if not bm.any():
            continue

        nearest_in_bin = float(np.min(rr[bm]))

        if nearest_in_bin < POCKET_LOOKAHEAD_R:
            occupied_bins += 1

            near = (POCKET_LOOKAHEAD_R - nearest_in_bin) / max(
                1e-6,
                POCKET_LOOKAHEAD_R - COLLISION_DIST_REF
            )
            near = float(np.clip(near, 0.0, 1.0))
            near_sum += near

    occ_ratio = occupied_bins / float(POCKET_BINS)

    if occupied_bins > 0:
        near_ratio = near_sum / float(occupied_bins)
    else:
        near_ratio = 0.0

    score = occ_ratio * near_ratio

    result["score"] = float(score)
    result["occ_ratio"] = float(occ_ratio)
    result["near_ratio"] = float(near_ratio)
    result["occupied_bins"] = int(occupied_bins)

    result["hit"] = (
        result["score"] >= POCKET_SCORE_TH and
        result["occ_ratio"] >= POCKET_OCC_RATIO_TH and
        result["near_ratio"] >= POCKET_NEAR_RATIO_TH and
        result["occupied_bins"] >= POCKET_MIN_OCC_BINS and
        result["points"] >= POCKET_MIN_POINTS
    )

    return result


def decide_pocket(left, right):
    """
    반환:
        "LEFT" / "RIGHT" / "AMBIGUOUS" / None
    """
    left_hit = left["hit"]
    right_hit = right["hit"]

    if left_hit and right_hit:
        diff = right["score"] - left["score"]

        if diff >= POCKET_SIDE_DOMINANCE_TH:
            return "RIGHT"

        if -diff >= POCKET_SIDE_DOMINANCE_TH:
            return "LEFT"

        return "AMBIGUOUS"

    if left_hit:
        return "LEFT"

    if right_hit:
        return "RIGHT"

    return None


def print_pocket_log(side, stat):
    print(
        f"[POCKET] {side:<5} "
        f"score={stat['score']:.2f} "
        f"occ={stat['occ_ratio']:.2f} "
        f"near={stat['near_ratio']:.2f} "
        f"nearest={stat['nearest']:.2f}m "
        f"bins={stat['occupied_bins']}/{POCKET_BINS} "
        f"pts={stat['points']}"
    )


def print_scan_log(points, left, right, decision):
    print(
        f"[SCAN] pts={len(points)} decision={decision} | "
        f"L(score={left['score']:.2f}, occ={left['occ_ratio']:.2f}, "
        f"near={left['near_ratio']:.2f}, bins={left['occupied_bins']}/{POCKET_BINS}, "
        f"pts={left['points']}) | "
        f"R(score={right['score']:.2f}, occ={right['occ_ratio']:.2f}, "
        f"near={right['near_ratio']:.2f}, bins={right['occupied_bins']}/{POCKET_BINS}, "
        f"pts={right['points']})"
    )


# =========================
# main
# =========================
def main():
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)

    print("[INFO] LiDAR pocket detector started.")
    print("[INFO] Move the robot by hand and check [POCKET] logs.")
    print("[INFO] Press Ctrl+C to stop.")

    last_scan_log = 0.0
    last_detect_log = 0.0

    try:
        while True:
            scan = lidar.get_scan()

            if scan is None:
                time.sleep(0.05)
                continue

            points = lidar_points_to_xy(scan)

            left = analyze_pocket_side(points, "LEFT", +1.0)
            right = analyze_pocket_side(points, "RIGHT", -1.0)

            decision = decide_pocket(left, right)

            now = time.time()

            if decision == "LEFT":
                if now - last_detect_log >= DETECT_LOG_INTERVAL:
                    print_pocket_log("LEFT", left)
                    last_detect_log = now

            elif decision == "RIGHT":
                if now - last_detect_log >= DETECT_LOG_INTERVAL:
                    print_pocket_log("RIGHT", right)
                    last_detect_log = now

            elif decision == "AMBIGUOUS":
                if now - last_detect_log >= DETECT_LOG_INTERVAL:
                    print(
                        f"[AMBIGUOUS] both sides blocked | "
                        f"L={left['score']:.2f}, R={right['score']:.2f}, "
                        f"diff={abs(left['score'] - right['score']):.2f}"
                    )
                    last_detect_log = now

            if now - last_scan_log >= SCAN_LOG_INTERVAL:
                print_scan_log(points, left, right, decision)
                last_scan_log = now

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")

    finally:
        lidar.close()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()