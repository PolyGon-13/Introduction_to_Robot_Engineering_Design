#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import cv2
import numpy as np
import serial


try:
    from picamera2 import Picamera2
    USE_PICAM = True
except ImportError:
    USE_PICAM = False


# 따라갈 색상: "RED", "BLUE", "YELLOW" 중 하나로 변경, None이면 모든 색 중 가장 큰 물체 추적
TARGET = "RED"
SHOW_WINDOW = True
MIN_AREA = 200
MAX_V, MAX_W, KP = 0.18, 0.70, 0.85
V_STEP, W_STEP, DEADBAND = 0.04, 0.15, 0.08
ARDU_PORT, ARDU_BAUD = "/dev/ttyS0", 9600

HSV_RANGES = {
    "RED": [([0, 90, 120], [10, 255, 255]), ([170, 90, 120], [179, 255, 255])],
    "BLUE": [([102, 85, 110], [126, 255, 255])],
    "YELLOW": [([22, 85, 140], [36, 255, 255])],
}
BOX_COLORS = {"RED": (0, 0, 255), "BLUE": (255, 0, 0), "YELLOW": (0, 255, 255)}


# ==============================
# LIDAR 장애물 회피 코드
# ==============================

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

RESET = b"\xA5\x40"
SCAN = b"\xA5\x20"
LIDAR_STOP = b"\xA5\x25"

ANGLE_OFFSET = 1.54
DIST_OFFSET = 0.0
ANGLE_SIGN = -1.0
MIN_Q = 1
MIN_D = 0.05
MAX_D = 2.5

ANG_MIN = -90.0
ANG_MAX = 90.0
ANG_STEP = 1.0

FREE_D = 0.35
MIN_GAP_DEG = 8.0

BASE_V = 0.18
AVOID_MAX_W = 0.90
TURN_GAIN = 1.05

LOOP_DT = 0.05
GRID = np.arange(ANG_MIN, ANG_MAX + 0.5 * ANG_STEP, ANG_STEP, dtype=np.float32)

# 카메라에서 색이 화면 좌우 끝에 있을 때 라이다 기준으로 몇 도까지 보정할지
COLOR_TO_LIDAR_DEG = 45.0

# 전방 이 각도 안에 FREE_D보다 가까운 장애물이 있으면 회피 모드
OBSTACLE_FRONT_DEG = 45.0


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

    min_bins = max(1, int(np.ceil(MIN_GAP_DEG / ANG_STEP)))

    return [(start, end) for start, end in gaps if end - start >= min_bins]


def obstacle_detected(scan):
    ranges = front_ranges(scan)

    front_zone = (GRID >= -OBSTACLE_FRONT_DEG) & (GRID <= OBSTACLE_FRONT_DEG)
    front_min = float(np.min(ranges[front_zone]))

    return front_min < FREE_D


def color_angle_from_target(target, width):
    if target is None:
        return 0.0

    _, x, _, w, _, _ = target

    err = (x + w / 2 - width / 2) / (width / 2)
    err = clamp(err, -1.0, 1.0)

    return clamp(-err * COLOR_TO_LIDAR_DEG, ANG_MIN, ANG_MAX)


def avoid_cmd(scan, color_deg=0.0):
    ranges = front_ranges(scan)
    gaps = find_gaps(ranges >= FREE_D)

    if not gaps:
        return 0.0, 0.0, 0.0, 0

    def gap_key(gap):
        start, end = gap
        center = 0.5 * (GRID[start] + GRID[end - 1])
        color_dist = abs(norm_deg(center - color_deg))

        return -color_dist, float(np.mean(ranges[start:end])), end - start

    start, end = max(gaps, key=gap_key)

    target_deg = clamp(color_deg, GRID[start], GRID[end - 1])
    w = clamp(TURN_GAIN * np.deg2rad(target_deg), -AVOID_MAX_W, AVOID_MAX_W)

    return BASE_V, w, target_deg, len(gaps)


# ==============================
# 색 추적 코드
# ==============================

def open_camera():
    if USE_PICAM:
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
        cam.start()
        return cam

    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    return cam


def read_frame(cam):
    if USE_PICAM:
        return True, cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)

    return cam.read()


def detect(frame):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    found = []

    for name, ranges in HSV_RANGES.items():
        mask = None

        for lower, upper in ranges:
            part = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = part if mask is None else cv2.bitwise_or(mask, part)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area >= MIN_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                found.append((name, x, y, w, h, int(area)))

    return found


def pick(found):
    targets = found if TARGET is None else [item for item in found if item[0] == TARGET]

    return max(targets, key=lambda item: item[5], default=None)


def follow_cmd(target, width):
    if target is None:
        return 0.0, 0.0

    _, x, _, w, _, _ = target

    err = (x + w / 2 - width / 2) / (width / 2)
    err = 0.0 if abs(err) < DEADBAND else err

    v = MAX_V * (1.0 - 0.45 * min(1.0, abs(err)))
    w = max(-MAX_W, min(-KP * err, MAX_W))

    return v, w


def rate_limit(prev, target, step):
    return prev + max(-step, min(target - prev, step))


def send_vw(motor, v, w):
    motor.write(f"V{v:.3f},{w:.3f}\n".encode("ascii"))


def stop(motor):
    motor.write(b"S\n")


def draw(frame, found):
    for name, x, y, w, h, area in found:
        color = BOX_COLORS[name]

        cx, cy = x + w // 2, y + h // 2

        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.circle(frame, (cx, cy), 5, color, -1)

        cv2.putText(
            frame,
            f"{name} {cx},{cy} {area}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )


def main():
    cam = None
    lidar = None
    motor = None

    cam = open_camera()
    lidar = RPLidarC1()
    motor = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)

    time.sleep(2.0)

    last_v = 0.0
    last_w = 0.0

    try:
        stop(motor)

        while True:
            ok, frame = read_frame(cam)

            if not ok:
                break

            found = detect(frame)
            target = pick(found)

            color_v, color_w = follow_cmd(target, frame.shape[1])

            scan, scan_time, scan_seq = lidar.get()
            obstacle = scan is not None and obstacle_detected(scan)

            if obstacle:
                color_deg = color_angle_from_target(target, frame.shape[1])
                v, w, target_deg, gap_count = avoid_cmd(scan, color_deg)

                print(
                    f"[AVOID] gap={gap_count} "
                    f"color_deg={color_deg:.0f} "
                    f"target={target_deg:.0f} "
                    f"v={v:.2f} "
                    f"w={w:.2f}"
                )

            elif target is None:
                v = 0.0
                w = 0.0

            else:
                v = color_v
                w = color_w

            last_v = rate_limit(last_v, v, V_STEP)
            last_w = rate_limit(last_w, w, W_STEP)

            send_vw(motor, last_v, last_w)

            if SHOW_WINDOW:
                draw(frame, found)
                cv2.imshow("follow", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(LOOP_DT)

    finally:
        if motor is not None:
            stop(motor)
            motor.close()

        if lidar is not None:
            lidar.close()

        if cam is not None:
            cam.stop() if USE_PICAM else cam.release()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
