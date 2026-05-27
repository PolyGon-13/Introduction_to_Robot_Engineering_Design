#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time
from collections import deque

import numpy as np
import serial


LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600

RESET = b"\xA5\x40"
SCAN = b"\xA5\x20"
LIDAR_STOP = b"\xA5\x25"

ANGLE_OFFSET = 1.54
DIST_OFFSET = 0.0
ANGLE_SIGN = -1.0

MIN_D = 0.05
MAX_D = 2.5
MIN_Q = 1
SCAN_HOLD = 0.30

ANG_MIN = -90.0
ANG_MAX = 90.0
ANG_STEP = 1.0
FREE_D = 0.25
MIN_GAP_DEG = 8.0

BASE_V = 0.18
MAX_W = 0.70
TURN_GAIN = 1.05
V_STEP = 0.04
W_STEP = 0.30
LOOP_DT = 0.05

GRID = np.arange(ANG_MIN, ANG_MAX + 0.5 * ANG_STEP, ANG_STEP, dtype=np.float32)


def clamp(x, lo, hi):
    return float(max(lo, min(x, hi)))


def rate(prev, target, step):
    return prev + clamp(target - prev, -step, step)


def norm_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


class Motor:
    def __init__(self):
        self.ser = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
        time.sleep(2.0)

    def vw(self, v, w):
        self.ser.write(f"V{v:.3f},{w:.3f}\n".encode("ascii"))

    def stop(self):
        self.ser.write(b"S\n")

    def close(self):
        if self.ser.is_open:
            self.ser.close()


class RPLidarC1:
    def __init__(self):
        self.ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
        self.q = deque(maxlen=3)
        self.lock = threading.Lock()
        self.running = True

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
                p = self.ser.read(5)
                if len(p) != 5:
                    continue

                start = p[0] & 1
                if ((p[0] & 2) >> 1) != (1 - start) or (p[1] & 1) != 1:
                    continue

                quality = p[0] >> 2
                angle = ((p[1] >> 1) | (p[2] << 7)) / 64.0
                dist = (p[3] | (p[4] << 8)) / 4.0

                if start and len(angles) > 50:
                    scan = (
                        np.array(angles, dtype=np.float32),
                        np.array(dists, dtype=np.float32),
                        np.array(qualities, dtype=np.float32),
                    )
                    with self.lock:
                        self.q.append((scan, time.time()))
                    angles, dists, qualities = [], [], []

                if dist > 0 and quality >= MIN_Q:
                    angles.append(angle)
                    dists.append(dist)
                    qualities.append(quality)
            except (serial.SerialException, OSError):
                time.sleep(1.0)

    def get(self):
        with self.lock:
            return self.q.pop() if self.q else (None, 0.0)

    def close(self):
        self.running = False
        try:
            self.ser.write(LIDAR_STOP)
        except Exception:
            pass
        self.thread.join(timeout=0.5)
        if self.ser.is_open:
            self.ser.close()


def ranges(scan):
    out = np.full(len(GRID), MAX_D, dtype=np.float32)
    if scan is None:
        return out

    angles, dists, qualities = scan
    dist_m = (dists + DIST_OFFSET) / 1000.0
    angle_deg = norm_deg(angles + ANGLE_OFFSET) * ANGLE_SIGN
    valid = (dist_m >= MIN_D) & (dist_m <= MAX_D) & (qualities >= MIN_Q)
    bins = np.rint((angle_deg[valid] - ANG_MIN) / ANG_STEP).astype(np.int32)

    for idx, dist in zip(bins, dist_m[valid]):
        if 0 <= idx < len(out) and dist < out[idx]:
            out[idx] = dist
    return out


def find_gaps(free):
    found, start = [], None
    for i, ok in enumerate(free):
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            found.append((start, i))
            start = None
    if start is not None:
        found.append((start, len(free)))

    min_bins = max(1, math.ceil(MIN_GAP_DEG / ANG_STEP))
    return [(s, e) for s, e in found if e - s >= min_bins]


def choose_cmd(scan):
    r = ranges(scan)
    gaps = find_gaps(r >= FREE_D)
    if not gaps:
        return 0.0, MAX_W, 0.0, 0

    centers = np.array([0.5 * (GRID[s] + GRID[e - 1]) for s, e in gaps])
    idx = int(np.argmin(np.abs(centers)))
    target_deg = float(centers[idx])
    w = clamp(TURN_GAIN * math.radians(target_deg), -MAX_W, MAX_W)
    return BASE_V, w, target_deg, len(gaps)


def main():
    lidar = RPLidarC1()
    motor = Motor()
    v = w = 0.0
    try:
        motor.stop()
        while True:
            scan, t = lidar.get()
            if scan is None or time.time() - t > SCAN_HOLD:
                tv, tw, deg, count = 0.0, 0.0, 0.0, 0
                print("[LIDAR] waiting...")
            else:
                tv, tw, deg, count = choose_cmd(scan)
                print(f"gap={count} target={deg:.0f} v={tv:.2f} w={tw:.2f}")

            v = rate(v, tv, V_STEP)
            w = rate(w, tw, W_STEP)
            motor.vw(v, w)
            time.sleep(LOOP_DT)
    except KeyboardInterrupt:
        print("\n[INFO] stop")
    finally:
        motor.stop()
        time.sleep(0.2)
        motor.close()
        lidar.close()


if __name__ == "__main__":
    main()
