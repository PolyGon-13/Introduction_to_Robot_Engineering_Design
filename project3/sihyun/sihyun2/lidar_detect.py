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
MAX_D = 0.75
MIN_Q = 1
SCAN_HOLD = 0.30

GAP_MIN = -90.0
GAP_MAX = 90.0
GAP_STEP = 2.0
GAP_FREE = 0.20
GAP_WIDTH = 10.0

BASE_V = 0.16
SLOW_V = 0.09
MAX_W = 0.70
KP = 1.0

V_STEP = 0.04
W_STEP = 0.15
LOOP_DT = 0.05
GRID = np.arange(GAP_MIN, GAP_MAX + GAP_STEP, GAP_STEP)


def clamp(x, lo, hi):
    return max(lo, min(x, hi))


def rate(prev, target, step):
    return prev + clamp(target - prev, -step, step)


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
        self.q = deque(maxlen=5)
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
        a, d, q = [], [], []
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

                if start and len(a) >= 50:
                    scan = (
                        np.array(a, dtype=np.float32),
                        np.array(d, dtype=np.float32),
                        np.array(q, dtype=np.float32),
                    )
                    with self.lock:
                        self.q.append((scan, time.time()))
                    a, d, q = [], [], []

                if dist > 0 and quality >= MIN_Q:
                    a.append(angle)
                    d.append(dist)
                    q.append(quality)
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


def polar(scan):
    if scan is None:
        return np.empty(0), np.empty(0)
    ang, dist, qual = scan
    dist = (dist + DIST_OFFSET) / 1000.0
    ang = ((ang + ANGLE_OFFSET + 180.0) % 360.0 - 180.0) * ANGLE_SIGN
    ok = (dist >= MIN_D) & (dist <= MAX_D) & (qual >= MIN_Q)
    return dist[ok], ang[ok]


def ranges(scan):
    r = np.full(len(GRID), MAX_D, dtype=np.float32)
    dist, ang = polar(scan)
    bins = np.rint((ang - GAP_MIN) / GAP_STEP).astype(np.int32)
    for i, d in zip(bins, dist):
        if 0 <= i < len(r):
            r[i] = min(r[i], d)
    return r


def gaps(free):
    out, start = [], None
    for i, ok in enumerate(free):
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(free)))
    need = max(1, math.ceil(GAP_WIDTH / GAP_STEP))
    return [(s, e) for s, e in out if e - s >= need]


def cmd(scan):
    r = ranges(scan)
    gs = gaps(r >= GAP_FREE)
    if not gs:
        return 0.0, MAX_W, 0.0, 0
    centers = np.array([0.5 * (GRID[s] + GRID[e - 1]) for s, e in gs])
    idx = int(np.argmin(np.abs(centers)))
    s, e = gs[idx]
    deg = float(centers[idx])
    v = SLOW_V if float(np.max(r[s:e])) < GAP_FREE + 0.10 else BASE_V
    w = clamp(KP * math.radians(deg), -MAX_W, MAX_W)
    return v, w, deg, len(gs)


def main():
    lidar, motor = RPLidarC1(), Motor()
    v = w = 0.0
    try:
        motor.stop()
        while True:
            scan, t = lidar.get()
            tv = tw = 0.0
            if scan is not None and time.time() - t <= SCAN_HOLD:
                tv, tw, deg, n = cmd(scan)
                print(f"gap={n} target={deg:.0f} v={tv:.2f} w={tw:.2f}")
            else:
                print("[LIDAR] waiting...")

            v, w = rate(v, tv, V_STEP), rate(w, tw, W_STEP)
            motor.vw(v, w)
            time.sleep(LOOP_DT)
    except KeyboardInterrupt:
        print("\n[INFO] stop")
    finally:
        motor.stop()
        motor.close()
        lidar.close()


if __name__ == "__main__":
    main()
