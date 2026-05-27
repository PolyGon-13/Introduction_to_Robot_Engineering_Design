#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

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
MIN_Q = 1
MIN_D = 0.05
MAX_D = 2.5

ANG_MIN = -90.0
ANG_MAX = 90.0
ANG_STEP = 1.0
FREE_D = 0.30
MIN_GAP_DEG = 8.0

BASE_V = 0.18
MAX_W = 0.90
TURN_GAIN = 1.05
V_STEP = 0.04
W_STEP = 0.20
LOOP_DT = 0.05

GRID = np.arange(ANG_MIN, ANG_MAX + 0.5 * ANG_STEP, ANG_STEP, dtype=np.float32)


def clamp(value, low, high):
    return float(max(low, min(value, high)))


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

    for idx, dist in zip(bins, dist_m[valid]):
        if 0 <= idx < len(ranges) and dist < ranges[idx]:
            ranges[idx] = dist

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


def avoid_cmd(scan):
    ranges = front_ranges(scan)

    # FREE_D 이상인 구간을 통과 가능한 공간으로 판단
    free = ranges >= FREE_D
    gaps = find_gaps(free)

    if not gaps:
        return 0.0, 0.0, 0.0, 0

    # 각 갭의 중앙 각도 계산
    centers = np.array(
        [0.5 * (GRID[start] + GRID[end - 1]) for start, end in gaps],
        dtype=np.float32
    )

    # ==============================
    # 갭 선택 방식 수정
    # 기존: 정면에 가장 가까운 갭 선택
    # 수정: 갭이 2개 이상이면 각 갭 내부 라이다 거리 평균을 구해서
    #       평균 거리가 가장 큰 갭 선택
    # ==============================
    if len(gaps) >= 2:
        gap_avgs = []
        gap_widths = []

        for start, end in gaps:
            gap_range_values = ranges[start:end]

            # 해당 갭 내부의 라이다 거리 평균
            avg_dist = float(np.mean(gap_range_values))

            # 해당 갭의 폭
            width = end - start

            gap_avgs.append(avg_dist)
            gap_widths.append(width)

        # 선택 우선순위
        # 1순위: 평균 거리값이 큰 갭
        # 2순위: 갭 폭이 넓은 갭
        # 3순위: 정면에 가까운 갭
        best = max(
            range(len(gaps)),
            key=lambda i: (
                gap_avgs[i],
                gap_widths[i],
                -abs(centers[i])
            )
        )

    else:
        # 갭이 1개면 그 갭 선택
        best = 0

    target_deg = float(centers[best])

    # 목표 각도에 따라 회전 속도 계산
    w = clamp(TURN_GAIN * np.deg2rad(target_deg), -MAX_W, MAX_W)

    return BASE_V, w, target_deg, len(gaps)


def main():
    lidar = RPLidarC1()
    motor = Motor()

    v = 0.0
    w = 0.0
    last_seq = -1

    try:
        motor.stop()

        while True:
            scan, scan_time, scan_seq = lidar.get()

            if scan is None:
                target_v = 0.0
                target_w = 0.0
                target_deg = 0.0
                gap_count = 0
                print("[LIDAR] waiting...")

            elif scan_seq == last_seq:
                time.sleep(LOOP_DT)
                continue

            else:
                target_v, target_w, target_deg, gap_count = avoid_cmd(scan)
                last_seq = scan_seq

                print(
                    f"gap={gap_count} "
                    f"target={target_deg:.0f} "
                    f"v={target_v:.2f} "
                    f"w={target_w:.2f}"
                )

            v = rate(v, target_v, V_STEP)
            w = rate(w, target_w, W_STEP)

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
