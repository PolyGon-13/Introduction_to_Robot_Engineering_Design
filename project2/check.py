#!/usr/bin/env python3
# RPLiDAR C1 angle/sign checker

import argparse
import math
import threading
import time

import numpy as np
import serial


LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
ANGLE_OFFSET_DEG = +1.54
DIST_OFFSET_MM = 0.0
FOV_HALF_DEG = 90.0
DISP_CLAMP = 3.0


class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.ser.write(bytes([0xA5, 0x40]))
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0xA5, 0x20]))
        self.ser.read(7)
        self.lock = threading.Lock()
        self.latest_scan = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buf_a, buf_d, buf_q = [], [], []
        while self.running:
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


def build_front_arrays(scan, angle_offset_deg, dist_offset_mm, flip_angle):
    if scan is None:
        return None
    a, d, q = scan
    if flip_angle:
        a = -(a + angle_offset_deg)
    else:
        a = a + angle_offset_deg
    d = (d + dist_offset_mm) / 1000.0
    a = ((a + 180.0) % 360.0) - 180.0
    mask = (np.abs(a) <= FOV_HALF_DEG) & (d > 0.03) & (d < DISP_CLAMP)
    if int(mask.sum()) == 0:
        return None
    theta = np.deg2rad(a[mask])
    dist = d[mask]
    quality = q[mask]
    x = dist * np.cos(theta)
    y = dist * np.sin(theta)
    return a[mask], theta, dist, quality, x, y


def min_sector(a_deg, dist, lo, hi):
    mask = (a_deg >= lo) & (a_deg <= hi)
    if not mask.any():
        return DISP_CLAMP
    return float(np.min(dist[mask]))


def nearest_point(a_deg, dist, x, y, lo=-90.0, hi=90.0):
    mask = (a_deg >= lo) & (a_deg <= hi)
    if not mask.any():
        return None
    idxs = np.where(mask)[0]
    i = idxs[int(np.argmin(dist[mask]))]
    return float(a_deg[i]), float(dist[i]), float(x[i]), float(y[i])


def print_scan_summary(front_data):
    a_deg, _theta, dist, _quality, x, y = front_data
    front10 = min_sector(a_deg, dist, -10.0, 10.0)
    front24 = min_sector(a_deg, dist, -24.0, 24.0)
    left_front = min_sector(a_deg, dist, -60.0, -15.0)
    right_front = min_sector(a_deg, dist, 15.0, 60.0)
    left_side = min_sector(a_deg, dist, -90.0, -60.0)
    right_side = min_sector(a_deg, dist, 60.0, 90.0)
    nearest = nearest_point(a_deg, dist, x, y)

    if nearest is None:
        print("[NO FRONT POINTS]")
        return

    ang, d, px, py = nearest
    side = "LEFT(y<0)" if py < -0.03 else ("RIGHT(y>0)" if py > 0.03 else "CENTER")
    print(
        f"front10={front10:.2f} front24={front24:.2f} | "
        f"sideL(-60..-15)={left_front:.2f} sideR(15..60)={right_front:.2f} | "
        f"L90={left_side:.2f} R90={right_side:.2f} | "
        f"nearest angle={ang:+.1f}deg d={d:.2f}m x={px:.2f} y={py:+.2f} {side}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Check RPLiDAR left/right angle sign using the same convention as maze_runner.py."
    )
    parser.add_argument("--port", default=LIDAR_PORT)
    parser.add_argument("--baud", type=int, default=LIDAR_BAUD)
    parser.add_argument("--offset", type=float, default=ANGLE_OFFSET_DEG)
    parser.add_argument("--dist-offset", type=float, default=DIST_OFFSET_MM)
    parser.add_argument("--interval", type=float, default=0.30)
    parser.add_argument("--flip-angle", action="store_true")
    args = parser.parse_args()

    print("[INFO] RPLiDAR angle checker")
    print("[INFO] Current convention: negative angle/y = LEFT, positive angle/y = RIGHT")
    print("[INFO] Put a box at LEFT-front, RIGHT-front, then CENTER-front and compare sideL/sideR/y.")
    print("[INFO] If left box appears as sideR or y>0, try --flip-angle.")

    lidar = RPLidarC1(args.port, args.baud)
    try:
        time.sleep(1.0)
        while True:
            scan = lidar.get_scan()
            front_data = build_front_arrays(
                scan, args.offset, args.dist_offset, args.flip_angle
            )
            if front_data is None:
                print("[WAIT] no valid front scan")
            else:
                print_scan_summary(front_data)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[INFO] 종료")
    finally:
        lidar.close()


if __name__ == "__main__":
    main()
