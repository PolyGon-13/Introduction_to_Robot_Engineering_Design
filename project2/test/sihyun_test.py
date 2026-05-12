#!/usr/bin/env python3

import serial
import time
import math
import threading
import numpy as np
import matplotlib.pyplot as plt


# =========================
# 라이다 설정
# =========================
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

ANGLE_OFFSET_DEG = +1.54
DIST_OFFSET_MM = 0.0
LIDAR_ANGLE_SIGN = -1.0

MIN_LIDAR_DIST_M = 0.05
MAX_LIDAR_DIST_M = 2.0     # 최대 표시 거리 2m
MIN_QUALITY = 1

PLOT_INTERVAL = 0.02       # 화면 업데이트 간격. 작을수록 빠름


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # Reset
        self.ser.write(bytes([0xA5, 0x40]))
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        # Start scan
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

                # 새 회전 시작
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
                print(f"[LIDAR] Serial Error: {e}")
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
            # Stop scan
            self.ser.write(bytes([0xA5, 0x25]))
        except Exception:
            pass

        time.sleep(0.1)
        self.ser.close()


def scan_to_xy(scan):
    """
    라이다 scan 값을 x, y 좌표로 변환
    x > 0 : 로봇 앞쪽
    y > 0 : 로봇 왼쪽
    y < 0 : 로봇 오른쪽
    """

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


def main():
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)

    print("[INFO] Lidar visualization started.")
    print("[INFO] Press Ctrl+C to stop.")

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))

    scatter = ax.scatter([], [], s=8)

    # 로봇 위치 표시
    robot_dot = ax.scatter([0], [0], s=80, marker="o")

    # 전방 방향 표시
    ax.arrow(
        0,
        0,
        0.35,
        0,
        head_width=0.06,
        head_length=0.08,
        length_includes_head=True,
    )

    ax.set_xlim(-MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M)
    ax.set_ylim(-MAX_LIDAR_DIST_M, MAX_LIDAR_DIST_M)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    ax.set_title("RPLidar Live XY Visualization")
    ax.set_xlabel("X meter : front +")
    ax.set_ylabel("Y meter : left + / right -")

    # 거리 원 표시
    for r in np.arange(0.5, MAX_LIDAR_DIST_M + 0.01, 0.5):
        circle = plt.Circle((0, 0), r, fill=False, linestyle="--", alpha=0.3)
        ax.add_patch(circle)

    try:
        while True:
            scan = lidar.get_scan()
            points = scan_to_xy(scan)

            if len(points) > 0:
                scatter.set_offsets(points)

                print(
                    f"[LIDAR] points={len(points)} "
                    f"min_dist={np.min(np.hypot(points[:, 0], points[:, 1])):.2f}m "
                    f"max_dist={np.max(np.hypot(points[:, 0], points[:, 1])):.2f}m"
                )

            else:
                scatter.set_offsets(np.empty((0, 2)))

            fig.canvas.draw()
            fig.canvas.flush_events()

            time.sleep(PLOT_INTERVAL)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")

    finally:
        lidar.close()
        plt.ioff()
        plt.close()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
