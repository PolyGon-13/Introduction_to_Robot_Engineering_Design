#!/usr/bin/env python3
# lidar_dump.py
# 한 스캔의 모든 점을 텍스트 파일로 저장 (분석용)

import serial
import time
import threading
import numpy as np

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800


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
            s_inv_flag = (data[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag):
                continue
            check_bit = data[1] & 0x01
            if check_bit != 1:
                continue
            quality = data[0] >> 2
            angle_q6 = ((data[1] >> 1) | (data[2] << 7))
            dist_q2 = (data[3] | (data[4] << 8))
            angle_deg = angle_q6 / 64.0
            dist_mm = dist_q2 / 4.0

            if s_flag == 1 and len(buf_a) > 50:
                with self.lock:
                    self.latest_scan = (
                        np.array(buf_a, dtype=np.float32),
                        np.array(buf_d, dtype=np.float32),
                        np.array(buf_q, dtype=np.float32),
                    )
                buf_a, buf_d, buf_q = [], [], []

            if dist_mm > 0 and quality > 0:
                buf_a.append(angle_deg)
                buf_d.append(dist_mm)
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


def dump_scan(scan, filename):
    a, d, q = scan
    # 각도 0~360 그대로 저장 (가공 안 함)
    with open(filename, 'w') as f:
        f.write("# RPLiDAR C1 raw scan\n")
        f.write("# columns: angle_deg, distance_mm, quality\n")
        for ai, di, qi in zip(a, d, q):
            f.write(f"{ai:7.3f}, {di:8.2f}, {int(qi):3d}\n")
    print(f"saved: {filename}  ({len(a)} points)")


def main():
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    print("워밍업 중...")
    time.sleep(2.5)

    # 3번 측정 (자세를 약간씩 바꿔가며 측정해도 좋음)
    for i in range(3):
        input(f"\n[측정 {i+1}/3] 로봇 자세 잡고 Enter 누르세요...")
        # 새로운 스캔 강제로 받기
        for _ in range(20):
            scan = lidar.get_scan()
            if scan is not None:
                break
            time.sleep(0.1)
        if scan is None:
            print("스캔 실패, 재시도")
            continue
        dump_scan(scan, f"scan_{i+1}.txt")

    lidar.close()
    print("\n완료. scan_1.txt, scan_2.txt, scan_3.txt 파일이 생성됨.")


if __name__ == "__main__":
    main()