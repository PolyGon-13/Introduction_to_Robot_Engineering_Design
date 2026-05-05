#!/usr/bin/env python3
# lidar_calibration_v2.py
# 직선 fitting 기반 정확한 캘리브레이션 (3번 측정 평균)

import serial, time, threading
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
            if len(data) != 5: continue
            s_flag = data[0] & 0x01
            s_inv  = (data[0] & 0x02) >> 1
            if s_inv != (1 - s_flag): continue
            if (data[1] & 0x01) != 1: continue
            quality = data[0] >> 2
            angle  = ((data[1] >> 1) | (data[2] << 7)) / 64.0
            dist   = (data[3] | (data[4] << 8)) / 4.0

            if s_flag == 1 and len(buf_a) > 50:
                with self.lock:
                    self.latest_scan = (np.array(buf_a), np.array(buf_d), np.array(buf_q))
                buf_a, buf_d, buf_q = [], [], []
            if dist > 0 and quality > 0:
                buf_a.append(angle); buf_d.append(dist); buf_q.append(quality)

    def get_scan(self):
        with self.lock:
            return self.latest_scan

    def close(self):
        self.running = False
        try: self.ser.write(bytes([0xA5, 0x25]))
        except: pass
        time.sleep(0.1); self.ser.close()


def fit_wall(scan, fov_half=25, q_min=30):
    """정면 벽에 직선 fitting하여 (벽 법선 각도, 수선거리, 잔차RMS) 반환"""
    a, d, q = scan
    a180 = ((a + 180.0) % 360.0) - 180.0
    m = (np.abs(a180) <= fov_half) & (d > 100) & (d < 2000) & (q >= q_min)
    if m.sum() < 20: return None
    aw, dw = a180[m], d[m]
    th = np.deg2rad(aw)
    x = dw * np.cos(th)
    y = dw * np.sin(th)
    # x = slope*y + intercept
    A = np.vstack([y, np.ones_like(y)]).T
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    slope, intercept = coef
    true_front = float(np.degrees(np.arctan2(-slope, 1.0)))
    perp_dist  = float(abs(intercept) / np.sqrt(1 + slope**2))
    res = x - (slope*y + intercept)
    rms = float(np.sqrt(np.mean(res**2)))
    return true_front, perp_dist, rms, len(aw)


def main():
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    print("워밍업...")
    time.sleep(2.5)

    print("="*60)
    print("LiDAR 캘리브레이션 v2 (직선 fitting)")
    print("="*60)
    print("벽 정면 50~100cm, 평행 정렬 후 측정합니다.")
    print()

    results = []
    for i in range(3):
        input(f"[{i+1}/3] 자세 잡고 Enter... ")
        # 새 스캔 받기
        for _ in range(20):
            scan = lidar.get_scan()
            if scan is not None: break
            time.sleep(0.1)
        if scan is None:
            print("  스캔 실패"); continue
        r = fit_wall(scan)
        if r is None:
            print("  벽 검출 실패 - 자세/거리 확인"); continue
        ang, dist, rms, n = r
        print(f"  → 정면각도: {ang:+.3f}°,  수선거리: {dist:.1f}mm,  "
              f"잔차RMS: {rms:.2f}mm  (점 {n}개)")
        results.append(r)

    lidar.close()

    if len(results) < 2:
        print("\n측정 부족. 다시 시도하세요.")
        return

    angs = np.array([r[0] for r in results])
    print()
    print("="*60)
    print(f"3회 평균 정면 각도: {angs.mean():+.3f}°")
    print(f"표준편차:           {angs.std():.3f}°")
    print(f"")
    if angs.std() < 0.5:
        print(f"✓ 일관성 우수")
        print(f"")
        print(f"  ANGLE_OFFSET_DEG = {-angs.mean():+.2f}")
        print(f"")
    elif angs.std() < 1.5:
        print(f"~ 일관성 보통, 평균값 사용")
        print(f"  ANGLE_OFFSET_DEG = {-angs.mean():+.2f}")
    else:
        print(f"✗ 일관성 낮음 - 자세 정렬 다시 확인")


if __name__ == "__main__":
    main() 