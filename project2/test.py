#!/usr/bin/env python3
# wall_distance_measure.py
# 트랙 좌·우 벽 거리 측정 (raw 덤프 + 즉석 통계)
# 캘리브레이션(ANGLE_OFFSET_DEG=+1.54) 적용된 상태로 분석/저장

import serial, time, threading
import numpy as np

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

# ─── 캘리브레이션 적용 ───
ANGLE_OFFSET_DEG = +1.54

# ─── 측정 영역 (보정 후 각도 기준) ───
# +각도 = 우측, -각도 = 좌측 (이전 시각화 검증 결과)
RIGHT_BAND = ( 70, 110)   # +90° 근처
LEFT_BAND  = (-110, -70)  # -90° 근처
FRONT_BAND = ( -15,  15)  # 정면 (참고)


class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.ser.write(bytes([0xA5, 0x40])); time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0xA5, 0x20])); self.ser.read(7)
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


def analyze_band(a180, d, q, band, label, q_min=20):
    lo, hi = band
    m = (a180 > lo) & (a180 < hi) & (d > 50) & (d < 5000) & (q >= q_min)
    if m.sum() < 5:
        return None
    sub_a, sub_d = a180[m], d[m]
    idx_min = int(np.argmin(sub_d))
    return {
        'label': label,
        'n': int(m.sum()),
        'closest_dist':  float(sub_d[idx_min]),
        'closest_angle': float(sub_a[idx_min]),
        'median': float(np.median(sub_d)),
        'mean':   float(np.mean(sub_d)),
        'std':    float(np.std(sub_d)),
    }


def dump_scan(scan, filename):
    a, d, q = scan
    a_corr = (a + ANGLE_OFFSET_DEG) % 360.0
    a180 = ((a_corr + 180.0) % 360.0) - 180.0
    idx = np.argsort(a180)
    with open(filename, 'w') as f:
        f.write(f"# RPLiDAR C1 wall scan (calibrated)\n")
        f.write(f"# ANGLE_OFFSET_DEG = {ANGLE_OFFSET_DEG:+.3f}\n")
        f.write(f"# columns: angle_180_deg, distance_mm, quality\n")
        for ai, di, qi in zip(a180[idx], d[idx], q[idx]):
            f.write(f"{ai:+8.3f}, {di:8.2f}, {int(qi):3d}\n")


def print_result(label, r, ruler_mm=None):
    if r is None:
        print(f"  {label:20s}  데이터 부족")
        return
    line = (f"  {label:20s}  n={r['n']:3d}  "
            f"최단 {r['closest_dist']:7.1f}mm@{r['closest_angle']:+6.2f}°  "
            f"중앙 {r['median']:7.1f}mm  평균 {r['mean']:7.1f}mm")
    if ruler_mm is not None:
        diff = r['closest_dist'] - ruler_mm
        line += f"  | 자: {ruler_mm}mm, 차: {diff:+.1f}mm"
    print(line)


def main():
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    print("워밍업..."); time.sleep(2.5)

    print("="*78)
    print("  트랙 양쪽 벽 거리 측정")
    print("="*78)
    print(f"적용 캘리브레이션: ANGLE_OFFSET_DEG = {ANGLE_OFFSET_DEG:+.2f}")
    print("로봇을 START 라인 위에 두고, 자로 좌·우 벽까지 거리를 미리 재두세요.")
    print("자로 잴 때는 라이다 회전축 중심에서 직각으로 측정.")
    print()

    measurements = []
    for i in range(3):
        print(f"\n--- 측정 {i+1}/3 ---")
        try:
            ruler_left  = input("  좌측(흰 벽) 자 측정값 mm (모르면 Enter): ").strip()
            ruler_right = input("  우측(코부키) 자 측정값 mm (모르면 Enter): ").strip()
            ruler_left  = float(ruler_left)  if ruler_left  else None
            ruler_right = float(ruler_right) if ruler_right else None
        except ValueError:
            ruler_left = ruler_right = None
        input("  자세 확정 후 Enter... ")

        scan = None
        for _ in range(20):
            scan = lidar.get_scan()
            if scan is not None: break
            time.sleep(0.1)
        if scan is None:
            print("  스캔 실패"); continue

        a, d, q = scan
        a_corr = (a + ANGLE_OFFSET_DEG) % 360.0
        a180 = ((a_corr + 180.0) % 360.0) - 180.0

        right = analyze_band(a180, d, q, RIGHT_BAND, "우측 (+90°)")
        left  = analyze_band(a180, d, q, LEFT_BAND,  "좌측 (-90°)")
        front = analyze_band(a180, d, q, FRONT_BAND, "정면 (참고)")

        print()
        print_result("좌측 (-90°)", left,  ruler_left)
        print_result("우측 (+90°)", right, ruler_right)
        print_result("정면 (참고)", front)

        fname = f"wall_scan_{i+1}.txt"
        dump_scan(scan, fname)
        print(f"  저장: {fname}")

        measurements.append({'left': left, 'right': right,
                             'ruler_left': ruler_left, 'ruler_right': ruler_right})

    lidar.close()

    # 전체 통계
    if len(measurements) >= 2:
        print("\n" + "="*78)
        print("  전체 통계")
        print("="*78)
        for key, name in [('left', '좌측 (-90°)'), ('right', '우측 (+90°)')]:
            vals = [m[key]['closest_dist'] for m in measurements if m[key] is not None]
            rulers = [m[f'ruler_{key}'] for m in measurements if m[f'ruler_{key}'] is not None]
            if len(vals) >= 2:
                print(f"\n{name}  최단거리 측정값: {[f'{v:.1f}' for v in vals]}")
                print(f"          → 평균 {np.mean(vals):.1f}mm,  편차 {np.std(vals):.1f}mm")
                if len(rulers) >= 1:
                    diffs = []
                    for m in measurements:
                        if m[key] is not None and m[f'ruler_{key}'] is not None:
                            diffs.append(m[key]['closest_dist'] - m[f'ruler_{key}'])
                    print(f"          → 자와 차: {[f'{d:+.1f}' for d in diffs]}mm")


if __name__ == "__main__":
    main()