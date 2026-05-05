#!/usr/bin/env python3
# lidar_calibration_wall.py
# 평평한 벽을 정면으로 마주 보고 세운 뒤 실행

import time, numpy as np
from maze_runner import RPLidarC1, LIDAR_PORT, LIDAR_BAUD

lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
time.sleep(2.0)

print("로봇을 평평한 벽 정면으로 약 0.5~1.0m 떨어진 거리에 세워주세요.")
print("로봇 좌우가 벽과 평행이 되도록 정렬 (양 옆 거리가 같게).")
print("Ctrl+C로 종료.\n")

try:
    while True:
        scan = lidar.get_scan()
        if scan is None:
            time.sleep(0.1); continue
        a, d, q = scan
        a180 = ((a + 180.0) % 360.0) - 180.0

        # ── 방법 1: 정면 ±20°에서 가장 가까운 점이 벽의 "수선의 발" ──
        front_mask = (np.abs(a180) <= 20) & (d > 50) & (d < 4000) & (q > 0)
        if front_mask.sum() < 10:
            print("정면 데이터 부족, 자세 확인"); time.sleep(0.5); continue

        sub_a = a180[front_mask]
        sub_d = d[front_mask]

        # 가장 가까운 점 = 벽까지 수선의 발 = 진짜 정면
        closest_idx = int(np.argmin(sub_d))
        closest_angle = float(sub_a[closest_idx])
        closest_dist  = float(sub_d[closest_idx])

        # ── 방법 2: 좌우 대칭성 검증 ──
        # 정면 ±10°와 ±20° 영역에서 좌우 평균 거리 비교
        left_band  = (a180 > 5)  & (a180 < 25)  & (d > 50) & (d < 4000)
        right_band = (a180 < -5) & (a180 > -25) & (d > 50) & (d < 4000)
        if left_band.sum() > 5 and right_band.sum() > 5:
            left_mean  = float(np.mean(d[left_band]))
            right_mean = float(np.mean(d[right_band]))
            symmetry_diff = left_mean - right_mean  # 양수면 좌측이 더 멈
        else:
            left_mean = right_mean = symmetry_diff = float('nan')

        print(f"[정면 수선] 각도: {closest_angle:+6.2f}°  거리: {closest_dist:7.1f}mm")
        print(f"[좌우 대칭] 좌(+5~+25°): {left_mean:7.1f}mm  우(-25~-5°): {right_mean:7.1f}mm  차: {symmetry_diff:+7.1f}mm")
        print(f"  → 권장 ANGLE_OFFSET_DEG = {-closest_angle:+.2f}\n")

        time.sleep(0.5)
except KeyboardInterrupt:
    lidar.close()