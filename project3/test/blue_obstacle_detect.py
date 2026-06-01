#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from pathlib import Path

import cv2
import numpy as np

# camera.py / lidar.py 는 project3/sihyun/test 에 있다. 그 폴더만 import 경로에 추가.
THIS_DIR = Path(__file__).resolve().parent
SIHYUN_TEST_DIR = THIS_DIR.parent / "sihyun" / "test"
if str(SIHYUN_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(SIHYUN_TEST_DIR))

from camera import close_camera, detect, open_camera, read_frame
from lidar import ANG_MAX, ANG_MIN, MAX_D, RPLidarC1, front_ranges


# ==============================
# 판정 파라미터 (현장 보정 대상)
# ==============================

TARGET_COLOR = "BLUE"

# 카메라 blob 의 가로 픽셀 위치 -> 방위각 변환용 초점거리(px). blue_target.py 와 동일값.
CAMERA_FOCAL_PX = 625.0
# 카메라와 라이다의 장착 방위(yaw) 오프셋(도). 두 센서 0도 축을 맞추는 보정값.
CAM_LIDAR_YAW_OFFSET_DEG = 0.0
# 카메라 +x(오른쪽) 가 라이다 각도의 어느 부호인지. lidar.py 규약(오른쪽=음수)에 맞춤.
CAM_AZIMUTH_SIGN = -1.0
# blob 방위각 주변에서 라이다를 살펴볼 윈도우 반폭(도).
# 좁게 둬야 한 화면에 다른 파란 장애물/영역이 같이 있어도 그쪽 라이다 반사를 끌어오지 않는다.
# (정렬 오차를 흡수할 정도만. 장애물이 통째로 안 잡히면 폭을 넓히지 말고
#  CAM_LIDAR_YAW_OFFSET_DEG / CAM_AZIMUTH_SIGN 정렬부터 맞출 것.)
LIDAR_WINDOW_HALF_DEG = 5.0

# 핵심 임계: blob 방위각에서 라이다 최소거리가 이 값보다 가까우면 "높이 있는 장애물"로 본다.
# (바닥 영역은 라이다 반사가 없어 None 이 되므로 자동으로 FLOOR 로 분류된다.)
OBSTACLE_MAX_D = 1.5

# ----- 아래는 보조(로그/표시) 전용: box 높이로 추정한 카메라 거리. 판정에는 쓰지 않음. -----
CAMERA_LIDAR_HEIGHT_DIFF_M = 0.45   # 카메라-라이다 수직 거리(m)
COLOR_TARGET_REAL_HEIGHT_M = 0.625  # 파란 물체 실제 높이 가정(m)

LABEL_COLORS = {"FLOOR": (0, 255, 0), "OBSTACLE": (0, 0, 255)}  # BGR


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def blob_lidar_angle_deg(target, frame_shape):
    """파란 blob 중심의 가로 위치를 라이다 각도(도)로 변환."""
    _, width = frame_shape[:2]
    cx = target[6]
    dx = cx - width / 2.0
    azimuth = np.degrees(np.arctan2(dx, CAMERA_FOCAL_PX))
    lidar_deg = CAM_LIDAR_YAW_OFFSET_DEG + CAM_AZIMUTH_SIGN * azimuth
    return clamp(lidar_deg, ANG_MIN, ANG_MAX)


def lidar_min_in_window(ranges, center_deg, half_width_deg):
    """[center-half, center+half] 윈도우 안 라이다 최소거리. 반사 없으면 None."""
    if ranges is None:
        return None

    lo = clamp(center_deg - half_width_deg, ANG_MIN, ANG_MAX)
    hi = clamp(center_deg + half_width_deg, ANG_MIN, ANG_MAX)
    i_lo = int(round(lo - ANG_MIN))
    i_hi = int(round(hi - ANG_MIN))
    if i_hi < i_lo:
        i_lo, i_hi = i_hi, i_lo
    i_hi = min(i_hi, len(ranges) - 1)
    if i_lo < 0 or i_lo > i_hi:
        return None

    window = ranges[i_lo:i_hi + 1]
    measured = window[np.isfinite(window) & (window < MAX_D)]  # MAX_D = 반사 없음
    if len(measured) == 0:
        return None

    return float(np.min(measured))


def camera_distance_to_target(target):
    """(보조) box 높이로 추정한 수평거리(m). 너무 작으면 None. 표시/로그 전용."""
    box_h = float(target[4])
    if box_h <= 0.0:
        return None

    camera_d = COLOR_TARGET_REAL_HEIGHT_M * CAMERA_FOCAL_PX / box_h
    if camera_d <= CAMERA_LIDAR_HEIGHT_DIFF_M:
        return None

    return float(np.sqrt(camera_d * camera_d - CAMERA_LIDAR_HEIGHT_DIFF_M * CAMERA_LIDAR_HEIGHT_DIFF_M))


def classify_blue_target(target, frame_shape, ranges):
    """파란 blob 1개를 FLOOR/OBSTACLE 로 분류.

    반환: (label, lidar_deg, d_lidar, d_cam)
      - label  : "FLOOR"(바닥 영역) 또는 "OBSTACLE"(높이 있는 장애물)
      - lidar_deg: blob 방위각(도)
      - d_lidar : 그 방위각 윈도우의 라이다 최소거리(m) 또는 None(반사 없음)
      - d_cam   : box 높이 기반 카메라 추정거리(m) 또는 None (보조 표시용)
    """
    lidar_deg = blob_lidar_angle_deg(target, frame_shape)
    # blob 폭이 아니라 '좁은 고정 윈도우'로 본다. (폭에 비례시키면 크게 찍힌 바닥 영역의
    # 윈도우가 옆 장애물 각도까지 덮어, 그 장애물 반사 때문에 바닥까지 OBSTACLE 로 오분류됨.)
    d_lidar = lidar_min_in_window(ranges, lidar_deg, LIDAR_WINDOW_HALF_DEG)
    d_cam = camera_distance_to_target(target)

    # 라이다 존재 기반 판정: 그 방위각에 근거리 반사가 있으면 높이 있는 장애물.
    is_obstacle = d_lidar is not None and d_lidar <= OBSTACLE_MAX_D
    label = "OBSTACLE" if is_obstacle else "FLOOR"
    return label, lidar_deg, d_lidar, d_cam


def floor_blue_targets(found, frame_shape, ranges):
    """detect() 결과에서 '바닥 파란 영역'으로 판정된 BLUE blob 만 반환."""
    floor = []
    for target in found:
        if target[0] != TARGET_COLOR:
            continue
        if classify_blue_target(target, frame_shape, ranges)[0] == "FLOOR":
            floor.append(target)
    return floor


def without_blue_obstacles(found, frame_shape, ranges):
    """detect() 결과에서 '높이 있는 파란 장애물'만 제거한 목록 반환.

    RED/YELLOW 및 바닥 BLUE 는 그대로 통과 -> main 의 pick() 앞에 끼워 넣기 좋은 형태.
        found = without_blue_obstacles(detect(frame), frame.shape, ranges)
    """
    result = []
    for target in found:
        if target[0] == TARGET_COLOR and classify_blue_target(target, frame_shape, ranges)[0] == "OBSTACLE":
            continue
        result.append(target)
    return result


def draw_classification(frame, found, ranges):
    """단독 실행용 시각화: FLOOR=초록, OBSTACLE=빨강."""
    for target in found:
        if target[0] != TARGET_COLOR:
            continue

        _, x, y, w, h, area, cx, cy, box, cnt = target
        label, lidar_deg, d_lidar, d_cam = classify_blue_target(target, frame.shape, ranges)
        color = LABEL_COLORS[label]

        cv2.drawContours(frame, [cnt], -1, color, 2)
        cv2.polylines(frame, [box], True, color, 2)
        cv2.circle(frame, (int(round(cx)), int(round(cy))), 5, color, -1)

        lidar_text = "none" if d_lidar is None else f"{d_lidar:.2f}m"
        cam_text = "--" if d_cam is None else f"{d_cam:.2f}m"
        text = f"{label} deg={lidar_deg:.0f} lidar={lidar_text} cam={cam_text}"
        cv2.putText(frame, text, (x, max(24, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    cv2.putText(
        frame,
        "BLUE floor(green) vs obstacle(red): q to quit",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )


def main():
    cam = lidar = None

    try:
        cam = open_camera()
        lidar = RPLidarC1()

        while True:
            ok, frame = read_frame(cam)
            if not ok:
                break

            scan, _, _ = lidar.get()
            ranges = front_ranges(scan) if scan is not None else None
            found = detect(frame)

            draw_classification(frame, found, ranges)
            cv2.imshow("blue_obstacle_detect", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\n[INFO] stop")

    finally:
        if lidar is not None:
            lidar.close()

        close_camera(cam)


if __name__ == "__main__":
    main()
