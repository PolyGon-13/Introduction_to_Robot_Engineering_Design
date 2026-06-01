#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from pathlib import Path

import cv2
import numpy as np

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from camera import close_camera, detect, open_camera, read_frame
from lidar import ANG_MAX, ANG_MIN, COLOR_TO_LIDAR_DEG, MAX_D, RPLidarC1, front_ranges


TARGET_COLOR = "BLUE"
CAMERA_LIDAR_HEIGHT_DIFF_M = 0.45
COLOR_TARGET_REAL_HEIGHT_M = 0.625
CAMERA_FOCAL_PX = 625.0


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def color_angle_from_target(target, frame_shape):
    height, width = frame_shape[:2]
    cx, cy = target[6], target[7]
    dx = cx - width / 2
    dy = max(1.0, height - cy)
    angle = np.arctan2(dx, dy)
    err = clamp(angle / (np.pi / 2), -1.0, 1.0)
    return clamp(-err * COLOR_TO_LIDAR_DEG, ANG_MIN, ANG_MAX)


def camera_distance_to_target(target):
    box_h = float(target[4])
    if box_h <= 0.0:
        return None

    camera_d = COLOR_TARGET_REAL_HEIGHT_M * CAMERA_FOCAL_PX / box_h
    if camera_d <= CAMERA_LIDAR_HEIGHT_DIFF_M:
        return None

    return float(np.sqrt(camera_d * camera_d - CAMERA_LIDAR_HEIGHT_DIFF_M * CAMERA_LIDAR_HEIGHT_DIFF_M))


def lidar_distance_at_target(ranges, target_deg):
    if ranges is None:
        return None

    index = int(round(clamp(target_deg, ANG_MIN, ANG_MAX) - ANG_MIN))
    if index < 0 or index >= len(ranges):
        return None

    lidar_d = float(ranges[index])
    if not np.isfinite(lidar_d) or lidar_d >= MAX_D:
        return None

    return lidar_d


def classify_color_target(target, frame_shape, ranges):
    camera_d = camera_distance_to_target(target)
    target_deg = color_angle_from_target(target, frame_shape)
    lidar_d = lidar_distance_at_target(ranges, target_deg)

    if camera_d is not None and lidar_d is not None and abs(camera_d - lidar_d) <= 0.25:
        return "OBSTACLE_COLOR", camera_d, lidar_d, target_deg

    return "TRACK_COLOR", camera_d, lidar_d, target_deg


def draw_blue_classification(frame, found, ranges):
    blue_targets = [item for item in found if item[0] == TARGET_COLOR]

    for target in blue_targets:
        _, x, y, w, h, area, cx, cy, box, cnt = target
        label, camera_d, lidar_d, target_deg = classify_color_target(target, frame.shape, ranges)
        color = (0, 0, 255) if label == "OBSTACLE_COLOR" else (255, 0, 0)

        cv2.drawContours(frame, [cnt], -1, color, 2)
        cv2.polylines(frame, [box], True, color, 2)
        cv2.circle(frame, (int(round(cx)), int(round(cy))), 5, color, -1)

        camera_text = "--" if camera_d is None else f"{camera_d:.2f}m"
        lidar_text = "--" if lidar_d is None else f"{lidar_d:.2f}m"
        text = f"{label} cam={camera_text} lidar={lidar_text} deg={target_deg:.0f} area={area}"
        cv2.putText(frame, text, (x, max(24, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    cv2.putText(
        frame,
        "BLUE camera-lidar check: q to quit",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
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

            draw_blue_classification(frame, found, ranges)
            cv2.imshow("blue_distance", frame)
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
