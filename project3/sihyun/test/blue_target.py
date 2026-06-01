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
from lidar import ANG_MAX, ANG_MIN, COLOR_TO_LIDAR_DEG, FREE_D, RPLidarC1, front_ranges


TARGET_COLOR = "BLUE"
CAMERA_LIDAR_HEIGHT_DIFF_M = 0.45
COLOR_TARGET_REAL_HEIGHT_M = 0.625
CAMERA_FOCAL_PX = 625.0
COLOR_OBSTACLE_DISTANCE_TOL_M = 0.40
COLOR_LIDAR_SAMPLE_DEG = 12
LIDAR_OBSTACLE_D_M = FREE_D


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def bottom_center_error(target, frame_shape):
    height, width = frame_shape[:2]
    cx, cy = target[6], target[7]
    dx = cx - width / 2
    dy = max(1.0, height - cy)
    angle = np.arctan2(dx, dy)
    return clamp(angle / (np.pi / 2), -1.0, 1.0)


def color_angle_from_target(target, frame_shape):
    err = bottom_center_error(target, frame_shape)
    return clamp(-err * COLOR_TO_LIDAR_DEG, ANG_MIN, ANG_MAX)


def camera_distance_to_target(target):
    box_h = float(target[4])
    if box_h <= 0.0:
        return None

    return COLOR_TARGET_REAL_HEIGHT_M * CAMERA_FOCAL_PX / box_h


def lidar_distance_at_deg(ranges, target_deg):
    if ranges is None or target_deg is None:
        return None

    center = int(round(clamp(target_deg, ANG_MIN, ANG_MAX) - ANG_MIN))
    radius = int(max(0, COLOR_LIDAR_SAMPLE_DEG))
    start = max(0, center - radius)
    end = min(len(ranges), center + radius + 1)
    if start >= end:
        return None

    window = ranges[start:end]
    valid = window[np.isfinite(window)]
    valid = valid[valid < FREE_D]
    if valid.size == 0:
        return None

    return float(np.min(valid))


def camera_horizontal_distance(camera_d):
    if camera_d is None or camera_d <= CAMERA_LIDAR_HEIGHT_DIFF_M:
        return None

    return float(np.sqrt(camera_d * camera_d - CAMERA_LIDAR_HEIGHT_DIFF_M * CAMERA_LIDAR_HEIGHT_DIFF_M))


def classify_color_target(target, frame_shape, ranges):
    camera_d = camera_distance_to_target(target)
    horizontal_d = camera_horizontal_distance(camera_d)
    target_deg = color_angle_from_target(target, frame_shape)
    lidar_d = lidar_distance_at_deg(ranges, target_deg)

    if lidar_d is None:
        return "UNKNOWN", camera_d, horizontal_d, lidar_d, target_deg

    if lidar_d <= LIDAR_OBSTACLE_D_M:
        return "OBSTACLE_COLOR", camera_d, horizontal_d, lidar_d, target_deg

    if horizontal_d is None:
        return "UNKNOWN", camera_d, horizontal_d, lidar_d, target_deg

    if abs(horizontal_d - lidar_d) <= COLOR_OBSTACLE_DISTANCE_TOL_M:
        return "OBSTACLE_COLOR", camera_d, horizontal_d, lidar_d, target_deg

    return "TRACK_COLOR", camera_d, horizontal_d, lidar_d, target_deg


def draw_blue_classification(frame, found, ranges):
    blue_targets = [item for item in found if item[0] == TARGET_COLOR]

    for target in blue_targets:
        _, x, y, w, h, area, cx, cy, box, cnt = target
        label, camera_d, horizontal_d, lidar_d, target_deg = classify_color_target(target, frame.shape, ranges)
        if label == "OBSTACLE_COLOR":
            color = (0, 0, 255)
        elif label == "UNKNOWN":
            color = (0, 255, 255)
        else:
            color = (255, 0, 0)

        cv2.drawContours(frame, [cnt], -1, color, 2)
        cv2.polylines(frame, [box], True, color, 2)
        cv2.circle(frame, (int(round(cx)), int(round(cy))), 5, color, -1)

        camera_text = "--" if camera_d is None else f"{camera_d:.2f}m"
        horizontal_text = "--" if horizontal_d is None else f"{horizontal_d:.2f}m"
        lidar_text = "--" if lidar_d is None else f"{lidar_d:.2f}m"
        text = f"{label} cam={camera_text} horiz={horizontal_text} lidar={lidar_text} deg={target_deg:.0f} area={area}"

        cv2.putText(
            frame,
            text,
            (x, max(24, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )

    cv2.putText(
        frame,
        "BLUE distance check: q to quit",
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
