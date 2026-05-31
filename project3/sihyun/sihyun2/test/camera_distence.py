#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import cv2
import numpy as np

from camera import (
    BOX_COLORS,
    close_camera,
    detect,
    open_camera,
    pick,
    read_frame,
)


TARGET_COLOR = "RED"
LOG_INTERVAL_S = 0.2

CAMERA_HEIGHT_M = 0.625
CAMERA_PITCH_DOWN_DEG = 45.0
CAMERA_VERTICAL_FOV_DEG = 52
CAMERA_ROBOT_FORWARD_OFFSET_M = 0.129
MIN_GROUND_DISTANCE_M = 0.02
MAX_GROUND_DISTANCE_M = 1.50


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def pinhole_ground_distance_to_target(target, frame_shape):
    if target is None:
        return None

    height = frame_shape[0]
    cy = float(target[7])
    fy = (height / 2.0) / np.tan(np.deg2rad(CAMERA_VERTICAL_FOV_DEG) / 2.0)
    ray_down_deg = CAMERA_PITCH_DOWN_DEG + np.rad2deg(np.arctan2(cy - height / 2.0, fy))

    if ray_down_deg <= 0.0:
        return None

    distance_m = CAMERA_HEIGHT_M / np.tan(np.deg2rad(ray_down_deg))
    distance_m -= CAMERA_ROBOT_FORWARD_OFFSET_M
    return clamp(distance_m, MIN_GROUND_DISTANCE_M, MAX_GROUND_DISTANCE_M)


def draw_distance(frame, target, distance_m):
    mode = f"{TARGET_COLOR}: not found"

    if target is not None:
        name, x, y, w, h, area, cx, cy, box, cnt = target
        color = BOX_COLORS[name]
        center = (int(round(cx)), int(round(cy)))
        distance_text = "--" if distance_m is None else f"{distance_m * 100.0:.1f}cm"
        mode = f"{name} distance={distance_text}"

        cv2.drawContours(frame, [cnt], -1, color, 2)
        cv2.polylines(frame, [box], True, color, 2)
        cv2.circle(frame, center, 5, color, -1)
        cv2.putText(
            frame,
            f"{name} {distance_text}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )

    cv2.putText(frame, mode, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)


def main():
    cam = None
    last_log_time = 0.0

    try:
        cam = open_camera()

        while True:
            ok, frame = read_frame(cam)
            if not ok:
                print("[CAMERA] frame read failed")
                break

            target = pick(detect(frame), TARGET_COLOR)
            distance_m = pinhole_ground_distance_to_target(target, frame.shape)

            now = time.time()
            if now - last_log_time >= LOG_INTERVAL_S:
                if target is None:
                    print("[RED] not found")
                elif distance_m is None:
                    print("[RED] distance unavailable")
                else:
                    print(f"[RED] distance={distance_m:.3f}m ({distance_m * 100.0:.1f}cm)")
                last_log_time = now

            draw_distance(frame, target, distance_m)
            cv2.imshow("red distance tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\n[INFO] stop")

    finally:
        close_camera(cam)


if __name__ == "__main__":
    main()
