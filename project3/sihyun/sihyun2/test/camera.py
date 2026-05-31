#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

try:
    from picamera2 import Picamera2

    USE_PICAM = True
except ImportError:
    USE_PICAM = False


MIN_AREA = 200
FOLLOW_MAX_V = 0.25
FOLLOW_MAX_W = 0.70
FOLLOW_KP = 0.85
DEADBAND = 0.08
CAMERA_ROTATION = cv2.ROTATE_90_COUNTERCLOCKWISE

HSV_RANGES = {
    "RED": [([0, 100, 120], [5, 255, 255]), ([165, 80, 120], [179, 255, 255])],
    "BLUE": [([104, 90, 90], [116, 255, 230])],
    "YELLOW": [([19, 130, 170], [25, 255, 255])],
}
HSV_RANGES = {
    name: [(np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)) for lower, upper in ranges]
    for name, ranges in HSV_RANGES.items()
}
BOX_COLORS = {"RED": (0, 0, 255), "BLUE": (255, 0, 0), "YELLOW": (0, 255, 255)}
MORPH_KERNEL = np.ones((5, 5), np.uint8)


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def open_camera():
    if USE_PICAM:
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
        cam.start()
        return cam

    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cam


def read_frame(cam):
    if USE_PICAM:
        frame = cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)
        return True, rotate_frame(frame)

    ok, frame = cam.read()
    if not ok:
        return False, frame

    return True, rotate_frame(frame)


def rotate_frame(frame):
    if CAMERA_ROTATION is None:
        return frame

    return cv2.rotate(frame, CAMERA_ROTATION)


def close_camera(cam):
    if cam is None:
        return

    cam.stop() if USE_PICAM else cam.release()
    cv2.destroyAllWindows()


def detect(frame):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    found = []

    for name, ranges in HSV_RANGES.items():
        mask = None

        for lower, upper in ranges:
            part = cv2.inRange(hsv, lower, upper)
            mask = part if mask is None else cv2.bitwise_or(mask, part)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area >= MIN_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                rect = cv2.minAreaRect(cnt)
                approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
                box = approx.reshape(-1, 2) if len(approx) >= 3 else cv2.boxPoints(rect).astype(np.int32)
                moments = cv2.moments(cnt)

                if moments["m00"] != 0:
                    cx = moments["m10"] / moments["m00"]
                    cy = moments["m01"] / moments["m00"]
                else:
                    cx = x + w / 2
                    cy = y + h / 2

                found.append((name, x, y, w, h, int(area), float(cx), float(cy), box, cnt))

    return found


def pick(found, target_name):
    targets = found if target_name is None else [item for item in found if item[0] == target_name]
    return max(targets, key=lambda item: item[5], default=None)


def bottom_center_error(target, frame_shape):
    height, width = frame_shape[:2]
    cx, cy = target[6], target[7]
    dx = cx - width / 2
    dy = max(1.0, height - cy)
    angle = np.arctan2(dx, dy)
    return clamp(angle / (np.pi / 2), -1.0, 1.0)


def x_center_error(target, frame_shape):
    _, width = frame_shape[:2]
    cx = target[6]
    return clamp((cx - width / 2) / (width / 2), -1.0, 1.0)


def follow_cmd(target, frame_shape, max_v=FOLLOW_MAX_V):
    if target is None:
        return 0.0, 0.0

    err = bottom_center_error(target, frame_shape)
    err = 0.0 if abs(err) < DEADBAND else err

    v = max_v * (1.0 - 0.45 * min(1.0, abs(err)))
    w = clamp(-FOLLOW_KP * err, -FOLLOW_MAX_W, FOLLOW_MAX_W)
    return v, w


def draw(frame, found, mode):
    for name, x, y, w, h, area, cx, cy, box, cnt in found:
        color = BOX_COLORS[name]
        center = (int(round(cx)), int(round(cy)))

        cv2.drawContours(frame, [cnt], -1, color, 2)
        cv2.polylines(frame, [box], True, color, 2)
        cv2.circle(frame, center, 5, color, -1)
        cv2.putText(
            frame,
            f"{name} {center[0]},{center[1]} {area}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

    cv2.putText(frame, mode, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.imshow("project3", frame)
    return cv2.waitKey(1) & 0xFF
