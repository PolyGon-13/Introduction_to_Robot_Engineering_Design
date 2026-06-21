#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

from picamera2 import Picamera2



MIN_AREA = 1000
FOLLOW_MAX_V = 0.25
FOLLOW_MAX_W = 1.0
FOLLOW_KP = 1.5
ALIGN_KP = 1.5  # 하단 1/10 구역에서 속도 0으로 방향만 보정할 때 쓰는 회전 비례계수
DEADBAND = 0.08
CAMERA_ROTATION = cv2.ROTATE_90_COUNTERCLOCKWISE

HSV_RANGES = {
    "RED": [([165, 110, 80], [179, 255, 255])],
    "YELLOW": [([19, 75, 85], [30, 255, 255])],
    "BLUE": [([104, 80, 45], [116, 255, 255])],
}
HSV_RANGES = {
    name: [(np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)) for lower, upper in ranges]
    for name, ranges in HSV_RANGES.items()
}
BOX_COLORS = {"RED": (0, 0, 255), "BLUE": (255, 0, 0), "YELLOW": (0, 255, 255)}
MORPH_KERNEL = np.ones((5, 5), np.uint8)

# main.py의 BOTTOM_LOST_RATIO / COLOR_FORWARD_CENTER_RATIO와 같은 값(화면 세로 비율 기준선)
BOTTOM_LOST_RATIO = 0.90  # 하단 1/10 (정렬/정지 판단)
COLOR_FORWARD_CENTER_RATIO = 0.95  # 하단 1/20 (전진/색 전환 판단)


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def open_camera():
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 400)}))
    cam.start()
    return cam


def read_frame(cam):
    frame = cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)
    return True, rotate_frame(frame)


def rotate_frame(frame):
    if CAMERA_ROTATION is None:
        return frame

    return cv2.rotate(frame, CAMERA_ROTATION)


def close_camera(cam):
    if cam is None:
        return

    cam.stop()
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


def follow_cmd(target, frame_shape, max_v=FOLLOW_MAX_V, kp=FOLLOW_KP):
    if target is None:
        return 0.0, 0.0

    err = bottom_center_error(target, frame_shape)
    err = 0.0 if abs(err) < DEADBAND else err

    v = max_v * (1.0 - 0.45 * min(1.0, abs(err)))
    w = clamp(-kp * err, -FOLLOW_MAX_W, FOLLOW_MAX_W)
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
    height, width = frame.shape[:2]

    # 색 중심 세로 위치 판단 기준선 (가로 선)
    y_bottom = int(height * BOTTOM_LOST_RATIO)
    y_forward = int(height * COLOR_FORWARD_CENTER_RATIO)
    cv2.line(frame, (0, y_bottom), (width, y_bottom), (0, 255, 0), 1)  # 1/10: 초록
    cv2.line(frame, (0, y_forward), (width, y_forward), (0, 165, 255), 1)  # 1/20: 주황
    cv2.putText(frame, "BOTTOM 0.90", (5, y_bottom - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    cv2.putText(frame, "FORWARD 0.95", (5, y_forward - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

    cv2.namedWindow("project3", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("project3", width, height)  # 창 크기를 프레임 화소와 동일하게 고정
    cv2.imshow("project3", frame)
    return cv2.waitKey(1) & 0xFF
