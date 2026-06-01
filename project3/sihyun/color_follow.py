#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import cv2
import numpy as np
import serial


try:
    from picamera2 import Picamera2
    USE_PICAM = True
except ImportError:
    USE_PICAM = False


# 따라갈 색상: "RED", "BLUE", "YELLOW" 중 하나로 변경, None이면 모든 색 중 가장 큰 물체 추적
TARGET = "RED"
SHOW_WINDOW = True
MIN_AREA = 200
MAX_V, MAX_W, KP = 0.18, 0.70, 0.85
V_STEP, W_STEP, DEADBAND = 0.04, 0.15, 0.08
ARDU_PORT, ARDU_BAUD = "/dev/ttyS0", 9600

HSV_RANGES = {
    "RED": [([0, 90, 120], [10, 255, 255]), ([170, 90, 120], [179, 255, 255])],
    "BLUE": [([102, 85, 110], [126, 255, 255])],
    "YELLOW": [([22, 85, 140], [36, 255, 255])],
}
BOX_COLORS = {"RED": (0, 0, 255), "BLUE": (255, 0, 0), "YELLOW": (0, 255, 255)}


def open_camera():
    if USE_PICAM:
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
        cam.start()
        return cam
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cam


def read_frame(cam):
    if USE_PICAM:
        return True, cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)
    return cam.read()


def detect(frame):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    found = []
    for name, ranges in HSV_RANGES.items():
        mask = None
        for lower, upper in ranges:
            part = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= MIN_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                found.append((name, x, y, w, h, int(area)))
    return found


def pick(found):
    targets = found if TARGET is None else [item for item in found if item[0] == TARGET]
    return max(targets, key=lambda item: item[5], default=None)


def follow_cmd(target, width):
    if target is None:
        return 0.0, 0.0
    _, x, _, w, _, _ = target
    err = (x + w / 2 - width / 2) / (width / 2)
    err = 0.0 if abs(err) < DEADBAND else err
    v = MAX_V * (1.0 - 0.45 * min(1.0, abs(err)))
    w = max(-MAX_W, min(-KP * err, MAX_W))
    return v, w


def rate_limit(prev, target, step):
    return prev + max(-step, min(target - prev, step))


def send_vw(motor, v, w):
    motor.write(f"V{v:.3f},{w:.3f}\n".encode("ascii"))


def stop(motor):
    motor.write(b"S\n")


def draw(frame, found):
    for name, x, y, w, h, area in found:
        color = BOX_COLORS[name]
        cx, cy = x + w // 2, y + h // 2
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.circle(frame, (cx, cy), 5, color, -1)
        cv2.putText(frame, f"{name} {cx},{cy} {area}", (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def main():
    cam = open_camera()
    motor = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    time.sleep(2.0)
    last_v = last_w = 0.0
    try:
        stop(motor)
        while True:
            ok, frame = read_frame(cam)
            if not ok:
                break
            found = detect(frame)
            v, w = follow_cmd(pick(found), frame.shape[1])
            last_v = rate_limit(last_v, v, V_STEP)
            last_w = rate_limit(last_w, w, W_STEP)
            send_vw(motor, last_v, last_w)

            if SHOW_WINDOW:
                draw(frame, found)
                cv2.imshow("follow", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            time.sleep(0.05)
    finally:
        stop(motor)
        motor.close()
        cam.stop() if USE_PICAM else cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
