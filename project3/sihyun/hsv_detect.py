#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np


try:
    from picamera2 import Picamera2
    USE_PICAM = True
except ImportError:
    USE_PICAM = False


CAMERA_SIZE = (640, 480)
WINDOW_FRAME = "HSV Detect"
WINDOW_MASK = "HSV Mask"

clicked_hsv = None
clicked_bgr = None
clicked_xy = None


def nothing(value):
    pass


def open_camera():
    if USE_PICAM:
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": CAMERA_SIZE}))
        cam.start()
        return cam

    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_SIZE[0])
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_SIZE[1])
    return cam


def read_frame(cam):
    if USE_PICAM:
        return True, cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)
    return cam.read()


def create_trackbars():
    cv2.namedWindow(WINDOW_FRAME)
    cv2.namedWindow(WINDOW_MASK)

    cv2.createTrackbar("H min", WINDOW_MASK, 0, 179, nothing)
    cv2.createTrackbar("H max", WINDOW_MASK, 179, 179, nothing)
    cv2.createTrackbar("S min", WINDOW_MASK, 0, 255, nothing)
    cv2.createTrackbar("S max", WINDOW_MASK, 255, 255, nothing)
    cv2.createTrackbar("V min", WINDOW_MASK, 0, 255, nothing)
    cv2.createTrackbar("V max", WINDOW_MASK, 255, 255, nothing)


def get_hsv_range():
    h_min = cv2.getTrackbarPos("H min", WINDOW_MASK)
    h_max = cv2.getTrackbarPos("H max", WINDOW_MASK)
    s_min = cv2.getTrackbarPos("S min", WINDOW_MASK)
    s_max = cv2.getTrackbarPos("S max", WINDOW_MASK)
    v_min = cv2.getTrackbarPos("V min", WINDOW_MASK)
    v_max = cv2.getTrackbarPos("V max", WINDOW_MASK)

    lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
    upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
    return lower, upper


def on_mouse(event, x, y, flags, data):
    global clicked_hsv, clicked_bgr, clicked_xy

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    frame = data["frame"]
    if frame is None:
        return

    height, width = frame.shape[:2]
    if x < 0 or x >= width or y < 0 or y >= height:
        return

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    clicked_xy = (x, y)
    clicked_bgr = frame[y, x].tolist()
    clicked_hsv = hsv[y, x].tolist()

    print(f"CLICK x={x}, y={y}, HSV={clicked_hsv}, BGR={clicked_bgr}")


def draw_info(frame, lower, upper):
    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    center_hsv = hsv[center_y, center_x].tolist()
    center_bgr = frame[center_y, center_x].tolist()

    cv2.drawMarker(frame, (center_x, center_y), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
    cv2.putText(frame, f"CENTER HSV={center_hsv}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(frame, f"CENTER BGR={center_bgr}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(frame, f"RANGE lower={lower.tolist()} upper={upper.tolist()}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

    if clicked_xy is not None:
        cv2.circle(frame, clicked_xy, 6, (0, 255, 255), 2)
        cv2.putText(frame, f"CLICK HSV={clicked_hsv}", (10, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(frame, f"CLICK BGR={clicked_bgr}", (10, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)


def main():
    cam = None
    frame_data = {"frame": None}

    try:
        cam = open_camera()
        create_trackbars()
        cv2.setMouseCallback(WINDOW_FRAME, on_mouse, frame_data)

        while True:
            ok, frame = read_frame(cam)
            if not ok:
                print("camera read failed")
                break

            frame_data["frame"] = frame.copy()
            lower, upper = get_hsv_range()

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            draw_info(frame, lower, upper)
            cv2.imshow(WINDOW_FRAME, frame)
            cv2.imshow(WINDOW_MASK, result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            if key == ord("p"):
                print(f"lower={lower.tolist()}, upper={upper.tolist()}")

    finally:
        if cam is not None:
            cam.stop() if USE_PICAM else cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
