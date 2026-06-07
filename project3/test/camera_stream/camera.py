#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2

try:
    from picamera2 import Picamera2
    USE_PICAM = True
except ImportError:
    USE_PICAM = False


CAMERA_SIZE = (640, 480)
WINDOW_NAME = "Camera View"

# 카메라를 반시계 방향 90도로 돌려 달았으므로,
# 화면은 시계 방향 90도 회전해서 보정
ROTATE_MODE = cv2.ROTATE_270_CLOCKWISE


def open_camera():
    if USE_PICAM:
        cam = Picamera2()
        cam.configure(
            cam.create_preview_configuration(
                main={"format": "RGB888", "size": CAMERA_SIZE}
            )
        )
        cam.start()
        return cam

    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_SIZE[0])
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_SIZE[1])
    return cam


def read_frame(cam):
    if USE_PICAM:
        frame = cam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return True, frame

    return cam.read()


def close_camera(cam):
    if USE_PICAM:
        cam.stop()
    else:
        cam.release()


def main():
    cam = open_camera()

    while True:
        ret, frame = read_frame(cam)

        if not ret:
            print("카메라 화면을 읽지 못했습니다.")
            break

        # 90도 회전 보정
        frame = cv2.rotate(frame, ROTATE_MODE)

        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF

        # q 또는 ESC 누르면 종료
        if key == ord("q") or key == 27:
            break

    close_camera(cam)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
