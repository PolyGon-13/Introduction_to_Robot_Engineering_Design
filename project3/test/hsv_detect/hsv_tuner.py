#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None


DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 400
DEFAULT_FPS = 20

MAIN_WINDOW = "HSV Tuner"


class Camera:
    """Picamera2 우선, 실패 시 USB/OpenCV 카메라로 폴백."""

    def __init__(self, args):
        self.args = args
        self.backend = "unknown"
        self.camera = None

    def open(self):
        if Picamera2 is not None and not self.args.usb_only:
            try:
                cam = Picamera2()
                config = cam.create_preview_configuration(
                    main={"format": "RGB888", "size": (self.args.width, self.args.height)}
                )
                cam.configure(config)
                try:
                    cam.set_controls({"ExposureValue": self.args.exposure_value})
                except Exception:
                    pass
                cam.start()
                time.sleep(1.0)
                self.backend = "picamera2"
                self.camera = cam
                print("[INFO] Picamera2 camera started")
                return True
            except Exception as exc:
                print(f"[WARN] Picamera2 failed: {exc}")
                print("[WARN] Falling back to USB/OpenCV camera")

        cap = cv2.VideoCapture(self.args.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
        cap.set(cv2.CAP_PROP_FPS, self.args.fps)

        if not cap.isOpened():
            print("[ERROR] Camera open failed")
            return False

        self.backend = "usb"
        self.camera = cap
        print("[INFO] USB/OpenCV camera started")
        return True

    def read(self):
        if self.backend == "picamera2":
            frame_rgb = self.camera.capture_array()
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            return True, apply_flip(frame, self.args)

        ok, frame = self.camera.read()
        if ok:
            frame = apply_flip(frame, self.args)
        return ok, frame

    def close(self):
        if self.camera is None:
            return
        if self.backend == "picamera2":
            self.camera.stop()
        else:
            self.camera.release()


def apply_flip(frame, args):
    if args.flip_horizontal and args.flip_vertical:
        return cv2.flip(frame, -1)
    if args.flip_horizontal:
        return cv2.flip(frame, 1)
    if args.flip_vertical:
        return cv2.flip(frame, 0)
    return frame


def _noop(_value):
    pass


def create_trackbars(args):
    """메인 창에 H/S/V 최소·최대 트랙바 6개를 만든다."""
    cv2.createTrackbar("H min", MAIN_WINDOW, args.h_min, 179, _noop)
    cv2.createTrackbar("H max", MAIN_WINDOW, args.h_max, 179, _noop)
    cv2.createTrackbar("S min", MAIN_WINDOW, args.s_min, 255, _noop)
    cv2.createTrackbar("S max", MAIN_WINDOW, args.s_max, 255, _noop)
    cv2.createTrackbar("V min", MAIN_WINDOW, args.v_min, 255, _noop)
    cv2.createTrackbar("V max", MAIN_WINDOW, args.v_max, 255, _noop)


def read_trackbars():
    lower = np.array(
        [
            cv2.getTrackbarPos("H min", MAIN_WINDOW),
            cv2.getTrackbarPos("S min", MAIN_WINDOW),
            cv2.getTrackbarPos("V min", MAIN_WINDOW),
        ],
        dtype=np.uint8,
    )
    upper = np.array(
        [
            cv2.getTrackbarPos("H max", MAIN_WINDOW),
            cv2.getTrackbarPos("S max", MAIN_WINDOW),
            cv2.getTrackbarPos("V max", MAIN_WINDOW),
        ],
        dtype=np.uint8,
    )
    return lower, upper


def build_mask(frame, lower, upper, args):
    """HSV inRange로 마스크 생성 후 노이즈 정리.

    Hue 최소>최대인 경우 빨강처럼 0/179 경계를 감싸는 범위로 처리한다.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if lower[0] <= upper[0]:
        mask = cv2.inRange(hsv, lower, upper)
    else:
        lower_a = np.array([0, lower[1], lower[2]], dtype=np.uint8)
        upper_a = np.array([upper[0], upper[1], upper[2]], dtype=np.uint8)
        lower_b = np.array([lower[0], lower[1], lower[2]], dtype=np.uint8)
        upper_b = np.array([179, upper[1], upper[2]], dtype=np.uint8)
        mask = cv2.bitwise_or(cv2.inRange(hsv, lower_a, upper_a),
                              cv2.inRange(hsv, lower_b, upper_b))

    if args.morph_kernel > 1:
        kernel = np.ones((args.morph_kernel, args.morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def draw_region_overlay(frame, mask, args):
    """인식된 영역만 정확한 윤곽선 + 반투명 채움으로 표시."""
    view = frame.copy()

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= args.min_area]

    if contours:
        fill = view.copy()
        cv2.drawContours(fill, contours, -1, (0, 255, 0), thickness=cv2.FILLED)
        view = cv2.addWeighted(fill, 0.35, view, 0.65, 0)
        cv2.drawContours(view, contours, -1, (0, 255, 0), 2)

    cv2.putText(view, f"regions={len(contours)}", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
    return view


def compose(view, mask):
    """좌: 오버레이 영상 / 우: 흑백 마스크 를 가로로 합친다."""
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.putText(mask_bgr, "mask", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
    return np.hstack([view, mask_bgr])


def parse_args():
    parser = argparse.ArgumentParser(description="Realtime HSV range tuner with live mask preview.")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--camera-index", type=int, default=0, help="USB camera index")
    parser.add_argument("--usb-only", action="store_true", help="Skip Picamera2")
    parser.add_argument("--exposure-value", type=float, default=-1.0, help="Picamera2 ExposureValue")
    parser.add_argument("--min-area", type=int, default=300, help="Smallest region area to outline")
    parser.add_argument("--morph-kernel", type=int, default=5, choices=(1, 3, 5, 7),
                        help="Mask cleanup kernel size")
    parser.add_argument("--h-min", type=int, default=0)
    parser.add_argument("--h-max", type=int, default=179)
    parser.add_argument("--s-min", type=int, default=80)
    parser.add_argument("--s-max", type=int, default=255)
    parser.add_argument("--v-min", type=int, default=80)
    parser.add_argument("--v-max", type=int, default=255)
    parser.add_argument("--no-flip-horizontal", dest="flip_horizontal", action="store_false")
    parser.add_argument("--no-flip-vertical", dest="flip_vertical", action="store_false")
    parser.set_defaults(flip_horizontal=True, flip_vertical=True)
    return parser.parse_args()


def main():
    args = parse_args()
    camera = Camera(args)
    if not camera.open():
        return 1

    cv2.namedWindow(MAIN_WINDOW, cv2.WINDOW_NORMAL)
    create_trackbars(args)
    cv2.moveWindow(MAIN_WINDOW, 40, 20)

    frame_delay = 1.0 / max(1, args.fps)

    try:
        while True:
            started_at = time.time()
            ok, frame = camera.read()
            if not ok:
                print("[WARN] camera read failed")
                time.sleep(0.1)
                continue

            lower, upper = read_trackbars()
            mask = build_mask(frame, lower, upper, args)
            view = draw_region_overlay(frame, mask, args)
            cv2.imshow(MAIN_WINDOW, compose(view, mask))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("p"):
                print(f"[HSV] lower=({lower[0]},{lower[1]},{lower[2]}) "
                      f"upper=({upper[0]},{upper[1]},{upper[2]})")

            elapsed = time.time() - started_at
            if elapsed < frame_delay:
                time.sleep(frame_delay - elapsed)

    except KeyboardInterrupt:
        print("\n[INFO] stopped by user")
    finally:
        camera.close()
        cv2.destroyAllWindows()
        print("[INFO] camera closed")

    return 0


if __name__ == "__main__":
    sys.exit(main())
