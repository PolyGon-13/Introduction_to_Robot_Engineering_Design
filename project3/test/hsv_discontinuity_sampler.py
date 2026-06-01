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
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 20


class Camera:
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


def circular_mean_hue(hue_values):
    if hue_values.size == 0:
        return 0.0

    angles = hue_values.astype(np.float32) * (2.0 * np.pi / 180.0)
    sin_mean = float(np.mean(np.sin(angles)))
    cos_mean = float(np.mean(np.cos(angles)))
    angle = np.arctan2(sin_mean, cos_mean)
    if angle < 0:
        angle += 2.0 * np.pi
    return float(angle * 180.0 / (2.0 * np.pi))


def color_gradient_mask(frame, threshold, blur_size):
    if blur_size > 1:
        frame = cv2.GaussianBlur(frame, (blur_size, blur_size), 0)

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
    grad_x = cv2.Sobel(lab, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(lab, cv2.CV_32F, 0, 1, ksize=3)
    grad = np.sqrt(np.sum(grad_x * grad_x + grad_y * grad_y, axis=2))
    grad_u8 = cv2.normalize(grad, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    _, edge_mask = cv2.threshold(grad_u8, threshold, 255, cv2.THRESH_BINARY)
    return edge_mask, grad_u8


def build_region_mask(frame, args):
    edge_mask, grad_u8 = color_gradient_mask(frame, args.edge_threshold, args.blur_size)

    edge_kernel = np.ones((args.edge_dilate, args.edge_dilate), np.uint8)
    edge_mask = cv2.morphologyEx(edge_mask, cv2.MORPH_CLOSE, edge_kernel)
    edge_mask = cv2.dilate(edge_mask, edge_kernel, iterations=1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    saturation_mask = cv2.inRange(
        hsv,
        np.array([0, args.min_saturation, args.min_value], dtype=np.uint8),
        np.array([179, 255, 255], dtype=np.uint8),
    )

    candidate_mask = cv2.bitwise_and(cv2.bitwise_not(edge_mask), saturation_mask)

    region_kernel = np.ones((args.region_close, args.region_close), np.uint8)
    candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_OPEN, region_kernel)
    candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_CLOSE, region_kernel)
    return candidate_mask, edge_mask, grad_u8, hsv


def contour_score(contour):
    x, y, w, h = cv2.boundingRect(contour)
    return y, x, -cv2.contourArea(contour), w * h


def analyze_regions(frame, args):
    region_mask, edge_mask, grad_u8, hsv = build_region_mask(frame, args)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    contours, _ = cv2.findContours(region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=contour_score)

    regions = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < args.min_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w < args.min_width or h < args.min_height:
            continue

        mask = np.zeros(region_mask.shape, dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
        pixels = mask > 0
        if int(np.count_nonzero(pixels)) < args.min_area:
            continue

        hsv_pixels = hsv[pixels]
        rgb_pixels = rgb[pixels]

        mean_h = circular_mean_hue(hsv_pixels[:, 0])
        mean_s = float(np.mean(hsv_pixels[:, 1]))
        mean_v = float(np.mean(hsv_pixels[:, 2]))
        mean_rgb = np.mean(rgb_pixels, axis=0)
        std_hsv = np.std(hsv_pixels.astype(np.float32), axis=0)

        moments = cv2.moments(contour)
        if moments["m00"] != 0:
            cx = int(round(moments["m10"] / moments["m00"]))
            cy = int(round(moments["m01"] / moments["m00"]))
        else:
            cx = x + w // 2
            cy = y + h // 2

        regions.append(
            {
                "id": len(regions) + 1,
                "bbox": (x, y, w, h),
                "center": (cx, cy),
                "area": int(area),
                "mean_hsv": (mean_h, mean_s, mean_v),
                "mean_rgb": tuple(float(v) for v in mean_rgb),
                "std_hsv": tuple(float(v) for v in std_hsv),
                "contour": contour,
            }
        )

        if len(regions) >= args.max_regions:
            break

    return regions, region_mask, edge_mask, grad_u8


def print_regions(regions, backend, frame_count):
    stamp = time.strftime("%H:%M:%S")
    print(f"[{stamp}] frame={frame_count} backend={backend} regions={len(regions)}")

    if not regions:
        print("  no region")
        return

    for region in regions:
        x, y, w, h = region["bbox"]
        cx, cy = region["center"]
        h_mean, s_mean, v_mean = region["mean_hsv"]
        r_mean, g_mean, b_mean = region["mean_rgb"]
        h_std, s_std, v_std = region["std_hsv"]

        print(
            "  #{id:02d} center=({cx:3d},{cy:3d}) bbox=({x:3d},{y:3d},{w:3d},{height:3d}) "
            "area={area:5d} HSV_mean=({hue:6.1f},{s:6.1f},{v:6.1f}) "
            "RGB_mean=({r:6.1f},{g:6.1f},{b:6.1f}) HSV_std=({hs:5.1f},{ss:5.1f},{vs:5.1f})".format(
                id=region["id"],
                cx=cx,
                cy=cy,
                x=x,
                y=y,
                w=w,
                height=h,
                area=region["area"],
                hue=h_mean,
                s=s_mean,
                v=v_mean,
                r=r_mean,
                g=g_mean,
                b=b_mean,
                hs=h_std,
                ss=s_std,
                vs=v_std,
            )
        )


def draw_overlay(frame, regions, args):
    view = frame.copy()
    for region in regions:
        x, y, w, h = region["bbox"]
        cx, cy = region["center"]
        h_mean, s_mean, v_mean = region["mean_hsv"]
        cv2.rectangle(view, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.drawContours(view, [region["contour"]], -1, (0, 180, 255), 1)
        cv2.circle(view, (cx, cy), 4, (0, 255, 0), -1)
        label = f"#{region['id']} H={h_mean:.0f} S={s_mean:.0f} V={v_mean:.0f}"
        cv2.putText(
            view,
            label,
            (x, max(18, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    info = (
        f"edge={args.edge_threshold} minS={args.min_saturation} "
        f"minV={args.min_value} minArea={args.min_area}"
    )
    cv2.putText(view, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    return view


def show_debug_windows(frame, regions, region_mask, edge_mask, grad_u8, args):
    cv2.imshow("HSV discontinuity sampler", draw_overlay(frame, regions, args))

    if args.show_masks:
        cv2.imshow("candidate regions", region_mask)
        cv2.imshow("color edges", edge_mask)
        cv2.imshow("color gradient", grad_u8)

    return cv2.waitKey(1) & 0xFF


def parse_args():
    parser = argparse.ArgumentParser(
        description="Print mean HSV/RGB values for camera regions split by abrupt color changes."
    )
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Frame width")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Frame height")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Camera FPS request")
    parser.add_argument("--camera-index", type=int, default=0, help="USB camera index")
    parser.add_argument("--usb-only", action="store_true", help="Skip Picamera2")
    parser.add_argument("--exposure-value", type=float, default=-1.0, help="Picamera2 ExposureValue")
    parser.add_argument("--edge-threshold", type=int, default=45, help="0-255 Lab gradient threshold")
    parser.add_argument("--min-saturation", type=int, default=45, help="Ignore low-saturation pixels")
    parser.add_argument("--min-value", type=int, default=40, help="Ignore dark pixels")
    parser.add_argument("--min-area", type=int, default=500, help="Smallest region area in pixels")
    parser.add_argument("--min-width", type=int, default=8, help="Smallest region bbox width")
    parser.add_argument("--min-height", type=int, default=8, help="Smallest region bbox height")
    parser.add_argument("--max-regions", type=int, default=12, help="Maximum printed regions per frame")
    parser.add_argument("--blur-size", type=int, default=5, choices=(1, 3, 5, 7), help="Noise blur kernel")
    parser.add_argument("--edge-dilate", type=int, default=3, choices=(1, 3, 5, 7), help="Edge thickness")
    parser.add_argument("--region-close", type=int, default=5, choices=(1, 3, 5, 7), help="Region cleanup kernel")
    parser.add_argument("--print-interval", type=float, default=1.0, help="Seconds between console prints")
    parser.add_argument("--no-window", action="store_true", help="Do not open cv2 preview windows")
    parser.add_argument("--show-masks", action="store_true", help="Show debug masks")
    parser.add_argument("--no-flip-horizontal", dest="flip_horizontal", action="store_false")
    parser.add_argument("--no-flip-vertical", dest="flip_vertical", action="store_false")
    parser.set_defaults(flip_horizontal=True, flip_vertical=True)
    return parser.parse_args()


def main():
    args = parse_args()
    camera = Camera(args)
    if not camera.open():
        return 1

    frame_delay = 1.0 / max(1, args.fps)
    last_print = 0.0
    frame_count = 0

    try:
        while True:
            started_at = time.time()
            ok, frame = camera.read()
            if not ok:
                print("[WARN] camera read failed")
                time.sleep(0.1)
                continue

            frame_count += 1
            regions, region_mask, edge_mask, grad_u8 = analyze_regions(frame, args)

            now = time.time()
            if now - last_print >= args.print_interval:
                print_regions(regions, camera.backend, frame_count)
                last_print = now

            if not args.no_window:
                key = show_debug_windows(frame, regions, region_mask, edge_mask, grad_u8, args)
                if key in (ord("q"), 27):
                    break
                if key == ord("p"):
                    print_regions(regions, camera.backend, frame_count)

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
