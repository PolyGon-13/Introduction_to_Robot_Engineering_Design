#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Checkerboard camera calibration helper for project3.

Recommended board for the image discussed in chat:
  - printed squares: 10 x 8
  - OpenCV inner corners: --cols 9 --rows 7

Typical workflow on the Raspberry Pi:
  1) Capture calibration images:
       python3 project3/test/checkerboard_calibration.py capture --cols 9 --rows 7

     Press SPACE or s whenever the checkerboard is detected.
     Move/tilt the board between shots. Collect roughly 15-30 images.

     Headless/SSH-friendly auto capture:
       python3 project3/test/checkerboard_calibration.py capture --cols 9 --rows 7 \
         --auto-count 25 --no-window

  2) Calibrate after measuring one printed square size:
       python3 project3/test/checkerboard_calibration.py calibrate --cols 9 --rows 7 \
         --square-size-m 0.025

  3) Verify the undistortion result:
       python3 project3/test/checkerboard_calibration.py verify

Outputs:
  - project3/test/calib_images/checker_*.png
  - project3/test/camera_calibration.json
"""

import argparse
import glob
import json
import os
import sys
import time
from datetime import datetime

import numpy as np

try:
    import cv2
except ImportError:
    print("[ERROR] OpenCV(cv2) is required. Install python3-opencv.")
    sys.exit(1)

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IMAGE_DIR = os.path.join(THIS_DIR, "calib_images")
DEFAULT_OUTPUT = os.path.join(THIS_DIR, "camera_calibration.json")

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 20
DEFAULT_COLS = 9
DEFAULT_ROWS = 7
DEFAULT_SQUARE_SIZE_M = 0.025
DEFAULT_EXPOSURE_VALUE = -1.0


def now_stamp():
    return datetime.now().astimezone().isoformat(timespec="seconds")


def make_pattern_size(args):
    return int(args.cols), int(args.rows)


def json_default(obj):
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    raise TypeError("Object of type {} is not JSON serializable".format(type(obj).__name__))


class Camera:
    def __init__(self, args):
        self.args = args
        self.backend = "unknown"
        self.camera = None

    def open(self):
        if Picamera2 is not None and not self.args.usb_only:
            try:
                picam2 = Picamera2()
                config = picam2.create_preview_configuration(
                    main={"format": "RGB888", "size": (self.args.width, self.args.height)}
                )
                picam2.configure(config)
                try:
                    picam2.set_controls({"ExposureValue": self.args.exposure_value})
                except Exception:
                    pass
                picam2.start()
                time.sleep(1.0)
                self.backend = "picamera2"
                self.camera = picam2
                print("[INFO] Picamera2 camera started")
                return True
            except Exception as exc:
                print("[WARN] Picamera2 failed: {}".format(exc))
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
            return True, self.apply_flip(frame)

        ok, frame = self.camera.read()
        if ok:
            frame = self.apply_flip(frame)
        return ok, frame

    def apply_flip(self, frame):
        if self.args.flip_horizontal and self.args.flip_vertical:
            return cv2.flip(frame, -1)
        if self.args.flip_horizontal:
            return cv2.flip(frame, 1)
        if self.args.flip_vertical:
            return cv2.flip(frame, 0)
        return frame

    def close(self):
        if self.camera is None:
            return
        if self.backend == "picamera2":
            self.camera.stop()
        else:
            self.camera.release()


def find_checkerboard(gray, pattern_size):
    """Return (found, corners). corners are refined float32 image points."""
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
        if found:
            return True, corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners.astype(np.float32)


def draw_capture_overlay(frame, found, saved_count, args):
    status = "FOUND" if found else "not found"
    color = (0, 220, 0) if found else (0, 0, 255)
    lines = [
        "checkerboard: {}  saved: {}".format(status, saved_count),
        "inner corners: {}x{}  SPACE/s: save  q: quit".format(args.cols, args.rows),
    ]
    for i, line in enumerate(lines):
        y = 28 + i * 28
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


def save_frame(out_dir, frame, saved_count):
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, "checker_{}_{:03d}.png".format(ts, saved_count + 1))
    ok = cv2.imwrite(path, frame)
    if not ok:
        raise RuntimeError("failed to write {}".format(path))
    return path


def command_capture(args):
    pattern_size = make_pattern_size(args)
    cam = Camera(args)
    if not cam.open():
        return 1

    print("[INFO] Board inner corners: {} x {}".format(args.cols, args.rows))
    print("[INFO] Output directory: {}".format(args.out_dir))
    if args.auto_count:
        print("[INFO] Auto capture target: {} images".format(args.auto_count))
    else:
        print("[INFO] Press SPACE or s to save a detected board. Press q to quit.")

    saved_count = len(glob.glob(os.path.join(args.out_dir, "checker_*.png")))
    new_saved_count = 0
    last_saved_at = 0.0
    delay = 1.0 / max(1, args.fps)

    try:
        while True:
            started_at = time.time()
            ok, frame = cam.read()
            if not ok:
                print("[WARN] Failed to read frame")
                time.sleep(0.1)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_checkerboard(gray, pattern_size)
            preview = frame.copy()
            if found:
                cv2.drawChessboardCorners(preview, pattern_size, corners, found)

            if args.auto_count and found:
                if time.time() - last_saved_at >= args.min_save_interval:
                    path = save_frame(args.out_dir, frame, saved_count)
                    saved_count += 1
                    new_saved_count += 1
                    last_saved_at = time.time()
                    print("[SAVE] {}".format(path))
                    if new_saved_count >= args.auto_count:
                        print("[INFO] Auto capture complete")
                        break

            if not args.no_window:
                draw_capture_overlay(preview, found, saved_count, args)
                cv2.imshow("checkerboard capture", preview)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key in (ord(" "), ord("s")):
                    if found:
                        path = save_frame(args.out_dir, frame, saved_count)
                        saved_count += 1
                        new_saved_count += 1
                        print("[SAVE] {}".format(path))
                    else:
                        print("[INFO] Checkerboard not detected; not saving")
            elif not args.auto_count:
                if found and time.time() - last_saved_at >= args.min_save_interval:
                    print("[FOUND] board visible; use --auto-count to save without a window")
                    last_saved_at = time.time()

            elapsed = time.time() - started_at
            if elapsed < delay:
                time.sleep(delay - elapsed)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted")
    finally:
        cam.close()
        if not args.no_window:
            cv2.destroyAllWindows()

    print("[INFO] Total saved images in directory: {}".format(saved_count))
    return 0


def collect_calibration_points(args):
    pattern_size = make_pattern_size(args)
    image_paths = sorted(glob.glob(args.images))
    if not image_paths:
        raise RuntimeError("no images matched {}".format(args.images))

    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(args.square_size_m)

    objpoints = []
    imgpoints = []
    used = []
    rejected = []
    image_size = None

    print("[INFO] Loading {} candidate images".format(len(image_paths)))
    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            rejected.append({"path": path, "reason": "read_failed"})
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        this_size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = this_size
        elif this_size != image_size:
            rejected.append({"path": path, "reason": "image_size_mismatch"})
            continue

        found, corners = find_checkerboard(gray, pattern_size)
        if not found:
            rejected.append({"path": path, "reason": "checkerboard_not_found"})
            continue

        objpoints.append(objp.copy())
        imgpoints.append(corners)
        used.append({"path": path})
        print("[USE] {}".format(path))

    if len(objpoints) < args.min_images:
        raise RuntimeError(
            "only {} usable images; need at least {}".format(len(objpoints), args.min_images)
        )

    return objpoints, imgpoints, image_size, used, rejected


def reprojection_errors(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs):
    per_image = []
    total_error_sq = 0.0
    total_points = 0
    for i, objp in enumerate(objpoints):
        projected, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        err = cv2.norm(imgpoints[i], projected, cv2.NORM_L2)
        n = len(projected)
        rms = float(np.sqrt((err * err) / max(1, n)))
        per_image.append(rms)
        total_error_sq += err * err
        total_points += n
    mean_rms = float(np.sqrt(total_error_sq / max(1, total_points)))
    return mean_rms, per_image


def command_calibrate(args):
    objpoints, imgpoints, image_size, used, rejected = collect_calibration_points(args)

    flags = 0
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
        flags=flags,
    )
    mean_error, per_image_errors = reprojection_errors(
        objpoints,
        imgpoints,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    for item, err in zip(used, per_image_errors):
        item["reprojection_rms_px"] = err

    result = {
        "schema": "project3_checkerboard_camera_calibration_v1",
        "created_at": now_stamp(),
        "board": {
            "inner_corners": {"cols": args.cols, "rows": args.rows},
            "square_size_m": args.square_size_m,
            "printed_squares_hint": {
                "cols": args.cols + 1,
                "rows": args.rows + 1,
            },
        },
        "image_size_px": {"width": image_size[0], "height": image_size[1]},
        "usable_image_count": len(used),
        "rejected_image_count": len(rejected),
        "opencv_calibrate_rms_px": float(rms),
        "mean_reprojection_rms_px": mean_error,
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs.ravel(),
        "calibration_flags": flags,
        "used_images": used,
        "rejected_images": rejected,
        "notes": [
            "This file calibrates camera intrinsics and lens distortion.",
            "For robot-center-to-color-center alignment, also compute a floor homography after this.",
            "If project3.py uses a different frame size, scale fx/fy/cx/cy or calibrate at that frame size.",
        ],
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False, default=json_default)

    print("\n[OK] Calibration saved: {}".format(args.output))
    print("[OK] usable images: {} / {}".format(len(used), len(used) + len(rejected)))
    print("[OK] OpenCV RMS: {:.4f}px".format(float(rms)))
    print("[OK] mean reprojection RMS: {:.4f}px".format(mean_error))
    print("[OK] camera_matrix:")
    print(camera_matrix)
    print("[OK] dist_coeffs:")
    print(dist_coeffs.ravel())
    return 0


def load_calibration(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    return data, camera_matrix, dist_coeffs


def command_verify(args):
    _, camera_matrix, dist_coeffs = load_calibration(args.calibration)
    cam = Camera(args)
    if not cam.open():
        return 1

    print("[INFO] Showing raw and undistorted frames. Press q to quit.")
    try:
        while True:
            ok, frame = cam.read()
            if not ok:
                print("[WARN] Failed to read frame")
                time.sleep(0.1)
                continue

            h, w = frame.shape[:2]
            new_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix,
                dist_coeffs,
                (w, h),
                args.alpha,
                (w, h),
            )
            undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_matrix)
            x, y, rw, rh = roi
            if args.crop and rw > 0 and rh > 0:
                undistorted = undistorted[y:y + rh, x:x + rw]

            raw_small = cv2.resize(frame, (w // 2, h // 2))
            und_h, und_w = undistorted.shape[:2]
            und_small = cv2.resize(undistorted, (w // 2, h // 2))
            view = np.hstack((raw_small, und_small))
            cv2.putText(view, "raw", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 0), 2)
            cv2.putText(
                view,
                "undistorted {}x{}".format(und_w, und_h),
                (w // 2 + 12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 220, 0),
                2,
            )
            cv2.imshow("calibration verify", view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()
    return 0


def add_camera_args(parser):
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Frame width")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Frame height")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Target FPS")
    parser.add_argument("--camera-index", type=int, default=0, help="USB camera index")
    parser.add_argument("--usb-only", action="store_true", help="Skip Picamera2")
    parser.add_argument(
        "--exposure-value",
        type=float,
        default=DEFAULT_EXPOSURE_VALUE,
        help="Picamera2 ExposureValue control",
    )
    parser.add_argument(
        "--no-flip-horizontal",
        dest="flip_horizontal",
        action="store_false",
        help="Disable horizontal flip",
    )
    parser.add_argument(
        "--no-flip-vertical",
        dest="flip_vertical",
        action="store_false",
        help="Disable vertical flip",
    )
    parser.set_defaults(flip_horizontal=True, flip_vertical=True)


def add_board_args(parser):
    parser.add_argument("--cols", type=int, default=DEFAULT_COLS, help="Inner corner columns")
    parser.add_argument("--rows", type=int, default=DEFAULT_ROWS, help="Inner corner rows")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture and calibrate a checkerboard camera model for project3."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    capture = sub.add_parser("capture", help="Capture checkerboard images")
    add_camera_args(capture)
    add_board_args(capture)
    capture.add_argument("--out-dir", default=DEFAULT_IMAGE_DIR, help="Image output directory")
    capture.add_argument("--auto-count", type=int, default=0, help="Auto-save N detected frames")
    capture.add_argument(
        "--min-save-interval",
        type=float,
        default=0.7,
        help="Minimum seconds between saved frames",
    )
    capture.add_argument("--no-window", action="store_true", help="Do not open cv2.imshow window")
    capture.set_defaults(func=command_capture)

    calibrate = sub.add_parser("calibrate", help="Compute camera matrix and distortion")
    add_board_args(calibrate)
    calibrate.add_argument(
        "--square-size-m",
        type=float,
        default=DEFAULT_SQUARE_SIZE_M,
        help="Measured printed square size in meters",
    )
    calibrate.add_argument(
        "--images",
        default=os.path.join(DEFAULT_IMAGE_DIR, "checker_*.png"),
        help="Glob for calibration images",
    )
    calibrate.add_argument("--output", default=DEFAULT_OUTPUT, help="Calibration JSON output path")
    calibrate.add_argument("--min-images", type=int, default=10, help="Minimum usable images")
    calibrate.set_defaults(func=command_calibrate)

    verify = sub.add_parser("verify", help="Show raw vs undistorted camera stream")
    add_camera_args(verify)
    verify.add_argument("--calibration", default=DEFAULT_OUTPUT, help="Calibration JSON path")
    verify.add_argument("--alpha", type=float, default=0.0, help="Undistort free-scaling alpha")
    verify.add_argument("--crop", action="store_true", help="Crop to valid undistorted ROI")
    verify.set_defaults(func=command_verify)

    return parser.parse_args()


def main():
    args = parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
