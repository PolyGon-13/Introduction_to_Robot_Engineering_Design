#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Merged: sihyun/test/main.py + test/restore_color_area/s_restore_square.py
# Fix applied: Problem 1 only — single RPLidarC1 instance (no second lidar open)
#

import math
import threading
import time
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import serial

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from camera import (
    BOX_COLORS,
    CAMERA_ROTATION,
    MIN_AREA,
    bottom_center_error,
    close_camera,
    detect,
    draw,
    follow_cmd,
    open_camera,
    pick,
    read_frame,
    x_center_error,
)
from lidar import (
    ANG_MAX,
    ANG_MIN,
    COLOR_TO_LIDAR_DEG,
    FREE_D,
    FRONT_LOG_ZONE,
    GRID,
    LEFT_ZONE,
    RPLidarC1,
    RIGHT_ZONE,
    SIDE_CLEAR_D,
    avoid_cmd,
    front_ranges,
    lidar_zone_distances,
    obstacle_detected,
)


# ===========================================================================
# HSV color following settings  (from sihyun/test/main.py)
# ===========================================================================

TARGET_SEQUENCE = ("RED", "YELLOW", "BLUE")
SHOW_WINDOW = True
BOTTOM_LOST_RATIO = 0.90
COLOR_SWITCH_PAUSE_MS = 1000
COLOR_FORWARD_CENTER_RATIO = 0.95
COLOR_EXIT_CENTER_ERR = 0.15

# ===========================================================================
# Motor settings  (from sihyun/test/main.py)
# ===========================================================================

ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600
DRIVE_V = 0.25
V_STEP = 0.04
FOLLOW_W_STEP = 0.25
AVOID_W_STEP = 0.20
LOOP_DT = 0.05
SEARCH_MAX_W = 1.0
SWITCH_SEARCH_W = 1.0
ODOM_LOG_INTERVAL = 0.5
WHEEL_R = 0.034
ENC_PPR = 1012.0
ENC_COUNTS_PER_M = ENC_PPR / (2.0 * np.pi * WHEEL_R)
POST_COLOR_FORWARD_M = 0.05
TURN_360_WHEEL_BASE_M = 0.18
TURN_360_COUNTS = np.pi * TURN_360_WHEEL_BASE_M * ENC_COUNTS_PER_M
AVOID_STOP_D = 0.15

# ===========================================================================
# Restore square settings  (from s_restore_square.py)
# ===========================================================================

DEFAULT_HOMOGRAPHY_FILE    = THIS_DIR / "ground_homography.npz"
DEFAULT_CAMERA_MATRIX_FILE = THIS_DIR / "camera_matrix.npy"
DEFAULT_DIST_COEFFS_FILE   = THIS_DIR / "dist_coeffs.npy"

CALIB_WIDTH  = 640.0
CALIB_HEIGHT = 480.0

SQUARE_SIZE_M           = 0.30
MIN_GROUND_POINTS       = 3
MIN_GROUND_SPAN_M       = 0.04
MAX_GROUND_SPAN_M       = 0.60
COMPLETE_FILL_RATIO     = 0.65
COMPLETE_SIZE_MIN_RATIO = 0.70
COMPLETE_SIZE_MAX_RATIO = 1.45
MAX_PROJECTED_AREA_RATIO = 2.0
CENTER_MARGIN_RATIO     = 0.15
BORDER_TOUCH_PX         = 18
OUT_OF_FRAME_PX         = 4
LIDAR_GUIDE_HALF_DEG    = 12.0
LIDAR_MAX_AGE_S         = 0.5
MIN_RESTORE_SHIFT_M     = 0.03
MAX_RESTORE_SHIFT_M     = 0.23

SELECTED_COLOR = (255, 255, 255)
CENTER_COLOR   = (0, 255, 0)

RESTORE_ARGS = SimpleNamespace(
    square_size             = SQUARE_SIZE_M,
    min_area                = MIN_AREA,
    min_ground_span         = MIN_GROUND_SPAN_M,
    max_ground_span         = MAX_GROUND_SPAN_M,
    complete_fill_ratio     = COMPLETE_FILL_RATIO,
    complete_size_min_ratio = COMPLETE_SIZE_MIN_RATIO,
    complete_size_max_ratio = COMPLETE_SIZE_MAX_RATIO,
    max_projected_area_ratio = MAX_PROJECTED_AREA_RATIO,
    center_margin           = CENTER_MARGIN_RATIO,
    border_touch_px         = float(BORDER_TOUCH_PX),
    out_of_frame_px         = float(OUT_OF_FRAME_PX),
    lidar_obstacle_d        = 0.60,
    lidar_half_deg          = LIDAR_GUIDE_HALF_DEG,
    lidar_max_age           = LIDAR_MAX_AGE_S,
    min_restore_shift       = MIN_RESTORE_SHIFT_M,
    max_restore_shift       = MAX_RESTORE_SHIFT_M,
    smooth                  = 0.65,
    homography_file         = str(DEFAULT_HOMOGRAPHY_FILE),
    camera_matrix_file      = str(DEFAULT_CAMERA_MATRIX_FILE),
    dist_coeffs_file        = str(DEFAULT_DIST_COEFFS_FILE),
)


# ===========================================================================
# Shared helpers
# ===========================================================================

def clamp(value, low, high):
    return float(max(low, min(value, high)))


def rate_limit(prev, target, step):
    return prev + clamp(target - prev, -step, step)


def millis():
    return int(time.monotonic() * 1000)


def wait_ms(duration_ms):
    deadline = millis() + int(duration_ms)
    while millis() < deadline:
        time.sleep(0.001)


# ===========================================================================
# Motor  (from sihyun/test/main.py)
# ===========================================================================

class Motor:
    def __init__(self):
        self.ser = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
        self.lock       = threading.Lock()
        self.write_lock = threading.Lock()
        self.running    = True
        self.odom_l     = 0
        self.odom_r     = 0
        self.odom_ms    = 0
        self.odom_time  = 0.0
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        while self.running:
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("O,"):
                    parts = line.split(",")
                    if len(parts) != 4:
                        continue

                    enc_l      = int(parts[1])
                    enc_r      = int(parts[2])
                    arduino_ms = int(parts[3])

                    with self.lock:
                        self.odom_l    = enc_l
                        self.odom_r    = enc_r
                        self.odom_ms   = arduino_ms
                        self.odom_time = time.time()
                elif line:
                    print(f"[ARDUINO] {line}")
            except (ValueError, serial.SerialException, OSError):
                time.sleep(0.05)

    def vw(self, v, w):
        with self.write_lock:
            self.ser.write(f"V{v:.3f},{w:.3f}\n".encode("ascii"))

    def stop(self):
        with self.write_lock:
            self.ser.write(b"S\n")

    def reset_encoders(self):
        with self.write_lock:
            self.ser.write(b"R\n")

    def get_odom(self):
        with self.lock:
            return self.odom_l, self.odom_r, self.odom_ms, self.odom_time

    def close(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join(timeout=0.5)
        if self.ser.is_open:
            self.ser.close()


# ===========================================================================
# Restore square classes  (from s_restore_square.py)
# ===========================================================================

class Undistorter:
    def __init__(self, camera_matrix, dist_coeffs, calib_width, calib_height):
        self.base_k  = np.asarray(camera_matrix, dtype=np.float64)
        self.dist    = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        self.calib_w = float(calib_width)
        self.calib_h = float(calib_height)
        self._maps   = {}

    def _maps_for(self, raw_w, raw_h):
        key = (raw_w, raw_h)
        if key not in self._maps:
            sx = raw_w / self.calib_w
            sy = raw_h / self.calib_h
            k = self.base_k.copy()
            k[0, 0] *= sx; k[0, 2] *= sx
            k[1, 1] *= sy; k[1, 2] *= sy
            map1, map2 = cv2.initUndistortRectifyMap(
                k, self.dist, None, k, (raw_w, raw_h), cv2.CV_16SC2
            )
            self._maps[key] = (map1, map2)
        return self._maps[key]

    def apply(self, frame):
        raw_image = _unrotate_to_raw_image(frame)
        raw_h, raw_w = raw_image.shape[:2]
        map1, map2 = self._maps_for(raw_w, raw_h)
        undistorted = cv2.remap(raw_image, map1, map2, cv2.INTER_LINEAR)
        return _rotate_to_display_image(undistorted)


class GroundProjector:
    def __init__(self, homography):
        self.homography         = np.asarray(homography, dtype=np.float32)
        self.inverse_homography = np.linalg.inv(self.homography).astype(np.float32)

    def raw_pixels_to_ground(self, raw_points):
        pts = np.asarray(raw_points, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, self.homography).reshape(-1, 2).astype(np.float32)

    def ground_to_raw_pixels(self, ground_points):
        pts = np.asarray(ground_points, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, self.inverse_homography).reshape(-1, 2).astype(np.float32)


# ===========================================================================
# Restore square functions  (from s_restore_square.py)
# ===========================================================================

def _unrotate_to_raw_image(frame):
    if CAMERA_ROTATION == cv2.ROTATE_90_COUNTERCLOCKWISE:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if CAMERA_ROTATION == cv2.ROTATE_90_CLOCKWISE:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if CAMERA_ROTATION == cv2.ROTATE_180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    return frame


def _rotate_to_display_image(raw_image):
    if CAMERA_ROTATION is None:
        return raw_image
    return cv2.rotate(raw_image, CAMERA_ROTATION)


def display_to_raw_points(points, frame_shape):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    display_h, display_w = frame_shape[:2]

    if CAMERA_ROTATION == cv2.ROTATE_90_COUNTERCLOCKWISE:
        raw_w = float(display_h)
        raw_x = raw_w - 1.0 - pts[:, 1]
        raw_y = pts[:, 0]
        return np.column_stack((raw_x, raw_y)).astype(np.float32)

    if CAMERA_ROTATION == cv2.ROTATE_90_CLOCKWISE:
        raw_h = float(display_w)
        raw_x = pts[:, 1]
        raw_y = raw_h - 1.0 - pts[:, 0]
        return np.column_stack((raw_x, raw_y)).astype(np.float32)

    if CAMERA_ROTATION == cv2.ROTATE_180:
        raw_x = float(display_w) - 1.0 - pts[:, 0]
        raw_y = float(display_h) - 1.0 - pts[:, 1]
        return np.column_stack((raw_x, raw_y)).astype(np.float32)

    return pts


def raw_to_display_points(points, frame_shape):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    display_h, display_w = frame_shape[:2]

    if CAMERA_ROTATION == cv2.ROTATE_90_COUNTERCLOCKWISE:
        raw_w = float(display_h)
        x = pts[:, 1]
        y = raw_w - 1.0 - pts[:, 0]
        return np.column_stack((x, y)).astype(np.float32)

    if CAMERA_ROTATION == cv2.ROTATE_90_CLOCKWISE:
        raw_h = float(display_w)
        x = raw_h - 1.0 - pts[:, 1]
        y = pts[:, 0]
        return np.column_stack((x, y)).astype(np.float32)

    if CAMERA_ROTATION == cv2.ROTATE_180:
        x = float(display_w) - 1.0 - pts[:, 0]
        y = float(display_h) - 1.0 - pts[:, 1]
        return np.column_stack((x, y)).astype(np.float32)

    return pts


def load_tuned_intrinsics(args):
    camera_matrix = None
    dist_coeffs   = None
    matrix_path = Path(args.camera_matrix_file).expanduser()
    dist_path   = Path(args.dist_coeffs_file).expanduser()
    if matrix_path.exists():
        camera_matrix = np.load(str(matrix_path)).astype(np.float64)
    if dist_path.exists():
        dist_coeffs = np.load(str(dist_path)).astype(np.float64).reshape(-1)
    return camera_matrix, dist_coeffs


def load_homography(path):
    data = np.load(str(path))
    return np.asarray(data["homography"], dtype=np.float32)


def contour_points(target):
    contour = target[9]
    hull = cv2.convexHull(contour)
    return hull.reshape(-1, 2).astype(np.float32)


def normalize(vec):
    norm = float(np.linalg.norm(vec))
    if norm < 1.0e-6:
        return None
    return (vec / norm).astype(np.float32)


def estimate_axes(ground_points):
    rect = cv2.minAreaRect(ground_points.reshape(-1, 1, 2).astype(np.float32))
    box  = cv2.boxPoints(rect).astype(np.float32)
    edges   = [box[(i + 1) % 4] - box[i] for i in range(4)]
    lengths = [float(np.linalg.norm(e)) for e in edges]
    if max(lengths, default=0.0) < 1.0e-6:
        return None, None
    idx = int(np.argmax(lengths))
    e1  = normalize(edges[idx])
    e2  = normalize(edges[(idx + 1) % 4])
    if e1 is None:
        return None, None
    if e2 is None or abs(float(np.dot(e1, e2))) > 0.35:
        e2 = np.array([-e1[1], e1[0]], dtype=np.float32)
    return e1, e2


def axis_intervals(values, square_size):
    v_min = float(np.min(values))
    v_max = float(np.max(values))
    span  = v_max - v_min
    if span >= square_size:
        center = 0.5 * (v_min + v_max)
        return [(center - 0.5 * square_size, center + 0.5 * square_size)]
    candidates = [(v_min, v_min + square_size), (v_max - square_size, v_max)]
    unique = []
    for item in candidates:
        if not any(abs(item[0] - p[0]) < 1.0e-4 and abs(item[1] - p[1]) < 1.0e-4 for p in unique):
            unique.append(item)
    return unique


def make_square(a_min, a_max, b_min, b_max, e1, e2):
    local = np.array(
        [[a_min, b_min], [a_max, b_min], [a_max, b_max], [a_min, b_max]],
        dtype=np.float32,
    )
    corners = local[:, 0:1] * e1[None, :] + local[:, 1:2] * e2[None, :]
    center  = 0.5 * (a_min + a_max) * e1 + 0.5 * (b_min + b_max) * e2
    return {"corners": corners.astype(np.float32), "center": center.astype(np.float32)}


def polygon_area(points):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    if len(pts) < 3:
        return 0.0
    return abs(float(cv2.contourArea(pts)))


def contour_fill_ratio(target):
    rect = cv2.minAreaRect(target[9])
    width, height = rect[1]
    rect_area = float(width * height)
    if rect_area <= 1.0:
        return 0.0
    return float(target[5]) / rect_area


def ground_polygon_dimensions(points):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    if len(pts) < 3:
        return None
    rect = cv2.minAreaRect(pts)
    width, height = rect[1]
    dims = sorted((float(width), float(height)))
    if dims[0] <= 1.0e-6 or dims[1] <= 1.0e-6:
        return None
    return dims[0], dims[1]


def target_display_polygon(target):
    contour = target[9]
    approx  = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
    if len(approx) == 4:
        return approx.reshape(-1, 2).astype(np.float32)
    rect = cv2.minAreaRect(contour)
    return cv2.boxPoints(rect).astype(np.float32)


def target_size_measurement(frame, target, projector):
    display_polygon = target_display_polygon(target)
    raw_polygon     = display_to_raw_points(display_polygon, frame.shape)
    ground_polygon  = projector.raw_pixels_to_ground(raw_polygon)
    return {
        "display_polygon": display_polygon,
        "fill_ratio":      contour_fill_ratio(target),
        "ground_size":     ground_polygon_dimensions(ground_polygon),
    }


def diagonal_center(points):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(pts) != 4:
        return np.mean(pts, axis=0)
    a = np.column_stack((pts[2] - pts[0], pts[1] - pts[3]))
    b = pts[1] - pts[0]
    try:
        t, _ = np.linalg.solve(a, b)
        return pts[0] + t * (pts[2] - pts[0])
    except np.linalg.LinAlgError:
        return np.mean(pts, axis=0)


def complete_observed_candidate(frame, target, projector, args):
    measurement = target_size_measurement(frame, target, projector)
    fill_ratio  = measurement["fill_ratio"]
    if fill_ratio < args.complete_fill_ratio:
        return None
    display_polygon = measurement["display_polygon"]
    dimensions = measurement["ground_size"]
    if dimensions is None:
        return None
    short_m, long_m = dimensions
    expected = args.square_size
    if (
        short_m < expected * args.complete_size_min_ratio
        or long_m  < expected * args.complete_size_min_ratio
        or short_m > expected * args.complete_size_max_ratio
        or long_m  > expected * args.complete_size_max_ratio
    ):
        return None
    display_center = diagonal_center(display_polygon)
    raw_center     = display_to_raw_points(display_center[None, :], frame.shape)
    ground_center  = projector.raw_pixels_to_ground(raw_center)
    if len(ground_center) == 0:
        return None
    return {
        "kind":            "image",
        "display_polygon": display_polygon,
        "display_center":  display_center.astype(np.float32),
        "center":          ground_center[0],
        "ground_size":     (short_m, long_m),
    }


def ground_shape_is_plausible(ground_points, args):
    x_span = float(np.max(ground_points[:, 0]) - np.min(ground_points[:, 0]))
    y_span = float(np.max(ground_points[:, 1]) - np.min(ground_points[:, 1]))
    max_span = max(x_span, y_span)
    return args.min_ground_span <= max_span <= args.max_ground_span


def ground_bearing_deg(point):
    x, y = float(point[0]), float(point[1])
    if x <= 1.0e-6:
        return None
    return clamp(math.degrees(math.atan2(y, x)), float(GRID[0]), float(GRID[-1]))


def lidar_min_distance_at(ranges, angle_deg, half_deg):
    if ranges is None or angle_deg is None:
        return None
    zone = np.abs(GRID - float(angle_deg)) <= float(half_deg)
    if not np.any(zone):
        return None
    values = ranges[zone]
    if len(values) == 0:
        return None
    return float(np.min(values))


def add_lidar_support(candidate, ranges, observed_center, args):
    restore_shift = float(np.linalg.norm(candidate["center"] - observed_center))
    if restore_shift < args.min_restore_shift:
        return None
    if restore_shift > args.max_restore_shift:
        return None
    angle_deg  = ground_bearing_deg(candidate["center"])
    obstacle_d = lidar_min_distance_at(ranges, angle_deg, args.lidar_half_deg)
    if obstacle_d is None or obstacle_d > args.lidar_obstacle_d:
        return None
    supported = dict(candidate)
    supported["lidar_angle"]    = angle_deg
    supported["lidar_distance"] = obstacle_d
    supported["lidar_score"]    = (
        (args.lidar_obstacle_d - obstacle_d) / max(args.lidar_obstacle_d, 1.0e-6)
        + 0.25 * (1.0 - min(1.0, restore_shift / args.max_restore_shift))
    )
    return supported


def filter_lidar_supported_candidates(candidates, ranges, observed_center, args):
    supported = []
    for candidate in candidates:
        item = add_lidar_support(candidate, ranges, observed_center, args)
        if item is not None:
            supported.append(item)
    return supported


def filter_near_observed_candidates(candidates, observed_center, args):
    return [
        c for c in candidates
        if float(np.linalg.norm(c["center"] - observed_center)) <= args.max_restore_shift
    ]


def touched_frame_sides(display_points, frame_shape, border_px):
    pts = np.asarray(display_points, dtype=np.float32).reshape(-1, 2)
    height, width = frame_shape[:2]
    sides = set()
    if float(np.min(pts[:, 0])) <= border_px:
        sides.add("left")
    if float(np.max(pts[:, 0])) >= width - 1 - border_px:
        sides.add("right")
    if float(np.min(pts[:, 1])) <= border_px:
        sides.add("top")
    if float(np.max(pts[:, 1])) >= height - 1 - border_px:
        sides.add("bottom")
    return sides


def project_candidate(candidate, projector, frame_shape):
    if candidate.get("kind") == "image":
        return np.rint(candidate["display_polygon"]).astype(np.int32)
    raw     = projector.ground_to_raw_pixels(candidate["corners"])
    display = raw_to_display_points(raw, frame_shape)
    if np.isnan(display).any():
        return None
    return np.rint(display).astype(np.int32)


def candidate_display_center(candidate, projector, frame_shape):
    if candidate.get("kind") == "image":
        return candidate["display_center"]
    raw_center = projector.ground_to_raw_pixels(candidate["center"][None, :])
    center     = raw_to_display_points(raw_center, frame_shape)[0]
    if np.isnan(center).any():
        return None
    return center


def candidate_extends_outside_sides(candidate, projector, frame_shape, observed_display_center, sides, args):
    polygon = project_candidate(candidate, projector, frame_shape)
    if polygon is None:
        return False
    center = candidate_display_center(candidate, projector, frame_shape)
    if center is None:
        return False
    height, width = frame_shape[:2]
    min_x  = float(np.min(polygon[:, 0]))
    max_x  = float(np.max(polygon[:, 0]))
    min_y  = float(np.min(polygon[:, 1]))
    max_y  = float(np.max(polygon[:, 1]))
    out_px = float(args.out_of_frame_px)
    if "left"   in sides and not (center[0] <= observed_display_center[0] and min_x <= -out_px):
        return False
    if "right"  in sides and not (center[0] >= observed_display_center[0] and max_x >= width  - 1 + out_px):
        return False
    if "top"    in sides and not (center[1] <= observed_display_center[1] and min_y <= -out_px):
        return False
    if "bottom" in sides and not (center[1] >= observed_display_center[1] and max_y >= height - 1 + out_px):
        return False
    return True


def filter_border_candidates(candidates, target, projector, frame_shape, args):
    display_points = contour_points(target)
    sides = touched_frame_sides(display_points, frame_shape, args.border_touch_px)
    if not sides:
        return candidates
    observed_display_center = np.mean(display_points, axis=0)
    outside = [
        c for c in candidates
        if candidate_extends_outside_sides(
            c, projector, frame_shape, observed_display_center, sides, args
        )
    ]
    return outside if outside else candidates


def restore_square_candidates(ground_points, square_size):
    if len(ground_points) < MIN_GROUND_POINTS:
        return []
    e1, e2 = estimate_axes(ground_points)
    if e1 is None or e2 is None:
        return []
    a = ground_points @ e1
    b = ground_points @ e2
    a_options = axis_intervals(a, square_size)
    b_options = axis_intervals(b, square_size)
    candidates = []
    for a_min, a_max in a_options:
        for b_min, b_max in b_options:
            candidates.append(make_square(a_min, a_max, b_min, b_max, e1, e2))
    return candidates


def choose_candidate(candidates, previous_center, observed_center):
    if not candidates:
        return None
    if previous_center is not None:
        return min(
            candidates,
            key=lambda item: (
                float(np.linalg.norm(item["center"] - previous_center)),
                -float(item.get("lidar_score", 0.0)),
            ),
        )
    return max(
        candidates,
        key=lambda item: (
            float(item.get("lidar_score", 0.0)),
            -abs(float(item["center"][1])),
            -abs(float(item["center"][0] - observed_center[0])),
        ),
    )


def center_inside_frame_margin(center, frame_shape, margin_ratio):
    height, width = frame_shape[:2]
    margin_x = width  * margin_ratio
    margin_y = height * margin_ratio
    return (
        -margin_x <= center[0] <= width  + margin_x
        and -margin_y <= center[1] <= height + margin_y
    )


def candidate_is_plausible(candidate, projector, frame_shape, target_area, args):
    polygon = project_candidate(candidate, projector, frame_shape)
    if polygon is None:
        return False
    projected_area = polygon_area(polygon)
    frame_area     = float(frame_shape[0] * frame_shape[1])
    if projected_area < 0.8 * target_area:
        return False
    if projected_area > args.max_projected_area_ratio * frame_area:
        return False
    center = candidate_display_center(candidate, projector, frame_shape)
    if center is None:
        return False
    return center_inside_frame_margin(center, frame_shape, args.center_margin)


def filter_plausible_candidates(candidates, projector, frame_shape, target_area, args):
    return [c for c in candidates if candidate_is_plausible(c, projector, frame_shape, target_area, args)]


def filtered_candidate(candidate, previous_center, alpha):
    if previous_center is None:
        return candidate
    if candidate.get("kind") == "image":
        return candidate
    center = alpha * previous_center + (1.0 - alpha) * candidate["center"]
    shift  = center - candidate["center"]
    result = dict(candidate)
    result["center"]  = center.astype(np.float32)
    result["corners"] = (candidate["corners"] + shift[None, :]).astype(np.float32)
    return result


def draw_cross(frame, point, color):
    x, y = int(round(point[0])), int(round(point[1]))
    cv2.line(frame, (x - 8, y), (x + 8, y), color, 2)
    cv2.line(frame, (x, y - 8), (x, y + 8), color, 2)


def draw_candidate(frame, candidate, projector, color, thickness, center_color=None):
    polygon = project_candidate(candidate, projector, frame.shape)
    if polygon is None:
        return False
    cv2.polylines(frame, [polygon], True, color, thickness)
    if center_color is not None:
        center = candidate_display_center(candidate, projector, frame.shape)
        if center is not None:
            draw_cross(frame, center, center_color)
    return True


def restore_target(frame, target, projector, ranges, args, previous_center):
    observed = complete_observed_candidate(frame, target, projector, args)
    if observed is not None:
        short_m, long_m = observed["ground_size"]
        return observed, [], f"observed full {short_m:.2f}x{long_m:.2f}m"

    pts_display   = contour_points(target)
    raw_points    = display_to_raw_points(pts_display, frame.shape)
    ground_points = projector.raw_pixels_to_ground(raw_points)

    if len(ground_points) < MIN_GROUND_POINTS:
        return None, [], "few ground points"
    if not ground_shape_is_plausible(ground_points, args):
        return None, [], "implausible span"

    candidates    = restore_square_candidates(ground_points, args.square_size)
    observed_center = np.mean(ground_points, axis=0)
    candidates    = filter_near_observed_candidates(candidates, observed_center, args)
    candidates    = filter_border_candidates(candidates, target, projector, frame.shape, args)
    candidates    = filter_plausible_candidates(candidates, projector, frame.shape, target[5], args)

    if not candidates:
        return None, [], "no plausible square"

    supported = []
    if ranges is not None:
        supported = filter_lidar_supported_candidates(candidates, ranges, observed_center, args)

    if supported:
        selected = choose_candidate(supported, previous_center, observed_center)
        selected = filtered_candidate(selected, previous_center, args.smooth)
        lidar_angle    = selected.get("lidar_angle")
        lidar_distance = selected.get("lidar_distance")
        status = f"lidar restored {lidar_angle:.0f}deg {lidar_distance:.2f}m"
        return selected, candidates, status

    selected = choose_candidate(candidates, previous_center, observed_center)
    selected = filtered_candidate(selected, previous_center, args.smooth)
    reason   = "no lidar" if ranges is None else "no obstacle"
    return selected, candidates, f"restored geom ({reason})"


# ===========================================================================
# main.py helper functions
# ===========================================================================

def color_angle_from_target(target, frame_shape):
    if target is None:
        return 0.0
    err = bottom_center_error(target, frame_shape)
    return clamp(err * COLOR_TO_LIDAR_DEG, ANG_MIN, ANG_MAX)


def update_color_memory(target, frame):
    last_color_deg = color_angle_from_target(target, frame.shape)
    center_ratio   = float(target[7]) / frame.shape[0]
    return last_color_deg, time.time(), center_ratio, x_center_error(target, frame.shape)


def clear_color_memory():
    return 0.0, 0.0, 0.0, 0.0


def search_next_cmd():
    return "SEARCH: next color", 0.0, SWITCH_SEARCH_W


def obstacle_in_zone(ranges, zone, clear_d=FREE_D):
    if ranges is None:
        return False
    return float(np.min(ranges[zone])) < clear_d


def color_path_clear(ranges, color_deg, clear_d=FREE_D):
    if ranges is None:
        return False
    color_side_zone = RIGHT_ZONE if color_deg >= 0.0 else LEFT_ZONE
    front_clear = not obstacle_in_zone(ranges, FRONT_LOG_ZONE, clear_d)
    side_clear  = not obstacle_in_zone(ranges, color_side_zone, clear_d)
    return front_clear and side_clear


def front_obstacle_distance(ranges):
    if ranges is None:
        return FREE_D
    return float(np.min(ranges[FRONT_LOG_ZONE]))


def scale_avoid_speed_for_front_obstacle(target_v, ranges):
    front_d = front_obstacle_distance(ranges)
    if front_d <= AVOID_STOP_D:
        return 0.0
    if front_d >= FREE_D:
        return target_v
    scale = (front_d - AVOID_STOP_D) / (FREE_D - AVOID_STOP_D)
    return target_v * clamp(scale, 0.0, 1.0)


def lost_color_obstacle_passed(ranges, last_color_deg):
    color_side_zone = RIGHT_ZONE if last_color_deg >= 0.0 else LEFT_ZONE
    front_clear = not obstacle_in_zone(ranges, FRONT_LOG_ZONE, FREE_D)
    side_clear  = not obstacle_in_zone(ranges, color_side_zone, SIDE_CLEAR_D)
    return front_clear and side_clear


def get_turn_start_odom(motor):
    enc_l, enc_r, _, odom_time = motor.get_odom()
    if odom_time == 0.0:
        return None
    return enc_l, enc_r


def completed_one_encoder_turn(motor, start_odom):
    if start_odom is None:
        return False
    enc_l, enc_r, _, odom_time = motor.get_odom()
    if odom_time == 0.0 or time.time() - odom_time > 0.5:
        return False
    start_l, start_r = start_odom
    left_counts  = abs(enc_l - start_l)
    right_counts = abs(enc_r - start_r)
    return 0.5 * (left_counts + right_counts) >= TURN_360_COUNTS


def drive_forward_by_encoder(motor, distance_m=POST_COLOR_FORWARD_M):
    while True:
        odom = motor.get_odom()
        if odom[3] != 0.0:
            break
        wait_ms(int(LOOP_DT * 1000))

    start_l, start_r, _, _ = odom
    target_counts = abs(distance_m) * ENC_COUNTS_PER_M
    forward_v = abs(DRIVE_V) if distance_m >= 0.0 else -abs(DRIVE_V)

    print(f"[ODOM] move {distance_m:.2f}m target_counts={target_counts:.0f}")

    while True:
        enc_l, enc_r, _, odom_time = motor.get_odom()
        if odom_time == 0.0 or time.time() - odom_time > 0.5:
            motor.stop()
            print("[ODOM] encoder data timeout during extra forward move")
            return False

        left_counts  = abs(enc_l - start_l)
        right_counts = abs(enc_r - start_r)
        if left_counts >= target_counts and right_counts >= target_counts:
            break

        motor.vw(forward_v, 0.0)
        wait_ms(int(LOOP_DT * 1000))

    motor.stop()
    left_m  = left_counts  / ENC_COUNTS_PER_M
    right_m = right_counts / ENC_COUNTS_PER_M
    ok = left_counts >= target_counts and right_counts >= target_counts
    print(
        f"[ODOM] extra forward done: "
        f"left={left_m:.2f}m({left_counts:.0f}) "
        f"right={right_m:.2f}m({right_counts:.0f})"
    )
    return ok


def log_lidar(elapsed, lidar_dist):
    if lidar_dist is None:
        print(f"[{elapsed:.2f}s] [LIDAR] waiting...")
        return
    (left_d, left_n), (front_d, front_n), (right_d, right_n) = lidar_dist
    print(
        f"[{elapsed:.2f}s] [LIDAR] "
        f"left={left_d:.2f}m({left_n}) "
        f"front={front_d:.2f}m({front_n}) "
        f"right={right_d:.2f}m({right_n})"
    )


def log_avoid(elapsed, tag, target_deg, target_v, target_w, gap_count):
    print(
        f"[{elapsed:.2f}s] [{tag}] gap={gap_count} "
        f"target={target_deg:.0f} "
        f"v={target_v:.2f} "
        f"w={target_w:.2f}"
    )


def log_odom(elapsed, odom):
    enc_l, enc_r, arduino_ms, odom_time = odom
    if odom_time == 0.0:
        print(f"[{elapsed:.2f}s] [ODOM] waiting...")
        return
    print(f"[{elapsed:.2f}s] [ODOM] left={enc_l} right={enc_r} arduino_ms={arduino_ms}")


def color_cmd(target, frame, ranges, has_obstacle, color_deg, elapsed):
    if has_obstacle:
        if color_path_clear(ranges, color_deg):
            target_v, target_w = follow_cmd(target, frame.shape, DRIVE_V)
            return "FOLLOW: color path clear", target_v, target_w
        target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, color_deg, DRIVE_V)
        target_v = scale_avoid_speed_for_front_obstacle(target_v, ranges)
        log_avoid(elapsed, "AVOID", target_deg, target_v, target_w, gap_count)
        return "AVOID: color + obstacle", target_v, target_w
    target_v, target_w = follow_cmd(target, frame.shape, DRIVE_V)
    return "FOLLOW: color only", target_v, target_w


def search_last_cmd(last_color_deg):
    w = -SEARCH_MAX_W if last_color_deg >= 0.0 else SEARCH_MAX_W
    return "SEARCH: last color direction", 0.0, w


def avoid_mode(mode, tag, ranges, color_deg, elapsed):
    target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, color_deg, DRIVE_V)
    target_v = scale_avoid_speed_for_front_obstacle(target_v, ranges)
    log_avoid(elapsed, tag, target_deg, target_v, target_w, gap_count)
    return mode, target_v, target_w


# ===========================================================================
# Merged main()
# ===========================================================================

def main():
    cam = lidar = motor = None
    last_v = last_w = 0.0
    last_color_deg = last_color_time = last_color_center_ratio = last_color_x_err = 0.0
    last_odom_log_time = 0.0
    color_lost_during_avoid    = False
    search_failed_avoid_active = False
    switch_search_active       = False
    switch_search_start_odom   = None
    target_index   = 0
    current_target = TARGET_SEQUENCE[target_index]
    previous_restore_center = None

    # Restore args (mutable copy with clamped values)
    rargs = SimpleNamespace(**vars(RESTORE_ARGS))
    rargs.smooth                  = clamp(rargs.smooth, 0.0, 0.95)
    rargs.complete_fill_ratio     = clamp(rargs.complete_fill_ratio, 0.1, 0.95)
    rargs.complete_size_min_ratio = clamp(rargs.complete_size_min_ratio, 0.1, 1.0)
    rargs.complete_size_max_ratio = max(rargs.complete_size_max_ratio, rargs.complete_size_min_ratio)
    rargs.center_margin           = clamp(rargs.center_margin, 0.0, 1.0)
    rargs.border_touch_px         = max(0.0, rargs.border_touch_px)
    rargs.out_of_frame_px         = max(0.0, rargs.out_of_frame_px)
    rargs.max_restore_shift       = max(rargs.max_restore_shift, rargs.min_restore_shift + 1.0e-6)

    # Calibration — graceful fallback if files are missing
    projector   = None
    undistorter = None
    try:
        camera_matrix, dist_coeffs = load_tuned_intrinsics(rargs)
        if camera_matrix is not None and dist_coeffs is not None:
            undistorter = Undistorter(camera_matrix, dist_coeffs, CALIB_WIDTH, CALIB_HEIGHT)
            print("[CALIB] undistorter loaded")
        h_path = Path(rargs.homography_file).expanduser()
        if h_path.exists():
            projector = GroundProjector(load_homography(h_path))
            print(f"[CALIB] homography loaded: {h_path}")
        else:
            print(f"[CALIB] homography not found ({h_path}), restore disabled")
    except Exception as exc:
        print(f"[CALIB] restore disabled: {exc}")

    try:
        cam   = open_camera()
        lidar = RPLidarC1()  # single instance shared with restore (Problem 1 fix)
        motor = Motor()
        motor.stop()
        motor.reset_encoders()
        motor_enabled = threading.Event()

        def wait_for_motor_start():
            input("[READY] Press Enter to enable motor...")
            motor_enabled.set()

        threading.Thread(target=wait_for_motor_start, daemon=True).start()
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            ok, frame = read_frame(cam)
            if not ok:
                break
            if undistorter is not None:
                frame = undistorter.apply(frame)

            found  = detect(frame)
            target = pick(found, current_target)
            found  = [target] if target is not None else []

            # Single lidar read with staleness check — shared with restore_target()
            scan, scan_time, _ = lidar.get()
            if scan is not None and time.time() - scan_time <= rargs.lidar_max_age:
                ranges = front_ranges(scan)
            else:
                ranges = None

            lidar_dist   = lidar_zone_distances(ranges)
            has_obstacle = obstacle_detected(ranges)

            if target is not None:
                last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = \
                    update_color_memory(target, frame)
                switch_search_active, search_failed_avoid_active, switch_search_start_odom = \
                    False, False, None

            if time.time() - last_odom_log_time >= ODOM_LOG_INTERVAL:
                log_odom(elapsed, motor.get_odom())
                last_odom_log_time = time.time()

            log_lidar(elapsed, lidar_dist)

            # Restore square overlay — passes same ranges, no second RPLidarC1
            if projector is not None and target is not None:
                selected, _, restore_status = restore_target(
                    frame, target, projector, ranges, rargs, previous_restore_center
                )
                if selected is not None:
                    previous_restore_center = selected["center"]
                    draw_candidate(frame, selected, projector, SELECTED_COLOR, 3, CENTER_COLOR)
                    cv2.putText(
                        frame, f"[R] {restore_status}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, SELECTED_COLOR, 1,
                    )
                else:
                    previous_restore_center = None
            elif target is None:
                previous_restore_center = None

            color_in_forward_zone = (
                motor_enabled.is_set()
                and target is not None
                and last_color_center_ratio >= COLOR_FORWARD_CENTER_RATIO
                and abs(last_color_x_err) <= COLOR_EXIT_CENTER_ERR
            )
            color_in_bottom_zone = (
                target is not None and last_color_center_ratio >= BOTTOM_LOST_RATIO
            )
            color_exited_bottom = (
                target is None
                and last_color_time > 0.0
                and last_color_center_ratio >= BOTTOM_LOST_RATIO
            )

            target_v = target_w = 0.0
            mode = "IDLE"

            if color_in_forward_zone:
                if target_index + 1 < len(TARGET_SEQUENCE):
                    prev_target    = current_target
                    target_index  += 1
                    current_target = TARGET_SEQUENCE[target_index]
                    previous_restore_center = None
                    mode = f"SWITCH: {prev_target}->{current_target}"
                    print(f"[{elapsed:.2f}s] [COLOR] {prev_target} done, now tracking {current_target}")
                    drive_forward_by_encoder(motor)
                    target_v, target_w, last_v, last_w = 0.0, 0.0, 0.0, 0.0
                    wait_ms(COLOR_SWITCH_PAUSE_MS)

                    scan, scan_time, _ = lidar.get()
                    if scan is not None and time.time() - scan_time <= rargs.lidar_max_age:
                        ranges = front_ranges(scan)
                    else:
                        ranges = None
                    lidar_dist   = lidar_zone_distances(ranges)
                    has_obstacle = obstacle_detected(ranges)
                    ok, frame    = read_frame(cam)
                    if not ok:
                        break
                    if undistorter is not None:
                        frame = undistorter.apply(frame)

                    found  = detect(frame)
                    target = pick(found, current_target)
                    found  = [target] if target is not None else []

                    if target is not None:
                        last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = \
                            update_color_memory(target, frame)
                        color_lost_during_avoid    = False
                        search_failed_avoid_active = False
                        if last_color_center_ratio >= BOTTOM_LOST_RATIO:
                            target_v, target_w = follow_cmd(target, frame.shape, DRIVE_V)
                            mode = "FOLLOW: bottom color"
                        else:
                            mode, target_v, target_w = color_cmd(
                                target, frame, ranges, has_obstacle, last_color_deg, elapsed
                            )
                    else:
                        mode, target_v, target_w = search_next_cmd()
                        last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = \
                            clear_color_memory()
                        color_lost_during_avoid, switch_search_active = False, True
                        switch_search_start_odom = get_turn_start_odom(motor)

                else:
                    mode = "STOP: color bottom"
                    drive_forward_by_encoder(motor)
                    target_v, target_w, last_v, last_w = 0.0, 0.0, 0.0, 0.0
                    last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = \
                        clear_color_memory()
                    color_lost_during_avoid, switch_search_active   = False, False
                    search_failed_avoid_active, switch_search_start_odom = False, None
                    previous_restore_center = None
                    print(f"[{elapsed:.2f}s] [COLOR] {current_target} done, mission complete")
                    break

            elif target is not None:
                color_lost_during_avoid    = False
                search_failed_avoid_active = False
                if color_in_bottom_zone:
                    target_v, target_w = follow_cmd(target, frame.shape, DRIVE_V)
                    mode = "FOLLOW: bottom color"
                else:
                    mode, target_v, target_w = color_cmd(
                        target, frame, ranges, has_obstacle, last_color_deg, elapsed
                    )

            elif target is None and search_failed_avoid_active:
                mode, target_v, target_w = avoid_mode(
                    "AVOID: search failed", "AVOID_SEARCH",
                    ranges, last_color_deg if last_color_time > 0.0 else None, elapsed,
                )

            elif color_exited_bottom:
                color_lost_during_avoid = True
                switch_search_active    = False
                if not lost_color_obstacle_passed(ranges, last_color_deg):
                    mode, target_v, target_w = avoid_mode(
                        "AVOID: lost color", "AVOID_LOST", ranges, last_color_deg, elapsed
                    )
                else:
                    mode, target_v, target_w = search_last_cmd(last_color_deg)

            elif target is None and last_color_time > 0.0 and not lost_color_obstacle_passed(ranges, last_color_deg):
                color_lost_during_avoid = True
                mode, target_v, target_w = avoid_mode(
                    "AVOID: lost color", "AVOID_LOST", ranges, last_color_deg, elapsed
                )

            elif target is None and switch_search_active:
                if switch_search_start_odom is None:
                    switch_search_start_odom = get_turn_start_odom(motor)
                if completed_one_encoder_turn(motor, switch_search_start_odom):
                    mode, target_v, target_w = avoid_mode(
                        "AVOID: search failed", "AVOID_SEARCH",
                        ranges, last_color_deg if last_color_time > 0.0 else None, elapsed,
                    )
                    color_lost_during_avoid, switch_search_active = True, False
                    search_failed_avoid_active   = True
                    switch_search_start_odom     = None
                else:
                    mode, target_v, target_w = search_next_cmd()

            elif target is None and color_lost_during_avoid:
                mode, target_v, target_w = search_last_cmd(last_color_deg)

            elif target is None:
                if last_color_time > 0.0:
                    mode, target_v, target_w = search_last_cmd(last_color_deg)
                    color_lost_during_avoid, switch_search_active = True, False
                else:
                    mode, target_v, target_w = avoid_mode(
                        "AVOID: no initial color", "AVOID_INITIAL", ranges, None, elapsed
                    )
                    color_lost_during_avoid, switch_search_active = True, False
                    search_failed_avoid_active   = True
                    switch_search_start_odom     = None

            if motor_enabled.is_set() and (
                target is not None or mode.startswith("AVOID") or mode.startswith("SEARCH")
            ):
                last_v = rate_limit(last_v, target_v, V_STEP)
                w_step = AVOID_W_STEP if mode.startswith("AVOID") else FOLLOW_W_STEP
                last_w = rate_limit(last_w, target_w, w_step)
                motor.vw(last_v, last_w)
            elif not motor_enabled.is_set():
                last_v, last_w = 0.0, 0.0
                motor.stop()

            if SHOW_WINDOW:
                if draw(frame, found, mode) == ord("q"):
                    break

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\n[INFO] stop")

    finally:
        if motor is not None:
            motor.stop()
            motor.close()
        if lidar is not None:
            lidar.close()
        close_camera(cam)


if __name__ == "__main__":
    main()
