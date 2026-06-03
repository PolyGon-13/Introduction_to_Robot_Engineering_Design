#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "sihyun" / "test"))

from camera import (
    BOX_COLORS,
    CAMERA_ROTATION,
    close_camera,
    detect,
    open_camera,
    pick,
    read_frame,
)
from lidar import FREE_D, GRID, RPLidarC1, front_ranges


TARGET_NAMES = ("RED", "YELLOW", "BLUE")
THIS_DIR = Path(__file__).resolve().parent
DEFAULT_HOMOGRAPHY_FILE = THIS_DIR / "restore_square_homography.npz"
DEFAULT_CAMERA_MATRIX_FILE = THIS_DIR.parent / "camera_calibration" / "camera_matrix.npy"
DEFAULT_DIST_COEFFS_FILE = THIS_DIR.parent / "camera_calibration" / "dist_coeffs.npy"

CALIB_WIDTH = 640.0
CALIB_HEIGHT = 480.0
CALIB_FX = 651.22884042
CALIB_FY = 676.79930888
CALIB_CX = 380.17305783
CALIB_CY = 221.65856353

CAMERA_HEIGHT_M = 0.65
PITCH_DOWN_DEG = 45.0
SQUARE_SIZE_M = 0.30
MIN_AREA = 1000
MIN_GROUND_POINTS = 3
MIN_GROUND_SPAN_M = 0.04
MAX_GROUND_SPAN_M = 0.45
COMPLETE_FILL_RATIO = 0.65
COMPLETE_SIZE_MIN_RATIO = 0.70
COMPLETE_SIZE_MAX_RATIO = 1.45
MAX_PROJECTED_AREA_RATIO = 2.0
CENTER_MARGIN_RATIO = 0.15
LIDAR_GUIDE_HALF_DEG = 12.0
LIDAR_MAX_AGE_S = 0.5
MIN_RESTORE_SHIFT_M = 0.03
PRINT_INTERVAL_S = 0.5
GROUND_FORWARD_SCALE = 1.0
GROUND_SIDE_SCALE = 1.0
CALIB_X_NEAR_M = 0.30
CALIB_X_FAR_M = 0.60
CALIB_Y_LEFT_M = -0.15
CALIB_Y_RIGHT_M = 0.15

SELECTED_COLOR = (255, 255, 255)
CANDIDATE_COLOR = (120, 120, 120)
CENTER_COLOR = (0, 255, 0)


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def frame_raw_size(frame_shape):
    display_h, display_w = frame_shape[:2]

    if CAMERA_ROTATION in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
        return float(display_h), float(display_w)

    return float(display_w), float(display_h)


def scaled_intrinsics(frame_shape, args):
    raw_w, raw_h = frame_raw_size(frame_shape)
    sx = raw_w / args.calib_width
    sy = raw_h / args.calib_height

    return (
        args.fx * sx,
        args.fy * sy,
        args.cx * sx,
        args.cy * sy,
    )


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


def unrotate_to_raw_image(frame):
    """표시 프레임(회전 적용됨)을 원본 센서 방향으로 되돌린다.

    내부 파라미터/왜곡 계수는 회전 전 raw 센서 좌표계에서 정의되므로,
    undistort 는 반드시 raw 방향에서 수행해야 한다.
    """
    if CAMERA_ROTATION == cv2.ROTATE_90_COUNTERCLOCKWISE:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if CAMERA_ROTATION == cv2.ROTATE_90_CLOCKWISE:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if CAMERA_ROTATION == cv2.ROTATE_180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    return frame


def rotate_to_display_image(raw_image):
    """raw 방향 이미지를 다시 표시(회전된) 방향으로 돌린다."""
    if CAMERA_ROTATION is None:
        return raw_image
    return cv2.rotate(raw_image, CAMERA_ROTATION)


class Undistorter:
    """튜닝된 camera_matrix/dist_coeffs 로 프레임의 렌즈 왜곡을 제거한다.

    new camera matrix 를 원본 K 로 두기 때문에 undistort 후에도 내부 파라미터는
    그대로 유지되고, 핀홀 지면 투영이 유효해진다. raw 해상도별 remap 맵을 캐시한다.
    """

    def __init__(self, camera_matrix, dist_coeffs, calib_width, calib_height):
        self.base_k = np.asarray(camera_matrix, dtype=np.float64)
        self.dist = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        self.calib_w = float(calib_width)
        self.calib_h = float(calib_height)
        self._maps = {}

    def _maps_for(self, raw_w, raw_h):
        key = (raw_w, raw_h)

        if key not in self._maps:
            sx = raw_w / self.calib_w
            sy = raw_h / self.calib_h
            k = self.base_k.copy()
            k[0, 0] *= sx
            k[0, 2] *= sx
            k[1, 1] *= sy
            k[1, 2] *= sy
            map1, map2 = cv2.initUndistortRectifyMap(
                k, self.dist, None, k, (raw_w, raw_h), cv2.CV_16SC2
            )
            self._maps[key] = (map1, map2)

        return self._maps[key]

    def apply(self, frame):
        raw_image = unrotate_to_raw_image(frame)
        raw_h, raw_w = raw_image.shape[:2]
        map1, map2 = self._maps_for(raw_w, raw_h)
        undistorted = cv2.remap(raw_image, map1, map2, cv2.INTER_LINEAR)
        return rotate_to_display_image(undistorted)


def load_tuned_intrinsics(args):
    """camera_matrix.npy / dist_coeffs.npy 를 읽어 내부 파라미터·왜곡 계수를 반환.

    파일이 없으면 (None, None) 을 돌려주고 호출측이 args 기본값으로 대체한다.
    """
    camera_matrix = None
    dist_coeffs = None

    matrix_path = Path(args.camera_matrix_file).expanduser()
    dist_path = Path(args.dist_coeffs_file).expanduser()

    if matrix_path.exists():
        camera_matrix = np.load(str(matrix_path)).astype(np.float64)

    if dist_path.exists():
        dist_coeffs = np.load(str(dist_path)).astype(np.float64).reshape(-1)

    return camera_matrix, dist_coeffs


class GroundProjector:
    def __init__(
        self,
        height_m,
        pitch_down_deg,
        fx,
        fy,
        cx,
        cy,
        forward_scale=1.0,
        side_scale=1.0,
        homography=None,
    ):
        self.height_m = float(height_m)
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.forward_scale = float(forward_scale)
        self.side_scale = float(side_scale)
        self.homography = None
        self.inverse_homography = None

        if homography is not None:
            self.homography = np.asarray(homography, dtype=np.float32)
            self.inverse_homography = np.linalg.inv(self.homography).astype(np.float32)

        theta = math.radians(pitch_down_deg)
        self.cam_right = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        self.cam_down = np.array([-math.sin(theta), 0.0, math.cos(theta)], dtype=np.float32)
        self.cam_forward = np.array([math.cos(theta), 0.0, math.sin(theta)], dtype=np.float32)

    def raw_pixels_to_ground(self, raw_points):
        pts = np.asarray(raw_points, dtype=np.float32).reshape(-1, 2)

        if self.homography is not None:
            mapped = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), self.homography)
            return mapped.reshape(-1, 2).astype(np.float32)

        x_cam = (pts[:, 0] - self.cx) / self.fx
        y_cam = (pts[:, 1] - self.cy) / self.fy

        rays = (
            x_cam[:, None] * self.cam_right
            + y_cam[:, None] * self.cam_down
            + self.cam_forward
        )
        valid = rays[:, 2] > 1.0e-6

        if not np.any(valid):
            return np.empty((0, 2), dtype=np.float32)

        scale = self.height_m / rays[valid, 2]
        ground = rays[valid, :2] * scale[:, None]
        ground[:, 0] *= self.forward_scale
        ground[:, 1] *= self.side_scale
        return ground.astype(np.float32)

    def ground_to_raw_pixels(self, ground_points):
        pts = np.asarray(ground_points, dtype=np.float32).reshape(-1, 2)

        if self.inverse_homography is not None:
            mapped = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), self.inverse_homography)
            return mapped.reshape(-1, 2).astype(np.float32)

        unscaled = pts.copy()
        unscaled[:, 0] /= max(self.forward_scale, 1.0e-6)
        unscaled[:, 1] /= max(self.side_scale, 1.0e-6)
        vecs = np.column_stack(
            (
                unscaled[:, 0],
                unscaled[:, 1],
                np.full(len(pts), self.height_m, dtype=np.float32),
            )
        )

        x_cam = vecs @ self.cam_right
        y_cam = vecs @ self.cam_down
        z_cam = vecs @ self.cam_forward
        valid = z_cam > 1.0e-6

        raw = np.full((len(pts), 2), np.nan, dtype=np.float32)
        raw[valid, 0] = self.fx * x_cam[valid] / z_cam[valid] + self.cx
        raw[valid, 1] = self.fy * y_cam[valid] / z_cam[valid] + self.cy
        return raw


def homography_path(args):
    return Path(args.homography_file).expanduser()


def calibration_ground_points(args):
    return np.array(
        [
            [args.calib_x_near, args.calib_y_left],
            [args.calib_x_near, args.calib_y_right],
            [args.calib_x_far, args.calib_y_right],
            [args.calib_x_far, args.calib_y_left],
        ],
        dtype=np.float32,
    )


def load_homography(path):
    data = np.load(str(path))
    return np.asarray(data["homography"], dtype=np.float32)


def save_homography(path, homography, raw_points, display_points, ground_points):
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        str(path),
        homography=np.asarray(homography, dtype=np.float32),
        raw_points=np.asarray(raw_points, dtype=np.float32),
        display_points=np.asarray(display_points, dtype=np.float32),
        ground_points=np.asarray(ground_points, dtype=np.float32),
    )


def draw_calibration_overlay(frame, clicked, ground_points):
    labels = ("near-left", "near-right", "far-right", "far-left")
    view = frame.copy()

    for idx, point in enumerate(clicked):
        x, y = int(round(point[0])), int(round(point[1]))
        cv2.circle(view, (x, y), 5, (0, 255, 0), -1)
        cv2.putText(
            view,
            str(idx + 1),
            (x + 8, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    next_idx = min(len(clicked), len(labels) - 1)
    next_xy = ground_points[next_idx]
    lines = [
        "Click 4 floor calibration points.",
        f"Next: {labels[next_idx]} x={next_xy[0]:.2f}m y={next_xy[1]:.2f}m",
        "Order: near-left, near-right, far-right, far-left",
        "r: reset, q: cancel",
    ]

    for idx, text in enumerate(lines):
        cv2.putText(
            view,
            text,
            (10, 28 + 24 * idx),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
        )

    return view


def calibrate_homography(cam, args, undistorter=None):
    ok, frame = read_frame(cam)

    if not ok:
        raise RuntimeError("[CALIB] camera frame read failed")

    if undistorter is not None:
        frame = undistorter.apply(frame)

    clicked = []
    ground_points = calibration_ground_points(args)
    window_name = "restore_square calibration"

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < 4:
            clicked.append((float(x), float(y)))

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse)

    while len(clicked) < 4:
        cv2.imshow(window_name, draw_calibration_overlay(frame, clicked, ground_points))
        key = cv2.waitKey(20) & 0xFF

        if key == ord("q"):
            cv2.destroyWindow(window_name)
            raise RuntimeError("[CALIB] homography calibration cancelled")

        if key == ord("r"):
            clicked.clear()

    cv2.imshow(window_name, draw_calibration_overlay(frame, clicked, ground_points))
    cv2.waitKey(300)
    cv2.destroyWindow(window_name)

    display_points = np.asarray(clicked, dtype=np.float32)
    raw_points = display_to_raw_points(display_points, frame.shape)
    homography, _ = cv2.findHomography(raw_points, ground_points, 0)

    if homography is None:
        raise RuntimeError("[CALIB] findHomography failed")

    path = homography_path(args)
    save_homography(path, homography, raw_points, display_points, ground_points)
    print(f"[CALIB] saved homography: {path}")
    return np.asarray(homography, dtype=np.float32)


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
    box = cv2.boxPoints(rect).astype(np.float32)
    edges = [box[(idx + 1) % 4] - box[idx] for idx in range(4)]
    lengths = [float(np.linalg.norm(edge)) for edge in edges]

    if max(lengths, default=0.0) < 1.0e-6:
        return None, None

    idx = int(np.argmax(lengths))
    e1 = normalize(edges[idx])
    e2 = normalize(edges[(idx + 1) % 4])

    if e1 is None:
        return None, None

    if e2 is None or abs(float(np.dot(e1, e2))) > 0.35:
        e2 = np.array([-e1[1], e1[0]], dtype=np.float32)

    return e1, e2


def axis_intervals(values, square_size):
    v_min = float(np.min(values))
    v_max = float(np.max(values))
    span = v_max - v_min

    if span >= square_size:
        center = 0.5 * (v_min + v_max)
        return [(center - 0.5 * square_size, center + 0.5 * square_size)]

    candidates = [
        (v_min, v_min + square_size),
        (v_max - square_size, v_max),
    ]
    unique = []

    for item in candidates:
        if not any(abs(item[0] - prev[0]) < 1.0e-4 and abs(item[1] - prev[1]) < 1.0e-4 for prev in unique):
            unique.append(item)

    return unique


def make_square(a_min, a_max, b_min, b_max, e1, e2):
    local = np.array(
        [
            [a_min, b_min],
            [a_max, b_min],
            [a_max, b_max],
            [a_min, b_max],
        ],
        dtype=np.float32,
    )
    corners = local[:, 0:1] * e1[None, :] + local[:, 1:2] * e2[None, :]
    center = 0.5 * (a_min + a_max) * e1 + 0.5 * (b_min + b_max) * e2
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
    approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)

    if len(approx) == 4:
        return approx.reshape(-1, 2).astype(np.float32)

    rect = cv2.minAreaRect(contour)
    return cv2.boxPoints(rect).astype(np.float32)


def target_size_measurement(frame, target, projector):
    display_polygon = target_display_polygon(target)
    raw_polygon = display_to_raw_points(display_polygon, frame.shape)
    ground_polygon = projector.raw_pixels_to_ground(raw_polygon)
    dimensions = ground_polygon_dimensions(ground_polygon)
    ground_area = polygon_area(ground_polygon) if dimensions is not None else None
    ground_span = None

    if len(ground_polygon) > 0:
        ground_span = (
            float(np.max(ground_polygon[:, 0]) - np.min(ground_polygon[:, 0])),
            float(np.max(ground_polygon[:, 1]) - np.min(ground_polygon[:, 1])),
        )

    return {
        "display_polygon": display_polygon,
        "pixel_area": int(target[5]),
        "bbox_w": int(target[3]),
        "bbox_h": int(target[4]),
        "fill_ratio": contour_fill_ratio(target),
        "ground_size": dimensions,
        "ground_span": ground_span,
        "ground_area": ground_area,
    }


def format_size_log(name, measurement, status):
    if measurement["ground_size"] is None:
        ground_text = "ground=unknown"
    else:
        short_m, long_m = measurement["ground_size"]
        area_m2 = measurement["ground_area"]
        span_x, span_y = measurement["ground_span"]
        ground_text = (
            f"ground={short_m:.3f}x{long_m:.3f}m "
            f"span_x={span_x:.3f}m span_y={span_y:.3f}m "
            f"area={area_m2:.4f}m2"
        )

    return (
        f"[SIZE] {name} "
        f"pixels={measurement['pixel_area']}px "
        f"bbox={measurement['bbox_w']}x{measurement['bbox_h']}px "
        f"fill={measurement['fill_ratio']:.2f} "
        f"{ground_text} "
        f"status={status}"
    )


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
    fill_ratio = measurement["fill_ratio"]

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
        or long_m < expected * args.complete_size_min_ratio
        or short_m > expected * args.complete_size_max_ratio
        or long_m > expected * args.complete_size_max_ratio
    ):
        return None

    display_center = diagonal_center(display_polygon)
    raw_center = display_to_raw_points(display_center[None, :], frame.shape)
    ground_center = projector.raw_pixels_to_ground(raw_center)

    if len(ground_center) == 0:
        return None

    return {
        "kind": "image",
        "display_polygon": display_polygon,
        "display_center": display_center.astype(np.float32),
        "center": ground_center[0],
        "ground_size": (short_m, long_m),
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

    angle_deg = ground_bearing_deg(candidate["center"])
    obstacle_d = lidar_min_distance_at(ranges, angle_deg, args.lidar_half_deg)

    if obstacle_d is None or obstacle_d > args.lidar_obstacle_d:
        return None

    supported = dict(candidate)
    supported["lidar_angle"] = angle_deg
    supported["lidar_distance"] = obstacle_d
    supported["lidar_score"] = (
        (args.lidar_obstacle_d - obstacle_d) / max(args.lidar_obstacle_d, 1.0e-6)
        + 0.25 * min(1.0, restore_shift / args.square_size)
    )
    return supported


def filter_lidar_supported_candidates(candidates, ranges, observed_center, args):
    supported = []

    for candidate in candidates:
        item = add_lidar_support(candidate, ranges, observed_center, args)

        if item is not None:
            supported.append(item)

    return supported


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


def project_candidate(candidate, projector, frame_shape):
    if candidate.get("kind") == "image":
        return np.rint(candidate["display_polygon"]).astype(np.int32)

    raw = projector.ground_to_raw_pixels(candidate["corners"])
    display = raw_to_display_points(raw, frame_shape)

    if np.isnan(display).any():
        return None

    return np.rint(display).astype(np.int32)


def candidate_display_center(candidate, projector, frame_shape):
    if candidate.get("kind") == "image":
        return candidate["display_center"]

    raw_center = projector.ground_to_raw_pixels(candidate["center"][None, :])
    center = raw_to_display_points(raw_center, frame_shape)[0]

    if np.isnan(center).any():
        return None

    return center


def center_inside_frame_margin(center, frame_shape, margin_ratio):
    height, width = frame_shape[:2]
    margin_x = width * margin_ratio
    margin_y = height * margin_ratio

    return (
        -margin_x <= center[0] <= width + margin_x
        and -margin_y <= center[1] <= height + margin_y
    )


def candidate_is_plausible(candidate, projector, frame_shape, target_area, args):
    polygon = project_candidate(candidate, projector, frame_shape)

    if polygon is None:
        return False

    projected_area = polygon_area(polygon)
    frame_area = float(frame_shape[0] * frame_shape[1])

    if projected_area < 0.8 * target_area:
        return False

    if projected_area > args.max_projected_area_ratio * frame_area:
        return False

    center = candidate_display_center(candidate, projector, frame_shape)

    if center is None:
        return False

    return center_inside_frame_margin(center, frame_shape, args.center_margin)


def filter_plausible_candidates(candidates, projector, frame_shape, target_area, args):
    return [
        candidate
        for candidate in candidates
        if candidate_is_plausible(candidate, projector, frame_shape, target_area, args)
    ]


def filtered_candidate(candidate, previous_center, alpha):
    if previous_center is None:
        return candidate

    if candidate.get("kind") == "image":
        return candidate

    center = alpha * previous_center + (1.0 - alpha) * candidate["center"]
    shift = center - candidate["center"]
    filtered = dict(candidate)
    filtered["center"] = center.astype(np.float32)
    filtered["corners"] = (candidate["corners"] + shift[None, :]).astype(np.float32)
    return filtered


def draw_cross(frame, point, color):
    x, y = int(round(point[0])), int(round(point[1]))
    cv2.line(frame, (x - 8, y), (x + 8, y), color, 2)
    cv2.line(frame, (x, y - 8), (x, y + 8), color, 2)


def draw_candidate(frame, candidate, projector, color, thickness):
    polygon = project_candidate(candidate, projector, frame.shape)

    if polygon is None:
        return False

    cv2.polylines(frame, [polygon], True, color, thickness)

    center = candidate_display_center(candidate, projector, frame.shape)

    if center is not None:
        draw_cross(frame, center, CENTER_COLOR)

    return True


def restore_target(frame, target, projector, ranges, args, previous_center):
    observed = complete_observed_candidate(frame, target, projector, args)

    if observed is not None:
        short_m, long_m = observed["ground_size"]
        return observed, [], f"observed full {short_m:.2f}x{long_m:.2f}m"

    pts_display = contour_points(target)
    raw_points = display_to_raw_points(pts_display, frame.shape)
    ground_points = projector.raw_pixels_to_ground(raw_points)

    if len(ground_points) < MIN_GROUND_POINTS:
        return None, [], "few ground points"

    if not ground_shape_is_plausible(ground_points, args):
        return None, [], "implausible span"

    candidates = restore_square_candidates(ground_points, args.square_size)
    candidates = filter_plausible_candidates(candidates, projector, frame.shape, target[5], args)

    if not candidates:
        return None, [], "no plausible square"

    observed_center = np.mean(ground_points, axis=0)
    if ranges is None:
        return None, candidates, "waiting lidar"

    supported = filter_lidar_supported_candidates(candidates, ranges, observed_center, args)

    if not supported:
        return None, candidates, "no lidar obstacle"

    selected = choose_candidate(supported, previous_center, observed_center)
    selected = filtered_candidate(selected, previous_center, args.smooth)
    lidar_angle = selected.get("lidar_angle")
    lidar_distance = selected.get("lidar_distance")
    status = f"lidar restored {lidar_angle:.0f}deg {lidar_distance:.2f}m"
    return selected, candidates, status


def draw_target(frame, target, selected, candidates, projector, status, args):
    name, x, y, w, h, area, cx, cy, box, cnt = target
    color = BOX_COLORS.get(name, (0, 255, 0))

    cv2.drawContours(frame, [cnt], -1, color, 2)
    cv2.polylines(frame, [box], True, color, 1)
    cv2.circle(frame, (int(round(cx)), int(round(cy))), 4, color, -1)

    if args.show_candidates:
        for candidate in candidates:
            draw_candidate(frame, candidate, projector, CANDIDATE_COLOR, 1)

    if selected is not None:
        draw_candidate(frame, selected, projector, SELECTED_COLOR, 3)

    cv2.putText(
        frame,
        f"{name} area={area} {status}",
        (x, max(24, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Draw a restored 30 cm square from a partial HSV color region."
    )
    parser.add_argument("--target", choices=TARGET_NAMES, default="RED")
    parser.add_argument("--square-size", type=float, default=SQUARE_SIZE_M)
    parser.add_argument("--homography-file", default=str(DEFAULT_HOMOGRAPHY_FILE))
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--use-homography", action="store_true")
    parser.add_argument("--camera-matrix-file", default=str(DEFAULT_CAMERA_MATRIX_FILE))
    parser.add_argument("--dist-coeffs-file", default=str(DEFAULT_DIST_COEFFS_FILE))
    parser.add_argument("--no-undistort", action="store_true",
                        help="튜닝된 왜곡 계수로 프레임 보정하는 단계를 끈다")
    parser.add_argument("--calib-x-near", type=float, default=CALIB_X_NEAR_M)
    parser.add_argument("--calib-x-far", type=float, default=CALIB_X_FAR_M)
    parser.add_argument("--calib-y-left", type=float, default=CALIB_Y_LEFT_M)
    parser.add_argument("--calib-y-right", type=float, default=CALIB_Y_RIGHT_M)
    parser.add_argument("--height", type=float, default=CAMERA_HEIGHT_M)
    parser.add_argument("--pitch-down", type=float, default=PITCH_DOWN_DEG)
    parser.add_argument("--fx", type=float, default=CALIB_FX)
    parser.add_argument("--fy", type=float, default=CALIB_FY)
    parser.add_argument("--cx", type=float, default=CALIB_CX)
    parser.add_argument("--cy", type=float, default=CALIB_CY)
    parser.add_argument("--calib-width", type=float, default=CALIB_WIDTH)
    parser.add_argument("--calib-height", type=float, default=CALIB_HEIGHT)
    parser.add_argument("--min-area", type=int, default=MIN_AREA)
    parser.add_argument("--min-ground-span", type=float, default=MIN_GROUND_SPAN_M)
    parser.add_argument("--max-ground-span", type=float, default=MAX_GROUND_SPAN_M)
    parser.add_argument("--complete-fill-ratio", type=float, default=COMPLETE_FILL_RATIO)
    parser.add_argument("--complete-size-min-ratio", type=float, default=COMPLETE_SIZE_MIN_RATIO)
    parser.add_argument("--complete-size-max-ratio", type=float, default=COMPLETE_SIZE_MAX_RATIO)
    parser.add_argument("--max-projected-area-ratio", type=float, default=MAX_PROJECTED_AREA_RATIO)
    parser.add_argument("--center-margin", type=float, default=CENTER_MARGIN_RATIO)
    parser.add_argument("--lidar-obstacle-d", type=float, default=FREE_D)
    parser.add_argument("--lidar-half-deg", type=float, default=LIDAR_GUIDE_HALF_DEG)
    parser.add_argument("--lidar-max-age", type=float, default=LIDAR_MAX_AGE_S)
    parser.add_argument("--min-restore-shift", type=float, default=MIN_RESTORE_SHIFT_M)
    parser.add_argument("--no-lidar", action="store_true")
    parser.add_argument("--print-interval", type=float, default=PRINT_INTERVAL_S)
    parser.add_argument("--ground-forward-scale", type=float, default=GROUND_FORWARD_SCALE)
    parser.add_argument("--ground-side-scale", type=float, default=GROUND_SIDE_SCALE)
    parser.add_argument("--smooth", type=float, default=0.65)
    parser.add_argument("--show-candidates", action="store_true")
    return parser.parse_args()


def selected_target(found, target_name, min_area):
    found = [item for item in found if item[5] >= min_area]
    return pick(found, target_name)


def main():
    args = parse_args()
    args.smooth = clamp(args.smooth, 0.0, 0.95)
    args.complete_fill_ratio = clamp(args.complete_fill_ratio, 0.1, 0.95)
    args.complete_size_min_ratio = clamp(args.complete_size_min_ratio, 0.1, 1.0)
    args.complete_size_max_ratio = max(args.complete_size_max_ratio, args.complete_size_min_ratio)
    args.center_margin = clamp(args.center_margin, 0.0, 1.0)
    args.print_interval = max(0.0, args.print_interval)
    args.ground_forward_scale = max(args.ground_forward_scale, 1.0e-6)
    args.ground_side_scale = max(args.ground_side_scale, 1.0e-6)
    cam = lidar = None
    previous_center = None
    last_size_print = 0.0
    homography = None
    undistorter = None

    camera_matrix, dist_coeffs = load_tuned_intrinsics(args)

    if camera_matrix is not None:
        args.fx = float(camera_matrix[0, 0])
        args.fy = float(camera_matrix[1, 1])
        args.cx = float(camera_matrix[0, 2])
        args.cy = float(camera_matrix[1, 2])
        print(
            f"[CALIB] intrinsics loaded fx={args.fx:.1f} fy={args.fy:.1f} "
            f"cx={args.cx:.1f} cy={args.cy:.1f}"
        )
    else:
        print("[CALIB] intrinsics .npy not found; using built-in tuned defaults")

    if not args.no_undistort and camera_matrix is not None and dist_coeffs is not None:
        undistorter = Undistorter(camera_matrix, dist_coeffs, args.calib_width, args.calib_height)
        print("[CALIB] lens undistortion enabled")
    else:
        print("[CALIB] lens undistortion disabled")

    try:
        cam = open_camera()

        if args.use_homography:
            path = homography_path(args)

            if args.calibrate or not path.exists():
                print(f"[CALIB] homography file not found or recalibration requested: {path}")
                homography = calibrate_homography(cam, args, undistorter)
            else:
                homography = load_homography(path)
                print(f"[CALIB] loaded homography: {path}")
        else:
            print("[CALIB] pinhole projection (no homography click calibration)")

        if not args.no_lidar:
            lidar = RPLidarC1()

        while True:
            ok, frame = read_frame(cam)

            if not ok:
                break

            if undistorter is not None:
                frame = undistorter.apply(frame)

            fx, fy, cx, cy = scaled_intrinsics(frame.shape, args)
            projector = GroundProjector(
                args.height,
                args.pitch_down,
                fx,
                fy,
                cx,
                cy,
                args.ground_forward_scale,
                args.ground_side_scale,
                homography,
            )
            ranges = None
            lidar_status = "LIDAR off" if args.no_lidar else "LIDAR waiting"

            if lidar is not None:
                scan, scan_time, scan_seq = lidar.get()

                if scan is not None:
                    scan_age = time.time() - scan_time

                    if scan_age <= args.lidar_max_age:
                        ranges = front_ranges(scan)
                        lidar_status = f"LIDAR seq={scan_seq}"
                    else:
                        lidar_status = f"LIDAR stale {scan_age:.1f}s"

            target = selected_target(detect(frame), args.target, args.min_area)

            if target is None:
                previous_center = None
                status = f"{args.target}: not found"
            else:
                selected, candidates, status = restore_target(
                    frame, target, projector, ranges, args, previous_center
                )

                if selected is not None:
                    previous_center = selected["center"]
                else:
                    previous_center = None

                draw_target(frame, target, selected, candidates, projector, status, args)

                now = time.time()

                if args.print_interval == 0.0 or now - last_size_print >= args.print_interval:
                    measurement = target_size_measurement(frame, target, projector)
                    print(format_size_log(target[0], measurement, status))
                    last_size_print = now

            cv2.putText(
                frame,
                f"target={args.target} {status} {lidar_status} q: quit",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )
            cv2.imshow("restore_square", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        if lidar is not None:
            lidar.close()

        close_camera(cam)


if __name__ == "__main__":
    main()
