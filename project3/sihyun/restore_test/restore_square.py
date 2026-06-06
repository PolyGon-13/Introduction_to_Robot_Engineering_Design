#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from pathlib import Path

import cv2
import numpy as np

from camera import (
    BOX_COLORS,
    CAMERA_ROTATION,
    MIN_AREA,
    clamp,
)
from lidar import GRID


THIS_DIR = Path(__file__).resolve().parent
DEFAULT_HOMOGRAPHY_FILE = THIS_DIR / "ground_homography.npz"
DEFAULT_CAMERA_MATRIX_FILE = THIS_DIR / "camera_matrix.npy"
DEFAULT_DIST_COEFFS_FILE = THIS_DIR / "dist_coeffs.npy"

CALIB_WIDTH = 640.0
CALIB_HEIGHT = 480.0

SQUARE_SIZE_M = 0.30
MIN_GROUND_POINTS = 3
MIN_GROUND_SPAN_M = 0.04
MAX_GROUND_SPAN_M = 0.60
COMPLETE_FILL_RATIO = 0.65
COMPLETE_SIZE_MIN_RATIO = 0.70
COMPLETE_SIZE_MAX_RATIO = 1.45
MAX_PROJECTED_AREA_RATIO = 2.0
CENTER_MARGIN_RATIO = 0.15
BORDER_TOUCH_PX = 18
OUT_OF_FRAME_PX = 4
LIDAR_GUIDE_HALF_DEG = 12.0
MIN_RESTORE_SHIFT_M = 0.03
MAX_RESTORE_SHIFT_M = 0.23

SELECTED_COLOR = (255, 255, 255)
CENTER_COLOR = (0, 255, 0)


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
    def __init__(self, homography):
        self.homography = np.asarray(homography, dtype=np.float32)
        self.inverse_homography = np.linalg.inv(self.homography).astype(np.float32)

    def raw_pixels_to_ground(self, raw_points):
        pts = np.asarray(raw_points, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, self.homography).reshape(-1, 2).astype(np.float32)

    def ground_to_raw_pixels(self, ground_points):
        pts = np.asarray(ground_points, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, self.inverse_homography).reshape(-1, 2).astype(np.float32)


def homography_path(args):
    return Path(args.homography_file).expanduser()


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

    return {
        "display_polygon": display_polygon,
        "fill_ratio": contour_fill_ratio(target),
        "ground_size": ground_polygon_dimensions(ground_polygon),
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

    if restore_shift > args.max_restore_shift:
        return None

    angle_deg = ground_bearing_deg(candidate["center"])
    obstacle_d = lidar_min_distance_at(ranges, angle_deg, args.lidar_half_deg)

    if obstacle_d is None or obstacle_d > args.lidar_obstacle_d:
        return None

    supported = dict(candidate)
    supported["lidar_angle"] = angle_deg
    supported["lidar_distance"] = obstacle_d
    return supported


def filter_lidar_supported_candidates(candidates, ranges, observed_center, args):
    supported = []

    for candidate in candidates:
        item = add_lidar_support(candidate, ranges, observed_center, args)

        if item is not None:
            supported.append(item)

    return supported


def filter_near_observed_candidates(candidates, observed_center, args):
    filtered = [
        candidate
        for candidate in candidates
        if float(np.linalg.norm(candidate["center"] - observed_center)) <= args.max_restore_shift
    ]
    return filtered if filtered else candidates


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


def candidate_extends_outside_sides(candidate, projector, frame_shape, observed_display_center, sides, args):
    polygon = project_candidate(candidate, projector, frame_shape)

    if polygon is None:
        return False

    center = candidate_display_center(candidate, projector, frame_shape)

    if center is None:
        return False

    height, width = frame_shape[:2]
    min_x = float(np.min(polygon[:, 0]))
    max_x = float(np.max(polygon[:, 0]))
    min_y = float(np.min(polygon[:, 1]))
    max_y = float(np.max(polygon[:, 1]))
    out_px = float(args.out_of_frame_px)

    if "left" in sides and not (center[0] <= observed_display_center[0] and min_x <= -out_px):
        return False
    if "right" in sides and not (center[0] >= observed_display_center[0] and max_x >= width - 1 + out_px):
        return False
    if "top" in sides and not (center[1] <= observed_display_center[1] and min_y <= -out_px):
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
        candidate
        for candidate in candidates
        if candidate_extends_outside_sides(
            candidate, projector, frame_shape, observed_display_center, sides, args
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


def visible_edge_anchored_candidates(frame, target, projector, square_size):
    polygon = target_display_polygon(target)

    if len(polygon) < 2:
        return []

    raw_polygon = display_to_raw_points(polygon, frame.shape)
    ground_polygon = projector.raw_pixels_to_ground(raw_polygon)
    observed_center = np.mean(ground_polygon, axis=0)
    height, width = frame.shape[:2]
    candidates = []

    for idx in range(len(ground_polygon)):
        d1 = polygon[idx]
        d2 = polygon[(idx + 1) % len(polygon)]
        display_edge = d2 - d1
        display_len = float(np.linalg.norm(display_edge))

        if display_len < 12.0:
            continue

        dx = abs(float(display_edge[0]))
        dy = abs(float(display_edge[1]))

        if dx < dy * 0.55:
            continue

        bottom_touch_count = int(d1[1] >= height - 1 - BORDER_TOUCH_PX) + int(
            d2[1] >= height - 1 - BORDER_TOUCH_PX
        )

        if bottom_touch_count == 2:
            continue

        p1 = ground_polygon[idx]
        p2 = ground_polygon[(idx + 1) % len(ground_polygon)]
        edge = p2 - p1
        length = float(np.linalg.norm(edge))

        if length < square_size * 0.35:
            continue

        direction = edge / length
        p2_fixed = p2
        normal = np.array([-direction[1], direction[0]], dtype=np.float32)
        edge_mid = 0.5 * (p1 + p2_fixed)
        anchor_score = (
            bottom_touch_count,
            float(0.5 * (d1[1] + d2[1])),
            -display_len,
        )

        for sign in (-1.0, 1.0):
            offset = normal * square_size * sign

            if float(np.dot(observed_center - edge_mid, offset)) <= 0.0:
                continue

            corners = np.array([p1, p2_fixed, p2_fixed + offset, p1 + offset], dtype=np.float32)
            center = np.mean(corners, axis=0).astype(np.float32)
            candidates.append(
                {
                    "corners": corners,
                    "center": center,
                    "anchored": True,
                    "anchor_score": anchor_score,
                }
            )

    return candidates


def candidate_moves_toward_frame_center(candidate, projector, frame_shape, observed_display_center):
    center = candidate_display_center(candidate, projector, frame_shape)

    if center is None:
        return False

    _, width = frame_shape[:2]
    frame_center_x = width / 2.0
    observed_to_frame = frame_center_x - observed_display_center[0]
    candidate_shift = center[0] - observed_display_center[0]

    if abs(observed_to_frame) < 1.0:
        return True

    return np.sign(observed_to_frame) * candidate_shift > 0.0


def filter_toward_frame_center(candidates, projector, frame_shape, observed_display_center):
    filtered = [
        candidate
        for candidate in candidates
        if candidate_moves_toward_frame_center(candidate, projector, frame_shape, observed_display_center)
    ]
    return filtered if filtered else candidates


def choose_candidate(candidates, previous_center, observed_center):
    if not candidates:
        return None

    if previous_center is not None:
        return min(
            candidates,
            key=lambda item: (
                0 if item.get("anchored") else 1,
                item.get("anchor_score", (9, 1.0e9, 0.0)),
                float(np.linalg.norm(item["center"] - previous_center)),
            ),
        )

    return min(
        candidates,
        key=lambda item: (
            0 if item.get("anchored") else 1,
            item.get("anchor_score", (9, 1.0e9, 0.0)),
            abs(float(item["center"][1])),
            abs(float(item["center"][0] - observed_center[0])),
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
    filtered = [
        candidate
        for candidate in candidates
        if candidate_is_plausible(candidate, projector, frame_shape, target_area, args)
    ]
    return filtered if filtered else candidates


def filtered_candidate(candidate, previous_center, alpha):
    if previous_center is None:
        return candidate

    if candidate.get("kind") == "image" or candidate.get("anchored"):
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

    pts_display = contour_points(target)
    raw_points = display_to_raw_points(pts_display, frame.shape)
    ground_points = projector.raw_pixels_to_ground(raw_points)

    if len(ground_points) < MIN_GROUND_POINTS:
        return None, [], "few ground points"

    if not ground_shape_is_plausible(ground_points, args):
        return None, [], "implausible span"

    anchored_candidates = visible_edge_anchored_candidates(frame, target, projector, args.square_size)
    candidates = anchored_candidates if anchored_candidates else restore_square_candidates(ground_points, args.square_size)
    observed_center = np.mean(ground_points, axis=0)
    observed_display_center = np.mean(pts_display, axis=0)
    candidates = filter_near_observed_candidates(candidates, observed_center, args)
    candidates = filter_border_candidates(candidates, target, projector, frame.shape, args)
    candidates = filter_plausible_candidates(candidates, projector, frame.shape, target[5], args)
    candidates = filter_toward_frame_center(candidates, projector, frame.shape, observed_display_center)

    if not candidates:
        return None, [], "no plausible square"

    supported = []
    if ranges is not None:
        supported = filter_lidar_supported_candidates(candidates, ranges, observed_center, args)

    if supported:
        selected = choose_candidate(supported, previous_center, observed_center)
        selected = filtered_candidate(selected, previous_center, args.smooth)
        lidar_angle = selected.get("lidar_angle")
        lidar_distance = selected.get("lidar_distance")
        status = f"lidar restored {lidar_angle:.0f}deg {lidar_distance:.2f}m"
        return selected, candidates, status

    # 라이다 확인이 안 돼도 기하학적 최적 후보로 복원 추정을 표시한다.
    selected = choose_candidate(candidates, previous_center, observed_center)
    selected = filtered_candidate(selected, previous_center, args.smooth)
    reason = "no lidar" if ranges is None else "no obstacle"
    return selected, candidates, f"restored geom ({reason})"


def draw_target(frame, target, selected, projector, status):
    name, x, y, w, h, area, cx, cy, box, cnt = target
    color = BOX_COLORS.get(name, (0, 255, 0))

    cv2.drawContours(frame, [cnt], -1, color, 2)
    cv2.polylines(frame, [box], True, color, 1)
    cv2.circle(frame, (int(round(cx)), int(round(cy))), 4, color, -1)

    if selected is not None:
        draw_candidate(frame, selected, projector, SELECTED_COLOR, 3, CENTER_COLOR)

    cv2.putText(
        frame,
        f"{name} area={area} {status}",
        (x, max(24, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
            color,
            2,
        )


def selected_target(found, target_name, min_area):
    found = [item for item in found if item[5] >= min_area]
    targets = found if target_name is None else [item for item in found if item[0] == target_name]
    return max(targets, key=lambda item: item[5], default=None)
