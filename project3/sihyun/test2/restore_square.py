#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import time

import cv2
import numpy as np

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

CALIB_WIDTH = 640.0
CALIB_HEIGHT = 480.0
CALIB_FX = 700.0
CALIB_FY = 700.0
CALIB_CX = 320.0
CALIB_CY = 240.0

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
GROUND_FORWARD_SCALE = 0.375
GROUND_SIDE_SCALE = 0.75

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


class GroundProjector:
    def __init__(self, height_m, pitch_down_deg, fx, fy, cx, cy, forward_scale=1.0, side_scale=1.0):
        self.height_m = float(height_m)
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.forward_scale = float(forward_scale)
        self.side_scale = float(side_scale)

        theta = math.radians(pitch_down_deg)
        self.cam_right = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        self.cam_down = np.array([-math.sin(theta), 0.0, math.cos(theta)], dtype=np.float32)
        self.cam_forward = np.array([math.cos(theta), 0.0, math.sin(theta)], dtype=np.float32)

    def raw_pixels_to_ground(self, raw_points):
        pts = np.asarray(raw_points, dtype=np.float32).reshape(-1, 2)
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

    try:
        cam = open_camera()

        if not args.no_lidar:
            lidar = RPLidarC1()

        while True:
            ok, frame = read_frame(cam)

            if not ok:
                break

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
