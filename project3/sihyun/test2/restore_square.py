#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math

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
MIN_AREA = 200
MIN_GROUND_POINTS = 3

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
    def __init__(self, height_m, pitch_down_deg, fx, fy, cx, cy):
        self.height_m = float(height_m)
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)

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
        return ground.astype(np.float32)

    def ground_to_raw_pixels(self, ground_points):
        pts = np.asarray(ground_points, dtype=np.float32).reshape(-1, 2)
        vecs = np.column_stack(
            (
                pts[:, 0],
                pts[:, 1],
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
        return min(candidates, key=lambda item: float(np.linalg.norm(item["center"] - previous_center)))

    return min(
        candidates,
        key=lambda item: (
            abs(float(item["center"][1])),
            abs(float(item["center"][0] - observed_center[0])),
        ),
    )


def project_candidate(candidate, projector, frame_shape):
    raw = projector.ground_to_raw_pixels(candidate["corners"])
    display = raw_to_display_points(raw, frame_shape)

    if np.isnan(display).any():
        return None

    return np.rint(display).astype(np.int32)


def filtered_candidate(candidate, previous_center, alpha):
    if previous_center is None:
        return candidate

    center = alpha * previous_center + (1.0 - alpha) * candidate["center"]
    shift = center - candidate["center"]
    return {
        "center": center.astype(np.float32),
        "corners": (candidate["corners"] + shift[None, :]).astype(np.float32),
    }


def draw_cross(frame, point, color):
    x, y = int(round(point[0])), int(round(point[1]))
    cv2.line(frame, (x - 8, y), (x + 8, y), color, 2)
    cv2.line(frame, (x, y - 8), (x, y + 8), color, 2)


def draw_candidate(frame, candidate, projector, color, thickness):
    polygon = project_candidate(candidate, projector, frame.shape)

    if polygon is None:
        return False

    cv2.polylines(frame, [polygon], True, color, thickness)

    raw_center = projector.ground_to_raw_pixels(candidate["center"][None, :])
    center = raw_to_display_points(raw_center, frame.shape)[0]

    if not np.isnan(center).any():
        draw_cross(frame, center, CENTER_COLOR)

    return True


def restore_target(frame, target, projector, args, previous_center):
    pts_display = contour_points(target)
    raw_points = display_to_raw_points(pts_display, frame.shape)
    ground_points = projector.raw_pixels_to_ground(raw_points)

    if len(ground_points) < MIN_GROUND_POINTS:
        return None, [], "few ground points"

    candidates = restore_square_candidates(ground_points, args.square_size)

    if not candidates:
        return None, [], "no square candidate"

    observed_center = np.mean(ground_points, axis=0)
    selected = choose_candidate(candidates, previous_center, observed_center)
    selected = filtered_candidate(selected, previous_center, args.smooth)
    return selected, candidates, "restored"


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
    parser.add_argument("--target", choices=("ALL",) + TARGET_NAMES, default="ALL")
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
    parser.add_argument("--smooth", type=float, default=0.65)
    parser.add_argument("--show-candidates", action="store_true")
    return parser.parse_args()


def selected_targets(found, target_name, min_area):
    found = [item for item in found if item[5] >= min_area]

    if target_name != "ALL":
        target = pick(found, target_name)
        return [] if target is None else [target]

    targets = []
    for name in TARGET_NAMES:
        target = pick(found, name)

        if target is not None:
            targets.append(target)

    return targets


def main():
    args = parse_args()
    args.smooth = clamp(args.smooth, 0.0, 0.95)
    cam = None
    previous_centers = {}

    try:
        cam = open_camera()

        while True:
            ok, frame = read_frame(cam)

            if not ok:
                break

            fx, fy, cx, cy = scaled_intrinsics(frame.shape, args)
            projector = GroundProjector(args.height, args.pitch_down, fx, fy, cx, cy)
            targets = selected_targets(detect(frame), args.target, args.min_area)
            active_names = set()

            for target in targets:
                name = target[0]
                previous = previous_centers.get(name)
                selected, candidates, status = restore_target(frame, target, projector, args, previous)

                if selected is not None:
                    previous_centers[name] = selected["center"]
                    active_names.add(name)

                draw_target(frame, target, selected, candidates, projector, status, args)

            for name in list(previous_centers):
                if name not in active_names:
                    previous_centers.pop(name, None)

            cv2.putText(
                frame,
                "q: quit",
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
        close_camera(cam)


if __name__ == "__main__":
    main()
