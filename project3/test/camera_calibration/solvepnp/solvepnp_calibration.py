#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

RESTORE_DIR = Path(__file__).resolve().parent.parent.parent / "restore_color_area"
sys.path.insert(0, str(RESTORE_DIR))
import restore_square as rs

THIS_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = THIS_DIR / "ground_homography.npz"

BOARD_COLS = 8
BOARD_ROWS = 8
SQUARE_MM = 21.0
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def build_object_points(cols, rows, square_m):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_m
    return objp


def load_intrinsics(matrix_path, dist_path):
    if not Path(matrix_path).exists() or not Path(dist_path).exists():
        raise RuntimeError(
            f"[PNP] intrinsics not found:\n  {matrix_path}\n  {dist_path}\n"
            "먼저 camera_calibration 의 내부 캘리브레이션을 완료하세요."
        )
    camera_matrix = np.load(str(matrix_path)).astype(np.float64)
    dist_coeffs = np.load(str(dist_path)).astype(np.float64).reshape(-1)
    return camera_matrix, dist_coeffs


def raw_camera_matrix(base_k, frame_shape, calib_w, calib_h):
    """K 를 현재 raw 해상도에 맞게 스케일. (undistort 후 raw 좌표계에서 solvePnP 에 사용)"""
    raw_w, raw_h = rs.frame_raw_size(frame_shape)
    sx = raw_w / calib_w
    sy = raw_h / calib_h
    k = base_k.copy()
    k[0, 0] *= sx
    k[0, 2] *= sx
    k[1, 1] *= sy
    k[1, 2] *= sy
    return k


def ground_frame_from_pose(rvec, tvec):
    """카메라 좌표계 기준 지면 프레임(전방/우측/원점, 높이, 피치)을 계산한다."""
    rot, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)

    normal = rot[:, 2].astype(np.float64)
    if np.dot(normal, np.array([0.0, 1.0, 0.0])) < 0.0:
        normal = -normal

    height = float(np.dot(normal, t))
    foot = height * normal

    optical = np.array([0.0, 0.0, 1.0])
    forward = optical - np.dot(optical, normal) * normal
    forward /= max(np.linalg.norm(forward), 1.0e-9)

    right = np.cross(forward, normal)
    if np.dot(right, np.array([1.0, 0.0, 0.0])) < 0.0:
        right = -right
    right /= max(np.linalg.norm(right), 1.0e-9)

    pitch_down_deg = float(np.degrees(np.arcsin(np.clip(np.dot(optical, normal), -1.0, 1.0))))
    return forward, right, foot, normal, height, pitch_down_deg


def camera_points_to_ground(points_cam, forward, right, foot):
    rel = points_cam - foot[None, :]
    x = rel @ forward
    y = rel @ right
    return np.column_stack((x, y)).astype(np.float32)


def estimate_homography(objp, raw_corners, k):
    ok, rvec, tvec = cv2.solvePnP(objp, raw_corners, k, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    rot, _ = cv2.Rodrigues(rvec)
    points_cam = (rot @ objp.T).T + tvec.reshape(3)
    forward, right, foot, normal, height, pitch = ground_frame_from_pose(rvec, tvec)
    ground_xy = camera_points_to_ground(points_cam, forward, right, foot)

    homography, _ = cv2.findHomography(raw_corners.reshape(-1, 2), ground_xy, 0)
    if homography is None:
        return None

    mapped = cv2.perspectiveTransform(
        raw_corners.reshape(-1, 1, 2), homography
    ).reshape(-1, 2)
    err = float(np.mean(np.linalg.norm(mapped - ground_xy, axis=1)))

    return {
        "homography": np.asarray(homography, dtype=np.float32),
        "rvec": rvec,
        "tvec": tvec,
        "height": height,
        "pitch_deg": pitch,
        "reproj_err_m": err,
        "ground_xy": ground_xy,
    }


def parse_args():
    parser = argparse.ArgumentParser(description="solvePnP ground-plane homography calibrator.")
    parser.add_argument("--board-cols", type=int, default=BOARD_COLS, help="내부 코너 가로 개수")
    parser.add_argument("--board-rows", type=int, default=BOARD_ROWS, help="내부 코너 세로 개수")
    parser.add_argument("--square-mm", type=float, default=SQUARE_MM, help="체커보드 한 칸 크기(mm)")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="저장할 npz 경로")
    parser.add_argument("--camera-matrix-file", default=str(rs.DEFAULT_CAMERA_MATRIX_FILE))
    parser.add_argument("--dist-coeffs-file", default=str(rs.DEFAULT_DIST_COEFFS_FILE))
    parser.add_argument("--no-undistort", action="store_true")
    return parser.parse_args()


def draw_overlay(frame, found, info):
    if found and info is not None:
        lines = [
            f"height={info['height']:.3f} m  pitch={info['pitch_deg']:.1f} deg",
            f"reproj_err={info['reproj_err_m']*1000:.1f} mm",
            "SPACE: save   q: quit",
        ]
        color = (0, 255, 0)
    elif found:
        lines = ["board found but pose failed", "q: quit"]
        color = (0, 0, 255)
    else:
        lines = ["Checkerboard NOT found - place board flat on the floor", "q: quit"]
        color = (0, 0, 255)

    for idx, text in enumerate(lines):
        cv2.putText(frame, text, (10, 28 + 24 * idx),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)


def save_result(path, info, args):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        str(out),
        homography=info["homography"],
        rvec=np.asarray(info["rvec"], dtype=np.float32),
        tvec=np.asarray(info["tvec"], dtype=np.float32),
        camera_height_m=np.float32(info["height"]),
        pitch_down_deg=np.float32(info["pitch_deg"]),
        square_mm=np.float32(args.square_mm),
        source=np.array("solvepnp"),
    )
    print(f"[PNP] saved ground homography: {out}")
    print(f"[PNP] height={info['height']:.3f}m pitch={info['pitch_deg']:.1f}deg "
          f"reproj_err={info['reproj_err_m']*1000:.1f}mm")


def main():
    args = parse_args()
    pattern = (args.board_cols, args.board_rows)
    square_m = args.square_mm / 1000.0
    objp = build_object_points(args.board_cols, args.board_rows, square_m)

    base_k, dist = load_intrinsics(args.camera_matrix_file, args.dist_coeffs_file)
    undistorter = None
    if not args.no_undistort:
        undistorter = rs.Undistorter(base_k, dist, rs.CALIB_WIDTH, rs.CALIB_HEIGHT)
        print("[PNP] lens undistortion enabled")

    cam = None
    try:
        cam = rs.open_camera()
        print("[PNP] place an 8x8(inner) 21mm checkerboard flat on the floor.")

        while True:
            ok, frame = rs.read_frame(cam)
            if not ok:
                break

            if undistorter is not None:
                frame = undistorter.apply(frame)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray, pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )

            info = None
            if found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)
                cv2.drawChessboardCorners(frame, pattern, corners, found)
                raw_corners = rs.display_to_raw_points(corners, frame.shape)
                k = raw_camera_matrix(base_k, frame.shape, rs.CALIB_WIDTH, rs.CALIB_HEIGHT)
                info = estimate_homography(objp, raw_corners.astype(np.float32), k)

            draw_overlay(frame, found, info)
            cv2.imshow("calibrate_ground_pnp", frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key == ord(" ") and info is not None:
                save_result(args.output, info, args)
                break

    finally:
        rs.close_camera(cam)


if __name__ == "__main__":
    main()


