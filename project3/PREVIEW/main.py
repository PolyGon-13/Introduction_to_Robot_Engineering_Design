#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Merged: sihyun/test/main.py + test/restore_color_area/s_restore_square.py
# Fix applied: Problem 1 — single RPLidarC1 instance (no second lidar open)
# Fix applied: Bug 1 — undistorter.apply() called after read_frame()
# Fix applied: Bug 2 / 차이점 1 — ranges split into obstacle (no staleness) and restore (staleness)
#

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
from square import (
    CALIB_WIDTH,
    CALIB_HEIGHT,
    RESTORE_ARGS,
    SELECTED_COLOR,
    CENTER_COLOR,
    Undistorter,
    GroundProjector,
    load_tuned_intrinsics,
    load_homography,
    restore_target,
    draw_candidate,
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

            # Obstacle avoidance uses any fresh scan; restore applies staleness guard separately
            scan, scan_time, _ = lidar.get()
            ranges = front_ranges(scan) if scan is not None else None
            restore_ranges = ranges if (scan is not None and time.time() - scan_time <= rargs.lidar_max_age) else None

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

            # Restore square overlay — passes restore_ranges, no second RPLidarC1
            if projector is not None and target is not None:
                selected, _, restore_status = restore_target(
                    frame, target, projector, restore_ranges, rargs, previous_restore_center
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
                    ranges = front_ranges(scan) if scan is not None else None
                    restore_ranges = ranges if (scan is not None and time.time() - scan_time <= rargs.lidar_max_age) else None
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
