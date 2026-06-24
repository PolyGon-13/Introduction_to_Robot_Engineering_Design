#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import threading
import time
import sys
from pathlib import Path

import numpy as np
import serial

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from camera import (
    ALIGN_KP,
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
    AVOID_MAX_W,
    AVOID_TURN_GAIN,
    AVOID_TURN_SIGN,
    COLOR_TO_LIDAR_DEG,
    FREE_D,
    FRONT_LOG_ZONE,
    LEFT_ZONE,
    OPPOSITE_WALL_GAIN,
    RPLidarC1,
    RIGHT_ZONE,
    SIDE_CLEAR_D,
    SIDE_CORRECT_MAX_DEG,
    avoid_cmd,
    front_ranges,
    lidar_zone_distances,
    obstacle_detected,
    zone_min_distance,
)

TARGET_SEQUENCE = ("RED", "YELLOW", "BLUE")
log_control = False
SHOW_WINDOW = log_control
DEBUG = log_control
BOTTOM_LOST_RATIO = 0.90
COLOR_SWITCH_PAUSE_MS = 1000
COLOR_FORWARD_CENTER_RATIO = 0.95
COLOR_EXIT_CENTER_ERR = 0.15

ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600
DRIVE_V = 0.25
V_STEP = 0.04
FOLLOW_W_STEP = 0.25
AVOID_W_STEP = 0.20
LOOP_DT = 0.05
SEARCH_MAX_W = 1.0
SWITCH_SEARCH_W = -1.5
ODOM_LOG_INTERVAL = 0.5
WHEEL_R = 0.034
ENC_PPR = 1012.0
ENC_COUNTS_PER_M = ENC_PPR / (2.0 * np.pi * WHEEL_R)
POST_COLOR_FORWARD_M = 0.02
TURN_360_WHEEL_BASE_M = 0.18
TURN_360_COUNTS = np.pi * TURN_360_WHEEL_BASE_M * ENC_COUNTS_PER_M
AVOID_STOP_D = 0.15
SPIRAL_GROWTH = 0.1
SPIRAL_MAX_RADIUS = 1.5
SPIRAL_LOOKAHEAD_M = 0.15
SPIRAL_MAX_ADVANCE = 0.5
SPIRAL_HEADING_KP = 1.5
SPIRAL_SKIP_ON_BLOCK = 1.0
SPIRAL_AVOID_ADVANCE = 0.025
RETURN_KP = 1.0
RETURN_DONE_M = 0.10
EXPLORE_MAX_W = 1.0
EXPLORE_TURN_SIGN = 1.0
EXPLORE_AVOID_D = 0.28

def dbg(*args, **kwargs):
    """DEBUG가 True일 때만 출력한다(디버그 로그 토글)."""
    if DEBUG:
        print(*args, **kwargs)

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

class NullWriter:
    """log_control이 False일 때 모든 print 출력을 버리는 stdout 대체."""

    def write(self, data):
        pass

    def flush(self):
        pass

class SpiralExplorer:
    """달팽이집(아르키메데스 나선)으로 탐색한다.
    오도메트리로 추적한 위치(x, y)를 기준으로, 원점(0,0)에 중심이 고정된
    나선 경로 r = SPIRAL_GROWTH·theta 위의 전방주시 목표점을 추종한다.
    반경은 SPIRAL_MAX_RADIUS(m)에서 멈추고, 회피로 밀려나도 원래 중심 나선으로 복귀한다."""

    def __init__(self, enc_l, enc_r, turn_sign=EXPLORE_TURN_SIGN):
        self.prev_l = enc_l
        self.prev_r = enc_r
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.spiral_theta = 0.0
        self.shrinking = False
        self.avoid_anchor = None
        self.turn_sign = turn_sign
        self.detail = ""

    def update_pose(self, enc_l, enc_r):
        """엔코더 증분으로 원점 기준 위치(x, y)와 헤딩(theta)을 적산한다."""
        dL = (enc_l - self.prev_l) / ENC_COUNTS_PER_M
        dR = (enc_r - self.prev_r) / ENC_COUNTS_PER_M
        self.prev_l = enc_l
        self.prev_r = enc_r
        d_center = 0.5 * (dL + dR)
        d_theta = (dR - dL) / TURN_360_WHEEL_BASE_M
        self.theta += d_theta
        self.x += d_center * np.cos(self.theta)
        self.y += d_center * np.sin(self.theta)

    def save_avoid_anchor(self):
        if self.avoid_anchor is not None:
            return

        step = -SPIRAL_SKIP_ON_BLOCK if self.shrinking else SPIRAL_SKIP_ON_BLOCK
        resume_theta = max(0.0, self.spiral_theta + step)
        tx, ty, radius = self.spiral_point(resume_theta)
        self.avoid_anchor = {
            "x": tx,
            "y": ty,
            "spiral_theta": resume_theta,
            "radius": radius,
        }

    def clear_avoid_anchor(self):
        self.avoid_anchor = None

    def has_avoid_anchor(self):
        return self.avoid_anchor is not None

    def return_to_avoid_anchor_command(self, ranges):
        if self.avoid_anchor is None:
            return "EXPLORE: no anchor", 0.0, 0.0

        dx = self.avoid_anchor["x"] - self.x
        dy = self.avoid_anchor["y"] - self.y
        dist = float(np.hypot(dx, dy))
        target_heading = np.arctan2(dy, dx)
        err = (target_heading - self.theta + np.pi) % (2.0 * np.pi) - np.pi
        target_w = clamp(RETURN_KP * err, -EXPLORE_MAX_W, EXPLORE_MAX_W)
        target_w = apply_symmetric_side_repulsion(target_w, ranges)
        nominal_v = DRIVE_V * max(0.0, 1.0 - abs(err) / (np.pi / 2.0))
        target_v = scale_avoid_speed_for_front_obstacle(nominal_v, ranges)
        self.detail = (
            f"anchor=({self.avoid_anchor['x']:+.2f},{self.avoid_anchor['y']:+.2f}) "
            f"pos=({self.x:+.2f},{self.y:+.2f}) d={dist:.2f}m "
            f"radius={self.avoid_anchor['radius']:.2f}m "
            f"err={np.degrees(err):+.0f} v={target_v:.3f} w={target_w:+.3f}"
        )
        return f"RETURN: spiral anchor d={dist:.2f}m", target_v, target_w

    def reached_avoid_anchor(self):
        if self.avoid_anchor is None:
            return False

        dist = float(np.hypot(self.x - self.avoid_anchor["x"], self.y - self.avoid_anchor["y"]))
        return dist <= RETURN_DONE_M

    def resume_from_avoid_anchor(self):
        if self.avoid_anchor is None:
            return

        self.spiral_theta = self.avoid_anchor["spiral_theta"]
        self.clear_avoid_anchor()

    def grow_spiral_while_avoiding(self):
        """장애물 회피·앵커 복귀 중에도 나선을 정상의 1/2 속도로 진행시킨다(진행방향 따라
        반경 증가/감소). 앵커도 같이 드리프트시켜 같은 막힌 영역에 갇히지 않게 한다."""
        step = -SPIRAL_AVOID_ADVANCE if self.shrinking else SPIRAL_AVOID_ADVANCE
        self.spiral_theta = max(0.0, self.spiral_theta + step)
        if self.avoid_anchor is not None:
            resume_theta = max(0.0, self.avoid_anchor["spiral_theta"] + step)
            tx, ty, radius = self.spiral_point(resume_theta)
            self.avoid_anchor = {
                "x": tx,
                "y": ty,
                "spiral_theta": resume_theta,
                "radius": radius,
            }

    def spiral_point(self, theta):
        """원점(0,0) 중심 아르키메데스 나선 위의 점. r = b·theta, 최대 반경 제한."""
        r = min(SPIRAL_GROWTH * theta, SPIRAL_MAX_RADIUS)
        ang = self.turn_sign * theta
        return r * np.cos(ang), r * np.sin(ang), r

    def command(self, enc_l, enc_r, ranges):
        step = -0.05 if self.shrinking else 0.05
        theta = self.spiral_theta
        advanced = 0.0
        tx, ty, radius = self.spiral_point(theta)
        while np.hypot(tx - self.x, ty - self.y) < SPIRAL_LOOKAHEAD_M and advanced < SPIRAL_MAX_ADVANCE:
            theta += step
            advanced += 0.05
            if self.shrinking and theta <= 0.0:
                theta = 0.0
                break
            tx, ty, radius = self.spiral_point(theta)
        self.spiral_theta = theta

        dist_origin = float(np.hypot(self.x, self.y))
        if not self.shrinking and dist_origin >= SPIRAL_MAX_RADIUS:
            self.shrinking = True
            self.turn_sign = -self.turn_sign
        elif self.shrinking and dist_origin <= RETURN_DONE_M:
            self.shrinking = False
            self.turn_sign = -self.turn_sign
            self.spiral_theta = 0.0

        desired_heading = np.arctan2(ty - self.y, tx - self.x)
        err = (desired_heading - self.theta + np.pi) % (2.0 * np.pi) - np.pi
        target_w = clamp(SPIRAL_HEADING_KP * err, -EXPLORE_MAX_W, EXPLORE_MAX_W)
        target_v = DRIVE_V * max(0.0, 1.0 - abs(err) / (np.pi / 2.0))

        phase = "in" if self.shrinking else "out"
        self.detail = (
            f"pos=({self.x:+.2f},{self.y:+.2f}) tgt=({tx:+.2f},{ty:+.2f}) phase={phase} "
            f"dist={dist_origin:.2f}m sp_theta={np.degrees(theta):.0f}deg radius={radius:.3f}m "
            f"err={np.degrees(err):+.0f} v={target_v:.3f} w={target_w:+.3f}"
        )
        mode = f"EXPLORE: spiral {phase} d={dist_origin:.2f}m"
        return mode, target_v, target_w

class Motor:
    def __init__(self):
        self.ser = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.running = True
        self.odom_l = 0
        self.odom_r = 0
        self.odom_ms = 0
        self.odom_time = 0.0
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

                    enc_l = int(parts[1])
                    enc_r = int(parts[2])
                    arduino_ms = int(parts[3])

                    with self.lock:
                        self.odom_l = enc_l
                        self.odom_r = enc_r
                        self.odom_ms = arduino_ms
                        self.odom_time = time.time()
                elif line:
                    print(f"[ARDUINO] {line}")
            except (ValueError, serial.SerialException, OSError):
                time.sleep(0.05)

    def vw(self, v, w):
        with self.write_lock:
            self.ser.write(f"V{v:.3f},{w:.3f}\n".encode("ascii"))

    def pivot(self, w):
        with self.write_lock:
            self.ser.write(f"P{w:.3f}\n".encode("ascii"))

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

def color_angle_from_target(target, frame_shape):
    if target is None:
        return 0.0

    err = bottom_center_error(target, frame_shape)
    return clamp(err * COLOR_TO_LIDAR_DEG, ANG_MIN, ANG_MAX)

def update_color_memory(target, frame):
    last_color_deg = color_angle_from_target(target, frame.shape)
    center_ratio = float(target[7]) / frame.shape[0]
    return last_color_deg, time.time(), center_ratio, x_center_error(target, frame.shape)

def clear_color_memory():
    return 0.0, 0.0, 0.0, 0.0

def search_next_cmd():
    return "SEARCH: next color", 0.0, SWITCH_SEARCH_W

def obstacle_in_zone(ranges, zone, clear_d=FREE_D):
    if ranges is None:
        return False

    return zone_min_distance(ranges, zone) < clear_d

def color_path_clear(ranges, color_deg, clear_d=FREE_D):
    if ranges is None:
        return False

    color_side_zone = RIGHT_ZONE if color_deg >= 0.0 else LEFT_ZONE
    front_clear = not obstacle_in_zone(ranges, FRONT_LOG_ZONE, clear_d)
    side_clear = not obstacle_in_zone(ranges, color_side_zone, clear_d)
    return front_clear and side_clear

def apply_color_side_repulsion(target_w, ranges, color_deg):
    if ranges is None:
        return target_w

    left_d = zone_min_distance(ranges, LEFT_ZONE)
    right_d = zone_min_distance(ranges, RIGHT_ZONE)
    left_risk = max(0.0, SIDE_CLEAR_D - left_d)
    right_risk = max(0.0, SIDE_CLEAR_D - right_d)
    if color_deg >= 0.0:
        side_correct_deg = OPPOSITE_WALL_GAIN * left_risk / SIDE_CLEAR_D * SIDE_CORRECT_MAX_DEG
    else:
        side_correct_deg = -OPPOSITE_WALL_GAIN * right_risk / SIDE_CLEAR_D * SIDE_CORRECT_MAX_DEG
    side_correct_deg = clamp(side_correct_deg, -SIDE_CORRECT_MAX_DEG, SIDE_CORRECT_MAX_DEG)
    repel_w = AVOID_TURN_SIGN * AVOID_TURN_GAIN * np.deg2rad(side_correct_deg)
    return clamp(target_w + repel_w, -AVOID_MAX_W, AVOID_MAX_W)

def apply_symmetric_side_repulsion(target_w, ranges):
    """복귀 주행용 대칭 측면 반발 조향. 좌/우 벽이 SIDE_CLEAR_D 안으로 들어오면
    가까운 쪽에서 멀어지는 방향으로 회전량을 더한다(양쪽 위험 차이로 보정)."""
    if ranges is None:
        return target_w

    left_d = zone_min_distance(ranges, LEFT_ZONE)
    right_d = zone_min_distance(ranges, RIGHT_ZONE)
    left_risk = max(0.0, SIDE_CLEAR_D - left_d)
    right_risk = max(0.0, SIDE_CLEAR_D - right_d)
    side_correct_deg = (left_risk - right_risk) / SIDE_CLEAR_D * SIDE_CORRECT_MAX_DEG
    side_correct_deg = clamp(side_correct_deg, -SIDE_CORRECT_MAX_DEG, SIDE_CORRECT_MAX_DEG)
    repel_w = AVOID_TURN_SIGN * AVOID_TURN_GAIN * np.deg2rad(side_correct_deg)
    return clamp(target_w + repel_w, -AVOID_MAX_W, AVOID_MAX_W)

def front_obstacle_distance(ranges):
    if ranges is None:
        return FREE_D

    return zone_min_distance(ranges, FRONT_LOG_ZONE)

def scale_speed_for_obstacle(target_v, dist):
    """장애물 거리(dist)에 따라 속도를 선형 감속한다. AVOID_STOP_D 이내면 0."""
    if dist <= AVOID_STOP_D:
        return 0.0
    if dist >= FREE_D:
        return target_v

    scale = (dist - AVOID_STOP_D) / (FREE_D - AVOID_STOP_D)
    return target_v * clamp(scale, 0.0, 1.0)

def scale_avoid_speed_for_front_obstacle(target_v, ranges):
    return scale_speed_for_obstacle(target_v, front_obstacle_distance(ranges))

def lost_color_obstacle_passed(ranges, last_color_deg):
    if last_color_deg >= 0.0:
        color_side_zone = RIGHT_ZONE
    else:
        color_side_zone = LEFT_ZONE

    front_clear = not obstacle_in_zone(ranges, FRONT_LOG_ZONE, FREE_D)
    side_clear = not obstacle_in_zone(ranges, color_side_zone, SIDE_CLEAR_D)
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
    left_counts = abs(enc_l - start_l)
    right_counts = abs(enc_r - start_r)
    return 0.5 * (left_counts + right_counts) >= TURN_360_COUNTS

def drive_forward_by_encoder(motor, distance_m=POST_COLOR_FORWARD_M, start_v=0.0):
    while True:
        odom = motor.get_odom()
        if odom[3] != 0.0:
            break
        wait_ms(int(LOOP_DT * 1000))

    start_l, start_r, _, _ = odom
    target_counts = abs(distance_m) * ENC_COUNTS_PER_M
    forward_v = abs(DRIVE_V) if distance_m >= 0.0 else -abs(DRIVE_V)

    print(f"[ODOM] move {distance_m:.2f}m target_counts={target_counts:.0f}")

    left_counts = right_counts = 0.0
    current_v = start_v
    while True:
        enc_l, enc_r, _, odom_time = motor.get_odom()
        if odom_time == 0.0 or time.time() - odom_time > 0.5:
            motor.stop()
            print("[ODOM] encoder data timeout during extra forward move")
            return False

        left_counts = abs(enc_l - start_l)
        right_counts = abs(enc_r - start_r)
        if left_counts >= target_counts and right_counts >= target_counts:
            break

        current_v = rate_limit(current_v, forward_v, V_STEP)
        motor.vw(current_v, 0.0)
        wait_ms(int(LOOP_DT * 1000))

    motor.stop()
    left_m = left_counts / ENC_COUNTS_PER_M
    right_m = right_counts / ENC_COUNTS_PER_M
    ok = left_counts >= target_counts and right_counts >= target_counts
    print(
        f"[ODOM] extra forward done: "
        f"left={left_m:.2f}m({left_counts:.0f}) "
        f"right={right_m:.2f}m({right_counts:.0f})"
    )
    return ok

def rotate_180_by_encoder(motor, cam=None, lidar=None, current_target=None, turn_sign=1.0, w=SWITCH_SEARCH_W):
    """제자리에서 엔코더 기준 180도 회전한다.
    좌/우 바퀴 누적 카운트 평균이 360도(TURN_360_COUNTS)의 절반에 도달하면 멈춘다.
    회전 도중 카메라로 추적색을 발견하거나 라이다로 장애물을 감지하면 즉시 멈춘다.
    반환값: "color"(색 발견), "obstacle"(장애물), "done"(180도 완료), "timeout"(엔코더 끊김)."""
    while True:
        odom = motor.get_odom()
        if odom[3] != 0.0:
            break
        wait_ms(int(LOOP_DT * 1000))

    start_l, start_r, _, _ = odom
    target_counts = TURN_360_COUNTS / 2.0
    target_w = turn_sign * abs(w)
    print(f"[ODOM] rotate 180deg target_counts={target_counts:.0f}")

    current_w = 0.0
    while True:
        if cam is not None and current_target is not None:
            ok, frame = read_frame(cam)
            target = pick(detect(frame), current_target) if ok else None
            if SHOW_WINDOW and ok:
                draw(frame, [target] if target is not None else [], "TURN 180")
            if target is not None:
                motor.stop()
                print("[ODOM] rotate 180 stop: color found")
                return "color"

        if lidar is not None:
            scan, _, _ = lidar.get()
            ranges = front_ranges(scan) if scan is not None else None
            if obstacle_detected(ranges):
                motor.stop()
                print("[ODOM] rotate 180 stop: obstacle")
                return "obstacle"

        enc_l, enc_r, _, odom_time = motor.get_odom()
        if odom_time == 0.0 or time.time() - odom_time > 0.5:
            motor.stop()
            print("[ODOM] encoder data timeout during 180 rotate")
            return "timeout"

        left_counts = abs(enc_l - start_l)
        right_counts = abs(enc_r - start_r)
        if 0.5 * (left_counts + right_counts) >= target_counts:
            break

        current_w = rate_limit(current_w, target_w, AVOID_W_STEP)
        motor.vw(0.0, current_w)
        wait_ms(int(LOOP_DT * 1000))

    motor.stop()
    print("[ODOM] rotate 180 done")
    return "done"

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

def log_avoid(elapsed, tag, target_deg, target_v, target_w, gap_count, gap_clear):
    clear_str = "open" if gap_clear == float("inf") else f"{gap_clear:.2f}m"
    print(
        f"[{elapsed:.2f}s] [{tag}] gap={gap_count} "
        f"clear={clear_str} "
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
            target_w = apply_color_side_repulsion(target_w, ranges, color_deg)
            return "FOLLOW: color path clear + repulse", target_v, target_w

        target_v, target_w, target_deg, gap_count, gap_clear = avoid_cmd(ranges, color_deg, DRIVE_V, color_follow=True)
        target_v = scale_avoid_speed_for_front_obstacle(target_v, ranges)
        log_avoid(elapsed, "AVOID", target_deg, target_v, target_w, gap_count, gap_clear)
        return "AVOID: color + obstacle", target_v, target_w

    target_v, target_w = follow_cmd(target, frame.shape, DRIVE_V)
    target_w = apply_color_side_repulsion(target_w, ranges, color_deg)
    return "FOLLOW: color only", target_v, target_w

def bottom_color_cmd(target, frame):
    if abs(x_center_error(target, frame.shape)) > COLOR_EXIT_CENTER_ERR:
        _, align_w = follow_cmd(target, frame.shape, DRIVE_V, ALIGN_KP)
        return "ALIGN: bottom color", 0.0, align_w

    target_v, target_w = follow_cmd(target, frame.shape, DRIVE_V)
    return "FOLLOW: bottom color", target_v, target_w

def search_last_cmd(last_color_deg):
    w = -SEARCH_MAX_W if last_color_deg >= 0.0 else SEARCH_MAX_W
    return "SEARCH: last color direction", 0.0, w

def avoid_mode(mode, tag, ranges, color_deg, elapsed):
    target_v, target_w, target_deg, gap_count, gap_clear = avoid_cmd(ranges, color_deg, DRIVE_V)
    target_v = scale_avoid_speed_for_front_obstacle(target_v, ranges)
    log_avoid(elapsed, tag, target_deg, target_v, target_w, gap_count, gap_clear)
    return mode, target_v, target_w

def main():
    cam = lidar = motor = None
    original_stdout = sys.stdout
    if not log_control:
        sys.stdout = NullWriter()
        os.environ["LIBCAMERA_LOG_LEVELS"] = "*:4"
    last_v = last_w = 0.0
    last_color_deg = last_color_time = last_color_center_ratio = last_color_x_err = 0.0
    last_odom_log_time = 0.0
    color_lost_during_avoid = False
    explore_active = False
    explorer = None
    switch_search_active = False
    switch_search_start_odom = None
    last_search_start_odom = None
    target_index = 0
    current_target = TARGET_SEQUENCE[target_index]

    try:
        cam = open_camera()
        lidar = RPLidarC1()
        motor = Motor()
        motor.stop()
        motor.reset_encoders()
        if log_control:
            original_stdout.write("[READY] Press Enter to start...")
            original_stdout.flush()

        if SHOW_WINDOW:
            start_pressed = threading.Event()

            def wait_enter():
                input()
                start_pressed.set()

            threading.Thread(target=wait_enter, daemon=True).start()
            while not start_pressed.is_set():
                ok, frame = read_frame(cam)
                if ok:
                    found = detect(frame)
                    target = pick(found, current_target)
                    found = [target] if target is not None else []
                    if draw(frame, found, "READY: press Enter") == ord("q"):
                        return
                time.sleep(LOOP_DT)
        else:
            input()

        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            ok, frame = read_frame(cam)
            if not ok:
                break

            found = detect(frame)
            target = pick(found, current_target)
            found = [target] if target is not None else []
            scan, _, _ = lidar.get()
            ranges = front_ranges(scan) if scan is not None else None
            lidar_dist = lidar_zone_distances(ranges)
            has_obstacle = obstacle_detected(ranges)

            if target is not None:
                last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = update_color_memory(target, frame)
                switch_search_active, explore_active, switch_search_start_odom = False, False, None
                explorer = None

            if DEBUG and time.time() - last_odom_log_time >= ODOM_LOG_INTERVAL:
                log_odom(elapsed, motor.get_odom())
                last_odom_log_time = time.time()

            if DEBUG:
                log_lidar(elapsed, lidar_dist)

            color_in_forward_zone = (
                target is not None
                and last_color_center_ratio >= COLOR_FORWARD_CENTER_RATIO
                and abs(last_color_x_err) <= COLOR_EXIT_CENTER_ERR
            )
            color_in_bottom_zone = target is not None and last_color_center_ratio >= BOTTOM_LOST_RATIO
            color_exited_bottom = target is None and last_color_time > 0.0 and last_color_center_ratio >= BOTTOM_LOST_RATIO
            if color_in_forward_zone:
                if target_index + 1 < len(TARGET_SEQUENCE):
                    prev_target = current_target
                    target_index += 1
                    current_target = TARGET_SEQUENCE[target_index]
                    mode = f"SWITCH: {prev_target}->{current_target}"
                    print(f"[{elapsed:.2f}s] [COLOR] {prev_target} done, now tracking {current_target}")
                    drive_forward_by_encoder(motor, start_v=last_v)
                    target_v, target_w, last_v, last_w = 0.0, 0.0, 0.0, 0.0
                    wait_ms(COLOR_SWITCH_PAUSE_MS)
                    scan, _, _ = lidar.get()
                    ranges = front_ranges(scan) if scan is not None else None
                    lidar_dist = lidar_zone_distances(ranges)
                    has_obstacle = obstacle_detected(ranges)
                    ok, frame = read_frame(cam)
                    if not ok:
                        break

                    found = detect(frame)
                    target = pick(found, current_target)
                    found = [target] if target is not None else []

                    if target is not None:
                        last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = update_color_memory(target, frame)
                        color_lost_during_avoid = False
                        explore_active, explorer = False, None
                        if last_color_center_ratio >= BOTTOM_LOST_RATIO:
                            mode, target_v, target_w = bottom_color_cmd(target, frame)
                        else:
                            mode, target_v, target_w = color_cmd(
                                target, frame, ranges, has_obstacle, last_color_deg, elapsed
                            )
                    else:
                        mode, target_v, target_w = search_next_cmd()
                        last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = clear_color_memory()
                        color_lost_during_avoid, switch_search_active = False, True
                        explore_active, explorer = False, None
                        switch_search_start_odom = get_turn_start_odom(motor)
                else:
                    mode = "STOP: color bottom"
                    drive_forward_by_encoder(motor, start_v=last_v)
                    target_v, target_w, last_v, last_w = 0.0, 0.0, 0.0, 0.0
                    last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = clear_color_memory()
                    color_lost_during_avoid, switch_search_active = False, False
                    explore_active, switch_search_start_odom = False, None
                    explorer = None
                    print(f"[{elapsed:.2f}s] [COLOR] {current_target} done, mission complete")
                    break

            elif target is not None:
                color_lost_during_avoid = False
                explore_active, explorer = False, None
                if color_in_bottom_zone:
                    mode, target_v, target_w = bottom_color_cmd(target, frame)
                else:
                    mode, target_v, target_w = color_cmd(
                        target, frame, ranges, has_obstacle, last_color_deg, elapsed
                    )

            elif target is None and explore_active:
                enc_l, enc_r, _, odom_time = motor.get_odom()
                if odom_time > 0.0:
                    explorer.update_pose(enc_l, enc_r)

                explore_blocked = (
                    front_obstacle_distance(ranges) <= EXPLORE_AVOID_D
                    or obstacle_in_zone(ranges, LEFT_ZONE, EXPLORE_AVOID_D)
                    or obstacle_in_zone(ranges, RIGHT_ZONE, EXPLORE_AVOID_D)
                )

                if explore_blocked:
                    explorer.save_avoid_anchor()
                    explorer.grow_spiral_while_avoiding()
                    mode, target_v, target_w = avoid_mode("AVOID: explore", "AVOID_EXPLORE", ranges, None, elapsed)
                elif explorer.has_avoid_anchor():
                    if explorer.reached_avoid_anchor():
                        explorer.resume_from_avoid_anchor()
                        mode, target_v, target_w = "EXPLORE: resume spiral", 0.0, 0.0
                        dbg(f"[{elapsed:.2f}s] [EXPLORE] resumed at saved spiral radius")
                    elif odom_time > 0.0:
                        explorer.grow_spiral_while_avoiding()
                        mode, target_v, target_w = explorer.return_to_avoid_anchor_command(ranges)
                        dbg(f"[{elapsed:.2f}s] [ANCHOR] {explorer.detail}")
                    else:
                        mode, target_v, target_w = "RETURN: wait odom", 0.0, 0.0
                elif odom_time > 0.0:
                    mode, target_v, target_w = explorer.command(enc_l, enc_r, ranges)
                    dbg(f"[{elapsed:.2f}s] [EXPLORE] {explorer.detail}")
                else:
                    mode, target_v, target_w = "EXPLORE: wait odom", 0.0, 0.0
                    dbg(f"[{elapsed:.2f}s] [EXPLORE] odom 대기중 (v=0 w=0)")

            elif color_exited_bottom:
                color_lost_during_avoid = True
                switch_search_active = False
                if not lost_color_obstacle_passed(ranges, last_color_deg):
                    mode, target_v, target_w = avoid_mode("AVOID: lost color", "AVOID_LOST", ranges, last_color_deg, elapsed)
                else:
                    mode, target_v, target_w = search_last_cmd(last_color_deg)

            elif target is None and last_color_time > 0.0 and not lost_color_obstacle_passed(ranges, last_color_deg):
                color_lost_during_avoid = True
                mode, target_v, target_w = avoid_mode("AVOID: lost color", "AVOID_LOST", ranges, last_color_deg, elapsed)

            elif target is None and switch_search_active:
                if switch_search_start_odom is None:
                    switch_search_start_odom = get_turn_start_odom(motor)

                if completed_one_encoder_turn(motor, switch_search_start_odom):
                    switch_search_active = False
                    switch_search_start_odom = None
                    last_v, last_w = 0.0, 0.0
                    turn_result = rotate_180_by_encoder(motor, cam, lidar, current_target)
                    if turn_result == "color":
                        explore_active, explorer = False, None
                        print(f"[{elapsed:.2f}s] [EXPLORE] 180 turn aborted: color found, follow")
                        continue
                    explore_active = True
                    enc_l, enc_r, _, _ = motor.get_odom()
                    explorer = SpiralExplorer(enc_l, enc_r)
                    print(f"[{elapsed:.2f}s] [EXPLORE] 360 search failed, 180 turn ({turn_result}), spiral explore (max r={SPIRAL_MAX_RADIUS}m)")
                    mode, target_v, target_w = explorer.command(enc_l, enc_r, ranges)
                    dbg(f"[{elapsed:.2f}s] [EXPLORE] {explorer.detail}")
                else:
                    enc_l, enc_r, _, ot = motor.get_odom()
                    if switch_search_start_odom is not None:
                        sl, sr = switch_search_start_odom
                        avg = 0.5 * (abs(enc_l - sl) + abs(enc_r - sr))
                        dbg(f"[{elapsed:.2f}s] [SPIN] {avg:.0f}/{TURN_360_COUNTS:.0f} odom_age={time.time() - ot:.2f}")
                    else:
                        dbg(f"[{elapsed:.2f}s] [SPIN] start_odom None (엔코더 대기중)")
                    mode, target_v, target_w = search_next_cmd()

            elif target is None and color_lost_during_avoid:
                mode, target_v, target_w = search_last_cmd(last_color_deg)

            elif target is None:
                if last_color_time > 0.0:
                    mode, target_v, target_w = search_last_cmd(last_color_deg)
                    color_lost_during_avoid, switch_search_active = True, False
                else:
                    switch_search_active, explore_active, switch_search_start_odom = False, False, None
                    mode, target_v, target_w = avoid_mode("AVOID: no initial color", "AVOID_INITIAL", ranges, None, elapsed)

            if mode.startswith("SEARCH: last color direction"):
                if has_obstacle:
                    last_search_start_odom = None
                else:
                    if last_search_start_odom is None:
                        last_search_start_odom = get_turn_start_odom(motor)
                    if completed_one_encoder_turn(motor, last_search_start_odom):
                        last_search_start_odom = None
                        color_lost_during_avoid, switch_search_active = False, False
                        explore_active = True
                        enc_l, enc_r, _, _ = motor.get_odom()
                        spin_sign = -1.0 if last_color_deg >= 0.0 else 1.0
                        explorer = SpiralExplorer(enc_l, enc_r, turn_sign=spin_sign)
                        print(f"[{elapsed:.2f}s] [EXPLORE] last-color spin 360 done, spiral explore (turn={spin_sign:+.0f}, max r={SPIRAL_MAX_RADIUS}m)")
                        mode, target_v, target_w = explorer.command(enc_l, enc_r, ranges)
                        dbg(f"[{elapsed:.2f}s] [EXPLORE] {explorer.detail}")
            else:
                last_search_start_odom = None

            if target is not None or mode.startswith("AVOID") or mode.startswith("SEARCH") or mode.startswith("EXPLORE") or mode.startswith("RETURN") or mode.startswith("ESCAPE") or mode.startswith("FORWARD"):
                last_v = 0.0 if mode.startswith("ALIGN") else rate_limit(last_v, target_v, V_STEP)
                w_step = AVOID_W_STEP if (mode.startswith("AVOID") or mode.startswith("EXPLORE") or mode.startswith("ESCAPE")) else FOLLOW_W_STEP
                last_w = rate_limit(last_w, target_w, w_step)
                if mode.startswith("ALIGN"):
                    motor.pivot(last_w)
                else:
                    motor.vw(last_v, last_w)

            dbg(
                f"[{elapsed:.2f}s] [STATE] mode={mode} "
                f"target={current_target}:{'O' if target is not None else 'X'} "
                f"cmd(v={target_v:+.2f} w={target_w:+.2f}) sent(v={last_v:+.2f} w={last_w:+.2f}) "
                f"obstacle={has_obstacle} center_ratio={last_color_center_ratio:.2f} "
                f"x_err={last_color_x_err:+.2f} last_deg={last_color_deg:+.0f} "
                f"motor=ON"
            )

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

        sys.stdout = original_stdout

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="색상영역 추적 로봇")
    parser.add_argument(
        "--log",
        action="store_true",
        help="로그·화면 출력 켜기 (지정하지 않으면 기본값: 꺼짐)",
    )
    args = parser.parse_args()
    log_control = args.log
    SHOW_WINDOW = log_control
    DEBUG = log_control
    main()
