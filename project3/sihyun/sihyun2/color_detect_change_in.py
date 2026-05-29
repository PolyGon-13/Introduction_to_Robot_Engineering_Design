#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import cv2
import numpy as np
import serial

try:
    from picamera2 import Picamera2
    USE_PICAM = True
except ImportError:
    USE_PICAM = False


TARGET_SEQUENCE = ("RED", "YELLOW", "BLUE")
SHOW_WINDOW = True
MIN_AREA = 200
BOTTOM_LOST_RATIO = 0.88
COLOR_SWITCH_PAUSE = 1.0
COLOR_PRIORITY_BOTTOM_RATIO = 0.75
COLOR_EXIT_CENTER_ERR = 0.30

FOLLOW_MAX_V = 0.18
FOLLOW_MAX_W = 0.70
FOLLOW_KP = 0.85
DEADBAND = 0.08

HSV_RANGES = {
    "RED": [([0, 100, 120], [5, 255, 255]), ([165, 80, 120], [179, 255, 255])],
    "BLUE": [([104, 90, 90], [116, 255, 230])],
    "YELLOW": [([19, 130, 170], [25, 255, 255])],
}
HSV_RANGES = {
    name: [(np.array(lo, np.uint8), np.array(hi, np.uint8)) for lo, hi in ranges]
    for name, ranges in HSV_RANGES.items()
}
BOX_COLORS = {"RED": (0, 0, 255), "BLUE": (255, 0, 0), "YELLOW": (0, 255, 255)}
MORPH_KERNEL = np.ones((5, 5), np.uint8)

ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600
V_STEP = 0.04
FOLLOW_W_STEP = 0.15
AVOID_W_STEP = 0.20
LOOP_DT = 0.05
SEARCH_MAX_W = 0.90
SEARCH_TURN_GAIN = 1.0
SWITCH_SEARCH_W = 0.90
ODOM_LOG_INTERVAL = 0.5
WHEEL_R = 0.034
ENC_PPR = 1012.0
ENC_COUNTS_PER_M = ENC_PPR / (2.0 * np.pi * WHEEL_R)
PRE_FORWARD_STOP_SEC = 0.0
POST_COLOR_FORWARD_M = 0.20
POST_COLOR_FORWARD_TIMEOUT = 5.0
ODOM_WAIT_TIMEOUT = 1.0

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
RESET = b"\xA5\x40"
SCAN = b"\xA5\x20"
LIDAR_STOP = b"\xA5\x25"

ANGLE_OFFSET = 1.54
DIST_OFFSET = 0.0
ANGLE_SIGN = -1.0
MIN_Q = 1
MIN_D = 0.01
MAX_D = 2.5
ANG_MIN = -90.0
ANG_MAX = 90.0
ANG_STEP = 1.0
FREE_D = 0.35
MIN_GAP_DEG = 8.0
OBSTACLE_FRONT_DEG = 90.0

AVOID_BASE_V = 0.18
AVOID_MAX_W = 0.90
AVOID_TURN_GAIN = 1.2
SIDE_CLEAR_D = 0.20
SIDE_CORRECT_MAX_DEG = 30.0
COLOR_TO_LIDAR_DEG = 45.0

GRID = np.arange(ANG_MIN, ANG_MAX + 0.5 * ANG_STEP, ANG_STEP, dtype=np.float32)
LEFT_ZONE = (GRID >= 10.0) & (GRID <= 80.0)
FRONT_LOG_ZONE = (GRID >= -10.0) & (GRID <= 10.0)
RIGHT_ZONE = (GRID >= -80.0) & (GRID <= -10.0)
OBSTACLE_ZONE = (GRID >= -OBSTACLE_FRONT_DEG) & (GRID <= OBSTACLE_FRONT_DEG)
MIN_GAP_BINS = max(1, int(np.ceil(MIN_GAP_DEG / ANG_STEP)))


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def rate_limit(prev, target, step):
    return prev + clamp(target - prev, -step, step)


def norm_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


class Motor:
    def __init__(self):
        self.ser = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.running = True
        self.odom_l = self.odom_r = self.odom_ms = 0
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
                    if len(parts) == 4:
                        with self.lock:
                            self.odom_l, self.odom_r, self.odom_ms = map(int, parts[1:])
                            self.odom_time = time.time()
                else:
                    print(f"[ARDUINO] {line}")
            except (ValueError, serial.SerialException, OSError):
                time.sleep(0.05)

    def _write(self, data):
        with self.write_lock:
            self.ser.write(data)

    def vw(self, v, w):
        self._write(f"V{v:.3f},{w:.3f}\n".encode("ascii"))

    def stop(self):
        self._write(b"S\n")

    def reset_encoders(self):
        self._write(b"R\n")

    def get_odom(self):
        with self.lock:
            return self.odom_l, self.odom_r, self.odom_ms, self.odom_time

    def close(self):
        self.running = False
        self.thread.join(timeout=0.5)
        if self.ser.is_open:
            self.ser.close()


class RPLidarC1:
    def __init__(self):
        self.ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
        self.lock = threading.Lock()
        self.running = True
        self.scan = None
        self.ser.write(RESET)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(SCAN)

        header = self.ser.read(7)
        if len(header) != 7 or header[:2] != b"\xA5\x5A":
            self.ser.close()
            raise RuntimeError("[LIDAR] response header error")

        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        angles, dists, qualities = [], [], []
        while self.running:
            try:
                packet = self.ser.read(5)
                if len(packet) != 5:
                    continue

                start = packet[0] & 1
                if ((packet[0] & 2) >> 1) != (1 - start) or (packet[1] & 1) != 1:
                    continue

                quality = packet[0] >> 2
                angle = ((packet[1] >> 1) | (packet[2] << 7)) / 64.0
                dist = (packet[3] | (packet[4] << 8)) / 4.0

                if start and len(angles) > 50:
                    with self.lock:
                        self.scan = tuple(np.array(v, dtype=np.float32) for v in (angles, dists, qualities))
                    angles, dists, qualities = [], [], []

                if dist > 0 and quality >= MIN_Q:
                    angles.append(angle)
                    dists.append(dist)
                    qualities.append(quality)
            except (serial.SerialException, OSError):
                time.sleep(1.0)

    def get(self):
        with self.lock:
            return self.scan

    def close(self):
        self.running = False
        try:
            self.ser.write(LIDAR_STOP)
        except Exception:
            pass
        self.thread.join(timeout=0.5)
        if self.ser.is_open:
            self.ser.close()


def open_camera():
    if USE_PICAM:
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
        cam.start()
    else:
        cam = cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cam


def read_frame(cam):
    return (True, cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)) if USE_PICAM else cam.read()


def detect(frame):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    found = []

    for name, ranges in HSV_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
            box = approx.reshape(-1, 2) if len(approx) >= 3 else cv2.boxPoints(cv2.minAreaRect(cnt)).astype(np.int32)
            moments = cv2.moments(cnt)
            if moments["m00"]:
                cx, cy = moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
            else:
                cx, cy = x + w / 2, y + h / 2
            found.append((name, x, y, w, h, int(area), float(cx), float(cy), box, cnt))

    return found


def pick(found, target_name):
    targets = found if target_name is None else [item for item in found if item[0] == target_name]
    return max(targets, key=lambda item: item[5], default=None)


def bottom_center_error(target, frame_shape):
    height, width = frame_shape[:2]
    return clamp(np.arctan2(target[6] - width / 2, max(1.0, height - target[7])) / (np.pi / 2), -1.0, 1.0)


def x_center_error(target, frame_shape):
    return clamp((target[6] - frame_shape[1] / 2) / (frame_shape[1] / 2), -1.0, 1.0)


def follow_cmd(target, frame_shape):
    if target is None:
        return 0.0, 0.0
    err = bottom_center_error(target, frame_shape)
    err = 0.0 if abs(err) < DEADBAND else err
    return FOLLOW_MAX_V * (1.0 - 0.45 * min(1.0, abs(err))), clamp(-FOLLOW_KP * err, -FOLLOW_MAX_W, FOLLOW_MAX_W)


def color_angle_from_target(target, frame_shape):
    return 0.0 if target is None else clamp(-bottom_center_error(target, frame_shape) * COLOR_TO_LIDAR_DEG, ANG_MIN, ANG_MAX)


def front_ranges(scan):
    ranges = np.full(len(GRID), MAX_D, dtype=np.float32)
    if scan is None:
        return ranges

    angles, dists, qualities = scan
    dist_m = (dists + DIST_OFFSET) / 1000.0
    angle_deg = norm_deg(angles + ANGLE_OFFSET) * ANGLE_SIGN
    valid = (dist_m >= MIN_D) & (dist_m <= MAX_D) & (qualities >= MIN_Q)
    bins = np.rint((angle_deg[valid] - ANG_MIN) / ANG_STEP).astype(np.int32)
    in_grid = (bins >= 0) & (bins < len(ranges))
    np.minimum.at(ranges, bins[in_grid], dist_m[valid][in_grid])
    return ranges


def find_gaps(free):
    gaps, start = [], None
    for idx, ok in enumerate(free):
        if ok and start is None:
            start = idx
        elif not ok and start is not None:
            gaps.append((start, idx))
            start = None
    if start is not None:
        gaps.append((start, len(free)))
    return [(start, end) for start, end in gaps if end - start >= MIN_GAP_BINS]


def zone_min_distance(ranges, zone):
    measured = ranges[zone][ranges[zone] < MAX_D]
    return MAX_D if len(measured) == 0 else float(np.min(measured))


def obstacle_detected(ranges):
    return ranges is not None and float(np.min(ranges[OBSTACLE_ZONE])) < FREE_D


def front_is_clear(ranges):
    return ranges is not None and float(np.min(ranges[FRONT_LOG_ZONE])) >= FREE_D


def lidar_zone_distances(ranges):
    if ranges is None:
        return None
    return tuple((zone_min_distance(ranges, zone), int(np.sum(ranges[zone] < MAX_D))) for zone in (LEFT_ZONE, FRONT_LOG_ZONE, RIGHT_ZONE))


def avoid_cmd(ranges, color_deg=None):
    safe_gaps = find_gaps(ranges >= FREE_D)
    if not safe_gaps:
        return 0.0, 0.0, 0.0, 0

    def center(gap):
        return 0.5 * (GRID[gap[0]] + GRID[gap[1] - 1])

    front_blocked = float(np.min(ranges[FRONT_LOG_ZONE])) < FREE_D
    if color_deg is not None and len(safe_gaps) >= 2 and not front_blocked:
        gap = min(safe_gaps, key=lambda g: abs(norm_deg(center(g) - color_deg)))
    else:
        gap = max(safe_gaps, key=lambda g: (g[1] - g[0], -abs(center(g))))

    target_deg = center(gap)
    left_risk = max(0.0, SIDE_CLEAR_D - zone_min_distance(ranges, LEFT_ZONE))
    right_risk = max(0.0, SIDE_CLEAR_D - zone_min_distance(ranges, RIGHT_ZONE))
    side_deg = clamp((right_risk - left_risk) / SIDE_CLEAR_D * SIDE_CORRECT_MAX_DEG, -SIDE_CORRECT_MAX_DEG, SIDE_CORRECT_MAX_DEG)
    target_deg = clamp(target_deg + side_deg, ANG_MIN, ANG_MAX)
    return AVOID_BASE_V, clamp(AVOID_TURN_GAIN * np.deg2rad(target_deg), -AVOID_MAX_W, AVOID_MAX_W), target_deg, len(safe_gaps)


def draw(frame, found, mode):
    for name, x, y, _, _, area, cx, cy, box, cnt in found:
        color = BOX_COLORS[name]
        center = int(round(cx)), int(round(cy))
        cv2.drawContours(frame, [cnt], -1, color, 2)
        cv2.polylines(frame, [box], True, color, 2)
        cv2.circle(frame, center, 5, color, -1)
        cv2.putText(frame, f"{name} {center[0]},{center[1]} {area}", (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(frame, mode, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)


def show_frame(frame, found, mode):
    if not SHOW_WINDOW:
        return False
    draw(frame, found, mode)
    cv2.imshow("project3", frame)
    return cv2.waitKey(1) & 0xFF == ord("q")


def update_color_memory(target, frame):
    return (
        color_angle_from_target(target, frame.shape),
        time.time(),
        float(np.max(target[8][:, 1])) / frame.shape[0],
        x_center_error(target, frame.shape),
    )


class EncoderForwardMove:
    def __init__(self, motor, distance_m=POST_COLOR_FORWARD_M):
        self.motor = motor
        self.distance_m = distance_m
        self.start_l = self.start_r = 0
        self.left_counts = self.right_counts = 0.0
        self.target_counts = abs(distance_m) * ENC_COUNTS_PER_M
        self.forward_v = abs(FOLLOW_MAX_V) if distance_m >= 0.0 else -abs(FOLLOW_MAX_V)
        self.timeout = max(POST_COLOR_FORWARD_TIMEOUT, abs(distance_m / max(abs(FOLLOW_MAX_V), 0.01)) * 2.0)
        self.wait_deadline = time.time() + ODOM_WAIT_TIMEOUT
        self.deadline = 0.0
        self.started = False

    def update(self):
        now = time.time()
        if not self.started:
            odom = self.motor.get_odom()
            if odom[3] == 0.0:
                self.motor.stop()
                if now > self.wait_deadline:
                    print("[ODOM] no encoder data, skip extra forward move")
                    return True, False
                return False, None
            self.start_l, self.start_r, _, _ = odom
            self.deadline = now + self.timeout
            self.started = True
            print(f"[ODOM] move {self.distance_m:.2f}m target_counts={self.target_counts:.0f}")

        enc_l, enc_r, _, odom_time = self.motor.get_odom()
        if odom_time == 0.0 or now - odom_time > 0.5:
            self.motor.stop()
            print("[ODOM] encoder data timeout during extra forward move")
            return True, False

        self.left_counts = abs(enc_l - self.start_l)
        self.right_counts = abs(enc_r - self.start_r)
        ok = self.left_counts >= self.target_counts and self.right_counts >= self.target_counts
        if ok or now > self.deadline:
            self.motor.stop()
            print(
                f"[ODOM] extra forward {'done' if ok else 'timeout'}: "
                f"left={self.left_counts / ENC_COUNTS_PER_M:.2f}m({self.left_counts:.0f}) "
                f"right={self.right_counts / ENC_COUNTS_PER_M:.2f}m({self.right_counts:.0f})"
            )
            return True, ok

        self.motor.vw(self.forward_v, 0.0)
        return False, None


def log_lidar(elapsed, lidar_dist):
    if lidar_dist is None:
        print(f"[{elapsed:.2f}s] [LIDAR] waiting...")
        return
    (ld, ln), (fd, fn), (rd, rn) = lidar_dist
    print(f"[{elapsed:.2f}s] [LIDAR] left={ld:.2f}m({ln}) front={fd:.2f}m({fn}) right={rd:.2f}m({rn})")


def log_avoid(elapsed, tag, target_deg, target_v, target_w, gap_count):
    print(f"[{elapsed:.2f}s] [{tag}] gap={gap_count} target={target_deg:.0f} v={target_v:.2f} w={target_w:.2f}")


def log_odom(elapsed, odom):
    enc_l, enc_r, arduino_ms, odom_time = odom
    msg = "waiting..." if odom_time == 0.0 else f"left={enc_l} right={enc_r} arduino_ms={arduino_ms}"
    print(f"[{elapsed:.2f}s] [ODOM] {msg}")


def color_cmd(target, frame, ranges, has_obstacle, color_deg, bottom_ratio, elapsed):
    if has_obstacle and not (bottom_ratio >= COLOR_PRIORITY_BOTTOM_RATIO and front_is_clear(ranges)):
        target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, color_deg)
        log_avoid(elapsed, "AVOID", target_deg, target_v, target_w, gap_count)
        return "AVOID: color + obstacle", target_v, target_w
    target_v, target_w = follow_cmd(target, frame.shape)
    return "FOLLOW: color only", target_v, target_w


def search_last_cmd(last_color_deg):
    return "SEARCH: last color direction", FOLLOW_MAX_V, clamp(SEARCH_TURN_GAIN * np.deg2rad(last_color_deg), -SEARCH_MAX_W, SEARCH_MAX_W)


def pause_or_forward(frame, found, motor, action, forward_move, finish_after_pause):
    if forward_move is None:
        forward_move = EncoderForwardMove(motor)
    done, _ = forward_move.update()
    if not done:
        return False, action, forward_move, finish_after_pause, 0.0
    finish_after_pause = finish_after_pause or action == "complete"
    return True, None, None, finish_after_pause, time.time() + COLOR_SWITCH_PAUSE


def main():
    cam = lidar = motor = None
    last_v = last_w = 0.0
    last_color_deg = last_color_time = last_color_bottom_ratio = last_color_x_err = 0.0
    last_odom_log_time = switch_pause_until = pending_forward_time = 0.0
    color_lost_during_avoid = switch_search_active = finish_after_pause = False
    search_start_time = pending_forward_action = forward_move = None
    target_index = 0
    current_target = TARGET_SEQUENCE[target_index]

    try:
        cam, lidar, motor = open_camera(), RPLidarC1(), Motor()
        motor.stop()
        motor.reset_encoders()
        input("Press Enter to start...")
        motor.stop()
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            ok, frame = read_frame(cam)
            if not ok:
                break

            found_all = detect(frame)
            target = pick(found_all, current_target)
            found = [target] if target is not None else []
            scan = lidar.get()
            ranges = front_ranges(scan) if scan is not None else None
            has_obstacle = obstacle_detected(ranges)
            now = time.time()

            if target is not None:
                last_color_deg, last_color_time, last_color_bottom_ratio, last_color_x_err = update_color_memory(target, frame)
                search_start_time, switch_search_active = None, False

            if now - last_odom_log_time >= ODOM_LOG_INTERVAL:
                log_odom(elapsed, motor.get_odom())
                last_odom_log_time = now
            log_lidar(elapsed, lidar_zone_distances(ranges))

            color_exited = target is None and last_color_time > 0.0 and last_color_bottom_ratio >= BOTTOM_LOST_RATIO
            color_exited_center = color_exited and abs(last_color_x_err) <= COLOR_EXIT_CENTER_ERR

            if pending_forward_action is not None:
                last_v = last_w = 0.0
                if now < pending_forward_time:
                    mode = "PAUSE: before forward"
                    motor.stop()
                else:
                    mode = "FORWARD: encoder"
                    done, pending_forward_action, forward_move, finish_after_pause, pause_until = pause_or_forward(
                        frame, found, motor, pending_forward_action, forward_move, finish_after_pause
                    )
                    if done:
                        switch_pause_until = pause_until
                if show_frame(frame, found, mode):
                    break
                time.sleep(LOOP_DT)
                continue

            if now < switch_pause_until:
                motor.stop()
                if show_frame(frame, found, "PAUSE: color switch"):
                    break
                time.sleep(LOOP_DT)
                continue

            if finish_after_pause:
                print(f"[{elapsed:.2f}s] [COLOR] {current_target} done, mission complete")
                break

            if target is not None:
                color_lost_during_avoid = False
                mode, target_v, target_w = color_cmd(target, frame, ranges, has_obstacle, last_color_deg, last_color_bottom_ratio, elapsed)

            elif color_exited_center:
                last_v = last_w = 0.0
                last_color_deg = last_color_time = last_color_bottom_ratio = last_color_x_err = 0.0
                color_lost_during_avoid, search_start_time = False, None
                pending_forward_time = time.time() + PRE_FORWARD_STOP_SEC
                motor.stop()

                if target_index + 1 < len(TARGET_SEQUENCE):
                    prev_target = current_target
                    target_index += 1
                    current_target = TARGET_SEQUENCE[target_index]
                    print(f"[{elapsed:.2f}s] [COLOR] {prev_target} done, now tracking {current_target}")
                    mode, target_v, target_w = f"SWITCH: {prev_target}->{current_target}", 0.0, 0.0
                    switch_search_active = True
                    pending_forward_action = "switch"
                else:
                    mode, target_v, target_w = "STOP: color bottom", 0.0, 0.0
                    switch_search_active = False
                    pending_forward_action = "complete"

            elif color_exited:
                color_lost_during_avoid, switch_search_active = True, False
                if has_obstacle:
                    search_start_time = None
                    target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, last_color_deg)
                    log_avoid(elapsed, "AVOID_LOST", target_deg, target_v, target_w, gap_count)
                    mode = "AVOID: lost color"
                else:
                    search_start_time = search_start_time or time.time()
                    mode, target_v, target_w = search_last_cmd(last_color_deg)

            elif target is None and has_obstacle and last_color_time > 0.0:
                color_lost_during_avoid, search_start_time = True, None
                target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, last_color_deg)
                log_avoid(elapsed, "AVOID_LOST", target_deg, target_v, target_w, gap_count)
                mode = "AVOID: lost color"

            elif target is None and switch_search_active:
                mode, target_v, target_w = "SEARCH: next color", 0.0, SWITCH_SEARCH_W

            elif target is None:
                if last_color_time > 0.0 or color_lost_during_avoid:
                    search_start_time = search_start_time or time.time()
                    mode, target_v, target_w = search_last_cmd(last_color_deg)
                    color_lost_during_avoid, switch_search_active = True, False
                else:
                    mode, target_v, target_w = "SEARCH: next color", 0.0, SWITCH_SEARCH_W
                    color_lost_during_avoid, switch_search_active = False, True

            if target is not None or mode.startswith(("AVOID", "SEARCH")):
                last_v = rate_limit(last_v, target_v, V_STEP)
                last_w = rate_limit(last_w, target_w, AVOID_W_STEP if mode.startswith("AVOID") else FOLLOW_W_STEP)
                motor.vw(last_v, last_w)

            if show_frame(frame, found, mode):
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
        if cam is not None:
            cam.stop() if USE_PICAM else cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
