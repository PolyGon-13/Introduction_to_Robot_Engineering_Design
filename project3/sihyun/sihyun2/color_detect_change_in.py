#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import cv2
import numpy as np
import serial


try:
    from picamera2 import Picamera2
    USE_PICAM = True  # Picamera2 사용 가능 여부
except ImportError:
    USE_PICAM = False  # Picamera2가 없으면 OpenCV USB 카메라 사용


# ==============================
# HSV color following settings
# ==============================

TARGET_SEQUENCE = ("RED", "YELLOW", "BLUE")  # Follow colors in this order.
SHOW_WINDOW = True  # 카메라 인식 화면을 띄울지 여부
MIN_AREA = 200  # 색상 물체로 인정할 최소 contour 면적
BOTTOM_LOST_RATIO = 0.88  # 색상이 화면 아래 88% 지점 아래에서 사라지면 정지로 판단
COLOR_SWITCH_PAUSE = 1.0  # 다음 색 추적 전 정지 시간(초)
COLOR_PRIORITY_BOTTOM_RATIO = 0.75  # 색상 박스 아래쪽이 화면 4등분 중 맨 아래 구역이면 색 추적 우선
COLOR_EXIT_CENTER_ERR = 0.20  # 이 가로 오차 안에서 아래로 사라질 때만 다음 색으로 전환

FOLLOW_MAX_V = 0.18  # 색 추적 모드 최대 전진 속도
FOLLOW_MAX_W = 0.70  # 색 추적 모드 최대 회전 속도
FOLLOW_KP = 0.85  # 색 중심 오차를 회전 속도로 바꾸는 비례 계수
DEADBAND = 0.08  # 화면 중심 근처 오차를 0으로 처리하는 범위

HSV_RANGES = {
    "RED": [([0, 100, 120], [5, 255, 255]), ([165, 80, 120], [179, 255, 255])],  # 빨간색 HSV 범위
    "BLUE": [([104, 90, 90], [116, 255, 230])],  # 파란색 HSV 범위
    "YELLOW": [([17, 100, 150], [27, 255, 255])],  # 노란색 HSV 범위
}
HSV_RANGES = {
    name: [(np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)) for lower, upper in ranges]
    for name, ranges in HSV_RANGES.items()
}  # cv2.inRange에서 바로 쓰도록 HSV 범위를 numpy 배열로 변환
BOX_COLORS = {"RED": (0, 0, 255), "BLUE": (255, 0, 0), "YELLOW": (0, 255, 255)}  # 화면 표시용 박스 색상(BGR)
MORPH_KERNEL = np.ones((5, 5), np.uint8)  # 색상 마스크 잡음 제거용 커널


# ==============================
# Motor settings
# ==============================

ARDU_PORT = "/dev/ttyS0"  # 아두이노 모터 제어 시리얼 포트
ARDU_BAUD = 9600  # 아두이노 시리얼 통신 속도
V_STEP = 0.04  # 전진 속도 명령의 루프당 최대 변화량
FOLLOW_W_STEP = 0.15  # 색 추적 모드 회전 속도 명령의 루프당 최대 변화량
AVOID_W_STEP = 0.20  # 장애물 회피 모드 회전 속도 명령의 루프당 최대 변화량
LOOP_DT = 0.05  # 메인 루프 대기 시간(초)
SEARCH_TIMEOUT = 3.0  # 회피 후 색을 다시 찾는 최대 시간(초)
SEARCH_MAX_W = 0.90  # 색 재탐색 모드 최대 회전 속도
SEARCH_TURN_GAIN = 1.0  # 마지막 색 방향을 회전 속도로 바꾸는 비례 계수
SWITCH_SEARCH_W = 0.90  # 다음 색이 안 보일 때 제자리 탐색 회전 속도
ODOM_LOG_INTERVAL = 0.5  # 엔코더 누적값 로그 출력 주기(초)
WHEEL_R = 0.034  # Arduino encoder distance calculation wheel radius(m)
ENC_PPR = 1012.0  # Arduino encoder counts per wheel revolution
ENC_COUNTS_PER_M = ENC_PPR / (2.0 * np.pi * WHEEL_R)
PRE_FORWARD_STOP_SEC = 0.5  # Stop before encoder-based extra forward move(s)
POST_COLOR_FORWARD_M = 0.10  # Move forward after a color exits bottom before pause(m)
POST_COLOR_FORWARD_TIMEOUT = 5.0  # Safety timeout for the extra forward move(s)
ODOM_WAIT_TIMEOUT = 1.0  # Max wait for Arduino odometry before extra move(s)


# ==============================
# RPLidar obstacle avoidance settings
# ==============================

LIDAR_PORT = "/dev/ttyUSB0"  # RPLidar 시리얼 포트
LIDAR_BAUD = 460800  # RPLidar 시리얼 통신 속도

RESET = b"\xA5\x40"  # 라이다 리셋 명령
SCAN = b"\xA5\x20"  # 라이다 스캔 시작 명령
LIDAR_STOP = b"\xA5\x25"  # 라이다 스캔 정지 명령

ANGLE_OFFSET = 1.54  # 라이다 장착 각도 보정값(도)
DIST_OFFSET = 0.0  # 라이다 거리 보정값(mm)
ANGLE_SIGN = -1.0  # 라이다 좌우 방향 보정 부호
MIN_Q = 1  # 사용할 라이다 측정 품질 최소값
MIN_D = 0.01  # 사용할 최소 거리(m)
MAX_D = 2.5  # 사용할 최대 거리(m)

ANG_MIN = -90.0  # 회피 계산에 사용할 최소 각도(오른쪽)
ANG_MAX = 90.0  # 회피 계산에 사용할 최대 각도(왼쪽)
ANG_STEP = 1.0  # 라이다 거리 배열의 각도 간격(도)
FREE_D = 0.35  # 이 거리 이상이면 빈 공간으로 판단(m)
MIN_GAP_DEG = 8.0  # 통과 가능한 gap으로 인정할 최소 각도 폭(도)
OBSTACLE_FRONT_DEG = 90.0  # 장애물 감지에 사용할 전방 각도 범위(좌우)

AVOID_BASE_V = 0.18  # 장애물 회피 모드 전진 속도
AVOID_MAX_W = 0.90  # 장애물 회피 모드 최대 회전 속도
AVOID_TURN_GAIN = 1.2  # 회피 목표 각도를 회전 속도로 바꾸는 비례 계수
SIDE_CLEAR_D = 0.20  # 좌우 장애물 거리 보정을 시작하는 기준 거리(m)
SIDE_CORRECT_MAX_DEG = 30.0  # 좌우 장애물 거리로 보정할 수 있는 최대 각도(도)
COLOR_TO_LIDAR_DEG = 45.0  # 카메라 화면 좌우 끝을 라이다 각도로 환산할 최대 각도(도)

GRID = np.arange(ANG_MIN, ANG_MAX + 0.5 * ANG_STEP, ANG_STEP, dtype=np.float32)  # 회피 계산용 각도 배열
LEFT_ZONE = (GRID >= 10.0) & (GRID <= 80.0)  # 왼쪽 장애물 거리 확인 구역
FRONT_LOG_ZONE = (GRID >= -10.0) & (GRID <= 10.0)  # 정면 장애물 확인 구역
RIGHT_ZONE = (GRID >= -80.0) & (GRID <= -10.0)  # 오른쪽 장애물 거리 확인 구역
OBSTACLE_ZONE = (GRID >= -OBSTACLE_FRONT_DEG) & (GRID <= OBSTACLE_FRONT_DEG)  # 장애물 있음/없음 판단 구역
MIN_GAP_BINS = max(1, int(np.ceil(MIN_GAP_DEG / ANG_STEP)))  # 최소 gap 각도를 배열 칸 수로 변환한 값


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


class RPLidarC1:
    def __init__(self):
        self.ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
        self.lock = threading.Lock()
        self.running = True
        self.scan = None
        self.scan_time = 0.0
        self.scan_seq = 0

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
                    scan = (
                        np.array(angles, dtype=np.float32),
                        np.array(dists, dtype=np.float32),
                        np.array(qualities, dtype=np.float32),
                    )

                    with self.lock:
                        self.scan = scan
                        self.scan_time = time.time()
                        self.scan_seq += 1

                    angles, dists, qualities = [], [], []

                if dist > 0 and quality >= MIN_Q:
                    angles.append(angle)
                    dists.append(dist)
                    qualities.append(quality)

            except (serial.SerialException, OSError):
                time.sleep(1.0)

    def get(self):
        with self.lock:
            return self.scan, self.scan_time, self.scan_seq

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
        return cam

    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cam


def read_frame(cam):
    if USE_PICAM:
        return True, cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)

    return cam.read()


def detect(frame):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    found = []

    for name, ranges in HSV_RANGES.items():
        mask = None

        for lower, upper in ranges:
            part = cv2.inRange(hsv, lower, upper)
            mask = part if mask is None else cv2.bitwise_or(mask, part)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area >= MIN_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                found.append((name, x, y, w, h, int(area)))

    return found


def pick(found, target_name):
    targets = found if target_name is None else [item for item in found if item[0] == target_name]
    return max(targets, key=lambda item: item[5], default=None)


def bottom_center_error(target, frame_shape):
    height, width = frame_shape[:2]
    _, x, y, w, h, _ = target
    cx = x + w / 2
    cy = y + h / 2
    dx = cx - width / 2
    dy = max(1.0, height - cy)
    angle = np.arctan2(dx, dy)
    return clamp(angle / (np.pi / 2), -1.0, 1.0)


def x_center_error(target, frame_shape):
    _, width = frame_shape[:2]
    _, x, _, w, _, _ = target
    return clamp((x + w / 2 - width / 2) / (width / 2), -1.0, 1.0)


def follow_cmd(target, frame_shape):
    if target is None:
        return 0.0, 0.0

    err = bottom_center_error(target, frame_shape)
    err = 0.0 if abs(err) < DEADBAND else err

    v = FOLLOW_MAX_V * (1.0 - 0.45 * min(1.0, abs(err)))
    w = clamp(-FOLLOW_KP * err, -FOLLOW_MAX_W, FOLLOW_MAX_W)
    return v, w


def color_angle_from_target(target, frame_shape):
    if target is None:
        return 0.0

    err = bottom_center_error(target, frame_shape)
    return clamp(-err * COLOR_TO_LIDAR_DEG, ANG_MIN, ANG_MAX)


def front_ranges(scan):
    ranges = np.full(len(GRID), MAX_D, dtype=np.float32)

    if scan is None:
        return ranges

    angles, dists, qualities = scan
    dist_m = (dists + DIST_OFFSET) / 1000.0
    angle_deg = norm_deg(angles + ANGLE_OFFSET) * ANGLE_SIGN

    valid = (dist_m >= MIN_D) & (dist_m <= MAX_D) & (qualities >= MIN_Q)
    bins = np.rint((angle_deg[valid] - ANG_MIN) / ANG_STEP).astype(np.int32)
    dists_valid = dist_m[valid]
    in_grid = (bins >= 0) & (bins < len(ranges))

    np.minimum.at(ranges, bins[in_grid], dists_valid[in_grid])
    return ranges


def find_gaps(free):
    gaps = []
    start = None

    for idx, ok in enumerate(free):
        if ok and start is None:
            start = idx
        elif not ok and start is not None:
            gaps.append((start, idx))
            start = None

    if start is not None:
        gaps.append((start, len(free)))

    return [(start, end) for start, end in gaps if end - start >= MIN_GAP_BINS]


def obstacle_detected(ranges):
    if ranges is None:
        return False

    return float(np.min(ranges[OBSTACLE_ZONE])) < FREE_D


def front_is_clear(ranges):
    if ranges is None:
        return False

    return float(np.min(ranges[FRONT_LOG_ZONE])) >= FREE_D


def lidar_zone_distances(ranges):
    if ranges is None:
        return None

    def zone_distance(zone):
        values = ranges[zone]
        measured = values[values < MAX_D]
        if len(measured) == 0:
            return MAX_D, 0
        return float(np.min(measured)), len(measured)

    return (
        zone_distance(LEFT_ZONE),
        zone_distance(FRONT_LOG_ZONE),
        zone_distance(RIGHT_ZONE),
    )


def zone_min_distance(ranges, zone):
    values = ranges[zone]
    measured = values[values < MAX_D]
    if len(measured) == 0:
        return MAX_D
    return float(np.min(measured))


def avoid_cmd(ranges, color_deg=None):
    safe_gaps = find_gaps(ranges >= FREE_D)  # 안전 거리 이상 비어 있는 gap 목록

    if not safe_gaps:
        return 0.0, 0.0, 0.0, 0

    def gap_width(gap):
        start, end = gap  # gap의 시작/끝 배열 인덱스
        return end - start

    def gap_center(gap):
        start, end = gap  # gap의 시작/끝 배열 인덱스
        return 0.5 * (GRID[start] + GRID[end - 1])

    front_blocked = float(np.min(ranges[FRONT_LOG_ZONE])) < FREE_D  # 정면이 안전 거리보다 가까이 막혔는지 여부

    if color_deg is not None and len(safe_gaps) >= 2 and not front_blocked:
        start, end = min(safe_gaps, key=lambda gap: abs(norm_deg(gap_center(gap) - color_deg)))
    else:
        start, end = max(safe_gaps, key=lambda gap: (gap_width(gap), -abs(gap_center(gap))))

    target_deg = float(0.5 * (GRID[start] + GRID[end - 1]))  # 선택한 gap 중심 각도
    left_d = zone_min_distance(ranges, LEFT_ZONE)  # 왼쪽 구역에서 가장 가까운 장애물 거리
    right_d = zone_min_distance(ranges, RIGHT_ZONE)  # 오른쪽 구역에서 가장 가까운 장애물 거리
    left_risk = max(0.0, SIDE_CLEAR_D - left_d)  # 왼쪽 장애물이 가까울수록 커지는 위험도
    right_risk = max(0.0, SIDE_CLEAR_D - right_d)  # 오른쪽 장애물이 가까울수록 커지는 위험도
    side_correct_deg = clamp(
        (right_risk - left_risk) / SIDE_CLEAR_D * SIDE_CORRECT_MAX_DEG,
        -SIDE_CORRECT_MAX_DEG,
        SIDE_CORRECT_MAX_DEG,
    )  # 가까운 측면 장애물에서 멀어지기 위한 각도 보정값
    target_deg = clamp(target_deg + side_correct_deg, ANG_MIN, ANG_MAX)

    w = clamp(AVOID_TURN_GAIN * np.deg2rad(target_deg), -AVOID_MAX_W, AVOID_MAX_W)  # 최종 회전 속도
    return AVOID_BASE_V, w, target_deg, len(safe_gaps)


def draw(frame, found, mode):
    for name, x, y, w, h, area in found:
        color = BOX_COLORS[name]
        cx, cy = x + w // 2, y + h // 2

        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.circle(frame, (cx, cy), 5, color, -1)
        cv2.putText(
            frame,
            f"{name} {cx},{cy} {area}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

    cv2.putText(frame, mode, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)


def update_color_memory(target, frame):
    last_color_deg = color_angle_from_target(target, frame.shape)
    _, _, y, _, h, _ = target
    return last_color_deg, time.time(), (y + h) / frame.shape[0], x_center_error(target, frame.shape)


def clear_color_memory():
    return 0.0, 0.0, 0.0, 0.0


def stop_motion(motor):
    motor.stop()
    return 0.0, 0.0, 0.0, 0.0


def search_next_cmd():
    return "SEARCH: next color", 0.0, SWITCH_SEARCH_W


def drive_forward_by_encoder(motor, distance_m=POST_COLOR_FORWARD_M):
    motor.stop()
    time.sleep(PRE_FORWARD_STOP_SEC)

    start_odom = None
    wait_start = time.time()

    while time.time() - wait_start <= ODOM_WAIT_TIMEOUT:
        odom = motor.get_odom()
        if odom[3] != 0.0:
            start_odom = odom
            break
        time.sleep(LOOP_DT)

    if start_odom is None:
        print("[ODOM] no encoder data, skip extra forward move")
        motor.stop()
        return False

    start_l, start_r, _, _ = start_odom
    target_counts = abs(distance_m) * ENC_COUNTS_PER_M
    forward_v = abs(FOLLOW_MAX_V) if distance_m >= 0.0 else -abs(FOLLOW_MAX_V)
    timeout = max(POST_COLOR_FORWARD_TIMEOUT, abs(distance_m / max(abs(FOLLOW_MAX_V), 0.01)) * 2.0)
    deadline = time.time() + timeout

    print(f"[ODOM] move {distance_m:.2f}m target_counts={target_counts:.0f}")

    left_counts = right_counts = 0.0
    while time.time() <= deadline:
        enc_l, enc_r, _, odom_time = motor.get_odom()
        if odom_time == 0.0 or time.time() - odom_time > 0.5:
            motor.stop()
            print("[ODOM] encoder data timeout during extra forward move")
            return False

        left_counts = abs(enc_l - start_l)
        right_counts = abs(enc_r - start_r)
        if left_counts >= target_counts and right_counts >= target_counts:
            break

        motor.vw(forward_v, 0.0)
        time.sleep(LOOP_DT)

    motor.stop()
    left_m = left_counts / ENC_COUNTS_PER_M
    right_m = right_counts / ENC_COUNTS_PER_M
    ok = left_counts >= target_counts and right_counts >= target_counts
    status = "done" if ok else "timeout"
    print(
        f"[ODOM] extra forward {status}: "
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


def color_cmd(target, frame, ranges, has_obstacle, color_deg, bottom_ratio, elapsed):
    priority = bottom_ratio >= COLOR_PRIORITY_BOTTOM_RATIO and front_is_clear(ranges)
    if has_obstacle and not priority:
        target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, color_deg)
        log_avoid(elapsed, "AVOID", target_deg, target_v, target_w, gap_count)
        return "AVOID: color + obstacle", target_v, target_w

    target_v, target_w = follow_cmd(target, frame.shape)
    return "FOLLOW: color only", target_v, target_w


def search_last_cmd(last_color_deg):
    w = clamp(SEARCH_TURN_GAIN * np.deg2rad(last_color_deg), -SEARCH_MAX_W, SEARCH_MAX_W)
    return "SEARCH: last color direction", 0.0, w


def main():
    cam = lidar = motor = None
    last_v = last_w = 0.0
    last_color_deg = last_color_time = last_color_bottom_ratio = last_color_x_err = 0.0
    last_odom_log_time = 0.0
    color_lost_during_avoid = False
    search_start_time = None
    switch_search_active = False
    target_index = 0
    current_target = TARGET_SEQUENCE[target_index]

    try:
        cam = open_camera()
        lidar = RPLidarC1()
        motor = Motor()
        motor.stop()
        motor.reset_encoders()
        start_time = time.time()  # 로그 출력용 시작 시각

        while True:
            elapsed = time.time() - start_time  # 프로그램 시작 후 경과 시간
            ok, frame = read_frame(cam)
            if not ok:
                break

            found = detect(frame)  # 현재 프레임에서 찾은 색상 물체 목록
            target = pick(found, current_target)  # 따라갈 대상 색상 물체
            found = [target] if target is not None else []
            scan, scan_time, scan_seq = lidar.get()  # 최신 라이다 스캔 데이터
            ranges = front_ranges(scan) if scan is not None else None  # 각도별 전방 거리 배열
            lidar_dist = lidar_zone_distances(ranges)  # 좌/정면/우측 로그용 최소 거리
            has_obstacle = obstacle_detected(ranges)  # 장애물 감지 여부

            if target is not None:
                last_color_deg, last_color_time, last_color_bottom_ratio, last_color_x_err = update_color_memory(target, frame)
                search_start_time, switch_search_active = None, False

            if time.time() - last_odom_log_time >= ODOM_LOG_INTERVAL:
                log_odom(elapsed, motor.get_odom())
                last_odom_log_time = time.time()

            log_lidar(elapsed, lidar_dist)

            color_exited_bottom = target is None and last_color_time > 0.0 and last_color_bottom_ratio >= BOTTOM_LOST_RATIO
            color_exited_bottom_center = color_exited_bottom and abs(last_color_x_err) <= COLOR_EXIT_CENTER_ERR
            if target is not None:
                color_lost_during_avoid = False
                mode, target_v, target_w = color_cmd(target, frame, ranges, has_obstacle, last_color_deg, last_color_bottom_ratio, elapsed)

            elif color_exited_bottom_center:
                if target_index + 1 < len(TARGET_SEQUENCE):
                    prev_target = current_target
                    target_index += 1
                    current_target = TARGET_SEQUENCE[target_index]
                    mode = f"SWITCH: {prev_target}->{current_target}"
                    print(f"[{elapsed:.2f}s] [COLOR] {prev_target} done, now tracking {current_target}")
                    drive_forward_by_encoder(motor)
                    target_v, target_w, last_v, last_w = 0.0, 0.0, 0.0, 0.0
                    time.sleep(COLOR_SWITCH_PAUSE)
                    ok, frame = read_frame(cam)
                    if not ok:
                        break

                    found = detect(frame)
                    target = pick(found, current_target)
                    found = [target] if target is not None else []

                    if target is not None:
                        last_color_deg, last_color_time, last_color_bottom_ratio, last_color_x_err = update_color_memory(target, frame)
                        color_lost_during_avoid = False
                        search_start_time = None
                        mode, target_v, target_w = color_cmd(target, frame, ranges, has_obstacle, last_color_deg, last_color_bottom_ratio, elapsed)
                    else:
                        mode, target_v, target_w = search_next_cmd()
                        last_color_deg, last_color_time, last_color_bottom_ratio, last_color_x_err = clear_color_memory()
                        color_lost_during_avoid, search_start_time = False, None
                        switch_search_active = True
                else:
                    mode = "STOP: color bottom"
                    drive_forward_by_encoder(motor)
                    target_v, target_w, last_v, last_w = 0.0, 0.0, 0.0, 0.0
                    last_color_deg, last_color_time, last_color_bottom_ratio, last_color_x_err = clear_color_memory()
                    color_lost_during_avoid, search_start_time, switch_search_active = False, None, False

            elif color_exited_bottom:
                color_lost_during_avoid = True
                switch_search_active = False
                if has_obstacle:
                    mode = "AVOID: lost color"
                    search_start_time = None
                    target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, None)
                    log_avoid(elapsed, "AVOID_LOST", target_deg, target_v, target_w, gap_count)
                else:
                    if search_start_time is None:
                        search_start_time = time.time()
                    mode = "SEARCH: last color"
                    target_v = 0.0
                    target_w = clamp(SEARCH_TURN_GAIN * np.deg2rad(last_color_deg), -SEARCH_MAX_W, SEARCH_MAX_W)

            elif target is None and has_obstacle and last_color_time > 0.0:
                mode = "AVOID: lost color"
                color_lost_during_avoid = True
                search_start_time = None
                target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, None)
                log_avoid(elapsed, "AVOID_LOST", target_deg, target_v, target_w, gap_count)

            elif target is None and switch_search_active:
                mode, target_v, target_w = search_next_cmd()

            elif target is None and color_lost_during_avoid:
                if search_start_time is None:
                    search_start_time = time.time()

                mode = "SEARCH: last color"
                if time.time() - search_start_time <= SEARCH_TIMEOUT:
                    target_v = 0.0
                    target_w = clamp(SEARCH_TURN_GAIN * np.deg2rad(last_color_deg), -SEARCH_MAX_W, SEARCH_MAX_W)
                else:
                    mode = "STOP: search timeout"
                    target_v, target_w, last_v, last_w = stop_motion(motor)
                    color_lost_during_avoid = False
                    search_start_time = None

            elif target is None:
                mode = "STOP: no color"
                target_v, target_w, last_v, last_w = stop_motion(motor)
                color_lost_during_avoid, search_start_time, switch_search_active = False, None, False

            if target is not None or mode.startswith("AVOID") or mode.startswith("SEARCH"):
                last_v = rate_limit(last_v, target_v, V_STEP)
                w_step = AVOID_W_STEP if mode.startswith("AVOID") else FOLLOW_W_STEP
                last_w = rate_limit(last_w, target_w, w_step)
                motor.vw(last_v, last_w)

            if SHOW_WINDOW:
                draw(frame, found, mode)
                cv2.imshow("project3", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
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
