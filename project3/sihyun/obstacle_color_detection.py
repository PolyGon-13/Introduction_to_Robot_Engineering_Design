#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time
import serial
import math
import threading


# =========================================================
# 카메라 설정
# =========================================================

USE_PICAMERA2 = True

try:
    from picamera2 import Picamera2
except ImportError:
    USE_PICAMERA2 = False


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_EXPOSURE_VALUE = -1.0

#반전
FLIP_HORIZONTAL = False #좌우반전
FLIP_VERTICAL = False   #상하반전

# 모니터/원격화면에서 영상 확인할 때 True
# SSH로 실행해서 cv2 창이 안 뜨면 False로 바꾸기
SHOW_WINDOW = True


def open_camera():
    """
    Raspberry Pi Camera Module이 있으면 Picamera2 사용
    없으면 USB 카메라 cv2.VideoCapture(0) 사용
    """
    global USE_PICAMERA2

    if USE_PICAMERA2:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
        )
        picam2.configure(config)
        try:
            picam2.set_controls({"ExposureValue": CAMERA_EXPOSURE_VALUE})
        except Exception:
            pass
        picam2.start()
        time.sleep(1.0)
        print("[INFO] Picamera2 camera started")
        return picam2

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("[ERROR] Camera open failed")
        return None

    print("[INFO] USB camera started")
    return cap


def get_frame(camera):
    """
    카메라에서 프레임 읽기
    """
    if USE_PICAMERA2:
        frame_rgb = camera.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        frame_bgr = apply_camera_flip(frame_bgr)
        return True, frame_bgr

    ret, frame = camera.read()
    if ret:
        frame = apply_camera_flip(frame)
    return ret, frame


def apply_camera_flip(frame):
    if FLIP_HORIZONTAL and FLIP_VERTICAL:
        return cv2.flip(frame, -1)
    if FLIP_HORIZONTAL:
        return cv2.flip(frame, 1)
    if FLIP_VERTICAL:
        return cv2.flip(frame, 0)
    return frame


def close_camera(camera):
    if camera is None:
        return

    if USE_PICAMERA2:
        camera.stop()
    else:
        camera.release()

    if SHOW_WINDOW:
        cv2.destroyAllWindows()


# =========================================================
# Arduino 모터 제어 설정
# 붙여넣은 텍스트 (2)의 방식:
#   V{v:.3f},{w:.3f}\n
#   S\n
# =========================================================

ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600

# 전진 속도/회전 속도 제한
MAX_V = 0.18          # 최대 전진 속도
MIN_V = 0.08          # 따라갈 때 최소 전진 속도
MAX_W = 0.70          # 최대 회전 속도
KP_TURN = 0.85        # 화면 중심 오차 -> 회전속도 변환 게인

# 물체가 오른쪽에 있는데 로봇이 왼쪽으로 돌면 True로 바꾸기
INVERT_W = False

# 값이 너무 튀지 않게 루프마다 변화량 제한
V_RATE_LIMIT = 0.04
W_RATE_LIMIT = 0.15

# 카메라 중심 기준 오차가 이 이하이면 회전하지 않음
CENTER_DEADBAND = 0.08

# 면적이 이 값보다 작으면 잡음으로 무시
MIN_AREA = 1200
MIN_EXTENT = 0.45

# 면적 기반 거리 제어
# 물체 면적이 TARGET_AREA보다 작으면 전진
# STOP_AREA보다 커지면 너무 가까운 것으로 보고 정지
TARGET_AREA = 15000
STOP_AREA = 28000

# 추종할 색
# None이면 RED, BLUE, YELLOW 중 가장 크게 보이는 색을 따라감
# "RED", "BLUE", "YELLOW" 중 하나로 바꾸면 해당 색만 따라감
TARGET_COLOR = "RED"

# 색을 못 찾았을 때 동작
# False: 정지
# True : 제자리에서 천천히 탐색 회전
SEARCH_WHEN_LOST = False
SEARCH_W = 0.25
TARGET_LOST_HOLD_SEC = 0.35


# =========================================================
# LiDAR obstacle avoidance settings
# =========================================================

USE_LIDAR_AVOIDANCE = True
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

LIDAR_ANGLE_OFFSET_DEG = 1.54
LIDAR_DIST_OFFSET_MM = 1.0
LIDAR_ANGLE_SIGN = -1.0

LIDAR_MIN_DIST_M = 0.05
LIDAR_MAX_DIST_M = 0.75
LIDAR_MIN_QUALITY = 1
LIDAR_SCAN_HOLD_S = 0.30

AVOID_MIN_ANGLE_DEG = -90.0
AVOID_MAX_ANGLE_DEG = 90.0
AVOID_ANGLE_STEP_DEG = 2.0

AVOID_FRONT_DIST = 0.34
AVOID_DANGER_DIST = 0.22
AVOID_COLLISION_DIST = 0.20
AVOID_IGNORE_NEAR_DIST = 0.05

AVOID_FRONT_Y_HALF = 0.20
SIDE_CHECK_X_MIN = -0.05
SIDE_CHECK_X_MAX = 0.25
SIDE_CHECK_Y_MIN = 0.05
SIDE_CHECK_Y_MAX = 0.35
SIDE_AVOID_WARN_DIST = 0.15
SIDE_AVOID_BLOCK_DIST = 0.08
SIDE_TIGHT_V = 0.12
SIDE_PUSH_MIN_W = 0.12
SIDE_PUSH_MAX_W = 0.45

AVOID_BUBBLE_RADIUS = 0.05
AVOID_FREE_DIST = 0.18
AVOID_MIN_GAP_WIDTH_DEG = 8.0
AVOID_TURN_GAIN = 1.0
AVOID_BLEND_GAIN = 0.85
AVOID_MAX_W = 0.80


def open_arduino():
    ardu = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    time.sleep(2.0)
    print(f"[INFO] Arduino connected: {ARDU_PORT}, {ARDU_BAUD}")
    return ardu


def send_vw(ardu, v, w):
    """
    Arduino로 선속도 v, 각속도 w 전송
    """
    msg = f"V{v:.3f},{w:.3f}\n"
    ardu.write(msg.encode())


def stop(ardu):
    """
    Arduino로 정지 명령 전송
    """
    ardu.write(b"S\n")


def rate_limit(prev_value, target_value, limit):
    delta = float(np.clip(target_value - prev_value, -limit, limit))
    return prev_value + delta


# =========================================================
# 색상 HSV 범위 설정
# OpenCV HSV 범위
# H: 0~179
# S: 0~255
# V: 0~255
# =========================================================

COLOR_RANGES = {
    "RED": [
        # 빨강은 HSV에서 0 근처와 179 근처 두 구간으로 나뉨
        (np.array([0, 90, 120]), np.array([10, 255, 255])),
        (np.array([170, 90, 120]), np.array([179, 255, 255])),
    ],
    "BLUE": [
        (np.array([102, 85, 110]), np.array([126, 255, 255])),
    ],
    "YELLOW": [
        (np.array([22, 85, 140]), np.array([36, 255, 255])),
    ],
}

BOX_COLOR = {
    "RED": (0, 0, 255),
    "BLUE": (255, 0, 0),
    "YELLOW": (0, 255, 255),
}

BLUR_SIZE = 5
OPEN_KERNEL_SIZE = 5
CLOSE_KERNEL_SIZE = 5


def detect_colors(frame):
    """
    프레임에서 빨강, 파랑, 노랑 색상 인식
    """
    if BLUR_SIZE > 1:
        frame = cv2.GaussianBlur(frame, (BLUR_SIZE, BLUR_SIZE), 0)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detected_results = []
    open_kernel = np.ones((OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE), np.uint8)
    close_kernel = np.ones((CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE), np.uint8)

    for color_name, ranges in COLOR_RANGES.items():
        mask_total = None

        for lower, upper in ranges:
            mask = cv2.inRange(hsv, lower, upper)

            if mask_total is None:
                mask_total = mask
            else:
                mask_total = cv2.bitwise_or(mask_total, mask)

        # 노이즈 제거
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, open_kernel)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, close_kernel)

        contours, _ = cv2.findContours(
            mask_total,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < MIN_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            extent = area / max(1.0, float(w * h))

            if extent < MIN_EXTENT:
                continue

            cx = x + w // 2
            cy = y + h // 2

            detected_results.append(
                {
                    "color": color_name,
                    "area": float(area),
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h,
                    "cx": cx,
                    "cy": cy,
                }
            )

    return detected_results


def choose_target(results):
    """
    여러 색이 동시에 보이면 가장 면적이 큰 물체를 추종 대상으로 선택
    """
    if TARGET_COLOR is None:
        candidates = results
    else:
        candidates = [r for r in results if r["color"] == TARGET_COLOR]

    if not candidates:
        return None

    return max(candidates, key=lambda r: r["area"])


def compute_follow_cmd(target, frame_width):
    """
    인식한 색상 물체의 위치/면적으로 v, w 계산

    cx가 화면 오른쪽이면 error_x > 0
    파일 2의 FGM 코드 기준으로 w > 0은 좌회전 방향으로 쓰였으므로,
    오른쪽 물체를 따라가려면 w는 음수로 보냄.
    """
    image_center_x = frame_width / 2.0

    # -1.0 ~ +1.0
    error_x = (target["cx"] - image_center_x) / image_center_x

    if abs(error_x) < CENTER_DEADBAND:
        error_x = 0.0

    # 오른쪽에 있으면 우회전해야 하므로 기본은 - 부호
    if INVERT_W:
        w = KP_TURN * error_x
    else:
        w = -KP_TURN * error_x

    w = float(np.clip(w, -MAX_W, MAX_W))

    area = target["area"]

    # 너무 가까우면 정지
    if area >= STOP_AREA:
        v = 0.0

    # 적당히 가까우면 천천히 접근 또는 정지
    elif area >= TARGET_AREA:
        v = 0.0

    # 멀리 있으면 전진
    else:
        ratio = (TARGET_AREA - area) / max(1.0, TARGET_AREA - MIN_AREA)
        ratio = float(np.clip(ratio, 0.0, 1.0))
        v = MIN_V + (MAX_V - MIN_V) * ratio

        # 많이 틀어야 하는 상황에서는 전진 속도 줄이기
        turn_slow = 1.0 - 0.45 * min(1.0, abs(error_x))
        v *= turn_slow

    v = float(np.clip(v, 0.0, MAX_V))

    return v, w, error_x


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.ser.write(bytes([0xA5, 0x40]))
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        self.ser.write(bytes([0xA5, 0x20]))
        header = self.ser.read(7)
        if len(header) != 7 or header[0] != 0xA5 or header[1] != 0x5A:
            self.ser.close()
            raise RuntimeError("[LIDAR] Response Header Error")

        self.lock = threading.Lock()
        self.latest_scan = None
        self.scan_seq = 0
        self.scan_time = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buf_a, buf_d, buf_q = [], [], []

        while self.running:
            try:
                data = self.ser.read(5)
                if len(data) != 5:
                    continue

                s_flag = data[0] & 0x01
                s_inv = (data[0] & 0x02) >> 1
                if s_inv != (1 - s_flag):
                    continue

                if (data[1] & 0x01) != 1:
                    continue

                quality = data[0] >> 2
                angle = ((data[1] >> 1) | (data[2] << 7)) / 64.0
                dist = (data[3] | (data[4] << 8)) / 4.0

                if s_flag == 1 and len(buf_a) > 50:
                    with self.lock:
                        self.latest_scan = (
                            np.array(buf_a, dtype=np.float32),
                            np.array(buf_d, dtype=np.float32),
                            np.array(buf_q, dtype=np.float32),
                        )
                        self.scan_seq += 1
                        self.scan_time = time.time()
                    buf_a, buf_d, buf_q = [], [], []

                if dist > 0 and quality > 0:
                    buf_a.append(angle)
                    buf_d.append(dist)
                    buf_q.append(quality)

            except (serial.SerialException, OSError) as e:
                print(f"[LIDAR] Serial Error: {e}, retrying in 1 second...")
                time.sleep(1.0)
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass

    def get_scan(self):
        with self.lock:
            return self.latest_scan, self.scan_seq, self.scan_time

    def close(self):
        self.running = False
        try:
            self.ser.write(bytes([0xA5, 0x25]))
        except Exception:
            pass
        time.sleep(0.1)
        try:
            self.ser.close()
        except Exception:
            pass


def open_lidar():
    if not USE_LIDAR_AVOIDANCE:
        return None

    try:
        lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
        print(f"[INFO] LiDAR connected: {LIDAR_PORT}, {LIDAR_BAUD}")
        return lidar
    except Exception as e:
        print(f"[WARN] LiDAR disabled: {e}")
        return None


def scan_to_angle_ranges(scan):
    angles_grid = np.arange(
        AVOID_MIN_ANGLE_DEG,
        AVOID_MAX_ANGLE_DEG + 0.5 * AVOID_ANGLE_STEP_DEG,
        AVOID_ANGLE_STEP_DEG,
        dtype=np.float32,
    )
    ranges = np.full(len(angles_grid), LIDAR_MAX_DIST_M, dtype=np.float32)
    counts = np.zeros(len(angles_grid), dtype=np.int16)

    if scan is None:
        return angles_grid, ranges, counts

    angles, dists, qualities = scan
    dist_m = (dists.astype(np.float32) + LIDAR_DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + LIDAR_ANGLE_OFFSET_DEG)
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    valid = (
        (dist_m >= max(LIDAR_MIN_DIST_M, AVOID_IGNORE_NEAR_DIST))
        & (dist_m <= LIDAR_MAX_DIST_M)
        & (qualities >= LIDAR_MIN_QUALITY)
    )
    if not valid.any():
        return angles_grid, ranges, counts

    dist_m = dist_m[valid]
    angle_deg = angle_deg[valid]
    bins = np.rint((angle_deg - AVOID_MIN_ANGLE_DEG) / AVOID_ANGLE_STEP_DEG).astype(np.int32)
    in_sector = (bins >= 0) & (bins < len(angles_grid))

    for bin_idx, dist in zip(bins[in_sector], dist_m[in_sector]):
        if dist < ranges[bin_idx]:
            ranges[bin_idx] = dist
        counts[bin_idx] += 1

    return angles_grid, ranges, counts


def lidar_points_to_xy(scan):
    if scan is None:
        return np.empty((0, 2), dtype=np.float32)

    angles, dists, qualities = scan
    dist_m = (dists.astype(np.float32) + LIDAR_DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + LIDAR_ANGLE_OFFSET_DEG)
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    valid = (
        (dist_m >= max(LIDAR_MIN_DIST_M, AVOID_IGNORE_NEAR_DIST))
        & (dist_m <= LIDAR_MAX_DIST_M)
        & (qualities >= LIDAR_MIN_QUALITY)
    )
    if not valid.any():
        return np.empty((0, 2), dtype=np.float32)

    angle_rad = np.deg2rad(angle_deg[valid])
    dist_m = dist_m[valid]
    x = dist_m * np.cos(angle_rad)
    y = dist_m * np.sin(angle_rad)
    return np.column_stack((x, y)).astype(np.float32)


def front_obstacle_distance(points):
    if len(points) == 0:
        return LIDAR_MAX_DIST_M

    mask = (
        (points[:, 0] > AVOID_IGNORE_NEAR_DIST)
        & (points[:, 0] < AVOID_FRONT_DIST)
        & (np.abs(points[:, 1]) < AVOID_FRONT_Y_HALF)
    )
    if not mask.any():
        return LIDAR_MAX_DIST_M
    return float(np.min(points[mask, 0]))


def side_obstacle_distances(points):
    left_dist = LIDAR_MAX_DIST_M
    right_dist = LIDAR_MAX_DIST_M

    if len(points) == 0:
        return left_dist, right_dist

    side_mask = (
        (points[:, 0] > SIDE_CHECK_X_MIN)
        & (points[:, 0] < SIDE_CHECK_X_MAX)
        & (np.hypot(points[:, 0], points[:, 1]) > AVOID_IGNORE_NEAR_DIST)
        & (np.abs(points[:, 1]) > SIDE_CHECK_Y_MIN)
        & (np.abs(points[:, 1]) < SIDE_CHECK_Y_MAX)
    )
    if not side_mask.any():
        return left_dist, right_dist

    side_points = points[side_mask]
    left_points = side_points[side_points[:, 1] > 0.0]
    right_points = side_points[side_points[:, 1] < 0.0]

    if len(left_points) > 0:
        left_dist = float(np.min(left_points[:, 1]))
    if len(right_points) > 0:
        right_dist = float(np.min(np.abs(right_points[:, 1])))

    return left_dist, right_dist


def apply_safety_bubble(ranges, counts):
    working = ranges.copy()
    valid_obstacles = (counts > 0) & (ranges < LIDAR_MAX_DIST_M)
    if not valid_obstacles.any():
        return working

    obstacle_ranges = np.where(valid_obstacles, ranges, np.inf)
    closest_idx = int(np.argmin(obstacle_ranges))
    closest_dist = float(ranges[closest_idx])
    half_angle = math.atan2(AVOID_BUBBLE_RADIUS, max(closest_dist, LIDAR_MIN_DIST_M))
    half_bins = int(math.ceil(math.degrees(half_angle) / AVOID_ANGLE_STEP_DEG))
    start = max(0, closest_idx - half_bins)
    end = min(len(working), closest_idx + half_bins + 1)
    working[start:end] = 0.0
    return working


def find_free_gaps(free_mask):
    gaps = []
    start = None

    for idx, free in enumerate(free_mask):
        if free and start is None:
            start = idx
        elif not free and start is not None:
            gaps.append((start, idx))
            start = None

    if start is not None:
        gaps.append((start, len(free_mask)))

    min_bins = max(1, int(math.ceil(AVOID_MIN_GAP_WIDTH_DEG / AVOID_ANGLE_STEP_DEG)))
    return [(start, end) for start, end in gaps if end - start >= min_bins]


def choose_gap_angle(angles_deg, ranges, gaps, desired_angle_rad):
    if not gaps:
        return 0.0, False

    desired_deg = math.degrees(desired_angle_rad)
    best_angle = 0.0
    best_score = -float("inf")

    for start, end in gaps:
        idxs = np.arange(start, end)
        gap_angles = angles_deg[idxs]
        clearance = np.clip(ranges[idxs] / LIDAR_MAX_DIST_M, 0.0, 1.0)
        desired_score = 1.0 - np.clip(np.abs(gap_angles - desired_deg) / 90.0, 0.0, 1.0)
        straight_score = 1.0 - np.clip(np.abs(gap_angles) / 90.0, 0.0, 1.0)
        scores = 1.8 * desired_score + 0.8 * clearance + 0.3 * straight_score

        local_best = int(np.argmax(scores))
        score = float(scores[local_best])
        if score > best_score:
            best_score = score
            best_angle = math.radians(float(gap_angles[local_best]))

    return best_angle, True


def obstacle_danger(front_dist, left_dist, right_dist):
    front_danger = float(
        np.clip(
            (AVOID_FRONT_DIST - front_dist)
            / max(1e-6, AVOID_FRONT_DIST - AVOID_DANGER_DIST),
            0.0,
            1.0,
        )
    )
    side_min = min(left_dist, right_dist)
    side_danger = float(
        np.clip(
            (SIDE_AVOID_WARN_DIST - side_min)
            / max(1e-6, SIDE_AVOID_WARN_DIST - SIDE_AVOID_BLOCK_DIST),
            0.0,
            1.0,
        )
    )
    return max(front_danger, side_danger), front_danger, side_danger


def side_push_w(side_dist):
    ratio = float(
        np.clip(
            (SIDE_AVOID_BLOCK_DIST - side_dist)
            / max(1e-6, SIDE_AVOID_BLOCK_DIST - LIDAR_MIN_DIST_M),
            0.0,
            1.0,
        )
    )
    return SIDE_PUSH_MIN_W + (SIDE_PUSH_MAX_W - SIDE_PUSH_MIN_W) * ratio


def constrain_turn_away_from_side(blended_w, left_dist, right_dist):
    if right_dist < SIDE_AVOID_WARN_DIST and blended_w < 0.0:
        blended_w = 0.0
    if left_dist < SIDE_AVOID_WARN_DIST and blended_w > 0.0:
        blended_w = 0.0

    if right_dist <= SIDE_AVOID_BLOCK_DIST:
        blended_w = max(blended_w, side_push_w(right_dist))
    if left_dist <= SIDE_AVOID_BLOCK_DIST:
        blended_w = min(blended_w, -side_push_w(left_dist))

    if left_dist <= SIDE_AVOID_BLOCK_DIST and right_dist <= SIDE_AVOID_BLOCK_DIST:
        blended_w = 0.0

    return float(np.clip(blended_w, -MAX_W, MAX_W))


def choose_stop_turn_w(left_dist, right_dist, color_w):
    """
    전방이 너무 가까워 멈춰야 할 때도
    W가 0이 되지 않도록 강제로 회전 방향을 만든다.
    """
    MIN_STOP_TURN_W = 0.35

    # 왼쪽이 더 가까우면 오른쪽으로 회전
    if left_dist + 0.03 < right_dist:
        return -AVOID_MAX_W

    # 오른쪽이 더 가까우면 왼쪽으로 회전
    if right_dist + 0.03 < left_dist:
        return AVOID_MAX_W

    # 좌우 거리가 비슷하면 기존 타겟 방향을 따라 회전
    if abs(color_w) >= 0.05:
        return MIN_STOP_TURN_W if color_w > 0.0 else -MIN_STOP_TURN_W

    # 타겟 방향도 애매하면 기본으로 왼쪽 회전
    return MIN_STOP_TURN_W


def compute_lidar_avoidance_cmd(lidar, color_v, color_w):
    if lidar is None:
        return color_v, color_w, {
            "mode": "NO_LIDAR",
            "front": LIDAR_MAX_DIST_M,
            "left": LIDAR_MAX_DIST_M,
            "right": LIDAR_MAX_DIST_M,
        }

    scan, _, scan_time = lidar.get_scan()
    now = time.time()
    if scan is None or now - scan_time > LIDAR_SCAN_HOLD_S:
        return color_v, color_w, {
            "mode": "LIDAR_WAIT",
            "front": LIDAR_MAX_DIST_M,
            "left": LIDAR_MAX_DIST_M,
            "right": LIDAR_MAX_DIST_M,
        }

    points = lidar_points_to_xy(scan)
    front_dist = front_obstacle_distance(points)
    left_dist, right_dist = side_obstacle_distances(points)
    side_close = min(left_dist, right_dist) < SIDE_AVOID_WARN_DIST

    if color_v <= 0.001 and abs(color_w) <= 0.05:
        return 0.0, 0.0, {
            "mode": "IDLE",
            "front": front_dist,
            "left": left_dist,
            "right": right_dist,
        }

    if front_dist >= AVOID_FRONT_DIST and not side_close:
        return color_v, color_w, {
            "mode": "COLOR",
            "front": front_dist,
            "left": left_dist,
            "right": right_dist,
        }

    angles_deg, ranges, counts = scan_to_angle_ranges(scan)
    bubble_ranges = apply_safety_bubble(ranges, counts)
    gaps = find_free_gaps(bubble_ranges >= AVOID_FREE_DIST)
    desired_angle = float(np.clip(color_w / max(0.001, KP_TURN), -math.pi / 2.0, math.pi / 2.0))
    gap_angle, has_gap = choose_gap_angle(angles_deg, bubble_ranges, gaps, desired_angle)
    danger, front_danger, side_danger = obstacle_danger(front_dist, left_dist, right_dist)

    if not has_gap:
        avoid_v = 0.0
        avoid_w = choose_stop_turn_w(left_dist, right_dist, color_w)
        return avoid_v, avoid_w, {
            "mode": "AVOID_STOP_TURN",
            "front": front_dist,
            "left": left_dist,
            "right": right_dist,
            "gap_deg": math.degrees(gap_angle),
            "gaps": len(gaps),
            "front_danger": front_danger,
            "side_danger": side_danger,
        }

    if front_dist <= AVOID_COLLISION_DIST:
        if color_v <= 0.001:
            return 0.0, 0.0, {
                "mode": "AVOID_HOLD",
                "front": front_dist,
                "left": left_dist,
                "right": right_dist,
                "gap_deg": math.degrees(gap_angle),
                "gaps": len(gaps),
                "front_danger": front_danger,
                "side_danger": side_danger,
            }

        # 장애물이 가까워도 빈 공간이 있으면 완전 정지하지 말고
        # 천천히 전진하면서 회피 방향으로 회전한다.
        avoid_v = min(color_v, 0.05)

        avoid_w = float(np.clip(
            AVOID_TURN_GAIN * gap_angle,
            -AVOID_MAX_W,
            AVOID_MAX_W,
        ))

        # gap_angle이 너무 작아서 회전값이 거의 0이면 강제 회전
        if abs(avoid_w) < 0.35:
            avoid_w = choose_stop_turn_w(left_dist, right_dist, color_w)

        avoid_w = constrain_turn_away_from_side(avoid_w, left_dist, right_dist)

        return avoid_v, avoid_w, {
            "mode": "AVOID_CREEP",
            "front": front_dist,
            "left": left_dist,
            "right": right_dist,
            "gap_deg": math.degrees(gap_angle),
            "gaps": len(gaps),
            "front_danger": front_danger,
            "side_danger": side_danger,
        }

    avoid_w = float(np.clip(AVOID_TURN_GAIN * gap_angle, -AVOID_MAX_W, AVOID_MAX_W))
    blended_w = (1.0 - AVOID_BLEND_GAIN * danger) * color_w
    blended_w += (AVOID_BLEND_GAIN * danger) * avoid_w
    blended_w = constrain_turn_away_from_side(blended_w, left_dist, right_dist)

    safe_v_limit = MAX_V * (1.0 - 0.75 * danger)
    if side_close:
        safe_v_limit = min(safe_v_limit, SIDE_TIGHT_V)
    blended_v = min(color_v, safe_v_limit)

    avoid_mode = "SIDE_GUARD" if side_close and front_dist >= AVOID_FRONT_DIST else "AVOID"

    return blended_v, blended_w, {
        "mode": avoid_mode,
        "front": front_dist,
        "left": left_dist,
        "right": right_dist,
        "gap_deg": math.degrees(gap_angle),
        "gaps": len(gaps),
        "front_danger": front_danger,
        "side_danger": side_danger,
    }


def draw_results(frame, results, target=None, cmd_v=0.0, cmd_w=0.0):
    """
    인식 결과 화면에 표시
    """
    frame_h, frame_w = frame.shape[:2]
    cv2.line(
        frame,
        (frame_w // 2, 0),
        (frame_w // 2, frame_h),
        (255, 255, 255),
        1,
    )

    for result in results:
        color = result["color"]
        x = result["x"]
        y = result["y"]
        w = result["w"]
        h = result["h"]
        cx = result["cx"]
        cy = result["cy"]
        area = result["area"]

        box_color = BOX_COLOR[color]
        thickness = 3 if result is target else 2

        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, thickness)
        cv2.circle(frame, (cx, cy), 5, box_color, -1)

        label = f"{color} area:{int(area)} cx:{cx}"
        if result is target:
            label = "[TARGET] " + label

        cv2.putText(
            frame,
            label,
            (x, max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            box_color,
            2,
        )

    status = f"cmd: V={cmd_v:.2f}, W={cmd_w:.2f}"
    cv2.putText(
        frame,
        status,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
    )

    return frame


# =========================================================
# 메인
# =========================================================

def main():
    camera = None
    lidar = None
    ardu = None

    last_v = 0.0
    last_w = 0.0
    last_log = 0.0
    last_target = None
    last_target_time = 0.0

    try:
        camera = open_camera()
        if camera is None:
            return

        lidar = open_lidar()

        ardu = open_arduino()
        stop(ardu)

        print("[INFO] Initialization complete.")
        print("[INFO] Press Enter to start.")
        try:
            input()
        except EOFError:
            print("[WARN] Could not read input. Starting immediately.")

        print("[INFO] Go!!")

        while True:
            ret, frame = get_frame(camera)

            if not ret:
                print("[ERROR] Failed to read frame")
                send_vw(ardu, 0.0, 0.0)
                time.sleep(0.05)
                continue

            results = detect_colors(frame)
            target = choose_target(results)
            now = time.time()

            if target is not None:
                last_target = target.copy()
                last_target_time = now
            elif (
                last_target is not None
                and now - last_target_time <= TARGET_LOST_HOLD_SEC
            ):
                target = last_target.copy()
                target["held"] = True

            if target is None:
                if SEARCH_WHEN_LOST:
                    target_v = 0.0
                    target_w = SEARCH_W
                    mode = "SEARCH"
                    error_x = 0.0
                else:
                    target_v = 0.0
                    target_w = 0.0
                    mode = "LOST"
                    error_x = 0.0
            else:
                target_v, target_w, error_x = compute_follow_cmd(
                    target,
                    frame.shape[1],
                )
                if target.get("held"):
                    mode = f"HOLD_{target['color']}"
                else:
                    mode = f"FOLLOW_{target['color']}"

            # 급격한 속도 변화 방지
            target_v, target_w, avoid_info = compute_lidar_avoidance_cmd(
                lidar,
                target_v,
                target_w,
            )

            if avoid_info["mode"] in ("IDLE", "AVOID_HOLD"):
                cmd_v = 0.0
                cmd_w = 0.0
            elif avoid_info["mode"] == "AVOID_STOP_TURN":
                cmd_v = 0.0
                cmd_w = rate_limit(last_w, target_w, W_RATE_LIMIT)
            elif avoid_info["mode"] in ("AVOID", "SIDE_GUARD", "AVOID_CREEP"):
                cmd_v = min(rate_limit(last_v, target_v, V_RATE_LIMIT), target_v)
                cmd_w = rate_limit(last_w, target_w, W_RATE_LIMIT)
            else:
                cmd_v = rate_limit(last_v, target_v, V_RATE_LIMIT)
                cmd_w = rate_limit(last_w, target_w, W_RATE_LIMIT)

            send_vw(ardu, cmd_v, cmd_w)

            last_v = cmd_v
            last_w = cmd_w

            if now - last_log > 0.25:
                if target is None:
                    print(
                        f"[{mode}/{avoid_info['mode']}] "
                        f"v={cmd_v:.2f} w={cmd_w:.2f} "
                        f"front={avoid_info['front']:.2f} "
                        f"L={avoid_info['left']:.2f} R={avoid_info['right']:.2f}"
                    )
                else:
                    print(
                        f"[{mode}/{avoid_info['mode']}] "
                        f"cx={target['cx']} area={int(target['area'])} "
                        f"err={error_x:.2f} "
                        f"v={cmd_v:.2f} w={cmd_w:.2f} "
                        f"front={avoid_info['front']:.2f} "
                        f"L={avoid_info['left']:.2f} R={avoid_info['right']:.2f}"
                    )
                last_log = now

            if SHOW_WINDOW:
                frame = draw_results(frame, results, target, cmd_v, cmd_w)
                cv2.imshow("Color Following Robot", frame)

                # q 누르면 종료
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt")

    finally:
        if ardu is not None:
            try:
                stop(ardu)
                time.sleep(0.2)
                ardu.close()
            except Exception:
                pass

        if lidar is not None:
            lidar.close()

        close_camera(camera)
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
