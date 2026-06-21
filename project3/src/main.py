#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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



# ==============================
# HSV color following settings
# ==============================

TARGET_SEQUENCE = ("RED", "YELLOW", "BLUE")  # Follow colors in this order.
SHOW_WINDOW = True  # 카메라 인식 화면을 띄울지 여부
DEBUG = True  # 디버그 로그 출력 켜고 끄기 (False면 상세 로그 전부 끔)
BOTTOM_LOST_RATIO = 0.90  # 색상이 화면 아래 1/10 지점 아래에서 사라지면 정지로 판단
COLOR_SWITCH_PAUSE_MS = 1000  # 다음 색 추적 전 정지 시간(ms)
COLOR_FORWARD_CENTER_RATIO = 0.95  # 색상 중심이 화면 하단 1/20 구역에 들어오면 전진
COLOR_EXIT_CENTER_ERR = 0.15  # 이 가로 오차 안에서 아래로 사라질 때만 다음 색으로 전환

# ==============================
# Motor settings
# ==============================

ARDU_PORT = "/dev/ttyS0"  # 아두이노 모터 제어 시리얼 포트
ARDU_BAUD = 9600  # 아두이노 시리얼 통신 속도
DRIVE_V = 0.25  # 색 추적, 장애물 회피, 색 완료 후 추가 전진에 공통으로 쓰는 전진 속도
V_STEP = 0.04  # 전진 속도 명령의 루프당 최대 변화량
FOLLOW_W_STEP = 0.25  # 색 추적 모드 회전 속도 명령의 루프당 최대 변화량
AVOID_W_STEP = 0.20  # 장애물 회피 모드 회전 속도 명령의 루프당 최대 변화량
LOOP_DT = 0.05  # 메인 루프 대기 시간(초)
SEARCH_MAX_W = 1.0  # 색 재탐색 모드 최대 회전 속도
SWITCH_SEARCH_W = 1.5  # 다음 색이 안 보일 때 제자리 탐색 회전 속도
ODOM_LOG_INTERVAL = 0.5  # 엔코더 누적값 로그 출력 주기(초)
WHEEL_R = 0.034  # Arduino encoder distance calculation wheel radius(m)
ENC_PPR = 1012.0  # Arduino encoder counts per wheel revolution
ENC_COUNTS_PER_M = ENC_PPR / (2.0 * np.pi * WHEEL_R)
POST_COLOR_FORWARD_M = 0.03  # Move forward after a color exits bottom before pause(m)
TURN_360_WHEEL_BASE_M = 0.18  # Distance between left/right wheels for encoder-based 360 turn(m)
TURN_360_COUNTS = np.pi * TURN_360_WHEEL_BASE_M * ENC_COUNTS_PER_M
AVOID_STOP_D = 0.15  # Stop avoid forward speed when a front obstacle is this close(m)
SPIRAL_GROWTH = 0.06  # 아르키메데스 나선 계수 b: 각도 1rad당 반경 증가량(m, 작을수록 촘촘)
SPIRAL_MAX_RADIUS = 1.5  # 나선 최대 반경(m), 원점에서 이 거리 넘으면 복귀
SPIRAL_LOOKAHEAD_M = 0.15  # 나선 경로에서 바라볼 목표점까지의 전방주시 거리(m)
SPIRAL_MAX_ADVANCE = 0.5  # 한 루프에 나선 각도를 최대 이만큼(rad)만 전진(목표점 점프 방지)
SPIRAL_HEADING_KP = 1.5  # 나선 목표점 방향으로 향하는 회전 비례계수
SPIRAL_SKIP_ON_BLOCK = 1.0  # 장애물로 막힐 때마다 나선 진행각을 이만큼(rad) 앞으로 건너뜀(같은 장애물 왕복 방지)
RETURN_KP = 1.0  # 원점 복귀 시 헤딩 오차(rad)에 대한 회전 비례계수
RETURN_DONE_M = 0.10  # 원점에 이 거리(m) 안으로 들어오면 복귀 완료로 보고 나선 재시작
EXPLORE_MAX_W = 1.0  # 탐색 회전 속도 제한
EXPLORE_TURN_SIGN = 1.0  # 탐색 회전 방향(+1: 좌회전, -1: 우회전)
EXPLORE_AVOID_D = 0.28  # 원점 복귀 중 정면/좌/우가 이 거리(m) 이내로 막히면 회피 발동



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


class Tee:
    """터미널(stdout)에 출력하면서 동시에 같은 내용을 로그 파일에도 기록한다."""

    def __init__(self, *streams):
        self.streams = streams
        self.lock = threading.Lock()

    def write(self, data):
        with self.lock:
            for s in self.streams:
                s.write(data)
                s.flush()

    def flush(self):
        with self.lock:
            for s in self.streams:
                s.flush()


class SpiralExplorer:
    """달팽이집(아르키메데스 나선)으로 탐색한다.
    오도메트리로 추적한 위치(x, y)를 기준으로, 원점(0,0)에 중심이 고정된
    나선 경로 r = SPIRAL_GROWTH·theta 위의 전방주시 목표점을 추종한다.
    반경은 SPIRAL_MAX_RADIUS(m)에서 멈추고, 회피로 밀려나도 원래 중심 나선으로 복귀한다."""

    def __init__(self, enc_l, enc_r, turn_sign=EXPLORE_TURN_SIGN):
        # 오도메트리 위치 추적 (시작점이 원점 0,0, 헤딩 0)
        self.prev_l = enc_l
        self.prev_r = enc_r
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.spiral_theta = 0.0  # 원점 고정 나선 경로상의 진행 각도(rad)
        self.returning = False  # True면 원점으로 복귀 중
        self.avoid_anchor = None
        self.turn_sign = turn_sign  # 나선 감기는 방향(+1 좌, -1 우). 인스턴스별 지정 가능
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

    def distance_from_origin(self):
        return float(np.hypot(self.x, self.y))

    def return_command(self, ranges):
        """원점(0,0)을 향해 방향을 맞추며 전진하는 복귀 명령."""
        dist = self.distance_from_origin()
        target_heading = np.arctan2(-self.y, -self.x)  # 원점을 가리키는 방향
        err = (target_heading - self.theta + np.pi) % (2.0 * np.pi) - np.pi  # [-pi, pi]
        target_w = clamp(RETURN_KP * err, -EXPLORE_MAX_W, EXPLORE_MAX_W)
        target_w = apply_symmetric_side_repulsion(target_w, ranges)  # 측면 벽 스침 방지
        # 방향이 많이 틀어졌으면 전진속도를 줄여 제자리에 가깝게 회전부터
        nominal_v = DRIVE_V * max(0.0, 1.0 - abs(err) / (np.pi / 2.0))
        target_v = scale_avoid_speed_for_front_obstacle(nominal_v, ranges)
        self.detail = (
            f"x={self.x:+.2f} y={self.y:+.2f} dist={dist:.2f}m "
            f"head={np.degrees(target_heading):+.0f} theta={np.degrees(self.theta):+.0f} "
            f"err={np.degrees(err):+.0f} v={target_v:.3f} w={target_w:+.3f}"
        )
        return f"RETURN: to origin d={dist:.2f}m", target_v, target_w

    def save_avoid_anchor(self):
        if self.avoid_anchor is not None:
            return

        # 장애물 직전 위치가 아니라, 나선 진행각을 SPIRAL_SKIP_ON_BLOCK만큼 앞으로
        # 건너뛴 지점을 복귀점으로 삼는다. 그래야 복귀 후 재개한 나선이 같은 장애물
        # 방향으로 다시 향하지 않는다(매 막힘마다 누적되어 점점 더 앞으로 빠져나감).
        resume_theta = self.spiral_theta + SPIRAL_SKIP_ON_BLOCK
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
        target_w = apply_symmetric_side_repulsion(target_w, ranges)  # 측면 벽 스침 방지
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

    def spiral_point(self, theta):
        """원점(0,0) 중심 아르키메데스 나선 위의 점. r = b·theta, 최대 반경 제한."""
        r = min(SPIRAL_GROWTH * theta, SPIRAL_MAX_RADIUS)
        ang = self.turn_sign * theta  # 회전 방향(+1 좌, -1 우), 인스턴스별
        return r * np.cos(ang), r * np.sin(ang), r

    def command(self, enc_l, enc_r, ranges):
        # 현재 위치(원점 기준 x, y)에서 나선 경로상 전방주시 목표점을 추종한다.
        # 목표점이 현재 위치에서 SPIRAL_LOOKAHEAD_M 이상 떨어질 때까지 나선 각도를 전진.
        theta = self.spiral_theta
        advanced = 0.0
        tx, ty, radius = self.spiral_point(theta)
        while np.hypot(tx - self.x, ty - self.y) < SPIRAL_LOOKAHEAD_M and advanced < SPIRAL_MAX_ADVANCE:
            theta += 0.05
            advanced += 0.05
            tx, ty, radius = self.spiral_point(theta)
        self.spiral_theta = theta

        # 순수 나선 추종: 목표점 방향과 현재 헤딩의 오차로만 회전/전진(장애물 회피 없음).
        desired_heading = np.arctan2(ty - self.y, tx - self.x)
        err = (desired_heading - self.theta + np.pi) % (2.0 * np.pi) - np.pi  # [-pi, pi]
        target_w = clamp(SPIRAL_HEADING_KP * err, -EXPLORE_MAX_W, EXPLORE_MAX_W)
        target_v = DRIVE_V * max(0.0, 1.0 - abs(err) / (np.pi / 2.0))

        self.detail = (
            f"pos=({self.x:+.2f},{self.y:+.2f}) tgt=({tx:+.2f},{ty:+.2f}) "
            f"sp_theta={np.degrees(theta):.0f}deg radius={radius:.3f}m "
            f"err={np.degrees(err):+.0f} v={target_v:.3f} w={target_w:+.3f}"
        )
        mode = f"EXPLORE: spiral r={radius:.2f}m"
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
        # 색 정렬용 피벗 회전: 한쪽 바퀴 정지, 한쪽 바퀴만 후진 (아두이노에서 처리)
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
    current_v = start_v  # 직전 속도에서 이어받아 램프(0으로 리셋하지 않아 불연속 제거)
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
        # 속도 0으로 방향만 보정: ALIGN 전용 Kp로 회전량 계산
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
    log_file = None
    if DEBUG:  # 디버그가 켜져 있을 때만 터미널+txt 로그를 남긴다.
        log_dir = THIS_DIR / "log"
        log_dir.mkdir(exist_ok=True)
        log_file = open(log_dir / f"robot_log_{time.strftime('%Y%m%d_%H%M%S')}.txt", "w", encoding="utf-8")
        sys.stdout = Tee(original_stdout, log_file)
        print(f"[LOG] logging to {log_file.name}")
    last_v = last_w = 0.0
    last_color_deg = last_color_time = last_color_center_ratio = last_color_x_err = 0.0
    last_odom_log_time = 0.0
    color_lost_during_avoid = False
    explore_active = False
    explorer = None
    switch_search_active = False
    switch_search_start_odom = None
    last_search_start_odom = None  # 마지막 색 방향 제자리 회전의 한바퀴 감지용 엔코더 시작값
    target_index = 0
    current_target = TARGET_SEQUENCE[target_index]

    try:
        cam = open_camera()
        lidar = RPLidarC1()
        motor = Motor()
        motor.stop()
        motor.reset_encoders()
        motor_enabled = threading.Event()

        def wait_for_motor_start():
            input("[READY] Press Enter to enable motor...")
            motor_enabled.set()

        threading.Thread(target=wait_for_motor_start, daemon=True).start()
        start_time = time.time()  # 로그 출력용 시작 시각

        while True:
            elapsed = time.time() - start_time  # 프로그램 시작 후 경과 시간
            ok, frame = read_frame(cam)
            if not ok:
                break

            found = detect(frame)  # 현재 프레임에서 찾은 색상 물체 목록
            target = pick(found, current_target)  # 따라갈 대상 색상 물체
            found = [target] if target is not None else []
            scan, _, _ = lidar.get()  # 최신 라이다 스캔 데이터
            ranges = front_ranges(scan) if scan is not None else None  # 각도별 전방 거리 배열
            lidar_dist = lidar_zone_distances(ranges)  # 좌/정면/우측 로그용 최소 거리
            has_obstacle = obstacle_detected(ranges)  # 장애물 감지 여부

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
                motor_enabled.is_set()
                and target is not None
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
                    explorer.update_pose(enc_l, enc_r)  # 원점 기준 위치 적산
                dist_origin = explorer.distance_from_origin()

                explore_blocked = (
                    front_obstacle_distance(ranges) <= EXPLORE_AVOID_D
                    or obstacle_in_zone(ranges, LEFT_ZONE, EXPLORE_AVOID_D)
                    or obstacle_in_zone(ranges, RIGHT_ZONE, EXPLORE_AVOID_D)
                )

                # 원점에서 1.5m를 벗어나면 원점 복귀 모드 진입
                if not explorer.returning and dist_origin > SPIRAL_MAX_RADIUS:
                    explorer.returning = True
                    explorer.clear_avoid_anchor()
                    print(f"[{elapsed:.2f}s] [EXPLORE] left {SPIRAL_MAX_RADIUS}m (d={dist_origin:.2f}m), returning to origin")

                if explorer.returning:
                    if dist_origin <= RETURN_DONE_M:
                        # 원점 도착 → 복귀 종료, 현재 위치를 새 원점으로 나선 재시작
                        print(f"[{elapsed:.2f}s] [EXPLORE] origin reached (d={dist_origin:.2f}m), restart spiral")
                        explorer = SpiralExplorer(enc_l, enc_r)
                        mode, target_v, target_w = "EXPLORE: origin reached", 0.0, 0.0
                    elif explore_blocked:
                        # 복귀 중에도 장애물이 막으면 회피 우선
                        mode, target_v, target_w = avoid_mode("AVOID: return", "AVOID_RETURN", ranges, None, elapsed)
                    elif odom_time > 0.0:
                        mode, target_v, target_w = explorer.return_command(ranges)
                        dbg(f"[{elapsed:.2f}s] [RETURN] {explorer.detail}")
                    else:
                        mode, target_v, target_w = "RETURN: wait odom", 0.0, 0.0
                elif explore_blocked:
                    explorer.save_avoid_anchor()
                    mode, target_v, target_w = avoid_mode("AVOID: explore", "AVOID_EXPLORE", ranges, None, elapsed)
                elif explorer.has_avoid_anchor():
                    if explorer.reached_avoid_anchor():
                        explorer.resume_from_avoid_anchor()
                        mode, target_v, target_w = "EXPLORE: resume spiral", 0.0, 0.0
                        dbg(f"[{elapsed:.2f}s] [EXPLORE] resumed at saved spiral radius")
                    elif odom_time > 0.0:
                        mode, target_v, target_w = explorer.return_to_avoid_anchor_command(ranges)
                        dbg(f"[{elapsed:.2f}s] [ANCHOR] {explorer.detail}")
                    else:
                        mode, target_v, target_w = "RETURN: wait odom", 0.0, 0.0
                elif odom_time > 0.0:
                    # 순수 나선 회전(장애물 회피 없음)
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
                    explore_active = True
                    enc_l, enc_r, _, _ = motor.get_odom()
                    explorer = SpiralExplorer(enc_l, enc_r)
                    print(f"[{elapsed:.2f}s] [EXPLORE] 360 search failed, spiral explore (max r={SPIRAL_MAX_RADIUS}m)")
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
                    # 처음부터 빨간색을 한 번도 못 봤으면 장애물 회피로 돌아다니며 찾는다.
                    switch_search_active, explore_active, switch_search_start_odom = False, False, None
                    mode, target_v, target_w = avoid_mode("AVOID: no initial color", "AVOID_INITIAL", ranges, None, elapsed)

            # 마지막 색 방향 제자리 회전(SEARCH: last color direction)이 장애물 없이
            # 연속으로 한 바퀴를 채우면 나선 탐색으로 넘어간다(장애물 만나면 카운트 리셋).
            if mode.startswith("SEARCH: last color direction"):
                if has_obstacle:
                    last_search_start_odom = None  # 장애물 발생 → 한바퀴 카운트 리셋
                else:
                    if last_search_start_odom is None:
                        last_search_start_odom = get_turn_start_odom(motor)
                    if completed_one_encoder_turn(motor, last_search_start_odom):
                        last_search_start_odom = None
                        color_lost_during_avoid, switch_search_active = False, False
                        explore_active = True
                        enc_l, enc_r, _, _ = motor.get_odom()
                        # 마지막 색이 오른쪽(>=0)이면 우(-1), 왼쪽이면 좌(+1)로 나선 감기
                        spin_sign = -1.0 if last_color_deg >= 0.0 else 1.0
                        explorer = SpiralExplorer(enc_l, enc_r, turn_sign=spin_sign)
                        print(f"[{elapsed:.2f}s] [EXPLORE] last-color spin 360 done, spiral explore (turn={spin_sign:+.0f}, max r={SPIRAL_MAX_RADIUS}m)")
                        mode, target_v, target_w = explorer.command(enc_l, enc_r, ranges)
                        dbg(f"[{elapsed:.2f}s] [EXPLORE] {explorer.detail}")
            else:
                last_search_start_odom = None  # 다른 모드로 빠지면 회전 카운트 리셋

            if motor_enabled.is_set() and (target is not None or mode.startswith("AVOID") or mode.startswith("SEARCH") or mode.startswith("EXPLORE") or mode.startswith("RETURN") or mode.startswith("ESCAPE") or mode.startswith("FORWARD")):
                last_v = 0.0 if mode.startswith("ALIGN") else rate_limit(last_v, target_v, V_STEP)
                w_step = AVOID_W_STEP if (mode.startswith("AVOID") or mode.startswith("EXPLORE") or mode.startswith("ESCAPE")) else FOLLOW_W_STEP
                last_w = rate_limit(last_w, target_w, w_step)
                if mode.startswith("ALIGN"):
                    # 색 정렬은 한 바퀴 정지 + 한 바퀴 후진 피벗으로 회전
                    motor.pivot(last_w)
                else:
                    motor.vw(last_v, last_w)
            elif not motor_enabled.is_set():
                last_v, last_w = 0.0, 0.0
                motor.stop()

            dbg(
                f"[{elapsed:.2f}s] [STATE] mode={mode} "
                f"target={current_target}:{'O' if target is not None else 'X'} "
                f"cmd(v={target_v:+.2f} w={target_w:+.2f}) sent(v={last_v:+.2f} w={last_w:+.2f}) "
                f"obstacle={has_obstacle} center_ratio={last_color_center_ratio:.2f} "
                f"x_err={last_color_x_err:+.2f} last_deg={last_color_deg:+.0f} "
                f"motor={'ON' if motor_enabled.is_set() else 'OFF'}"
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
        if log_file is not None:
            log_file.close()


if __name__ == "__main__":
    main()
