#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project 3: 라이다 lean DWA + 카메라 색상영역 시퀀스 추적
#   카메라로 현재 타깃 색 영역의 방위 β를 구해 DWA 목표 방위로 사용.
#   미검출 시에는 직진하지 않고 제자리 탐색 회전으로 타깃을 다시 찾는다.
#   도착 판정은 CLOSE→CREEP→HOLD 상태로 패치 위까지 더 들어간 뒤 정지한다.
#   좌표계: 로봇 기준 x=전방, y=좌측(+).  w>0 = 좌회전.  β>0 = 타깃이 좌측.

import time
import math
import threading
import numpy as np
import serial

try:
    import cv2
    _CV2_OK = True
except Exception:
    _CV2_OK = False        # cv2 없으면 카메라 비활성화 → 라이다 단독 동작

# ===================== 설정 =====================
ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

# 라이다 캘리브 (project2 검증값)
ANGLE_OFFSET_DEG = 1.54
DIST_OFFSET_MM = 0.0
LIDAR_ANGLE_SIGN = -1.0
MIN_RANGE_M = 0.05
MAX_RANGE_M = 2.5          # project2의 0.75 캡을 상향 → 더 일찍 반응
MIN_QUALITY = 1
SCAN_HOLD_S = 0.30
MAX_POINTS = 500           # 계산용 최대 포인트 수

# 로봇 / 주행
ROBOT_RADIUS = 0.13
COLLISION_DIST = ROBOT_RADIUS + 0.06   # 이 안으로 들어오는 궤적은 충돌로 간주
SLOW_DIST = 0.35                       # 이 거리부터 감속 시작
CRUISE_V = 0.15                        # 기본 직진 속도 (보수적)
V_MIN_RATIO = 0.4                      # 감속 시 최소 속도 비율
MAX_W = 1.0                            # 최대 회전속도
W_RATE = 0.30                          # 루프당 w 변화 제한

# 엔코더 odometry / 시험 영역 keep-in
ODOM_WHEEL_R = 0.034                   # project3.ino WHEEL_R와 맞춘 값
ODOM_WHEEL_BASE = 0.179                # project3.ino WHEEL_BASE와 맞춘 값
ODOM_PPR = 1012.0                      # project3.ino PPR와 맞춘 값
ODOM_COUNT_PER_RAD = ODOM_PPR / (2.0 * math.pi)
ODOM_START_X = 0.0                     # centered 좌표계 기준 시작 x
ODOM_START_Y = -1.0                    # centered 좌표계 기준 시작 y
ODOM_START_TH = math.pi / 2.0          # +y 방향으로 출발
ODOM_HOLD_S = 0.50                     # 이 시간 이상 엔코더 피드백이 없으면 안전 정지
KEEPIN_ENABLED = True
KEEPIN_SIZE_M = 2.30                   # 2m 시험영역보다 약간 큰 가상 주행 제한
KEEPIN_MARGIN_M = ROBOT_RADIUS         # 로봇 중심이 이만큼 안쪽에 남도록 제한

# DWA
HORIZON_T = 1.2
DT = 0.10
STEPS = int(HORIZON_T / DT)
W_SET = np.linspace(-MAX_W, MAX_W, 11)
CLEAR_CAP = 0.5                        # 이 이상 뚫려 있으면 동일 취급
W_CLEAR = 1.5                          # 여유(clearance) 가중치  ┐ goal/clearance
W_GOAL = 1.0                           # 목표 방향 가중치        ┘ 비율 = 핵심 노브

GOAL_BEARING_FALLBACK = 0.0            # 호환용 상수. Controller는 미검출 시 SEARCH sweep 사용
LOOP_DT = 0.05

# ===================== 카메라 / 색상 인식 =====================
CAM_W = 320
CAM_H = 240
CAM_EXPOSURE = -1.0
FLIP_180 = True                        # 카메라 180° 반전 보정 (sihyun 검증값: 상하+좌우)
HFOV_DEG = 62.0                        # 수평 화각 placeholder (Step 8서 캘리브). 부호가 더 중요
HALF_HFOV_RAD = math.radians(HFOV_DEG) * 0.5
BEARING_SIGN = -1.0                    # cx 우측(+err)→우회전(β<0). 하드웨어서 반대로 돌면 +1.0
VISION_HOLD_S = 0.30                   # 인식 결과 신선도 (이보다 오래되면 미검출 취급)
VISION_MIN_DT = 0.05                   # 비전 처리 최소 주기(최대 ~20Hz로 CPU 양보)

DEFAULT_TARGET = "RED"                 # 시작 타깃 색
TARGET_SEQUENCE = ["RED", "YELLOW", "BLUE"]   # 통과 순서
ARRIVE_CY = 0.85                       # 코앞 감지 → 정지 전 close/creep 진입
CLOSE_APPROACH_V = 0.07                # 코앞 감지 후 카메라를 보며 더 들어가는 저속
CLOSE_APPROACH_MAX_S = 2.20            # 카메라가 계속 보여도 CLOSE를 끝내는 최대 시간
BLIND_CREEP_V = 0.07                   # 카메라가 패치를 잃은 뒤 짧게 더 들어가는 속도
BLIND_CREEP_DIST_M = 0.12              # 패치 위로 바퀴를 올리기 위한 추가 전진 거리
BLIND_CREEP_S = BLIND_CREEP_DIST_M / max(1e-6, BLIND_CREEP_V)  # 호환/리포트용 환산 시간
DWELL_S = 1.10                         # 패치 위 정지 유지 시간
SEARCH_V = 0.06                        # 타깃 미검출 시 천천히 전진하며 탐색
SEARCH_W = 0.45                        # 탐색 중 회전 속도 상한(막혔을 때 회전 fallback)
SEARCH_BEARING = 0.65                  # 미검출 탐색 때 DWA에 주는 가상 목표 방위
SEARCH_SWEEP_S = 3.0                   # 오래 못 찾으면 탐색 방향을 바꾸는 주기
TARGET_LOST_MEMORY_S = 0.60            # 마지막으로 본 방향을 기억하는 시간


def blind_creep_duration():
    """BLIND_CREEP_DIST_M만큼 전진하기 위한 CREEP 지속 시간."""
    return BLIND_CREEP_DIST_M / max(1e-6, BLIND_CREEP_V)


# HSV 범위 (sihyun 검증값).  RED는 H=0/179 양쪽 두 구간.
COLOR_RANGES = {
    "RED": [
        (np.array([0, 100, 80]), np.array([10, 255, 255])),
        (np.array([170, 100, 80]), np.array([179, 255, 255])),
    ],
    "YELLOW": [(np.array([20, 100, 100]), np.array([35, 255, 255]))],
    "BLUE": [(np.array([95, 100, 80]), np.array([130, 255, 255]))],
}
CAM_AREA = float(CAM_W * CAM_H)
MIN_AREA = 300                         # 640x480의 1200을 320x240로 환산(동일 면적비 ≈0.39%)
MIN_EXTENT = 0.45                      # 박스 채움 비율(가는/흩어진 잡음 제거)
BLUR_SIZE = 5
MORPH_KERNEL = np.ones((5, 5), np.uint8)


# ===================== 유틸 =====================
def normalize_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def rate_limit(prev, target, lim):
    return prev + float(np.clip(target - prev, -lim, lim))


# ===================== 아두이노 =====================
def open_arduino():
    ardu = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    time.sleep(2.0)
    print(f"[INFO] Arduino connected: {ARDU_PORT}")
    return ardu


def send_vw(ardu, v, w):
    ardu.write(f"V{v:.3f},{w:.3f}\n".encode())


def stop(ardu):
    ardu.write(b"S\n")


def wrap_rad(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class OdomPose:
    """centered map 좌표계의 로봇 위치. x/y 단위 m, theta 단위 rad."""
    def __init__(self, x=ODOM_START_X, y=ODOM_START_Y, theta=ODOM_START_TH):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

    def as_tuple(self):
        return self.x, self.y, self.theta


class WheelOdometry:
    """Arduino가 보내는 엔코더 누적 카운트(O,L,R,ms)로 Pi에서 위치를 적분."""
    def __init__(self):
        self.pose = OdomPose()
        self.prev_l = None
        self.prev_r = None
        self.last_msg_t = None
        self.last_arduino_ms = None
        self.total_distance_m = 0.0
        self.line_count = 0

    def reset(self, x=ODOM_START_X, y=ODOM_START_Y, theta=ODOM_START_TH):
        self.pose = OdomPose(x, y, theta)
        self.prev_l = None
        self.prev_r = None
        self.last_msg_t = None
        self.last_arduino_ms = None
        self.total_distance_m = 0.0
        self.line_count = 0

    def fresh(self, now):
        return self.last_msg_t is not None and (now - self.last_msg_t) <= ODOM_HOLD_S

    def update_counts(self, enc_l, enc_r, now=None, arduino_ms=None):
        if self.prev_l is None or self.prev_r is None:
            self.prev_l, self.prev_r = enc_l, enc_r
            self.last_msg_t = now if now is not None else time.time()
            self.last_arduino_ms = arduino_ms
            return False

        d_l = enc_l - self.prev_l
        d_r = enc_r - self.prev_r
        self.prev_l, self.prev_r = enc_l, enc_r

        dist_l = (d_l / ODOM_COUNT_PER_RAD) * ODOM_WHEEL_R
        dist_r = (d_r / ODOM_COUNT_PER_RAD) * ODOM_WHEEL_R
        ds = 0.5 * (dist_r + dist_l)
        dth = (dist_r - dist_l) / ODOM_WHEEL_BASE

        th_mid = self.pose.theta + 0.5 * dth
        self.pose.x += ds * math.cos(th_mid)
        self.pose.y += ds * math.sin(th_mid)
        self.pose.theta = wrap_rad(self.pose.theta + dth)
        self.total_distance_m += abs(ds)
        self.last_msg_t = now if now is not None else time.time()
        self.last_arduino_ms = arduino_ms
        return True

    def parse_line(self, line, now=None):
        line = line.strip()
        if not line.startswith("O,"):
            return False
        parts = line.split(",")
        if len(parts) < 4:
            return False
        try:
            enc_l = int(parts[1])
            enc_r = int(parts[2])
            arduino_ms = int(parts[3])
        except ValueError:
            return False
        self.line_count += 1
        self.update_counts(enc_l, enc_r, now=now, arduino_ms=arduino_ms)
        return True

    def poll_serial(self, ardu, now=None, max_lines=20):
        if now is None:
            now = time.time()
        parsed = 0
        try:
            waiting = int(getattr(ardu, "in_waiting", 0))
            while waiting > 0 and parsed < max_lines:
                raw = ardu.readline()
                if not raw:
                    break
                line = raw.decode(errors="ignore") if isinstance(raw, (bytes, bytearray)) else str(raw)
                if self.parse_line(line, now=now):
                    parsed += 1
                waiting = int(getattr(ardu, "in_waiting", 0))
        except Exception:
            return parsed
        return parsed


# ===================== 라이다 =====================
class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.ser.write(bytes([0xA5, 0x40]))   # RESET
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0xA5, 0x20]))   # SCAN
        header = self.ser.read(7)
        if len(header) != 7 or header[0] != 0xA5 or header[1] != 0x5A:
            self.ser.close()
            raise RuntimeError("[LIDAR] Response Header Error")

        self.lock = threading.Lock()
        self.latest_scan = None
        self.scan_time = 0.0
        self.backlog = 0          # 진단용: 직전에 OS 버퍼에 밀려 있던 바이트 수
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        # OS 시리얼 버퍼를 매번 통째로 읽어 비운다 → 라이다 데이터가 누적/지연되지 않음.
        # 잔여(불완전) 바이트는 parser에 보존해 5바이트 패킷 정렬을 유지.
        # 조립된 스캔은 latest_scan 하나만 유지 → 파이썬 객체도 쌓이지 않음.
        parser = bytearray()
        buf_a, buf_d, buf_q = [], [], []
        while self.running:
            try:
                n = self.ser.in_waiting
                self.backlog = n
                chunk = self.ser.read(n if n > 0 else 1)
                if not chunk:
                    continue
                parser.extend(chunk)

                n_pkt = len(parser) // 5
                for i in range(n_pkt):
                    p = parser[i * 5:i * 5 + 5]
                    s_flag = p[0] & 0x01
                    if ((p[0] & 0x02) >> 1) != (1 - s_flag):
                        continue
                    if (p[1] & 0x01) != 1:
                        continue

                    quality = p[0] >> 2
                    angle = ((p[1] >> 1) | (p[2] << 7)) / 64.0
                    dist = (p[3] | (p[4] << 8)) / 4.0

                    if s_flag == 1 and len(buf_a) > 50:
                        with self.lock:
                            self.latest_scan = (
                                np.array(buf_a, dtype=np.float32),
                                np.array(buf_d, dtype=np.float32),
                                np.array(buf_q, dtype=np.float32),
                            )
                            self.scan_time = time.time()
                        buf_a, buf_d, buf_q = [], [], []

                    if dist > 0 and quality > 0:
                        buf_a.append(angle)
                        buf_d.append(dist)
                        buf_q.append(quality)

                del parser[:n_pkt * 5]            # 처리한 패킷 제거, 잔여 바이트 보존

                if len(buf_a) > 2000:             # 회전 경계 미검출 시 무한 증가 방지(안전망)
                    buf_a, buf_d, buf_q = [], [], []
            except (serial.SerialException, OSError) as e:
                print(f"[LIDAR] Serial Error: {e}, retry in 1s")
                time.sleep(1.0)
                parser.clear()
                buf_a, buf_d, buf_q = [], [], []
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass

    def get_scan(self):
        with self.lock:
            return self.latest_scan, self.scan_time

    def close(self):
        self.running = False
        try:
            self.ser.write(bytes([0xA5, 0x25]))   # STOP
        except Exception:
            pass
        time.sleep(0.1)
        try:
            self.ser.close()
        except Exception:
            pass


# 스캔 → 전방(x>0) 장애물 점 (x,y) 배열
def lidar_points_to_xy(scan):
    if scan is None:
        return np.empty((0, 2), dtype=np.float32)

    angles, dists, qualities = scan
    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0
    angle_deg = LIDAR_ANGLE_SIGN * normalize_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG)

    valid = (
        (dist_m >= MIN_RANGE_M)
        & (dist_m <= MAX_RANGE_M)
        & (qualities >= MIN_QUALITY)
    )
    if not valid.any():
        return np.empty((0, 2), dtype=np.float32)

    dist_m = dist_m[valid]
    angle_rad = np.deg2rad(angle_deg[valid])
    x = dist_m * np.cos(angle_rad)
    y = dist_m * np.sin(angle_rad)

    ahead = x > 0.0   # 전방만 (지나친/뒤쪽 점은 무시)
    if not ahead.any():
        return np.empty((0, 2), dtype=np.float32)

    points = np.column_stack((x[ahead], y[ahead])).astype(np.float32)
    if len(points) > MAX_POINTS:
        order = np.argsort(points[:, 0] ** 2 + points[:, 1] ** 2)
        points = points[order[:MAX_POINTS]]
    return points


# ===================== 카메라 색상 인식 =====================
def open_camera():
    """Picamera2 우선, 없으면 USB. (cam, is_picam) 반환. 실패 시 (None, False)."""
    if not _CV2_OK:
        print("[CAM] cv2 미설치 → 카메라 비활성화")
        return None, False
    try:
        from picamera2 import Picamera2
        picam = Picamera2()
        cfg = picam.create_preview_configuration(
            main={"format": "RGB888", "size": (CAM_W, CAM_H)}
        )
        picam.configure(cfg)
        try:
            picam.set_controls({"ExposureValue": CAM_EXPOSURE})
        except Exception:
            pass
        picam.start()
        time.sleep(1.0)
        print(f"[CAM] Picamera2 started {CAM_W}x{CAM_H}")
        return picam, True
    except Exception as e:
        print(f"[CAM] Picamera2 불가({e}) → USB 시도")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        if not cap.isOpened():
            print("[CAM] 카메라 열기 실패 → 비활성화")
            return None, False
        print(f"[CAM] USB camera started {CAM_W}x{CAM_H}")
        return cap, False


class ColorPerception:
    """전용 스레드에서 현재 타깃 색만 인식 → 최신 결과 1칸만 유지(프레임 누적 없음).
       결과 튜플: (visible, bearing_rad, area_ratio, cy_norm, n_blobs)."""

    def __init__(self, target_color=DEFAULT_TARGET):
        self.cam, self.is_picam = open_camera()
        self.enabled = self.cam is not None
        self.target_color = target_color
        self.lock = threading.Lock()
        self.result = (False, 0.0, 0.0, 0.0, 0)
        self.stamp = 0.0
        self.proc_ms = 0.0
        self.running = True
        if self.enabled:
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

    def set_target(self, color):
        with self.lock:
            self.target_color = color

    def get(self):
        with self.lock:
            return self.result, self.stamp

    def _grab_bgr(self):
        if self.is_picam:
            frame = cv2.cvtColor(self.cam.capture_array(), cv2.COLOR_RGB2BGR)
        else:
            ok, frame = self.cam.read()
            if not ok:
                return None
        if FLIP_180:
            frame = cv2.flip(frame, -1)
        return frame

    def _detect(self, frame, color):
        if BLUR_SIZE > 1:
            frame = cv2.GaussianBlur(frame, (BLUR_SIZE, BLUR_SIZE), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = None
        for lo, hi in COLOR_RANGES[color]:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best = None
        best_area = 0.0
        n = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if area / max(1.0, float(w * h)) < MIN_EXTENT:
                continue
            n += 1
            if area > best_area:
                best_area = area
                best = (x, y, w, h)

        if best is None:
            return (False, 0.0, 0.0, 0.0, 0)

        x, y, w, h = best
        cx = x + w * 0.5
        err = (cx - CAM_W * 0.5) / (CAM_W * 0.5)   # -1(좌) .. +1(우)
        bearing = BEARING_SIGN * err * HALF_HFOV_RAD
        area_ratio = best_area / CAM_AREA          # 0..1 (대략 가까울수록 큼)
        cy_norm = (y + h) / CAM_H                  # 영역 하단의 세로위치(1=프레임 바닥=가까움)
        return (True, float(bearing), float(area_ratio), float(cy_norm), n)

    def _loop(self):
        while self.running:
            t0 = time.time()
            frame = self._grab_bgr()
            if frame is None:
                time.sleep(0.02)
                continue
            with self.lock:
                color = self.target_color
            res = self._detect(frame, color)
            with self.lock:
                self.result = res
                self.stamp = time.time()
                self.proc_ms = (self.stamp - t0) * 1000.0
            dt = time.time() - t0
            if dt < VISION_MIN_DT:
                time.sleep(VISION_MIN_DT - dt)

    def close(self):
        self.running = False
        time.sleep(0.1)
        try:
            if self.is_picam:
                self.cam.stop()
            elif self.cam is not None:
                self.cam.release()
        except Exception:
            pass


# ===================== lean DWA =====================
_TS = (np.arange(STEPS) + 1) * DT   # 예측 시점들


# (v,w)로 HORIZON_T 동안 그릴 로봇 기준 궤적 (xs, ys)
def predict_xy(v, w):
    if abs(w) < 1e-4:
        return v * _TS, np.zeros_like(_TS)
    r = v / w
    return r * np.sin(w * _TS), r * (1.0 - np.cos(w * _TS))


# 궤적이 장애물 점들에 가장 가까이 지나가는 거리
def path_clearance(v, w, points):
    if len(points) == 0:
        return MAX_RANGE_M
    xs, ys = predict_xy(v, w)
    dx = xs[:, None] - points[None, :, 0]
    dy = ys[:, None] - points[None, :, 1]
    return float(np.sqrt(dx * dx + dy * dy).min())


def _pose_tuple(pose):
    if pose is None:
        return None
    if hasattr(pose, "x") and hasattr(pose, "y") and hasattr(pose, "theta"):
        return float(pose.x), float(pose.y), float(pose.theta)
    try:
        if len(pose) >= 3:
            return float(pose[0]), float(pose[1]), float(pose[2])
    except TypeError:
        return None
    return None


def keepin_boundary_margin(v, w, pose):
    """예측 궤적의 2.3m keep-in 경계 여유. 0 미만이면 영역 밖으로 나가는 후보."""
    if not KEEPIN_ENABLED:
        return MAX_RANGE_M
    pose_t = _pose_tuple(pose)
    if pose_t is None:
        return MAX_RANGE_M

    px, py, th = pose_t
    allowed_half = KEEPIN_SIZE_M * 0.5 - KEEPIN_MARGIN_M
    if allowed_half <= 0.0:
        return -MAX_RANGE_M

    xs, ys = predict_xy(v, w)
    cs, sn = math.cos(th), math.sin(th)
    gx = px + xs * cs - ys * sn
    gy = py + xs * sn + ys * cs

    left = gx + allowed_half
    right = allowed_half - gx
    bottom = gy + allowed_half
    top = allowed_half - gy
    return float(np.minimum.reduce((left, right, bottom, top)).min())


def command_clearance(v, w, points, pose=None):
    """장애물 여유와 keep-in 경계 여유를 함께 반영한 DWA용 clearance."""
    obs_clr = path_clearance(v, w, points)
    bnd_margin = keepin_boundary_margin(v, w, pose)
    # 경계는 margin 0이 '딱 제한선'이므로 COLLISION_DIST 기준 clearance로 변환한다.
    bnd_clr = COLLISION_DIST + bnd_margin
    return min(obs_clr, bnd_clr)


# 후보 w들을 평가해 (v, w) 선택.  반환: v, w, best_clear, blocked
def choose_cmd(points, goal_bearing, prev_w, pose=None):
    best_w = 0.0
    best_clear = 0.0
    best_score = -float("inf")

    for w in W_SET:
        clr = command_clearance(CRUISE_V, w, points, pose)
        if clr < COLLISION_DIST:
            score = -1000.0 + clr            # 충돌 후보: 강한 페널티(여유 큰 순으로 정렬)
        else:
            theta_f = w * HORIZON_T
            goal_align = 0.5 * (1.0 + math.cos(theta_f - goal_bearing))
            clear_norm = min(clr, CLEAR_CAP) / CLEAR_CAP
            score = W_CLEAR * clear_norm + W_GOAL * goal_align

        if score > best_score:
            best_score = score
            best_w = w
            best_clear = clr

    if best_clear < COLLISION_DIST:
        return 0.0, 0.0, best_clear, True    # 막힘 → 정지 (탈출은 Step 5)

    # 가까울수록 감속
    span = max(1e-6, SLOW_DIST - COLLISION_DIST)
    v = CRUISE_V * float(np.clip((best_clear - COLLISION_DIST) / span, V_MIN_RATIO, 1.0))
    w = rate_limit(prev_w, best_w, W_RATE)
    return v, w, best_clear, False


# ===================== 시퀀스 컨트롤러 =====================
# 한 제어주기의 의사결정 = main() 과 시뮬레이터가 공유하는 단일 진실원.
# 여기(상수/choose_cmd/tick)를 고치면 시뮬레이터 동작도 그대로 바뀐다
# (sim 이 이 코드를 import 해 직접 호출하므로 글루를 다시 짤 필요가 없다).
class Controller:
    """색 시퀀스 추적 + 패치 위로 더 들어간 뒤 정지 + DWA 조향."""

    def __init__(self):
        self.seq_idx = 0           # 현재 추적 색 = TARGET_SEQUENCE[seq_idx]
        self.phase = "SEEK"
        self.close_until = 0.0
        self.creep_until = 0.0
        self.dwell_until = 0.0     # >0 이면 패치 위 정지 유지가 끝나는 시각
        self.last_w = 0.0
        self.goal = 0.0            # 직전 목표 방위 (로깅/시각화용)
        self.clr = 0.0             # 직전 여유거리
        self.last_seen_t = -1e9
        self.last_seen_bearing = 0.0
        self.search_dir = 1.0
        self.search_switch_t = 0.0

    @property
    def done(self):
        return self.seq_idx >= len(TARGET_SEQUENCE)

    @property
    def target_color(self):
        return None if self.done else TARGET_SEQUENCE[self.seq_idx]

    def _reset_for_next_target(self):
        self.phase = "SEEK"
        self.close_until = 0.0
        self.creep_until = 0.0
        self.dwell_until = 0.0
        self.last_w = 0.0
        self.goal = 0.0
        self.clr = 0.0
        self.last_seen_t = -1e9
        self.last_seen_bearing = 0.0
        self.search_dir = 1.0
        self.search_switch_t = 0.0

    def _note_seen(self, bearing, now):
        self.last_seen_t = now
        self.last_seen_bearing = bearing
        if abs(bearing) > 0.05:
            self.search_dir = 1.0 if bearing > 0.0 else -1.0

    def _straight_creep_cmd(self, points, v, state, pose=None):
        self.goal = 0.0
        self.clr = command_clearance(max(v, 0.05), 0.0, points, pose)
        if self.clr < COLLISION_DIST:
            self.last_w = 0.0
            return 0.0, 0.0, "BLOCKED"
        self.last_w = 0.0
        return v, 0.0, state

    def _search_cmd(self, points, now, pose=None):
        if now - self.last_seen_t <= TARGET_LOST_MEMORY_S:
            if abs(self.last_seen_bearing) > 0.05:
                self.search_dir = 1.0 if self.last_seen_bearing > 0.0 else -1.0
                goal = max(-SEARCH_BEARING, min(SEARCH_BEARING, self.last_seen_bearing))
            else:
                goal = SEARCH_BEARING * self.search_dir
        else:
            if self.search_switch_t <= 0.0:
                self.search_switch_t = now + SEARCH_SWEEP_S
            elif now >= self.search_switch_t:
                self.search_dir *= -1.0
                self.search_switch_t = now + SEARCH_SWEEP_S
            goal = SEARCH_BEARING * self.search_dir

        self.goal = goal
        turn_scale = max(0.35, min(1.0, abs(goal) / max(1e-6, SEARCH_BEARING)))
        target_w = SEARCH_W * (1.0 if goal >= 0.0 else -1.0) * turn_scale

        # SEARCH는 카메라 footprint를 쓸어야 하므로 CRUISE_V 기준 DWA보다
        # 느린 실제 탐색 속도(SEARCH_V) 기준으로 안전한 후보를 고른다.
        chosen_w = target_w
        chosen_clr = -1.0
        for cand_w in (target_w, 0.0, -target_w):
            clr = command_clearance(SEARCH_V, cand_w, points, pose)
            if clr > chosen_clr:
                chosen_w, chosen_clr = cand_w, clr
            if clr >= COLLISION_DIST:
                chosen_w, chosen_clr = cand_w, clr
                break

        if chosen_clr < COLLISION_DIST:
            rot_w = SEARCH_W * self.search_dir
            rot_clr = command_clearance(0.0, rot_w, points, pose)
            self.clr = rot_clr
            if rot_clr >= COLLISION_DIST:
                w = rate_limit(self.last_w, rot_w, W_RATE)
                self.last_w = w
                return 0.0, w, "SEARCH"
            self.last_w = 0.0
            return 0.0, 0.0, "BLOCKED"

        self.clr = chosen_clr
        w = rate_limit(self.last_w, chosen_w, W_RATE)
        self.last_w = w
        return SEARCH_V, w, "SEARCH"

    def tick(self, points, seen, bearing, cy_norm, now, pose=None):
        """한 주기 의사결정. 반환 (v, w, state).
           state ∈ DONE/HOLD/ARRIVE/CREEP/CLOSE/SEARCH/BLOCKED/SEEK.
           색 전환·정지 타이머는 내부 갱신."""
        self.goal = 0.0
        self.clr = 0.0
        if self.done:                              # 완주 → 정지 유지
            self.last_w = 0.0
            return 0.0, 0.0, "DONE"

        if seen:
            self._note_seen(bearing, now)

        if self.phase == "HOLD":                   # 패치 위 정지 유지
            self.last_w = 0.0
            if now >= self.dwell_until:
                self.seq_idx += 1                  # 다음 색 (넘치면 done)
                self._reset_for_next_target()
            return 0.0, 0.0, "HOLD"

        if self.phase == "CREEP":                  # 카메라가 잃은 뒤 짧게 더 전진
            if now < self.creep_until:
                return self._straight_creep_cmd(points, BLIND_CREEP_V, "CREEP", pose)
            self.phase = "HOLD"
            self.dwell_until = now + DWELL_S
            self.last_w = 0.0
            return 0.0, 0.0, "ARRIVE"

        if self.phase == "CLOSE":                  # 코앞 감지 후 바로 멈추지 않고 진입
            if now >= self.close_until:
                self.phase = "CREEP"
                self.creep_until = now + blind_creep_duration()
                return self._straight_creep_cmd(points, BLIND_CREEP_V, "CREEP", pose)
            if seen:
                self.goal = bearing
                v, w, clr, blocked = choose_cmd(points, self.goal, self.last_w, pose)
                self.last_w = w
                self.clr = clr
                if blocked:
                    return 0.0, 0.0, "BLOCKED"
                return min(v, CLOSE_APPROACH_V), w, "CLOSE"
            self.phase = "CREEP"
            self.creep_until = now + blind_creep_duration()
            return self._straight_creep_cmd(points, BLIND_CREEP_V, "CREEP", pose)

        if seen and cy_norm >= ARRIVE_CY:
            self.phase = "CLOSE"
            self.close_until = now + CLOSE_APPROACH_MAX_S
            self.goal = bearing
            v, w, clr, blocked = choose_cmd(points, self.goal, self.last_w, pose)
            self.last_w = w
            self.clr = clr
            if blocked:
                return 0.0, 0.0, "BLOCKED"
            return min(v, CLOSE_APPROACH_V), w, "CLOSE"

        if not seen:
            return self._search_cmd(points, now, pose)

        self.goal = bearing
        v, w, clr, blocked = choose_cmd(points, self.goal, self.last_w, pose)
        self.last_w = w
        self.clr = clr
        return v, w, ("BLOCKED" if blocked else "SEEK")


# ===================== 메인 =====================
def main():
    ardu = open_arduino()
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    cam = ColorPerception(TARGET_SEQUENCE[0])
    time.sleep(2.0)
    stop(ardu)

    print(f"[INFO] camera={'ON' if cam.enabled else 'OFF'}, target={cam.target_color}")
    print("[INFO] Init complete. Press Enter to start.")
    try:
        input()
    except EOFError:
        print("[WARN] No stdin. Starting immediately.")
    try:
        ardu.reset_input_buffer()
    except Exception:
        pass
    print("[INFO] Go!!")

    ctrl = Controller()
    odom = WheelOdometry()
    last_log = 0.0

    try:
        while True:
            now = time.time()
            odom.poll_serial(ardu, now)
            if KEEPIN_ENABLED and not odom.fresh(now):
                send_vw(ardu, 0.0, 0.0)
                ctrl.last_w = 0.0
                if now - last_log > 0.3:
                    print("[ODOM_WAIT] encoder feedback missing; holding stop")
                    last_log = now
                time.sleep(LOOP_DT)
                continue

            scan, scan_time = lidar.get_scan()

            # 신선한 스캔이 없으면 안전하게 정지
            if scan is None or now - scan_time > SCAN_HOLD_S:
                send_vw(ardu, 0.0, 0.0)
                ctrl.last_w = 0.0
                time.sleep(LOOP_DT)
                continue

            points = lidar_points_to_xy(scan)
            cam.set_target(ctrl.target_color)      # 시퀀스 진행에 맞춰 카메라 타깃 동기화
            (visible, bearing, area_ratio, cy_norm, ncnt), cam_stamp = cam.get()
            seen = visible and (now - cam_stamp) <= VISION_HOLD_S

            v, w, state = ctrl.tick(points, seen, bearing, cy_norm, now, pose=odom.pose)
            send_vw(ardu, v, w)
            if ctrl.done:
                print("[DONE] RED→YELLOW→BLUE 완주")
                break

            if now - last_log > 0.3:
                bnd = keepin_boundary_margin(max(v, 0.05), w, odom.pose)
                print(
                    f"[{state}] tgt={ctrl.target_color} see={'Y' if seen else 'n'} "
                    f"gb={ctrl.goal:+.2f} v={v:.2f} w={w:+.2f} "
                    f"cy={cy_norm:.2f} clr={ctrl.clr:.2f} pts={len(points)} "
                    f"pose=({odom.pose.x:+.2f},{odom.pose.y:+.2f},{math.degrees(odom.pose.theta):+.0f}) "
                    f"bnd={bnd:+.2f} bk={lidar.backlog} vms={cam.proc_ms:.0f}"
                )
                last_log = now

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        pass
    finally:
        stop(ardu)
        time.sleep(0.2)
        ardu.close()
        lidar.close()
        cam.close()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
