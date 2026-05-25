#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project 3: 라이다 lean DWA + 카메라 색상영역 시퀀스 추적
#   카메라로 현재 타깃 색 영역의 방위 β를 구해 DWA 목표 방위로 사용.
#   미검출 시에는 제자리 스윕 후 odometry 기반 waypoint를 훑으며 전역 탐색한다.
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
SLOW_DIST = 0.22                       # 이 거리부터 감속 시작
CRUISE_V = 2.0                         # 기본 직진 속도
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
V_SET_RATIOS = np.array([1.0, 0.70, 0.47, 0.33])
V_SET_MIN = 0.03
V_SET = np.maximum(V_SET_MIN, CRUISE_V * V_SET_RATIOS)
CLEAR_CAP = 0.5                        # 이 이상 뚫려 있으면 동일 취급
W_CLEAR = 1.5                          # 여유(clearance) 가중치  ┐ goal/clearance
W_GOAL = 1.0                           # 목표 방향 가중치        ┘ 비율 = 핵심 노브
W_SPEED = 0.25                         # 같은 안전도면 빠른 후보를 약간 선호
W_TURN = 0.08                          # 불필요하게 큰 회전을 약간 억제
W_SMOOTH = 0.10                        # 직전 회전속도와 가까운 후보를 약간 선호

# 목표 방향이 장애물로 막혔을 때 라이다 gap을 임시 목표로 쓰는 회피.
GAP_BEARING_MAX = 1.20                 # gap 후보 최대 좌우 방위각(rad)
GAP_BEARING_STEP = 0.15                # gap 후보 샘플 간격(rad)
GAP_WINDOW_RAD = 0.22                  # 후보 방위 주변 장애물 확인 폭(rad)
GAP_MIN_CLEAR_MARGIN = 0.02            # gap 후보가 가져야 하는 최소 추가 여유
GAP_PASS_LOOKAHEAD_M = 0.75            # gap 후보가 실제 통로인지 확인할 고정 전방 거리
GAP_PASS_SIDE_MARGIN_M = 0.06          # 로봇 좌우 폭에 더할 통과 여유
GAP_W_PASS = 1.00                      # 통로 폭 여유 점수 비중
GAP_W_CLEAR = 1.20
GAP_W_TARGET = 1.00
GAP_W_TURN = 0.15
GAP_W_AWAY = 0.60
AVOID_TRIGGER_MARGIN = 0.02            # 직접 경로가 이 여유보다 좁으면 gap 회피를 검토
AVOID_RELEASE_MARGIN = 0.06            # 직접 목표 경로가 이만큼 여유로워지면 회피 해제
AVOID_LOST_MEMORY_S = 1.50             # 회피 중 색을 잠깐 잃어도 gap 방향으로 유지하는 시간
ESCAPE_ROTATE_MARGIN = 0.01            # 안전 여유 안에서도 회전 탈출을 허용할 최소 몸체 여유
ESCAPE_CREEP_MAX_V = 0.035             # 안전 여유 안에서 허용할 탈출용 최대 저속 전진
ESCAPE_REVERSE_V = 0.04                # 안전 여유 안쪽에서만 쓰는 후진 탈출 속도 크기
ESCAPE_REVERSE_MIN_GAIN = 0.002        # 후진 후보가 현재 여유보다 최소 이만큼 좋아야 채택

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

# practice/camera_2/practice10.py 의 임시 카메라 행렬(640x480 기준)을 현재 해상도에 맞춰 축소.
# 체커보드 실측 전까지만 쓰는 초기값이며, 중심 정렬은 아래 바닥 footprint 실측값과 함께 근사한다.
PRACTICE10_FX_640 = 700.0
PRACTICE10_FY_480 = 700.0
PRACTICE10_CX_640 = 320.0
PRACTICE10_CY_480 = 240.0
CAMERA_FX_PX = PRACTICE10_FX_640 * (CAM_W / 640.0)
CAMERA_FY_PX = PRACTICE10_FY_480 * (CAM_H / 480.0)
CAMERA_CX_PX = PRACTICE10_CX_640 * (CAM_W / 640.0)
CAMERA_CY_PX = PRACTICE10_CY_480 * (CAM_H / 480.0)

# 실측한 top-down 바닥 시야. 로봇 좌표 x=전방, y=좌측.
ROBOT_BODY_LENGTH_M = 0.165
ROBOT_BODY_WIDTH_M = 0.21
CAMERA_FORWARD_OFFSET_M = -0.05
CAMERA_LEFT_OFFSET_M = 0.0
CAMERA_REAR_BLIND_M = 0.25
CAMERA_RECT_WIDTH_M = 0.39
CAMERA_RECT_DEPTH_M = 0.54

# ===================== 시퀀스 추적 / 탐색 동작 =====================
DEFAULT_TARGET = "RED"                 # 시작 타깃 색
TARGET_SEQUENCE = ["RED", "YELLOW", "BLUE"]   # 통과 순서
ARRIVE_CY = 0.85                       # 코앞 감지 → 정지 전 close/creep 진입
APPROACH_CY = 0.65                     # 이 이상 가까우면 저속 정렬 접근으로 전환
APPROACH_ALIGN_MAX = 0.12              # 이 방위각 안에 안정적으로 들어와야 commit 허용
APPROACH_STABLE_S = 0.10               # 정렬 상태가 유지되어야 하는 최소 시간
BEARING_SMOOTH_ALPHA = 0.35            # 카메라 bearing 저역통과 필터 비율
CLOSE_LOST_HOLD_CY = 0.95              # 이만큼 가까이 본 뒤 잃으면 패치 위로 보고 정지
CLOSE_APPROACH_V = 0.07                # 코앞 감지 후 카메라를 보며 더 들어가는 저속
CLOSE_APPROACH_MAX_S = 2.20            # 카메라가 계속 보여도 CLOSE를 끝내는 최대 시간
BLIND_CREEP_V = 0.07                   # 카메라가 패치를 잃은 뒤 짧게 더 들어가는 속도
BLIND_CREEP_DIST_M = 0.12              # 패치 위로 바퀴를 올리기 위한 추가 전진 거리
CREEP_STEER_GAIN = 0.45                # CREEP 중 마지막 bearing을 유지하는 작은 조향 게인
CREEP_MAX_W = 0.16                     # CREEP 중 보정 회전속도 상한
DWELL_S = 1.20                         # 패치 위 정지 유지 시간
CENTER_TOL_M = 0.055                   # 로봇 중심과 색상 중심의 허용 오차
CENTER_MAX_V = CRUISE_V                # 중심 정렬 접근 속도; v_scale(거리 비례)이 감속 담당
CENTER_SLOW_RADIUS_M = 0.35            # 중심 목표에 가까워질수록 감속하는 반경
CENTER_GOAL_ALPHA = 0.35               # 색상 중심 world 좌표 저역통과 비율
CENTER_LOST_MEMORY_S = 2.00            # 색을 잃어도 마지막 중심 좌표로 마무리하는 시간
CENTER_MAX_BEARING = 1.20              # 중심 목표 방위각 제한
SEARCH_W = 0.80                        # 탐색 스윕 회전 속도 상한
SEARCH_SWEEP_ANGLE_DEG = 35.0          # 현재 자세 기준 좌우로 훑는 각도
SEARCH_SWEEP_ANGLE_RAD = math.radians(SEARCH_SWEEP_ANGLE_DEG)
SEARCH_SETTLE_RAD = 0.08               # 목표 스윕 각도에 이 정도 가까우면 반대로 전환
SEARCH_SCAN_EDGE_HITS = 2              # 좌/우 끝점을 이 횟수만큼 훑고도 안 보이면 전진 탐색
SEARCH_DRIVE_V = CRUISE_V              # 스캔 후 미검출 시 전진 탐색 속도
SEARCH_DRIVE_S = 1.20                  # 한 번 스캔한 뒤 전진 탐색하는 시간
RE_SCAN_DIST_M = 0.80                 # 이 거리 이상 이동 후 타깃 미검출이면 제자리 재스캔
SEARCH_SWEEP_S = 0.80                  # odometry가 없을 때 방향을 바꾸는 시간 fallback
TARGET_LOST_MEMORY_S = 0.60            # 마지막으로 본 방향을 기억하는 시간

# Odometry 기반 전역 탐색. 색이 안 보이면 2.3m keep-in 영역 안의 waypoint를 훑는다.
EXPLORE_ENABLED = True
EXPLORE_SPACING_M = 0.55               # 카메라 폭 0.39m보다 약간 작게 겹치며 훑는 간격
EXPLORE_EDGE_MARGIN_M = 0.12           # keep-in 경계에서 waypoint가 추가로 떨어질 거리
EXPLORE_REACH_DIST_M = 0.12            # waypoint 도착 판정 거리
EXPLORE_SCAN_EDGE_HITS = 0             # waypoint 도착 후 스캔 횟수. 0이면 멈추지 않고 다음 지점으로 간다
EXPLORE_MAX_V = CRUISE_V               # 전역 탐색 중 최대 전진 속도
EXPLORE_SLOW_DIST_M = 0.28             # waypoint 근처 감속 거리
EXPLORE_TURN_IN_PLACE_RAD = 1.10       # 목표가 크게 옆/뒤면 먼저 제자리 회전
EXPLORE_BLOCKED_S = 0.90               # 이 시간 이상 막히면 해당 waypoint를 포기
EXPLORE_STUCK_S = 2.00                 # 거의 움직이지 못하면 다른 waypoint로 전환
EXPLORE_STUCK_DIST_M = 0.04            # stuck 판정에 쓰는 최소 이동량

# 목표가 아닌 색을 보았을 때의 위치 기억. 나중에 그 색이 목표가 되면 먼저 찾아가고,
# 현재 목표가 아니면 그 주변 waypoint를 후순위로 밀어 탐색 시간을 줄인다.
COLOR_MEMORY_TTL_S = 240.0
WRONG_COLOR_AVOID_RADIUS_M = 0.55
WRONG_COLOR_SCORE_PENALTY = 1.60
TARGET_MEMORY_REACHED_DIST_M = 0.35
TARGET_MEMORY_MAX_V = CRUISE_V


def blind_creep_duration():
    """BLIND_CREEP_DIST_M만큼 전진하기 위한 CREEP 지속 시간."""
    return BLIND_CREEP_DIST_M / max(1e-6, BLIND_CREEP_V)


def camera_ground_forward_range_robot():
    """로봇 중심 기준 카메라 바닥 footprint의 전방 시작/끝 x 좌표."""
    rear_x = -ROBOT_BODY_LENGTH_M * 0.5
    near_x = rear_x + CAMERA_REAR_BLIND_M
    return near_x, near_x + CAMERA_RECT_DEPTH_M


def pixel_to_robot_ground(u_px, v_px):
    """색상 중심 픽셀을 로봇 기준 바닥 좌표(x=전방, y=좌측)로 근사 변환.

    practice10.py의 임시 intrinsics로 픽셀을 정규화하고, 실측한 직사각형
    바닥 시야(CAMERA_RECT_WIDTH/DEPTH)에 선형 대응시킨다.
    """
    near_x, far_x = camera_ground_forward_range_robot()
    top_ray = (0.0 - CAMERA_CY_PX) / max(1e-6, CAMERA_FY_PX)
    bottom_ray = (CAM_H - CAMERA_CY_PX) / max(1e-6, CAMERA_FY_PX)
    ray_v = (float(v_px) - CAMERA_CY_PX) / max(1e-6, CAMERA_FY_PX)
    frac_down = float(np.clip((ray_v - top_ray) / max(1e-6, bottom_ray - top_ray), 0.0, 1.0))
    x_robot = far_x - frac_down * (far_x - near_x)

    left_ray = (0.0 - CAMERA_CX_PX) / max(1e-6, CAMERA_FX_PX)
    right_ray = (CAM_W - CAMERA_CX_PX) / max(1e-6, CAMERA_FX_PX)
    ray_u = (float(u_px) - CAMERA_CX_PX) / max(1e-6, CAMERA_FX_PX)
    frac_right = float(np.clip((ray_u - left_ray) / max(1e-6, right_ray - left_ray), 0.0, 1.0))
    y_camera = (0.5 - frac_right) * CAMERA_RECT_WIDTH_M

    return (
        float(x_robot),
        float(CAMERA_LEFT_OFFSET_M + y_camera),
    )


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
MIN_AREA = 20                          # 작은 일부 노출도 쓰되, 점 잡음만 거르는 최소 픽셀 면적
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
    """전용 스레드에서 색상 인식 → 최신 결과 1칸만 유지(프레임 누적 없음).
       결과 튜플:
       (visible, bearing_rad, area_ratio, cy_norm, n_blobs, target_x_m, target_y_m, all_detections)."""

    def __init__(self, target_color=DEFAULT_TARGET):
        self.cam, self.is_picam = open_camera()
        self.enabled = self.cam is not None
        self.target_color = target_color
        self.lock = threading.Lock()
        self.result = (False, 0.0, 0.0, 0.0, 0, None, None, {})
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

    def _detect_color_from_hsv(self, hsv, color):
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
        best_cnt = None
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
                best_cnt = cnt

        if best is None:
            return (False, 0.0, 0.0, 0.0, 0, None, None)

        x, y, w, h = best
        # bounding box보다 contour moment 중심이 색종이 중심 추정에 더 안정적이다.
        cx = x + w * 0.5
        cy = y + h * 0.5
        if best_cnt is not None:
            m = cv2.moments(best_cnt)
            if abs(m["m00"]) > 1e-6:
                cx = m["m10"] / m["m00"]
                cy = m["m01"] / m["m00"]
        err = (cx - CAM_W * 0.5) / (CAM_W * 0.5)   # -1(좌) .. +1(우)
        bearing = BEARING_SIGN * err * HALF_HFOV_RAD
        area_ratio = best_area / CAM_AREA          # 0..1 (대략 가까울수록 큼)
        cy_norm = (y + h) / CAM_H                  # 영역 하단의 세로위치(1=프레임 바닥=가까움)
        target_x, target_y = pixel_to_robot_ground(cx, cy)
        return (True, float(bearing), float(area_ratio), float(cy_norm), n, target_x, target_y)

    def _detect(self, frame, color):
        if BLUR_SIZE > 1:
            frame = cv2.GaussianBlur(frame, (BLUR_SIZE, BLUR_SIZE), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        all_detections = {}
        for c in TARGET_SEQUENCE:
            res = self._detect_color_from_hsv(hsv, c)
            if not res[0]:
                continue
            all_detections[c] = {
                "visible": True,
                "bearing_rad": res[1],
                "area_ratio": res[2],
                "cy_norm": res[3],
                "n_blobs": res[4],
                "target_center_robot_frame_m": {"x_m": res[5], "y_m": res[6]},
            }

        if color in all_detections:
            d = all_detections[color]
            center = d["target_center_robot_frame_m"]
            return (
                True,
                d["bearing_rad"],
                d["area_ratio"],
                d["cy_norm"],
                d["n_blobs"],
                center["x_m"],
                center["y_m"],
                all_detections,
            )
        return (False, 0.0, 0.0, 0.0, 0, None, None, all_detections)

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


def _speed_candidates():
    """CRUISE_V에서 저속까지 내려가며 DWA 후보 속도를 만든다."""
    vals = [CRUISE_V, V_SET_MIN]
    try:
        vals.extend(float(v) for v in np.asarray(V_SET).ravel())
    except Exception:
        pass
    vals = [max(0.0, float(v)) for v in vals if float(v) > 1e-6]
    vals = sorted({round(v, 5): v for v in vals}.values(), reverse=True)
    return vals if vals else [max(0.0, CRUISE_V)]


def escape_rotate_clearance_min():
    """전진은 금지하되 제자리 회전 탈출은 허용할 최소 라이다 여유."""
    return ROBOT_RADIUS + ESCAPE_ROTATE_MARGIN


def _escape_rotate_cmd(points, goal_bearing, prev_w, pose=None):
    if command_clearance(0.0, 0.0, points, pose) < escape_rotate_clearance_min():
        return None

    if abs(goal_bearing) > 0.05:
        target_w = max(-MAX_W, min(MAX_W, goal_bearing / max(0.3, HORIZON_T * 0.5)))
    elif abs(prev_w) > 0.05:
        target_w = math.copysign(max(0.25, 0.35 * MAX_W), prev_w)
    else:
        target_w = max(0.25, 0.35 * MAX_W)

    w = rate_limit(prev_w, target_w, W_RATE)
    if abs(w) < 1e-4:
        w = math.copysign(min(MAX_W, max(0.25, 0.35 * MAX_W)), target_w or 1.0)
    clr = command_clearance(0.0, w, points, pose)
    if clr >= escape_rotate_clearance_min():
        return 0.0, w, clr, False
    return None


def _escape_reverse_cmd(points, goal_bearing, prev_w, pose=None):
    """전방 장애물에 너무 붙었을 때만 쓰는 짧은 후진 탈출 후보."""
    current = command_clearance(0.0, 0.0, points, pose)
    if current >= COLLISION_DIST or current < escape_rotate_clearance_min():
        return None

    v = -min(abs(ESCAPE_REVERSE_V), max(0.02, CRUISE_V * 0.5))
    best = None
    max_w = max(1e-6, abs(MAX_W))
    for w in W_SET:
        w = float(w)
        clr = command_clearance(v, w, points, pose)
        if clr < current + ESCAPE_REVERSE_MIN_GAIN:
            continue
        theta_f = w * HORIZON_T
        goal_align = 0.5 * (1.0 + math.cos(wrap_rad(theta_f - goal_bearing)))
        smooth_norm = abs(w - prev_w) / max_w
        score = min(clr, CLEAR_CAP) / max(1e-6, CLEAR_CAP) + 0.20 * goal_align - W_SMOOTH * smooth_norm
        if best is None or score > best[0]:
            best = (score, w, clr)

    if best is None:
        return None
    w = rate_limit(prev_w, best[1], W_RATE)
    clr = command_clearance(v, w, points, pose)
    if clr >= current + ESCAPE_REVERSE_MIN_GAIN:
        return v, w, clr, False
    return None


# 후보 (v,w)들을 평가해 선택.  반환: v, w, best_clear, blocked
def choose_cmd(points, goal_bearing, prev_w, pose=None):
    best_v = 0.0
    best_w = 0.0
    best_clear = 0.0
    best_score = -float("inf")
    best_blocked_clear = -float("inf")
    best_escape_v = 0.0
    best_escape_w = 0.0
    best_escape_score = -float("inf")

    max_w = max(1e-6, abs(MAX_W))
    cruise_v = max(1e-6, CRUISE_V)
    clear_cap = max(1e-6, CLEAR_CAP)
    escape_min = escape_rotate_clearance_min()
    escape_v_max = max(0.0, ESCAPE_CREEP_MAX_V)

    for v in _speed_candidates():
        for w in W_SET:
            w = float(w)
            clr = command_clearance(v, w, points, pose)
            if clr > best_blocked_clear:
                best_blocked_clear = clr

            if clr < COLLISION_DIST:
                if 0.0 < v <= escape_v_max + 1e-6 and clr >= escape_min:
                    theta_f = w * HORIZON_T
                    goal_align = 0.5 * (1.0 + math.cos(wrap_rad(theta_f - goal_bearing)))
                    clear_norm = min(clr, clear_cap) / clear_cap
                    speed_norm = min(v, cruise_v) / cruise_v
                    smooth_norm = abs(w - prev_w) / max_w
                    esc_score = (
                        W_CLEAR * clear_norm
                        + 0.7 * W_GOAL * goal_align
                        - 0.35 * speed_norm
                        - W_SMOOTH * smooth_norm
                    )
                    if esc_score > best_escape_score:
                        best_escape_score = esc_score
                        best_escape_v = v
                        best_escape_w = w
                continue

            theta_f = w * HORIZON_T
            goal_align = 0.5 * (1.0 + math.cos(wrap_rad(theta_f - goal_bearing)))
            clear_norm = min(clr, clear_cap) / clear_cap
            speed_norm = min(v, cruise_v) / cruise_v
            turn_norm = abs(w) / max_w
            smooth_norm = abs(w - prev_w) / max_w
            score = (
                W_CLEAR * clear_norm
                + W_GOAL * goal_align
                + W_SPEED * speed_norm
                - W_TURN * turn_norm
                - W_SMOOTH * smooth_norm
            )

            if score > best_score:
                best_score = score
                best_v = v
                best_w = w
                best_clear = clr

    if best_score == -float("inf"):
        reverse = _escape_reverse_cmd(points, goal_bearing, prev_w, pose)
        if reverse is not None:
            return reverse
        if best_escape_score > -float("inf"):
            w = rate_limit(prev_w, best_escape_w, W_RATE)
            clr = command_clearance(best_escape_v, w, points, pose)
            if clr >= escape_min:
                return best_escape_v, w, clr, False
            for rv in sorted([s for s in _speed_candidates() if 0.0 < s <= best_escape_v + 1e-6], reverse=True):
                retry_clear = command_clearance(rv, w, points, pose)
                if retry_clear >= escape_min:
                    return rv, w, retry_clear, False
        escape = _escape_rotate_cmd(points, goal_bearing, prev_w, pose)
        if escape is not None:
            return escape
        return 0.0, 0.0, max(0.0, best_blocked_clear), True

    # 가까울수록 감속하되, 이미 저속 후보를 고른 경우 그 속도보다 빠르게 올리지 않는다.
    span = max(1e-6, SLOW_DIST - COLLISION_DIST)
    v_limit = CRUISE_V * float(np.clip((best_clear - COLLISION_DIST) / span, V_MIN_RATIO, 1.0))
    v = min(best_v, v_limit)
    w = rate_limit(prev_w, best_w, W_RATE)

    # rate limit 때문에 실제로 보낼 w가 후보 w와 달라질 수 있어 최종 명령도 다시 검사한다.
    final_clear = command_clearance(v, w, points, pose)
    if final_clear < COLLISION_DIST:
        for rv in sorted([s for s in _speed_candidates() if s <= v + 1e-6], reverse=True):
            retry_clear = command_clearance(rv, w, points, pose)
            if retry_clear >= COLLISION_DIST:
                return rv, w, retry_clear, False
        for rv in sorted([s for s in _speed_candidates() if 0.0 < s <= min(v, escape_v_max) + 1e-6], reverse=True):
            retry_clear = command_clearance(rv, w, points, pose)
            if retry_clear >= escape_min:
                return rv, w, retry_clear, False
        reverse = _escape_reverse_cmd(points, goal_bearing, prev_w, pose)
        if reverse is not None:
            return reverse
        rotate_clear = command_clearance(0.0, w, points, pose)
        if rotate_clear >= escape_rotate_clearance_min():
            return 0.0, w, rotate_clear, False
        escape = _escape_rotate_cmd(points, goal_bearing, prev_w, pose)
        if escape is not None:
            return escape
        return 0.0, 0.0, final_clear, True

    return v, w, final_clear, False


def sector_clearance(points, bearing, window_rad):
    """특정 방위각 주변의 가장 가까운 라이다 점 거리."""
    if len(points) == 0:
        return MAX_RANGE_M
    ang = np.arctan2(points[:, 1], points[:, 0])
    diff = (ang - bearing + math.pi) % (2.0 * math.pi) - math.pi
    mask = np.abs(diff) <= max(1e-6, window_rad)
    if not mask.any():
        return MAX_RANGE_M
    return float(np.linalg.norm(points[mask], axis=1).min())


def nearest_sector_bearing(points, bearing, window_rad):
    """특정 방위각 주변에서 가장 가까운 라이다 점의 방위각."""
    if len(points) == 0:
        return None
    ang = np.arctan2(points[:, 1], points[:, 0])
    diff = (ang - bearing + math.pi) % (2.0 * math.pi) - math.pi
    mask = np.abs(diff) <= max(1e-6, window_rad)
    if not mask.any():
        return None
    idxs = np.flatnonzero(mask)
    d = np.linalg.norm(points[idxs], axis=1)
    return float(ang[idxs[int(np.argmin(d))]])


def gap_required_half_width():
    """로봇이 gap을 통과하는 데 필요한 좌우 반폭."""
    body_half = 0.5 * float(globals().get("ROBOT_BODY_WIDTH_M", ROBOT_RADIUS * 2.0))
    return max(0.0, body_half + max(0.0, GAP_PASS_SIDE_MARGIN_M))


def gap_corridor_clearance(points, bearing, lookahead=None):
    """후보 bearing으로 고정 길이 통로를 깔았을 때의 좌우 여유.

    반환값이 0보다 작으면 로봇 폭+여유를 가진 직사각형 통로 안에 라이다 점이
    들어온 것이므로, 저속으로는 잠깐 전진 가능해 보여도 실제 gap으로 쓰지 않는다.
    """
    if len(points) == 0:
        return MAX_RANGE_M

    lookahead = GAP_PASS_LOOKAHEAD_M if lookahead is None else lookahead
    lookahead = max(0.10, min(float(lookahead), MAX_RANGE_M))
    half_width = gap_required_half_width()
    half_len = 0.5 * float(globals().get("ROBOT_BODY_LENGTH_M", ROBOT_RADIUS * 2.0))

    c, s = math.cos(bearing), math.sin(bearing)
    forward = points[:, 0] * c + points[:, 1] * s
    lateral = -points[:, 0] * s + points[:, 1] * c

    # 몸체 길이만큼 앞뒤로 살짝 부풀려서 회전 직후 코앞의 좁은 통로도 걸러낸다.
    mask = (forward >= -half_len) & (forward <= lookahead + half_len)
    if not mask.any():
        return MAX_RANGE_M

    side_margin = np.abs(lateral[mask]) - half_width
    return float(side_margin.min())


def _gap_bearing_candidates(target_bearing):
    limit = max(0.05, GAP_BEARING_MAX)
    step = max(0.03, GAP_BEARING_STEP)
    n = max(1, int(math.ceil(limit / step)))
    vals = [0.0, float(np.clip(target_bearing, -limit, limit))]
    for i in range(1, n + 1):
        b = min(limit, i * step)
        vals.extend((b, -b))
    uniq = []
    seen = set()
    for v in vals:
        k = round(float(v), 4)
        if k not in seen:
            seen.add(k)
            uniq.append(float(v))
    return uniq


def choose_gap_cmd(points, target_bearing, prev_w, pose=None):
    """직접 목표가 막힐 때 가장 열린 gap을 임시 목표로 고른다."""
    best = None
    min_sector = COLLISION_DIST + GAP_MIN_CLEAR_MARGIN
    max_turn = max(1e-6, GAP_BEARING_MAX)
    target_sector = sector_clearance(points, target_bearing, GAP_WINDOW_RAD)
    blocker_bearing = None
    if target_sector < SLOW_DIST:
        blocker_bearing = nearest_sector_bearing(points, target_bearing, GAP_WINDOW_RAD)

    for bearing in _gap_bearing_candidates(target_bearing):
        pass_clear = gap_corridor_clearance(points, bearing)
        if pass_clear < 0.0:
            continue

        sector = sector_clearance(points, bearing, GAP_WINDOW_RAD)
        if sector < min_sector:
            continue

        v, w, clr, blocked = choose_cmd(points, bearing, prev_w, pose)
        if blocked:
            continue

        clear_norm = min(clr, max(1e-6, CLEAR_CAP)) / max(1e-6, CLEAR_CAP)
        sector_norm = min(sector, max(1e-6, CLEAR_CAP)) / max(1e-6, CLEAR_CAP)
        pass_norm = min(pass_clear, max(1e-6, CLEAR_CAP)) / max(1e-6, CLEAR_CAP)
        target_align = 0.5 * (1.0 + math.cos(wrap_rad(bearing - target_bearing)))
        turn_norm = abs(bearing) / max_turn
        away_norm = 0.0
        if blocker_bearing is not None:
            away_norm = min(abs(wrap_rad(bearing - blocker_bearing)), max_turn) / max_turn
        score = (
            GAP_W_CLEAR * sector_norm
            + GAP_W_PASS * pass_norm
            + 0.40 * clear_norm
            + GAP_W_TARGET * target_align
            + GAP_W_AWAY * away_norm
            - GAP_W_TURN * turn_norm
        )

        if best is None or score > best["score"]:
            best = {
                "bearing": bearing,
                "v": v,
                "w": w,
                "clearance": clr,
                "sector_clearance": sector,
                "pass_clearance": pass_clear,
                "required_half_width": gap_required_half_width(),
                "lookahead_m": max(0.10, min(GAP_PASS_LOOKAHEAD_M, MAX_RANGE_M)),
                "score": score,
            }
    return best


# ===================== 시퀀스 컨트롤러 =====================
# 한 제어주기의 의사결정 = main() 과 시뮬레이터가 공유하는 단일 진실원.
# 여기(상수/choose_cmd/tick)를 고치면 시뮬레이터 동작도 그대로 바뀐다
# (sim 이 이 코드를 import 해 직접 호출하므로 글루를 다시 짤 필요가 없다).
class Controller:
    """색 시퀀스 추적 + 패치 위로 더 들어간 뒤 정지 + DWA 조향."""

    def __init__(self):
        self.seq_idx = 0           # 현재 추적 색 = TARGET_SEQUENCE[seq_idx]
        self.phase = "INIT_SCAN"   # 시작 시 360° 회전으로 color_memory 선구성
        self.init_scan_prev_theta = None
        self.init_scan_rotated = 0.0
        self.init_scan_start_t = None
        self.close_until = 0.0
        self.creep_until = 0.0
        self.dwell_until = 0.0     # >0 이면 패치 위 정지 유지가 끝나는 시각
        self.last_w = 0.0
        self.goal = 0.0            # 직전 목표 방위 (로깅/시각화용)
        self.clr = 0.0             # 직전 여유거리
        self.last_seen_t = -1e9
        self.last_seen_bearing = 0.0
        self.last_seen_cy = 0.0
        self.smooth_bearing = 0.0
        self.close_peak_cy = 0.0
        self.approach_align_since = -1.0
        self.approach_ready = False
        self.search_dir = 1.0
        self.search_switch_t = 0.0
        self.search_anchor_theta = None
        self.search_switch_t = 0.0
        self.search_mode = "SCAN"
        self.search_edge_hits = 0
        self.search_drive_until = 0.0
        self.search_force_explore = False
        self.explore_waypoints = []
        self.explore_visited = []
        self.explore_deferred = []
        self.explore_target_idx = None
        self.explore_cycle = 0
        self.explore_blocked_since = 0.0
        self.explore_last_progress_t = 0.0
        self.explore_last_progress_pose = None
        self.explore_last_dist = None
        self.explore_anchor_x = None
        self.explore_anchor_y = None
        self.explore_priority_count = 0
        self.avoid_active = False
        self.avoid_goal = 0.0
        self.avoid_direct_clr = 0.0
        self.avoid_target_sector_clear = 0.0
        self.avoid_sector_clear = 0.0
        self.avoid_pass_clear = 0.0
        self.avoid_required_half_width = 0.0
        self.avoid_lookahead = 0.0
        self.avoid_score = 0.0
        self.center_goal_x = None      # centered/world 좌표계의 목표 색상 중심 x
        self.center_goal_y = None
        self.last_target_x = None      # 로봇 좌표계의 최근 색상 중심 x
        self.last_target_y = None
        self.last_center_dist = None
        self.color_memory = {}         # color -> centered/world 좌표계의 최근 관측 위치
        self.rescan_dist_m = 0.0       # 마지막 스캔 이후 누적 이동 거리
        self.rescan_last_pose = None   # 직전 tick pose (거리 적분용)

    @property
    def done(self):
        return self.seq_idx >= len(TARGET_SEQUENCE)

    @property
    def target_color(self):
        return None if self.done else TARGET_SEQUENCE[self.seq_idx]

    def _reset_for_next_target(self):
        self.phase = "POST_SCAN"
        self.init_scan_prev_theta = None
        self.init_scan_rotated = 0.0
        self.init_scan_start_t = None
        self.close_until = 0.0
        self.creep_until = 0.0
        self.dwell_until = 0.0
        self.last_w = 0.0
        self.goal = 0.0
        self.clr = 0.0
        self.last_seen_t = -1e9
        self.last_seen_bearing = 0.0
        self.last_seen_cy = 0.0
        self.smooth_bearing = 0.0
        self.close_peak_cy = 0.0
        self.approach_align_since = -1.0
        self.approach_ready = False
        self.search_dir = 1.0
        self.search_switch_t = 0.0
        self.search_anchor_theta = None
        self.search_mode = "SCAN"
        self.search_edge_hits = 0
        self.search_drive_until = 0.0
        self.search_force_explore = True
        self.rescan_dist_m = 0.0
        self.rescan_last_pose = None
        self._reset_explore()
        self._reset_avoidance()
        self._reset_center()

    def _reset_avoidance(self, direct_clr=0.0, target_sector_clear=0.0):
        self.avoid_active = False
        self.avoid_goal = 0.0
        self.avoid_direct_clr = direct_clr
        self.avoid_target_sector_clear = target_sector_clear
        self.avoid_sector_clear = 0.0
        self.avoid_pass_clear = 0.0
        self.avoid_required_half_width = gap_required_half_width()
        self.avoid_lookahead = max(0.10, min(GAP_PASS_LOOKAHEAD_M, MAX_RANGE_M))
        self.avoid_score = 0.0

    def _reset_center(self):
        self.center_goal_x = None
        self.center_goal_y = None
        self.last_target_x = None
        self.last_target_y = None
        self.last_center_dist = None

    def _memory_valid(self, mem, now):
        return (
            mem is not None and
            now - mem.get("time_s", -1e9) <= COLOR_MEMORY_TTL_S and
            math.isfinite(mem.get("x_m", float("nan"))) and
            math.isfinite(mem.get("y_m", float("nan")))
        )

    def _remember_color_detection(self, color, det, pose, now):
        if color not in TARGET_SEQUENCE or not det or not det.get("visible", False):
            return
        pose_t = _pose_tuple(pose)
        if pose_t is None:
            return
        center = det.get("target_center_robot_frame_m") or {}
        try:
            tx = float(center.get("x_m"))
            ty = float(center.get("y_m"))
        except (TypeError, ValueError):
            return
        if not (math.isfinite(tx) and math.isfinite(ty)):
            return

        px, py, th = pose_t
        cs, sn = math.cos(th), math.sin(th)
        gx = px + tx * cs - ty * sn
        gy = py + tx * sn + ty * cs
        self.color_memory[color] = {
            "x_m": gx,
            "y_m": gy,
            "time_s": now,
            "bearing_rad": det.get("bearing_rad"),
            "area_ratio": det.get("area_ratio"),
            "cy_norm": det.get("cy_norm"),
        }

    def _note_color_detections(self, detections, pose, now):
        if not detections:
            return
        for color, det in detections.items():
            self._remember_color_detection(color, det, pose, now)

    def _wrong_color_waypoint_penalty(self, wx, wy, now):
        penalty = 0.0
        current = self.target_color
        radius = max(1e-6, WRONG_COLOR_AVOID_RADIUS_M)
        for color, mem in self.color_memory.items():
            if color == current or not self._memory_valid(mem, now):
                continue
            d = math.hypot(wx - mem["x_m"], wy - mem["y_m"])
            if d < radius:
                penalty += WRONG_COLOR_SCORE_PENALTY * (1.0 - d / radius)
        return penalty

    def _target_memory(self, now):
        color = self.target_color
        if color is None:
            return None
        mem = self.color_memory.get(color)
        return mem if self._memory_valid(mem, now) else None

    def _reset_explore(self):
        self.explore_waypoints = []
        self.explore_visited = []
        self.explore_deferred = []
        self.explore_target_idx = None
        self.explore_cycle = 0
        self.explore_blocked_since = 0.0
        self.explore_last_progress_t = 0.0
        self.explore_last_progress_pose = None
        self.explore_last_dist = None
        self.explore_anchor_x = None
        self.explore_anchor_y = None
        self.explore_priority_count = 0

    def _note_seen(self, bearing, cy_norm, now):
        if self.last_seen_t < -1e8:
            self.smooth_bearing = bearing
        else:
            a = max(0.0, min(1.0, BEARING_SMOOTH_ALPHA))
            self.smooth_bearing = (1.0 - a) * self.smooth_bearing + a * bearing
        self.last_seen_t = now
        self.last_seen_bearing = self.smooth_bearing
        self.last_seen_cy = cy_norm
        if self.phase == "CLOSE":
            self.close_peak_cy = max(self.close_peak_cy, cy_norm)
        if abs(self.last_seen_bearing) > 0.05:
            self.search_dir = 1.0 if self.last_seen_bearing > 0.0 else -1.0
        self.search_anchor_theta = None
        self.search_mode = "SCAN"
        self.search_edge_hits = 0
        self.search_drive_until = 0.0
        self.search_force_explore = False
        self.rescan_dist_m = 0.0
        self.rescan_last_pose = None

    def _update_approach_quality(self, cy_norm, now):
        aligned = cy_norm >= ARRIVE_CY and abs(self.smooth_bearing) <= APPROACH_ALIGN_MAX
        if aligned:
            if self.approach_align_since < 0.0:
                self.approach_align_since = now
            if now - self.approach_align_since >= APPROACH_STABLE_S:
                self.approach_ready = True
        else:
            self.approach_align_since = -1.0
            self.approach_ready = False
        return self.approach_ready

    def _begin_creep(self, now):
        self.phase = "CREEP"
        self.creep_until = now + blind_creep_duration()

    def _begin_hold(self, now):
        self.phase = "HOLD"
        self.dwell_until = now + DWELL_S
        self.last_w = 0.0
        self.clr = 0.0
        self.close_peak_cy = 0.0
        self._reset_avoidance()
        self._begin_search_scan()
        return 0.0, 0.0, "ARRIVE"

    def _init_scan_cmd(self, points, now, pose):
        """제자리 1회전. INIT_SCAN / POST_SCAN / RE_SCAN 세 페이즈 모두 처리."""
        pose_t = _pose_tuple(pose)
        phase_label = self.phase
        if pose_t is not None:
            _, _, theta = pose_t
            if self.init_scan_prev_theta is None:
                self.init_scan_prev_theta = theta
                self.init_scan_rotated = 0.0
            else:
                self.init_scan_rotated += abs(wrap_rad(theta - self.init_scan_prev_theta))
                self.init_scan_prev_theta = theta
            if self.init_scan_rotated >= 2.0 * math.pi - 0.15:
                return self._finish_spin_scan(points, now, pose)
        else:
            if self.init_scan_start_t is None:
                self.init_scan_start_t = now
            if now - self.init_scan_start_t >= 2.0 * math.pi / max(1e-6, SEARCH_W) + 1.0:
                return self._finish_spin_scan(points, now, pose)

        target_w = SEARCH_W
        if command_clearance(0.0, target_w, points, pose) < escape_rotate_clearance_min():
            target_w = -SEARCH_W
        if command_clearance(0.0, target_w, points, pose) < escape_rotate_clearance_min():
            self.last_w = 0.0
            return 0.0, 0.0, phase_label
        w = rate_limit(self.last_w, target_w, W_RATE)
        self.last_w = w
        return 0.0, w, phase_label

    def _finish_spin_scan(self, points, now, pose):
        """360° 스캔 완료. 기억 위치 있으면 SEEK, 없으면 EXPLORE로 아레나 커버리지."""
        self.last_w = 0.0
        self.phase = "SEEK"
        self.rescan_dist_m = 0.0
        self.rescan_last_pose = None

        if self._target_memory(now) is not None:
            return 0.0, 0.0, "SEEK"    # 다음 tick에서 MEMORY_GOTO가 안내

        # 타깃 미발견 → EXPLORE_GOTO로 아레나를 체계적으로 커버하며 탐색.
        # (단순 직진은 장애물에 막히거나 타깃 반대 방향으로 갈 수 있음)
        return self._begin_and_try_search_drive(points, now, pose)

    def _begin_search_scan(self):
        self._reset_avoidance()
        self.search_mode = "SCAN"
        self.search_anchor_theta = None
        self.search_switch_t = 0.0
        self.search_edge_hits = 0
        self.search_drive_until = 0.0

    def _begin_search_drive(self, now):
        self.search_mode = "DRIVE"
        self.search_anchor_theta = None
        self.search_switch_t = 0.0
        self.search_edge_hits = 0
        self.search_drive_until = now + SEARCH_DRIVE_S

    def _search_drive_cmd(self, points, pose=None):
        self._reset_avoidance()
        self.goal = 0.0
        v, w, clr, blocked = choose_cmd(points, self.goal, self.last_w, pose)
        self.clr = clr
        if blocked:
            self.last_w = 0.0
            return 0.0, 0.0, "BLOCKED"
        v = min(v, SEARCH_DRIVE_V)
        self.last_w = w
        return v, w, "SEARCH_DRIVE"

    def _explore_half_span(self):
        if KEEPIN_ENABLED:
            half = KEEPIN_SIZE_M * 0.5 - KEEPIN_MARGIN_M - EXPLORE_EDGE_MARGIN_M
        else:
            half = 1.0
        return max(EXPLORE_REACH_DIST_M * 2.0, half)

    def _build_explore_waypoints(self):
        """centered odometry 좌표계에서 keep-in 내부를 훑는 lawnmower waypoint."""
        half = self._explore_half_span()
        spacing = max(0.15, EXPLORE_SPACING_M)
        coarse = [
            (0.0, -half), (half, -half), (half, 0.0), (half, half),
            (0.0, half), (-half, half), (-half, 0.0), (-half, -half),
            (0.0, 0.0),
        ]
        n = max(2, int(math.floor((2.0 * half) / spacing)) + 1)
        coords = np.linspace(-half, half, n)
        waypoints = [(float(x), float(y)) for x, y in coarse]
        self.explore_priority_count = len(waypoints)
        for row_i, y in enumerate(coords):
            xs = coords if row_i % 2 == 0 else coords[::-1]
            for x in xs:
                pt = (float(x), float(y))
                if any(math.hypot(pt[0] - qx, pt[1] - qy) < 0.05 for qx, qy in waypoints):
                    continue
                waypoints.append(pt)
        return waypoints

    def _ensure_explore_plan(self):
        if not self.explore_waypoints:
            self.explore_waypoints = self._build_explore_waypoints()
            self.explore_visited = [False] * len(self.explore_waypoints)
            self.explore_deferred = [0] * len(self.explore_waypoints)
            self.explore_target_idx = None
        elif (len(self.explore_visited) != len(self.explore_waypoints) or
              len(self.explore_deferred) != len(self.explore_waypoints)):
            self.explore_visited = [False] * len(self.explore_waypoints)
            self.explore_deferred = [0] * len(self.explore_waypoints)
            self.explore_target_idx = None
        return bool(self.explore_waypoints)

    def _select_next_explore_waypoint(self, pose_t, now, skip_current=False, prefer_far=False):
        if not self._ensure_explore_plan():
            return False
        if skip_current and self.explore_target_idx is not None:
            if prefer_far:
                self.explore_deferred[self.explore_target_idx] = max(
                    self.explore_deferred[self.explore_target_idx], 4)
            else:
                self.explore_visited[self.explore_target_idx] = True

        self.explore_deferred = [max(0, d - 1) for d in self.explore_deferred]

        if all(self.explore_visited):
            self.explore_visited = [False] * len(self.explore_waypoints)
            self.explore_deferred = [0] * len(self.explore_waypoints)
            self.explore_cycle += 1

        px, py, _ = pose_t
        if self.explore_anchor_x is None or self.explore_anchor_y is None:
            self.explore_anchor_x = px
            self.explore_anchor_y = py
        priority_stop = min(self.explore_priority_count, len(self.explore_waypoints))
        priority_candidates = [
            i for i in range(priority_stop)
            if not self.explore_visited[i] and self.explore_deferred[i] <= 0
        ]
        candidate_indices = priority_candidates or [
            i for i, visited in enumerate(self.explore_visited)
            if not visited and self.explore_deferred[i] <= 0
        ]
        if not candidate_indices:
            self.explore_deferred = [0] * len(self.explore_waypoints)
            candidate_indices = [
                i for i, visited in enumerate(self.explore_visited)
                if not visited
            ]

        best_i = None
        best_score = float("inf")
        for i in candidate_indices:
            wx, wy = self.explore_waypoints[i]
            dist = math.hypot(wx - px, wy - py)
            anchor_dist = math.hypot(wx - self.explore_anchor_x, wy - self.explore_anchor_y)
            wrong_penalty = self._wrong_color_waypoint_penalty(wx, wy, now)
            if prefer_far:
                score = -dist + 0.10 * anchor_dist + 1.5 * wrong_penalty
            else:
                score = (
                    anchor_dist
                    + 0.20 * dist
                    + wrong_penalty
                    + 0.02 * (i / max(1, len(self.explore_waypoints) - 1))
                )
            if score < best_score:
                best_score = score
                best_i = i

        if best_i is None:
            return False
        self.explore_target_idx = best_i
        self.search_mode = "EXPLORE_GOTO"
        self.explore_blocked_since = 0.0
        self.explore_last_progress_t = now
        self.explore_last_progress_pose = (px, py)
        wx, wy = self.explore_waypoints[best_i]
        self.explore_last_dist = math.hypot(wx - px, wy - py)
        return True

    def _begin_explore_goto(self, now, pose=None):
        pose_t = _pose_tuple(pose)
        if (not EXPLORE_ENABLED) or pose_t is None:
            self._begin_search_drive(now)
            return False
        return self._select_next_explore_waypoint(pose_t, now)

    def _explore_target_error_robot(self, pose_t):
        if self.explore_target_idx is None:
            return None
        try:
            wx, wy = self.explore_waypoints[self.explore_target_idx]
        except (IndexError, TypeError):
            return None
        px, py, th = pose_t
        dx = wx - px
        dy = wy - py
        cs, sn = math.cos(th), math.sin(th)
        return (
            dx * cs + dy * sn,
            -dx * sn + dy * cs,
            math.hypot(dx, dy),
            wx,
            wy,
        )

    def _finish_explore_waypoint(self):
        if self.explore_target_idx is not None and self.explore_visited:
            if 0 <= self.explore_target_idx < len(self.explore_visited):
                self.explore_visited[self.explore_target_idx] = True
                if self.explore_deferred:
                    self.explore_deferred[self.explore_target_idx] = 0
        self.explore_target_idx = None
        self.explore_blocked_since = 0.0
        self.explore_last_dist = None

    def _explore_stuck(self, pose_t, dist, now):
        if self.explore_last_dist is None or dist < self.explore_last_dist - EXPLORE_STUCK_DIST_M:
            self.explore_last_dist = dist
            self.explore_last_progress_t = now
            self.explore_last_progress_pose = (pose_t[0], pose_t[1])
            return False
        return now - self.explore_last_progress_t >= EXPLORE_STUCK_S

    def _explore_cmd(self, points, now, pose=None):
        pose_t = _pose_tuple(pose)
        if pose_t is None:
            if self.search_mode == "DRIVE" and now < self.search_drive_until:
                return self._search_drive_cmd(points, pose)
            self._begin_search_drive(now)
            return self._search_drive_cmd(points, pose)

        if self.explore_target_idx is None:
            if not self._select_next_explore_waypoint(pose_t, now):
                return self._search_cmd(points, now, pose)

        err = self._explore_target_error_robot(pose_t)
        if err is None:
            if not self._select_next_explore_waypoint(pose_t, now):
                return self._search_cmd(points, now, pose)
            err = self._explore_target_error_robot(pose_t)
            if err is None:
                return self._search_cmd(points, now, pose)

        ex, ey, dist, _, _ = err
        if dist <= EXPLORE_REACH_DIST_M:
            self._finish_explore_waypoint()
            if EXPLORE_SCAN_EDGE_HITS > 0:
                self._begin_search_scan()
                return self._search_cmd(points, now, pose)
            if self._select_next_explore_waypoint(pose_t, now):
                return self._explore_cmd(points, now, pose)
            self._begin_search_scan()
            return self._search_cmd(points, now, pose)

        if self._explore_stuck(pose_t, dist, now):
            self._select_next_explore_waypoint(pose_t, now, skip_current=True, prefer_far=True)
            return self._explore_cmd(points, now, pose)

        goal_bearing = math.atan2(ey, ex)
        if abs(goal_bearing) > EXPLORE_TURN_IN_PLACE_RAD:
            target_w = max(-SEARCH_W, min(SEARCH_W, 1.6 * goal_bearing))
            clr = command_clearance(0.0, target_w, points, pose)
            self.goal = goal_bearing
            self.clr = clr
            if clr >= escape_rotate_clearance_min():
                self._reset_avoidance()
                w = rate_limit(self.last_w, target_w, W_RATE)
                self.last_w = w
                return 0.0, w, "EXPLORE_TURN"

        v, w, state = self._goal_cmd(points, goal_bearing, pose, allow_avoid=True)
        if state == "BLOCKED":
            if self.explore_blocked_since <= 0.0:
                self.explore_blocked_since = now
            if now - self.explore_blocked_since >= EXPLORE_BLOCKED_S:
                self._select_next_explore_waypoint(pose_t, now, skip_current=True, prefer_far=True)
                return 0.0, 0.0, "EXPLORE_BLOCKED"
            # 타임아웃 전: 서 있지 않고 탈출 회전으로 다른 경로 모색
            escape = _escape_rotate_cmd(points, goal_bearing, self.last_w, pose)
            if escape is not None:
                v_e, w_e, clr_e, _ = escape
                self.last_w = w_e
                self.clr = clr_e
                return v_e, w_e, "EXPLORE_BLOCKED"
            return 0.0, 0.0, "EXPLORE_BLOCKED"

        self.explore_blocked_since = 0.0
        v_scale = float(np.clip(dist / max(1e-6, EXPLORE_SLOW_DIST_M), 0.45, 1.0))
        v = min(v, EXPLORE_MAX_V * v_scale)
        if state == "AVOID":
            return v, w, "EXPLORE_AVOID"
        return v, w, "EXPLORE_GOTO"

    def _target_memory_cmd(self, points, now, pose=None):
        mem = self._target_memory(now)
        pose_t = _pose_tuple(pose)
        if mem is None or pose_t is None:
            return None

        px, py, th = pose_t
        dx = mem["x_m"] - px
        dy = mem["y_m"] - py
        dist = math.hypot(dx, dy)
        if dist <= TARGET_MEMORY_REACHED_DIST_M:
            # 기억 위치까지 왔는데도 안 보이면 잘못된/가려진 기억으로 보고 일반 탐색으로 복귀.
            self.color_memory.pop(self.target_color, None)
            self._begin_search_scan()
            return self._search_cmd(points, now, pose)

        cs, sn = math.cos(th), math.sin(th)
        ex = dx * cs + dy * sn
        ey = -dx * sn + dy * cs
        goal_bearing = math.atan2(ey, ex)
        v, w, state = self._goal_cmd(points, goal_bearing, pose, allow_avoid=True)
        if state == "BLOCKED":
            return 0.0, 0.0, "MEMORY_BLOCKED"
        v = min(v, TARGET_MEMORY_MAX_V)
        if state == "AVOID":
            return v, w, "MEMORY_AVOID"
        return v, w, "MEMORY_GOTO"

    def _goal_cmd(self, points, goal_bearing, pose=None, allow_avoid=True):
        """목표 bearing으로 DWA를 시도하고, 막힌 경우 열린 gap을 임시 목표로 쓴다."""
        self.goal = goal_bearing
        v, w, clr, blocked = choose_cmd(points, goal_bearing, self.last_w, pose)
        target_sector_clear = sector_clearance(points, goal_bearing, GAP_WINDOW_RAD)
        self.avoid_direct_clr = clr
        self.avoid_target_sector_clear = target_sector_clear

        if not allow_avoid:
            self._reset_avoidance(clr, target_sector_clear)
            self.last_w = w
            self.clr = clr
            return v, w, ("BLOCKED" if blocked else "SEEK")

        release_clear = COLLISION_DIST + AVOID_RELEASE_MARGIN
        trigger_clear = COLLISION_DIST + AVOID_TRIGGER_MARGIN
        if self.avoid_active and (not blocked) and clr >= release_clear:
            self._reset_avoidance(clr, target_sector_clear)
            self.last_w = w
            self.clr = clr
            return v, w, "SEEK"

        need_gap = (
            blocked
            or self.avoid_active
            or clr < trigger_clear
            or target_sector_clear < SLOW_DIST
        )
        if need_gap:
            gap = choose_gap_cmd(points, goal_bearing, self.last_w, pose)
            if gap is not None:
                self.avoid_active = True
                self.avoid_goal = gap["bearing"]
                self.avoid_target_sector_clear = target_sector_clear
                self.avoid_sector_clear = gap["sector_clearance"]
                self.avoid_pass_clear = gap["pass_clearance"]
                self.avoid_required_half_width = gap["required_half_width"]
                self.avoid_lookahead = gap["lookahead_m"]
                self.avoid_score = gap["score"]
                self.goal = gap["bearing"]
                self.last_w = gap["w"]
                self.clr = gap["clearance"]
                return gap["v"], gap["w"], "AVOID"

        if blocked:
            self._reset_avoidance(clr, target_sector_clear)
            self.last_w = 0.0
            self.clr = clr
            return 0.0, 0.0, "BLOCKED"

        self._reset_avoidance(clr, target_sector_clear)
        self.last_w = w
        self.clr = clr
        return v, w, "SEEK"

    def _update_center_goal(self, target_xy, pose):
        if target_xy is None or pose is None:
            return False
        pose_t = _pose_tuple(pose)
        if pose_t is None:
            return False
        try:
            tx, ty = float(target_xy[0]), float(target_xy[1])
        except (TypeError, ValueError, IndexError):
            return False
        if not (math.isfinite(tx) and math.isfinite(ty)):
            return False

        px, py, th = pose_t
        cs, sn = math.cos(th), math.sin(th)
        gx = px + tx * cs - ty * sn
        gy = py + tx * sn + ty * cs
        if self.center_goal_x is None or self.center_goal_y is None:
            self.center_goal_x = gx
            self.center_goal_y = gy
        else:
            a = max(0.0, min(1.0, CENTER_GOAL_ALPHA))
            self.center_goal_x = (1.0 - a) * self.center_goal_x + a * gx
            self.center_goal_y = (1.0 - a) * self.center_goal_y + a * gy
        self.last_target_x = tx
        self.last_target_y = ty
        return True

    def _center_error_robot(self, pose):
        pose_t = _pose_tuple(pose)
        if pose_t is None or self.center_goal_x is None or self.center_goal_y is None:
            return None
        px, py, th = pose_t
        dx = self.center_goal_x - px
        dy = self.center_goal_y - py
        cs, sn = math.cos(th), math.sin(th)
        return (
            dx * cs + dy * sn,
            -dx * sn + dy * cs,
        )

    def _center_cmd(self, points, target_xy, now, pose=None, visible=True):
        if visible and self._update_center_goal(target_xy, pose):
            self.phase = "CENTER"
        elif self.center_goal_x is None or self.center_goal_y is None:
            return self._search_cmd(points, now, pose)
        else:
            self.phase = "CENTER_BLIND"

        err = self._center_error_robot(pose)
        if err is None:
            return self._search_cmd(points, now, pose)

        ex, ey = err
        dist = math.hypot(ex, ey)
        self.last_center_dist = dist
        self.last_target_x = ex
        self.last_target_y = ey

        if dist <= CENTER_TOL_M:
            return self._begin_hold(now)

        # 최종 목표가 로봇 중심이므로 가까운 구간에서는 속도를 줄인다.
        goal_bearing = math.atan2(ey, max(0.03, ex))
        goal_bearing = max(-CENTER_MAX_BEARING, min(CENTER_MAX_BEARING, goal_bearing))
        v, w, state = self._goal_cmd(points, goal_bearing, pose, allow_avoid=True)
        v_scale = float(np.clip(dist / max(1e-6, CENTER_SLOW_RADIUS_M), 0.25, 1.0))
        v = min(v, CENTER_MAX_V * v_scale)
        if ex < CENTER_TOL_M * 0.5 and abs(ey) > CENTER_TOL_M:
            v = min(v, CENTER_MAX_V * 0.35)
        if state == "BLOCKED":
            return 0.0, 0.0, "BLOCKED"
        return v, w, ("CENTER" if visible and state == "SEEK" else "CENTER_BLIND" if state == "SEEK" else state)

    def _begin_and_try_search_drive(self, points, now, pose=None):
        if self._begin_explore_goto(now, pose):
            return self._explore_cmd(points, now, pose)
        self._begin_search_drive(now)
        v, w, state = self._search_drive_cmd(points, pose)
        if state == "BLOCKED":
            self._begin_search_scan()
        return v, w, state

    def _creep_cmd(self, points, v, state, pose=None):
        steer = CREEP_STEER_GAIN * self.last_seen_bearing
        steer = max(-CREEP_MAX_W, min(CREEP_MAX_W, steer))
        self.goal = self.last_seen_bearing
        self.clr = command_clearance(max(v, 0.05), steer, points, pose)
        if self.clr < COLLISION_DIST and abs(steer) > 1e-6:
            straight_clr = command_clearance(max(v, 0.05), 0.0, points, pose)
            if straight_clr >= COLLISION_DIST:
                steer = 0.0
                self.clr = straight_clr
        if self.clr < COLLISION_DIST:
            self.last_w = 0.0
            return 0.0, 0.0, "BLOCKED"
        w = rate_limit(self.last_w, steer, W_RATE)
        self.last_w = w
        return v, w, state

    def _search_cmd(self, points, now, pose=None):
        if self.search_force_explore and _pose_tuple(pose) is not None:
            self.search_force_explore = False
            return self._begin_and_try_search_drive(points, now, pose)

        if self.search_mode == "DRIVE":
            if now < self.search_drive_until:
                v, w, state = self._search_drive_cmd(points, pose)
                if state != "BLOCKED":
                    return v, w, state
            self._begin_search_scan()
        elif self.search_mode == "EXPLORE_GOTO":
            return self._explore_cmd(points, now, pose)

        pose_t = _pose_tuple(pose)
        target_w = SEARCH_W * self.search_dir
        edge_limit = EXPLORE_SCAN_EDGE_HITS if self.explore_waypoints else SEARCH_SCAN_EDGE_HITS

        if pose_t is not None:
            theta = pose_t[2]
            if self.search_anchor_theta is None:
                self.search_anchor_theta = theta
                if now - self.last_seen_t <= TARGET_LOST_MEMORY_S and abs(self.last_seen_bearing) > 0.05:
                    self.search_dir = 1.0 if self.last_seen_bearing > 0.0 else -1.0

            target_theta = self.search_anchor_theta + self.search_dir * SEARCH_SWEEP_ANGLE_RAD
            err = wrap_rad(target_theta - theta)
            if abs(err) <= SEARCH_SETTLE_RAD:
                self.search_dir *= -1.0
                self.search_edge_hits += 1
                if self.search_edge_hits >= edge_limit:
                    return self._begin_and_try_search_drive(points, now, pose)
                target_theta = self.search_anchor_theta + self.search_dir * SEARCH_SWEEP_ANGLE_RAD
                err = wrap_rad(target_theta - theta)

            self.goal = err
            target_w = max(-SEARCH_W, min(SEARCH_W, 2.2 * err))
            min_w = min(SEARCH_W, max(0.20, 0.35 * SEARCH_W))
            if abs(target_w) < min_w:
                target_w = min_w * (1.0 if err >= 0.0 else -1.0)
        else:
            if self.search_switch_t <= 0.0:
                self.search_switch_t = now + SEARCH_SWEEP_S
            elif now >= self.search_switch_t:
                self.search_dir *= -1.0
                self.search_edge_hits += 1
                if self.search_edge_hits >= edge_limit:
                    return self._begin_and_try_search_drive(points, now, pose)
                self.search_switch_t = now + SEARCH_SWEEP_S
            self.goal = SEARCH_SWEEP_ANGLE_RAD * self.search_dir
            target_w = SEARCH_W * self.search_dir

        self.clr = command_clearance(0.0, target_w, points, pose)
        if self.clr < escape_rotate_clearance_min():
            self.last_w = 0.0
            return 0.0, 0.0, "BLOCKED"

        w = rate_limit(self.last_w, target_w, W_RATE)
        self.last_w = w
        return 0.0, w, "SEARCH"

    def tick(self, points, seen, bearing, cy_norm, now, pose=None, target_xy=None,
             color_detections=None):
        """한 주기 의사결정. 반환 (v, w, state).
           state ∈ DONE/INIT_SCAN/HOLD/ARRIVE/CENTER/CENTER_BLIND/CREEP/CLOSE/SEARCH/
                    SEARCH_DRIVE/EXPLORE_GOTO/EXPLORE_TURN/EXPLORE_AVOID/
                    EXPLORE_BLOCKED/MEMORY_GOTO/MEMORY_AVOID/AVOID/BLOCKED/SEEK.
           색 전환·정지 타이머는 내부 갱신."""
        self.goal = 0.0
        self.clr = 0.0
        if self.done:                              # 완주 → 정지 유지
            self.last_w = 0.0
            return 0.0, 0.0, "DONE"

        self._note_color_detections(color_detections, pose, now)

        if seen:
            self._note_seen(bearing, cy_norm, now)

        if self.phase in ("INIT_SCAN", "POST_SCAN", "RE_SCAN"):   # 제자리 스캔: 타깃이 보이면 즉시 접근
            if seen and target_xy is not None:
                self.phase = "SEEK"
            else:
                return self._init_scan_cmd(points, now, pose)

        if self.phase == "HOLD":                   # 패치 위 정지 유지
            self.last_w = 0.0
            if now >= self.dwell_until:
                self.seq_idx += 1                  # 다음 색 (넘치면 done)
                self._reset_for_next_target()
            return 0.0, 0.0, "HOLD"

        if seen and target_xy is not None:
            return self._center_cmd(points, target_xy, now, pose, visible=True)

        if (not seen and self.phase in ("CENTER", "CENTER_BLIND") and
                self.center_goal_x is not None and self.center_goal_y is not None and
                now - self.last_seen_t <= CENTER_LOST_MEMORY_S):
            return self._center_cmd(points, None, now, pose, visible=False)

        if self.phase in ("CENTER", "CENTER_BLIND") and now - self.last_seen_t > CENTER_LOST_MEMORY_S:
            self._reset_center()
            self.phase = "SEEK"

        if self.phase == "CREEP":                  # 카메라가 잃은 뒤 짧게 더 전진
            if now < self.creep_until:
                return self._creep_cmd(points, BLIND_CREEP_V, "CREEP", pose)
            return self._begin_hold(now)

        if self.phase == "CLOSE":                  # 코앞 감지 후 바로 멈추지 않고 진입
            if now >= self.close_until:
                if self.close_peak_cy >= CLOSE_LOST_HOLD_CY:
                    return self._begin_hold(now)
                if self.approach_ready:
                    self._begin_creep(now)
                    return self._creep_cmd(points, BLIND_CREEP_V, "CREEP", pose)
                self.close_until = now + CLOSE_APPROACH_MAX_S
                self.approach_align_since = -1.0
                self.approach_ready = False
            if seen:
                self.close_peak_cy = max(self.close_peak_cy, cy_norm)
                self._update_approach_quality(cy_norm, now)
                self.goal = self.smooth_bearing
                target_sector = sector_clearance(points, self.goal, GAP_WINDOW_RAD)
                if self.close_peak_cy < CLOSE_LOST_HOLD_CY and target_sector < SLOW_DIST:
                    self.phase = "SEEK"
                    self.approach_align_since = -1.0
                    self.approach_ready = False
                    return self._goal_cmd(points, self.goal, pose, allow_avoid=True)
                self._reset_avoidance()
                v, w, clr, blocked = choose_cmd(points, self.goal, self.last_w, pose)
                self.last_w = w
                self.clr = clr
                if blocked:
                    return 0.0, 0.0, "BLOCKED"
                return min(v, CLOSE_APPROACH_V), w, "CLOSE"
            if self.close_peak_cy >= CLOSE_LOST_HOLD_CY or self.last_seen_cy >= CLOSE_LOST_HOLD_CY:
                return self._begin_hold(now)
            if self.approach_ready:
                self._begin_creep(now)
                return self._creep_cmd(points, BLIND_CREEP_V, "CREEP", pose)
            self.phase = "SEEK"
            self.close_peak_cy = 0.0
            self.approach_align_since = -1.0
            self.approach_ready = False
            self._reset_avoidance()
            return self._search_cmd(points, now, pose)

        if seen and cy_norm >= APPROACH_CY:
            target_sector = sector_clearance(points, self.smooth_bearing, GAP_WINDOW_RAD)
            if cy_norm < CLOSE_LOST_HOLD_CY and target_sector < SLOW_DIST:
                return self._goal_cmd(points, self.smooth_bearing, pose, allow_avoid=True)
            self.phase = "CLOSE"
            self.close_until = now + CLOSE_APPROACH_MAX_S
            self.close_peak_cy = max(self.close_peak_cy, cy_norm)
            self._update_approach_quality(cy_norm, now)
            self.goal = self.smooth_bearing
            self._reset_avoidance()
            v, w, clr, blocked = choose_cmd(points, self.goal, self.last_w, pose)
            self.last_w = w
            self.clr = clr
            if blocked:
                return 0.0, 0.0, "BLOCKED"
            return min(v, CLOSE_APPROACH_V), w, "CLOSE"

        if not seen:
            # odometry 기반 이동 거리 누적
            pose_t = _pose_tuple(pose)
            if pose_t is not None:
                if self.rescan_last_pose is not None:
                    self.rescan_dist_m += math.hypot(
                        pose_t[0] - self.rescan_last_pose[0],
                        pose_t[1] - self.rescan_last_pose[1],
                    )
                self.rescan_last_pose = (pose_t[0], pose_t[1])
            # RE_SCAN_DIST_M 이상 이동 후 타깃 미검출 → 제자리 재스캔
            if self.phase == "SEEK" and self.rescan_dist_m >= RE_SCAN_DIST_M:
                self.phase = "RE_SCAN"
                self.init_scan_prev_theta = None
                self.init_scan_rotated = 0.0
                self.init_scan_start_t = None
                self.rescan_dist_m = 0.0
                self.rescan_last_pose = None
                return self._init_scan_cmd(points, now, pose)
            if self.avoid_active and now - self.last_seen_t <= AVOID_LOST_MEMORY_S:
                keep_goal = self.avoid_goal if abs(self.avoid_goal) > 0.03 else self.last_seen_bearing
                v, w, state = self._goal_cmd(points, keep_goal, pose, allow_avoid=True)
                return v, w, ("AVOID" if state != "BLOCKED" else "BLOCKED")
            mem_cmd = self._target_memory_cmd(points, now, pose)
            if mem_cmd is not None:
                return mem_cmd
            self._reset_avoidance()
            return self._search_cmd(points, now, pose)

        return self._goal_cmd(points, self.smooth_bearing, pose, allow_avoid=True)


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
            cam_res, cam_stamp = cam.get()
            if len(cam_res) >= 7:
                visible, bearing, area_ratio, cy_norm, ncnt, target_x, target_y = cam_res[:7]
                target_xy = (target_x, target_y) if target_x is not None and target_y is not None else None
            else:
                visible, bearing, area_ratio, cy_norm, ncnt = cam_res
                target_xy = None
            color_detections = cam_res[7] if len(cam_res) >= 8 and isinstance(cam_res[7], dict) else {}
            vision_fresh = (now - cam_stamp) <= VISION_HOLD_S
            seen = visible and vision_fresh
            if not vision_fresh:
                color_detections = {}

            v, w, state = ctrl.tick(points, seen, bearing, cy_norm, now, pose=odom.pose,
                                    target_xy=target_xy, color_detections=color_detections)
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
