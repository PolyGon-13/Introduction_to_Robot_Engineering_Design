#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[실습] 장애물 회피 — 라즈베리파이 단일 스크립트 (올인원)

과제 요약:
  - Slamtec RPLidar C1으로 장애물 인식, 자율 주행으로 골까지.
  - 시작 후 추가 입력 없음. 60초 이내 도달 목표.
  - 아두이노: /dev/ttyS0, 9600,  V선속도,각속도\\n  /  정지 S\\n
  - 골: 오도메트리 목표 (3.0, 0), 거리 0.15m 이내에서 S\\n 후 종료.
  - 출발선→골 전방 3.0m, LiDAR 0° = 로봇 전방.

의존성: pip install pyserial

실행:
  python3 robot_complete.py --arduino /dev/ttyS0 --lidar /dev/ttyUSB0

시작: 과제 시간 제한을 위해 기본은 출발 지연 없음(--start-delay 0).
종료: 터미널에서 S 입력 후 Enter 시 아두이노에 S\\n 전송 후 종료 (Ctrl+C도 가능).
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import serial

# ---------------------------------------------------------------------------
# 파라미터 / 유틸
# ---------------------------------------------------------------------------

Point2 = Tuple[float, float]
ScanPoint = Tuple[int, float, float]  # quality, angle_deg, distance_m


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class Params:
    GOAL_X_M: float = 3.0
    GOAL_Y_M: float = 0.0
    GOAL_TOL_M: float = 0.15

    ROBOT_RADIUS_M: float = 0.16
    COLLISION_DIST_M: float = 0.21
    ANGLE_OFFSET_DEG: float = 0.0
    LIDAR_MIN_M: float = 0.05
    LIDAR_MAX_M: float = 2.5
    MIN_X_FOR_PLANNING_M: float = 0.10

    arduino_port: str = "/dev/ttyS0"
    arduino_baud: int = 9600
    lidar_port: str = "/dev/ttyUSB0"
    lidar_baud: int = 460800

    hz: float = 25.0
    mission_timeout_s: float = 60.0

    lidar_min_quality: int = 4

    v_max_mps: float = 0.20
    v_cruise_mps: float = 0.16
    w_max_rps: float = 0.55
    w_goal_gain: float = 1.1
    w_avoid_gain: float = 0.9

    max_dw_dt: float = 2.5

    d_stop_m: float = 0.26
    d_slow_m: float = 0.55
    d_repel_m: float = 0.95

    slow_near_goal_radius_m: float = 0.45
    v_near_goal_max_mps: float = 0.10


def scan_to_xy(scan: List[ScanPoint], p: Params) -> List[Point2]:
    off = math.radians(p.ANGLE_OFFSET_DEG)
    pts: List[Point2] = []
    for q, ang_deg, dist_m in scan:
        if q < p.lidar_min_quality:
            continue
        if dist_m < p.LIDAR_MIN_M or dist_m > p.LIDAR_MAX_M:
            continue
        a = math.radians(ang_deg) + off
        x = dist_m * math.cos(a)
        y = dist_m * math.sin(a)
        if x < p.MIN_X_FOR_PLANNING_M - 0.02:
            continue
        pts.append((float(x), float(y)))
    return pts


def goal_distance_m(rx: float, ry: float, p: Params) -> float:
    return math.hypot(p.GOAL_X_M - rx, p.GOAL_Y_M - ry)


def goal_heading_error(rx: float, ry: float, theta: float, p: Params) -> float:
    dx = p.GOAL_X_M - rx
    dy = p.GOAL_Y_M - ry
    if math.hypot(dx, dy) < 1e-6:
        return 0.0
    return wrap_pi(math.atan2(dy, dx) - theta)


class RateLimiterW:
    def __init__(self, max_dw_dt: float) -> None:
        self.max_dw_dt = float(max_dw_dt)
        self.w = 0.0

    def step(self, w_cmd: float, dt: float) -> float:
        dt = max(dt, 1e-3)
        dw = w_cmd - self.w
        lim = self.max_dw_dt * dt
        dw = clamp(dw, -lim, lim)
        self.w += dw
        return self.w


class ObstacleAvoidController:
    def __init__(self, p: Params) -> None:
        self.p = p

    def compute(
        self,
        scan_xy: List[Point2],
        heading_err: float,
        dist_goal: float,
    ) -> Tuple[float, float]:
        p = self.p
        gx = math.cos(heading_err)
        gy = math.sin(heading_err)

        rep_x = rep_y = 0.0
        min_front = 99.0
        min_any = 99.0

        for x, y in scan_xy:
            d = math.hypot(x, y)
            if d < 1e-6:
                continue
            min_any = min(min_any, d)
            if x > 0.02 and abs(math.atan2(y, x)) < math.radians(38):
                min_front = min(min_front, d)
            if d < p.d_repel_m:
                wgt = (1.0 / d - 1.0 / p.d_repel_m) / (d * d)
                rep_x += -wgt * (x / d)
                rep_y += -wgt * (y / d)

        alpha = 2.0 if min_any < p.d_slow_m else 1.0
        vx = gx + alpha * rep_x
        vy = gy + alpha * rep_y
        mag = math.hypot(vx, vy)
        if mag > 1e-6:
            vx /= mag
            vy /= mag

        steer = math.atan2(vy, max(0.08, vx))
        w_goal = clamp(p.w_goal_gain * steer, -p.w_max_rps, p.w_max_rps)

        if min_front < p.d_slow_m:
            side_L = side_R = 99.0
            for x, y in scan_xy:
                if x < 0.05:
                    continue
                ang = math.atan2(y, x)
                if math.radians(15) < ang < math.radians(85):
                    side_L = min(side_L, math.hypot(x, y))
                if math.radians(-85) < ang < math.radians(-15):
                    side_R = min(side_R, math.hypot(x, y))
            if side_L > side_R + 0.05:
                w_goal += p.w_avoid_gain * 0.55
            elif side_R > side_L + 0.05:
                w_goal -= p.w_avoid_gain * 0.55

        w_goal = clamp(w_goal, -p.w_max_rps, p.w_max_rps)

        if min_front < p.d_stop_m:
            v = 0.0
        elif min_front < p.d_slow_m:
            t = (min_front - p.d_stop_m) / max(p.d_slow_m - p.d_stop_m, 1e-6)
            v = p.v_cruise_mps * clamp(t, 0.0, 1.0)
        else:
            v = p.v_cruise_mps

        if dist_goal < p.slow_near_goal_radius_m:
            cap = p.v_near_goal_max_mps + (p.v_cruise_mps - p.v_near_goal_max_mps) * clamp(
                dist_goal / p.slow_near_goal_radius_m, 0.0, 1.0
            )
            v = min(v, cap)

        v = clamp(v, 0.0, p.v_max_mps)
        if min_any < p.COLLISION_DIST_M + 0.06:
            v = min(v, p.v_cruise_mps * 0.3)
        if min_any < p.COLLISION_DIST_M:
            v = 0.0
        return float(v), float(w_goal)


def propagate_odom(x: float, y: float, theta: float, v: float, w: float, dt: float) -> Tuple[float, float, float]:
    dt = max(dt, 0.0)
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta = wrap_pi(theta + w * dt)
    return x, y, theta


# ---------------------------------------------------------------------------
# RPLidar C1
# ---------------------------------------------------------------------------


class RPLidarC1:
    def __init__(self, port: str, baudrate: int = 460800, timeout: float = 0.05) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: List[ScanPoint] = []
        self._last_scan_wall_s: float = 0.0

    def connect(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            self.port,
            self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )

    def disconnect(self) -> None:
        self.stop_scanner()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _write_cmd(self, payload: bytes) -> None:
        assert self._ser is not None
        self._ser.write(payload)
        self._ser.flush()

    def reset(self) -> None:
        self.connect()
        assert self._ser is not None
        self._write_cmd(bytes([0xA5, 0x40]))
        time.sleep(0.05)
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

    def stop_device(self) -> None:
        if not self._ser or not self._ser.is_open:
            return
        try:
            self._write_cmd(bytes([0xA5, 0x25]))
            time.sleep(0.01)
        except Exception:
            pass

    def start_scan(self) -> None:
        self.connect()
        assert self._ser is not None
        self._write_cmd(bytes([0xA5, 0x20]))
        desc = bytearray()
        deadline = time.time() + 2.0
        while len(desc) < 7 and time.time() < deadline:
            chunk = self._ser.read(7 - len(desc))
            if chunk:
                desc.extend(chunk)
            else:
                time.sleep(0.001)
        if len(desc) != 7:
            raise RuntimeError(f"LiDAR descriptor expected 7 bytes, got {len(desc)!r}")

    def _parse_node(self, pkt: bytes) -> Optional[Tuple[int, float, float, bool]]:
        if len(pkt) != 5:
            return None
        b0, b1, b2, b3, b4 = pkt
        s_flag = bool(b0 & 0x01)
        quality = int(b0 >> 2)
        angle_q6 = (b1 >> 1) | (b2 << 7)
        angle_deg = angle_q6 / 64.0
        dist_raw = b3 | (b4 << 8)
        dist_mm = dist_raw / 4.0
        dist_m = dist_mm / 1000.0
        return quality, angle_deg, dist_m, s_flag

    def _scanner_loop(self) -> None:
        assert self._ser is not None
        buf = bytearray()
        current: List[ScanPoint] = []

        while not self._stop.is_set():
            chunk = self._ser.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= 5:
                pkt = bytes(buf[:5])
                del buf[:5]
                parsed = self._parse_node(pkt)
                if parsed is None:
                    continue
                quality, angle_deg, dist_m, s_flag = parsed
                if dist_m <= 0.0:
                    continue
                if s_flag:
                    if len(current) > 30:
                        with self._lock:
                            self._latest = current
                            self._last_scan_wall_s = time.time()
                    current = [(quality, angle_deg, dist_m)]
                else:
                    current.append((quality, angle_deg, dist_m))

    def start_scanner_thread(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self.reset()
        self.start_scan()
        self._stop.clear()
        self._thread = threading.Thread(target=self._scanner_loop, daemon=True)
        self._thread.start()

    def stop_scanner(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.stop_device()
        try:
            if self._ser and self._ser.is_open:
                self._ser.reset_input_buffer()
        except Exception:
            pass

    def get_scan(self) -> List[ScanPoint]:
        with self._lock:
            return list(self._latest)

    def last_scan_age_s(self) -> float:
        return time.time() - self._last_scan_wall_s


# ---------------------------------------------------------------------------
# 아두이노
# ---------------------------------------------------------------------------


class MotorSerial:
    def __init__(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.02, write_timeout=0.2)

    def send_velocity(self, v: float, w: float) -> None:
        self.ser.write(f"V{v:.3f},{w:.3f}\n".encode("ascii"))
        self.ser.flush()

    def send_stop(self) -> None:
        self.ser.write(b"S\n")
        self.ser.flush()

    def close(self) -> None:
        try:
            self.send_stop()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


def start_stdin_stop_listener(stop_evt: threading.Event) -> None:
    """터미널에서 줄 단위로 읽음: 대소문자 무관하게 'S'만 있으면 stop_evt 설정."""

    def _read_loop() -> None:
        try:
            for line in sys.stdin:
                if line.strip().upper() == "S":
                    stop_evt.set()
                    return
        except Exception:
            pass

    if sys.stdin.isatty():
        threading.Thread(target=_read_loop, daemon=True).start()


# ---------------------------------------------------------------------------
# 메인
# ---------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser(description="장애물 회피 자율주행 (LiDAR + Arduino)")
    ap.add_argument("--arduino", default="/dev/ttyS0", help="아두이노 시리얼 포트")
    ap.add_argument("--lidar", default="/dev/ttyUSB0", help="RPLidar C1 시리얼 포트")
    ap.add_argument(
        "--start-delay",
        type=float,
        default=0.0,
        help="출발 전 추가 대기 [s] (기본 0, 과제 시간 절약)",
    )
    args = ap.parse_args()

    p = Params()
    p.arduino_port = args.arduino
    p.lidar_port = args.lidar

    lidar = RPLidarC1(p.lidar_port, baudrate=p.lidar_baud)
    motor: Optional[MotorSerial] = None
    running = True
    stop_stdin = threading.Event()
    start_stdin_stop_listener(stop_stdin)

    def _sig(_s, _f):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    try:
        motor = MotorSerial(p.arduino_port, p.arduino_baud)
        # ttyS0(GPIO UART)면 USB 리셋 대기 불필요. 시리얼 버퍼만 짧게 안정화.
        time.sleep(0.12)
        lidar.start_scanner_thread()
        if args.start_delay > 0:
            time.sleep(args.start_delay)

        if sys.stdin.isatty():
            print("종료: 터미널에 S 입력 후 Enter  (Ctrl+C도 가능)", flush=True)

        ctrl = ObstacleAvoidController(p)
        lim_w = RateLimiterW(p.max_dw_dt)
        x = y = theta = 0.0
        t0 = time.time()
        last = t0
        goal_sent = False
        dt_nom = 1.0 / p.hz

        while running:
            if stop_stdin.is_set():
                break
            now = time.time()
            if now - t0 > p.mission_timeout_s:
                break
            dt = now - last
            if dt < dt_nom:
                time.sleep(max(0.0, dt_nom - dt))
                now = time.time()
                dt = now - last
            last = now
            dt = max(min(dt, 0.25), 1.0 / 200.0)

            scan = lidar.get_scan()
            if lidar.last_scan_age_s() > 0.35:
                motor.send_velocity(0.0, 0.0)
                time.sleep(0.02)
                continue

            pts = scan_to_xy(scan, p)
            dg = goal_distance_m(x, y, p)
            if dg < p.GOAL_TOL_M and not goal_sent:
                motor.send_stop()
                goal_sent = True
                break

            h_err = goal_heading_error(x, y, theta, p)
            v_cmd, w_raw = ctrl.compute(pts, h_err, dg)
            w_cmd = lim_w.step(w_raw, dt)

            if v_cmd < 0.02 and abs(w_cmd) < 0.05:
                motor.send_stop()
            else:
                motor.send_velocity(v_cmd, w_cmd)

            x, y, theta = propagate_odom(x, y, theta, v_cmd, w_cmd, dt)

    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        raise
    finally:
        try:
            if motor:
                motor.send_stop()
                motor.close()
        except Exception:
            pass
        lidar.stop_scanner()
        lidar.disconnect()
        print("DONE")


if __name__ == "__main__":
    main()
