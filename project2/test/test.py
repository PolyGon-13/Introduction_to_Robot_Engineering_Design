#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import math
import threading
from typing import List, Tuple, Optional

import numpy as np


# ============================================================
# 통신 설정
# ============================================================
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600


# ============================================================
# LIDAR 보정 / 필터 설정
# ============================================================
ANGLE_OFFSET_DEG = 1.54
LIDAR_ANGLE_SIGN = -1.0

MIN_LIDAR_DIST_M = 0.05
MAX_LIDAR_DIST_M = 2.50
MIN_QUALITY = 1

FGM_FOV_DEG = 180
ANGLE_STEP_DEG = 1.0

SCAN_TIMEOUT_S = 0.40
LOOP_DT_S = 0.05


# ============================================================
# 로봇 / 주행 설정
# ============================================================
# 로봇 폭 20cm 기준
ROBOT_RADIUS_M = 0.10
SAFETY_MARGIN_M = 0.07
COLLISION_DIST_M = ROBOT_RADIUS_M + SAFETY_MARGIN_M

ACTIVE_OBSTACLE_DIST_M = 0.70
FRONT_DANGER_DIST_M = 0.23
SIDE_WARN_DIST_M = 0.30

BASE_V = 0.17
MIN_V = 0.08
MAX_V = 0.20

FGM_TURN_GAIN = 1.15
MAX_ABS_W = 0.75


# ============================================================
# FGM 점수 가중치
# ============================================================
CLEARANCE_WEIGHT = 3.0
FORWARD_WEIGHT = 1.5
TURN_PENALTY_WEIGHT = 0.8
SIDE_REPULSE_WEIGHT = 1.2


# ============================================================
# RPLIDAR 명령
# ============================================================
RPLIDAR_SYNC_BYTE = 0xA5
RPLIDAR_CMD_STOP = 0x25
RPLIDAR_CMD_SCAN = 0x20


def normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


# ============================================================
# RPLIDAR Reader
# 표준 SCAN 5바이트 샘플 파싱 방식
# ============================================================
class RPLidarReader:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None

        self.running = False
        self.thread: Optional[threading.Thread] = None

        self.lock = threading.Lock()
        self.latest_scan: List[Tuple[float, float]] = []
        self.latest_time = 0.0

    def _send_cmd(self, cmd: int):
        if self.ser is None:
            return
        self.ser.write(bytes([RPLIDAR_SYNC_BYTE, cmd]))
        self.ser.flush()

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        time.sleep(0.5)

        self._send_cmd(RPLIDAR_CMD_STOP)
        time.sleep(0.1)

        self.ser.reset_input_buffer()

        self._send_cmd(RPLIDAR_CMD_SCAN)

        # response descriptor 7바이트 제거
        _ = self.ser.read(7)

        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.ser is not None:
            try:
                self._send_cmd(RPLIDAR_CMD_STOP)
                time.sleep(0.05)
            except Exception:
                pass

            try:
                self.ser.close()
            except Exception:
                pass

            self.ser = None

    def _read_loop(self):
        current_scan: List[Tuple[float, float]] = []

        while self.running:
            try:
                if self.ser is None:
                    break

                data = self.ser.read(5)
                if len(data) != 5:
                    continue

                b0, b1, b2, b3, b4 = data

                start_flag = b0 & 0x01
                inverse_start_flag = (b0 >> 1) & 0x01
                quality = b0 >> 2

                # start flag와 inverse start flag는 서로 달라야 정상
                if start_flag == inverse_start_flag:
                    continue

                # check bit
                if (b1 & 0x01) != 0x01:
                    continue

                angle_q6 = ((b1 >> 1) | (b2 << 7))
                angle_deg_raw = angle_q6 / 64.0

                dist_q2 = b3 | (b4 << 8)
                dist_m = dist_q2 / 4000.0

                angle_deg = normalize_angle_deg(
                    LIDAR_ANGLE_SIGN * (angle_deg_raw + ANGLE_OFFSET_DEG)
                )

                # 새 회전 시작이면 이전 scan 저장
                if start_flag:
                    if len(current_scan) > 30:
                        with self.lock:
                            self.latest_scan = current_scan[:]
                            self.latest_time = time.time()
                    current_scan = []

                if (
                    quality >= MIN_QUALITY
                    and MIN_LIDAR_DIST_M <= dist_m <= MAX_LIDAR_DIST_M
                ):
                    current_scan.append((angle_deg, dist_m))

            except Exception:
                time.sleep(0.01)

    def get_scan(self) -> Tuple[List[Tuple[float, float]], float]:
        with self.lock:
            return self.latest_scan[:], self.latest_time


# ============================================================
# Arduino Motor
# ============================================================
class ArduinoMotor:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)

    def send_velocity(self, v: float, w: float):
        msg = f"V{v:.3f},{w:.3f}\n"
        self.ser.write(msg.encode("utf-8"))
        self.ser.flush()

    def stop(self):
        try:
            self.ser.write(b"S\n")
            self.ser.flush()
        except Exception:
            pass

    def close(self):
        self.stop()
        try:
            self.ser.close()
        except Exception:
            pass


# ============================================================
# Scan 변환
# ============================================================
def scan_to_distance_bins(scan: List[Tuple[float, float]]):
    """
    -90도 ~ +90도 전방 영역을 1도 단위 거리 배열로 변환.

    각도 기준:
    0도  = 정면
    +각도 = 왼쪽
    -각도 = 오른쪽
    """

    half_fov = FGM_FOV_DEG / 2.0

    angles_deg = np.arange(
        -half_fov,
        half_fov + ANGLE_STEP_DEG,
        ANGLE_STEP_DEG,
        dtype=np.float32,
    )

    distances = np.full(
        angles_deg.shape,
        MAX_LIDAR_DIST_M,
        dtype=np.float32,
    )

    for angle_deg, dist_m in scan:
        if -half_fov <= angle_deg <= half_fov:
            idx = int(round((angle_deg + half_fov) / ANGLE_STEP_DEG))

            if 0 <= idx < len(distances):
                if dist_m < distances[idx]:
                    distances[idx] = dist_m

    return angles_deg, distances


def get_sector_min(
    angles_deg: np.ndarray,
    distances: np.ndarray,
    min_deg: float,
    max_deg: float,
    default: float = MAX_LIDAR_DIST_M,
) -> float:
    mask = (angles_deg >= min_deg) & (angles_deg <= max_deg)

    if not np.any(mask):
        return default

    return float(np.min(distances[mask]))


# ============================================================
# FGM 장애물 Bubble 생성
# ============================================================
def apply_obstacle_bubbles(
    angles_deg: np.ndarray,
    distances: np.ndarray,
) -> np.ndarray:
    """
    가까운 장애물 주변 각도를 막는다.
    로봇 반지름 + 안전거리만큼 피하기 위한 부분.
    """

    blocked = np.zeros_like(distances, dtype=bool)

    for i, d in enumerate(distances):
        if d >= ACTIVE_OBSTACLE_DIST_M:
            continue

        if d <= COLLISION_DIST_M:
            bubble_deg = 35.0
        else:
            ratio = min(1.0, COLLISION_DIST_M / max(d, 1e-6))
            bubble_deg = math.degrees(math.asin(ratio)) + 5.0

        angle_center = angles_deg[i]
        blocked |= np.abs(angles_deg - angle_center) <= bubble_deg

    return blocked


# ============================================================
# FGM 제어
# ============================================================
def fgm_control(scan: List[Tuple[float, float]]):
    angles_deg, distances = scan_to_distance_bins(scan)

    front_dist = get_sector_min(angles_deg, distances, -10.0, 10.0)
    left_dist = get_sector_min(angles_deg, distances, 35.0, 90.0)
    right_dist = get_sector_min(angles_deg, distances, -90.0, -35.0)

    blocked = apply_obstacle_bubbles(angles_deg, distances)

    # 정면이 너무 가까우면 정면 후보 강제 차단
    front_danger_mask = (
        (np.abs(angles_deg) <= 12.0)
        & (distances <= FRONT_DANGER_DIST_M)
    )
    blocked |= front_danger_mask

    best_score = -1e9
    best_angle_deg = 0.0
    valid_count = 0

    for i, angle_deg in enumerate(angles_deg):
        if blocked[i]:
            continue

        valid_count += 1

        # 후보 방향 주변 여유 거리
        local_mask = np.abs(angles_deg - angle_deg) <= 8.0
        local_clearance = float(np.min(distances[local_mask]))

        clearance_score = min(
            local_clearance,
            ACTIVE_OBSTACLE_DIST_M,
        ) / ACTIVE_OBSTACLE_DIST_M

        # 정면 선호
        forward_score = math.cos(math.radians(float(angle_deg)))

        # 큰 회전 감점
        turn_penalty = abs(float(angle_deg)) / 90.0

        # 좌우 가까운 장애물 반발
        side_penalty = 0.0

        # 왼쪽 장애물이 가까우면 왼쪽 후보 감점
        if left_dist < SIDE_WARN_DIST_M and angle_deg > 0:
            pressure = (SIDE_WARN_DIST_M - left_dist) / SIDE_WARN_DIST_M
            side_penalty += (
                SIDE_REPULSE_WEIGHT
                * pressure
                * (float(angle_deg) / 90.0)
            )

        # 오른쪽 장애물이 가까우면 오른쪽 후보 감점
        if right_dist < SIDE_WARN_DIST_M and angle_deg < 0:
            pressure = (SIDE_WARN_DIST_M - right_dist) / SIDE_WARN_DIST_M
            side_penalty += (
                SIDE_REPULSE_WEIGHT
                * pressure
                * (abs(float(angle_deg)) / 90.0)
            )

        score = (
            CLEARANCE_WEIGHT * clearance_score
            + FORWARD_WEIGHT * forward_score
            - TURN_PENALTY_WEIGHT * turn_penalty
            - side_penalty
        )

        if score > best_score:
            best_score = score
            best_angle_deg = float(angle_deg)

    # 모든 방향이 막힌 경우: 더 넓은 쪽으로 제자리 회전
    if valid_count == 0:
        v = 0.0

        if left_dist > right_dist:
            best_angle_deg = 45.0
            w = MAX_ABS_W
        else:
            best_angle_deg = -45.0
            w = -MAX_ABS_W

        obstacle_flag = 1

        return (
            v,
            w,
            best_angle_deg,
            front_dist,
            left_dist,
            right_dist,
            obstacle_flag,
            valid_count,
        )

    target_rad = math.radians(best_angle_deg)

    w = FGM_TURN_GAIN * target_rad
    w = float(np.clip(w, -MAX_ABS_W, MAX_ABS_W))

    # 정면이 가까울수록 감속
    front_slow_ratio = np.clip(
        (front_dist - FRONT_DANGER_DIST_M) / 0.45,
        0.0,
        1.0,
    )

    # 회전각이 클수록 감속
    turn_slow_ratio = 1.0 - 0.55 * min(abs(best_angle_deg) / 90.0, 1.0)

    v = BASE_V * front_slow_ratio * turn_slow_ratio
    v = float(np.clip(v, MIN_V, MAX_V))

    # 정면 충돌 위험이면 전진 없이 회전만
    if front_dist <= FRONT_DANGER_DIST_M:
        v = 0.0

    obstacle_flag = 1 if front_dist < ACTIVE_OBSTACLE_DIST_M else 0

    return (
        v,
        w,
        best_angle_deg,
        front_dist,
        left_dist,
        right_dist,
        obstacle_flag,
        valid_count,
    )


# ============================================================
# Main
# ============================================================
def main():
    lidar = RPLidarReader(LIDAR_PORT, LIDAR_BAUD)
    motor = ArduinoMotor(ARDU_PORT, ARDU_BAUD)

    try:
        print("[INFO] Starting RPLidar...")
        lidar.start()

        print("[INFO] Warming up...")
        time.sleep(2.0)

        input("[INFO] Initialization Complete. Press Enter to start!")

        print("[INFO] Go!!")

        while True:
            scan, scan_time = lidar.get_scan()

            if len(scan) == 0 or (time.time() - scan_time) > SCAN_TIMEOUT_S:
                motor.stop()
                print("[WARN] No recent scan. Stop.")
                time.sleep(LOOP_DT_S)
                continue

            (
                v,
                w,
                target_deg,
                front,
                left,
                right,
                obs,
                valid_count,
            ) = fgm_control(scan)

            motor.send_velocity(v, w)

            print(
                f"[FGM] "
                f"obs={obs} "
                f"target={target_deg:+.1f}deg "
                f"v={v:.2f} "
                f"w={w:.2f} "
                f"front={front:.2f} "
                f"left={left:.2f} "
                f"right={right:.2f} "
                f"valid={valid_count}"
            )

            time.sleep(LOOP_DT_S)

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt")

    finally:
        print("[INFO] Shutdown...")
        motor.stop()
        lidar.stop()
        motor.close()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
