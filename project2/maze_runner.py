#!/usr/bin/env python3
# maze_runner.py
# RPLiDAR C1 + Follow-the-Gap + Lane-Keeping 회피 주행

import serial
import time
import math
import numpy as np
import threading

# ─────────────── 사용자 튜닝 파라미터 ───────────────
LIDAR_PORT      = "/dev/ttyUSB0"
LIDAR_BAUD      = 460800
ARDU_PORT       = "/dev/ttyS0"      # GPIO UART
ARDU_BAUD       = 9600

# 라이다 캘리브레이션 (실측 후 보정)
ANGLE_OFFSET_DEG = 0.0     # 라이다 0°와 로봇 정면 차이 [deg]
DIST_OFFSET_MM   = 0.0     # 거리 보정 [mm] (보통 0)

# 로봇 형상
ROBOT_HALF_WIDTH = 0.10    # [m] 로봇 반폭(센서~외곽). 안전 마진 포함
ROBOT_FORWARD    = 0.05    # [m] 라이다~로봇 앞면 거리

# FTG 파라미터
FOV_HALF_DEG     = 90      # 정면 ±90° 사용
SAFETY_RADIUS    = 0.45    # [m] 이거보다 가까운 점 주변은 0으로 마스킹
MIN_GAP_DIST     = 0.50    # [m] 갭으로 인정할 최소 거리
DISP_CLAMP       = 3.0     # [m] 너무 먼 점은 이 값으로 클립

# 속도 정책
V_MAX            = 0.35    # [m/s] 직진 최고 속도
V_MIN            = 0.12    # [m/s] 회피 시 최저 속도
W_MAX            = 1.6     # [rad/s] 최대 각속도
K_STEER          = 1.6     # 비례 조향 게인 [rad/s per rad]
BRAKE_DIST       = 0.80    # [m] 이거 이내면 V 감속 시작
EMERG_STOP_DIST  = 0.30    # [m] 이거보다 가까우면 V=0 + 회전만

# 라인 키핑 (좌측 벽 추종 보조)
LANE_LEFT_TARGET = 0.50    # [m] 좌측 벽까지 유지하고 싶은 거리(실측 후 보정)
K_LANE           = 0.6     # 라인 추종 게인 (rad/s per m 오차)
LANE_ACTIVE_DIST = 1.5     # [m] 정면이 이거보다 멀 때만 라인 키핑 활성화

# 미션 종료 조건
MISSION_DURATION_S = 55.0  # 60초 한계 안에서 안전 마진 5초

# ─────────────── 라이다 드라이버 (Practice3 기반 + 스캔 조립) ───────────────
class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self._reset()
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self._start_scan()
        self.lock = threading.Lock()
        self.latest_scan = None  # dict: angle_deg(np) -> dist_m(np)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _reset(self):
        self.ser.write(bytes([0xA5, 0x40]))

    def _start_scan(self):
        self.ser.write(bytes([0xA5, 0x20]))
        # response descriptor (7 bytes) 무시
        self.ser.read(7)

    def _loop(self):
        buf_a, buf_d, buf_q = [], [], []
        while self.running:
            data = self.ser.read(5)
            if len(data) != 5:
                continue
            s_flag     = data[0] & 0x01
            s_inv_flag = (data[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag):
                continue
            check_bit  = data[1] & 0x01
            if check_bit != 1:
                continue
            quality   = data[0] >> 2
            angle_q6  = ((data[1] >> 1) | (data[2] << 7))
            dist_q2   = (data[3] | (data[4] << 8))
            angle_deg = angle_q6 / 64.0
            dist_mm   = dist_q2 / 4.0

            # 새 스캔 시작
            if s_flag == 1 and len(buf_a) > 50:
                with self.lock:
                    self.latest_scan = (
                        np.array(buf_a, dtype=np.float32),
                        np.array(buf_d, dtype=np.float32),
                        np.array(buf_q, dtype=np.float32),
                    )
                buf_a, buf_d, buf_q = [], [], []

            if dist_mm > 0 and quality > 0:  # 0거리/품질0 제거
                buf_a.append(angle_deg)
                buf_d.append(dist_mm)
                buf_q.append(quality)

    def get_scan(self):
        with self.lock:
            return self.latest_scan

    def close(self):
        self.running = False
        try:
            self.ser.write(bytes([0xA5, 0x25]))  # STOP
        except Exception:
            pass
        time.sleep(0.1)
        self.ser.close()


# ─────────────── 스캔 → 정면 거리 배열 변환 ───────────────
def build_front_array(scan, n_bins=181):
    """
    scan: (angles_deg, dists_mm, quality)
    반환: (theta_rad[n_bins], dist_m[n_bins]) — -90°~+90°를 1° 간격으로
    """
    if scan is None:
        return None, None
    a_deg, d_mm, _q = scan

    # 각도 보정
    a_deg = a_deg + ANGLE_OFFSET_DEG
    d_m   = (d_mm + DIST_OFFSET_MM) / 1000.0

    # RPLiDAR 좌표계: 0°=정면(+X), CCW로 증가한다고 가정 (캘리브 단계에서 확인 필수)
    # -> 0~360 데이터를 -180~+180으로 변환
    a_deg = ((a_deg + 180.0) % 360.0) - 180.0

    # 정면 ±FOV_HALF만 추출
    mask = np.abs(a_deg) <= FOV_HALF_DEG
    a_deg = a_deg[mask]; d_m = d_m[mask]

    # bin으로 정렬 (각도 1° 단위)
    bins = np.linspace(-FOV_HALF_DEG, FOV_HALF_DEG, n_bins)
    dist_binned = np.full(n_bins, np.nan, dtype=np.float32)
    if len(a_deg) > 0:
        idx = np.clip(((a_deg + FOV_HALF_DEG)).astype(np.int32), 0, n_bins - 1)
        # 한 bin에 여러 점 → 최솟값(가장 가까운 것)
        for i, di in zip(idx, d_m):
            if np.isnan(dist_binned[i]) or di < dist_binned[i]:
                dist_binned[i] = di

    # NaN(빈 bin)은 이웃 평균/클램프 값으로 채움
    if np.all(np.isnan(dist_binned)):
        return None, None
    nan_mask = np.isnan(dist_binned)
    dist_binned[nan_mask] = DISP_CLAMP
    dist_binned = np.clip(dist_binned, 0.0, DISP_CLAMP)

    theta_rad = np.deg2rad(bins)
    return theta_rad, dist_binned


# ─────────────── Follow-the-Gap 핵심 ───────────────
def follow_the_gap(theta, dist):
    """
    theta: [rad] (-pi/2 .. pi/2),  dist: [m]
    반환: target_angle_rad, min_front_dist
    """
    # 1) 가장 가까운 점 주변에 안전 거품 적용
    closest_i = int(np.argmin(dist))
    closest_d = float(dist[closest_i])
    proc = dist.copy()
    if closest_d < SAFETY_RADIUS:
        # 거품 반각 = atan(R / d), 최소 0.05 rad
        bubble_rad = max(0.05, math.atan(ROBOT_HALF_WIDTH / max(closest_d, 0.05)))
        bin_step = abs(theta[1] - theta[0])
        bubble_bins = int(bubble_rad / bin_step)
        lo = max(0, closest_i - bubble_bins)
        hi = min(len(proc), closest_i + bubble_bins + 1)
        proc[lo:hi] = 0.0

    # 2) 갭 후보: dist > MIN_GAP_DIST 인 연속 구간
    valid = proc > MIN_GAP_DIST
    if not valid.any():
        # 길이 0 → 즉시 정지/후진해야 함
        return None, closest_d

    # 가장 긴 연속 True 구간 찾기
    best_lo, best_hi, best_len = 0, 0, 0
    i = 0
    while i < len(valid):
        if valid[i]:
            j = i
            while j < len(valid) and valid[j]:
                j += 1
            if (j - i) > best_len:
                best_len = j - i
                best_lo, best_hi = i, j
            i = j
        else:
            i += 1

    # 3) 갭 안에서 가장 깊은 (가장 멀리 보이는) 점을 목표로
    seg = proc[best_lo:best_hi]
    deepest_local = int(np.argmax(seg))
    target_i = best_lo + deepest_local
    target_angle = float(theta[target_i])

    # 정면 ±10°에서의 최소 거리 (감속 판단용)
    front_mask = np.abs(theta) <= np.deg2rad(10.0)
    min_front = float(np.min(dist[front_mask]))
    return target_angle, min_front


# ─────────────── 좌측 벽 거리 추정 (라인 키핑) ───────────────
def left_wall_distance(scan):
    if scan is None:
        return None
    a_deg, d_mm, _ = scan
    a_deg = a_deg + ANGLE_OFFSET_DEG
    a_deg = ((a_deg + 180.0) % 360.0) - 180.0
    # 좌측 70~110° 영역의 중앙값을 좌측 벽 거리로
    mask = (a_deg > 70) & (a_deg < 110) & (d_mm > 50) & (d_mm < 4000)
    if mask.sum() < 5:
        return None
    return float(np.median(d_mm[mask])) / 1000.0


# ─────────────── 메인 루프 ───────────────
def main():
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    ardu  = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    print("[INFO] 워밍업 2초...")
    time.sleep(2.0)

    def send_vw(v, w):
        msg = f"V{v:.3f},{w:.3f}\n"
        ardu.write(msg.encode())

    def stop():
        ardu.write(b"S\n")

    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            if t > MISSION_DURATION_S:
                print("[INFO] 시간 초과, 정지")
                break

            scan = lidar.get_scan()
            if scan is None:
                send_vw(0, 0); time.sleep(0.05); continue

            theta, dist = build_front_array(scan, n_bins=181)
            if theta is None:
                send_vw(0, 0); time.sleep(0.05); continue

            target_angle, min_front = follow_the_gap(theta, dist)

            if target_angle is None:
                # 모든 방향 막힘 → 천천히 후진하며 좌회전 시도
                send_vw(-0.10, 1.0)
                time.sleep(0.1); continue

            # 1차: 비례 조향
            w_ftg = K_STEER * target_angle

            # 2차: 라인 키핑 (정면이 충분히 멀 때만)
            w_lane = 0.0
            if min_front > LANE_ACTIVE_DIST:
                lwd = left_wall_distance(scan)
                if lwd is not None:
                    err = LANE_LEFT_TARGET - lwd      # +면 너무 멀다(좌측으로 더 붙어야)
                    # 좌측이 +각도라고 가정 → 좌로 가려면 w>0
                    w_lane = K_LANE * err
            w = w_ftg + w_lane
            w = float(np.clip(w, -W_MAX, W_MAX))

            # 속도: 정면 가까울수록 감속
            if min_front < EMERG_STOP_DIST:
                v = 0.0
                # 회전만 하면서 갭을 찾는다
            else:
                # cos 기반 + 거리 기반 감속
                v_curve = V_MAX * max(0.0, math.cos(target_angle))
                v_brake = V_MAX * np.clip((min_front - EMERG_STOP_DIST) /
                                          (BRAKE_DIST - EMERG_STOP_DIST), 0.0, 1.0)
                v = max(V_MIN, min(v_curve, v_brake))

            send_vw(v, w)

            # ~10Hz 루프 (LiDAR 자체가 10Hz라 자연 동기화)
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        stop()
        time.sleep(0.2)
        ardu.close()
        lidar.close()
        print("[INFO] 종료")


if __name__ == "__main__":
    main()