#!/usr/bin/env python3
# maze_runner.py
# RPLiDAR C1 + Follow-the-Gap + Lane-Keeping 회피 주행

import serial
import time
import math
import threading
import numpy as np

# ─────────────── 사용자 튜닝 파라미터 ───────────────
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
ARDU_PORT  = "/dev/ttyS0"
ARDU_BAUD  = 9600

# 라이다 캘리브레이션 (확정값)
ANGLE_OFFSET_DEG = +1.54
DIST_OFFSET_MM   = 0.0

# 좌표계: +각도 = 우측, -각도 = 좌측 (CW)

# 로봇 형상
ROBOT_HALF_WIDTH = 0.10    # [m]

# FTG 파라미터
FOV_HALF_DEG    = 90
SAFETY_RADIUS   = 0.45     # [m]
MIN_GAP_DIST    = 0.50     # [m]
DISP_CLAMP      = 3.0      # [m]

# 속도/조향 정책: 정상 회피 중에는 정지/감속/후진 없이 조향으로만 피함
V_CRUISE        = 0.18     # [m/s]
W_MAX           = 1.25     # [rad/s]
TURN_BOOST_DIST = 0.75     # [m] 이보다 가까우면 속도 대신 조향량 증가
TURN_BOOST_GAIN = 0.45
NO_GAP_TURN_W   = 1.0      # [rad/s] gap이 없을 때 넓은 쪽으로 강제 조향
OBSTACLE_HALF_DEG = 32.0
OBSTACLE_ON_DIST  = 1.15   # [m]
OBSTACLE_OFF_DIST = 1.45   # [m]
AVOID_W_MIN       = 0.45   # [rad/s]
AVOID_W_MAX       = 1.05   # [rad/s]
AVOID_SWITCH_MARGIN = 0.35 # [m]
W_DEADBAND        = 0.05   # [rad/s]
W_SMOOTH_ALPHA    = 0.35
W_RATE_LIMIT_STEP = 0.12   # [rad/s] per loop
SCAN_HOLD_S     = 0.30     # [s] 짧은 스캔 누락은 직전 명령 유지
LOOP_DT_S       = 0.05

# 라인 키핑 (좌측 흰 벽 추종)
K_LANE          = 0.20
LANE_ACTIVE_DIST = 1.5     # 정면이 이거보다 멀 때만 활성
LANE_DEADBAND_M  = 0.04
W_LANE_LIMIT     = 0.25

# 시작 시점 자동 보정 fallback
LANE_LEFT_TARGET_FALLBACK = 0.55   # 측정 1번 결과

# 미션
MISSION_DURATION_S = 55.0


# ─────────────── 라이다 드라이버 ───────────────
class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.ser.write(bytes([0xA5, 0x40])); time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0xA5, 0x20])); self.ser.read(7)
        self.lock = threading.Lock()
        self.latest_scan = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buf_a, buf_d, buf_q = [], [], []
        while self.running:
            data = self.ser.read(5)
            if len(data) != 5: continue
            s_flag = data[0] & 0x01
            s_inv  = (data[0] & 0x02) >> 1
            if s_inv != (1 - s_flag): continue
            if (data[1] & 0x01) != 1: continue
            quality = data[0] >> 2
            angle  = ((data[1] >> 1) | (data[2] << 7)) / 64.0
            dist   = (data[3] | (data[4] << 8)) / 4.0
            if s_flag == 1 and len(buf_a) > 50:
                with self.lock:
                    self.latest_scan = (np.array(buf_a, dtype=np.float32),
                                        np.array(buf_d, dtype=np.float32),
                                        np.array(buf_q, dtype=np.float32))
                buf_a, buf_d, buf_q = [], [], []
            if dist > 0 and quality > 0:
                buf_a.append(angle); buf_d.append(dist); buf_q.append(quality)

    def get_scan(self):
        with self.lock:
            return self.latest_scan

    def close(self):
        self.running = False
        try: self.ser.write(bytes([0xA5, 0x25]))
        except: pass
        time.sleep(0.1); self.ser.close()


# ─────────────── 정면 ±FOV 거리 배열 ───────────────
def build_front_array(scan, n_bins=181):
    if scan is None: return None, None
    a, d, _q = scan
    a = a + ANGLE_OFFSET_DEG
    d = (d + DIST_OFFSET_MM) / 1000.0
    a = ((a + 180.0) % 360.0) - 180.0
    m = np.abs(a) <= FOV_HALF_DEG
    a, d = a[m], d[m]
    bins = np.linspace(-FOV_HALF_DEG, FOV_HALF_DEG, n_bins)
    out = np.full(n_bins, np.nan, dtype=np.float32)
    if len(a) > 0:
        idx = np.clip((a + FOV_HALF_DEG).astype(np.int32), 0, n_bins - 1)
        for i, di in zip(idx, d):
            if np.isnan(out[i]) or di < out[i]:
                out[i] = di
    if np.all(np.isnan(out)): return None, None
    out[np.isnan(out)] = DISP_CLAMP
    out = np.clip(out, 0.0, DISP_CLAMP)
    theta = np.deg2rad(bins)
    return theta, out


# ─────────────── Follow-the-Gap ───────────────
def follow_the_gap(theta, dist):
    closest_i = int(np.argmin(dist))
    closest_d = float(dist[closest_i])
    proc = dist.copy()
    if closest_d < SAFETY_RADIUS:
        bubble_rad = max(0.05, math.atan(ROBOT_HALF_WIDTH / max(closest_d, 0.05)))
        bin_step = abs(theta[1] - theta[0])
        bb = int(bubble_rad / bin_step)
        lo = max(0, closest_i - bb)
        hi = min(len(proc), closest_i + bb + 1)
        proc[lo:hi] = 0.0

    valid = proc > MIN_GAP_DIST
    if not valid.any():
        return None, closest_d

    best_lo, best_hi, best_len = 0, 0, 0
    i = 0
    while i < len(valid):
        if valid[i]:
            j = i
            while j < len(valid) and valid[j]: j += 1
            if (j - i) > best_len:
                best_len = j - i
                best_lo, best_hi = i, j
            i = j
        else:
            i += 1

    seg = proc[best_lo:best_hi]
    target_i = best_lo + int(np.argmax(seg))
    target_angle = float(theta[target_i])

    front_mask = np.abs(theta) <= np.deg2rad(10.0)
    min_front = float(np.min(dist[front_mask]))
    return target_angle, min_front


def min_in_angle(theta, dist, half_deg):
    mask = np.abs(theta) <= np.deg2rad(half_deg)
    if not mask.any():
        return DISP_CLAMP
    return float(np.min(dist[mask]))


def clearance_score(values):
    if len(values) == 0:
        return 0.0
    return float(np.percentile(values, 75))


def choose_no_gap_turn(theta, dist):
    left_mask = (theta > np.deg2rad(-90.0)) & (theta < np.deg2rad(-20.0))
    right_mask = (theta > np.deg2rad(20.0)) & (theta < np.deg2rad(90.0))
    left_score = clearance_score(dist[left_mask])
    right_score = clearance_score(dist[right_mask])
    return NO_GAP_TURN_W if left_score >= right_score else -NO_GAP_TURN_W


def side_clearance(theta, dist):
    left_mask = (theta > np.deg2rad(-85.0)) & (theta < np.deg2rad(-25.0))
    right_mask = (theta > np.deg2rad(25.0)) & (theta < np.deg2rad(85.0))
    return clearance_score(dist[left_mask]), clearance_score(dist[right_mask])


def choose_avoid_dir(theta, dist, current_dir):
    left_score, right_score = side_clearance(theta, dist)
    if current_dir > 0 and left_score + AVOID_SWITCH_MARGIN >= right_score:
        return current_dir
    if current_dir < 0 and right_score + AVOID_SWITCH_MARGIN >= left_score:
        return current_dir
    return 1.0 if left_score >= right_score else -1.0


def avoid_turn_strength(trigger_front):
    ratio = np.clip((OBSTACLE_OFF_DIST - trigger_front) /
                    max(OBSTACLE_OFF_DIST - MIN_GAP_DIST, 0.01),
                    0.0, 1.0)
    return AVOID_W_MIN + (AVOID_W_MAX - AVOID_W_MIN) * ratio


def apply_deadband(value, deadband):
    return 0.0 if abs(value) < deadband else value


def smooth_steering(prev_w, target_w):
    filtered = prev_w + W_SMOOTH_ALPHA * (target_w - prev_w)
    step = float(np.clip(filtered - prev_w,
                         -W_RATE_LIMIT_STEP,
                         W_RATE_LIMIT_STEP))
    return prev_w + step


# ─────────────── 좌측(흰 벽) 거리: -90° 영역 ───────────────
def left_wall_distance(scan):
    """-90° 근처 ±15° 영역의 median [m]."""
    if scan is None: return None
    a, d, _ = scan
    a = a + ANGLE_OFFSET_DEG
    a = ((a + 180.0) % 360.0) - 180.0
    mask = (a > -105) & (a < -75) & (d > 50) & (d < 4000)
    if mask.sum() < 5: return None
    return float(np.median(d[mask])) / 1000.0


# ─────────────── 메인 ───────────────
def main():
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    ardu  = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    print("[INFO] 워밍업 2초..."); time.sleep(2.0)

    # ─── 시작 시점 자동 보정 ───
    print("[CALIB] 좌측 벽 거리 측정 (1초)...")
    samples = []
    t_end = time.time() + 1.0
    while time.time() < t_end:
        s = lidar.get_scan()
        if s is not None:
            d = left_wall_distance(s)
            if d is not None: samples.append(d)
        time.sleep(0.05)
    if len(samples) >= 5:
        lane_target = float(np.median(samples))
        print(f"[CALIB] LANE_LEFT_TARGET = {lane_target:.3f}m  "
              f"(n={len(samples)}, σ={np.std(samples)*1000:.1f}mm)")
    else:
        lane_target = LANE_LEFT_TARGET_FALLBACK
        print(f"[CALIB] 측정 실패, fallback {lane_target}m 사용")

    def send_vw(v, w):
        ardu.write(f"V{v:.3f},{w:.3f}\n".encode())

    def stop():
        ardu.write(b"S\n")

    print("[INFO] 3초 후 주행 시작...")
    time.sleep(3.0)
    print("[GO]")
    t0 = time.time()
    last_scan_ok = 0.0
    last_v, last_w = 0.0, 0.0
    last_log = 0.0
    obstacle_mode = False
    avoid_dir = 0.0
    try:
        while True:
            t = time.time() - t0
            if t > MISSION_DURATION_S:
                print("[INFO] 시간 초과 정지")
                break

            scan = lidar.get_scan()
            if scan is None:
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    send_vw(last_v, last_w)
                else:
                    send_vw(0, 0)
                time.sleep(LOOP_DT_S); continue

            theta, dist = build_front_array(scan)
            if theta is None:
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    send_vw(last_v, last_w)
                else:
                    send_vw(0, 0)
                time.sleep(LOOP_DT_S); continue
            last_scan_ok = time.time()

            target_angle, min_front = follow_the_gap(theta, dist)
            trigger_front = min_in_angle(theta, dist, OBSTACLE_HALF_DEG)
            if target_angle is None:
                v = V_CRUISE
                avoid_dir = choose_no_gap_turn(theta, dist) / abs(NO_GAP_TURN_W)
                w_target = avoid_dir * NO_GAP_TURN_W
                w = smooth_steering(last_w, w_target)
                send_vw(v, w)
                last_v, last_w = v, w
                time.sleep(LOOP_DT_S); continue

            if obstacle_mode:
                if trigger_front > OBSTACLE_OFF_DIST:
                    obstacle_mode = False
                    avoid_dir = 0.0
            elif trigger_front < OBSTACLE_ON_DIST:
                obstacle_mode = True
                avoid_dir = choose_avoid_dir(theta, dist, avoid_dir)

            w_ftg = 0.0
            if obstacle_mode:
                avoid_dir = choose_avoid_dir(theta, dist, avoid_dir)
                w_ftg = avoid_dir * avoid_turn_strength(trigger_front)

            w_lane = 0.0
            if (not obstacle_mode) and trigger_front > LANE_ACTIVE_DIST:
                lwd = left_wall_distance(scan)
                if lwd is not None:
                    err = lane_target - lwd
                    if abs(err) > LANE_DEADBAND_M:
                        lane_err = err - math.copysign(LANE_DEADBAND_M, err)
                        w_lane = float(np.clip(-K_LANE * lane_err,
                                               -W_LANE_LIMIT,
                                               W_LANE_LIMIT))

            turn_boost = 1.0
            if obstacle_mode and trigger_front < TURN_BOOST_DIST:
                ratio = np.clip((TURN_BOOST_DIST - trigger_front) /
                                max(TURN_BOOST_DIST - MIN_GAP_DIST, 0.01),
                                0.0, 1.0)
                turn_boost += TURN_BOOST_GAIN * ratio
            w_target = float(np.clip((w_ftg + w_lane) * turn_boost,
                                     -W_MAX,
                                     W_MAX))
            w_target = apply_deadband(w_target, W_DEADBAND)
            w = smooth_steering(last_w, w_target)
            v = V_CRUISE

            send_vw(v, w)
            last_v, last_w = v, w
            if time.time() - last_log > 0.25:
                mode = "AVOID" if obstacle_mode else "STRAIGHT"
                print(f"[RUN] {mode} v={v:.2f} w={w:.2f} "
                      f"gap={math.degrees(target_angle):.1f} "
                      f"front={min_front:.2f} trig={trigger_front:.2f} "
                      f"dir={avoid_dir:+.0f}")
                last_log = time.time()
            time.sleep(LOOP_DT_S)

    except KeyboardInterrupt:
        pass
    finally:
        stop(); time.sleep(0.2)
        ardu.close(); lidar.close()
        print("[INFO] 종료")


if __name__ == "__main__":
    main()
