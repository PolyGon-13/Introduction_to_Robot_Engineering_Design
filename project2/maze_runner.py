#!/usr/bin/env python3
# maze_runner.py
# RPLiDAR C1 + corridor 기하 기반 회피 주행

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

# 라이다 전처리 파라미터
FOV_HALF_DEG    = 90
DISP_CLAMP      = 3.0      # [m]

# 속도/조향 정책: 정상 회피 중에는 정지/감속/후진 없이 조향으로만 피함
V_CRUISE        = 0.18     # [m/s]
W_MAX           = 0.75     # [rad/s]
ROBOT_WIDTH      = 0.20    # [m] 바퀴 포함 실측 폭
ROBOT_LENGTH     = 0.17    # [m] 전후 길이
ROBOT_HALF_WIDTH = ROBOT_WIDTH * 0.5
ROBOT_FRONT_FROM_LIDAR = ROBOT_LENGTH * 0.5
SAFETY_MARGIN    = 0.060   # [m]
CORRIDOR_HALF    = ROBOT_HALF_WIDTH + SAFETY_MARGIN
CORRIDOR_LOOKAHEAD = 0.95  # [m]
MIN_OBS_X        = ROBOT_FRONT_FROM_LIDAR
BLOCK_MIN_POINTS = 3
OBSTACLE_X_WINDOW = 0.18   # [m]
PASS_TARGET_MARGIN = 0.035 # [m]
FRONT_HALF_DEG    = 16.0
HEADING_MAX_DEG   = 62.0
PATH_HALF_DEG     = 10.0
HEADING_MIN_CLEAR = 0.50   # [m]
TARGET_CLEAR_DIST = 1.25   # [m]
K_HEADING         = 1.0
HEADING_DEADBAND_DEG = 4.0
FORWARD_COST      = 0.60
PREV_HEADING_COST = 0.85
CLEARANCE_COST    = 1.80
SWITCH_SIGN_COST  = 0.80
LOCKED_SIDE_CLEAR_MIN = 0.28 # [m]
LOCK_RELEASE_SIDE_CLEAR = 0.45 # [m]
SIDE_GUARD_DIST   = 0.42   # [m]
SIDE_GUARD_GAIN   = 1.1
SIDE_GUARD_W_MAX  = 0.30   # [rad/s]
SIDE_HARD_DIST    = 0.24   # [m]
SIDE_HARD_GAIN    = 2.6
SIDE_HARD_W_MAX   = 0.55   # [rad/s]
SIDE_HARD_MIN_TURN = 0.38  # [rad/s]
W_DEADBAND        = 0.05   # [rad/s]
W_SMOOTH_ALPHA    = 0.25
W_RATE_LIMIT_STEP = 0.06   # [rad/s] per loop
W_HARD_SMOOTH_ALPHA = 0.65
W_HARD_RATE_LIMIT_STEP = 0.18 # [rad/s] per loop
SCAN_HOLD_S     = 0.30     # [s] 짧은 스캔 누락은 직전 명령 유지
LOOP_DT_S       = 0.05

# 라인 키핑 (좌측 흰 벽 추종)
K_LANE          = 0.0
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


# ─────────────── corridor 침범 기반 회피각 계산 ───────────────
def min_in_angle(theta, dist, half_deg):
    mask = np.abs(theta) <= np.deg2rad(half_deg)
    if not mask.any():
        return DISP_CLAMP
    return float(np.min(dist[mask]))


def min_in_sector(theta, dist, deg_lo, deg_hi):
    lo = np.deg2rad(deg_lo)
    hi = np.deg2rad(deg_hi)
    mask = (theta >= lo) & (theta <= hi)
    if not mask.any():
        return DISP_CLAMP
    return float(np.min(dist[mask]))


def sector_clearance(theta, dist, center_rad, half_deg):
    half = np.deg2rad(half_deg)
    mask = (theta >= center_rad - half) & (theta <= center_rad + half)
    if not mask.any():
        return DISP_CLAMP
    return float(np.percentile(dist[mask], 20))


def polar_to_xy(theta, dist):
    x = dist * np.cos(theta)
    y = dist * np.sin(theta)
    return x, y


def find_corridor_blocker(theta, dist):
    x, y = polar_to_xy(theta, dist)
    mask = ((x > MIN_OBS_X) &
            (x < CORRIDOR_LOOKAHEAD) &
            (np.abs(y) < CORRIDOR_HALF))
    if int(mask.sum()) < BLOCK_MIN_POINTS:
        return None

    bx = x[mask]
    by = y[mask]
    near_x = float(np.percentile(bx, 10))
    cluster = mask & (x <= near_x + OBSTACLE_X_WINDOW)
    if int(cluster.sum()) < BLOCK_MIN_POINTS:
        cluster = mask

    cx = x[cluster]
    cy = y[cluster]
    return {
        "x": float(np.percentile(cx, 20)),
        "y": float(np.median(cy)),
        "y_min": float(np.percentile(cy, 10)),
        "y_max": float(np.percentile(cy, 90)),
        "points": int(cluster.sum()),
        "front": float(np.min(bx)),
    }


def score_pass_heading(heading, clear, front, prev_heading):
    cost = FORWARD_COST * abs(heading)
    cost += PREV_HEADING_COST * abs(heading - prev_heading)
    cost += CLEARANCE_COST * max(0.0, TARGET_CLEAR_DIST - clear)

    prev_sign = math.copysign(1.0, prev_heading) if abs(prev_heading) > 0.12 else 0.0
    heading_sign = math.copysign(1.0, heading) if abs(heading) > 0.12 else 0.0
    if prev_sign != 0.0 and heading_sign != 0.0 and heading_sign != prev_sign:
        cost += SWITCH_SIGN_COST
    if clear < HEADING_MIN_CLEAR:
        cost += 5.0
    return cost


def choose_heading_corridor(theta, dist, prev_heading, locked_side):
    blocker = find_corridor_blocker(theta, dist)
    straight_clear = sector_clearance(theta, dist, 0.0, PATH_HALF_DEG)
    front = min_in_angle(theta, dist, FRONT_HALF_DEG)
    if blocker is None:
        return 0.0, front, straight_clear, None, 0.0, 0.0, 0

    pass_half = CORRIDOR_HALF + PASS_TARGET_MARGIN
    left_y = blocker["y_min"] - pass_half
    right_y = blocker["y_max"] + pass_half
    left_heading = np.clip(math.atan2(left_y, blocker["x"]),
                           -np.deg2rad(HEADING_MAX_DEG),
                           np.deg2rad(HEADING_MAX_DEG))
    right_heading = np.clip(math.atan2(right_y, blocker["x"]),
                            -np.deg2rad(HEADING_MAX_DEG),
                            np.deg2rad(HEADING_MAX_DEG))

    left_clear = sector_clearance(theta, dist, left_heading, PATH_HALF_DEG)
    right_clear = sector_clearance(theta, dist, right_heading, PATH_HALF_DEG)
    left_cost = score_pass_heading(left_heading, left_clear, blocker["front"], prev_heading)
    right_cost = score_pass_heading(right_heading, right_clear, blocker["front"], prev_heading)

    if locked_side < 0 and left_clear >= LOCKED_SIDE_CLEAR_MIN:
        return left_heading, blocker["front"], left_clear, blocker, left_clear, right_clear, locked_side
    if locked_side > 0 and right_clear >= LOCKED_SIDE_CLEAR_MIN:
        return right_heading, blocker["front"], right_clear, blocker, left_clear, right_clear, locked_side

    if left_cost <= right_cost:
        return left_heading, blocker["front"], left_clear, blocker, left_clear, right_clear, -1
    return right_heading, blocker["front"], right_clear, blocker, left_clear, right_clear, 1


def side_guard_w(theta, dist):
    left_near = min_in_sector(theta, dist, -60.0, -15.0)
    right_near = min_in_sector(theta, dist, 15.0, 60.0)
    left_push = max(0.0, SIDE_GUARD_DIST - left_near)
    right_push = max(0.0, SIDE_GUARD_DIST - right_near)
    if left_near < SIDE_HARD_DIST or right_near < SIDE_HARD_DIST:
        hard_left = max(0.0, SIDE_HARD_DIST - left_near)
        hard_right = max(0.0, SIDE_HARD_DIST - right_near)
        w = SIDE_HARD_GAIN * (hard_right - hard_left)
        return float(np.clip(w, -SIDE_HARD_W_MAX, SIDE_HARD_W_MAX)), left_near, right_near
    w = SIDE_GUARD_GAIN * (right_push - left_push)
    return float(np.clip(w, -SIDE_GUARD_W_MAX, SIDE_GUARD_W_MAX)), left_near, right_near


def apply_deadband(value, deadband):
    return 0.0 if abs(value) < deadband else value


def apply_side_priority(w_target, left_near, right_near):
    hard_side = 0
    if right_near < SIDE_HARD_DIST and right_near <= left_near:
        hard_side = 1
        w_target = max(w_target, SIDE_HARD_MIN_TURN)
    elif left_near < SIDE_HARD_DIST and left_near < right_near:
        hard_side = -1
        w_target = min(w_target, -SIDE_HARD_MIN_TURN)
    return float(np.clip(w_target, -W_MAX, W_MAX)), hard_side


def smooth_steering(prev_w, target_w, alpha=W_SMOOTH_ALPHA, rate_limit=W_RATE_LIMIT_STEP):
    filtered = prev_w + alpha * (target_w - prev_w)
    step = float(np.clip(filtered - prev_w,
                         -rate_limit,
                         rate_limit))
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
    prev_heading = 0.0
    locked_side = 0
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

            heading, front_dist, path_clear, blocker, left_clear, right_clear, chosen_side = (
                choose_heading_corridor(theta, dist, prev_heading, locked_side)
            )
            w_guard, left_near, right_near = side_guard_w(theta, dist)
            if chosen_side != 0:
                locked_side = chosen_side
            elif left_near > LOCK_RELEASE_SIDE_CLEAR and right_near > LOCK_RELEASE_SIDE_CLEAR:
                locked_side = 0

            w_lane = 0.0
            if heading == 0.0 and front_dist > LANE_ACTIVE_DIST:
                lwd = left_wall_distance(scan)
                if lwd is not None:
                    err = lane_target - lwd
                    if abs(err) > LANE_DEADBAND_M:
                        lane_err = err - math.copysign(LANE_DEADBAND_M, err)
                        w_lane = float(np.clip(-K_LANE * lane_err,
                                               -W_LANE_LIMIT,
                                               W_LANE_LIMIT))

            w_heading = -K_HEADING * heading
            w_target = float(np.clip(w_heading + w_guard + w_lane,
                                     -W_MAX,
                                     W_MAX))
            w_target, hard_side = apply_side_priority(w_target, left_near, right_near)
            w_target = apply_deadband(w_target, W_DEADBAND)
            if hard_side != 0:
                w = smooth_steering(last_w, w_target,
                                    W_HARD_SMOOTH_ALPHA,
                                    W_HARD_RATE_LIMIT_STEP)
            else:
                w = smooth_steering(last_w, w_target)
            v = V_CRUISE

            send_vw(v, w)
            last_v, last_w = v, w
            prev_heading = heading
            if time.time() - last_log > 0.25:
                mode = "HARD" if hard_side != 0 else ("AVOID" if abs(heading) > np.deg2rad(HEADING_DEADBAND_DEG) else ("GUARD" if abs(w_guard) > W_DEADBAND else "STRAIGHT"))
                obs_x = blocker["x"] if blocker is not None else 0.0
                obs_y = blocker["y"] if blocker is not None else 0.0
                obs_n = blocker["points"] if blocker is not None else 0
                lock_txt = "L" if locked_side < 0 else ("R" if locked_side > 0 else "-")
                hard_txt = "L" if hard_side < 0 else ("R" if hard_side > 0 else "-")
                print(f"[RUN] {mode} v={v:.2f} w={w:.2f} "
                      f"head={math.degrees(heading):.1f} "
                      f"front={front_dist:.2f} clear={path_clear:.2f} "
                      f"obs=({obs_x:.2f},{obs_y:.2f},n={obs_n}) "
                      f"passL={left_clear:.2f} passR={right_clear:.2f} "
                      f"sideL={left_near:.2f} sideR={right_near:.2f} "
                      f"lock={lock_txt} hard={hard_txt}")
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
