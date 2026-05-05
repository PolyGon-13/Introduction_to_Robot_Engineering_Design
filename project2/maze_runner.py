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
V_CRUISE        = 0.14     # [m/s]
W_MAX           = 1.10     # [rad/s]
ROBOT_WIDTH      = 0.20    # [m] 바퀴 포함 실측 폭
ROBOT_LENGTH     = 0.17    # [m] 전후 길이
ROBOT_HALF_WIDTH = ROBOT_WIDTH * 0.5
ROBOT_FRONT_FROM_LIDAR = ROBOT_LENGTH * 0.5
SAFETY_MARGIN    = 0.080   # [m]
CORRIDOR_HALF    = ROBOT_HALF_WIDTH + SAFETY_MARGIN
DETECT_MARGIN    = 0.140   # [m]
DETECT_CORRIDOR_HALF = ROBOT_HALF_WIDTH + DETECT_MARGIN
CORRIDOR_LOOKAHEAD = 1.25  # [m]
MIN_OBS_X        = ROBOT_FRONT_FROM_LIDAR
BLOCK_MIN_POINTS = 3
OBSTACLE_X_WINDOW = 0.24   # [m]
FRONT_HALF_DEG    = 24.0
HEADING_MAX_DEG   = 62.0
HEADING_SAMPLE_DEG = 4.0
HEADING_MIN_CLEAR = 0.40   # [m]
TARGET_CLEAR_DIST = 0.95   # [m]
K_HEADING         = 1.45
HEADING_DEADBAND_DEG = 4.0
FORWARD_COST      = 0.60
PREV_HEADING_COST = 0.75
CLEARANCE_COST    = 1.80
SWITCH_SIGN_COST  = 1.40
CLEAR_DECISIVE_MARGIN = 0.20 # [m]
FRONT_URGENT_DIST   = 0.75   # [m]
FRONT_CRITICAL_DIST = 0.32   # [m]
FORWARD_COST_URGENT      = 0.25
PREV_HEADING_COST_URGENT = 0.15
CLEARANCE_COST_URGENT    = 5.00
SWITCH_SIGN_COST_URGENT  = 3.00
MIN_AVOID_W_URGENT       = 0.50 # [rad/s]
TURN_BALANCE_CLAMP       = 0.75 # [rad], 오른쪽 누적 회전이 +
TURN_BALANCE_DECAY_PER_S = 0.22
SIDE_GUARD_DIST   = 0.45   # [m]
SIDE_GUARD_GAIN   = 0.9
SIDE_GUARD_W_MAX  = 0.30   # [rad/s]
W_DEADBAND        = 0.05   # [rad/s]
W_SMOOTH_ALPHA    = 0.25
W_RATE_LIMIT_STEP = 0.10   # [rad/s] per loop
W_URGENT_SMOOTH_ALPHA    = 0.50
W_URGENT_RATE_LIMIT_STEP = 0.22 # [rad/s] per loop
SCAN_HOLD_S     = 0.30     # [s] 짧은 스캔 누락은 직전 명령 유지
LOOP_DT_S       = 0.05

V_SCALE_FRONT_NEAR = 0.25
V_SCALE_FRONT_FAR  = 0.70
V_SCALE_CLEAR_NEAR = 0.30
V_SCALE_CLEAR_FAR  = 0.80

NO_PATH_CLEAR_THRESH = 0.40
NO_PATH_FRONT_THRESH = 0.30
ROTATE_IN_PLACE_W    = 0.80

STUCK_FRONT_THRESH   = 0.20
STUCK_TRIGGER_COUNT  = 10
STUCK_RECOVERY_V     = -0.06
STUCK_RESET_COUNT    = 30

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


def swept_corridor_clearance(theta, dist, heading):
    x, y = polar_to_xy(theta, dist)
    c = math.cos(heading)
    s = math.sin(heading)
    along = x * c + y * s
    lateral = -x * s + y * c
    mask = ((along > MIN_OBS_X) &
            (along < CORRIDOR_LOOKAHEAD) &
            (np.abs(lateral) < CORRIDOR_HALF))
    if int(mask.sum()) < BLOCK_MIN_POINTS:
        return DISP_CLAMP
    return float(np.percentile(along[mask], 10))


def polar_to_xy(theta, dist):
    x = dist * np.cos(theta)
    y = dist * np.sin(theta)
    return x, y


def front_urgency(front):
    if front >= FRONT_URGENT_DIST:
        return 0.0
    if front <= FRONT_CRITICAL_DIST:
        return 1.0
    return (FRONT_URGENT_DIST - front) / (FRONT_URGENT_DIST - FRONT_CRITICAL_DIST)


def find_corridor_blocker(theta, dist):
    x, y = polar_to_xy(theta, dist)
    mask = ((x > MIN_OBS_X) &
            (x < CORRIDOR_LOOKAHEAD) &
            (np.abs(y) < DETECT_CORRIDOR_HALF))
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


def score_pass_heading(heading, clear, front, prev_heading, urgency):
    forward_cost = FORWARD_COST + (FORWARD_COST_URGENT - FORWARD_COST) * urgency
    prev_cost = PREV_HEADING_COST + (PREV_HEADING_COST_URGENT - PREV_HEADING_COST) * urgency
    clear_cost = CLEARANCE_COST + (CLEARANCE_COST_URGENT - CLEARANCE_COST) * urgency
    switch_cost = SWITCH_SIGN_COST + (SWITCH_SIGN_COST_URGENT - SWITCH_SIGN_COST) * urgency

    cost = forward_cost * abs(heading)
    cost += prev_cost * abs(heading - prev_heading)
    cost += clear_cost * max(0.0, TARGET_CLEAR_DIST - clear)

    prev_sign = math.copysign(1.0, prev_heading) if abs(prev_heading) > 0.12 else 0.0
    heading_sign = math.copysign(1.0, heading) if abs(heading) > 0.12 else 0.0
    if prev_sign != 0.0 and heading_sign != 0.0 and heading_sign != prev_sign:
        cost += switch_cost
    if clear < HEADING_MIN_CLEAR:
        cost += 5.0 * (1.0 + urgency)
    return cost


def choose_side_from_candidates(theta, dist, prev_heading, trigger_dist):
    urgency = front_urgency(trigger_dist)
    best_left = (None, -1.0, float("inf"))
    best_right = (None, -1.0, float("inf"))

    headings_deg = np.arange(HEADING_SAMPLE_DEG,
                             HEADING_MAX_DEG + 0.1,
                             HEADING_SAMPLE_DEG)
    for deg in headings_deg:
        for sign in (-1.0, 1.0):
            heading = sign * np.deg2rad(deg)
            clear = swept_corridor_clearance(theta, dist, heading)
            cost = score_pass_heading(heading, clear, trigger_dist,
                                      prev_heading, urgency)
            if sign < 0.0:
                if cost < best_left[2]:
                    best_left = (heading, clear, cost)
            elif cost < best_right[2]:
                best_right = (heading, clear, cost)

    left_heading, left_clear, left_cost = best_left
    right_heading, right_clear, right_cost = best_right

    if left_heading is None:
        left_heading, left_clear, left_cost = -np.deg2rad(HEADING_MAX_DEG), 0.0, float("inf")
    if right_heading is None:
        right_heading, right_clear, right_cost = np.deg2rad(HEADING_MAX_DEG), 0.0, float("inf")

    if urgency > 0.35 and abs(left_clear - right_clear) > CLEAR_DECISIVE_MARGIN:
        if left_clear > right_clear:
            return left_heading, left_clear, left_clear, right_clear, left_cost, right_cost, urgency
        return right_heading, right_clear, left_clear, right_clear, left_cost, right_cost, urgency

    if left_cost <= right_cost:
        return left_heading, left_clear, left_clear, right_clear, left_cost, right_cost, urgency
    return right_heading, right_clear, left_clear, right_clear, left_cost, right_cost, urgency


def choose_heading_corridor(theta, dist, prev_heading):
    blocker = find_corridor_blocker(theta, dist)
    straight_clear = swept_corridor_clearance(theta, dist, 0.0)
    front = min_in_angle(theta, dist, FRONT_HALF_DEG)
    trigger_dist = min(front, straight_clear)
    if blocker is not None:
        trigger_dist = min(trigger_dist, blocker["front"])

    if straight_clear >= TARGET_CLEAR_DIST and front >= FRONT_URGENT_DIST:
        return 0.0, front, straight_clear, blocker, 0.0, 0.0, 0.0, 0.0, 0.0

    heading, clear, left_clear, right_clear, left_cost, right_cost, urgency = (
        choose_side_from_candidates(theta, dist, prev_heading, trigger_dist)
    )
    return heading, trigger_dist, clear, blocker, left_clear, right_clear, left_cost, right_cost, urgency


def side_guard_w(theta, dist):
    left_near = min_in_sector(theta, dist, -60.0, -15.0)
    right_near = min_in_sector(theta, dist, 15.0, 60.0)
    left_push = max(0.0, SIDE_GUARD_DIST - left_near)
    right_push = max(0.0, SIDE_GUARD_DIST - right_near)
    w = SIDE_GUARD_GAIN * (right_push - left_push)
    return float(np.clip(w, -SIDE_GUARD_W_MAX, SIDE_GUARD_W_MAX)), left_near, right_near


def apply_deadband(value, deadband):
    return 0.0 if abs(value) < deadband else value


def apply_min_avoid_turn(w_target, heading, urgency):
    if urgency <= 0.0 or abs(heading) <= np.deg2rad(HEADING_DEADBAND_DEG):
        return w_target
    min_w = MIN_AVOID_W_URGENT * urgency
    turn_sign = -math.copysign(1.0, heading)
    if w_target * turn_sign < min_w:
        return turn_sign * min_w
    return w_target


def smooth_steering(prev_w, target_w, urgency=0.0):
    alpha = W_SMOOTH_ALPHA + (W_URGENT_SMOOTH_ALPHA - W_SMOOTH_ALPHA) * urgency
    rate_limit = W_RATE_LIMIT_STEP + (W_URGENT_RATE_LIMIT_STEP - W_RATE_LIMIT_STEP) * urgency
    filtered = prev_w + alpha * (target_w - prev_w)
    step = float(np.clip(filtered - prev_w,
                         -rate_limit,
                         rate_limit))
    return prev_w + step


def cruise_velocity(front, clear):
    s_f = float(np.clip((front - V_SCALE_FRONT_NEAR) /
                        (V_SCALE_FRONT_FAR - V_SCALE_FRONT_NEAR),
                        0.0, 1.0))
    s_c = float(np.clip((clear - V_SCALE_CLEAR_NEAR) /
                        (V_SCALE_CLEAR_FAR - V_SCALE_CLEAR_NEAR),
                        0.0, 1.0))
    return V_CRUISE * min(s_f, s_c)


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
    turn_balance = 0.0
    last_cmd_time = time.time()
    stuck_counter = 0
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

            heading, front_dist, path_clear, blocker, left_clear, right_clear, left_cost, right_cost, urgency = (
                choose_heading_corridor(theta, dist, prev_heading)
            )
            w_guard, left_near, right_near = side_guard_w(theta, dist)

            stuck_counter = stuck_counter + 1 if front_dist < STUCK_FRONT_THRESH else 0
            no_good_path = max(left_clear, right_clear) < NO_PATH_CLEAR_THRESH
            critical_front = front_dist < NO_PATH_FRONT_THRESH

            if stuck_counter > STUCK_TRIGGER_COUNT:
                current_mode = "STUCK"
                v = STUCK_RECOVERY_V
                target_w = math.copysign(W_MAX, left_clear - right_clear)
                w = smooth_steering(last_w, target_w, 1.0)
                if stuck_counter > STUCK_RESET_COUNT:
                    stuck_counter = 0
            elif no_good_path and critical_front:
                current_mode = "ROTATE"
                v = 0.0
                target_w = math.copysign(ROTATE_IN_PLACE_W, left_clear - right_clear)
                w = smooth_steering(last_w, target_w, 1.0)
            else:
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
                w_target = apply_min_avoid_turn(w_target, heading, urgency)
                w_target = apply_deadband(w_target, W_DEADBAND)
                w = smooth_steering(last_w, w_target, urgency)
                v = cruise_velocity(front_dist, path_clear)
                if abs(heading) > np.deg2rad(HEADING_DEADBAND_DEG):
                    current_mode = "AVOID"
                elif abs(w_guard) > W_DEADBAND:
                    current_mode = "GUARD"
                else:
                    current_mode = "STRAIGHT"

            send_vw(v, w)
            now = time.time()
            dt_cmd = max(0.0, min(0.20, now - last_cmd_time))
            last_cmd_time = now
            is_avoiding = abs(heading) > np.deg2rad(HEADING_DEADBAND_DEG)
            if is_avoiding:
                turn_balance = float(np.clip(turn_balance - w * dt_cmd,
                                             -TURN_BALANCE_CLAMP,
                                             TURN_BALANCE_CLAMP))
            else:
                balance_decay = max(0.0, 1.0 - TURN_BALANCE_DECAY_PER_S * dt_cmd)
                turn_balance *= balance_decay
            last_v, last_w = v, w
            prev_heading = heading
            if time.time() - last_log > 0.25:
                obs_x = blocker["x"] if blocker is not None else 0.0
                obs_y = blocker["y"] if blocker is not None else 0.0
                obs_n = blocker["points"] if blocker is not None else 0
                print(f"[RUN] {current_mode} v={v:.2f} w={w:.2f} "
                      f"head={math.degrees(heading):.1f} "
                      f"front={front_dist:.2f} clear={path_clear:.2f} "
                      f"obs=({obs_x:.2f},{obs_y:.2f},n={obs_n}) "
                      f"passL={left_clear:.2f} passR={right_clear:.2f} "
                      f"sideL={left_near:.2f} sideR={right_near:.2f} "
                      f"urg={urgency:.2f} cL={left_cost:.2f} cR={right_cost:.2f} "
                      f"bal={math.degrees(turn_balance):.1f} stk={stuck_counter}")
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