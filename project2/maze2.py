#!/usr/bin/env python3
# maze2.py
# RPLiDAR C1 + 후보 각속도 평가 기반 로컬 플래너

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
LIDAR_ANGLE_SIGN = -1.0

# 좌표계: +각도/+y = 좌측, -각도/-y = 우측

# 라이다 전처리 파라미터
MIN_LIDAR_DIST_M = 0.05
MAX_LIDAR_DIST_M = 2.5
MIN_QUALITY = 1
MIN_X_FOR_PLANNING = -0.10
MAX_EVAL_POINTS = 720
SCAN_HOLD_S = 0.30
LOOP_DT_S = 0.05

# 로컬 플래너 파라미터
BASE_V = 0.18
W_CANDIDATES = [-1.20, -0.90, -0.60, -0.35, -0.15, 0.0,
                0.15, 0.35, 0.60, 0.90, 1.20]
PREDICT_TIME = 0.80
PREDICT_DT = 0.10
ROBOT_RADIUS = 0.13
SAFETY_MARGIN = 0.08
COLLISION_DIST = ROBOT_RADIUS + SAFETY_MARGIN
CLEARANCE_CAP = 1.0
FRONT_CORRIDOR_HALF = COLLISION_DIST
ACTIVE_FRONT_DIST = 0.95
W_CMD_RATE_LIMIT = 0.35
W_CMD_RATE_LIMIT_URGENT = 0.70
URGENT_FRONT_DIST = 0.45

clearance_weight = 3.0
collision_weight = 100.0
forward_weight = 1.0
far_forward_weight = 2.2
turn_weight = 0.25
far_turn_weight = 0.55
smooth_weight = 0.5

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


def normalize_angle_deg(angle):
    angle = (angle + 180.0) % 360.0 - 180.0
    return angle


def lidar_points_to_xy(scan):
    if scan is None:
        return np.empty((0, 2), dtype=np.float32)

    angles, dists, qualities = scan
    dist_m = (dists.astype(np.float32) + DIST_OFFSET_MM) / 1000.0
    angle_deg = normalize_angle_deg(angles.astype(np.float32) + ANGLE_OFFSET_DEG)
    angle_deg = LIDAR_ANGLE_SIGN * angle_deg

    mask = ((dist_m >= MIN_LIDAR_DIST_M) &
            (dist_m <= MAX_LIDAR_DIST_M) &
            (qualities >= MIN_QUALITY))
    if not mask.any():
        return np.empty((0, 2), dtype=np.float32)

    dist_m = dist_m[mask]
    angle_rad = np.deg2rad(angle_deg[mask])
    x = dist_m * np.cos(angle_rad)
    y = dist_m * np.sin(angle_rad)

    mask_xy = x >= MIN_X_FOR_PLANNING
    if not mask_xy.any():
        return np.empty((0, 2), dtype=np.float32)

    points = np.column_stack((x[mask_xy], y[mask_xy])).astype(np.float32)
    if len(points) > MAX_EVAL_POINTS:
        order = np.argsort(points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1])
        points = points[order[:MAX_EVAL_POINTS]]
    return points


def predict_trajectory(v, w):
    traj = []
    x = 0.0
    y = 0.0
    theta = 0.0
    t = 0.0
    while t < PREDICT_TIME:
        x += v * math.cos(theta) * PREDICT_DT
        y += v * math.sin(theta) * PREDICT_DT
        theta += w * PREDICT_DT
        traj.append((x, y, theta))
        t += PREDICT_DT
    return np.array(traj, dtype=np.float32)


def front_distance(points):
    if len(points) == 0:
        return MAX_LIDAR_DIST_M
    mask = ((points[:, 0] > 0.0) &
            (points[:, 0] < ACTIVE_FRONT_DIST) &
            (np.abs(points[:, 1]) < FRONT_CORRIDOR_HALF))
    if not mask.any():
        return MAX_LIDAR_DIST_M
    return float(np.min(points[mask, 0]))


def evaluate_candidate(v, w, points, prev_w, front_dist):
    traj = predict_trajectory(v, w)
    if len(points) == 0:
        min_clearance = MAX_LIDAR_DIST_M
    else:
        traj_xy = traj[:, :2]
        diff = traj_xy[:, None, :] - points[None, :, :]
        dist = np.sqrt(np.sum(diff * diff, axis=2))
        min_clearance = float(np.min(dist))

    max_abs_w = max(abs(wc) for wc in W_CANDIDATES)
    front_factor = float(np.clip((ACTIVE_FRONT_DIST - front_dist) /
                                 max(1e-6, ACTIVE_FRONT_DIST - COLLISION_DIST),
                                 0.0, 1.0))
    forward_w = forward_weight + (1.0 - front_factor) * far_forward_weight
    turn_w = turn_weight + (1.0 - front_factor) * far_turn_weight

    score = 0.0
    score += clearance_weight * min(min_clearance, CLEARANCE_CAP)
    if min_clearance < COLLISION_DIST:
        score -= collision_weight * (COLLISION_DIST - min_clearance + 1.0)
    score += forward_w * (1.0 - abs(w) / max_abs_w)
    score -= turn_w * abs(w)
    score -= smooth_weight * abs(w - prev_w)

    return score, min_clearance


def rate_limit_w(prev_w, target_w, urgent=False):
    limit = W_CMD_RATE_LIMIT_URGENT if urgent else W_CMD_RATE_LIMIT
    delta = float(np.clip(target_w - prev_w, -limit, limit))
    return prev_w + delta


def choose_best_cmd(scan, prev_w):
    points = lidar_points_to_xy(scan)
    if len(points) == 0:
        return BASE_V, rate_limit_w(prev_w, 0.0), {
            "score": 0.0,
            "clear": MAX_LIDAR_DIST_M,
            "front": MAX_LIDAR_DIST_M,
            "points": 0,
            "collision": False,
            "raw_w": 0.0,
        }

    fdist = front_distance(points)
    best_w = 0.0
    best_score = -float("inf")
    best_clearance = -float("inf")
    all_collision = True
    best_clear_w = 0.0
    best_clear_score = -float("inf")

    for w in W_CANDIDATES:
        score, clearance = evaluate_candidate(BASE_V, w, points, prev_w, fdist)
        collision = clearance < COLLISION_DIST
        if not collision:
            all_collision = False
        clear_score = clearance + 0.03 * abs(w) - 0.02 * abs(w - prev_w)
        if clear_score > best_clear_score:
            best_clear_score = clear_score
            best_clear_w = w
        if score > best_score:
            best_score = score
            best_w = w
            best_clearance = clearance

    if all_collision:
        best_w = best_clear_w
        _, best_clearance = evaluate_candidate(BASE_V, best_w, points, prev_w, fdist)

    raw_best_w = best_w
    best_w = rate_limit_w(prev_w, best_w, fdist < URGENT_FRONT_DIST or all_collision)
    return BASE_V, best_w, {
        "score": best_score,
        "clear": best_clearance,
        "front": fdist,
        "points": len(points),
        "collision": best_clearance < COLLISION_DIST,
        "raw_w": raw_best_w,
    }


# ─────────────── 메인 ───────────────
def main():
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    ardu  = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    print("[INFO] 워밍업 2초..."); time.sleep(2.0)

    def send_vw(v, w):
        ardu.write(f"V{v:.3f},{w:.3f}\n".encode())

    def stop():
        ardu.write(b"S\n")

    stop()
    print("[READY] 초기화 완료. Enter를 누르면 후보 평가형 로컬 플래너 주행 시작.")
    try:
        input()
    except EOFError:
        print("[WARN] 표준입력을 읽을 수 없어 바로 주행 시작")
    print("[GO]")

    t0 = time.time()
    last_scan_ok = 0.0
    last_v, last_w = BASE_V, 0.0
    last_log = 0.0

    try:
        while True:
            if time.time() - t0 > MISSION_DURATION_S:
                print("[INFO] 시간 초과 정지")
                break

            scan = lidar.get_scan()
            if scan is None:
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    send_vw(last_v, last_w)
                else:
                    send_vw(0.0, 0.0)
                time.sleep(LOOP_DT_S); continue

            last_scan_ok = time.time()
            v, w, info = choose_best_cmd(scan, last_w)
            send_vw(v, w)
            last_v, last_w = v, w

            if time.time() - last_log > 0.25:
                print(f"[RUN2] v={v:.2f} w={w:.2f} raw={info['raw_w']:.2f} "
                      f"front={info['front']:.2f} clear={info['clear']:.2f} "
                      f"score={info['score']:.2f} pts={info['points']} "
                      f"coll={int(info['collision'])}")
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
