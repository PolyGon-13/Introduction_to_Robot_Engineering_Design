#!/usr/bin/env python3
# maze3.py
# RPLiDAR C1 + MPPI-inspired local planner

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

BASE_V = 0.15
FRONT_ANGLE_OFFSET_DEG = -1.54
LIDAR_ANGLE_SIGN = -1.0
MIN_LIDAR_DIST_M = 0.05
MAX_LIDAR_DIST_M = 2.0
QUALITY_MIN = 0
MIN_X_FOR_PLANNING = -0.10
MAX_EVAL_POINTS = 600

HORIZON_STEPS = 8
DT = 0.1
NUM_SAMPLES = 80
W_MIN = -1.2
W_MAX = 1.2
W_NOISE_STD = 0.45
V_NOISE_STD = 0.0

ROBOT_RADIUS = 0.13
SAFETY_MARGIN = 0.08
COLLISION_DIST = ROBOT_RADIUS + SAFETY_MARGIN
NEAR_DIST = 0.45
FRONT_CONE_DEG = 30.0
FRONT_DANGER_DIST = 0.60

COLLISION_COST_WEIGHT = 1000.0
NEAR_COST_WEIGHT = 20.0
TURN_COST_WEIGHT = 0.25
SMOOTH_COST_WEIGHT = 0.5
HEADING_COST_WEIGHT = 0.15
FRONT_DANGER_WEIGHT = 10.0

SCAN_HOLD_S = 0.30
LOOP_DT_S = 0.05
MISSION_DURATION_S = 55.0
LOG_PERIOD_S = 0.25
RNG_SEED = None

prev_w_sequence = None
rng = np.random.default_rng(RNG_SEED)


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
    return (angle + 180.0) % 360.0 - 180.0


def lidar_points_to_xy(scan_points):
    if scan_points is None:
        return np.empty((0, 4), dtype=np.float32)

    angles, dists, qualities = scan_points
    distance_m = dists.astype(np.float32) / 1000.0
    angle_corrected = normalize_angle_deg(angles.astype(np.float32) - FRONT_ANGLE_OFFSET_DEG)
    angle_corrected = normalize_angle_deg(LIDAR_ANGLE_SIGN * angle_corrected)

    mask = ((distance_m >= MIN_LIDAR_DIST_M) &
            (distance_m <= MAX_LIDAR_DIST_M) &
            (qualities >= QUALITY_MIN))
    if not mask.any():
        return np.empty((0, 4), dtype=np.float32)

    distance_m = distance_m[mask]
    angle_corrected = angle_corrected[mask]
    angle_rad = np.deg2rad(angle_corrected)
    x = distance_m * np.cos(angle_rad)
    y = distance_m * np.sin(angle_rad)

    front_mask = x >= MIN_X_FOR_PLANNING
    if not front_mask.any():
        return np.empty((0, 4), dtype=np.float32)

    points = np.column_stack((x[front_mask],
                              y[front_mask],
                              angle_corrected[front_mask],
                              distance_m[front_mask])).astype(np.float32)
    if len(points) > MAX_EVAL_POINTS:
        order = np.argsort(points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1])
        points = points[order[:MAX_EVAL_POINTS]]
    return points


def rollout_trajectory(v_sequence, w_sequence):
    x = 0.0
    y = 0.0
    theta = 0.0
    traj = np.zeros((HORIZON_STEPS, 5), dtype=np.float32)

    for i in range(HORIZON_STEPS):
        v = float(v_sequence[i])
        w = float(w_sequence[i])
        x += v * math.cos(theta) * DT
        y += v * math.sin(theta) * DT
        theta += w * DT
        traj[i] = (x, y, theta, v, w)

    return traj


def compute_min_clearance(traj, points):
    if len(points) == 0:
        return MAX_LIDAR_DIST_M

    traj_xy = traj[:, :2]
    point_xy = points[:, :2]
    diff = traj_xy[:, None, :] - point_xy[None, :, :]
    dist = np.sqrt(np.sum(diff * diff, axis=2))
    return float(np.min(dist))


def has_front_danger(points):
    if len(points) == 0:
        return False
    mask = ((np.abs(points[:, 2]) <= FRONT_CONE_DEG) &
            (points[:, 3] <= FRONT_DANGER_DIST) &
            (points[:, 0] > 0.0))
    return bool(mask.any())


def evaluate_sequence(v_sequence, w_sequence, points, prev_w):
    traj = rollout_trajectory(v_sequence, w_sequence)
    min_clearance = compute_min_clearance(traj, points)
    front_danger = has_front_danger(points)

    cost = 0.0

    if min_clearance < COLLISION_DIST:
        cost += COLLISION_COST_WEIGHT * (COLLISION_DIST - min_clearance + 1.0)

    if min_clearance < NEAR_DIST:
        cost += NEAR_COST_WEIGHT * (NEAR_DIST - min_clearance)

    last_w = prev_w
    for _, _, theta, _, w in traj:
        cost += TURN_COST_WEIGHT * abs(float(w))
        cost += SMOOTH_COST_WEIGHT * abs(float(w) - last_w)
        cost += HEADING_COST_WEIGHT * abs(float(theta))
        last_w = float(w)

    if front_danger:
        first_w = float(w_sequence[0])
        cost += FRONT_DANGER_WEIGHT * max(0.0, 1.0 - abs(first_w) / max(abs(W_MIN), abs(W_MAX)))

    return cost, min_clearance, front_danger


def make_nominal_w_sequence():
    global prev_w_sequence

    if prev_w_sequence is None:
        return np.zeros(HORIZON_STEPS, dtype=np.float32)

    shifted = np.zeros(HORIZON_STEPS, dtype=np.float32)
    shifted[:-1] = prev_w_sequence[1:]
    shifted[-1] = prev_w_sequence[-1]
    return shifted


def sample_w_sequences(nominal_w_sequence):
    noise = rng.normal(0.0, W_NOISE_STD, size=(NUM_SAMPLES, HORIZON_STEPS)).astype(np.float32)
    sampled = np.clip(nominal_w_sequence[None, :] + noise, W_MIN, W_MAX)

    deterministic_values = [0.0, -0.3, 0.3, -0.7, 0.7, -1.1, 1.1]
    deterministic = [np.full(HORIZON_STEPS, val, dtype=np.float32)
                     for val in deterministic_values]
    deterministic.append(np.clip(nominal_w_sequence, W_MIN, W_MAX).astype(np.float32))

    return list(sampled) + deterministic


def choose_mppi_cmd(scan_points, prev_w):
    global prev_w_sequence

    points = lidar_points_to_xy(scan_points)

    if len(points) == 0:
        if prev_w_sequence is None:
            best_w = prev_w
        else:
            best_w = float(prev_w_sequence[0])
        return BASE_V, float(np.clip(best_w, W_MIN, W_MAX)), {
            "cost": 0.0,
            "clear": MAX_LIDAR_DIST_M,
            "points": 0,
            "front_danger": False,
        }

    nominal_w = make_nominal_w_sequence()
    w_sequences = sample_w_sequences(nominal_w)
    v_sequence = np.full(HORIZON_STEPS, BASE_V, dtype=np.float32)

    best_cost = float("inf")
    best_w_sequence = None
    best_clearance = 0.0
    best_front_danger = False

    for w_sequence in w_sequences:
        cost, clearance, front_danger = evaluate_sequence(v_sequence, w_sequence, points, prev_w)
        if cost < best_cost:
            best_cost = cost
            best_w_sequence = w_sequence
            best_clearance = clearance
            best_front_danger = front_danger

    if best_w_sequence is None:
        best_w = prev_w
    else:
        prev_w_sequence = np.array(best_w_sequence, dtype=np.float32)
        best_w = float(best_w_sequence[0])

    best_w = float(np.clip(best_w, W_MIN, W_MAX))
    return BASE_V, best_w, {
        "cost": best_cost,
        "clear": best_clearance,
        "points": len(points),
        "front_danger": best_front_danger,
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
    print("[READY] 초기화 완료. Enter를 누르면 MPPI-inspired local planner 주행 시작.")
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
            v, w, info = choose_mppi_cmd(scan, last_w)
            send_vw(v, w)
            last_v, last_w = v, w

            if time.time() - last_log > LOG_PERIOD_S:
                print(f"[RUN3] v={v:.2f} w={w:.2f} cost={info['cost']:.2f} "
                      f"clear={info['clear']:.2f} pts={info['points']} "
                      f"front={int(info['front_danger'])}")
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
