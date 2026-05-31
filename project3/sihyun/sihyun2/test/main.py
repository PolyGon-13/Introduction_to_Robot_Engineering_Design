#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time


import numpy as np
import serial

from camera import (
    bottom_center_error,
    close_camera,
    detect,
    draw,
    follow_cmd,
    open_camera,
    pick,
    read_frame,
    x_center_error,
)
from lidar import (
    ANG_MAX,
    ANG_MIN,
    COLOR_TO_LIDAR_DEG,
    FREE_D,
    FRONT_LOG_ZONE,
    LEFT_ZONE,
    RPLidarC1,
    RIGHT_ZONE,
    avoid_cmd,
    front_ranges,
    lidar_zone_distances,
    obstacle_detected,
)



# ==============================
# HSV color following settings
# ==============================

TARGET_SEQUENCE = ("RED", "YELLOW", "BLUE")  # Follow colors in this order.
SHOW_WINDOW = True  # 카메라 인식 화면을 띄울지 여부
BOTTOM_LOST_RATIO = 0.88  # 색상이 화면 아래 88% 지점 아래에서 사라지면 정지로 판단
COLOR_SWITCH_PAUSE_MS = 1000  # 다음 색 추적 전 정지 시간(ms)
COLOR_FORWARD_CENTER_RATIO = 0.95  # 색상 중심이 화면 하단 1/20 구역에 들어오면 전진
COLOR_EXIT_CENTER_ERR = 0.30  # 이 가로 오차 안에서 아래로 사라질 때만 다음 색으로 전환

# ==============================
# Motor settings
# ==============================

ARDU_PORT = "/dev/ttyS0"  # 아두이노 모터 제어 시리얼 포트
ARDU_BAUD = 9600  # 아두이노 시리얼 통신 속도
DRIVE_V = 0.25  # 색 추적, 장애물 회피, 색 완료 후 추가 전진에 공통으로 쓰는 전진 속도
V_STEP = 0.04  # 전진 속도 명령의 루프당 최대 변화량
FOLLOW_W_STEP = 0.15  # 색 추적 모드 회전 속도 명령의 루프당 최대 변화량
AVOID_W_STEP = 0.20  # 장애물 회피 모드 회전 속도 명령의 루프당 최대 변화량
LOOP_DT = 0.05  # 메인 루프 대기 시간(초)
SEARCH_MAX_W = 1.0  # 색 재탐색 모드 최대 회전 속도
SEARCH_TURN_GAIN = 1.0  # 마지막 색 방향을 회전 속도로 바꾸는 비례 계수
SWITCH_SEARCH_W = 1.0  # 다음 색이 안 보일 때 제자리 탐색 회전 속도
ODOM_LOG_INTERVAL = 0.5  # 엔코더 누적값 로그 출력 주기(초)
WHEEL_R = 0.034  # Arduino encoder distance calculation wheel radius(m)
ENC_PPR = 1012.0  # Arduino encoder counts per wheel revolution
ENC_COUNTS_PER_M = ENC_PPR / (2.0 * np.pi * WHEEL_R)
PRE_FORWARD_STOP_MS = 500  # Stop before encoder-based extra forward move(ms)
POST_COLOR_FORWARD_M = 0.0  # Move forward after a color exits bottom before pause(m)
TURN_360_WHEEL_BASE_M = 0.18  # Distance between left/right wheels for encoder-based 360 turn(m)
TURN_360_COUNTS = np.pi * TURN_360_WHEEL_BASE_M * ENC_COUNTS_PER_M


def clamp(value, low, high):
    return float(max(low, min(value, high)))


def rate_limit(prev, target, step):
    return prev + clamp(target - prev, -step, step)


def millis():
    return int(time.monotonic() * 1000)


def wait_ms(duration_ms):
    deadline = millis() + int(duration_ms)
    while millis() < deadline:
        time.sleep(0.001)


class Motor:
    def __init__(self):
        self.ser = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.running = True
        self.odom_l = 0
        self.odom_r = 0
        self.odom_ms = 0
        self.odom_time = 0.0
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        while self.running:
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("O,"):
                    parts = line.split(",")
                    if len(parts) != 4:
                        continue

                    enc_l = int(parts[1])
                    enc_r = int(parts[2])
                    arduino_ms = int(parts[3])

                    with self.lock:
                        self.odom_l = enc_l
                        self.odom_r = enc_r
                        self.odom_ms = arduino_ms
                        self.odom_time = time.time()
                elif line:
                    print(f"[ARDUINO] {line}")
            except (ValueError, serial.SerialException, OSError):
                time.sleep(0.05)

    def vw(self, v, w):
        with self.write_lock:
            self.ser.write(f"V{v:.3f},{w:.3f}\n".encode("ascii"))

    def stop(self):
        with self.write_lock:
            self.ser.write(b"S\n")

    def reset_encoders(self):
        with self.write_lock:
            self.ser.write(b"R\n")

    def get_odom(self):
        with self.lock:
            return self.odom_l, self.odom_r, self.odom_ms, self.odom_time

    def close(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join(timeout=0.5)
        if self.ser.is_open:
            self.ser.close()


def color_angle_from_target(target, frame_shape):
    if target is None:
        return 0.0

    err = bottom_center_error(target, frame_shape)
    return clamp(-err * COLOR_TO_LIDAR_DEG, ANG_MIN, ANG_MAX)


def update_color_memory(target, frame):
    last_color_deg = color_angle_from_target(target, frame.shape)
    center_ratio = float(target[7]) / frame.shape[0]
    return last_color_deg, time.time(), center_ratio, x_center_error(target, frame.shape)


def clear_color_memory():
    return 0.0, 0.0, 0.0, 0.0


def search_next_cmd():
    return "SEARCH: next color", 0.0, SWITCH_SEARCH_W


def obstacle_in_zone(ranges, zone):
    if ranges is None:
        return False

    return float(np.min(ranges[zone])) < FREE_D


def lost_color_obstacle_passed(ranges, last_color_deg):
    if last_color_deg >= 0.0:
        color_side_zone = LEFT_ZONE
    else:
        color_side_zone = RIGHT_ZONE

    return not obstacle_in_zone(ranges, FRONT_LOG_ZONE | color_side_zone)


def get_turn_start_odom(motor):
    enc_l, enc_r, _, odom_time = motor.get_odom()
    if odom_time == 0.0:
        return None
    return enc_l, enc_r


def completed_one_encoder_turn(motor, start_odom):
    if start_odom is None:
        return False

    enc_l, enc_r, _, odom_time = motor.get_odom()
    if odom_time == 0.0 or time.time() - odom_time > 0.5:
        return False

    start_l, start_r = start_odom
    left_counts = abs(enc_l - start_l)
    right_counts = abs(enc_r - start_r)
    return 0.5 * (left_counts + right_counts) >= TURN_360_COUNTS


def drive_forward_by_encoder(motor, distance_m=POST_COLOR_FORWARD_M):
    motor.stop()
    wait_ms(PRE_FORWARD_STOP_MS)

    while True:
        odom = motor.get_odom()
        if odom[3] != 0.0:
            break
        wait_ms(int(LOOP_DT * 1000))

    start_l, start_r, _, _ = odom
    target_counts = abs(distance_m) * ENC_COUNTS_PER_M
    forward_v = abs(DRIVE_V) if distance_m >= 0.0 else -abs(DRIVE_V)

    print(f"[ODOM] move {distance_m:.2f}m target_counts={target_counts:.0f}")

    left_counts = right_counts = 0.0
    while True:
        enc_l, enc_r, _, odom_time = motor.get_odom()
        if odom_time == 0.0 or time.time() - odom_time > 0.5:
            motor.stop()
            print("[ODOM] encoder data timeout during extra forward move")
            return False

        left_counts = abs(enc_l - start_l)
        right_counts = abs(enc_r - start_r)
        if left_counts >= target_counts and right_counts >= target_counts:
            break

        motor.vw(forward_v, 0.0)
        wait_ms(int(LOOP_DT * 1000))

    motor.stop()
    left_m = left_counts / ENC_COUNTS_PER_M
    right_m = right_counts / ENC_COUNTS_PER_M
    ok = left_counts >= target_counts and right_counts >= target_counts
    print(
        f"[ODOM] extra forward done: "
        f"left={left_m:.2f}m({left_counts:.0f}) "
        f"right={right_m:.2f}m({right_counts:.0f})"
    )
    return ok


def log_lidar(elapsed, lidar_dist):
    if lidar_dist is None:
        print(f"[{elapsed:.2f}s] [LIDAR] waiting...")
        return

    (left_d, left_n), (front_d, front_n), (right_d, right_n) = lidar_dist
    print(
        f"[{elapsed:.2f}s] [LIDAR] "
        f"left={left_d:.2f}m({left_n}) "
        f"front={front_d:.2f}m({front_n}) "
        f"right={right_d:.2f}m({right_n})"
    )


def log_avoid(elapsed, tag, target_deg, target_v, target_w, gap_count):
    print(
        f"[{elapsed:.2f}s] [{tag}] gap={gap_count} "
        f"target={target_deg:.0f} "
        f"v={target_v:.2f} "
        f"w={target_w:.2f}"
    )


def log_odom(elapsed, odom):
    enc_l, enc_r, arduino_ms, odom_time = odom
    if odom_time == 0.0:
        print(f"[{elapsed:.2f}s] [ODOM] waiting...")
        return

    print(f"[{elapsed:.2f}s] [ODOM] left={enc_l} right={enc_r} arduino_ms={arduino_ms}")


def color_cmd(target, frame, ranges, has_obstacle, color_deg, center_ratio, elapsed):
    if has_obstacle:
        target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, color_deg, DRIVE_V)
        log_avoid(elapsed, "AVOID", target_deg, target_v, target_w, gap_count)
        return "AVOID: color + obstacle", target_v, target_w

    target_v, target_w = follow_cmd(target, frame.shape, DRIVE_V)
    return "FOLLOW: color only", target_v, target_w


def search_last_cmd(last_color_deg):
    w = SEARCH_MAX_W if last_color_deg >= 0.0 else -SEARCH_MAX_W
    return "SEARCH: last color direction", 0.0, w


def avoid_mode(mode, tag, ranges, color_deg, elapsed):
    target_v, target_w, target_deg, gap_count = avoid_cmd(ranges, color_deg, DRIVE_V)
    log_avoid(elapsed, tag, target_deg, target_v, target_w, gap_count)
    return mode, target_v, target_w


def main():
    cam = lidar = motor = None
    last_v = last_w = 0.0
    last_color_deg = last_color_time = last_color_center_ratio = last_color_x_err = 0.0
    last_odom_log_time = 0.0
    color_lost_during_avoid = False
    search_failed_avoid_active = False
    search_start_time = None
    switch_search_active = False
    switch_search_start_odom = None
    target_index = 0
    current_target = TARGET_SEQUENCE[target_index]

    try:
        cam = open_camera()
        lidar = RPLidarC1()
        motor = Motor()
        motor.stop()
        motor.reset_encoders()
        motor_enabled = threading.Event()

        def wait_for_motor_start():
            input("[READY] Press Enter to enable motor...")
            motor_enabled.set()

        threading.Thread(target=wait_for_motor_start, daemon=True).start()
        start_time = time.time()  # 로그 출력용 시작 시각

        while True:
            elapsed = time.time() - start_time  # 프로그램 시작 후 경과 시간
            ok, frame = read_frame(cam)
            if not ok:
                break

            found = detect(frame)  # 현재 프레임에서 찾은 색상 물체 목록
            target = pick(found, current_target)  # 따라갈 대상 색상 물체
            found = [target] if target is not None else []
            scan, scan_time, scan_seq = lidar.get()  # 최신 라이다 스캔 데이터
            ranges = front_ranges(scan) if scan is not None else None  # 각도별 전방 거리 배열
            lidar_dist = lidar_zone_distances(ranges)  # 좌/정면/우측 로그용 최소 거리
            has_obstacle = obstacle_detected(ranges)  # 장애물 감지 여부

            if target is not None:
                last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = update_color_memory(target, frame)
                search_start_time, switch_search_active, search_failed_avoid_active, switch_search_start_odom = None, False, False, None

            if time.time() - last_odom_log_time >= ODOM_LOG_INTERVAL:
                log_odom(elapsed, motor.get_odom())
                last_odom_log_time = time.time()

            log_lidar(elapsed, lidar_dist)

            color_in_forward_zone = (
                motor_enabled.is_set()
                and target is not None
                and last_color_center_ratio >= COLOR_FORWARD_CENTER_RATIO
                and abs(last_color_x_err) <= COLOR_EXIT_CENTER_ERR
            )
            color_exited_bottom = target is None and last_color_time > 0.0 and last_color_center_ratio >= BOTTOM_LOST_RATIO
            if color_in_forward_zone:
                if target_index + 1 < len(TARGET_SEQUENCE):
                    prev_target = current_target
                    target_index += 1
                    current_target = TARGET_SEQUENCE[target_index]
                    mode = f"SWITCH: {prev_target}->{current_target}"
                    print(f"[{elapsed:.2f}s] [COLOR] {prev_target} done, now tracking {current_target}")
                    drive_forward_by_encoder(motor)
                    target_v, target_w, last_v, last_w = 0.0, 0.0, 0.0, 0.0
                    wait_ms(COLOR_SWITCH_PAUSE_MS)
                    ok, frame = read_frame(cam)
                    if not ok:
                        break

                    found = detect(frame)
                    target = pick(found, current_target)
                    found = [target] if target is not None else []

                    if target is not None:
                        last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = update_color_memory(target, frame)
                        color_lost_during_avoid = False
                        search_failed_avoid_active = False
                        search_start_time = None
                        mode, target_v, target_w = color_cmd(
                            target, frame, ranges, has_obstacle, last_color_deg, last_color_center_ratio, elapsed
                        )
                    else:
                        mode, target_v, target_w = search_next_cmd()
                        last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = clear_color_memory()
                        color_lost_during_avoid, search_start_time, switch_search_active = False, None, True
                        switch_search_start_odom = get_turn_start_odom(motor)
                else:
                    mode = "STOP: color bottom"
                    drive_forward_by_encoder(motor)
                    target_v, target_w, last_v, last_w = 0.0, 0.0, 0.0, 0.0
                    last_color_deg, last_color_time, last_color_center_ratio, last_color_x_err = clear_color_memory()
                    color_lost_during_avoid, search_start_time, switch_search_active = False, None, False
                    search_failed_avoid_active, switch_search_start_odom = False, None
                    print(f"[{elapsed:.2f}s] [COLOR] {current_target} done, mission complete")
                    break

            elif target is not None:
                color_lost_during_avoid = False
                search_failed_avoid_active = False
                mode, target_v, target_w = color_cmd(
                    target, frame, ranges, has_obstacle, last_color_deg, last_color_center_ratio, elapsed
                )

            elif target is None and search_failed_avoid_active:
                mode, target_v, target_w = avoid_mode("AVOID: search failed", "AVOID_SEARCH", ranges, last_color_deg if last_color_time > 0.0 else None, elapsed)

            elif color_exited_bottom:
                color_lost_during_avoid = True
                switch_search_active = False
                if not lost_color_obstacle_passed(ranges, last_color_deg):
                    search_start_time = None
                    mode, target_v, target_w = avoid_mode("AVOID: lost color", "AVOID_LOST", ranges, last_color_deg, elapsed)
                else:
                    if search_start_time is None:
                        search_start_time = time.time()
                    mode, target_v, target_w = search_last_cmd(last_color_deg)

            elif target is None and last_color_time > 0.0 and not lost_color_obstacle_passed(ranges, last_color_deg):
                color_lost_during_avoid = True
                search_start_time = None
                mode, target_v, target_w = avoid_mode("AVOID: lost color", "AVOID_LOST", ranges, last_color_deg, elapsed)

            elif target is None and switch_search_active:
                if switch_search_start_odom is None:
                    switch_search_start_odom = get_turn_start_odom(motor)

                if completed_one_encoder_turn(motor, switch_search_start_odom):
                    mode, target_v, target_w = avoid_mode("AVOID: search failed", "AVOID_SEARCH", ranges, last_color_deg if last_color_time > 0.0 else None, elapsed)
                    color_lost_during_avoid, switch_search_active = True, False
                    search_failed_avoid_active = True
                    search_start_time = None
                    switch_search_start_odom = None
                else:
                    mode, target_v, target_w = search_next_cmd()

            elif target is None and color_lost_during_avoid:
                if search_start_time is None:
                    search_start_time = time.time()

                mode, target_v, target_w = search_last_cmd(last_color_deg)

            elif target is None:
                if last_color_time > 0.0:
                    if search_start_time is None:
                        search_start_time = time.time()
                    mode, target_v, target_w = search_last_cmd(last_color_deg)
                    color_lost_during_avoid, switch_search_active = True, False
                else:
                    mode, target_v, target_w = avoid_mode("AVOID: no initial color", "AVOID_INITIAL", ranges, None, elapsed)
                    color_lost_during_avoid, switch_search_active = True, False
                    search_failed_avoid_active = True
                    switch_search_start_odom = None

            if motor_enabled.is_set() and (target is not None or mode.startswith("AVOID") or mode.startswith("SEARCH")):
                last_v = rate_limit(last_v, target_v, V_STEP)
                w_step = AVOID_W_STEP if mode.startswith("AVOID") else FOLLOW_W_STEP
                last_w = rate_limit(last_w, target_w, w_step)
                motor.vw(last_v, last_w)
            elif not motor_enabled.is_set():
                last_v, last_w = 0.0, 0.0
                motor.stop()

            if SHOW_WINDOW:
                if draw(frame, found, mode) == ord("q"):
                    break

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\n[INFO] stop")

    finally:
        if motor is not None:
            motor.stop()
            motor.close()

        if lidar is not None:
            lidar.close()

        close_camera(cam)


if __name__ == "__main__":
    main()
