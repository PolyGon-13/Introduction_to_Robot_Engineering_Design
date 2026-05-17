#!/usr/bin/env python3

import math
import time
from queue import Empty, Full, Queue
from threading import Thread

import serial


LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800
ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600

ANGLE_OFFSET_DEG = 1.54
DIST_OFFSET_MM = 0.0
LIDAR_ANGLE_SIGN = 1.0

MIN_LIDAR_DIST_MM = 50.0
MAX_LIDAR_DIST_MM = 600.0
MIN_QUALITY = 1

FGM_MIN_ANGLE_DEG = -90.0
FGM_MAX_ANGLE_DEG = 90.0
FGM_CENTER_INDEX = 90.0
FGM_FREE_THRESHOLD_MM = 400.0
FGM_MIN_GAP_WIDTH_DEG = 8.0
FGM_CENTER_BIAS_PER_DEG = 1.0
MIN_SCAN_POINTS = 40

BASE_V = 0.20
MIN_V = 0.15
MAX_ABS_W = 0.70
FGM_TURN_GAIN = 1.05
STRAIGHT_DEADBAND_DEG = 8.0
FRONT_DANGER_MM = 190.0

COMMAND_WRITE_SLEEP_S = 0.01
LOG_INTERVAL_S = 0.5


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value, low, high):
    return max(low, min(high, value))


def arduino_writer(arduino_ser, data_queue):
    while True:
        v, w = data_queue.get()
        arduino_ser.write(f"V{v:.3f},{w:.3f}\n".encode())
        time.sleep(COMMAND_WRITE_SLEEP_S)


def put_latest(data_queue, command):
    try:
        data_queue.put_nowait(command)
    except Full:
        try:
            data_queue.get_nowait()
        except Empty:
            pass
        try:
            data_queue.put_nowait(command)
        except Full:
            pass


def start_lidar(lidar_ser):
    lidar_ser.write(bytes([0xA5, 0x40]))
    time.sleep(1.0)
    lidar_ser.reset_input_buffer()

    lidar_ser.write(bytes([0xA5, 0x20]))
    header = lidar_ser.read(7)
    if len(header) != 7 or header[0] != 0xA5 or header[1] != 0x5A:
        raise RuntimeError("[LIDAR] Response Header Error")


def stop_lidar(lidar_ser):
    try:
        lidar_ser.write(bytes([0xA5, 0x25]))
    except Exception:
        pass


def new_distance_array():
    return [0.0] * 181


def parse_lidar_packet(data):
    if len(data) != 5:
        return None

    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        return None

    if (data[1] & 0x01) != 1:
        return None

    quality = data[0] >> 2
    if quality < MIN_QUALITY:
        return None

    angle_q6 = (data[1] >> 1) | (data[2] << 7)
    raw_angle = angle_q6 / 64.0
    angle = normalize_angle_deg(raw_angle + ANGLE_OFFSET_DEG)
    angle = normalize_angle_deg(LIDAR_ANGLE_SIGN * angle)

    distance_q2 = data[3] | (data[4] << 8)
    distance = (distance_q2 / 4.0) + DIST_OFFSET_MM
    if distance < MIN_LIDAR_DIST_MM:
        return None
    distance = min(distance, MAX_LIDAR_DIST_MM)

    if angle < FGM_MIN_ANGLE_DEG or angle > FGM_MAX_ANGLE_DEG:
        return None

    return s_flag, angle, distance


def Follow_the_Gap_Method(distances, threshold):
    gaps = []
    current_start = None

    for angle, distance in enumerate(distances):
        if distance >= threshold:
            if current_start is None:
                current_start = angle
        elif current_start is not None:
            gaps.append((current_start, angle - 1))
            current_start = None

    if current_start is not None:
        gaps.append((current_start, len(distances) - 1))

    min_gap_bins = max(1, int(math.ceil(FGM_MIN_GAP_WIDTH_DEG)))
    valid_gaps = [
        (start, end)
        for start, end in gaps
        if (end - start + 1) >= min_gap_bins
    ]
    if not valid_gaps:
        return FGM_CENTER_INDEX, False, 0

    best_start, best_end = max(
        valid_gaps,
        key=lambda gap: (
            (gap[1] - gap[0] + 1)
            - FGM_CENTER_BIAS_PER_DEG
            * abs(((gap[0] + gap[1]) / 2.0) - FGM_CENTER_INDEX)
        ),
    )

    target_angle = clamp(FGM_CENTER_INDEX, best_start, best_end)
    return target_angle, True, best_end - best_start + 1


def choose_drive_command(target_angle_deg, target_distance_mm, has_gap):
    if not has_gap or target_distance_mm < FRONT_DANGER_MM:
        return 0.0, 0.0

    target_angle_rad = math.radians(target_angle_deg)
    w = clamp(FGM_TURN_GAIN * target_angle_rad, -MAX_ABS_W, MAX_ABS_W)

    if abs(target_angle_deg) <= STRAIGHT_DEADBAND_DEG:
        v = BASE_V
    else:
        v = MIN_V

    return v, w


def main():
    lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
    arduino_ser = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    data_queue = Queue(maxsize=1)

    writer = Thread(target=arduino_writer, args=(arduino_ser, data_queue), daemon=True)
    writer.start()

    distance_array = new_distance_array()
    points_in_scan = 0
    last_log = 0.0

    try:
        start_lidar(lidar_ser)
        put_latest(data_queue, (0.0, 0.0))
        print("[INFO] fgm_add started.")

        while True:
            data = lidar_ser.read(5)
            parsed = parse_lidar_packet(data)
            if parsed is None:
                continue

            s_flag, angle, distance = parsed

            if s_flag == 1 and points_in_scan >= MIN_SCAN_POINTS:
                desired_angle, has_gap, gap_width = Follow_the_Gap_Method(
                    distance_array,
                    FGM_FREE_THRESHOLD_MM,
                )
                target_angle = desired_angle - FGM_CENTER_INDEX
                target_index = int(round(desired_angle))
                target_distance = distance_array[target_index]

                v, w = choose_drive_command(target_angle, target_distance, has_gap)
                put_latest(data_queue, (v, w))

                now = time.time()
                if now - last_log >= LOG_INTERVAL_S:
                    print(
                        f"[FGM_ADD] tgt={target_angle:.1f}deg "
                        f"dist={target_distance / 1000.0:.2f} "
                        f"gap={gap_width} safe={int(has_gap)} "
                        f"v={v:.2f} w={w:.2f}"
                    )
                    last_log = now

                distance_array = new_distance_array()
                points_in_scan = 0

            angle_index = int(round(angle - FGM_MIN_ANGLE_DEG))
            if 0 <= angle_index < len(distance_array):
                distance_array[angle_index] = distance
                points_in_scan += 1

    except KeyboardInterrupt:
        pass
    finally:
        try:
            arduino_ser.write(b"S\n")
        except Exception:
            pass
        stop_lidar(lidar_ser)
        time.sleep(0.2)
        try:
            arduino_ser.close()
        except Exception:
            pass
        try:
            lidar_ser.close()
        except Exception:
            pass
        print("[INFO] fgm_add stopped.")


if __name__ == "__main__":
    main()
