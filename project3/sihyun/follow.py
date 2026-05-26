#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time
import serial
import threading

# =========================================================
# 카메라 설정
# =========================================================

USE_PICAMERA2 = True

try:
    from picamera2 import Picamera2
except ImportError:
    USE_PICAMERA2 = False


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_EXPOSURE_VALUE = -1.0

#반전
FLIP_HORIZONTAL = False #좌우반전
FLIP_VERTICAL = False   #상하반전

# 모니터/원격화면에서 영상 확인할 때 True
# SSH로 실행해서 cv2 창이 안 뜨면 False로 바꾸기
SHOW_WINDOW = True


def open_camera():
    """
    Raspberry Pi Camera Module이 있으면 Picamera2 사용
    없으면 USB 카메라 cv2.VideoCapture(0) 사용
    """
    global USE_PICAMERA2

    if USE_PICAMERA2:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
        )
        picam2.configure(config)
        try:
            picam2.set_controls({"ExposureValue": CAMERA_EXPOSURE_VALUE})
        except Exception:
            pass
        picam2.start()
        time.sleep(1.0)
        print("[INFO] Picamera2 camera started")
        return picam2

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("[ERROR] Camera open failed")
        return None

    print("[INFO] USB camera started")
    return cap


def get_frame(camera):
    """
    카메라에서 프레임 읽기
    """
    if USE_PICAMERA2:
        frame_rgb = camera.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        frame_bgr = apply_camera_flip(frame_bgr)
        return True, frame_bgr

    ret, frame = camera.read()
    if ret:
        frame = apply_camera_flip(frame)
    return ret, frame


def apply_camera_flip(frame):
    if FLIP_HORIZONTAL and FLIP_VERTICAL:
        return cv2.flip(frame, -1)
    if FLIP_HORIZONTAL:
        return cv2.flip(frame, 1)
    if FLIP_VERTICAL:
        return cv2.flip(frame, 0)
    return frame


def close_camera(camera):
    if camera is None:
        return

    if USE_PICAMERA2:
        camera.stop()
    else:
        camera.release()

    if SHOW_WINDOW:
        cv2.destroyAllWindows()


# =========================================================
# Arduino 모터 제어 설정
# 붙여넣은 텍스트 (2)의 방식:
#   V{v:.3f},{w:.3f}\n
#   S\n
# =========================================================

ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

# 전진 속도/회전 속도 제한
MAX_V = 0.18          # 최대 전진 속도
MIN_V = 0.08          # 따라갈 때 최소 전진 속도
MAX_W = 0.60          # 최대 회전 속도
KP_TURN = 0.85        # 화면 중심 오차 -> 회전속도 변환 게인

# 물체가 오른쪽에 있는데 로봇이 왼쪽으로 돌면 True로 바꾸기
INVERT_W = False

# 값이 너무 튀지 않게 루프마다 변화량 제한
V_RATE_LIMIT = 0.04
W_RATE_LIMIT = 0.15

# 카메라 중심 기준 오차가 이 이하이면 회전하지 않음
CENTER_DEADBAND = 0.08

# 면적이 이 값보다 작으면 잡음으로 무시
MIN_AREA = 1200
MIN_EXTENT = 0.45

# 면적 기반 거리 제어
# 물체 면적이 TARGET_AREA보다 작으면 전진
# STOP_AREA보다 커지면 너무 가까운 것으로 보고 정지
TARGET_AREA = 15000
STOP_AREA = 28000

# 추종할 색
# None이면 RED, BLUE, YELLOW 중 가장 크게 보이는 색을 따라감
# "RED", "BLUE", "YELLOW" 중 하나로 바꾸면 해당 색만 따라감
TARGET_COLOR = None

# 색을 못 찾았을 때 동작
# False: 정지
# True : 제자리에서 천천히 탐색 회전
SEARCH_WHEN_LOST = False
SEARCH_W = 0.25
TARGET_LOST_HOLD_SEC = 0.35


def open_arduino():
    ardu = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    time.sleep(2.0)
    print(f"[INFO] Arduino connected: {ARDU_PORT}, {ARDU_BAUD}")
    return ardu


def send_vw(ardu, v, w):
    """
    Arduino로 선속도 v, 각속도 w 전송
    """
    msg = f"V{v:.3f},{w:.3f}\n"
    ardu.write(msg.encode())


def stop(ardu):
    """
    Arduino로 정지 명령 전송
    """
    ardu.write(b"S\n")


def rate_limit(prev_value, target_value, limit):
    delta = float(np.clip(target_value - prev_value, -limit, limit))
    return prev_value + delta


class RPLidarC1:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.ser.write(bytes([0xA5, 0x40]))
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        self.ser.write(bytes([0xA5, 0x20]))
        header = self.ser.read(7)
        if len(header) != 7 or header[0] != 0xA5 or header[1] != 0x5A:
            self.ser.close()
            raise RuntimeError("[LIDAR] Response Header Error")

        self.lock = threading.Lock()
        self.latest_scan = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buf_a, buf_d, buf_q = [], [], []
        while self.running:
            try:
                data = self.ser.read(5)
                if len(data) != 5:
                    continue

                s_flag = data[0] & 0x01
                s_inv = (data[0] & 0x02) >> 1
                if s_inv != (1 - s_flag):
                    continue

                if (data[1] & 0x01) != 1:
                    continue

                quality = data[0] >> 2
                angle = ((data[1] >> 1) | (data[2] << 7)) / 64.0
                dist = (data[3] | (data[4] << 8)) / 4.0

                if s_flag == 1 and len(buf_a) > 50:
                    with self.lock:
                        self.latest_scan = (
                            np.array(buf_a, dtype=np.float32),
                            np.array(buf_d, dtype=np.float32),
                            np.array(buf_q, dtype=np.float32),
                        )
                    buf_a, buf_d, buf_q = [], [], []

                if dist > 0 and quality > 0:
                    buf_a.append(angle)
                    buf_d.append(dist)
                    buf_q.append(quality)

            except (serial.SerialException, OSError) as e:
                print(f"[LIDAR] Serial Error: {e}, retrying in 1 second...")
                time.sleep(1.0)
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass

    def get_scan(self):
        with self.lock:
            return self.latest_scan

    def close(self):
        self.running = False
        try:
            self.ser.write(bytes([0xA5, 0x25]))
        except Exception:
            pass
        time.sleep(0.1)
        self.ser.close()


def open_lidar():
    try:
        lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
        print(f"[INFO] LiDAR connected: {LIDAR_PORT}, {LIDAR_BAUD}")
        return lidar
    except Exception as e:
        print(f"[WARN] LiDAR disabled: {e}")
        return None


# =========================================================
# 색상 HSV 범위 설정
# OpenCV HSV 범위
# H: 0~179
# S: 0~255
# V: 0~255
# =========================================================

COLOR_RANGES = {
    "RED": [
        # 빨강은 HSV에서 0 근처와 179 근처 두 구간으로 나뉨
        (np.array([0, 90, 120]), np.array([10, 255, 255])),
        (np.array([170, 90, 120]), np.array([179, 255, 255])),
    ],
    "BLUE": [
        (np.array([102, 85, 110]), np.array([126, 255, 255])),
    ],
    "YELLOW": [
        (np.array([22, 85, 140]), np.array([36, 255, 255])),
    ],
}

BOX_COLOR = {
    "RED": (0, 0, 255),
    "BLUE": (255, 0, 0),
    "YELLOW": (0, 255, 255),
}

BLUR_SIZE = 5
OPEN_KERNEL_SIZE = 5
CLOSE_KERNEL_SIZE = 5


def detect_colors(frame):
    """
    프레임에서 빨강, 파랑, 노랑 색상 인식
    """
    if BLUR_SIZE > 1:
        frame = cv2.GaussianBlur(frame, (BLUR_SIZE, BLUR_SIZE), 0)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detected_results = []
    open_kernel = np.ones((OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE), np.uint8)
    close_kernel = np.ones((CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE), np.uint8)

    for color_name, ranges in COLOR_RANGES.items():
        mask_total = None

        for lower, upper in ranges:
            mask = cv2.inRange(hsv, lower, upper)

            if mask_total is None:
                mask_total = mask
            else:
                mask_total = cv2.bitwise_or(mask_total, mask)

        # 노이즈 제거
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, open_kernel)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, close_kernel)

        contours, _ = cv2.findContours(
            mask_total,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < MIN_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            extent = area / max(1.0, float(w * h))

            if extent < MIN_EXTENT:
                continue

            cx = x + w // 2
            cy = y + h // 2

            detected_results.append(
                {
                    "color": color_name,
                    "area": float(area),
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h,
                    "cx": cx,
                    "cy": cy,
                }
            )

    return detected_results


def choose_target(results):
    """
    여러 색이 동시에 보이면 가장 면적이 큰 물체를 추종 대상으로 선택
    """
    if TARGET_COLOR is None:
        candidates = results
    else:
        candidates = [r for r in results if r["color"] == TARGET_COLOR]

    if not candidates:
        return None

    return max(candidates, key=lambda r: r["area"])


def compute_follow_cmd(target, frame_width):
    """
    인식한 색상 물체의 위치/면적으로 v, w 계산

    cx가 화면 오른쪽이면 error_x > 0
    파일 2의 FGM 코드 기준으로 w > 0은 좌회전 방향으로 쓰였으므로,
    오른쪽 물체를 따라가려면 w는 음수로 보냄.
    """
    image_center_x = frame_width / 2.0

    # -1.0 ~ +1.0
    error_x = (target["cx"] - image_center_x) / image_center_x

    if abs(error_x) < CENTER_DEADBAND:
        error_x = 0.0

    # 오른쪽에 있으면 우회전해야 하므로 기본은 - 부호
    if INVERT_W:
        w = KP_TURN * error_x
    else:
        w = -KP_TURN * error_x

    w = float(np.clip(w, -MAX_W, MAX_W))

    area = target["area"]

    # 너무 가까우면 정지
    if area >= STOP_AREA:
        v = 0.0

    # 적당히 가까우면 천천히 접근 또는 정지
    elif area >= TARGET_AREA:
        v = 0.0

    # 멀리 있으면 전진
    else:
        ratio = (TARGET_AREA - area) / max(1.0, TARGET_AREA - MIN_AREA)
        ratio = float(np.clip(ratio, 0.0, 1.0))
        v = MIN_V + (MAX_V - MIN_V) * ratio

        # 많이 틀어야 하는 상황에서는 전진 속도 줄이기
        turn_slow = 1.0 - 0.45 * min(1.0, abs(error_x))
        v *= turn_slow

    v = float(np.clip(v, 0.0, MAX_V))

    return v, w, error_x


def draw_results(frame, results, target=None, cmd_v=0.0, cmd_w=0.0):
    """
    인식 결과 화면에 표시
    """
    frame_h, frame_w = frame.shape[:2]
    cv2.line(
        frame,
        (frame_w // 2, 0),
        (frame_w // 2, frame_h),
        (255, 255, 255),
        1,
    )

    for result in results:
        color = result["color"]
        x = result["x"]
        y = result["y"]
        w = result["w"]
        h = result["h"]
        cx = result["cx"]
        cy = result["cy"]
        area = result["area"]

        box_color = BOX_COLOR[color]
        thickness = 3 if result is target else 2

        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, thickness)
        cv2.circle(frame, (cx, cy), 5, box_color, -1)

        label = f"{color} area:{int(area)} cx:{cx}"
        if result is target:
            label = "[TARGET] " + label

        cv2.putText(
            frame,
            label,
            (x, max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            box_color,
            2,
        )

    status = f"cmd: V={cmd_v:.2f}, W={cmd_w:.2f}"
    cv2.putText(
        frame,
        status,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
    )

    return frame


# =========================================================
# 메인
# =========================================================

def main():
    camera = None
    ardu = None
    lidar = None

    last_v = 0.0
    last_w = 0.0
    last_log = 0.0
    last_target = None
    last_target_time = 0.0

    try:
        camera = open_camera()
        if camera is None:
            return

        lidar = open_lidar()

        ardu = open_arduino()
        stop(ardu)

        print("[INFO] Initialization complete.")
        print("[INFO] Press Enter to start.")
        try:
            input()
        except EOFError:
            print("[WARN] Could not read input. Starting immediately.")

        print("[INFO] Go!!")

        while True:
            ret, frame = get_frame(camera)

            if not ret:
                print("[ERROR] Failed to read frame")
                send_vw(ardu, 0.0, 0.0)
                time.sleep(0.05)
                continue

            results = detect_colors(frame)
            target = choose_target(results)
            now = time.time()

            if target is not None:
                last_target = target.copy()
                last_target_time = now
            elif (
                last_target is not None
                and now - last_target_time <= TARGET_LOST_HOLD_SEC
            ):
                target = last_target.copy()
                target["held"] = True

            if target is None:
                if SEARCH_WHEN_LOST:
                    target_v = 0.0
                    target_w = SEARCH_W
                    mode = "SEARCH"
                    error_x = 0.0
                else:
                    target_v = 0.0
                    target_w = 0.0
                    mode = "LOST"
                    error_x = 0.0
            else:
                target_v, target_w, error_x = compute_follow_cmd(
                    target,
                    frame.shape[1],
                )
                if target.get("held"):
                    mode = f"HOLD_{target['color']}"
                else:
                    mode = f"FOLLOW_{target['color']}"

            # 급격한 속도 변화 방지
            cmd_v = rate_limit(last_v, target_v, V_RATE_LIMIT)
            cmd_w = rate_limit(last_w, target_w, W_RATE_LIMIT)

            send_vw(ardu, cmd_v, cmd_w)

            last_v = cmd_v
            last_w = cmd_w

            if now - last_log > 0.25:
                if target is None:
                    print(f"[{mode}] v={cmd_v:.2f} w={cmd_w:.2f}")
                else:
                    print(
                        f"[{mode}] "
                        f"cx={target['cx']} area={int(target['area'])} "
                        f"err={error_x:.2f} "
                        f"v={cmd_v:.2f} w={cmd_w:.2f}"
                    )
                last_log = now

            if SHOW_WINDOW:
                frame = draw_results(frame, results, target, cmd_v, cmd_w)
                cv2.imshow("Color Following Robot", frame)

                # q 누르면 종료
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt")

    finally:
        if ardu is not None:
            try:
                stop(ardu)
                time.sleep(0.2)
                ardu.close()
            except Exception:
                pass

        if lidar is not None:
            try:
                lidar.close()
            except Exception:
                pass

        close_camera(camera)
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
