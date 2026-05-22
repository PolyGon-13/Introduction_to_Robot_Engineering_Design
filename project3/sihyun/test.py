import cv2
import numpy as np
import time

# ==============================
# 카메라 설정
# ==============================

USE_PICAMERA2 = True

try:
    from picamera2 import Picamera2
except ImportError:
    USE_PICAMERA2 = False


def open_camera():
    """
    Raspberry Pi Camera Module이 있으면 Picamera2 사용
    없으면 USB 카메라 cv2.VideoCapture(0) 사용
    """
    global USE_PICAMERA2

    if USE_PICAMERA2:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (640, 480)}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(1)
        print("[INFO] Picamera2 camera started")
        return picam2
    else:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

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
        return True, frame_bgr
    else:
        ret, frame = camera.read()
        return ret, frame


# ==============================
# 색상 HSV 범위 설정
# ==============================
# OpenCV HSV 범위
# H: 0~179
# S: 0~255
# V: 0~255

COLOR_RANGES = {
    "RED": [
        # 빨강은 HSV에서 0 근처와 179 근처 두 구간으로 나뉨
        (np.array([0, 100, 80]), np.array([10, 255, 255])),
        (np.array([170, 100, 80]), np.array([179, 255, 255]))
    ],
    "BLUE": [
        (np.array([95, 100, 80]), np.array([130, 255, 255]))
    ],
    "YELLOW": [
        (np.array([20, 100, 100]), np.array([35, 255, 255]))
    ]
}

BOX_COLOR = {
    "RED": (0, 0, 255),
    "BLUE": (255, 0, 0),
    "YELLOW": (0, 255, 255)
}

MIN_AREA = 800  # 너무 작은 잡음 제거용 최소 면적


def detect_colors(frame):
    """
    프레임에서 빨강, 파랑, 노랑 색상 인식
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    detected_results = []

    for color_name, ranges in COLOR_RANGES.items():
        mask_total = None

        # 색상 범위 마스크 생성
        for lower, upper in ranges:
            mask = cv2.inRange(hsv, lower, upper)

            if mask_total is None:
                mask_total = mask
            else:
                mask_total = cv2.bitwise_or(mask_total, mask)

        # 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel)

        # 윤곽선 검출
        contours, _ = cv2.findContours(
            mask_total,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < MIN_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2

            detected_results.append({
                "color": color_name,
                "area": area,
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "cx": cx,
                "cy": cy
            })

    return detected_results


def draw_results(frame, results):
    """
    인식 결과 화면에 표시
    """
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

        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
        cv2.circle(frame, (cx, cy), 5, box_color, -1)

        text = f"{color} area:{int(area)} center:({cx},{cy})"
        cv2.putText(
            frame,
            text,
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            box_color,
            2
        )

    return frame


def main():
    camera = open_camera()

    if camera is None:
        return

    while True:
        ret, frame = get_frame(camera)

        if not ret:
            print("[ERROR] Failed to read frame")
            break

        results = detect_colors(frame)

        # 터미널 출력
        if results:
            for r in results:
                print(
                    f"[DETECT] color={r['color']} "
                    f"center=({r['cx']},{r['cy']}) "
                    f"area={int(r['area'])}"
                )

        frame = draw_results(frame, results)

        cv2.imshow("Raspberry Pi Color Detection", frame)

        # q 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if USE_PICAMERA2:
        camera.stop()
    else:
        camera.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
