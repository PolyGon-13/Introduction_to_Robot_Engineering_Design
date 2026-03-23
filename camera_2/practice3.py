import cv2
import numpy as np

# 카메라 열기
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("Camera Not Found.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to return image")
        break

    # === 1. RGB 기반 피부 검출 ===
    # 피부색 범위 설정 (RGB)
    lower_rgb = np.array([45, 30, 20], dtype=np.uint8)  # 어두운 피부까지 포함
    upper_rgb = np.array([255, 200, 160], dtype=np.uint8)  # 밝은 피부까지 포함

    # 피부색 마스크 생성 (RGB)
    rgb_mask = cv2.inRange(frame, lower_rgb, upper_rgb)

    # 윤곽선 찾기 (RGB)
    contours_rgb, _ = cv2.findContours(rgb_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # RGB 윤곽선 표시용 이미지 복사
    rgb_result = frame.copy()

    for c in contours_rgb:
        if cv2.contourArea(c) < 1000:  # 작은 영역 무시
            continue

        # 윤곽선 및 바운딩 박스 표시 (파란색 & 초록색)
        cv2.drawContours(rgb_result, [c], -1, (255, 0, 0), 2)
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(rgb_result, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # === 2. HSV 기반 피부 검출 ===
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 피부색 범위 설정 (HSV)
    lower_hsv = np.array([0, 20, 70], dtype=np.uint8)
    upper_hsv = np.array([20, 255, 255], dtype=np.uint8)

    # 피부색 마스크 생성 (HSV)
    hsv_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # 윤곽선 찾기 (HSV)
    contours_hsv, _ = cv2.findContours(hsv_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # HSV 윤곽선 표시용 이미지 복사
    hsv_result = frame.copy()

    for c in contours_hsv:
        if cv2.contourArea(c) < 1000:  # 작은 영역 무시
            continue

        # 윤곽선 및 바운딩 박스 표시 (파란색 & 초록색)
        cv2.drawContours(hsv_result, [c], -1, (255, 0, 0), 2)
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(hsv_result, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 결과 출력
    cv2.imshow("RGB Skin Detection", rgb_result)  # RGB 기반 피부 검출 결과
    cv2.imshow("HSV Skin Detection", hsv_result)  # HSV 기반 피부 검출 결과
    cv2.imshow("RGB Mask", rgb_mask)  # RGB 마스크
    cv2.imshow("HSV Mask", hsv_mask)  # HSV 마스크

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
