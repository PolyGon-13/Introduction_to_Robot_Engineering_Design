import cv2
import numpy as np

# 웹캠 열기
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# HSV 색상 범위 설정 (형광 녹색 & 형광 노랑 계열)
lower_green = np.array([15, 50, 50])  # HSV 형광 초록 하한값
upper_green = np.array([80, 255, 255])  # HSV 형광 초록 상한값

lower_yellow = np.array([20, 100, 100])  # HSV 형광 노랑 하한값
upper_yellow = np.array([40, 255, 255])  # HSV 형광 노랑 상한값

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # BGR → HSV 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 형광 조끼 색상 마스크 (초록 + 노랑)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask = cv2.bitwise_or(mask_green, mask_yellow)  # 두 개의 마스크 결합

    # 윤곽선 찾기
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if cv2.contourArea(cnt) > 1000:  # 1000이하 면적의 윤곽선 제거 (너무 작은 객체와 노이즈 제거)
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 화면 출력
    cv2.imshow("Fluorescent Vest Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
